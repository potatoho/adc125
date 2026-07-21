module adc_axis64_packer #(
    parameter FIFO_DEPTH = 4096,
    parameter WORDS_PER_PACKET = 131072
)(
    input             rst,

    input             adc1_clk,
    input      [11:0] adc1_data_a,
    input      [11:0] adc1_data_b,

    input             adc2_clk,
    input      [11:0] adc2_data_a,
    input      [11:0] adc2_data_b,

    input             m_axis_aclk,
    input             m_axis_aresetn,
    output reg [63:0] m_axis_tdata,
    output reg        m_axis_tvalid,
    input             m_axis_tready,
    output     [7:0]  m_axis_tkeep,
    output reg        m_axis_tlast
);

wire [23:0] adc1_fifo_dout;
wire [23:0] adc2_fifo_dout;
wire        adc1_fifo_empty;
wire        adc2_fifo_empty;

wire axis_rst = ~m_axis_aresetn;

assign m_axis_tkeep = 8'hFF;

wire can_send;
assign can_send = (~adc1_fifo_empty) && (~adc2_fifo_empty) &&
                  (~m_axis_tvalid || m_axis_tready);

reg [$clog2(WORDS_PER_PACKET)-1:0] packet_cnt;

wire fifo_rd_en;
assign fifo_rd_en = can_send;

xpm_fifo_async #(
    .FIFO_MEMORY_TYPE   ("auto"),
    .FIFO_WRITE_DEPTH   (FIFO_DEPTH),
    .WRITE_DATA_WIDTH   (24),
    .READ_DATA_WIDTH    (24),
    .READ_MODE          ("fwft"),
    .FIFO_READ_LATENCY  (0)
) adc1_async_fifo (
    .rst        (rst),
    .wr_clk     (adc1_clk),
    .wr_en      (1'b1),
    .din        ({adc1_data_a, adc1_data_b}),
    .full       (),

    .rd_clk     (m_axis_aclk),
    .rd_en      (fifo_rd_en),
    .dout       (adc1_fifo_dout),
    .empty      (adc1_fifo_empty),

    .sleep      (1'b0),
    .injectsbiterr(1'b0),
    .injectdbiterr(1'b0)
);

xpm_fifo_async #(
    .FIFO_MEMORY_TYPE   ("auto"),
    .FIFO_WRITE_DEPTH   (FIFO_DEPTH),
    .WRITE_DATA_WIDTH   (24),
    .READ_DATA_WIDTH    (24),
    .READ_MODE          ("fwft"),
    .FIFO_READ_LATENCY  (0)
) adc2_async_fifo (
    .rst        (rst),
    .wr_clk     (adc2_clk),
    .wr_en      (1'b1),
    .din        ({adc2_data_a, adc2_data_b}),
    .full       (),

    .rd_clk     (m_axis_aclk),
    .rd_en      (fifo_rd_en),
    .dout       (adc2_fifo_dout),
    .empty      (adc2_fifo_empty),

    .sleep      (1'b0),
    .injectsbiterr(1'b0),
    .injectdbiterr(1'b0)
);

always @(posedge m_axis_aclk) begin
    if (axis_rst) begin
        m_axis_tdata  <= 64'd0;
        m_axis_tvalid <= 1'b0;
        m_axis_tlast  <= 1'b0;
        packet_cnt    <= 0;
    end else begin
        if (can_send) begin
            m_axis_tdata <= {
                8'd0, adc2_fifo_dout,
                8'd0, adc1_fifo_dout
            };

            m_axis_tvalid <= 1'b1;
            m_axis_tlast  <= (packet_cnt == WORDS_PER_PACKET-1);

            if (packet_cnt == WORDS_PER_PACKET-1)
                packet_cnt <= 0;
            else
                packet_cnt <= packet_cnt + 1'b1;
        end else if (m_axis_tready) begin
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
        end
    end
end

endmodule


module axis64_trigger_buffer #(
    parameter integer DEPTH  = 262144,
    parameter integer ADDR_W = $clog2(DEPTH),
    parameter integer FORCE_FOOTER = 1
)(
    input  wire        clk,
    input  wire        resetn,

    input  wire [63:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,

    input  wire        trig_in,

    input  wire [31:0] cfg_control,
    input  wire [31:0] cfg_frame_size,
    input  wire [31:0] cfg_pretrigger_size,
    input  wire [31:0] cfg_sample_decimation,
    input  wire [31:0] cfg_trigger_cfg,
    input  wire [31:0] cfg_self_threshold,
    input  wire [31:0] cfg_channel_mask,
    input  wire [31:0] cfg_output_cfg,
    input  wire [31:0] cfg_baseline_shift,

    output reg  [63:0] m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output wire [7:0]  m_axis_tkeep,
    output reg         m_axis_tlast
);

localparam integer DEFAULT_FRAME_SIZE = 131072;
localparam integer DEFAULT_PRE_SIZE   = 16384;
localparam integer FOOTER_WORDS       = 10;
localparam [15:0]  FOOTER_WORDS_U16   = FOOTER_WORDS;

localparam [63:0] FOOTER_MAGIC        = 64'h00000000_FEE70001;


function [7:0] effective_baseline_shift;
    input [31:0] requested_shift;
    input [31:0] frame_words;
    integer i;
    reg [7:0] result;
begin
    result = 0;

    for (i = 0; i < 31; i = i + 1) begin
        if (
            (requested_shift >= i) &&
            ((32'd1 << i) <= frame_words)
        ) begin
            result = i;
        end
    end

    effective_baseline_shift = result;
end
endfunction


function [15:0] abs16;
    input signed [15:0] value;
begin
    abs16 = value[15] ? (~value + 16'd1) : value;
end
endfunction


// ============================================================
// Configuration range protection
// ============================================================

wire [31:0] cfg_frame_effective;

assign cfg_frame_effective =
    (cfg_frame_size == 0) ?
        ((DEFAULT_FRAME_SIZE <= DEPTH) ?
            DEFAULT_FRAME_SIZE : DEPTH) :
    (cfg_frame_size > DEPTH) ?
        DEPTH :
        cfg_frame_size;


wire [31:0] cfg_pre_effective;

assign cfg_pre_effective =
    (cfg_pretrigger_size == 0) ?
        ((DEFAULT_PRE_SIZE <= cfg_frame_effective) ?
            DEFAULT_PRE_SIZE : cfg_frame_effective) :
    (cfg_pretrigger_size > cfg_frame_effective) ?
        cfg_frame_effective :
        cfg_pretrigger_size;


wire [31:0] cfg_decimation_effective;

assign cfg_decimation_effective =
    (cfg_sample_decimation == 0) ?
        32'd1 :
        cfg_sample_decimation;


// ============================================================
// AXI Stream input
// ============================================================

assign s_axis_tready = 1'b1;
assign m_axis_tkeep  = 8'hFF;

wire input_sample_valid;

assign input_sample_valid =
    s_axis_tvalid && s_axis_tready;


// ============================================================
// External trigger synchronizer
// ============================================================

reg trig_ff1;
reg trig_ff2;
reg trig_ff2_d;

always @(posedge clk) begin
    if (!resetn) begin
        trig_ff1   <= 1'b0;
        trig_ff2   <= 1'b0;
        trig_ff2_d <= 1'b0;
    end else begin
        trig_ff1   <= trig_in;
        trig_ff2   <= trig_ff1;
        trig_ff2_d <= trig_ff2;
    end
end

wire ext_trig_rise;

assign ext_trig_rise = trig_ff2 && !trig_ff2_d;


// ============================================================
// Soft trigger synchronizer
// ============================================================

reg soft_ff1;
reg soft_ff2;
reg soft_ff2_d;

always @(posedge clk) begin
    if (!resetn) begin
        soft_ff1   <= 1'b0;
        soft_ff2   <= 1'b0;
        soft_ff2_d <= 1'b0;
    end else begin
        soft_ff1   <= cfg_control[1];
        soft_ff2   <= soft_ff1;
        soft_ff2_d <= soft_ff2;
    end
end

wire soft_trig_rise;

assign soft_trig_rise = soft_ff2 && !soft_ff2_d;


// ============================================================
// Capture state
// ============================================================

localparam CAP_IDLE = 2'd0;
localparam CAP_POST = 2'd1;
localparam CAP_HOLD = 2'd2;

reg [1:0] cap_state;


reg [31:0] frame_size_latched;
reg [31:0] pre_samples_latched;
reg [31:0] post_samples_latched;
reg [31:0] decimation_latched;
reg [31:0] output_cfg_latched;
reg [7:0]  baseline_shift_latched;
reg [3:0]  output_channel_mask_latched;


wire [31:0] active_decimation;

assign active_decimation =
    (cap_state == CAP_IDLE) ?
        cfg_decimation_effective :
        decimation_latched;


// ============================================================
// Decimation
// ============================================================

reg [31:0] decimation_count;

wire decimated_sample_valid;

assign decimated_sample_valid =
    input_sample_valid &&
    (
        (active_decimation <= 1) ||
        (decimation_count >= active_decimation - 1'b1)
    );

always @(posedge clk) begin
    if (!resetn) begin
        decimation_count <= 0;
    end else if (input_sample_valid) begin
        if (
            (active_decimation <= 1) ||
            (decimation_count >= active_decimation - 1'b1)
        ) begin
            decimation_count <= 0;
        end else begin
            decimation_count <= decimation_count + 1'b1;
        end
    end
end


// ============================================================
// Self trigger compare
// ============================================================

wire [11:0] adc1_a_sample;
wire [11:0] adc1_b_sample;
wire [11:0] adc2_a_sample;
wire [11:0] adc2_b_sample;
wire [11:0] self_threshold;
wire [3:0]  self_trigger_mask;
wire [3:0]  output_channel_mask;

assign adc1_a_sample = s_axis_tdata[23:12];
assign adc1_b_sample = s_axis_tdata[11:0];
assign adc2_a_sample = s_axis_tdata[55:44];
assign adc2_b_sample = s_axis_tdata[43:32];
assign self_threshold = cfg_self_threshold[11:0];
assign self_trigger_mask = cfg_channel_mask[3:0];
assign output_channel_mask =
    (cfg_channel_mask[7:4] == 4'b0000) ?
        cfg_channel_mask[3:0] :
        cfg_channel_mask[7:4];

wire self_trig_level;

assign self_trig_level =
    (self_trigger_mask[0] && (adc1_a_sample >= self_threshold)) ||
    (self_trigger_mask[1] && (adc1_b_sample >= self_threshold)) ||
    (self_trigger_mask[2] && (adc2_a_sample >= self_threshold)) ||
    (self_trigger_mask[3] && (adc2_b_sample >= self_threshold));

reg self_trig_level_d;

always @(posedge clk) begin
    if (!resetn) begin
        self_trig_level_d <= 1'b0;
    end else if (decimated_sample_valid) begin
        self_trig_level_d <= self_trig_level;
    end
end

wire self_trig_rise;

assign self_trig_rise =
    decimated_sample_valid &&
    self_trig_level &&
    !self_trig_level_d;


wire [2:0] trigger_enable;

assign trigger_enable = cfg_trigger_cfg[2:0];

wire trigger_event;

assign trigger_event =
    (trigger_enable[0] && ext_trig_rise) ||
    (trigger_enable[1] && soft_trig_rise) ||
    (trigger_enable[2] && self_trig_rise);


// ============================================================
// Ring buffer write control
// ============================================================

reg [ADDR_W-1:0] wr_ptr;
reg [31:0]       history_count;

wire ring_wr_en;

assign ring_wr_en =
    decimated_sample_valid &&
    (cap_state != CAP_HOLD);


wire [ADDR_W-1:0] wr_ptr_after_current;

assign wr_ptr_after_current =
    wr_ptr + (ring_wr_en ? 1'b1 : 1'b0);


wire prebuffer_ready;

assign prebuffer_ready =
    (cfg_pre_effective == 0) ||
    (history_count >= cfg_pre_effective);


always @(posedge clk) begin
    if (!resetn) begin
        wr_ptr        <= 0;
        history_count <= 0;
    end else if (ring_wr_en) begin
        wr_ptr <= wr_ptr + 1'b1;

        if (history_count < DEPTH)
            history_count <= history_count + 1'b1;
    end
end


// ============================================================
// UltraRAM ring buffer
// ============================================================

wire                 ring_rd_en;
wire [ADDR_W-1:0]    ring_rd_addr;
wire [63:0]          ring_rd_data;

xpm_memory_sdpram #(
    .ADDR_WIDTH_A            (ADDR_W),
    .ADDR_WIDTH_B            (ADDR_W),
    .AUTO_SLEEP_TIME         (0),
    .BYTE_WRITE_WIDTH_A      (64),
    .CLOCKING_MODE           ("common_clock"),
    .ECC_MODE                ("no_ecc"),
    .MEMORY_INIT_FILE        ("none"),
    .MEMORY_INIT_PARAM       ("0"),
    .MEMORY_OPTIMIZATION     ("true"),
    .MEMORY_PRIMITIVE        ("ultra"),
    .MEMORY_SIZE             (DEPTH * 64),
    .MESSAGE_CONTROL         (0),
    .READ_DATA_WIDTH_B       (64),
    .READ_LATENCY_B          (1),
    .READ_RESET_VALUE_B      ("0"),
    .RST_MODE_B              ("SYNC"),
    .SIM_ASSERT_CHK          (0),
    .USE_EMBEDDED_CONSTRAINT (0),
    .USE_MEM_INIT            (0),
    .WAKEUP_TIME             ("disable_sleep"),
    .WRITE_DATA_WIDTH_A      (64),
    .WRITE_MODE_B            ("read_first")
) ring_mem_inst (
    .clka           (clk),
    .ena            (ring_wr_en),
    .wea            (1'b1),
    .addra          (wr_ptr),
    .dina           (s_axis_tdata),
    .injectsbiterra (1'b0),
    .injectdbiterra (1'b0),

    .clkb           (clk),
    .enb            (ring_rd_en),
    .addrb          (ring_rd_addr),
    .doutb          (ring_rd_data),
    .rstb           (~resetn),
    .regceb         (1'b1),
    .sleep          (1'b0),
    .sbiterrb       (),
    .dbiterrb       ()
);


// ============================================================
// Trigger capture control
// ============================================================

reg [ADDR_W-1:0] trig_ptr;
reg [ADDR_W-1:0] frame_start_addr;
reg [31:0]       post_count;
reg              frame_valid;

wire frame_done;

assign frame_done =
    m_axis_tvalid &&
    m_axis_tready &&
    m_axis_tlast;


always @(posedge clk) begin
    if (!resetn) begin
        cap_state            <= CAP_IDLE;
        trig_ptr             <= 0;
        frame_start_addr     <= 0;
        post_count           <= 0;
        frame_valid          <= 1'b0;

        frame_size_latched   <= DEFAULT_FRAME_SIZE;
        pre_samples_latched  <= DEFAULT_PRE_SIZE;
        post_samples_latched <= DEFAULT_FRAME_SIZE - DEFAULT_PRE_SIZE;
        decimation_latched   <= 1;
        output_cfg_latched   <= 32'd0;
        baseline_shift_latched <= 8'd10;
        output_channel_mask_latched <= 4'hF;
    end else begin
        case (cap_state)

            CAP_IDLE: begin
                post_count <= 0;

                if (
                    trigger_event &&
                    prebuffer_ready &&
                    !frame_valid
                ) begin
                    frame_size_latched   <= cfg_frame_effective;
                    pre_samples_latched  <= cfg_pre_effective;
                    post_samples_latched <=
                        cfg_frame_effective - cfg_pre_effective;
                    decimation_latched   <= cfg_decimation_effective;
                    output_cfg_latched   <= cfg_output_cfg;
                    baseline_shift_latched <=
                        effective_baseline_shift(
                            cfg_baseline_shift,
                            (cfg_pre_effective == 0) ?
                                32'd1 :
                                cfg_pre_effective
                        );
                    output_channel_mask_latched <= output_channel_mask;

                    trig_ptr <= wr_ptr_after_current;

                    if (
                        (cfg_frame_effective -
                         cfg_pre_effective) == 0
                    ) begin
                        frame_start_addr <=
                            wr_ptr_after_current -
                            cfg_pre_effective[ADDR_W-1:0];

                        frame_valid <= 1'b1;
                        cap_state   <= CAP_HOLD;
                    end else begin
                        cap_state <= CAP_POST;
                    end
                end
            end


            CAP_POST: begin
                if (decimated_sample_valid) begin
                    if (
                        post_count + 1'b1 >=
                        post_samples_latched
                    ) begin
                        frame_start_addr <=
                            trig_ptr -
                            pre_samples_latched[ADDR_W-1:0];

                        frame_valid <= 1'b1;
                        cap_state   <= CAP_HOLD;
                    end else begin
                        post_count <= post_count + 1'b1;
                    end
                end
            end


            CAP_HOLD: begin
                if (frame_done) begin
                    frame_valid <= 1'b0;
                    cap_state   <= CAP_IDLE;
                end
            end


            default: begin
                cap_state <= CAP_IDLE;
            end

        endcase
    end
end


// ============================================================
// AXI Stream playback
// ============================================================

localparam [2:0] ST_IDLE      = 3'd0;
localparam [2:0] ST_BASE_READ = 3'd1;
localparam [2:0] ST_BASE_WAIT = 3'd2;
localparam [2:0] ST_READ      = 3'd3;
localparam [2:0] ST_WAIT      = 3'd4;
localparam [2:0] ST_SEND      = 3'd5;
localparam [2:0] ST_FOOTER    = 3'd6;

reg [2:0]        play_state;
reg [31:0]       play_index;
reg [ADDR_W-1:0] play_addr;
reg [3:0]        footer_index;

wire footer_enable;
wire baseline_correct_enable;
wire analysis_enable;
wire [7:0] footer_output_cfg_report;
wire [31:0] baseline_sample_count;

assign footer_enable =
    (FORCE_FOOTER != 0) ?
        1'b1 :
        (output_cfg_latched[1] | cfg_output_cfg[1]);
assign baseline_correct_enable = output_cfg_latched[0];
assign analysis_enable = footer_enable || baseline_correct_enable;
assign footer_output_cfg_report =
    output_cfg_latched[7:0] |
    {5'd0, cfg_output_cfg[2:1], 1'b0} |
    ((FORCE_FOOTER != 0) ? 8'h02 : 8'h00);
assign baseline_sample_count = 32'd1 << baseline_shift_latched;

assign ring_rd_en =
    (play_state == ST_BASE_READ) ||
    (play_state == ST_READ);

assign ring_rd_addr =
    play_addr;

reg [63:0] footer_data;

wire [11:0] rd_adc1_a_sample;
wire [11:0] rd_adc1_b_sample;
wire [11:0] rd_adc2_a_sample;
wire [11:0] rd_adc2_b_sample;
wire [63:0] masked_raw_data;
wire signed [15:0] corrected_adc1_a;
wire signed [15:0] corrected_adc1_b;
wire signed [15:0] corrected_adc2_a;
wire signed [15:0] corrected_adc2_b;
wire signed [63:0] corrected_adc1_a_ext;
wire signed [63:0] corrected_adc1_b_ext;
wire signed [63:0] corrected_adc2_a_ext;
wire signed [63:0] corrected_adc2_b_ext;
wire signed [15:0] output_adc1_a;
wire signed [15:0] output_adc1_b;
wire signed [15:0] output_adc2_a;
wire signed [15:0] output_adc2_b;
wire [63:0] corrected_waveform_data;
wire [63:0] waveform_data;

reg [47:0] baseline_sum_adc1_a;
reg [47:0] baseline_sum_adc1_b;
reg [47:0] baseline_sum_adc2_a;
reg [47:0] baseline_sum_adc2_b;

reg [15:0] baseline_mean_adc1_a;
reg [15:0] baseline_mean_adc1_b;
reg [15:0] baseline_mean_adc2_a;
reg [15:0] baseline_mean_adc2_b;

reg signed [15:0] peak_value_adc1_a;
reg signed [15:0] peak_value_adc1_b;
reg signed [15:0] peak_value_adc2_a;
reg signed [15:0] peak_value_adc2_b;

reg [15:0] peak_abs_adc1_a;
reg [15:0] peak_abs_adc1_b;
reg [15:0] peak_abs_adc2_a;
reg [15:0] peak_abs_adc2_b;

reg [31:0] peak_index_adc1_a;
reg [31:0] peak_index_adc1_b;
reg [31:0] peak_index_adc2_a;
reg [31:0] peak_index_adc2_b;

reg signed [63:0] integral_adc1_a;
reg signed [63:0] integral_adc1_b;
reg signed [63:0] integral_adc2_a;
reg signed [63:0] integral_adc2_b;

wire [47:0] baseline_sum_next_adc1_a;
wire [47:0] baseline_sum_next_adc1_b;
wire [47:0] baseline_sum_next_adc2_a;
wire [47:0] baseline_sum_next_adc2_b;

assign rd_adc1_a_sample = ring_rd_data[23:12];
assign rd_adc1_b_sample = ring_rd_data[11:0];
assign rd_adc2_a_sample = ring_rd_data[55:44];
assign rd_adc2_b_sample = ring_rd_data[43:32];

assign baseline_sum_next_adc1_a =
    baseline_sum_adc1_a + rd_adc1_a_sample;
assign baseline_sum_next_adc1_b =
    baseline_sum_adc1_b + rd_adc1_b_sample;
assign baseline_sum_next_adc2_a =
    baseline_sum_adc2_a + rd_adc2_a_sample;
assign baseline_sum_next_adc2_b =
    baseline_sum_adc2_b + rd_adc2_b_sample;

assign masked_raw_data = {
    8'd0,
    (output_channel_mask_latched[2] ? rd_adc2_a_sample : 12'd0),
    (output_channel_mask_latched[3] ? rd_adc2_b_sample : 12'd0),
    8'd0,
    (output_channel_mask_latched[0] ? rd_adc1_a_sample : 12'd0),
    (output_channel_mask_latched[1] ? rd_adc1_b_sample : 12'd0)
};

assign corrected_adc1_a =
    $signed({4'd0, rd_adc1_a_sample}) -
    $signed(baseline_mean_adc1_a);
assign corrected_adc1_b =
    $signed({4'd0, rd_adc1_b_sample}) -
    $signed(baseline_mean_adc1_b);
assign corrected_adc2_a =
    $signed({4'd0, rd_adc2_a_sample}) -
    $signed(baseline_mean_adc2_a);
assign corrected_adc2_b =
    $signed({4'd0, rd_adc2_b_sample}) -
    $signed(baseline_mean_adc2_b);

assign corrected_adc1_a_ext = {{48{corrected_adc1_a[15]}}, corrected_adc1_a};
assign corrected_adc1_b_ext = {{48{corrected_adc1_b[15]}}, corrected_adc1_b};
assign corrected_adc2_a_ext = {{48{corrected_adc2_a[15]}}, corrected_adc2_a};
assign corrected_adc2_b_ext = {{48{corrected_adc2_b[15]}}, corrected_adc2_b};

assign output_adc1_a =
    (!output_cfg_latched[2] || output_channel_mask_latched[0]) ?
        corrected_adc1_a : 16'sd0;
assign output_adc1_b =
    (!output_cfg_latched[2] || output_channel_mask_latched[1]) ?
        corrected_adc1_b : 16'sd0;
assign output_adc2_a =
    (!output_cfg_latched[2] || output_channel_mask_latched[2]) ?
        corrected_adc2_a : 16'sd0;
assign output_adc2_b =
    (!output_cfg_latched[2] || output_channel_mask_latched[3]) ?
        corrected_adc2_b : 16'sd0;

assign corrected_waveform_data = {
    output_adc2_b,
    output_adc2_a,
    output_adc1_b,
    output_adc1_a
};

assign waveform_data =
    baseline_correct_enable ?
        corrected_waveform_data :
    output_cfg_latched[2] ?
        masked_raw_data :
        ring_rd_data;

always @(*) begin
    case (footer_index)
        4'd0: footer_data = FOOTER_MAGIC;
        4'd1: footer_data = {
            16'h0001,
            8'h00,
            footer_output_cfg_report,
            8'h00,
            baseline_shift_latched,
            FOOTER_WORDS_U16
        };
        4'd2: footer_data = {
            baseline_mean_adc2_b,
            baseline_mean_adc2_a,
            baseline_mean_adc1_b,
            baseline_mean_adc1_a
        };
        4'd3: footer_data = {
            peak_value_adc2_b,
            peak_value_adc2_a,
            peak_value_adc1_b,
            peak_value_adc1_a
        };
        4'd4: footer_data = {
            peak_index_adc1_b,
            peak_index_adc1_a
        };
        4'd5: footer_data = {
            peak_index_adc2_b,
            peak_index_adc2_a
        };
        4'd6: footer_data = integral_adc1_a;
        4'd7: footer_data = integral_adc1_b;
        4'd8: footer_data = integral_adc2_a;
        4'd9: footer_data = integral_adc2_b;
        default: footer_data = 64'd0;
    endcase
end


always @(posedge clk) begin
    if (!resetn) begin
        play_state    <= ST_IDLE;
        play_index    <= 0;
        play_addr     <= 0;
        footer_index  <= 0;

        m_axis_tdata  <= 0;
        m_axis_tvalid <= 1'b0;
        m_axis_tlast  <= 1'b0;

        baseline_sum_adc1_a <= 0;
        baseline_sum_adc1_b <= 0;
        baseline_sum_adc2_a <= 0;
        baseline_sum_adc2_b <= 0;

        baseline_mean_adc1_a <= 0;
        baseline_mean_adc1_b <= 0;
        baseline_mean_adc2_a <= 0;
        baseline_mean_adc2_b <= 0;

        peak_value_adc1_a <= 0;
        peak_value_adc1_b <= 0;
        peak_value_adc2_a <= 0;
        peak_value_adc2_b <= 0;

        peak_abs_adc1_a <= 0;
        peak_abs_adc1_b <= 0;
        peak_abs_adc2_a <= 0;
        peak_abs_adc2_b <= 0;

        peak_index_adc1_a <= 0;
        peak_index_adc1_b <= 0;
        peak_index_adc2_a <= 0;
        peak_index_adc2_b <= 0;

        integral_adc1_a <= 0;
        integral_adc1_b <= 0;
        integral_adc2_a <= 0;
        integral_adc2_b <= 0;
    end else begin
        case (play_state)

            ST_IDLE: begin
                m_axis_tvalid <= 1'b0;
                m_axis_tlast  <= 1'b0;
                play_index    <= 0;
                footer_index  <= 0;

                if (frame_valid) begin
                    play_addr <= frame_start_addr;

                    baseline_sum_adc1_a <= 0;
                    baseline_sum_adc1_b <= 0;
                    baseline_sum_adc2_a <= 0;
                    baseline_sum_adc2_b <= 0;

                    baseline_mean_adc1_a <= 0;
                    baseline_mean_adc1_b <= 0;
                    baseline_mean_adc2_a <= 0;
                    baseline_mean_adc2_b <= 0;

                    peak_value_adc1_a <= 0;
                    peak_value_adc1_b <= 0;
                    peak_value_adc2_a <= 0;
                    peak_value_adc2_b <= 0;

                    peak_abs_adc1_a <= 0;
                    peak_abs_adc1_b <= 0;
                    peak_abs_adc2_a <= 0;
                    peak_abs_adc2_b <= 0;

                    peak_index_adc1_a <= 0;
                    peak_index_adc1_b <= 0;
                    peak_index_adc2_a <= 0;
                    peak_index_adc2_b <= 0;

                    integral_adc1_a <= 0;
                    integral_adc1_b <= 0;
                    integral_adc2_a <= 0;
                    integral_adc2_b <= 0;

                    if (analysis_enable) begin
                        play_state <= ST_BASE_READ;
                    end else begin
                        play_state <= ST_READ;
                    end
                end
            end


            ST_BASE_READ: begin
                play_state <= ST_BASE_WAIT;
            end


            ST_BASE_WAIT: begin
                if (
                    play_index ==
                    baseline_sample_count - 1'b1
                ) begin
                    baseline_mean_adc1_a <=
                        baseline_sum_next_adc1_a >>
                        baseline_shift_latched;
                    baseline_mean_adc1_b <=
                        baseline_sum_next_adc1_b >>
                        baseline_shift_latched;
                    baseline_mean_adc2_a <=
                        baseline_sum_next_adc2_a >>
                        baseline_shift_latched;
                    baseline_mean_adc2_b <=
                        baseline_sum_next_adc2_b >>
                        baseline_shift_latched;

                    play_index <= 0;
                    play_addr  <= frame_start_addr;

                    peak_value_adc1_a <= 0;
                    peak_value_adc1_b <= 0;
                    peak_value_adc2_a <= 0;
                    peak_value_adc2_b <= 0;

                    peak_abs_adc1_a <= 0;
                    peak_abs_adc1_b <= 0;
                    peak_abs_adc2_a <= 0;
                    peak_abs_adc2_b <= 0;

                    peak_index_adc1_a <= 0;
                    peak_index_adc1_b <= 0;
                    peak_index_adc2_a <= 0;
                    peak_index_adc2_b <= 0;

                    integral_adc1_a <= 0;
                    integral_adc1_b <= 0;
                    integral_adc2_a <= 0;
                    integral_adc2_b <= 0;

                    play_state <= ST_READ;
                end else begin
                    baseline_sum_adc1_a <= baseline_sum_next_adc1_a;
                    baseline_sum_adc1_b <= baseline_sum_next_adc1_b;
                    baseline_sum_adc2_a <= baseline_sum_next_adc2_a;
                    baseline_sum_adc2_b <= baseline_sum_next_adc2_b;

                    play_index <= play_index + 1'b1;
                    play_addr  <= play_addr + 1'b1;
                    play_state <= ST_BASE_READ;
                end
            end


            ST_READ: begin
                play_state <= ST_WAIT;
            end


            ST_WAIT: begin
                m_axis_tdata  <= waveform_data;
                m_axis_tvalid <= 1'b1;

                m_axis_tlast <=
                    (play_index ==
                     frame_size_latched - 1'b1) &&
                    !footer_enable;

                integral_adc1_a <= integral_adc1_a + corrected_adc1_a_ext;
                integral_adc1_b <= integral_adc1_b + corrected_adc1_b_ext;
                integral_adc2_a <= integral_adc2_a + corrected_adc2_a_ext;
                integral_adc2_b <= integral_adc2_b + corrected_adc2_b_ext;

                if (
                    (play_index == 0) ||
                    (abs16(corrected_adc1_a) > peak_abs_adc1_a)
                ) begin
                    peak_abs_adc1_a   <= abs16(corrected_adc1_a);
                    peak_value_adc1_a <= corrected_adc1_a;
                    peak_index_adc1_a <= play_index;
                end

                if (
                    (play_index == 0) ||
                    (abs16(corrected_adc1_b) > peak_abs_adc1_b)
                ) begin
                    peak_abs_adc1_b   <= abs16(corrected_adc1_b);
                    peak_value_adc1_b <= corrected_adc1_b;
                    peak_index_adc1_b <= play_index;
                end

                if (
                    (play_index == 0) ||
                    (abs16(corrected_adc2_a) > peak_abs_adc2_a)
                ) begin
                    peak_abs_adc2_a   <= abs16(corrected_adc2_a);
                    peak_value_adc2_a <= corrected_adc2_a;
                    peak_index_adc2_a <= play_index;
                end

                if (
                    (play_index == 0) ||
                    (abs16(corrected_adc2_b) > peak_abs_adc2_b)
                ) begin
                    peak_abs_adc2_b   <= abs16(corrected_adc2_b);
                    peak_value_adc2_b <= corrected_adc2_b;
                    peak_index_adc2_b <= play_index;
                end

                play_state <= ST_SEND;
            end


            ST_SEND: begin
                if (m_axis_tvalid && m_axis_tready) begin
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast  <= 1'b0;

                    if (
                        play_index ==
                        frame_size_latched - 1'b1
                    ) begin
                        if (footer_enable) begin
                            footer_index <= 0;
                            play_state   <= ST_FOOTER;
                        end else begin
                            play_state <= ST_IDLE;
                        end
                    end else begin
                        play_index <= play_index + 1'b1;
                        play_addr  <= play_addr + 1'b1;
                        play_state <= ST_READ;
                    end
                end
            end


            ST_FOOTER: begin
                if (!m_axis_tvalid) begin
                    m_axis_tdata  <= footer_data;
                    m_axis_tvalid <= 1'b1;
                    m_axis_tlast  <= (footer_index == FOOTER_WORDS-1);
                end else if (m_axis_tvalid && m_axis_tready) begin
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast  <= 1'b0;

                    if (footer_index == FOOTER_WORDS-1) begin
                        play_state <= ST_IDLE;
                    end else begin
                        footer_index <= footer_index + 1'b1;
                    end
                end
            end


            default: begin
                play_state <= ST_IDLE;
            end

        endcase
    end
end

endmodule
