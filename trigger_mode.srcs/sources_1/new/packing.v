module adc_axis64_packer #(
    parameter FIFO_DEPTH = 4096,
    parameter WORDS_PER_PACKET = 131072
)(
    // Reset inputs are already synchronized in their own clock domains.
    input             adc1_reset,
    input             adc2_reset,
    input             axis_resetn,

    input             adc1_clk,
    input      [11:0] adc1_data_a,
    input      [11:0] adc1_data_b,

    input             adc2_clk,
    input      [11:0] adc2_data_a,
    input      [11:0] adc2_data_b,

    input             m_axis_aclk,
    input             m_axis_aresetn,

    // Pulses synchronous to m_axis_aclk.
    input             alarm_clear,
    input             frame_alarm_clear,

    // Sticky status synchronized to m_axis_aclk.
    output            adc1_fifo_full_alarm,
    output            adc2_fifo_full_alarm,
    output            adc1_fifo_full_frame,
    output            adc2_fifo_full_frame,

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
wire        adc1_fifo_full;
wire        adc2_fifo_full;
wire        adc1_fifo_wr_rst_busy;
wire        adc1_fifo_rd_rst_busy;
wire        adc2_fifo_wr_rst_busy;
wire        adc2_fifo_rd_rst_busy;

wire axis_rst = ~(m_axis_aresetn & axis_resetn);

assign m_axis_tkeep = 8'hFF;

wire can_send;
assign can_send =
    (~axis_rst) &&
    (~adc1_fifo_rd_rst_busy) &&
    (~adc2_fifo_rd_rst_busy) &&
    (~adc1_fifo_empty) &&
    (~adc2_fifo_empty) &&
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
    .rst        (adc1_reset),
    .wr_clk     (adc1_clk),
    .wr_en      (!adc1_reset &&
                 !adc1_fifo_wr_rst_busy &&
                 !adc1_fifo_full),
    .din        ({adc1_data_a, adc1_data_b}),
    .full       (adc1_fifo_full),

    .rd_clk     (m_axis_aclk),
    .rd_en      (fifo_rd_en),
    .dout       (adc1_fifo_dout),
    .empty      (adc1_fifo_empty),
    .wr_rst_busy(adc1_fifo_wr_rst_busy),
    .rd_rst_busy(adc1_fifo_rd_rst_busy),

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
    .rst        (adc2_reset),
    .wr_clk     (adc2_clk),
    .wr_en      (!adc2_reset &&
                 !adc2_fifo_wr_rst_busy &&
                 !adc2_fifo_full),
    .din        ({adc2_data_a, adc2_data_b}),
    .full       (adc2_fifo_full),

    .rd_clk     (m_axis_aclk),
    .rd_en      (fifo_rd_en),
    .dout       (adc2_fifo_dout),
    .empty      (adc2_fifo_empty),
    .wr_rst_busy(adc2_fifo_wr_rst_busy),
    .rd_rst_busy(adc2_fifo_rd_rst_busy),

    .sleep      (1'b0),
    .injectsbiterr(1'b0),
    .injectdbiterr(1'b0)
);

// ============================================================
// FIFO full alarm capture
//
// global_* : remains set until alarm_clear or rst
// frame_*  : remains set until frame_alarm_clear or rst
//
// alarm_clear and frame_alarm_clear are m_axis_aclk pulses.
// Toggle synchronizers safely transfer each clear request into
// the two independent ADC write-clock domains.
// ============================================================

reg alarm_clear_toggle;
reg frame_clear_toggle;

always @(posedge m_axis_aclk) begin
    if (axis_rst) begin
        alarm_clear_toggle <= 1'b0;
        frame_clear_toggle <= 1'b0;
    end else begin
        if (alarm_clear)
            alarm_clear_toggle <= ~alarm_clear_toggle;

        if (frame_alarm_clear)
            frame_clear_toggle <= ~frame_clear_toggle;
    end
end

reg adc1_alarm_clr_ff1;
reg adc1_alarm_clr_ff2;
reg adc1_alarm_clr_ff2_d;
reg adc1_frame_clr_ff1;
reg adc1_frame_clr_ff2;
reg adc1_frame_clr_ff2_d;

reg adc2_alarm_clr_ff1;
reg adc2_alarm_clr_ff2;
reg adc2_alarm_clr_ff2_d;
reg adc2_frame_clr_ff1;
reg adc2_frame_clr_ff2;
reg adc2_frame_clr_ff2_d;

wire adc1_alarm_clear_pulse =
    adc1_alarm_clr_ff2 ^ adc1_alarm_clr_ff2_d;
wire adc1_frame_clear_pulse =
    adc1_frame_clr_ff2 ^ adc1_frame_clr_ff2_d;

wire adc2_alarm_clear_pulse =
    adc2_alarm_clr_ff2 ^ adc2_alarm_clr_ff2_d;
wire adc2_frame_clear_pulse =
    adc2_frame_clr_ff2 ^ adc2_frame_clr_ff2_d;

always @(posedge adc1_clk) begin
    if (adc1_reset) begin
        adc1_alarm_clr_ff1   <= 1'b0;
        adc1_alarm_clr_ff2   <= 1'b0;
        adc1_alarm_clr_ff2_d <= 1'b0;
        adc1_frame_clr_ff1   <= 1'b0;
        adc1_frame_clr_ff2   <= 1'b0;
        adc1_frame_clr_ff2_d <= 1'b0;
    end else begin
        adc1_alarm_clr_ff1   <= alarm_clear_toggle;
        adc1_alarm_clr_ff2   <= adc1_alarm_clr_ff1;
        adc1_alarm_clr_ff2_d <= adc1_alarm_clr_ff2;

        adc1_frame_clr_ff1   <= frame_clear_toggle;
        adc1_frame_clr_ff2   <= adc1_frame_clr_ff1;
        adc1_frame_clr_ff2_d <= adc1_frame_clr_ff2;
    end
end

always @(posedge adc2_clk) begin
    if (adc2_reset) begin
        adc2_alarm_clr_ff1   <= 1'b0;
        adc2_alarm_clr_ff2   <= 1'b0;
        adc2_alarm_clr_ff2_d <= 1'b0;
        adc2_frame_clr_ff1   <= 1'b0;
        adc2_frame_clr_ff2   <= 1'b0;
        adc2_frame_clr_ff2_d <= 1'b0;
    end else begin
        adc2_alarm_clr_ff1   <= alarm_clear_toggle;
        adc2_alarm_clr_ff2   <= adc2_alarm_clr_ff1;
        adc2_alarm_clr_ff2_d <= adc2_alarm_clr_ff2;

        adc2_frame_clr_ff1   <= frame_clear_toggle;
        adc2_frame_clr_ff2   <= adc2_frame_clr_ff1;
        adc2_frame_clr_ff2_d <= adc2_frame_clr_ff2;
    end
end

reg adc1_full_global_sticky;
reg adc2_full_global_sticky;
reg adc1_full_frame_sticky;
reg adc2_full_frame_sticky;

always @(posedge adc1_clk) begin
    if (adc1_reset) begin
        adc1_full_global_sticky <= 1'b0;
        adc1_full_frame_sticky  <= 1'b0;
    end else begin
        // Set has priority if clear and full occur together.
        if (adc1_alarm_clear_pulse)
            adc1_full_global_sticky <= adc1_fifo_full;
        else if (adc1_fifo_full)
            adc1_full_global_sticky <= 1'b1;

        if (adc1_frame_clear_pulse)
            adc1_full_frame_sticky <= adc1_fifo_full;
        else if (adc1_fifo_full)
            adc1_full_frame_sticky <= 1'b1;
    end
end

always @(posedge adc2_clk) begin
    if (adc2_reset) begin
        adc2_full_global_sticky <= 1'b0;
        adc2_full_frame_sticky  <= 1'b0;
    end else begin
        // Set has priority if clear and full occur together.
        if (adc2_alarm_clear_pulse)
            adc2_full_global_sticky <= adc2_fifo_full;
        else if (adc2_fifo_full)
            adc2_full_global_sticky <= 1'b1;

        if (adc2_frame_clear_pulse)
            adc2_full_frame_sticky <= adc2_fifo_full;
        else if (adc2_fifo_full)
            adc2_full_frame_sticky <= 1'b1;
    end
end

(* ASYNC_REG = "TRUE" *) reg adc1_global_sync1;
(* ASYNC_REG = "TRUE" *) reg adc1_global_sync2;
(* ASYNC_REG = "TRUE" *) reg adc2_global_sync1;
(* ASYNC_REG = "TRUE" *) reg adc2_global_sync2;
(* ASYNC_REG = "TRUE" *) reg adc1_frame_sync1;
(* ASYNC_REG = "TRUE" *) reg adc1_frame_sync2;
(* ASYNC_REG = "TRUE" *) reg adc2_frame_sync1;
(* ASYNC_REG = "TRUE" *) reg adc2_frame_sync2;

always @(posedge m_axis_aclk) begin
    if (axis_rst) begin
        adc1_global_sync1 <= 1'b0;
        adc1_global_sync2 <= 1'b0;
        adc2_global_sync1 <= 1'b0;
        adc2_global_sync2 <= 1'b0;
        adc1_frame_sync1  <= 1'b0;
        adc1_frame_sync2  <= 1'b0;
        adc2_frame_sync1  <= 1'b0;
        adc2_frame_sync2  <= 1'b0;
    end else begin
        adc1_global_sync1 <= adc1_full_global_sticky;
        adc1_global_sync2 <= adc1_global_sync1;
        adc2_global_sync1 <= adc2_full_global_sticky;
        adc2_global_sync2 <= adc2_global_sync1;

        adc1_frame_sync1 <= adc1_full_frame_sticky;
        adc1_frame_sync2 <= adc1_frame_sync1;
        adc2_frame_sync1 <= adc2_full_frame_sticky;
        adc2_frame_sync2 <= adc2_frame_sync1;
    end
end

assign adc1_fifo_full_alarm = adc1_global_sync2;
assign adc2_fifo_full_alarm = adc2_global_sync2;
assign adc1_fifo_full_frame = adc1_frame_sync2;
assign adc2_fifo_full_frame = adc2_frame_sync2;


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

    // FIFO full status from adc_axis64_packer, synchronous to clk.
    input  wire        adc1_fifo_full_frame,
    input  wire        adc2_fifo_full_frame,
    output reg         frame_alarm_clear,

    // cfg_control:
    //   bit1 = software trigger pulse
    //   bit1 = software trigger pulse
    //   bit2 = DMA/playback arm pulse and capture_overrun_sticky clear pulse
    //   bit5 = acquisition enable level
    // CONTROL[2] arms playback after Vitis has armed S2MM DMA.
    // CONTROL[5] snapshots the capture configuration on its rising edge and
    // enables pre-trigger acquisition. Deasserting CONTROL[5] prevents new
    // triggers; an already accepted post-trigger capture finishes normally.
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
    output reg         m_axis_tlast,

    output wire [1:0]  bank0_state,
    output wire [1:0]  bank1_state,
    output wire        active_capture_bank,
    output wire        active_playback_bank,
    output reg         capture_overrun_sticky
);

localparam integer BANK_DEPTH       = 155648;
localparam integer BANK_ADDR_W      = $clog2(BANK_DEPTH);
localparam [31:0]  BANK_DEPTH_U32   = BANK_DEPTH;
localparam integer DEFAULT_FRAME_SIZE = 131072;
localparam integer DEFAULT_PRE_SIZE = 16384;
localparam [31:0]  ADC2_PRETRIGGER_REDUCE = 32'd0;
localparam integer FOOTER_WORDS     = 16;
localparam [15:0]  FOOTER_WORDS_U16 = FOOTER_WORDS;

localparam [63:0] FOOTER_MAGIC      = 64'h00000000_FEE70001;

localparam [1:0] BANK_FREE          = 2'd0;
localparam [1:0] BANK_CAPTURE       = 2'd1;
localparam [1:0] BANK_READY         = 2'd2;
localparam [1:0] BANK_PLAY          = 2'd3;

localparam [1:0] CAP_PRE            = 2'd0;
localparam [1:0] CAP_POST           = 2'd1;
localparam [1:0] CAP_WAIT_FREE      = 2'd2;

localparam [2:0] ST_IDLE            = 3'd0;
localparam [2:0] ST_BASE_READ       = 3'd1;
localparam [2:0] ST_READ            = 3'd2;
localparam [2:0] ST_FOOTER          = 3'd3;


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


function [BANK_ADDR_W-1:0] bank_addr_inc;
    input [BANK_ADDR_W-1:0] addr;
begin
    if (addr == BANK_DEPTH - 1)
        bank_addr_inc = {BANK_ADDR_W{1'b0}};
    else
        bank_addr_inc = addr + 1'b1;
end
endfunction


function [BANK_ADDR_W-1:0] bank_addr_sub;
    input [BANK_ADDR_W-1:0] base;
    input [31:0] offset;
    reg [31:0] base_ext;
    reg [31:0] result;
begin
    base_ext = base;

    if (offset == BANK_DEPTH) begin
        result = base_ext;
    end else if (base_ext >= offset) begin
        result = base_ext - offset;
    end else begin
        result = BANK_DEPTH + base_ext - offset;
    end

    bank_addr_sub = result[BANK_ADDR_W-1:0];
end
endfunction


// ============================================================
// Configuration range protection
// ============================================================

wire [31:0] cfg_frame_requested_effective;

assign cfg_frame_requested_effective =
    (cfg_frame_size == 0) ?
        ((DEFAULT_FRAME_SIZE <= BANK_DEPTH) ?
            DEFAULT_FRAME_SIZE : BANK_DEPTH) :
    (cfg_frame_size > BANK_DEPTH) ?
        BANK_DEPTH :
        cfg_frame_size;


wire [31:0] cfg_pre_requested_effective;

assign cfg_pre_requested_effective =
    (cfg_pretrigger_size == 0) ?
        ((DEFAULT_PRE_SIZE <= cfg_frame_requested_effective) ?
            DEFAULT_PRE_SIZE : cfg_frame_requested_effective) :
    (cfg_pretrigger_size > cfg_frame_requested_effective) ?
        cfg_frame_requested_effective :
        cfg_pretrigger_size;


wire [31:0] cfg_decimation_requested_effective;

assign cfg_decimation_requested_effective =
    (cfg_sample_decimation == 0) ?
        32'd1 :
        cfg_sample_decimation;

wire [3:0] cfg_output_channel_mask_requested;

assign cfg_output_channel_mask_requested =
    (cfg_channel_mask[7:4] == 4'b0000) ?
        cfg_channel_mask[3:0] :
        cfg_channel_mask[7:4];

reg [31:0] active_frame_effective;
reg [31:0] active_pre_effective;
reg [31:0] active_decimation_effective;
reg [31:0] active_trigger_cfg;
reg [31:0] active_self_threshold;
reg [31:0] active_channel_mask_cfg;
reg [31:0] active_output_cfg;
reg [31:0] active_baseline_shift_cfg;
reg [3:0]  active_output_channel_mask;
reg [31:0] active_config_seq;

wire [31:0] cfg_frame_effective;
wire [31:0] cfg_pre_effective;
wire [31:0] cfg_decimation_effective;
wire [31:0] cfg_post_effective;
wire [31:0] cfg_capture_post_effective;

assign cfg_frame_effective      = active_frame_effective;
assign cfg_pre_effective        = active_pre_effective;
assign cfg_decimation_effective = active_decimation_effective;
assign cfg_post_effective       = cfg_frame_effective - cfg_pre_effective;
assign cfg_capture_post_effective =
    ((cfg_pre_effective != 0) && (cfg_frame_effective < BANK_DEPTH_U32)) ?
        (cfg_post_effective + ADC2_PRETRIGGER_REDUCE) :
        cfg_post_effective;


// ============================================================
// AXI Stream input
// ============================================================

assign s_axis_tready = 1'b1;
assign m_axis_tkeep  = 8'hFF;

wire input_sample_valid;

assign input_sample_valid =
    s_axis_tvalid && s_axis_tready;


// ============================================================
// Trigger synchronizers
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


reg arm_ff1;
reg arm_ff2;
reg arm_ff2_d;

always @(posedge clk) begin
    if (!resetn) begin
        arm_ff1   <= 1'b0;
        arm_ff2   <= 1'b0;
        arm_ff2_d <= 1'b0;
    end else begin
        arm_ff1   <= cfg_control[2];
        arm_ff2   <= arm_ff1;
        arm_ff2_d <= arm_ff2;
    end
end

wire arm_rise;

assign arm_rise = arm_ff2 && !arm_ff2_d;


reg acq_en_ff1;
reg acq_en_ff2;
reg acq_en_ff2_d;

always @(posedge clk) begin
    if (!resetn) begin
        acq_en_ff1   <= 1'b0;
        acq_en_ff2   <= 1'b0;
        acq_en_ff2_d <= 1'b0;
    end else begin
        acq_en_ff1   <= cfg_control[5];
        acq_en_ff2   <= acq_en_ff1;
        acq_en_ff2_d <= acq_en_ff2;
    end
end

wire acquisition_enable;
wire acquisition_enable_rise;

assign acquisition_enable      = acq_en_ff2;
assign acquisition_enable_rise = acq_en_ff2 && !acq_en_ff2_d;


// ============================================================
// Bank state and capture metadata
// ============================================================

reg [1:0] bank0_state_reg;
reg [1:0] bank1_state_reg;
reg       capture_bank;
reg       playback_bank;
reg [1:0] cap_state;

reg [BANK_ADDR_W-1:0] bank0_wr_ptr;
reg [BANK_ADDR_W-1:0] bank1_wr_ptr;
reg [31:0]            bank0_history_count;
reg [31:0]            bank1_history_count;
reg [BANK_ADDR_W-1:0] bank0_trig_ptr;
reg [BANK_ADDR_W-1:0] bank1_trig_ptr;
reg [BANK_ADDR_W-1:0] bank0_frame_start_addr;
reg [BANK_ADDR_W-1:0] bank1_frame_start_addr;

reg [31:0] bank0_frame_size;
reg [31:0] bank1_frame_size;
reg [31:0] bank0_pre_samples;
reg [31:0] bank1_pre_samples;
reg [31:0] bank0_post_samples;
reg [31:0] bank1_post_samples;
reg [31:0] bank0_decimation;
reg [31:0] bank1_decimation;
reg [31:0] bank0_trigger_cfg;
reg [31:0] bank1_trigger_cfg;
reg [31:0] bank0_self_threshold;
reg [31:0] bank1_self_threshold;
reg [31:0] bank0_channel_mask_cfg;
reg [31:0] bank1_channel_mask_cfg;
reg [31:0] bank0_config_seq;
reg [31:0] bank1_config_seq;
reg [31:0] bank0_output_cfg;
reg [31:0] bank1_output_cfg;
reg [7:0]  bank0_baseline_shift;
reg [7:0]  bank1_baseline_shift;
reg [3:0]  bank0_channel_mask;
reg [3:0]  bank1_channel_mask;
reg [1:0]  bank0_fifo_full_latched;
reg [1:0]  bank1_fifo_full_latched;
reg [31:0] event_frame_id_counter;
reg [31:0] bank0_event_frame_id;
reg [31:0] bank1_event_frame_id;
reg [15:0] bank0_ignored_trigger_count;
reg [15:0] bank1_ignored_trigger_count;
reg [15:0] bank0_busy_when_trigger_count;
reg [15:0] bank1_busy_when_trigger_count;

reg [31:0] post_count;

assign bank0_state          = bank0_state_reg;
assign bank1_state          = bank1_state_reg;
assign active_capture_bank  = capture_bank;
assign active_playback_bank = playback_bank;

wire [31:0] active_decimation;
wire [31:0] active_history_count;
wire        active_prebuffer_ready;
wire        capture_bank_is_valid;
wire        capture_in_pre_state;
wire [31:0] bank0_capture_post_samples;
wire [31:0] bank1_capture_post_samples;

assign active_decimation =
    (cap_state == CAP_POST) ?
        (capture_bank ? bank1_decimation : bank0_decimation) :
        cfg_decimation_effective;

assign active_history_count =
    capture_bank ? bank1_history_count : bank0_history_count;

assign active_prebuffer_ready =
    (cfg_pre_effective == 0) ||
    (active_history_count >= cfg_pre_effective);

assign capture_bank_is_valid =
    capture_bank ?
        (bank1_state_reg == BANK_CAPTURE) :
        (bank0_state_reg == BANK_CAPTURE);

assign capture_in_pre_state =
    (cap_state == CAP_PRE) &&
    capture_bank_is_valid;

assign bank0_capture_post_samples =
    ((bank0_pre_samples != 0) && (bank0_frame_size < BANK_DEPTH_U32)) ?
        (bank0_post_samples + ADC2_PRETRIGGER_REDUCE) :
        bank0_post_samples;

assign bank1_capture_post_samples =
    ((bank1_pre_samples != 0) && (bank1_frame_size < BANK_DEPTH_U32)) ?
        (bank1_post_samples + ADC2_PRETRIGGER_REDUCE) :
        bank1_post_samples;


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
assign self_threshold = active_self_threshold[11:0];
assign self_trigger_mask = active_channel_mask_cfg[3:0];
assign output_channel_mask = active_output_channel_mask;

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

wire self_trig_edge;
wire self_trigger_falling;

assign self_trigger_falling = active_trigger_cfg[3];

assign self_trig_edge =
    decimated_sample_valid &&
    (
        self_trigger_falling ?
            (!self_trig_level && self_trig_level_d) :
            (self_trig_level && !self_trig_level_d)
    );

wire [2:0] trigger_enable;
wire       trigger_event;

assign trigger_enable = active_trigger_cfg[2:0];

assign trigger_event =
    (trigger_enable[0] && ext_trig_rise) ||
    (trigger_enable[1] && soft_trig_rise) ||
    (trigger_enable[2] && self_trig_edge);


// ============================================================
// UltraRAM banks
// ============================================================

wire bank_capture_write_allowed;
wire bank0_wr_en;
wire bank1_wr_en;
wire [63:0] ring_wr_data;
reg  [11:0] ch3_delay_sample;
reg         ch3_delay_valid;

assign bank_capture_write_allowed =
    ((cap_state == CAP_PRE) && acquisition_enable) ||
    (cap_state == CAP_POST);

assign ring_wr_data = {
    s_axis_tdata[63:56],
    (ch3_delay_valid ? ch3_delay_sample : s_axis_tdata[55:44]),
    s_axis_tdata[43:0]
};

assign bank0_wr_en =
    decimated_sample_valid &&
    (capture_bank == 1'b0) &&
    (bank0_state_reg == BANK_CAPTURE) &&
    bank_capture_write_allowed;

assign bank1_wr_en =
    decimated_sample_valid &&
    (capture_bank == 1'b1) &&
    (bank1_state_reg == BANK_CAPTURE) &&
    bank_capture_write_allowed;

wire [BANK_ADDR_W-1:0] bank0_wr_ptr_after_current;
wire [BANK_ADDR_W-1:0] bank1_wr_ptr_after_current;

assign bank0_wr_ptr_after_current =
    bank0_wr_en ? bank_addr_inc(bank0_wr_ptr) : bank0_wr_ptr;

assign bank1_wr_ptr_after_current =
    bank1_wr_en ? bank_addr_inc(bank1_wr_ptr) : bank1_wr_ptr;

wire                 ring_rd_en;
wire [BANK_ADDR_W-1:0] ring_rd_addr;
wire [63:0]          bank0_rd_data;
wire [63:0]          bank1_rd_data;
wire [63:0]          selected_ring_rd_data;

xpm_memory_sdpram #(
    .ADDR_WIDTH_A            (BANK_ADDR_W),
    .ADDR_WIDTH_B            (BANK_ADDR_W),
    .AUTO_SLEEP_TIME         (0),
    .BYTE_WRITE_WIDTH_A      (64),
    .CLOCKING_MODE           ("common_clock"),
    .ECC_MODE                ("no_ecc"),
    .MEMORY_INIT_FILE        ("none"),
    .MEMORY_INIT_PARAM       ("0"),
    .MEMORY_OPTIMIZATION     ("true"),
    .MEMORY_PRIMITIVE        ("ultra"),
    .MEMORY_SIZE             (BANK_DEPTH * 64),
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
) bank0_mem_inst (
    .clka           (clk),
    .ena            (bank0_wr_en),
    .wea            (1'b1),
    .addra          (bank0_wr_ptr),
    .dina           (ring_wr_data),
    .injectsbiterra (1'b0),
    .injectdbiterra (1'b0),

    .clkb           (clk),
    .enb            (ring_rd_en && (playback_bank == 1'b0)),
    .addrb          (ring_rd_addr),
    .doutb          (bank0_rd_data),
    .rstb           (~resetn),
    .regceb         (1'b1),
    .sleep          (1'b0),
    .sbiterrb       (),
    .dbiterrb       ()
);

xpm_memory_sdpram #(
    .ADDR_WIDTH_A            (BANK_ADDR_W),
    .ADDR_WIDTH_B            (BANK_ADDR_W),
    .AUTO_SLEEP_TIME         (0),
    .BYTE_WRITE_WIDTH_A      (64),
    .CLOCKING_MODE           ("common_clock"),
    .ECC_MODE                ("no_ecc"),
    .MEMORY_INIT_FILE        ("none"),
    .MEMORY_INIT_PARAM       ("0"),
    .MEMORY_OPTIMIZATION     ("true"),
    .MEMORY_PRIMITIVE        ("ultra"),
    .MEMORY_SIZE             (BANK_DEPTH * 64),
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
) bank1_mem_inst (
    .clka           (clk),
    .ena            (bank1_wr_en),
    .wea            (1'b1),
    .addra          (bank1_wr_ptr),
    .dina           (ring_wr_data),
    .injectsbiterra (1'b0),
    .injectdbiterra (1'b0),

    .clkb           (clk),
    .enb            (ring_rd_en && (playback_bank == 1'b1)),
    .addrb          (ring_rd_addr),
    .doutb          (bank1_rd_data),
    .rstb           (~resetn),
    .regceb         (1'b1),
    .sleep          (1'b0),
    .sbiterrb       (),
    .dbiterrb       ()
);

assign selected_ring_rd_data =
    (playback_bank == 1'b0) ?
        bank0_rd_data :
        bank1_rd_data;


// ============================================================
// Playback latched frame configuration
// ============================================================

reg [BANK_ADDR_W-1:0] play_frame_start_addr;
reg [31:0]            play_frame_size;
reg [31:0]            play_pre_samples;
reg [31:0]            play_post_samples;
reg [31:0]            play_decimation;
reg [31:0]            play_trigger_cfg;
reg [31:0]            play_self_threshold;
reg [31:0]            play_channel_mask_cfg;
reg [31:0]            play_config_seq;
reg [31:0]            play_output_cfg;
reg [7:0]             play_baseline_shift;
reg [3:0]             play_channel_mask;
reg [1:0]             play_fifo_full_latched;

reg [2:0]             play_state;
reg [31:0]            play_index;
reg [31:0]            play_issue_index;
reg [BANK_ADDR_W-1:0] play_addr;
reg [4:0]             footer_index;
reg                   playback_armed;
reg                   acquisition_armed;
reg                   ring_rd_valid;
reg [31:0]            ring_rd_index;
reg [63:0]            adc2_align_delay0;
reg [63:0]            adc2_align_delay1;
reg [63:0]            adc2_align_delay2;
reg [63:0]            adc2_align_delay3;
reg [31:0]            adc2_align_delay_count;
reg                   waveform_last_loaded;
reg                   footer_last_loaded;
reg [31:0]            play_event_frame_id;
reg [15:0]            play_ignored_trigger_count;
reg [15:0]            play_busy_when_trigger_count;

wire footer_enable;
wire baseline_correct_enable;
wire analysis_enable;
wire [7:0] footer_output_cfg_report;
wire [7:0] footer_status_flags;
wire [31:0] baseline_sample_count;
wire       frame_invalid;

assign footer_enable =
    (FORCE_FOOTER != 0) ?
        1'b1 :
        play_output_cfg[1];

assign baseline_correct_enable = play_output_cfg[0];
assign analysis_enable = footer_enable || baseline_correct_enable;
assign footer_output_cfg_report =
    play_output_cfg[7:0] |
    ((FORCE_FOOTER != 0) ? 8'h02 : 8'h00);
assign footer_status_flags = {
    4'd0,
    frame_invalid,
    frame_invalid,
    play_fifo_full_latched[1],
    play_fifo_full_latched[0]
};
assign baseline_sample_count = 32'd1 << play_baseline_shift;
assign frame_invalid = |play_fifo_full_latched;

wire output_slot_available;
wire base_read_issue;
wire stream_read_slot_available;
wire stream_read_issue;
wire adc2_playback_advance;
wire [31:0] stream_read_limit;
wire stream_output_valid;
wire [31:0] stream_output_index;

assign output_slot_available =
    !m_axis_tvalid || m_axis_tready;

assign base_read_issue =
    (play_state == ST_BASE_READ) &&
    (play_issue_index < baseline_sample_count);

assign stream_read_slot_available =
    !ring_rd_valid ||
    (ring_rd_valid && output_slot_available);

assign stream_read_issue =
    (play_state == ST_READ) &&
    output_slot_available &&
    stream_read_slot_available &&
    !waveform_last_loaded &&
    (play_issue_index < stream_read_limit);

assign adc2_playback_advance =
    (ADC2_PRETRIGGER_REDUCE != 0) &&
    (play_pre_samples != 0) &&
    (play_frame_size < BANK_DEPTH_U32);

assign stream_read_limit =
    play_frame_size +
    (adc2_playback_advance ? ADC2_PRETRIGGER_REDUCE : 32'd0);

assign stream_output_valid =
    ring_rd_valid &&
    (
        !adc2_playback_advance ||
        (adc2_align_delay_count >= ADC2_PRETRIGGER_REDUCE)
    );

assign stream_output_index =
    adc2_playback_advance ?
        (ring_rd_index - ADC2_PRETRIGGER_REDUCE) :
        ring_rd_index;

assign ring_rd_en =
    base_read_issue ||
    stream_read_issue;

assign ring_rd_addr =
    play_addr;

wire frame_done;

assign frame_done =
    m_axis_tvalid &&
    m_axis_tready &&
    m_axis_tlast;

wire frame_busy;
wire fifo_busy;
wire capture_busy;
wire accept_trigger;

assign frame_busy =
    capture_busy ||
    fifo_busy;

assign fifo_busy = |current_fifo_full;

assign capture_busy =
    !(
        (cap_state == CAP_PRE) &&
        capture_bank_is_valid &&
        active_prebuffer_ready
    );

assign accept_trigger =
    trigger_event &&
    acquisition_armed &&
    acquisition_enable &&
    (cap_state == CAP_PRE) &&
    capture_bank_is_valid &&
    active_prebuffer_ready &&
    !fifo_busy;


// ============================================================
// Capture and playback state machines
// ============================================================

wire [1:0] current_fifo_full;

assign current_fifo_full = {
    adc2_fifo_full_frame,
    adc1_fifo_full_frame
};

always @(posedge clk) begin
    if (!resetn) begin
        bank0_state_reg <= BANK_CAPTURE;
        bank1_state_reg <= BANK_FREE;
        capture_bank    <= 1'b0;
        playback_bank   <= 1'b0;
        cap_state       <= CAP_PRE;

        bank0_wr_ptr <= 0;
        bank1_wr_ptr <= 0;
        bank0_history_count <= 0;
        bank1_history_count <= 0;
        bank0_trig_ptr <= 0;
        bank1_trig_ptr <= 0;
        bank0_frame_start_addr <= 0;
        bank1_frame_start_addr <= 0;

        bank0_frame_size <= DEFAULT_FRAME_SIZE;
        bank1_frame_size <= DEFAULT_FRAME_SIZE;
        bank0_pre_samples <= DEFAULT_PRE_SIZE;
        bank1_pre_samples <= DEFAULT_PRE_SIZE;
        bank0_post_samples <= DEFAULT_FRAME_SIZE - DEFAULT_PRE_SIZE;
        bank1_post_samples <= DEFAULT_FRAME_SIZE - DEFAULT_PRE_SIZE;
        bank0_decimation <= 1;
        bank1_decimation <= 1;
        bank0_trigger_cfg <= 32'd0;
        bank1_trigger_cfg <= 32'd0;
        bank0_self_threshold <= 32'd0;
        bank1_self_threshold <= 32'd0;
        bank0_channel_mask_cfg <= 32'h0000000F;
        bank1_channel_mask_cfg <= 32'h0000000F;
        bank0_config_seq <= 32'd0;
        bank1_config_seq <= 32'd0;
        bank0_output_cfg <= 32'd0;
        bank1_output_cfg <= 32'd0;
        bank0_baseline_shift <= 8'd10;
        bank1_baseline_shift <= 8'd10;
        bank0_channel_mask <= 4'hF;
        bank1_channel_mask <= 4'hF;
        bank0_fifo_full_latched <= 2'b00;
        bank1_fifo_full_latched <= 2'b00;
        event_frame_id_counter <= 32'd0;
        bank0_event_frame_id <= 32'd0;
        bank1_event_frame_id <= 32'd0;
        bank0_ignored_trigger_count <= 16'd0;
        bank1_ignored_trigger_count <= 16'd0;
        bank0_busy_when_trigger_count <= 16'd0;
        bank1_busy_when_trigger_count <= 16'd0;

        active_frame_effective <= DEFAULT_FRAME_SIZE;
        active_pre_effective <= DEFAULT_PRE_SIZE;
        active_decimation_effective <= 32'd1;
        active_trigger_cfg <= 32'd0;
        active_self_threshold <= 32'd0;
        active_channel_mask_cfg <= 32'h0000000F;
        active_output_cfg <= 32'd0;
        active_baseline_shift_cfg <= 32'd10;
        active_output_channel_mask <= 4'hF;
        active_config_seq <= 32'd0;

        ch3_delay_sample <= 12'd0;
        ch3_delay_valid <= 1'b0;

        post_count <= 0;
        frame_alarm_clear <= 1'b0;
        capture_overrun_sticky <= 1'b0;

        play_frame_start_addr <= 0;
        play_frame_size <= DEFAULT_FRAME_SIZE;
        play_pre_samples <= DEFAULT_PRE_SIZE;
        play_post_samples <= DEFAULT_FRAME_SIZE - DEFAULT_PRE_SIZE;
        play_decimation <= 32'd1;
        play_trigger_cfg <= 32'd0;
        play_self_threshold <= 32'd0;
        play_channel_mask_cfg <= 32'h0000000F;
        play_config_seq <= 32'd0;
        play_output_cfg <= 32'd0;
        play_baseline_shift <= 8'd10;
        play_channel_mask <= 4'hF;
        play_fifo_full_latched <= 2'b00;
        play_state <= ST_IDLE;
        play_index <= 0;
        play_issue_index <= 0;
        play_addr <= 0;
        footer_index <= 0;
        playback_armed <= 1'b0;
        acquisition_armed <= 1'b0;
        ring_rd_valid <= 1'b0;
        ring_rd_index <= 0;
        adc2_align_delay0 <= 64'd0;
        adc2_align_delay1 <= 64'd0;
        adc2_align_delay2 <= 64'd0;
        adc2_align_delay3 <= 64'd0;
        adc2_align_delay_count <= 32'd0;
        waveform_last_loaded <= 1'b0;
        footer_last_loaded <= 1'b0;
        play_event_frame_id <= 32'd0;
        play_ignored_trigger_count <= 16'd0;
        play_busy_when_trigger_count <= 16'd0;

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
        frame_alarm_clear <= 1'b0;

        if (arm_rise) begin
            capture_overrun_sticky <= 1'b0;
            playback_armed <= 1'b1;
            if (acquisition_enable)
                acquisition_armed <= 1'b1;
        end

        if (acquisition_enable_rise) begin
            acquisition_armed <= 1'b1;
            event_frame_id_counter <= 32'd0;
        end else if (!acquisition_enable && (cap_state != CAP_POST)) begin
            acquisition_armed <= 1'b0;
        end

        if (acquisition_enable_rise || (arm_rise && acquisition_enable)) begin
            active_frame_effective <= cfg_frame_requested_effective;
            active_pre_effective <= cfg_pre_requested_effective;
            active_decimation_effective <= cfg_decimation_requested_effective;
            active_trigger_cfg <= cfg_trigger_cfg;
            active_self_threshold <= cfg_self_threshold;
            active_channel_mask_cfg <= cfg_channel_mask;
            active_output_cfg <= cfg_output_cfg;
            active_baseline_shift_cfg <= cfg_baseline_shift;
            active_output_channel_mask <= cfg_output_channel_mask_requested;
            active_config_seq <= active_config_seq + 1'b1;
        end

        if (decimated_sample_valid) begin
            ch3_delay_sample <= adc2_a_sample;
            ch3_delay_valid <= 1'b1;
        end

        if (accept_trigger) begin
            event_frame_id_counter <= event_frame_id_counter + 1'b1;
            if (capture_bank == 1'b0) begin
                bank0_event_frame_id <= event_frame_id_counter + 1'b1;
                bank0_ignored_trigger_count <= 16'd0;
                bank0_busy_when_trigger_count <= 16'd0;
            end else begin
                bank1_event_frame_id <= event_frame_id_counter + 1'b1;
                bank1_ignored_trigger_count <= 16'd0;
                bank1_busy_when_trigger_count <= 16'd0;
            end
        end else if (trigger_event) begin
            if (capture_bank_is_valid) begin
                if (capture_bank == 1'b0) begin
                    bank0_ignored_trigger_count <=
                        bank0_ignored_trigger_count + 1'b1;

                    if (frame_busy)
                        bank0_busy_when_trigger_count <=
                            bank0_busy_when_trigger_count + 1'b1;
                end else begin
                    bank1_ignored_trigger_count <=
                        bank1_ignored_trigger_count + 1'b1;

                    if (frame_busy)
                        bank1_busy_when_trigger_count <=
                            bank1_busy_when_trigger_count + 1'b1;
                end
            end
        end

        // Input samples are always drained from adc_axis64_packer, but only
        // accept_trigger may start a frame. External triggers seen while the
        // active capture bank is busy are counted for debug and otherwise
        // ignored. They must not reset pointers/counters, clear FIFO frame
        // status, increment frame identity, or restart capture.
        if (bank0_wr_en) begin
            bank0_wr_ptr <= bank_addr_inc(bank0_wr_ptr);
            if (bank0_history_count < BANK_DEPTH)
                bank0_history_count <= bank0_history_count + 1'b1;
        end

        if (bank1_wr_en) begin
            bank1_wr_ptr <= bank_addr_inc(bank1_wr_ptr);
            if (bank1_history_count < BANK_DEPTH)
                bank1_history_count <= bank1_history_count + 1'b1;
        end

        case (play_state)
            ST_IDLE: begin
                m_axis_tvalid <= 1'b0;
                m_axis_tlast  <= 1'b0;
                play_index    <= 0;
                footer_index  <= 0;

                if (playback_armed && (bank0_state_reg == BANK_READY)) begin
                    bank0_state_reg <= BANK_PLAY;
                    playback_bank   <= 1'b0;
                    playback_armed  <= 1'b0;
                    play_frame_start_addr <= bank0_frame_start_addr;
                    play_frame_size <= bank0_frame_size;
                    play_pre_samples <= bank0_pre_samples;
                    play_post_samples <= bank0_post_samples;
                    play_decimation <= bank0_decimation;
                    play_trigger_cfg <= bank0_trigger_cfg;
                    play_self_threshold <= bank0_self_threshold;
                    play_channel_mask_cfg <= bank0_channel_mask_cfg;
                    play_config_seq <= bank0_config_seq;
                    play_output_cfg <= bank0_output_cfg;
                    play_baseline_shift <= bank0_baseline_shift;
                    play_channel_mask <= bank0_channel_mask;
                    play_fifo_full_latched <= bank0_fifo_full_latched;
                    play_event_frame_id <= bank0_event_frame_id;
                    play_ignored_trigger_count <=
                        bank0_ignored_trigger_count;
                    play_busy_when_trigger_count <=
                        bank0_busy_when_trigger_count;
                    play_addr <= bank0_frame_start_addr;
                    play_issue_index <= 0;
                    ring_rd_valid <= 1'b0;
                    ring_rd_index <= 0;
                    adc2_align_delay0 <= 64'd0;
                    adc2_align_delay1 <= 64'd0;
                    adc2_align_delay2 <= 64'd0;
                    adc2_align_delay3 <= 64'd0;
                    adc2_align_delay_count <= 32'd0;
                    waveform_last_loaded <= 1'b0;
                    footer_last_loaded <= 1'b0;

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

                    if (
                        (FORCE_FOOTER != 0) ||
                        bank0_output_cfg[1] ||
                        bank0_output_cfg[0]
                    )
                        play_state <= ST_BASE_READ;
                    else
                        play_state <= ST_READ;
                end else if (playback_armed && (bank1_state_reg == BANK_READY)) begin
                    bank1_state_reg <= BANK_PLAY;
                    playback_bank   <= 1'b1;
                    playback_armed  <= 1'b0;
                    play_frame_start_addr <= bank1_frame_start_addr;
                    play_frame_size <= bank1_frame_size;
                    play_pre_samples <= bank1_pre_samples;
                    play_post_samples <= bank1_post_samples;
                    play_decimation <= bank1_decimation;
                    play_trigger_cfg <= bank1_trigger_cfg;
                    play_self_threshold <= bank1_self_threshold;
                    play_channel_mask_cfg <= bank1_channel_mask_cfg;
                    play_config_seq <= bank1_config_seq;
                    play_output_cfg <= bank1_output_cfg;
                    play_baseline_shift <= bank1_baseline_shift;
                    play_channel_mask <= bank1_channel_mask;
                    play_fifo_full_latched <= bank1_fifo_full_latched;
                    play_event_frame_id <= bank1_event_frame_id;
                    play_ignored_trigger_count <=
                        bank1_ignored_trigger_count;
                    play_busy_when_trigger_count <=
                        bank1_busy_when_trigger_count;
                    play_addr <= bank1_frame_start_addr;
                    play_issue_index <= 0;
                    ring_rd_valid <= 1'b0;
                    ring_rd_index <= 0;
                    adc2_align_delay0 <= 64'd0;
                    adc2_align_delay1 <= 64'd0;
                    adc2_align_delay2 <= 64'd0;
                    adc2_align_delay3 <= 64'd0;
                    adc2_align_delay_count <= 32'd0;
                    waveform_last_loaded <= 1'b0;
                    footer_last_loaded <= 1'b0;

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

                    if (
                        (FORCE_FOOTER != 0) ||
                        bank1_output_cfg[1] ||
                        bank1_output_cfg[0]
                    )
                        play_state <= ST_BASE_READ;
                    else
                        play_state <= ST_READ;
                end
            end

            ST_BASE_READ: begin
                if (base_read_issue) begin
                    ring_rd_valid <= 1'b1;
                    ring_rd_index <= play_issue_index;
                    play_issue_index <= play_issue_index + 1'b1;
                    play_addr <= bank_addr_inc(play_addr);
                end else if (ring_rd_valid) begin
                    ring_rd_valid <= 1'b0;
                end

                if (ring_rd_valid) begin
                    if (ring_rd_index == baseline_sample_count - 1'b1) begin
                        baseline_mean_adc1_a <=
                            baseline_sum_next_adc1_a >>
                            play_baseline_shift;
                        baseline_mean_adc1_b <=
                            baseline_sum_next_adc1_b >>
                            play_baseline_shift;
                        baseline_mean_adc2_a <=
                            baseline_sum_next_adc2_a >>
                            play_baseline_shift;
                        baseline_mean_adc2_b <=
                            baseline_sum_next_adc2_b >>
                            play_baseline_shift;

                        play_index <= 0;
                        play_issue_index <= 0;
                        play_addr <= play_frame_start_addr;
                        ring_rd_valid <= 1'b0;
                        adc2_align_delay0 <= 64'd0;
                        adc2_align_delay1 <= 64'd0;
                        adc2_align_delay2 <= 64'd0;
                        adc2_align_delay3 <= 64'd0;
                        adc2_align_delay_count <= 32'd0;

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
                    end
                end
            end

            ST_READ: begin
                if (m_axis_tvalid && m_axis_tready && waveform_last_loaded) begin
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast  <= 1'b0;
                    waveform_last_loaded <= 1'b0;
                    ring_rd_valid <= 1'b0;

                    if (footer_enable) begin
                        footer_index <= 0;
                        footer_last_loaded <= 1'b0;
                        play_state <= ST_FOOTER;
                    end else begin
                        if (playback_bank == 1'b0)
                            bank0_state_reg <= BANK_FREE;
                        else
                            bank1_state_reg <= BANK_FREE;

                        play_state <= ST_IDLE;
                    end
                end else begin
                    if (stream_output_valid && output_slot_available) begin
                        m_axis_tdata  <= waveform_data;
                        m_axis_tvalid <= 1'b1;
                        m_axis_tlast  <=
                            (stream_output_index == play_frame_size - 1'b1) &&
                            !footer_enable;
                        play_index <= stream_output_index;

                        integral_adc1_a <=
                            integral_adc1_a + corrected_adc1_a_ext;
                        integral_adc1_b <=
                            integral_adc1_b + corrected_adc1_b_ext;
                        integral_adc2_a <=
                            integral_adc2_a + corrected_adc2_a_ext;
                        integral_adc2_b <=
                            integral_adc2_b + corrected_adc2_b_ext;

                        if (
                            (stream_output_index == 0) ||
                            (abs16(corrected_adc1_a) > peak_abs_adc1_a)
                        ) begin
                            peak_abs_adc1_a   <= abs16(corrected_adc1_a);
                            peak_value_adc1_a <= corrected_adc1_a;
                            peak_index_adc1_a <= stream_output_index;
                        end

                        if (
                            (stream_output_index == 0) ||
                            (abs16(corrected_adc1_b) > peak_abs_adc1_b)
                        ) begin
                            peak_abs_adc1_b   <= abs16(corrected_adc1_b);
                            peak_value_adc1_b <= corrected_adc1_b;
                            peak_index_adc1_b <= stream_output_index;
                        end

                        if (
                            (stream_output_index == 0) ||
                            (abs16(corrected_adc2_a) > peak_abs_adc2_a)
                        ) begin
                            peak_abs_adc2_a   <= abs16(corrected_adc2_a);
                            peak_value_adc2_a <= corrected_adc2_a;
                            peak_index_adc2_a <= stream_output_index;
                        end

                        if (
                            (stream_output_index == 0) ||
                            (abs16(corrected_adc2_b) > peak_abs_adc2_b)
                        ) begin
                            peak_abs_adc2_b   <= abs16(corrected_adc2_b);
                            peak_value_adc2_b <= corrected_adc2_b;
                            peak_index_adc2_b <= stream_output_index;
                        end

                        if (stream_output_index == play_frame_size - 1'b1)
                            waveform_last_loaded <= 1'b1;
                    end else if (m_axis_tvalid && m_axis_tready) begin
                        m_axis_tvalid <= 1'b0;
                        m_axis_tlast  <= 1'b0;
                    end

                    if (stream_read_issue) begin
                        if (ring_rd_valid && output_slot_available) begin
                            adc2_align_delay3 <= adc2_align_delay2;
                            adc2_align_delay2 <= adc2_align_delay1;
                            adc2_align_delay1 <= adc2_align_delay0;
                            adc2_align_delay0 <= selected_ring_rd_data;
                            if (adc2_align_delay_count < ADC2_PRETRIGGER_REDUCE)
                                adc2_align_delay_count <=
                                    adc2_align_delay_count + 1'b1;
                        end
                        ring_rd_valid <= 1'b1;
                        ring_rd_index <= play_issue_index;
                        play_issue_index <= play_issue_index + 1'b1;
                        play_addr <= bank_addr_inc(play_addr);
                    end else if (ring_rd_valid && output_slot_available) begin
                        ring_rd_valid <= 1'b0;
                        adc2_align_delay3 <= adc2_align_delay2;
                        adc2_align_delay2 <= adc2_align_delay1;
                        adc2_align_delay1 <= adc2_align_delay0;
                        adc2_align_delay0 <= selected_ring_rd_data;
                        if (adc2_align_delay_count < ADC2_PRETRIGGER_REDUCE)
                            adc2_align_delay_count <=
                                adc2_align_delay_count + 1'b1;
                    end
                end
            end

            ST_FOOTER: begin
                if (m_axis_tvalid && m_axis_tready && footer_last_loaded) begin
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast  <= 1'b0;
                    footer_last_loaded <= 1'b0;

                    if (playback_bank == 1'b0)
                        bank0_state_reg <= BANK_FREE;
                    else
                        bank1_state_reg <= BANK_FREE;

                    play_state <= ST_IDLE;
                end else if (output_slot_available) begin
                    m_axis_tdata  <= footer_data;
                    m_axis_tvalid <= 1'b1;
                    m_axis_tlast  <= (footer_index == FOOTER_WORDS-1);

                    if (footer_index == FOOTER_WORDS-1) begin
                        footer_last_loaded <= 1'b1;
                    end else begin
                        footer_index <= footer_index + 1'b1;
                    end
                end
            end

            default: begin
                play_state <= ST_IDLE;
            end
        endcase

        case (cap_state)
            CAP_PRE: begin
                post_count <= 0;

                // Non-power-of-two BANK_DEPTH is handled only through
                // bank_addr_inc and bank_addr_sub. No natural bit overflow is
                // used as a wrap condition.
                if (accept_trigger) begin
                    if (capture_bank == 1'b0) begin
                        bank0_frame_size <= cfg_frame_effective;
                        bank0_pre_samples <= cfg_pre_effective;
                        bank0_post_samples <=
                            cfg_post_effective;
                        bank0_decimation <= cfg_decimation_effective;
                        bank0_trigger_cfg <= active_trigger_cfg;
                        bank0_self_threshold <= active_self_threshold;
                        bank0_channel_mask_cfg <= active_channel_mask_cfg;
                        bank0_config_seq <= active_config_seq;
                        bank0_output_cfg <= active_output_cfg;
                        bank0_baseline_shift <=
                            effective_baseline_shift(
                                active_baseline_shift_cfg,
                                (cfg_pre_effective == 0) ?
                                    32'd1 :
                                    cfg_pre_effective
                            );
                        bank0_channel_mask <= output_channel_mask;
                        bank0_trig_ptr <= bank0_wr_ptr_after_current;
                        bank0_fifo_full_latched <= current_fifo_full;

                        if (cfg_capture_post_effective == 0) begin
                            bank0_frame_start_addr <=
                                bank_addr_sub(
                                    bank0_wr_ptr_after_current,
                                    cfg_pre_effective
                                );
                            bank0_state_reg <= BANK_READY;
                            if (
                                (bank1_state_reg == BANK_FREE) ||
                                (frame_done && playback_bank == 1'b1)
                            ) begin
                                capture_bank <= 1'b1;
                                bank1_state_reg <= BANK_CAPTURE;
                                bank1_wr_ptr <= 0;
                                bank1_history_count <= 0;
                                bank1_fifo_full_latched <= 2'b00;
                                frame_alarm_clear <= 1'b1;
                                cap_state <= CAP_PRE;
                            end else begin
                                cap_state <= CAP_WAIT_FREE;
                            end
                        end else begin
                            cap_state <= CAP_POST;
                        end
                    end else begin
                        bank1_frame_size <= cfg_frame_effective;
                        bank1_pre_samples <= cfg_pre_effective;
                        bank1_post_samples <=
                            cfg_post_effective;
                        bank1_decimation <= cfg_decimation_effective;
                        bank1_trigger_cfg <= active_trigger_cfg;
                        bank1_self_threshold <= active_self_threshold;
                        bank1_channel_mask_cfg <= active_channel_mask_cfg;
                        bank1_config_seq <= active_config_seq;
                        bank1_output_cfg <= active_output_cfg;
                        bank1_baseline_shift <=
                            effective_baseline_shift(
                                active_baseline_shift_cfg,
                                (cfg_pre_effective == 0) ?
                                    32'd1 :
                                    cfg_pre_effective
                            );
                        bank1_channel_mask <= output_channel_mask;
                        bank1_trig_ptr <= bank1_wr_ptr_after_current;
                        bank1_fifo_full_latched <= current_fifo_full;

                        if (cfg_capture_post_effective == 0) begin
                            bank1_frame_start_addr <=
                                bank_addr_sub(
                                    bank1_wr_ptr_after_current,
                                    cfg_pre_effective
                                );
                            bank1_state_reg <= BANK_READY;
                            if (
                                (bank0_state_reg == BANK_FREE) ||
                                (frame_done && playback_bank == 1'b0)
                            ) begin
                                capture_bank <= 1'b0;
                                bank0_state_reg <= BANK_CAPTURE;
                                bank0_wr_ptr <= 0;
                                bank0_history_count <= 0;
                                bank0_fifo_full_latched <= 2'b00;
                                frame_alarm_clear <= 1'b1;
                                cap_state <= CAP_PRE;
                            end else begin
                                cap_state <= CAP_WAIT_FREE;
                            end
                        end else begin
                            cap_state <= CAP_POST;
                        end
                    end
                end
            end

            CAP_POST: begin
                // Additional triggers during post-trigger capture are ignored.
                // FIFO-full status continues to accumulate into the bank that
                // owns this frame, and the bank is released only after DMA
                // accepts the final waveform/footer TLAST word.
                if (capture_bank == 1'b0)
                    bank0_fifo_full_latched <=
                        bank0_fifo_full_latched | current_fifo_full;
                else
                    bank1_fifo_full_latched <=
                        bank1_fifo_full_latched | current_fifo_full;

                if (decimated_sample_valid) begin
                    if (capture_bank == 1'b0) begin
                        if (post_count + 1'b1 >= bank0_capture_post_samples) begin
                            bank0_frame_start_addr <=
                                bank_addr_sub(
                                    bank0_trig_ptr,
                                    bank0_pre_samples
                                );
                            bank0_state_reg <= BANK_READY;
                            post_count <= 0;
                            if (
                                (bank1_state_reg == BANK_FREE) ||
                                (frame_done && playback_bank == 1'b1)
                            ) begin
                                capture_bank <= 1'b1;
                                bank1_state_reg <= BANK_CAPTURE;
                                bank1_wr_ptr <= 0;
                                bank1_history_count <= 0;
                                bank1_fifo_full_latched <= 2'b00;
                                frame_alarm_clear <= 1'b1;
                                cap_state <= CAP_PRE;
                            end else begin
                                cap_state <= CAP_WAIT_FREE;
                            end
                        end else begin
                            post_count <= post_count + 1'b1;
                        end
                    end else begin
                        if (post_count + 1'b1 >= bank1_capture_post_samples) begin
                            bank1_frame_start_addr <=
                                bank_addr_sub(
                                    bank1_trig_ptr,
                                    bank1_pre_samples
                                );
                            bank1_state_reg <= BANK_READY;
                            post_count <= 0;
                            if (
                                (bank0_state_reg == BANK_FREE) ||
                                (frame_done && playback_bank == 1'b0)
                            ) begin
                                capture_bank <= 1'b0;
                                bank0_state_reg <= BANK_CAPTURE;
                                bank0_wr_ptr <= 0;
                                bank0_history_count <= 0;
                                bank0_fifo_full_latched <= 2'b00;
                                frame_alarm_clear <= 1'b1;
                                cap_state <= CAP_PRE;
                            end else begin
                                cap_state <= CAP_WAIT_FREE;
                            end
                        end else begin
                            post_count <= post_count + 1'b1;
                        end
                    end
                end
            end

            CAP_WAIT_FREE: begin
                if (
                    (bank0_state_reg == BANK_FREE) ||
                    (frame_done && playback_bank == 1'b0)
                ) begin
                    capture_bank <= 1'b0;
                    bank0_state_reg <= BANK_CAPTURE;
                    bank0_wr_ptr <= 0;
                    bank0_history_count <= 0;
                    bank0_fifo_full_latched <= 2'b00;
                    frame_alarm_clear <= 1'b1;
                    post_count <= 0;
                    cap_state <= CAP_PRE;
                end else if (
                    (bank1_state_reg == BANK_FREE) ||
                    (frame_done && playback_bank == 1'b1)
                ) begin
                    capture_bank <= 1'b1;
                    bank1_state_reg <= BANK_CAPTURE;
                    bank1_wr_ptr <= 0;
                    bank1_history_count <= 0;
                    bank1_fifo_full_latched <= 2'b00;
                    frame_alarm_clear <= 1'b1;
                    post_count <= 0;
                    cap_state <= CAP_PRE;
                end
            end

            default: begin
                cap_state <= CAP_WAIT_FREE;
            end
        endcase
    end
end


// ============================================================
// Waveform analysis and footer generation
// ============================================================

reg [63:0] footer_data;

wire [63:0] adc2_advanced_ring_rd_data;
wire [63:0] effective_ring_rd_data;
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

assign adc2_advanced_ring_rd_data = {
    selected_ring_rd_data[63:32],
    adc2_align_delay3[31:0]
};

assign effective_ring_rd_data =
    ((play_state == ST_READ) && adc2_playback_advance) ?
        adc2_advanced_ring_rd_data :
        selected_ring_rd_data;

assign rd_adc1_a_sample = effective_ring_rd_data[23:12];
assign rd_adc1_b_sample = effective_ring_rd_data[11:0];
assign rd_adc2_a_sample = effective_ring_rd_data[55:44];
assign rd_adc2_b_sample = effective_ring_rd_data[43:32];

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
    (play_channel_mask[2] ? rd_adc2_a_sample : 12'd0),
    (play_channel_mask[3] ? rd_adc2_b_sample : 12'd0),
    8'd0,
    (play_channel_mask[0] ? rd_adc1_a_sample : 12'd0),
    (play_channel_mask[1] ? rd_adc1_b_sample : 12'd0)
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
    (!play_output_cfg[2] || play_channel_mask[0]) ?
        corrected_adc1_a : 16'sd0;
assign output_adc1_b =
    (!play_output_cfg[2] || play_channel_mask[1]) ?
        corrected_adc1_b : 16'sd0;
assign output_adc2_a =
    (!play_output_cfg[2] || play_channel_mask[2]) ?
        corrected_adc2_a : 16'sd0;
assign output_adc2_b =
    (!play_output_cfg[2] || play_channel_mask[3]) ?
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
    play_output_cfg[2] ?
        masked_raw_data :
        effective_ring_rd_data;

always @(*) begin
    case (footer_index)
        5'd0: footer_data = FOOTER_MAGIC;
        5'd1: footer_data = {
            16'h0002,
            8'h00,
            footer_output_cfg_report,
            footer_status_flags,
            play_baseline_shift,
            FOOTER_WORDS_U16
        };
        5'd2: footer_data = {
            baseline_mean_adc2_b,
            baseline_mean_adc2_a,
            baseline_mean_adc1_b,
            baseline_mean_adc1_a
        };
        5'd3: footer_data = {
            peak_value_adc2_b,
            peak_value_adc2_a,
            peak_value_adc1_b,
            peak_value_adc1_a
        };
        5'd4: footer_data = {
            peak_index_adc1_b,
            peak_index_adc1_a
        };
        5'd5: footer_data = {
            peak_index_adc2_b,
            peak_index_adc2_a
        };
        5'd6: footer_data = integral_adc1_a;
        5'd7: footer_data = integral_adc1_b;
        5'd8: footer_data = integral_adc2_a;
        5'd9: footer_data = integral_adc2_b;
        5'd10: footer_data = {
            play_event_frame_id,
            play_ignored_trigger_count,
            play_busy_when_trigger_count
        };
        5'd11: footer_data = {
            play_frame_size,
            play_pre_samples
        };
        5'd12: footer_data = {
            play_post_samples,
            play_decimation
        };
        5'd13: footer_data = {
            play_trigger_cfg,
            play_self_threshold
        };
        5'd14: footer_data = {
            play_channel_mask_cfg,
            play_output_cfg
        };
        5'd15: footer_data = {
            24'd0,
            play_baseline_shift,
            play_config_seq
        };
        default: footer_data = 64'd0;
    endcase
end

endmodule
