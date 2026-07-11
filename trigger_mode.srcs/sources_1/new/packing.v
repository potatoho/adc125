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
    parameter integer PRE_SAMPLES   = 16384,
    parameter integer POST_SAMPLES  = 114688,
    parameter integer TOTAL_SAMPLES = PRE_SAMPLES + POST_SAMPLES,

    parameter integer DEPTH  = 262144,
    parameter integer ADDR_W = $clog2(DEPTH)
)(
    input  wire        clk,
    input  wire        resetn,

    input  wire [63:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,

    input  wire        trig_in,

    output reg  [63:0] m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output wire [7:0]  m_axis_tkeep,
    output reg         m_axis_tlast
);

localparam [ADDR_W-1:0] PRE_SAMPLES_AW = PRE_SAMPLES;
localparam integer COUNT_W = $clog2(TOTAL_SAMPLES + 1);

assign s_axis_tready = 1'b1;
assign m_axis_tkeep  = 8'hFF;

wire sample_valid = s_axis_tvalid & s_axis_tready;

// trigger sync
reg trig_ff1, trig_ff2;

always @(posedge clk) begin
    if (!resetn) begin
        trig_ff1 <= 1'b0;
        trig_ff2 <= 1'b0;
    end else begin
        trig_ff1 <= trig_in;
        trig_ff2 <= trig_ff1;
    end
end

wire trig_rise = trig_ff1 & ~trig_ff2;

// ring buffer control
reg [ADDR_W-1:0] wr_ptr;
reg [COUNT_W-1:0] sample_seen;
reg armed;

wire ring_wr_en;
wire ring_rd_en;
wire [ADDR_W-1:0] ring_rd_addr;
wire [63:0] ring_rd_data;

// Explicit UltraRAM implementation (simple dual port, common clock)
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

always @(posedge clk) begin
    if (!resetn) begin
        wr_ptr      <= 0;
        sample_seen <= 0;
        armed       <= 1'b0;
    end else begin
        if (ring_wr_en) begin
            wr_ptr <= wr_ptr + 1'b1;

            if (!armed) begin
                if (sample_seen < PRE_SAMPLES)
                    sample_seen <= sample_seen + 1'b1;

                if (sample_seen + 1'b1 >= PRE_SAMPLES)
                    armed <= 1'b1;
            end
        end
    end
end

localparam CAP_IDLE = 2'd0;
localparam CAP_POST = 2'd1;
localparam CAP_HOLD = 2'd2;

reg [1:0] cap_state;
reg [ADDR_W-1:0] trig_ptr;
reg [ADDR_W-1:0] frame_start_addr;
reg [COUNT_W-1:0] post_cnt;
reg frame_valid;

wire frame_done =
    m_axis_tvalid && m_axis_tready && m_axis_tlast;

// Freeze the captured ring contents until the final AXI word is accepted.
assign ring_wr_en = sample_valid && (cap_state != CAP_HOLD);

always @(posedge clk) begin
    if (!resetn) begin
        cap_state   <= CAP_IDLE;
        trig_ptr    <= 0;
        frame_start_addr <= 0;
        post_cnt    <= 0;
        frame_valid <= 1'b0;
    end else begin
        case (cap_state)
            CAP_IDLE: begin
                post_cnt <= 0;

                if (trig_rise && armed && !frame_valid) begin
                    trig_ptr  <= wr_ptr;
                    cap_state <= CAP_POST;
                end
            end

            CAP_POST: begin
                if (sample_valid) begin
                    if (post_cnt + 1'b1 >= POST_SAMPLES) begin
                        frame_start_addr <= trig_ptr - PRE_SAMPLES_AW;
                        frame_valid      <= 1'b1;
                        cap_state        <= CAP_HOLD;
                    end else begin
                        post_cnt <= post_cnt + 1'b1;
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

// AXIS playback directly from the frozen UltraRAM ring buffer
localparam ST_IDLE = 2'd0;
localparam ST_READ = 2'd1;
localparam ST_WAIT = 2'd2;
localparam ST_SEND = 2'd3;

reg [1:0] state;
reg [COUNT_W-1:0] play_idx;
reg [ADDR_W-1:0] play_addr;

assign ring_rd_en   = (state == ST_READ);
assign ring_rd_addr = play_addr;

always @(posedge clk) begin
    if (!resetn) begin
        state         <= ST_IDLE;
        play_idx      <= 0;
        play_addr     <= 0;
        m_axis_tdata  <= 0;
        m_axis_tvalid <= 1'b0;
        m_axis_tlast  <= 1'b0;
    end else begin
        case (state)
            ST_IDLE: begin
                m_axis_tvalid <= 1'b0;
                m_axis_tlast  <= 1'b0;
                play_idx      <= 0;

                if (frame_valid) begin
                    play_addr <= frame_start_addr;
                    state <= ST_READ;
                end
            end

            ST_READ: begin
                // Issue one synchronous UltraRAM read.
                state <= ST_WAIT;
            end

            ST_WAIT: begin
                // READ_LATENCY_B=1: ring_rd_data is now valid.
                m_axis_tdata  <= ring_rd_data;
                m_axis_tvalid <= 1'b1;
                m_axis_tlast  <= (play_idx == TOTAL_SAMPLES-1);
                state         <= ST_SEND;
            end

            ST_SEND: begin
                if (m_axis_tvalid && m_axis_tready) begin
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast  <= 1'b0;

                    if (play_idx == TOTAL_SAMPLES-1)
                        state <= ST_IDLE;
                    else begin
                        play_idx <= play_idx + 1'b1;
                        play_addr <= play_addr + 1'b1;
                        state <= ST_READ;
                    end
                end
            end

            default: begin
                state <= ST_IDLE;
            end
        endcase
    end
end

endmodule
