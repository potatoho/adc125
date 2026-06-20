module adc_axis64_packer #(
    parameter FIFO_DEPTH = 131072,
    parameter WORDS_PER_PACKET = 8192
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