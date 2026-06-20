module top(
    input                     sys_clk_p,
    input                     sys_clk_n,
    input                     rst_n,

    output                    adc1_clk_ref,
    output                    adc2_clk_ref,

    output                    adc1_spi_ce,
    output                    adc1_spi_sclk,
    inout                     adc1_spi_io,
    input                     adc1_clk_p,
    input                     adc1_clk_n,
    input  [11:0]             adc1_data_p,
    input  [11:0]             adc1_data_n,

    output                    adc2_spi_ce,
    output                    adc2_spi_sclk,
    inout                     adc2_spi_io,
    input                     adc2_clk_p,
    input                     adc2_clk_n,
    input  [11:0]             adc2_data_p,
    input  [11:0]             adc2_data_n
    
);

wire clk_50m;
wire clk_125m;
wire locked;

wire [9:0]  adc1_lut_index;
wire [24:0] adc1_lut_data;
wire [9:0]  adc2_lut_index;
wire [24:0] adc2_lut_data;

wire        adc1_clk;
wire        adc2_clk;

wire [11:0] adc1_data;
wire [11:0] adc2_data;

(* mark_debug = "true" *) wire [11:0] adc1_data_a;
(* mark_debug = "true" *) wire [11:0] adc1_data_b;
(* mark_debug = "true" *) wire [11:0] adc2_data_a;
(* mark_debug = "true" *) wire [11:0] adc2_data_b;

(* MARK_DEBUG="true" *) reg [11:0] adc1_data_a_d0;
(* MARK_DEBUG="true" *) reg [11:0] adc1_data_b_d0;
(* MARK_DEBUG="true" *) reg [11:0] adc2_data_a_d0;
(* MARK_DEBUG="true" *) reg [11:0] adc2_data_b_d0;

wire [63:0] m_axis_tdata;
wire        m_axis_tvalid;
wire        m_axis_tready;
wire [7:0]  m_axis_tkeep;
wire        m_axis_tlast;

wire        axis_aclk;
wire        axis_aresetn;

assign adc1_clk_ref = clk_125m;
assign adc2_clk_ref = clk_125m;

IBUFDS #(
    .DIFF_TERM("TRUE"),
    .IBUF_LOW_PWR("TRUE"),
    .IOSTANDARD("LVDS_25")
) IBUFDS_adc1_clk (
    .O  (adc1_clk),
    .I  (adc1_clk_p),
    .IB (adc1_clk_n)
);

IBUFDS #(
    .DIFF_TERM("TRUE"),
    .IBUF_LOW_PWR("TRUE"),
    .IOSTANDARD("LVDS_25")
) IBUFDS_adc2_clk (
    .O  (adc2_clk),
    .I  (adc2_clk_p),
    .IB (adc2_clk_n)
);

genvar i;
generate
    for (i = 0; i < 12; i = i + 1) begin : IBUFDS_DATAS

        IBUFDS #(
            .DIFF_TERM("TRUE"),
            .IBUF_LOW_PWR("TRUE"),
            .IOSTANDARD("LVDS_25")
        ) IBUFDS_adc1_data (
            .O  (adc1_data[i]),
            .I  (adc1_data_p[i]),
            .IB (adc1_data_n[i])
        );

        IDDR #(
            .DDR_CLK_EDGE("OPPOSITE_EDGE"),
            .INIT_Q1(1'b0),
            .INIT_Q2(1'b0),
            .SRTYPE("ASYNC")
        ) IDDR_adc1_data (
            .Q1 (adc1_data_b[i]),
            .Q2 (adc1_data_a[i]),
            .C  (adc1_clk),
            .CE (1'b1),
            .D  (adc1_data[i]),
            .R  (1'b0),
            .S  (1'b0)
        );

        IBUFDS #(
            .DIFF_TERM("TRUE"),
            .IBUF_LOW_PWR("TRUE"),
            .IOSTANDARD("LVDS_25")
        ) IBUFDS_adc2_data (
            .O  (adc2_data[i]),
            .I  (adc2_data_p[i]),
            .IB (adc2_data_n[i])
        );

        IDDR #(
            .DDR_CLK_EDGE("OPPOSITE_EDGE"),
            .INIT_Q1(1'b0),
            .INIT_Q2(1'b0),
            .SRTYPE("ASYNC")
        ) IDDR_adc2_data (
            .Q1 (adc2_data_b[i]),
            .Q2 (adc2_data_a[i]),
            .C  (adc2_clk),
            .CE (1'b1),
            .D  (adc2_data[i]),
            .R  (1'b0),
            .S  (1'b0)
        );

    end
endgenerate

always @(posedge adc1_clk) begin
    adc1_data_a_d0 <= adc1_data_a;
    adc1_data_b_d0 <= adc1_data_b;
end

always @(posedge adc2_clk) begin
    adc2_data_a_d0 <= adc2_data_a;
    adc2_data_b_d0 <= adc2_data_b;
end

clk_wiz_0 sys_pll_m0 (
    .clk_in1_p (sys_clk_p),
    .clk_in1_n (sys_clk_n),
    .clk_out1  (clk_50m),
    .clk_out2  (clk_125m),
    .locked    (locked)
);

lut_config lut_config_adc1 (
    .lut_index (adc1_lut_index),
    .lut_data  (adc1_lut_data)
);

spi_config spi_config_adc1 (
    .rst          (~locked),
    .clk          (clk_50m),
    .clk_div_cnt  (16'd500),
    .lut_index    (adc1_lut_index),
    .lut_reg_addr (adc1_lut_data[23:8]),
    .lut_reg_data (adc1_lut_data[7:0]),
    .error        (),
    .done         (),
    .spi_ce       (adc1_spi_ce),
    .spi_sclk     (adc1_spi_sclk),
    .spi_io       (adc1_spi_io)
);

lut_config lut_config_adc2 (
    .lut_index (adc2_lut_index),
    .lut_data  (adc2_lut_data)
);

spi_config spi_config_adc2 (
    .rst          (~locked),
    .clk          (clk_50m),
    .clk_div_cnt  (16'd500),
    .lut_index    (adc2_lut_index),
    .lut_reg_addr (adc2_lut_data[23:8]),
    .lut_reg_data (adc2_lut_data[7:0]),
    .error        (),
    .done         (),
    .spi_ce       (adc2_spi_ce),
    .spi_sclk     (adc2_spi_sclk),
    .spi_io       (adc2_spi_io)
);

adc_dma_wrapper adc_dma_wrapper_inst (
    .rst            (~axis_aresetn),

    .adc1_clk       (adc1_clk),
    .adc1_data_a    (adc1_data_a_d0),
    .adc1_data_b    (adc1_data_b_d0),

    .adc2_clk       (adc2_clk),
    .adc2_data_a    (adc2_data_a_d0),
    .adc2_data_b    (adc2_data_b_d0),

    .m_axis_aclk    (axis_aclk),
    .m_axis_aresetn (axis_aresetn),

    .m_axis_tdata   (m_axis_tdata),
    .m_axis_tvalid  (m_axis_tvalid),
    .m_axis_tready  (m_axis_tready),
    .m_axis_tkeep   (m_axis_tkeep),
    .m_axis_tlast   (m_axis_tlast)
);

design_1_wrapper design_1_i (
    .S_AXIS_ADC_tdata   (m_axis_tdata),
    .S_AXIS_ADC_tlast   (m_axis_tlast),
    .S_AXIS_ADC_tready  (m_axis_tready),
    .S_AXIS_ADC_tvalid  (m_axis_tvalid),

    .axis_aresetn       (axis_aresetn),
    .pl_clk0            (axis_aclk)
);

endmodule