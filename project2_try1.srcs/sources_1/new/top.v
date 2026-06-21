module top(
    input                     sys_clk_p,
    input                     sys_clk_n,

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
wire clk_200m;
wire locked;
wire idelayctrl_rdy;

reg [3:0] idelay_rst_sync = 4'b1111;
wire idelayctrl_rst = idelay_rst_sync[3];

wire [9:0]  adc1_lut_index;
wire [24:0] adc1_lut_data;
wire [9:0]  adc2_lut_index;
wire [24:0] adc2_lut_data;

wire adc1_clk_ibuf;
wire adc1_clk_io;
wire adc1_clk_fabric;

wire adc2_clk_ibuf;
wire adc2_clk_io;
wire adc2_clk_fabric;

wire [11:0] adc1_data_ibuf;
wire [11:0] adc1_data_delay;
wire [11:0] adc2_data_ibuf;

(* mark_debug = "true" *) wire [11:0] adc1_data_a;
(* mark_debug = "true" *) wire [11:0] adc1_data_b;
(* mark_debug = "true" *) wire [11:0] adc2_data_a;
(* mark_debug = "true" *) wire [11:0] adc2_data_b;

(* mark_debug = "true" *) reg [11:0] adc1_data_a_d0;
(* mark_debug = "true" *) reg [11:0] adc1_data_b_d0;
(* mark_debug = "true" *) reg [11:0] adc2_data_a_d0;
(* mark_debug = "true" *) reg [11:0] adc2_data_b_d0;

wire [63:0] m_axis_tdata;
wire        m_axis_tvalid;
wire        m_axis_tready;
wire [7:0]  m_axis_tkeep;
wire        m_axis_tlast;

wire        axis_aclk;
wire        axis_aresetn;

assign adc1_clk_ref = clk_125m;
assign adc2_clk_ref = clk_125m;


// ============================================================
// System PLL
// ============================================================

clk_wiz_0 sys_pll_m0 (
    .clk_in1_p (sys_clk_p),
    .clk_in1_n (sys_clk_n),
    .clk_out1  (clk_50m),
    .clk_out2  (clk_125m),
    .clk_out3  (clk_200m), // IDELAYCTRL 및 IDELAYE3 제어용 200MHz
    .locked    (locked)
);


// ============================================================
// IDELAYCTRL reset sync to clk_200m
// ============================================================

always @(posedge clk_200m or negedge locked) begin
    if (!locked)
        idelay_rst_sync <= 4'b1111;
    else
        idelay_rst_sync <= {idelay_rst_sync[2:0], 1'b0};
end


// ============================================================
// IDELAYCTRL
// ============================================================

IDELAYCTRL #(
    .SIM_DEVICE("ULTRASCALE")
) IDELAYCTRL_inst (
    .RDY    (idelayctrl_rdy),
    .REFCLK (clk_200m),
    .RST    (idelayctrl_rst)
);


// ============================================================
// ADC1 DCO input clock (BUFIO와 BUFG 병렬 유지)
// ============================================================

IBUFDS #(
    .DIFF_TERM("TRUE"),
    .IBUF_LOW_PWR("FALSE"),
    .IOSTANDARD("LVDS")
) IBUFDS_adc1_clk (
    .O  (adc1_clk_ibuf),
    .I  (adc1_clk_p),
    .IB (adc1_clk_n)
);

BUFIO BUFIO_adc1_clk (
    .I (adc1_clk_ibuf),
    .O (adc1_clk_io)      // IDDRE1.C 수신용 고속 로컬 클럭
);

BUFG BUFG_adc1_clk (
    .I (adc1_clk_ibuf),
    .O (adc1_clk_fabric)  // fabric 레지스터 동기화용 글로벌 클럭
);


// ============================================================
// ADC2 DCO input clock (BUFIO와 BUFG 병렬 유지)
// ============================================================

IBUFDS #(
    .DIFF_TERM("TRUE"),
    .IBUF_LOW_PWR("FALSE"),
    .IOSTANDARD("LVDS")
) IBUFDS_adc2_clk (
    .O  (adc2_clk_ibuf),
    .I  (adc2_clk_p),
    .IB (adc2_clk_n)
);

BUFIO BUFIO_adc2_clk (
    .I (adc2_clk_ibuf),
    .O (adc2_clk_io)      // IDDRE1.C 수신용 고속 로컬 클럭
);

BUFG BUFG_adc2_clk (
    .I (adc2_clk_ibuf),
    .O (adc2_clk_fabric)  // fabric 레지스터 동기화용 글로벌 클럭
);


// ============================================================
// ADC LVDS data input + IDELAYE3 정정 + IDDRE1
// ============================================================

genvar i;
generate
for (i = 0; i < 12; i = i + 1) begin : GEN_ADC_INPUTS

    // --------------------------------------------------------
    // ADC1 채널: IBUFDS -> IDELAYE3(IDATAIN) -> IDDRE1.D
    // --------------------------------------------------------
    IBUFDS #(
        .DIFF_TERM("TRUE"),
        .IBUF_LOW_PWR("FALSE"),
        .IOSTANDARD("LVDS")
    ) IBUFDS_adc1_data (
        .O  (adc1_data_ibuf[i]),
        .I  (adc1_data_p[i]),
        .IB (adc1_data_n[i])
    );

    IDELAYE3 #(
        .CASCADE("NONE"),
        .DELAY_FORMAT("TIME"),
        .DELAY_SRC("IDATAIN"),          // 정정: 외부 입력이므로 IDATAIN 설정
        .DELAY_TYPE("FIXED"),
        .DELAY_VALUE(300),              // 정정: 변별력 있는 비교를 위해 300ps 인위적 추가
        .IS_CLK_INVERTED(1'b0),
        .IS_RST_INVERTED(1'b0),
        .REFCLK_FREQUENCY(200.0),
        .SIM_DEVICE("ULTRASCALE_PLUS"),
        .UPDATE_MODE("ASYNC")
    ) IDELAYE3_adc1_data (
        .CASC_IN      (1'b0),
        .CASC_OUT     (),
        .CASC_RETURN  (1'b0),
        .CE           (1'b0),
        .CLK          (clk_200m),       // 정정: 안정을 위해 제어 클럭도 200MHz로 통일
        .CNTVALUEOUT  (),
        .CNTVALUEIN   (9'd0),
        .DATAIN       (1'b0),           // 정정: DATAIN은 0 바인딩
        .EN_VTC(1'b1),
        .IDATAIN      (adc1_data_ibuf[i]), // 정정: 외부 입력 신호는 여기로 연결
        .INC          (1'b0),
        .LOAD         (1'b0),
        .RST          (idelayctrl_rst),
        .DATAOUT      (adc1_data_delay[i])
    );

    IDDRE1 #(
        .DDR_CLK_EDGE("SAME_EDGE_PIPELINED"),
        .IS_CB_INVERTED(1'b1),
        .IS_C_INVERTED(1'b0)
    ) IDDRE1_adc1_data (
        .Q1 (adc1_data_a[i]),
        .Q2 (adc1_data_b[i]),
        .C  (adc1_clk_io),              // 정정: BUFIO에서 나온 고속 클럭 연결
        .CB (adc1_clk_io),
        .D  (adc1_data_delay[i]),
        .R  (1'b0)
    );

    // --------------------------------------------------------
    // ADC2 채널: IBUFDS -> IDDRE1.D (기존 안정 구조 유지)
    // --------------------------------------------------------
    IBUFDS #(
        .DIFF_TERM("TRUE"),
        .IBUF_LOW_PWR("FALSE"),
        .IOSTANDARD("LVDS")
    ) IBUFDS_adc2_data (
        .O  (adc2_data_ibuf[i]),
        .I  (adc2_data_p[i]),
        .IB (adc2_data_n[i])
    );

    IDDRE1 #(
        .DDR_CLK_EDGE("SAME_EDGE_PIPELINED"),
        .IS_CB_INVERTED(1'b1),
        .IS_C_INVERTED(1'b0)
    ) IDDRE1_adc2_data (
        .Q1 (adc2_data_b[i]),
        .Q2 (adc2_data_a[i]),
        .C  (adc2_clk_io),              // 정정: BUFIO에서 나온 고속 클럭 연결
        .CB (adc2_clk_io),
        .D  (adc2_data_ibuf[i]),
        .R  (1'b0)
    );

end
endgenerate


// ============================================================
// ADC fabric-domain registers (BUFG 클럭 도메인 수신)
// ============================================================

always @(posedge adc1_clk_fabric) begin
    adc1_data_a_d0 <= adc1_data_a;
    adc1_data_b_d0 <= adc1_data_b;
end

always @(posedge adc2_clk_fabric) begin
    adc2_data_a_d0 <= adc2_data_a;
    adc2_data_b_d0 <= adc2_data_b;
end


// ============================================================
// SPI config
// ============================================================

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


// ============================================================
// ADC DMA
// ============================================================

adc_dma_wrapper adc_dma_wrapper_inst (
    .rst            (~axis_aresetn),

    .adc1_clk       (adc1_clk_fabric),
    .adc1_data_a    (adc1_data_a_d0),
    .adc1_data_b    (adc1_data_b_d0),

    .adc2_clk       (adc2_clk_fabric),
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


// ============================================================
// PS / AXI
// ============================================================

design_1_wrapper design_1_i (
    .S_AXIS_ADC_tdata   (m_axis_tdata),
    .S_AXIS_ADC_tlast   (m_axis_tlast),
    .S_AXIS_ADC_tready  (m_axis_tready),
    .S_AXIS_ADC_tvalid  (m_axis_tvalid),

    .axis_aresetn       (axis_aresetn),
    .pl_clk0            (axis_aclk)
);

endmodule
