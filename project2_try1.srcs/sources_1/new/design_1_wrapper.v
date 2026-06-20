//Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2022.2 (win64) Build 3671981 Fri Oct 14 05:00:03 MDT 2022
//Date        : Sun Jun 14 15:16:43 2026
//Host        : sunset running 64-bit major release  (build 9200)
//Command     : generate_target design_1_wrapper.bd
//Design      : design_1_wrapper
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module design_1_wrapper
   (S_AXIS_ADC_tdata,
    S_AXIS_ADC_tlast,
    S_AXIS_ADC_tready,
    S_AXIS_ADC_tvalid,
    axis_aresetn,
    pl_clk0);
  input [63:0]S_AXIS_ADC_tdata;
  input S_AXIS_ADC_tlast;
  output S_AXIS_ADC_tready;
  input S_AXIS_ADC_tvalid;
  output axis_aresetn;
  output pl_clk0;

  wire [63:0]S_AXIS_ADC_tdata;
  wire S_AXIS_ADC_tlast;
  wire S_AXIS_ADC_tready;
  wire S_AXIS_ADC_tvalid;
  wire axis_aresetn;
  wire pl_clk0;

  design_1 design_1_i
       (.S_AXIS_ADC_tdata(S_AXIS_ADC_tdata),
        .S_AXIS_ADC_tlast(S_AXIS_ADC_tlast),
        .S_AXIS_ADC_tready(S_AXIS_ADC_tready),
        .S_AXIS_ADC_tvalid(S_AXIS_ADC_tvalid),
        .axis_aresetn(axis_aresetn),
        .pl_clk0(pl_clk0));
endmodule
