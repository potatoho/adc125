module adc_dma_wrapper (
    input             rst,

    input             adc1_clk,
    input      [11:0] adc1_data_a,
    input      [11:0] adc1_data_b,

    input             adc2_clk,
    input      [11:0] adc2_data_a,
    input      [11:0] adc2_data_b,

    input             m_axis_aclk,
    input             m_axis_aresetn,

    output [63:0]     m_axis_tdata,
    output            m_axis_tvalid,
    input             m_axis_tready,
    output [7:0]      m_axis_tkeep,
    output            m_axis_tlast
);

adc_axis64_packer adc_axis64_packer_inst (
    .rst            (rst),

    .adc1_clk       (adc1_clk),
    .adc1_data_a    (adc1_data_a),
    .adc1_data_b    (adc1_data_b),

    .adc2_clk       (adc2_clk),
    .adc2_data_a    (adc2_data_a),
    .adc2_data_b    (adc2_data_b),

    .m_axis_aclk    (m_axis_aclk),
    .m_axis_aresetn (m_axis_aresetn),

    .m_axis_tdata   (m_axis_tdata),
    .m_axis_tvalid  (m_axis_tvalid),
    .m_axis_tready  (m_axis_tready),
    .m_axis_tkeep   (m_axis_tkeep),
    .m_axis_tlast   (m_axis_tlast)
);

endmodule