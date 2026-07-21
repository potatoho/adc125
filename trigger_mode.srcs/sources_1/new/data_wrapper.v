module adc_dma_wrapper (
    input             rst,

    input             adc1_clk,
    input      [11:0] adc1_data_a,
    input      [11:0] adc1_data_b,

    input             adc2_clk,
    input      [11:0] adc2_data_a,
    input      [11:0] adc2_data_b,

    input             trigger_in,

    // AXI4-Lite configuration
    input      [31:0] control,
    input      [31:0] frame_size,
    input      [31:0] pretrigger_size,
    input      [31:0] sample_decimation,
    input      [31:0] trigger_cfg,
    input      [31:0] self_threshold,
    input      [31:0] channel_mask,
    input      [31:0] output_cfg,
    input      [31:0] baseline_shift,

    input             m_axis_aclk,
    input             m_axis_aresetn,

    output     [63:0] m_axis_tdata,
    output            m_axis_tvalid,
    input             m_axis_tready,
    output     [7:0]  m_axis_tkeep,
    output            m_axis_tlast
);

wire [63:0] pack_tdata;
wire        pack_tvalid;
wire        pack_tready;
wire [7:0]  pack_tkeep;
wire        pack_tlast;


// ============================================================
// ADC data packer
// ============================================================

adc_axis64_packer #(
    .FIFO_DEPTH       (4096),
    .WORDS_PER_PACKET (131072)
) adc_axis64_packer_inst (
    .rst             (rst),

    .adc1_clk        (adc1_clk),
    .adc1_data_a     (adc1_data_a),
    .adc1_data_b     (adc1_data_b),

    .adc2_clk        (adc2_clk),
    .adc2_data_a     (adc2_data_a),
    .adc2_data_b     (adc2_data_b),

    .m_axis_aclk     (m_axis_aclk),
    .m_axis_aresetn  (m_axis_aresetn),

    .m_axis_tdata    (pack_tdata),
    .m_axis_tvalid   (pack_tvalid),
    .m_axis_tready   (pack_tready),
    .m_axis_tkeep    (pack_tkeep),
    .m_axis_tlast    (pack_tlast)
);


// ============================================================
// Trigger capture buffer
// ============================================================

axis64_trigger_buffer #(
    .DEPTH (262144)
) trigger_buffer_inst (
    .clk                   (m_axis_aclk),
    .resetn                (m_axis_aresetn),

    .s_axis_tdata          (pack_tdata),
    .s_axis_tvalid         (pack_tvalid),
    .s_axis_tready         (pack_tready),

    .trig_in               (trigger_in),

    .cfg_control           (control),
    .cfg_frame_size        (frame_size),
    .cfg_pretrigger_size   (pretrigger_size),
    .cfg_sample_decimation (sample_decimation),
    .cfg_trigger_cfg       (trigger_cfg),
    .cfg_self_threshold    (self_threshold),
    .cfg_channel_mask      (channel_mask),
    .cfg_output_cfg        (output_cfg),
    .cfg_baseline_shift    (baseline_shift),

    .m_axis_tdata          (m_axis_tdata),
    .m_axis_tvalid         (m_axis_tvalid),
    .m_axis_tready         (m_axis_tready),
    .m_axis_tkeep          (m_axis_tkeep),
    .m_axis_tlast          (m_axis_tlast)
);

endmodule
