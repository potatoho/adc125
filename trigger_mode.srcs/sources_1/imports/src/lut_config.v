module lut_config(
    input  [9:0]       lut_index,
    output reg [24:0]  lut_data
);

always @(*) begin
    case (lut_index)

        10'd0: lut_data <= {16'h0000, 8'h18};

        // 0x08: power mode - normal operation
        10'd1: lut_data <= {16'h0008, 8'h00};

        // 0x09: global clock - divide by 1
        10'd2: lut_data <= {16'h0009, 8'h00};

        // 0x0D: test mode off
        10'd3: lut_data <= {16'h000D, 8'h00};

        // 0x14: LVDS output, offset binary
        10'd4: lut_data <= {16'h0014, 8'h40};

        // 0x15: output adjust default
        10'd5: lut_data <= {16'h0015, 8'h00};

        // 0x16: DCO phase adjust 없음
        10'd6: lut_data <= {16'h0016, 8'h00};

        // 0x17: output delay 없음
        10'd7: lut_data <= {16'h0017, 8'h00};

        // 0x18: VREF Select
        // 0xC0 = 2.0 Vp-p full-scale, power-up default
        10'd8: lut_data <= {16'h0018, 8'hC0};

        // 0xFF: transfer / shadow register update
        10'd9: lut_data <= {16'h00FF, 8'h01};

        default: lut_data <= {16'hFFFF, 8'hFF};

    endcase
end

endmodule