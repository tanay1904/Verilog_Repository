module u2_to_mixer_format (
    input  wire signed [23:0] u2_sfix24_en16,
    output wire signed [35:0] u2_sfix36_en30
);

    // Convert sfix24_En16 to sfix36_En30
    //
    // Difference in fraction length:
    // En30 - En16 = 14
    //
    // Therefore:
    // raw_out = raw_in << 14

    assign u2_sfix36_en30 = $signed({{12{u2_sfix24_en16[23]}}, u2_sfix24_en16}) <<< 14;

endmodule