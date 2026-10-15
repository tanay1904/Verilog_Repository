module theta_sfix32En29_to_sfix24En16 (
    input  wire signed [31:0] theta_sfix32_en29,   // sfix32_En29
    output wire signed [23:0] angles_1_sfix24_en16 // sfix24_En16
);

    /*
        Real value of input  = theta_sfix32_en29 / 2^29
        Real value of output = angles_1_sfix24_en16 / 2^16

        Therefore:

        angles_1_sfix24_en16 = theta_sfix32_en29 / 2^(29-16)
                              = theta_sfix32_en29 / 2^13

        So we arithmetic-right-shift by 13 bits.
    */

    assign angles_1_sfix24_en16 = theta_sfix32_en29 >>> 13;

endmodule