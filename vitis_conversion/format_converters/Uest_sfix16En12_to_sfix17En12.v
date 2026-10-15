module Uest_sfix16En12_to_sfix17En12 (
    input  wire signed [15:0] Uest_sfix16_en12,  // sfix16_En12
    output wire signed [16:0] Uest_sfix17_en12   // sfix17_En12
);

    /*
        Same fractional length: En12 -> En12
        Therefore no scaling is needed.

        Only bit width changes:
        signed 16-bit -> signed 17-bit

        So copy the sign bit once on the left.
    */

    assign Uest_sfix17_en12 = {Uest_sfix16_en12[15], Uest_sfix16_en12};

endmodule