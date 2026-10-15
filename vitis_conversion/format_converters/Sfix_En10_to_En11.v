`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10.07.2026 16:01:55
// Design Name: 
// Module Name: Sfix_En10_to_En11
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module sfix16_en10_to_en11
(
    input  wire signed [15:0] in,
    output reg  signed [15:0] out
);

always @(*) begin

    // Positive overflow
    if (in > 16'sd16383)
        out = 16'sh7FFF;

    // Negative overflow
    else if (in < -16'sd16384)
        out = 16'sh8000;

    // Safe to convert
    else
        out = in <<< 1;

end

endmodule