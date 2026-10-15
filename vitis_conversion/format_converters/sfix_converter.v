`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.07.2026 17:22:11
// Design Name: 
// Module Name: sfix_converter
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

module sfix_converter #(
    parameter WIDTH  = 16,
    parameter EN_IN  = 10,
    parameter EN_OUT = 14
)(
    input  signed [WIDTH-1:0] din,
    output signed [WIDTH-1:0] dout
);

    generate
    if (EN_OUT >= EN_IN) begin
        assign dout = din <<< (EN_OUT - EN_IN);
    end else begin
        assign dout = din >>> (EN_IN - EN_OUT);
    end
    endgenerate

endmodule
