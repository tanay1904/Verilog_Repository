`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 29.09.2026 10:54:56
// Design Name: 
// Module Name: mux_2_input
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


module mux2_param #(
    parameter integer WIDTH = 16
)(
    input  wire             sel,  // 0 = in0 (Auto), 1 = in1 (Manual)
    input  wire [WIDTH-1:0] in0,  // Input 0
    input  wire [WIDTH-1:0] in1,  // Input 1
    output wire [WIDTH-1:0] out   // Selected Output
);
    assign out = sel ? in1 : in0;
endmodule
