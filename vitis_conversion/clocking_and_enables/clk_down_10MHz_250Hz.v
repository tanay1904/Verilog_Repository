`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08.07.2026 18:47:47
// Design Name: 
// Module Name: clk_down_10MHz_250Hz
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


module pulse_gen
#(
    parameter CLK_FREQ = 10_000_000,
    parameter PULSE_FREQ = 250
)
(
    input  wire clk,
    input  wire rstn,
    output reg  pulse_out_250Hz
);

localparam integer COUNT_MAX = CLK_FREQ / PULSE_FREQ - 1;
reg [15:0] counter;

always @(posedge clk) begin
    if (!rstn) begin
        counter <= 0;
        pulse_out_250Hz <= 1'b0;
    end
    else if (counter == COUNT_MAX) begin
        counter <= 0;
        pulse_out_250Hz <= 1'b1;
    end
    else begin
        counter <= counter + 1;
        pulse_out_250Hz <= 1'b0;
    end
end

endmodule