`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06.06.2026 23:10:38
// Design Name: 
// Module Name: ce_generator
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


module ce_12k5_from_10mhz (
    input  wire clk,
    input  wire reset,
    output reg  ce_12k5
);

reg [9:0] count;  // enough for 0 to 799

always @(posedge clk or posedge reset) begin
    if (reset) begin
        count   <= 10'd0; 
        ce_12k5 <= 1'b0;
    end else begin
        if (count == 10'd799) begin
            count   <= 10'd0;
            ce_12k5 <= 1'b1;
        end else begin
            count   <= count + 1'b1;
            ce_12k5 <= 1'b0;
        end
    end
end

endmodule