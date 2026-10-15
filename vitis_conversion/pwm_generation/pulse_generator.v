`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 28.07.2026 12:55:35
// Design Name: 
// Module Name: pulse_generator
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


module pulse_generator(
    input wire clk,
    input wire reset,
    input wire clk_enable,
    output reg  clk_enable_250hz
);

localparam integer CLK_FREQ_HZ = 10_000_000;
localparam integer CE_FREQ_HZ  = 250;
localparam integer CE_DIVIDE   = CLK_FREQ_HZ / CE_FREQ_HZ;

localparam integer PULSE_WIDTH = 1;    // High for 1 clocks

reg [15:0] ce_counter;
reg [15:0] pulse_counter;

always @(posedge clk or posedge reset) begin
    if (reset) begin
        ce_counter       <= 0;
        pulse_counter    <= 0;
        clk_enable_250hz <= 1'b0;
    end
    else begin
        if (clk_enable)begin
            // Generate a new pulse every 40000 clocks
            if (ce_counter == CE_DIVIDE-1) begin
                ce_counter    <= 0;
                pulse_counter <= PULSE_WIDTH;
            end
            else begin
                ce_counter <= ce_counter + 1'b1;
            end
    
            // Stretch the pulse
            if (pulse_counter != 0) begin
                pulse_counter    <= pulse_counter - 1'b1;
                clk_enable_250hz <= 1'b1;
            end
            else begin
                clk_enable_250hz <= 1'b0;
            end
        end
    end
end

endmodule