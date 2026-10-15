`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 25.09.2026 18:00:45
// Design Name: 
// Module Name: pwm_generator
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


`timescale 1ns / 1ps

module quad_thrust_to_pwm (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               valid_in,

    // Inputs: Motor thrust values (sfix30En16)
    input  wire signed [29:0] thrust_m0,
    input  wire signed [29:0] thrust_m1,
    input  wire signed [29:0] thrust_m2,
    input  wire signed [29:0] thrust_m3,

    // Outputs: Integer pulse widths in microseconds (1000 to 2000)
    output wire               valid_out,
    output wire        [15:0] pwm_m0,
    output wire        [15:0] pwm_m1,
    output wire        [15:0] pwm_m2,
    output wire        [15:0] pwm_m3
);

    wire [3:0] vld_channel;
    assign valid_out = vld_channel[0]; // Symmetrical latency across all channels

    thrust_to_pwm_channel u_ch0 (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (valid_in),
        .thrust_in (thrust_m0),
        .valid_out (vld_channel[0]),
        .pwm_out   (pwm_m0)
    );

    thrust_to_pwm_channel u_ch1 (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (valid_in),
        .thrust_in (thrust_m1),
        .valid_out (vld_channel[1]),
        .pwm_out   (pwm_m1)
    );

    thrust_to_pwm_channel u_ch2 (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (valid_in),
        .thrust_in (thrust_m2),
        .valid_out (vld_channel[2]),
        .pwm_out   (pwm_m2)
    );

    thrust_to_pwm_channel u_ch3 (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (valid_in),
        .thrust_in (thrust_m3),
        .valid_out (vld_channel[3]),
        .pwm_out   (pwm_m3)
    );

endmodule
