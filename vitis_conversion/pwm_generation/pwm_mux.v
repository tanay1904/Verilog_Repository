`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 29.09.2026 10:49:38
// Design Name: 
// Module Name: pwm_mux
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


module quad_pwm_mux #(
    parameter integer WIDTH = 16   // Default 16-bit PWM counter width
)(
    // Mode Select: 1 = Manual/Calibration (GPIO), 0 = Flight (quad_thrust)
    input  wire             sel_manual,
    // Autonomous Flight Controller Inputs (from quad_thrust_to_pwm_0)
    input  wire [WIDTH-1:0] auto_pwm1,
    input  wire [WIDTH-1:0] auto_pwm2,
    input  wire [WIDTH-1:0] auto_pwm3,
    input  wire [WIDTH-1:0] auto_pwm4,
    // Manual / Bench Calibration Inputs (from pwm1_gpio_17 .. pwm4_gpio_20)
    input  wire [WIDTH-1:0] man_pwm1,
    input  wire [WIDTH-1:0] man_pwm2,
    input  wire [WIDTH-1:0] man_pwm3,
    input  wire [WIDTH-1:0] man_pwm4,
    // Multiplexed Outputs (to PWM_channels_v1_0 duty_int_1 .. duty_int_4)
    output wire [WIDTH-1:0] duty_out1,
    output wire [WIDTH-1:0] duty_out2,
    output wire [WIDTH-1:0] duty_out3,
    output wire [WIDTH-1:0] duty_out4
);
    // ------------------------------------------------------------------------
    // Channel 1 MUX Instance
    // ------------------------------------------------------------------------
    mux2_param #(
        .WIDTH(WIDTH)
    ) u_mux_ch1 (
        .sel (sel_manual),
        .in0 (auto_pwm1),
        .in1 (man_pwm1),
        .out (duty_out1)
    );
    // ------------------------------------------------------------------------
    // Channel 2 MUX Instance
    // ------------------------------------------------------------------------
    mux2_param #(
        .WIDTH(WIDTH)
    ) u_mux_ch2 (
        .sel (sel_manual),
        .in0 (auto_pwm2),
        .in1 (man_pwm2),
        .out (duty_out2)
    );
    // ------------------------------------------------------------------------
    // Channel 3 MUX Instance
    // ------------------------------------------------------------------------
    mux2_param #(
        .WIDTH(WIDTH)
    ) u_mux_ch3 (
        .sel (sel_manual),
        .in0 (auto_pwm3),
        .in1 (man_pwm3),
        .out (duty_out3)
    );
    // ------------------------------------------------------------------------
    // Channel 4 MUX Instance
    // ------------------------------------------------------------------------
    mux2_param #(
        .WIDTH(WIDTH)
    ) u_mux_ch4 (
        .sel (sel_manual),
        .in0 (auto_pwm4),
        .in1 (man_pwm4),
        .out (duty_out4)
    );
endmodule