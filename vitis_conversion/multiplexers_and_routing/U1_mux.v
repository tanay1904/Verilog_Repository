`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 29.09.2026 10:56:24
// Design Name: 
// Module Name: U1_mux
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


module U1_mux #(
    WIDTH = 24
)
(
    // Mode Select: 1 = Manual/Calibration (GPIO), 0 = Flight (quad_thrust)
    input  wire             sel_manual,
    // Autonomous Flight Controller Inputs (from quad_thrust_to_pwm_0)
    input  wire [WIDTH-1:0] auto_U1,
    // Manual / Bench Calibration Inputs (from pwm1_gpio_17 .. pwm4_gpio_20)
    input  wire [WIDTH-1:0] man_U1,
    // Multiplexed Outputs (to PWM_channels_v1_0 duty_int_1 .. duty_int_4)
    output wire [WIDTH-1:0] U1_out
    );
    mux2_param #(
        .WIDTH(WIDTH)
    ) u_mux_ch1 (
        .sel (sel_manual),
        .in0 (auto_U1),
        .in1 (man_U1),
        .out (U1_out)
    );    
endmodule
