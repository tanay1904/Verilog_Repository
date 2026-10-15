`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 20.09.2026 23:25:09
// Design Name: 
// Module Name: rc_cmd_mux
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
`default_nettype none

module rc_input_mux (
    input  wire sel, // 0 = HIL (Physical Pins), 1 = PIL (Software from C)

    // Hardware inputs from physical board pins (HIL)
    input  wire hw_roll,
    input  wire hw_pitch,
    input  wire hw_throttle,
    input  wire hw_yaw,

    // Software bit-bang inputs from AXI GPIO (PIL)
    input  wire sw_roll,
    input  wire sw_pitch,
    input  wire sw_throttle,
    input  wire sw_yaw,

    // Outputs feeding directly into rc_input_decoder
    output wire pwm_roll_out,
    output wire pwm_pitch_out,
    output wire pwm_throttle_out,
    output wire pwm_yaw_out
);

    assign pwm_roll_out     = sel ? sw_roll     : hw_roll;
    assign pwm_pitch_out    = sel ? sw_pitch    : hw_pitch;
    assign pwm_throttle_out = sel ? sw_throttle : hw_throttle;
    assign pwm_yaw_out      = sel ? sw_yaw      : hw_yaw;

endmodule
`default_nettype wire