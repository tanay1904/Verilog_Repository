`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03.09.2026 23:48:11
// Design Name: 
// Module Name: rc_input_decoder_top
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
//////////////////////////////////////////////////////////////////////////////////
// Top-level wrapper for RC Input Decoder on Basys-3 board
// - Connects 4 RC PWM inputs to rc_input_decoder
// - Exposes failsafe on LED0 for visual indication
// - Optionally exposes decoded channels to PMOD/logic analyzer pins
//////////////////////////////////////////////////////////////////////////////////

module rc_input_decoder_top (
    input  wire clk,           // 100 MHz clock input from board (W5 on Basys-3)
    input  wire btn_reset,     // Pushbutton reset (e.g., BTNU), active high

    input  wire pwm_roll,      // Roll channel PWM input from RC receiver
    input  wire pwm_pitch,     // Pitch channel PWM input
    input  wire pwm_throttle,  // Throttle channel PWM input
    input  wire pwm_yaw,       // Yaw channel PWM input

    output wire led0,          // LED for failsafe indicator (D1 on Basys-3)

    // Optional debug outputs (can be pinned to PMOD or LEDs in XDC)
    output wire signed [11:0] dbg_roll,
    output wire signed [11:0] dbg_pitch,
    output wire signed [11:0] dbg_throttle,
    output wire signed [11:0] dbg_yaw
);

    // Internal wires for decoded channels and failsafe
    wire signed [11:0] roll;
    wire signed [11:0] pitch;
    wire signed [11:0] throttle;
    wire signed [11:0] yaw;
    wire failsafe;

    // Simple reset: directly use pushbutton (you can add debouncing if needed)
    wire rst = btn_reset;

    // Instantiate the updated RC input decoder
    rc_input_decoder rc_decoder_inst (
        .clk(clk),
        .rst(rst),
        .pwm_roll(pwm_roll),
        .pwm_pitch(pwm_pitch),
        .pwm_throttle(pwm_throttle),
        .pwm_yaw(pwm_yaw),
        .roll(roll),
        .pitch(pitch),
        .throttle(throttle),
        .yaw(yaw),
        .failsafe(failsafe)
    );

    // Failsafe LED: ON when failsafe is active (any channel invalid / timed out)
    assign led0 = failsafe;

    // Expose decoded outputs for debugging / LA / future use by flight controller
    assign dbg_roll     = roll;
    assign dbg_pitch    = pitch;
    assign dbg_throttle = throttle;
    assign dbg_yaw      = yaw;

endmodule
