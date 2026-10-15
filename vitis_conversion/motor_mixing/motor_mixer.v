`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 20.09.2026 15:04:46
// Design Name: 
// Module Name: motor_mixer
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

//////////////////////////////////////////////////////////////////////////////////
// Module Name: motor_mixer_direct
// Description:
//   Pure hardware motor mixer for X-configuration quadcopters / rovers.
//   Directly connects rc_input_decoder outputs to PWM_channels inputs.
//   Zero CPU latency - autonomous hardware datapath.
//
// Motor Mapping (Standard Quad-X):
//   Motor 1 (Rear-Right)  = Base - Roll + Pitch + Yaw
//   Motor 2 (Front-Right) = Base + Roll - Pitch + Yaw
//   Motor 3 (Rear-Left)   = Base + Roll + Pitch - Yaw
//   Motor 4 (Front-Left)  = Base - Roll - Pitch - Yaw
//////////////////////////////////////////////////////////////////////////////////

module motor_mixer #(
    // Timer scale matching PWM_channels (1.0ms - 2.0ms pulse at 500Hz)
    parameter integer DUTY_MIN        = 3333, // 1.0 ms pulse (Motor Stop / Idle)
    parameter integer DUTY_MAX        = 6667, // 2.0 ms pulse (Full Throttle)
    
    // Scaling authority for stick inputs (+/- duty counts added to baseline)
    parameter integer ROLL_SCALE      = 400,  // Max +/- 400 duty counts at full roll
    parameter integer PITCH_SCALE     = 400,  // Max +/- 400 duty counts at full pitch
    parameter integer YAW_SCALE       = 300,  // Max +/- 300 duty counts at full yaw
    
    // Safety thresholds
    parameter integer THROTTLE_CUTOFF = -900, // Below -900 (out of -1000 to +1000) -> Motors Idle
    parameter integer DEADBAND        = 30    // +/- 30 stick deadband around center (0)
)(
    input  wire clk,
    input  wire rstn, // Active-low reset (matches PWM_channels)

    // 4 Decoded Channels from rc_input_decoder
    input  wire signed [11:0] roll,     // Normalized: -1000 to +1000
    input  wire signed [11:0] pitch,    // Normalized: -1000 to +1000
    input  wire signed [11:0] throttle, // Normalized: -1000 to +1000
    input  wire signed [11:0] yaw,      // Normalized: -1000 to +1000
    input  wire               failsafe, // 1 = Signal Lost / Timeout

    // 4 Duty Outputs directly to PWM_channels (duty_int_1 to duty_int_4)
    output reg [15:0] duty_int_1, // Motor 1: Rear-Right (RR)
    output reg [15:0] duty_int_2, // Motor 2: Front-Right (FR)
    output reg [15:0] duty_int_3, // Motor 3: Rear-Left (RL)
    output reg [15:0] duty_int_4  // Motor 4: Front-Left (FL)
);

    // Internal 32-bit signed math registers to prevent overflow
    reg signed [31:0] base_duty;
    reg signed [31:0] r_effort;
    reg signed [31:0] p_effort;
    reg signed [31:0] y_effort;

    reg signed [31:0] m1_calc;
    reg signed [31:0] m2_calc;
    reg signed [31:0] m3_calc;
    reg signed [31:0] m4_calc;

    // Shift throttle from [-1000, +1000] to [0, 2000]
    wire signed [31:0] throttle_shifted = $signed({{20{throttle[11]}}, throttle}) + 32'sd1000;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            duty_int_1 <= DUTY_MIN[15:0];
            duty_int_2 <= DUTY_MIN[15:0];
            duty_int_3 <= DUTY_MIN[15:0];
            duty_int_4 <= DUTY_MIN[15:0];
        end else begin
            // -------------------------------------------------------------
            // 1. SAFETY INTERLOCK: Failsafe OR Throttle Stick at Minimum
            // -------------------------------------------------------------
            if (failsafe || (throttle <= THROTTLE_CUTOFF)) begin
                duty_int_1 <= DUTY_MIN[15:0];
                duty_int_2 <= DUTY_MIN[15:0];
                duty_int_3 <= DUTY_MIN[15:0];
                duty_int_4 <= DUTY_MIN[15:0];
            end else begin

                // ---------------------------------------------------------
                // 2. BASE THROTTLE: Map [0 .. 2000] -> [DUTY_MIN .. DUTY_MAX]
                // ---------------------------------------------------------
                base_duty = DUTY_MIN + ((throttle_shifted * (DUTY_MAX - DUTY_MIN)) / 32'sd2000);

                // ---------------------------------------------------------
                // 3. ROLL EFFORT (Symmetric with Center Deadband)
                // ---------------------------------------------------------
                if (roll > -DEADBAND && roll < DEADBAND)
                    r_effort = 32'sd0;
                else
                    r_effort = ($signed({{20{roll[11]}}, roll}) * ROLL_SCALE) / 32'sd1000;

                // ---------------------------------------------------------
                // 4. PITCH EFFORT (Symmetric with Center Deadband)
                // ---------------------------------------------------------
                if (pitch > -DEADBAND && pitch < DEADBAND)
                    p_effort = 32'sd0;
                else
                    p_effort = ($signed({{20{pitch[11]}}, pitch}) * PITCH_SCALE) / 32'sd1000;

                // ---------------------------------------------------------
                // 5. YAW EFFORT (Symmetric with Center Deadband)
                // ---------------------------------------------------------
                if (yaw > -DEADBAND && yaw < DEADBAND)
                    y_effort = 32'sd0;
                else
                    y_effort = ($signed({{20{yaw[11]}}, yaw}) * YAW_SCALE) / 32'sd1000;

                // ---------------------------------------------------------
                // 6. 4-MOTOR MIXING MATRIX (Standard Quad-X Frame)
                // ---------------------------------------------------------
                m1_calc = base_duty - r_effort + p_effort + y_effort; // Motor 1: Rear-Right
                m2_calc = base_duty + r_effort - p_effort + y_effort; // Motor 2: Front-Right
                m3_calc = base_duty + r_effort + p_effort - y_effort; // Motor 3: Rear-Left
                m4_calc = base_duty - r_effort - p_effort - y_effort; // Motor 4: Front-Left

                // ---------------------------------------------------------
                // 7. HARDWARE CLAMPING: Saturate within [DUTY_MIN, DUTY_MAX]
                // ---------------------------------------------------------
                // Motor 1
                if (m1_calc < DUTY_MIN)      duty_int_1 <= DUTY_MIN[15:0];
                else if (m1_calc > DUTY_MAX) duty_int_1 <= DUTY_MAX[15:0];
                else                         duty_int_1 <= m1_calc[15:0];

                // Motor 2
                if (m2_calc < DUTY_MIN)      duty_int_2 <= DUTY_MIN[15:0];
                else if (m2_calc > DUTY_MAX) duty_int_2 <= DUTY_MAX[15:0];
                else                         duty_int_2 <= m2_calc[15:0];

                // Motor 3
                if (m3_calc < DUTY_MIN)      duty_int_3 <= DUTY_MIN[15:0];
                else if (m3_calc > DUTY_MAX) duty_int_3 <= DUTY_MAX[15:0];
                else                         duty_int_3 <= m3_calc[15:0];

                // Motor 4
                if (m4_calc < DUTY_MIN)      duty_int_4 <= DUTY_MIN[15:0];
                else if (m4_calc > DUTY_MAX) duty_int_4 <= DUTY_MAX[15:0];
                else                         duty_int_4 <= m4_calc[15:0];
            end
        end
    end

endmodule
`default_nettype wire