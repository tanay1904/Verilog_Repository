`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.07.2026 16:52:13
// Design Name: 
// Module Name: Alti_Yaw_top
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


`timescale 1 ns / 1 ns
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 16.07.2026 16:52:13
// Design Name:
// Module Name: Alti_Yaw_top
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

module Alti_Yaw_top #(
parameter CLK_FREQ_HZ    = 10_000_000,
parameter SAMPLE_RATE_HZ  = 250
)(
input  wire clk,
input  wire resetn,

// Altitude / filter inputs
input  wire signed [23:0] barometer,            // sfix24_En16  // Altitude in metres
input  wire signed [23:0] angles_0,             // sfix24_En16  // Roll angle in rads (0 for alitude and yaw test)
input  wire signed [23:0] angles_1,             // sfix24_En16  // Pitch angle in rads(0 for alitude and yaw test)
input  wire signed [23:0] accelrometer_0,       // sfix24_En16  // ax in m/s^2
input  wire signed [23:0] accelrometer_1,       // sfix24_En16  // ay in m/s^2
input  wire signed [23:0] accelrometer_2,       // sfix24_En16  // az in m/s^2

// Filter constants from AXI GPIO
input  wire signed [23:0] K_bias,               // sfix24_En16
input  wire signed [31:0] K_vel,                // sfix32_En20
input  wire signed [23:0] K_pos,                // sfix24_En18

// Altitude controller inputs
input  wire signed [15:0] Z_setpoint,           // sfix16_En10  // Altitude in metres
input  wire signed [23:0] feedforward_gravity,  // sfix24_En18  // acc in m/s^2
input  wire                ready,

// PID gains
input  wire signed [15:0] Kp,                   // sfix16_En10
input  wire signed [15:0] Ki,                   // sfix16_En10
input  wire signed [15:0] Kd,                   // sfix16_En10

// Yaw inputs
input  wire signed [15:0] gz_in,                // sfix16_En10  // Angular Velocity rad/s
input  wire signed [15:0] mag_yaw_in,           // sfix16_En12  // Angle in rads
input  wire signed [15:0] yaw_req,              // sfix16_En12  // Angle in rads
input  wire signed [15:0] kconst,               // sfix16_En12 
input  wire signed [15:0] kconst_2,             // sfix16_En14
input  wire signed [15:0] k_int,                // sfix16_En12

// Outputs
output wire signed [23:0] Z_est,                    // sfix24_En16  // Altitude in metres
output wire signed [23:0] V_est,                    // sfix24_En16  // Vertical speed in m/s
output wire signed [23:0] U1_thrust,                // Force in Newtons

output wire signed [15:0] U4,                       // sfix16_En12  // Yaw Torque in nm
output wire signed [15:0] w_current_yaw,            // sfix16_En10  // Yaw angle in rads
output wire signed [15:0] w_current_yaw_rate_En10   // sfix16_En10  // Yaw angle in rads
);

wire reset;
wire ce_out;
wire clk_en_250Hz;

assign reset = ~resetn;

pulse_gen u_pulse_gen (
    .clk             (clk),
    .rstn            (resetn),
    .pulse_out_250Hz (clk_en_250Hz)
);

Flight_Controller_Top #(
    .CLK_FREQ_HZ    (CLK_FREQ_HZ),
    .SAMPLE_RATE_HZ (SAMPLE_RATE_HZ)
) u_Flight_Controller_Top (
    .clk                  (clk),
    .reset                (reset),

    .barometer            (barometer),
    .angles_0             (angles_0),
    .angles_1             (angles_1),
    .accelrometer_0       (accelrometer_0),
    .accelrometer_1       (accelrometer_1),
    .accelrometer_2       (accelrometer_2),

    .K_bias               (K_bias),
    .K_vel                (K_vel),
    .K_pos                (K_pos),
    .Z_setpoint           (Z_setpoint),
    .feedforward_gravity  (feedforward_gravity),

    .ready                (ready),

    .Kp                   (Kp),
    .Ki                   (Ki),
    .Kd                   (Kd),

    .ce_out               (ce_out),

    .Z_est                (Z_est),
    .V_est                (V_est),
    .U1_thrust            (U1_thrust)
);

yaw_control_top u_yaw_control_top (
    .clk                    (clk),
    .resetn                 (resetn),
    .clk_enable             (clk_en_250Hz),
    .ready                  (ready),

    .kconst                 (kconst),
    .kconst_2               (kconst_2),
    .k_int                  (k_int),

    .gz_in                  (gz_in),
    .mag_yaw_in             (mag_yaw_in),
    .yaw_req                (yaw_req),

    .ce_out                 (ce_out),

    .U4                     (U4),
    .w_current_yaw         (w_current_yaw),
    .w_current_yaw_rate_En10(w_current_yaw_rate_En10)
);

endmodule
