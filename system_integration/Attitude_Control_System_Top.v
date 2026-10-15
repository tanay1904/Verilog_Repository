`timescale 1 ns / 1 ns

module Attitude_Control_System_Top (
    input         clk,
    input         reset,
    input         clk_enable,
    
    // Controller Reference Input
    input signed  [23:0] phi_raw_req,  // sfix24_En16 (Target Roll Angle)
    
    // Updated Sensor Inputs matching new filter bitwidths
    input signed  [15:0] gx,           // sfix16_En10
    input signed  [15:0] gy,           // sfix16_En10
    input signed  [15:0] ax,           // sfix16_En11
    input signed  [15:0] ay,           // sfix16_En11
    input signed  [15:0] az,           // sfix16_En11
    input         [11:0] dt,           // ufix12_En12
    
    // System Hardware Outputs
    output        ce_out,
    output signed [23:0] U2,           // sfix24_En16 (Control Torque/Command)
    
    // Exposed Internal Filter States (For Testbench Monitoring & Logging)
    output signed [15:0] phi_out,      // sfix16_En14
    output signed [15:0] p_filt_out,   // sfix16_En10 (Updated to match filter output)
    input   signed [23:0] K_ang,//sfix24En18
    input   signed [31:0] Kp_rate, //sfix32En20
    input   signed [23:0] Ki_rate, //sfix24En18
    input   signed [23:0] Kd_rate //sfix24En18
    
);

  // Internal Interconnect Wires
  wire signed [15:0] filter_phi;
  wire signed [15:0] filter_theta;    // Updated to 16-bit to match filter definition
  wire signed [15:0] filter_p_filt;
  wire signed [15:0] filter_q_filt;
  
  wire signed [23:0] bridge_phi_meas;
  wire signed [23:0] bridge_p_meas;
  
  wire filter_ce;
  wire ctrl_ce;

  // ---------------------------------------------------------------------------
  // 1. Instantiate the Complementary Filter
  // ---------------------------------------------------------------------------
  Complementary_filter u_filter (
    .clk(clk),
    .reset(reset),
    .clk_enable(clk_enable),
    .gx(gx),
    .gy(gy),
    .ax(ax),
    .ay(ay),
    .az(az),
    .dt(dt),
    .ce_out(filter_ce),
    .phi(filter_phi),
    .theta(filter_theta),
    .p_filt(filter_p_filt),
    .q_filt(filter_q_filt)
  );

  // ---------------------------------------------------------------------------
  // 2. Fixed-Point Data Type Format Matching / Bridging
  // ---------------------------------------------------------------------------
  // Convert phi (sfix16_En14) to phi_meas (sfix24_En16):
  // Shift left by 2 bits to add 2 fractional bits, sign extend upper 6 bits.
  assign bridge_phi_meas = { {6{filter_phi[15]}}, filter_phi, 2'b00 };

  // Convert p_filt (sfix16_En10) to p_meas (sfix24_En16):
  // Shift left by 6 bits to add 6 fractional bits, sign extend upper 2 bits.
  assign bridge_p_meas   = { {2{filter_p_filt[15]}}, filter_p_filt, 6'b000000 };

  // ---------------------------------------------------------------------------
  // 3. Instantiate the Attitude Roll Controller
  // ---------------------------------------------------------------------------
  Attitude_Roll_Controller_2 u_controller (
    .clk(clk),
    .reset(reset),
    .clk_enable(clk_enable),
    .phi_raw_req(phi_raw_req),
    .phi_meas(bridge_phi_meas),
    .p_meas(bridge_p_meas),
    .K_ang(K_ang),
    .Kp_rate(Kp_rate),
    .Ki_rate(Ki_rate),
    .Kd_rate(Kd_rate),
    .ce_out(ctrl_ce),
    .U2(U2)
  );

  // Assign Outputs
  assign ce_out       = ctrl_ce;
  assign phi_out      = filter_phi;
  assign p_filt_out   = filter_p_filt;

endmodule