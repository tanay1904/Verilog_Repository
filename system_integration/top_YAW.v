// -------------------------------------------------------------
// File Name: yaw_control_top.v
// Description: Top-level wrapper integrating the Yaw Filter 
//              and Yaw Controller with matching case-sensitive ports.
// -------------------------------------------------------------

`timescale 1 ns / 1 ns

module yaw_control_top
          (clk,
           resetn,
           clk_enable,
           kconst,
           kconst_2,
           k_int,
           gz_in,                   // Top-level Gyro Z raw input
           mag_yaw_in,              // Top-level Magnetometer raw input
           yaw_req,                 // Target Yaw setpoint input // sfix16_En12
           ce_out,                  // Synchronized downstream clock enable
           U4,                      // Actuator control command output
           ready,
           w_current_yaw,           // Filter output yaw
           w_current_yaw_rate_En10  // Filter output yaw rate
           );
  
  input clk_enable;         
  input clk;
  input resetn;
  input signed [15:0] gz_in;
  input signed [15:0] mag_yaw_in;
  input signed [15:0] yaw_req;
  input signed [15:0] kconst;  // sfix16_En12
  input signed [15:0] kconst_2;  // sfix16_En14
  input signed [15:0] k_int;  // sfix16_En14
  input ready;
  output ce_out;
  output signed [15:0] U4;
  output signed [15:0] w_current_yaw; 
  output signed [15:0] w_current_yaw_rate_En10; 

  // Interconnect wires linking the Filter outputs to the Controller inputs
//  wire signed [15:0] w_current_yaw;       // Connects psi_filt to current_yaw
//  wire signed [15:0] w_current_yaw_rate;  // Connects r_filt to current_yaw_rate
  wire f_ce_out_internal;                          // Filter clock enable output
  wire c_ce_out;                          // Controller clock enable output
  wire signed [15:0] w_current_yaw_rate_En11;
  // -----------------------------------------------------------
  // 1. Yaw Filter Instance (Exact Case & Port Matching)
  // -----------------------------------------------------------
  Yaw_filter u_Yaw_filter (
    .clk         (clk),
    .resetn      (resetn),
    .clk_enable  (clk_enable),
    .gz          (gz_in),
    .mag_yaw     (mag_yaw_in),
    .ce_out      (f_ce_out_internal),
    .psi_filt    (w_current_yaw),
    .r_filt      (w_current_yaw_rate_En10)
  );

  // -----------------------------------------------------------
  // 2. Yaw Controller Instance (Exact Case & Port Matching)
  // -----------------------------------------------------------
  yaw_controller u_yaw_controller (
    .clk                        (clk),
    .resetn                     (resetn),
    .clk_enable                 (f_ce_out_internal),
    .kconst                     (kconst),
    .kconst_2                   (kconst_2),
    .k_int                      (k_int),
    .yaw_req                    (yaw_req),
    .current_yaw                (w_current_yaw),
    .current_yaw_rate           (w_current_yaw_rate_En11),
    .ready                      (ready),
    .ce_out                     (c_ce_out),
    .U4                         (U4)
  );
  
     sfix16_en10_to_en11 u_format_converter(
    .in                 (w_current_yaw_rate_En10),
    .out                (w_current_yaw_rate_En11)
   );

  // Controller's clock enable output as the top-level status
    assign ce_out = c_ce_out;
endmodule // yaw_control_top