// -------------------------------------------------------------
//
// File Name: Attitude_Position_Top.v
// Description: Top level module for FIL (FPGA-in-the-Loop) testing.
//              PURE WIRE VERSION (Zero Latency Interconnects)
//
// =====================================================================
// FIX LOG (this revision)
// =====================================================================
// Two format-scaling bugs bracketing Acceleration_vectirs were found and
// corrected. Both were introduced by incorrect assumptions in THIS file's
// comments/rescale wires -- Acceleration_vectirs.v and Position_Filter.v
// themselves were always internally consistent.
//
//   1. INPUT SIDE (ax/ay/az -> Acceleration_vectirs):
//      Acceleration_vectirs.v's own header declares its ax/ay/az ports as
//      sfix16_En11 -- exactly what this module already receives at its
//      own ax/ay/az inputs. The old code below built `ax_en10` etc. via
//      a >>1 (arithmetic right shift), silently HALVING every
//      accelerometer sample before feeding it to the rotation matrix.
//      That rescale is removed; ax/ay/az are now passed straight through.
//
//   2. OUTPUT SIDE (Acceleration_vectirs -> Position_Filter):
//      Acceleration_vectirs.v's own header ALSO declares its ax_global/
//      ay_global OUTPUTS as sfix16_En11 (not En10, as the old comment
//      here incorrectly assumed). Position_Filter.v's own header states
//      its ax_global/ay_global INPUTS are sfix16_En11 too ("moved from
//      sfix16_En12 to sfix16_En11 ... rescaled to En12 internally") --
//      i.e. the two modules already agree on En11 at this boundary, and
//      Position_Filter does its own (correctly saturating) En11->En12
//      step-up internally. The old code here instead built
//      `{ax_global_w[13:0], 2'b00}` -- a raw, UNSATURATED 2-bit left
//      shift that (a) assumed the wrong source format (En10, not En11)
//      and (b) dropped the top 2 bits with no overflow check at all.
//      For realistic accelerations (a few m/s^2, raw En11 magnitude in
//      the low thousands), multiplying by 4 overflows a 16-bit word
//      every tick, wrapping to a corrupted, often wrong-signed value.
//      This is very likely why Vest/Yest stayed near zero (noisy, not
//      cleanly zero) while the true simulated velocity grew unbounded --
//      Position_Filter was integrating an aliased signal, not a genuine
//      near-zero one. That rescale is removed; ax_global_w/ay_global_w
//      are now passed straight through to Position_Filter.
//
//   A SEPARATE, properly saturating En11->En12 step-up (matching the
//   exact pattern Position_Filter.v uses internally for the same
//   conversion) is still needed for the embedded pure-wire double
//   integrator below, which synthesizes X_raw_out/Y_raw_out as a
//   simulated position measurement. That conversion is now done
//   correctly (see ax_global_en12_sat / ay_global_en12_sat).
//
//   NOTE / STILL TO VERIFY: the embedded integrator's `>>>8` / `>>>12`
//   shift constants were tuned against the OLD (corrupted, effectively
//   ~8x-scaled-and-aliased) ax_global_en12 signal. Now that the upstream
//   signal is a clean, correctly-scaled En12 value, these shift amounts
//   may need to be re-derived/re-checked in simulation -- I've kept them
//   as-is here since I don't have enough information to be certain of
//   the originally-intended internal precision of vx_integ_reg/
//   vy_integ_reg, but flagging this explicitly so it isn't mistaken for
//   a confirmed-correct value. A dimensional-consistency check (En12 *
//   En12 = En24, so >>>12 -- not >>>8 -- would keep the velocity
//   accumulator in the same En12 format as everything else) suggests
//   >>>8 may itself have been compensating for the old bug's ~8x error;
//   please re-verify X_raw_out/Y_raw_out converge to sensible values
//   against v_true/u_true once you re-run.
// =====================================================================

`timescale 1 ns / 1 ns

module Attitude_Position_Top
          (
           clk,            // 10 MHz FPGA clock
           reset,
           gx,             // sfix16_En10
           gy,             // sfix16_En10
           ax,             // sfix16_En11
           ay,             // sfix16_En11
           az,             // sfix16_En11
           ax_rm,          // sfix16_En11 
           ay_rm,          // sfix16_En11
           az_rm,          // sfix16_En11
           Psi,            // sfix16_En12
           x_setpoint,     // sfix16_En12
           y_setpoint,     // sfix16_En12
           vx_ff,          // sfix16_En12
           vy_ff,          // sfix16_En12
           ax_ff,          // sfix16_En12
           ay_ff,          // sfix16_En12
           U2,             // sfix24_En16 (Roll controller output)
           X_raw_out,      // Exposing to Block Design
           Y_raw_out,      // Exposing to Block Design
           phi,            // Exposing to Block Design
           theta,          // Exposing to Block Design
           p_filt,         // Exposing to Block Design
           q_filt,         // Exposing to Block Design
           
           // GAINS EXPOSED TO BLOCK DESIGN (From MATLAB position_gains struct)
           Kp_vel,
           Ki_vel,
           inv_g,
           Kp_pos,
           Ki_pos,
           Kw_pos,
           v_max,
           
           // NEW: GAINS EXPOSED TO BLOCK DESIGN (From MATLAB attitude_gains struct)
           Ki_rate,
           Kp_rate,
           Kd_rate,
           Kang
           );

  input               clk;
  input               reset;
  input   signed [15:0] gx;        
  input   signed [15:0] gy;        
  input   signed [15:0] ax;        
  input   signed [15:0] ay;        
  input   signed [15:0] az;        
  input   signed [15:0] ax_rm;        
  input   signed [15:0] ay_rm;        
  input   signed [15:0] az_rm;        
  input   signed [15:0] Psi;        
  input   signed [15:0] x_setpoint; 
  input   signed [15:0] y_setpoint; 
  input   signed [15:0] vx_ff;      
  input   signed [15:0] vy_ff;      
  input   signed [15:0] ax_ff;      
  input   signed [15:0] ay_ff;
  
  // POSITION GAIN INPUT DECLARATIONS
  input   signed [15:0] Kp_vel;
  input   signed [15:0] Ki_vel;
  input   signed [15:0] inv_g;
  input   signed [15:0] Kp_pos;
  input   signed [15:0] Ki_pos;
  input   signed [15:0] Kw_pos;
  input   signed [15:0] v_max;

  // NEW: ATTITUDE GAIN INPUT DECLARATIONS
  input   signed [15:0] Ki_rate;
  input   signed [15:0] Kp_rate;
  input   signed [15:0] Kd_rate;
  input   signed [15:0] Kang;

  output  signed [23:0] U2;      
  
  // OUTPUT DECLARATIONS
  output  signed [15:0] X_raw_out;
  output  signed [15:0] Y_raw_out;
  output  signed [15:0] phi;
  output  signed [15:0] theta;
  output  signed [15:0] p_filt;
  output  signed [15:0] q_filt;
      
pulse_generator u_pulse_generator(
    .clk(clk),
    .reset(reset),
    .pulse(clk_enable_250hz)
);
  wire [11:0] dt_const = 12'd16; // 0.004 in ufix12_En12

  // ===================================================================
  // Combinational Interconnect Wires (Zero Latency)
  // ===================================================================
  wire ce_out_attfilt;
  wire signed [15:0] phi_w;       // sfix16_En14
  wire signed [15:0] theta_w;     // sfix16_En14
  wire signed [15:0] p_filt_w;    // sfix16_En10
  wire signed [15:0] q_filt_w;    // sfix16_En10

  wire ce_out_posfilt;
  wire signed [15:0] Uest_w;      // sfix16_En12
  wire signed [15:0] Vest_w;      // sfix16_En12
  wire signed [15:0] Yest_w;      // sfix16_En12
  wire signed [15:0] Xest_w;      // sfix16_En12

  wire signed [15:0] ax_global_w; // sfix16_En11 (per Acceleration_vectirs.v header)
  wire signed [15:0] ay_global_w; // sfix16_En11 (per Acceleration_vectirs.v header)

  wire ce_out_posctrl;
  wire signed [17:0] theta_cmd_18; // sfix18_En12
  wire signed [17:0] phi_cmd_18;   // sfix18_En12

  wire ce_out_rollctrl;
  wire signed [23:0] U2_w;

  // ===================================================================
  // Format Scaling & Formats (Resolving Bit/Fraction Mismatches)
  // ===================================================================

  // Complementary filter outputs to Acceleration vectors
  // Accel vectors expects sfix16_En10 for angles. Filter outputs are sfix16_En14.
  wire signed [15:0] phi_en10   = { {4{phi_w[15]}}, phi_w[15:4] };
  wire signed [15:0] theta_en10 = { {4{theta_w[15]}}, theta_w[15:4] };
  
  // Top level Psi is sfix16_En12. Accel vectors expects sfix16_En10.
  wire signed [15:0] psi_en10   = { {2{Psi[15]}}, Psi[15:2] };
  
  // -------------------------------------------------------------------
  // FIX (Bug 1): ax/ay/az are sfix16_En11 at the top level, and
  // Acceleration_vectirs.v's own header declares its ax/ay/az INPUT
  // ports as sfix16_En11 too -- they already match. No rescale needed.
  // (Previously this did `{ax[15], ax[15:1]}`, an unwanted >>1 that
  // halved every accelerometer sample before the rotation matrix saw it.)
  // -------------------------------------------------------------------
//  wire signed [15:0] ax_en11 = ax;
//  wire signed [15:0] ay_en11 = ay;
//  wire signed [15:0] az_en11 = az;
  // -------------------------------------------------------------------
  // FIX (Bug 2): ax_global_w/ay_global_w are sfix16_En11 (per
  // Acceleration_vectirs.v's own header), and Position_Filter.v's own
  // header declares its ax_global/ay_global INPUT ports as sfix16_En11
  // too (it does its own saturating En11->En12 step-up internally) --
  // they already match. No rescale needed at this boundary at all.
  // (Previously this did `{ax_global_w[13:0], 2'b00}`, which assumed
  // the wrong source format AND had no saturation check, overflowing
  // for any realistic acceleration magnitude.)
  // -------------------------------------------------------------------
  wire signed [15:0] ax_global_en11 = ax_global_w;
  wire signed [15:0] ay_global_en11 = ay_global_w;

  // -------------------------------------------------------------------
  // A properly SATURATING En11 -> En12 step-up is still needed for the
  // embedded double-integrator plant below (it wants En12 to match
  // dt_const's En12 format). This mirrors exactly the saturating
  // 1-bit-shift pattern Position_Filter.v itself uses internally for
  // this identical conversion -- i.e. this is now done the same way
  // twice (once here for the X_raw/Y_raw synth, once inside
  // Position_Filter for its own feedforward term), rather than the old
  // single broken 2-bit unsaturated shift.
  // -------------------------------------------------------------------
  wire signed [15:0] ax_global_en12_sat =
      ((ax_global_w[15] == 1'b0) && (ax_global_w[14] != 1'b0) ? 16'sb0111111111111111 :
      ((ax_global_w[15] == 1'b1) && (ax_global_w[14] != 1'b1) ? 16'sb1000000000000000 :
      {ax_global_w[14:0], 1'b0}));
  wire signed [15:0] ay_global_en12_sat =
      ((ay_global_w[15] == 1'b0) && (ay_global_w[14] != 1'b0) ? 16'sb0111111111111111 :
      ((ay_global_w[15] == 1'b1) && (ay_global_w[14] != 1'b1) ? 16'sb1000000000000000 :
      {ay_global_w[14:0], 1'b0}));

  // ===================================================================
  // Embedded Pure-Wire Double Integrator Plant
  // ===================================================================
  reg signed [31:0] vx_integ_reg;
  reg signed [31:0] vy_integ_reg;
  reg signed [31:0] x_integ_reg;
  reg signed [31:0] y_integ_reg;

  // NOTE: now sourced from the correctly-scaled, saturating En12 signal
  // (ax_global_en12_sat) instead of the old broken ax_global_en12.
  wire signed [31:0] ax_ext = { {16{ax_global_en12_sat[15]}}, ax_global_en12_sat };
  wire signed [31:0] ay_ext = { {16{ay_global_en12_sat[15]}}, ay_global_en12_sat };

  always @(posedge clk or posedge reset) begin
    if (reset) begin
      vx_integ_reg <= 32'd0;
      vy_integ_reg <= 32'd0;
      x_integ_reg  <= 32'd0;
      y_integ_reg  <= 32'd0;
    end else if (clk_enable_250hz) begin
      // Integrated directly from the live combinational wires (Zero Latency)
      // See file header NOTE: these shift amounts were tuned against the
      // old (corrupted) upstream signal and should be re-verified now
      // that ax_ext/ay_ext are a clean, correctly-scaled En12 value.
      vx_integ_reg <= vx_integ_reg + ((ax_ext * $signed({20'd0, dt_const})) >>> 12);
      vy_integ_reg <= vy_integ_reg + ((ay_ext * $signed({20'd0, dt_const})) >>> 12);
//12 , not 8
      x_integ_reg  <= x_integ_reg + ((vx_integ_reg * $signed({20'd0, dt_const})) >>> 12);
      y_integ_reg  <= y_integ_reg + ((vy_integ_reg * $signed({20'd0, dt_const})) >>> 12);
    end
  end

  // Drive the new output ports with the integrated values
  assign X_raw_out = x_integ_reg[27:12];
  assign Y_raw_out = y_integ_reg[27:12];

  // ===================================================================
  // Bus mappings for Position Controller 6 (needs sfix18_En12)
  // ===================================================================
  wire signed [17:0] Psi_18        = { {2{Psi[15]}}, Psi };
  wire signed [17:0] x_setpoint_18 = { {2{x_setpoint[15]}}, x_setpoint };
  wire signed [17:0] y_setpoint_18 = { {2{y_setpoint[15]}}, y_setpoint };
  wire signed [17:0] Uest_18       = { {2{Uest_w[15]}}, Uest_w };
  wire signed [17:0] Vest_18       = { {2{Vest_w[15]}}, Vest_w };
  wire signed [17:0] Yest_18       = { {2{Yest_w[15]}}, Yest_w };
  wire signed [17:0] Xest_18       = { {2{Xest_w[15]}}, Xest_w };
  wire signed [17:0] vx_ff_18      = { {2{vx_ff[15]}}, vx_ff };
  wire signed [17:0] vy_ff_18      = { {2{vy_ff[15]}}, vy_ff };
  wire signed [17:0] ax_ff_18      = { {2{ax_ff[15]}}, ax_ff };
  wire signed [17:0] ay_ff_18      = { {2{ay_ff[15]}}, ay_ff };
  
  // Position Gains (18-bit extension)
  wire signed [17:0] Kp_pos_18     = { {2{Kp_pos[15]}}, Kp_pos };
  wire signed [17:0] Ki_pos_18     = { {2{Ki_pos[15]}}, Ki_pos };
  wire signed [17:0] Kw_pos_18     = { {2{Kw_pos[15]}}, Kw_pos };
  wire signed [17:0] Kp_vel_18     = { {2{Kp_vel[15]}}, Kp_vel };
  wire signed [17:0] Ki_vel_18     = { {2{Ki_vel[15]}}, Ki_vel };
  wire signed [17:0] inv_g_18      = { {2{inv_g[15]}}, inv_g };
  wire signed [17:0] v_max_18      = { {2{v_max[15]}}, v_max };

  // ===================================================================
  // Bus mappings for Attitude Roll Controller 2 (needs sfix24_En16)
  // ===================================================================
  wire signed [23:0] phi_meas_ext    = { {6{phi_w[15]}}, phi_w, 2'b00 };
  wire signed [23:0] p_meas_ext      = { {2{p_filt_w[15]}}, p_filt_w, 6'b000000 };
  wire signed [23:0] phi_raw_req_ext = { {2{phi_cmd_18[17]}}, phi_cmd_18, 4'b0000 };

  // ===================================================================
  // Module Instantiations
  // ===================================================================

  Complementary_filter u_Complementary_filter (
    .clk        (clk),
    .reset      (reset),
    .clk_enable (clk_enable_250hz),
    .gx         (gx),
    .gy         (gy),
    .ax         (ax),
    .ay         (ay),
    .az         (az),
    .dt         (dt_const),
    .ce_out     (ce_out_attfilt),
    .phi        (phi_w),
    .theta      (theta_w),
    .p_filt     (p_filt_w),
    .q_filt     (q_filt_w)
  );

  Acceleration_vectirs u_Acceleration_vectirs (
    .ax         (ax_rm),
    .ay         (ay_rm),
    .az         (az_rm),
    .phi        (phi_en10),   
    .theta      (theta_en10), 
    .psi        (psi_en10),
    .ax_global  (ax_global_w),
    .ay_global  (ay_global_w)
  );

  Position_Filter u_Position_Filter (
    .clk        (clk),
    .reset      (reset),
    .clk_enable (clk_enable_250hz),
    .X_raw      (X_raw_out),        
    .Y_raw      (Y_raw_out),        
    .ax_global  (ax_global_en11),  
    .ay_global  (ay_global_en11),  
    .ce_out     (ce_out_posfilt),
    .Uest       (Uest_w),
    .Vest       (Vest_w),
    .Yest       (Yest_w),
    .Xest       (Xest_w)
  );

  Position_Controller_6 u_Position_Controller_6 (
    .clk        (clk),
    .reset      (reset),
    .clk_enable (clk_enable_250hz),
    .Psi        (Psi_18),
    .x_setpoint (x_setpoint_18),
    .y_setpoint (y_setpoint_18),
    .U_est      (Uest_18),
    .V_est      (Vest_18),
    .y_est      (Yest_18),
    .x_est      (Xest_18),
    .vx_ff      (vx_ff_18),
    .vy_ff      (vy_ff_18),
    .ax_ff      (ax_ff_18),
    .ay_ff      (ay_ff_18),
    .Kp_pos     (Kp_pos_18),
    .Ki_pos     (Ki_pos_18),
    .Kw_pos     (Kw_pos_18),
    .Kp_vel     (Kp_vel_18),
    .Ki_vel     (Ki_vel_18),
    .inv_g      (inv_g_18),
    .v_max      (v_max_18),
    .ce_out     (ce_out_posctrl),
    .theta      (theta_cmd_18),   
    .phi        (phi_cmd_18)
  );

  Attitude_Roll_Controller_2 u_Attitude_Roll_Controller_2 (
    .clk         (clk),
    .reset       (reset),
    .clk_enable  (clk_enable_250hz),
    .phi_raw_req (phi_raw_req_ext),
    .phi_meas    (phi_meas_ext),
    .p_meas      (p_meas_ext),
    
    // NEW: PASS UPDATED GAINS TO THE ATTITUDE CONTROLLER
    .Ki_rate     (Ki_rate),
    .Kp_rate     (Kp_rate),
    .Kd_rate     (Kd_rate),
    .Kang        (Kang),
    
    .ce_out      (ce_out_rollctrl),
    .U2          (U2_w)
  );

  // Direct Combinational assignment back to final pin output
  assign U2 = U2_w;
  
  // Expose filter wires to the module's output pins
  assign phi    = phi_w;
  assign theta  = theta_w;
  assign p_filt = p_filt_w;
  assign q_filt = q_filt_w;

endmodule



//// -------------------------------------------------------------
////
//// File Name: Attitude_Position_Top.v
//// Description: Top level module for FIL (FPGA-in-the-Loop) testing.
////              PURE WIRE VERSION (Zero Latency Interconnects)
////
//// -------------------------------------------------------------

//`timescale 1 ns / 1 ns

//module Attitude_Position_Top
//          (
//           clk,            // 10 MHz FPGA clock
//           reset,
//           gx,             // sfix16_En10
//           gy,             // sfix16_En10
//           ax,             // sfix16_En11
//           ay,             // sfix16_En11
//           az,             // sfix16_En11
//           Psi,            // sfix16_En12
//           x_setpoint,     // sfix16_En12
//           y_setpoint,     // sfix16_En12
//           vx_ff,          // sfix16_En12
//           vy_ff,          // sfix16_En12
//           ax_ff,          // sfix16_En12
//           ay_ff,          // sfix16_En12
//           U2,             // sfix24_En16 (Roll controller output)
//           X_raw_out,      // Exposing to Block Design
//           Y_raw_out,      // Exposing to Block Design
//           phi,            // Exposing to Block Design
//           theta,          // Exposing to Block Design
//           p_filt,         // Exposing to Block Design
//           q_filt,         // Exposing to Block Design
           
//           // GAINS EXPOSED TO BLOCK DESIGN (From MATLAB position_gains struct)
//           Kp_vel,
//           Ki_vel,
//           inv_g,
//           Kp_pos,
//           Ki_pos,
//           Kw_pos,
//           v_max,
           
//           // NEW: GAINS EXPOSED TO BLOCK DESIGN (From MATLAB attitude_gains struct)
//           Ki_rate,
//           Kp_rate,
//           Kd_rate,
//           Kang
//           );

//  input               clk;
//  input               reset;
//  input   signed [15:0] gx;        
//  input   signed [15:0] gy;        
//  input   signed [15:0] ax;        
//  input   signed [15:0] ay;        
//  input   signed [15:0] az;        
//  input   signed [15:0] Psi;        
//  input   signed [15:0] x_setpoint; 
//  input   signed [15:0] y_setpoint; 
//  input   signed [15:0] vx_ff;      
//  input   signed [15:0] vy_ff;      
//  input   signed [15:0] ax_ff;      
//  input   signed [15:0] ay_ff;
  
//  // POSITION GAIN INPUT DECLARATIONS
//  input   signed [15:0] Kp_vel;
//  input   signed [15:0] Ki_vel;
//  input   signed [15:0] inv_g;
//  input   signed [15:0] Kp_pos;
//  input   signed [15:0] Ki_pos;
//  input   signed [15:0] Kw_pos;
//  input   signed [15:0] v_max;

//  // NEW: ATTITUDE GAIN INPUT DECLARATIONS
//  input   signed [15:0] Ki_rate;
//  input   signed [15:0] Kp_rate;
//  input   signed [15:0] Kd_rate;
//  input   signed [15:0] Kang;

//  output  signed [23:0] U2;      
  
//  // OUTPUT DECLARATIONS
//  output  signed [15:0] X_raw_out;
//  output  signed [15:0] Y_raw_out;
//  output  signed [15:0] phi;
//  output  signed [15:0] theta;
//  output  signed [15:0] p_filt;
//  output  signed [15:0] q_filt;
      

//  // ===================================================================
//  // Clock Enable Generation (10 MHz -> 250 Hz)
//  // ===================================================================
//  localparam integer CLK_FREQ_HZ = 10_000_000;
//  localparam integer CE_FREQ_HZ  = 250;
//  localparam integer CE_DIVIDE   = CLK_FREQ_HZ / CE_FREQ_HZ; 
//  localparam integer CE_CNT_W    = 16;

//  reg [CE_CNT_W-1:0] ce_counter;
//  reg                clk_enable_250hz;

//  always @(posedge clk or posedge reset) begin
//    if (reset) begin
//      ce_counter       <= {CE_CNT_W{1'b0}};
//      clk_enable_250hz <= 1'b0;
//    end
//    else begin
//      if (ce_counter == CE_DIVIDE - 1) begin
//        ce_counter       <= {CE_CNT_W{1'b0}};
//        clk_enable_250hz <= 1'b1;   
//      end
//      else begin
//        ce_counter       <= ce_counter + 1'b1;
//        clk_enable_250hz <= 1'b0;
//      end
//    end
//  end

//  wire [11:0] dt_const = 12'd16; // 0.004 in ufix12_En12

//  // ===================================================================
//  // Combinational Interconnect Wires (Zero Latency)
//  // ===================================================================
//  wire ce_out_attfilt;
//  wire signed [15:0] phi_w;       // sfix16_En14
//  wire signed [15:0] theta_w;     // sfix16_En14
//  wire signed [15:0] p_filt_w;    // sfix16_En10
//  wire signed [15:0] q_filt_w;    // sfix16_En10

//  wire ce_out_posfilt;
//  wire signed [15:0] Uest_w;      // sfix16_En12
//  wire signed [15:0] Vest_w;      // sfix16_En12
//  wire signed [15:0] Yest_w;      // sfix16_En12
//  wire signed [15:0] Xest_w;      // sfix16_En12
  
//   wire signed [15:0] ax_global_w; // sfix16_En11 (per Acceleration_vectirs.v header)
//   wire signed [15:0] ay_global_w; // sfix16_En11 (per Acceleration_vectirs.v header)

////  wire signed [15:0] ax_global_w; // sfix16_En10
////  wire signed [15:0] ay_global_w; // sfix16_En10

//  wire ce_out_posctrl;
//  wire signed [17:0] theta_cmd_18; // sfix18_En12
//  wire signed [17:0] phi_cmd_18;   // sfix18_En12

//  wire ce_out_rollctrl;
//  wire signed [23:0] U2_w;

//  // ===================================================================
//  // Format Scaling & Formats (Resolving Bit/Fraction Mismatches)
//  // ===================================================================

//  // Complementary filter outputs to Acceleration vectors
//  // Accel vectors expects sfix16_En10 for angles. Filter outputs are sfix16_En14.
//  wire signed [15:0] phi_en10   = { {4{phi_w[15]}}, phi_w[15:4] };
//  wire signed [15:0] theta_en10 = { {4{theta_w[15]}}, theta_w[15:4] };
  
//  // Top level Psi is sfix16_En12. Accel vectors expects sfix16_En10.
//  wire signed [15:0] psi_en10   = { {2{Psi[15]}}, Psi[15:2] };
  
//  // Top level ax, ay, az are sfix16_En11. Accel vectors expects sfix16_En10.
//  wire signed [15:0] ax_en10    = { ax[15], ax[15:1] };
//  wire signed [15:0] ay_en10    = { ay[15], ay[15:1] };
//  wire signed [15:0] az_en10    = { az[15], az[15:1] };

//  // Acceleration vectors outputs to Position Filter & Integrator
//  // Accel vectors output is sfix16_En10. Position filter expects sfix16_En12.
//  wire signed [15:0] ax_global_en12 = { ax_global_w[13:0], 2'b00 };
//  wire signed [15:0] ay_global_en12 = { ay_global_w[13:0], 2'b00 };

//  // ===================================================================
//  // Embedded Pure-Wire Double Integrator Plant
//  // ===================================================================
//  reg signed [31:0] vx_integ_reg;
//  reg signed [31:0] vy_integ_reg;
//  reg signed [31:0] x_integ_reg;
//  reg signed [31:0] y_integ_reg;

//  wire signed [31:0] ax_ext = { {16{ax_global_en12[15]}}, ax_global_en12 };
//  wire signed [31:0] ay_ext = { {16{ay_global_en12[15]}}, ay_global_en12 };

//  always @(posedge clk or posedge reset) begin
//    if (reset) begin
//      vx_integ_reg <= 32'd0;
//      vy_integ_reg <= 32'd0;
//      x_integ_reg  <= 32'd0;
//      y_integ_reg  <= 32'd0;
//    end else if (clk_enable_250hz) begin
//      // Integrated directly from the live combinational wires (Zero Latency)
//      vx_integ_reg <= vx_integ_reg + ((ax_ext * $signed({20'd0, dt_const})) >>> 8);
//      vy_integ_reg <= vy_integ_reg + ((ay_ext * $signed({20'd0, dt_const})) >>> 8);

//      x_integ_reg  <= x_integ_reg + ((vx_integ_reg * $signed({20'd0, dt_const})) >>> 12);
//      y_integ_reg  <= y_integ_reg + ((vy_integ_reg * $signed({20'd0, dt_const})) >>> 12);
//    end
//  end

//  // Drive the new output ports with the integrated values
//  assign X_raw_out = x_integ_reg[27:12];
//  assign Y_raw_out = y_integ_reg[27:12];

//  // ===================================================================
//  // Bus mappings for Position Controller 6 (needs sfix18_En12)
//  // ===================================================================
//  wire signed [17:0] Psi_18        = { {2{Psi[15]}}, Psi };
//  wire signed [17:0] x_setpoint_18 = { {2{x_setpoint[15]}}, x_setpoint };
//  wire signed [17:0] y_setpoint_18 = { {2{y_setpoint[15]}}, y_setpoint };
//  wire signed [17:0] Uest_18       = { {2{Uest_w[15]}}, Uest_w };
//  wire signed [17:0] Vest_18       = { {2{Vest_w[15]}}, Vest_w };
//  wire signed [17:0] Yest_18       = { {2{Yest_w[15]}}, Yest_w };
//  wire signed [17:0] Xest_18       = { {2{Xest_w[15]}}, Xest_w };
//  wire signed [17:0] vx_ff_18      = { {2{vx_ff[15]}}, vx_ff };
//  wire signed [17:0] vy_ff_18      = { {2{vy_ff[15]}}, vy_ff };
//  wire signed [17:0] ax_ff_18      = { {2{ax_ff[15]}}, ax_ff };
//  wire signed [17:0] ay_ff_18      = { {2{ay_ff[15]}}, ay_ff };
  
//  // Position Gains (18-bit extension)
//  wire signed [17:0] Kp_pos_18     = { {2{Kp_pos[15]}}, Kp_pos };
//  wire signed [17:0] Ki_pos_18     = { {2{Ki_pos[15]}}, Ki_pos };
//  wire signed [17:0] Kw_pos_18     = { {2{Kw_pos[15]}}, Kw_pos };
//  wire signed [17:0] Kp_vel_18     = { {2{Kp_vel[15]}}, Kp_vel };
//  wire signed [17:0] Ki_vel_18     = { {2{Ki_vel[15]}}, Ki_vel };
//  wire signed [17:0] inv_g_18      = { {2{inv_g[15]}}, inv_g };
//  wire signed [17:0] v_max_18      = { {2{v_max[15]}}, v_max };

//  // ===================================================================
//  // Bus mappings for Attitude Roll Controller 2 (needs sfix24_En16)
//  // ===================================================================
//  wire signed [23:0] phi_meas_ext    = { {6{phi_w[15]}}, phi_w, 2'b00 };
//  wire signed [23:0] p_meas_ext      = { {2{p_filt_w[15]}}, p_filt_w, 6'b000000 };
//  wire signed [23:0] phi_raw_req_ext = { {2{phi_cmd_18[17]}}, phi_cmd_18, 4'b0000 };

//  // ===================================================================
//  // Module Instantiations
//  // ===================================================================

//  Complementary_filter u_Complementary_filter (
//    .clk        (clk),
//    .reset      (reset),
//    .clk_enable (clk_enable_250hz),
//    .gx         (gx),
//    .gy         (gy),
//    .ax         (ax),
//    .ay         (ay),
//    .az         (az),
//    .dt         (dt_const),
//    .ce_out     (ce_out_attfilt),
//    .phi        (phi_w),
//    .theta      (theta_w),
//    .p_filt     (p_filt_w),
//    .q_filt     (q_filt_w)
//  );

//  Acceleration_vectirs u_Acceleration_vectirs (
//    .ax         (ax_en10),
//    .ay         (ay_en10),
//    .az         (az_en10),
//    .phi        (phi_en10),   
//    .theta      (theta_en10), 
//    .psi        (psi_en10),
//    .ax_global  (ax_global_w),
//    .ay_global  (ay_global_w)
//  );

//  Position_Filter u_Position_Filter (
//    .clk        (clk),
//    .reset      (reset),
//    .clk_enable (clk_enable_250hz),
//    .X_raw      (X_raw_out),        
//    .Y_raw      (Y_raw_out),        
//    .ax_global  (ax_global_en12),  
//    .ay_global  (ay_global_en12),  
//    .ce_out     (ce_out_posfilt),
//    .Uest       (Uest_w),
//    .Vest       (Vest_w),
//    .Yest       (Yest_w),
//    .Xest       (Xest_w)
//  );

//  Position_Controller_6 u_Position_Controller_6 (
//    .clk        (clk),
//    .reset      (reset),
//    .clk_enable (clk_enable_250hz),
//    .Psi        (Psi_18),
//    .x_setpoint (x_setpoint_18),
//    .y_setpoint (y_setpoint_18),
//    .U_est      (Uest_18),
//    .V_est      (Vest_18),
//    .y_est      (Yest_18),
//    .x_est      (Xest_18),
//    .vx_ff      (vx_ff_18),
//    .vy_ff      (vy_ff_18),
//    .ax_ff      (ax_ff_18),
//    .ay_ff      (ay_ff_18),
//    .Kp_pos     (Kp_pos_18),
//    .Ki_pos     (Ki_pos_18),
//    .Kw_pos     (Kw_pos_18),
//    .Kp_vel     (Kp_vel_18),
//    .Ki_vel     (Ki_vel_18),
//    .inv_g      (inv_g_18),
//    .v_max      (v_max_18),
//    .ce_out     (ce_out_posctrl),
//    .theta      (theta_cmd_18),   
//    .phi        (phi_cmd_18)
//  );

//  Attitude_Roll_Controller_2 u_Attitude_Roll_Controller_2 (
//    .clk         (clk),
//    .reset       (reset),
//    .clk_enable  (clk_enable_250hz),
//    .phi_raw_req (phi_raw_req_ext),
//    .phi_meas    (phi_meas_ext),
//    .p_meas      (p_meas_ext),
    
//    // NEW: PASS UPDATED GAINS TO THE ATTITUDE CONTROLLER
//    .Ki_rate     (Ki_rate),
//    .Kp_rate     (Kp_rate),
//    .Kd_rate     (Kd_rate),
//    .Kang        (Kang),
    
//    .ce_out      (ce_out_rollctrl),
//    .U2          (U2_w)
//  );

//  // Direct Combinational assignment back to final pin output
//  assign U2 = U2_w;
  
//  // Expose filter wires to the module's output pins
//  assign phi    = phi_w;
//  assign theta  = theta_w;
//  assign p_filt = p_filt_w;
//  assign q_filt = q_filt_w;

//endmodule