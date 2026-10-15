// =================================================================
// File Name: Flight_Controller_Top.v
// Description: Top module connecting alpha3rd_order_filter and
//              Altitude_Controller4, with hardware-generated
//              250 Hz sample-rate clock enable (matches MATLAB
//              base rate of 0.004 s). No software / AXI GPIO
//              involvement in sample timing.
// =================================================================
`timescale 1 ns / 1 ns

module Flight_Controller_Top
        #(
          parameter CLK_FREQ_HZ    = 10_000_000, // Actual FCLK_CLK0 / MMCM output, set in Zynq PS Clock Config
          parameter SAMPLE_RATE_HZ = 250          // Matches MATLAB model base rate: 0.004 s
        )
          (clk,
           resetn,
           ready,
           Kp_in,
           Kd_in,
           Ki_in,
           K_bias_in,
           K_pos_in,
           K_vel_in,
           barometer,
           angles_0,
           angles_1,
           accelrometer_0,
           accelrometer_1,
           accelrometer_2,
           Z_setpoint,
           feedforward_gravity,
           ce_out,
           Vest,
           Zest,
           U1_thrust);

  // -------------------------------------------------------------
  // Global Inputs
  // -------------------------------------------------------------
  input   clk;
  input   resetn;
  input   ready;
  // NOTE: clk_enable is no longer a top-level port. It is generated
  // internally by a free-running counter below, so the 4 ms sample
  // tick is a hardware guarantee, independent of software timing.

  // Filter Specific Inputs
  input   signed [23:0] barometer;           // sfix24_En16
  input   signed [23:0] angles_0;            // sfix24_En16
  input   signed [23:0] angles_1;            // sfix24_En16
  input   signed [23:0] accelrometer_0;      // sfix24_En16
  input   signed [23:0] accelrometer_1;      // sfix24_En16
  input   signed [23:0] accelrometer_2;      // sfix24_En16

  // Controller Specific Inputs
  input   signed [15:0] Z_setpoint;          // sfix16_En10
  input   signed [23:0] feedforward_gravity; // sfix24_En18

  // PID Constants
  input   signed [15:0] Kp_in;  // sfix16_En10
  input   signed [15:0] Kd_in;  // sfix16_En10
  input   signed [15:0] Ki_in;  // sfix16_En10

  // 
  input   signed [23:0] K_bias_in;  // sfix24_En16
  input   signed [31:0] K_vel_in;   // sfix32_En20
  input   signed [23:0] K_pos_in;   // sfix24_En18

  // Global Outputs
  output  ce_out;
  output  signed [23:0] U1_thrust;          // sfix24_En18
  output  signed [23:0] Zest;               // sfix24_En16
  output  signed [23:0] Vest;               // sfix24_En16
  // -------------------------------------------------------------
  // Internal Interconnect Wires
  // -------------------------------------------------------------
  wire    filter_ce_out;
  wire    controller_ce_out;

  // reset
  wire reset = ~resetn;
  // 24-bit outputs from the 3rd order filter (sfix24_En16)
  wire signed [23:0] Z_est_24;
  wire signed [23:0] V_est_24;

  // 16-bit scaled inputs for the Altitude Controller (sfix16_En10)
  wire signed [15:0] Z_est_16;
  wire signed [15:0] V_est_16;

  // Mapping filter angles (sfix24_En16) down to controller angles (sfix16_En10)
  wire signed [15:0] Angles_phi_theta_0;
  wire signed [15:0] Angles_phi_theta_1;

  // -------------------------------------------------------------
  // Bit-Width Alignment / Fixed-Point Scaling
  // Dropping 6 bits of fraction to convert En16 -> En10
  // -------------------------------------------------------------
  assign Z_est_16 = Z_est_24[21:6];
  assign V_est_16 = V_est_24[21:6];

  assign Angles_phi_theta_0 = angles_0[21:6];
  assign Angles_phi_theta_1 = angles_1[21:6];

  // Output Clock Enable Logic (Both subsystems operate at identical 0.004 base rates)
  assign ce_out = filter_ce_out & controller_ce_out;

  // -------------------------------------------------------------
  // Hardware Sample-Rate Generator
  // Free-running counter divides clk down to a single-cycle pulse
  // at SAMPLE_RATE_HZ, replacing the old software-toggled clk_enable.
  // This is the direct hardware equivalent of Simulink's fixed-step
  // base-rate timing that HDL Coder assumed when generating the
  // filter and controller math.
  // -------------------------------------------------------------
  localparam integer DIVIDER_MAX = (CLK_FREQ_HZ / SAMPLE_RATE_HZ) - 1;
  localparam integer DIVIDER_WIDTH = $clog2(DIVIDER_MAX + 1);

  reg [DIVIDER_WIDTH-1:0] div_counter;
  wire clk_enable_hw;

  always @(posedge clk or posedge reset) begin
    if (reset) begin
      div_counter <= {DIVIDER_WIDTH{1'b0}};
    end else if (div_counter == DIVIDER_MAX[DIVIDER_WIDTH-1:0]) begin
      div_counter <= {DIVIDER_WIDTH{1'b0}};
    end else begin
      div_counter <= div_counter + 1'b1;
    end
  end

  assign clk_enable_hw = (div_counter == DIVIDER_MAX[DIVIDER_WIDTH-1:0]);

  // -------------------------------------------------------------
  // Subsystem Instantiations
  // -------------------------------------------------------------

  // 1. Alpha 3rd Order Filter
  alpha3rd_order_filter u_alpha3rd_order_filter (
      .clk             (clk),
      .reset           (reset),
      .clk_enable      (clk_enable_hw),
      .K_bias_in       (K_bias_in),
      .K_pos_in        (K_pos_in),
      .K_vel_in        (K_vel_in),
      .barometer       (barometer),
      .angles_0        (angles_0),
      .angles_1        (angles_1),
      .accelrometer_0  (accelrometer_0),
      .accelrometer_1  (accelrometer_1),
      .accelrometer_2  (accelrometer_2),
      .ce_out          (filter_ce_out),
      .Z_est           (Z_est_24),
      .V_est           (V_est_24)
  );
  assign Zest = Z_est_24;
  assign Vest = V_est_24;
  // 2. Altitude Controller
  Altitude_Controller4 u_Altitude_Controller4 (
      .clk                 (clk),
      .reset               (reset),
      .clk_enable          (clk_enable_hw),
      .ready               (ready),
      .Kp                  (Kp_in),
      .Kd                  (Kd_in),
      .Ki                  (Ki_in),
      .Z_setpoint          (Z_setpoint),
      .Z_est               (Z_est_16),
      .feedforward_gravity (feedforward_gravity),
      .V_est               (V_est_16),
      .Angles_phi_theta_0  (Angles_phi_theta_0),
      .Angles_phi_theta_1  (Angles_phi_theta_1),
      .ce_out              (controller_ce_out),
      .U1_thrust           (U1_thrust)
  );

endmodule