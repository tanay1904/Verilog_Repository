// =====================================================================
// File Name : tb_closed_loop_fil.sv
// Description: Closed-loop FIL testbench for Attitude_Position_Top.
//
//   Loop under test:
//     Complementary Filter -> Body-to-Global Accel -> Position Filter
//     -> Position Controller -> Roll Controller -> PLANT -> (back to
//     Complementary Filter via gx/gy/ax/ay/az)
//
//   The plant is the exact nonlinear 6DOF model supplied:
//
//     u_dot = (v*r - w*q) + g*sin(theta)
//     v_dot = (w*p - u*r) - g*sin(phi)*cos(theta)
//     w_dot = (u*q - v*p) - g*cos(phi)*cos(theta) + U1/m
//     p_dot = ((Iy-Iz)/Ix)*q*r + U2/Ix
//     q_dot = ((Iz-Ix)/Iy)*p*r + U3/Iy
//     r_dot = ((Ix-Iy)/Iz)*p*q + U4/Iz
//     z_dot = -u*sin(theta) + v*sin(phi)*cos(theta) + w*cos(phi)*cos(theta)
//
//   =================================================================
//   CHANGE LOG (this revision)
//   =================================================================
//   - Added a substepped plant integrator. Previously the full nonlinear
//     6DOF + attitude-kinematics equations were advanced with a single
//     forward-Euler step of dt = 0.004 s (tied 1:1 to the DUT's 250 Hz
//     clock enable). Given Ix = 0.0030 kg*m^2 and the roll-torque
//     saturation rail (~1.1612, decoded from the controller's Saturation
//     constant), the resulting angular acceleration at the rail is
//     ~387 rad/s^2 -- a single 4 ms step moves p by ~1.5 rad/s, which by
//     itself produces bang-bang-looking chatter in p/U2 independent of
//     controller tuning. This is a plant-model fidelity problem, not a
//     controller bug.
//
//     Fix: the plant + attitude kinematics now integrate at a much finer
//     internal step (SUBSTEPS per controller tick, dt_sub = dt/SUBSTEPS),
//     while the control input U2 is sampled ONCE per controller tick and
//     held constant (zero-order hold) across all substeps -- exactly
//     matching how a real continuous-time plant is exercised against a
//     fixed-rate discrete controller in FIL/HIL setups. The DUT's
//     sample timing, clock-enable rate, and IMU synthesis are completely
//     unchanged; only the internal plant fidelity improved.
//
//   - Still open / NOT fixed here: Y_est / V_est staying near zero while
//     v_true grows without bound. That is a position-loop issue upstream
//     of what this testbench can address -- see chat notes. It depends
//     on the exact transform inside Acceleration_vectirs.v (not yet
//     provided), which converts body-frame ax/ay/az + phi/theta into
//     ax_global/ay_global for Position_Filter. Diagnostic hooks are left
//     in comments below (see section 7) for once that file is available.
//   =================================================================
//
//   NOTE on IMU synthesis: the accelerometer channels (ax/ay/az) are
//   driven from the gravity vector resolved into body axes
//   (-g*sin(theta), g*sin(phi)*cos(theta), g*cos(phi)*cos(theta)) --
//   the standard convention every complementary/AHRS filter relies on
//   for an absolute tilt reference, and it is exact here because thrust
//   is held at U1 = m*g (hover-compensating assumption), so body-frame
//   specific force truly is (0,0,g) every tick. Gyro channels (gx/gy)
//   are driven directly from p_st/q_st.
//
//   NOTE on X_raw_out/Y_raw_out display format: the top-level file does
//   not comment their Q-format explicitly. Bit-tracing the internal
//   integrator (Q16 accumulator, [27:12] slice) suggests something
//   coarser than En12; they are printed here assuming En12 purely for
//   human-readable debug -- this does not affect the correctness of
//   the stimulus driven into the DUT.
// =====================================================================

`timescale 1ns / 1ps

module tb_closed_loop_fil();

    // =================================================================
    // 1. Clock and Reset
    // =================================================================
    logic clk;      // 10Mhz (100ns)
    logic reset;    

    localparam real CLK_PERIOD_NS = 100.0; // 10 MHz
    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD_NS/2.0) clk = ~clk;
    end

    // =================================================================
    // 2. DUT I/O
    // =================================================================
    logic signed [15:0] gx, gy;                     // Initially 0; Taken from Gyroscope(IMU)
    logic signed [15:0] ax, ay, az;                 // Initially 0, 0, -9.81    Accelerometer
    logic signed [15:0] ax_rm, ay_rm, az_rm;        // Initially 0, 0, -9.81    Accelerometer
    logic signed [15:0] Psi;                        // Yaw angle 0
    logic signed [15:0] x_setpoint, y_setpoint;     // variable: Initiallly set x = 0, y = 0.2 
    logic signed [15:0] vx_ff, vy_ff, ax_ff, ay_ff; // Feed forward for High level; Assume 0 for low level

    logic signed [15:0] Kp_vel, Ki_vel, inv_g;      // 0.5097 ; 0.6371 ; 0.1019 ; 
    logic signed [15:0] Kp_pos, Ki_pos, Kw_pos, v_max;  // 1.0000; 0.25 ; 0.5 ; 5.0000
    logic signed [15:0] Ki_rate, Kp_rate, Kd_rate, Kang;// 4.7250; 0.3381; 0.0065; 4.7600

    wire signed [23:0] U2_out;                          // Initially 0
    wire signed [15:0] X_raw_out, Y_raw_out;            // Initially 0
    wire signed [15:0] phi_out, theta_out;              // Initially 0
    wire signed [15:0] p_filt_out, q_filt_out;          // Initially 0

    // =================================================================
    // 3. Physical Plant Parameters (real MATLAB values)
    // =================================================================
    real m  = 0.7;      // Mass (kg)
    real g  = 9.81;      // Gravity (m/s^2)
    real Ix = 0.0030;    // Roll inertia (kg*m^2)
    real Iy = 0.0033;    // Pitch inertia (kg*m^2)
    real Iz = 0.0060;    // Yaw inertia (kg*m^2)
    real dt = 0.004;     // 250 Hz controller tick -- must match DUT's ce rate

    // ---- Substepped integration settings ----
    // The controller/DUT still only sees a new U2 once every dt = 4 ms.
    // Internally we advance the continuous plant SUBSTEPS times per tick
    // at dt_sub, holding U2 (and U1/U3/U4) constant across the substeps.
    localparam int SUBSTEPS = 40;             // 4ms / 40 = 100 us per substep
    real dt_sub;
    initial dt_sub = dt / SUBSTEPS;

    // Persistent translational / rotational states (mirrors MATLAB persistents)
    real u_st = 0.0, v_st = 0.0, w_st = 0.0;
    real p_st = 0.0, q_st = 0.0, r_st = 0.0;
    real z_st = 0.0;

    // True attitude states (integrated in TB, fed back into plant eqns)
    real phi_st = 0.0, theta_st = 0.0;

    // Derivatives (kept as named variables for clarity / debug printing)
    real u_dot, v_dot, w_dot;               // 0.0
    real p_dot, q_dot, r_dot;               // 0.0
    real z_dot;                             // 0.0
    real phi_dot, theta_dot;                // 0.0

    // Control inputs to plant (sampled once per controller tick, held
    // constant across all substeps within that tick -- zero-order hold,
    // matching how the real DUT's actuator command only updates at its
    // own 250 Hz rate)
    real U1, U2_r, U3, U4;  // 0, (starts with 0), 0, 0
//assign U2_r = 1.0;
    // =================================================================
    // 4. Fixed-point <-> real conversion helpers
    // =================================================================
    function automatic real sfix_to_real(input longint signed val, input int frac_bits);
        sfix_to_real = real'(val) / (2.0 ** frac_bits);
    endfunction

    function automatic logic signed [15:0] real_to_sfix16(input real val, input int frac_bits);
        longint scaled;
        scaled = longint'($rtoi(val * (2.0 ** frac_bits)));
        if (scaled > 32767)  scaled = 32767;
        if (scaled < -32768) scaled = -32768;
        real_to_sfix16 = scaled[15:0];
    endfunction

    // =================================================================
    // 5. DUT Instantiation
    // =================================================================
    Attitude_Position_Top uut (
        .clk(clk),
        .reset(reset),
        .gx(gx), .gy(gy),
        .ax(ax), .ay(ay), .az(az),
        .ax_rm(ax), .ay_rm(ay), .az_rm(az),
        .Psi(Psi),
        .x_setpoint(x_setpoint), .y_setpoint(y_setpoint),
        .vx_ff(vx_ff), .vy_ff(vy_ff),
        .ax_ff(ax_ff), .ay_ff(ay_ff),
        .U2(U2_out),
        .X_raw_out(X_raw_out), .Y_raw_out(Y_raw_out),
        .phi(phi_out), .theta(theta_out),
        .p_filt(p_filt_out), .q_filt(q_filt_out),
        .Kp_vel(Kp_vel), .Ki_vel(Ki_vel), .inv_g(inv_g),
        .Kp_pos(Kp_pos), .Ki_pos(Ki_pos), .Kw_pos(Kw_pos), .v_max(v_max),
        .Ki_rate(Ki_rate), .Kp_rate(Kp_rate), .Kd_rate(Kd_rate), .Kang(Kang)
    );

    // =================================================================
    // 6. Stimulus: gains, reset, setpoints, run length
    // =================================================================
    localparam longint RESET_NS       = 64'd1_000;          // 1us deassert reset
    localparam longint SETTLE_NS      = 64'd50_000_000;     // 50 ms after reset before commanding setpoint
    localparam longint SIM_TIME_NS    = 64'd30_000_000_000; // 60 s total

    initial begin
        // Zero all DUT inputs initially
        gx = 0; gy = 0; ax = 0; ay = 0; az = 0; Psi = 0;
        x_setpoint = 0; y_setpoint = 0;
        vx_ff = 0; vy_ff = 0; ax_ff = 0; ay_ff = 0;

        // Position-loop gains (En12)
        Kp_vel = real_to_sfix16(0.5097, 12);
        Ki_vel = real_to_sfix16(0.6371, 12);
        inv_g  = real_to_sfix16(0.1019, 12);
        Kp_pos = real_to_sfix16(1.0000, 12);
        Ki_pos = real_to_sfix16(0.2500, 12);
        Kw_pos = real_to_sfix16(0.5000, 12);
        v_max  = real_to_sfix16(5.0000, 12);

        // Attitude-loop gains (En12)
        Ki_rate = real_to_sfix16(4.7250, 12);
        Kp_rate = real_to_sfix16(0.3381, 12);
        Kd_rate = real_to_sfix16(0.0065, 12);
        Kang    = real_to_sfix16(4.7600, 12);

        // Reset pulse
        reset = 1'b1;
        #(RESET_NS);
        reset = 1'b0;

        // Let the loop settle at zero setpoint briefly, then command
        // x_setpoint = 0, y_setpoint = 0.2 m (roll-axis step test)
//        #(SETTLE_NS);
        x_setpoint = real_to_sfix16(0.0, 12);
        y_setpoint = real_to_sfix16(0.2, 12);

        #(SIM_TIME_NS);
        $display("Simulation complete: %0.1f seconds elapsed.", SIM_TIME_NS/1.0e9);
        $finish;
    end

    // =================================================================
    // 7. Plant Physics + IMU Synthesis
    //    Triggered directly off the DUT's own 250 Hz clock-enable so the
    //    plant and controller are guaranteed to update on the exact same
    //    tick -- no separate/duplicated divider logic to drift out of
    //    sync with the DUT.
    //
    //    U2 (and U1/U3/U4) are sampled ONCE at the top of the tick, then
    //    the nonlinear plant + attitude kinematics are advanced SUBSTEPS
    //    times at dt_sub with that control input held fixed. Only after
    //    all substeps complete are the next-cycle IMU values (gx/gy/ax/
    //    ay/az) synthesized and latched for the DUT's next sample.
    // =================================================================
    integer print_counter = 0;
    integer sub;
    // Hardcoded ax_rm, ay_rm, az_rm
    initial begin    
        az_rm = real_to_sfix16(1.0, 11);
        ay_rm = az_rm;
        ax_rm = ay_rm; 
    end
    always @(posedge clk) begin
        if (reset) begin
            u_st = 0.0; v_st = 0.0; w_st = 0.0;
            p_st = 0.0; q_st = 0.0; r_st = 0.0;
            z_st = 0.0;
            phi_st = 0.0; theta_st = 0.0;
            print_counter = 0;
            gx <= 0; gy <= 0; ax <= 0; ay <= 0; az <= 0;
        end
        else if (uut.clk_enable_250hz) begin

            // ---- A. Sample control inputs ONCE per controller tick ----
            // U2 from DUT: sfix24_En16. Held constant (ZOH) for all
            // substeps below -- the controller itself does not run any
            // faster than this, so this matches reality.
            U2_r = sfix_to_real($signed(U2_out), 16);
            U1   = m * g;   // hover-compensating thrust assumption
            U3   = 0.0;     // no pitch actuation in this design
            U4   = 0.0;     // no yaw actuation in this design

            // ---- B. Substepped nonlinear 6DOF + attitude kinematics ----
            for (sub = 0; sub < SUBSTEPS; sub = sub + 1) begin

                // Derivatives (uses PRE-update states, standard Euler)
                u_dot = 0;
                v_dot = - g*phi_st;
                w_dot = - g*phi_st+(U1/m);

                p_dot =  (U2_r/Ix);
                q_dot = 0;
                r_dot = 0;



                phi_dot   = p_st;
                theta_dot = 0;

                // Forward-Euler substep (blocking, real state vars)
                u_st = u_st + (u_dot * dt_sub);
                v_st = v_st + (v_dot * dt_sub);
                w_st = w_st + (w_dot * dt_sub);
                p_st = p_st + (p_dot * dt_sub);
                q_st = q_st + (q_dot * dt_sub);
                r_st = r_st + (r_dot * dt_sub);
                z_st = z_st + (z_dot * dt_sub);

                phi_st   = phi_st   + (phi_dot   * dt_sub);
                theta_st = theta_st + (theta_dot * dt_sub);
            end

            // ---- C. Synthesize next-cycle IMU data (unchanged) ----
            // Gyroscope: direct angular rate readout. Format sfix16_En10.
            gx <= real_to_sfix16(p_st, 10);
            gy <= real_to_sfix16(q_st, 10);

            // Accelerometer: gravity vector resolved into body axes
            // (standard AHRS/complementary-filter convention). Format
            // sfix16_En11. This is exact here because U1 = m*g always,
            // so body-frame specific force truly is (0,0,g).
            ax <= real_to_sfix16(-g*$sin(theta_st), 11);
            ay <= real_to_sfix16( g*$sin(phi_st)*$cos(theta_st), 11);
            az <= real_to_sfix16( g*$cos(phi_st)*$cos(theta_st), 11);

//            ax_rm <= real_to_sfix16(u_dot - g*$sin(theta_st), 11);
//            ay_rm <= real_to_sfix16(v_dot + g*$sin(phi_st)*$cos(theta_st), 11);
//            az_rm <= real_to_sfix16(w_dot + g*$cos(phi_st)*$cos(theta_st), 11);

            // Yaw stays at true 0 (r == 0 for this roll-axis test, no yaw dynamics)
            Psi <= real_to_sfix16(0.0, 12);

            // ---- D. Diagnostic (TB-side, independent of DUT internals) ----
            // Expected global-frame acceleration IF Acceleration_vectirs
            // correctly reconstructs a_global = R(phi_est,theta_est)*f_body + g_e.
            // Since U1 = m*g exactly, f_body = (0,0,g), so the horizontal
            // global channels should equal the true translational drivers
            // below -- NOT zero. Compare this against whatever internal
            // ax_global/ay_global signal you can hierarchically probe
            // once Acceleration_vectirs.v is available (e.g. via
            // uut.<instance_name>.ax_global if it's exposed):
            //   expected_ax_global ≈ g*sin(theta_st)
            //   expected_ay_global ≈ -g*sin(phi_st)*cos(theta_st)
            // If the DUT's internal signal reads ~0 while these are not,
            // the bug is inside Acceleration_vectirs (likely subtracting
            // a rotated gravity estimate instead of the fixed vertical
            // g_e, or rotating in the wrong sense). If it matches these
            // expected values but Position_Filter's Vest/Yest still don't
            // move, the bug is downstream in Position_Filter's k1/k2/gain
            // wiring instead (less likely -- that source looks numerically
            // sound on inspection).

            // ---- E. Console printing (every 0.1 s = 25 ticks @ 250 Hz) ----
            print_counter = print_counter + 1;
            if (print_counter >= 25) begin
                $display("t=%0.3fs | phi_true=%0.4f phi_est=%0.4f | v_true=%0.4f V_est=%0.4f Y_est=%0.4f | p=%0.3f U2=%0.4f | ay_global_expected = %0.4f | az_rm_expected=%0.4f",
                    $realtime / 1.0e9,
                    phi_st,
                    sfix_to_real($signed(phi_out), 14),
                    v_st,
                    sfix_to_real($signed(uut.Vest_w), 12),
                    sfix_to_real($signed(uut.Yest_w), 12),
                    p_st,
                    U2_r,
                    -g*$sin(phi_st)*$cos(theta_st),
//                    u_dot - g*$sin(theta_st),
//                     g*$sin(phi_st)*$cos(theta_st),
                     g*$cos(phi_st)*$cos(theta_st)
//                    sfix_to_real(ax_rm, 11),
//                    sfix_to_real(ay_rm, 11),
//                    sfix_to_real(az_rm, 11)
                    );
                print_counter = 0;
            end
        end
    end

endmodule


////simplified

//`timescale 1 ns / 1 ns

//module tb_closed_loop_fil();

//    // ===================================================================
//    // 1. Clock and Reset Generation
//    // ===================================================================
//    logic clk;
//    logic reset;
    
//    // 10 MHz Clock = 100 ns period
//    initial begin
//        clk = 0;
//        forever #50 clk = ~clk;
//    end

//    // ===================================================================
//    // 2. DUT Input/Output Signals
//    // ===================================================================
    
//    // IMU and Setpoint Inputs (to DUT)
//    logic signed [15:0] gx, gy;
//    logic signed [15:0] ax, ay, az;
//    logic signed [15:0] Psi;
//    logic signed [15:0] x_setpoint, y_setpoint;
//    logic signed [15:0] vx_ff, vy_ff, ax_ff, ay_ff;
    
//    // Position Controller Gains (to DUT)
//    logic signed [15:0] Kp_vel, Ki_vel, inv_g;
//    logic signed [15:0] Kp_pos, Ki_pos, Kw_pos, v_max;
    
//    // Attitude Controller Gains (to DUT)
//    logic signed [15:0] Ki_rate, Kp_rate, Kd_rate, Kang;
    
//    // DUT Outputs
//    wire signed [23:0] U2_out;
//    wire signed [15:0] X_raw_out, Y_raw_out;
//    wire signed [15:0] phi_out, theta_out;
//    wire signed [15:0] p_filt_out, q_filt_out;

//    // ===================================================================
//    // 3. Plant Setup (Real/Floating-Point Physics Model)
//    // ===================================================================
    
//    // Physical Parameters (from MATLAB)
//    real m  = 0.7;       // Mass (kg)
//    real g  = 9.81;      // Gravity (m/s^2)
//    real Ix = 0.0030;    // Roll Inertia (kg*m^2)
//    real Iy = 0.0033;    // Pitch Inertia (kg*m^2)
//    real Iz = 0.0060;    // Yaw Inertia (kg*m^2)
//    real dt = 0.004;     // 250 Hz integration time step
    
//    // Internal States
//    real u_st=0, v_st=0, w_st=0;
//    real p_st=0, q_st=0, r_st=0;
//    real z_st=0;
    
//    // NEW: True Physical Attitude States
//    real phi_st=0, theta_st=0;
//    real phi_dot, theta_dot;
    
//    // Kinematic rates
//    real u_dot, v_dot, w_dot;
//    real p_dot, q_dot, r_dot;
//    real z_dot;
    
//    // Accelerometer Readings (Specific Force)
//    real ax_real, ay_real, az_real;

//    // System Inputs (Converted from DUT outputs)
//    real U1, U2, U3, U4;
//    real phi_real, theta_real;

//    // ===================================================================
//    // 4. Fixed-Point to Real Conversion Functions
//    // ===================================================================
    
//    function real sfix_to_real(input integer val, input integer frac_bits);
//        sfix_to_real = real'(val) / (2.0 ** frac_bits);
//    endfunction

//    function logic [15:0] real_to_sfix16(input real val, input integer frac_bits);
//        integer scaled;
//        scaled = $rtoi(val * (2.0 ** frac_bits));
//        // Saturation logic to prevent overflow wrapping
//        if (scaled > 32767) scaled = 32767;
//        if (scaled < -32768) scaled = -32768;
//        real_to_sfix16 = scaled[15:0];
//    endfunction

//    // ===================================================================
//    // 5. Instantiate the DUT (Device Under Test)
//    // ===================================================================
//    Attitude_Position_Top uut (
//        .clk(clk),
//        .reset(reset),
//        .gx(gx), .gy(gy),
//        .ax(ax), .ay(ay), .az(az),
//        .Psi(Psi),
//        .x_setpoint(x_setpoint), .y_setpoint(y_setpoint),
//        .vx_ff(vx_ff), .vy_ff(vy_ff),
//        .ax_ff(ax_ff), .ay_ff(ay_ff),
//        .U2(U2_out),
//        .X_raw_out(X_raw_out), .Y_raw_out(Y_raw_out),
//        .phi(phi_out), .theta(theta_out),
//        .p_filt(p_filt_out), .q_filt(q_filt_out),
//        .Kp_vel(Kp_vel), .Ki_vel(Ki_vel), .inv_g(inv_g),
//        .Kp_pos(Kp_pos), .Ki_pos(Ki_pos), .Kw_pos(Kw_pos), .v_max(v_max),
//        .Ki_rate(Ki_rate), .Kp_rate(Kp_rate), .Kd_rate(Kd_rate), .Kang(Kang)
//    );
    


//    // ===================================================================
//    // 6. Test Sequence and Gains Initialization
//    // ===================================================================
//    initial begin
//        // Initialize all inputs to 0
//        gx = 0; gy = 0; ax = 0; ay = 0; az = 0; Psi = 0;
//        x_setpoint = 0; y_setpoint = 0;
//        vx_ff = 0; vy_ff = 0; ax_ff = 0; ay_ff = 0;
        
//        // Let the simulator compute the hex values dynamically!
//        // Assuming En12 format for Position Gains
//        Kp_vel = real_to_sfix16(0.5097, 12);
//        Ki_vel = real_to_sfix16(0.6371, 12);
//        inv_g  = real_to_sfix16(0.1019, 12);
//        Kp_pos = real_to_sfix16(1.0000, 12);
//        Ki_pos = real_to_sfix16(0.2500, 12);
//        Kw_pos = real_to_sfix16(0.5000, 12);
//        v_max  = real_to_sfix16(5.0000, 12);
        
//        // Assuming En12 format for Attitude Gains
//        Ki_rate = real_to_sfix16(4.7250, 12);
//        Kp_rate = real_to_sfix16(0.3381, 12);
//        Kd_rate = real_to_sfix16(0.0065, 12);
//        Kang    = real_to_sfix16(4.7600, 12);

//        // Apply Reset
//        reset = 1;
//        #1000;
//        reset = 0;

//        // Command a Y-setpoint (Roll Axis) after 10ms
//        // Command 0.2 meters in En12 format
//        #10_000_000; 
//        y_setpoint = real_to_sfix16(0.2, 12); 
//        x_setpoint = 16'h0000; 

//        // Run simulation for 80 seconds 
//        repeat (40) begin
//            #2_000_000_000; 
//        end 
        
//        $display("Simulation complete: 80 seconds elapsed.");
//        $finish;
//    end

//    // ===================================================================
//    // 7. Plant Physics Simulation (Triggers at 250Hz / every 40,000 clocks)
//    // ===================================================================
//    integer ce_counter = 0;
//    integer print_counter = 0; // Counter to manage console printing frequency

//    always @(posedge clk) begin
//        if (reset) begin
//            ce_counter <= 0;
//            print_counter <= 0;
//        end else begin
//            if (ce_counter == 39999) begin // 10MHz / 250Hz = 40,000 cycles
//                ce_counter <= 0;

//                // A. Extract Plant Inputs from DUT
//                // U2 is sfix24_En16. Sign extend to 32 bits, then convert.
//                U2 = sfix_to_real($signed({{8{U2_out[23]}}, U2_out}), 16);
                
//                // Assuming hover throttle to balance gravity so the drone doesn't drop
//                U1 = m * g; 
//                U3 = 0.0; // Pitch control assumed zero for this specific roll-focused test
//                U4 = 0.0; // Yaw control assumed zero
                
//                // Get Current Orientation from DUT Filters (sfix16_En14) (Still tracked for printing, but no longer drives plant)
//                phi_real   = sfix_to_real($signed(phi_out), 14);
//                theta_real = sfix_to_real($signed(theta_out), 14);

//                // B. Calculate Translational and Rotational Accelerations
//                // FIX: Use true plant states (phi_st, theta_st) instead of DUT outputs
//                u_dot = 0; //(v_st*r_st - w_st*q_st) + g*$sin(theta_st);
//                v_dot = - g*$sin(phi_st); //(w_st*p_st - u_st*r_st) - g*$sin(phi_st)*$cos(theta_st);
//                w_dot = (- v_st*p_st) - g*$cos(phi_st)+ 9.81;//(u_st*q_st - v_st*p_st) - g*$cos(phi_st)*$cos(theta_st) + (U1/m)+ 9.81;

//                p_dot = (U2/Ix);//((Iy - Iz)/Ix)*q_st*r_st + (U2/Ix);
//                q_dot = 0; // ((Iz - Ix)/Iy)*p_st*r_st + (U3/Iy);
//                r_dot = 0;// ((Ix - Iy)/Iz)*p_st*q_st + (U4/Iz);

//                z_dot = 0;//-u_st*$sin(theta_st) + v_st*$sin(phi_st)*$cos(theta_st) + w_st*$cos(phi_st)*$cos(theta_st);

//                // NEW: Calculate true attitude rates (simplified kinematics for hover)
//                phi_dot = p_st + (q_st*$sin(phi_st) + r_st*$cos(phi_st))*$tan(theta_st);
//                theta_dot = q_st*$cos(phi_st) - r_st*$sin(phi_st);
//// phi dot = p
//                // C. Forward Euler Integration
//                u_st = u_st + (u_dot * dt);
//                v_st = v_st + (v_dot * dt);
//                w_st = w_st + (w_dot * dt);
//                p_st = p_st + (p_dot * dt);
//                q_st = q_st + (q_dot * dt);
//                r_st = r_st + (r_dot * dt);
//                z_st = z_st + (z_dot * dt);
                
//                // NEW: Integrate true attitude
//                phi_st = phi_st + (phi_dot * dt);
//                theta_st = theta_st + (theta_dot * dt);

//                // D. Convert Physics States to IMU Sensor Data for Next Cycle
//                // Gyroscope measures angular rates directly (p, q, r). Format: sfix16_En10
//                gx <= real_to_sfix16(p_st, 10);
//                gy <= real_to_sfix16(q_st, 10);
                
//                // Accelerometer measures Specific Force
//                // FIX: Use phi_st and theta_st
//                ax_real = u_dot ;
//                ay_real = v_dot ; 
//                az_real = w_dot ; 
                
//                // Convert Accelerometer specific force to sfix16_En11
//                ax <= real_to_sfix16(ax_real, 11);
//                ay <= real_to_sfix16(ay_real, 11);
//                az <= real_to_sfix16(az_real, 11);
               

//                // E. Console Printing
//                print_counter = print_counter + 1;
//                if (print_counter >= 25) begin // Print every 0.1 seconds
//                    // Print physical quantities by converting the fixed-point wires back to real numbers
//                    $display("Time: %0.3f s |IMU[gx:%0.3f|gy:%0.3f|ay:%0.3f|az:%0.3f]|ay_global:%0.3f|RAW_POS[Y:%0.3f] | EST_POS[V:%0.3f|Y:%0.3f] | ATT[phi:%0.3f] | CTRL[U2:%0.3f]", 
//                        $realtime / 1_000_000_000.0, 
//                        sfix_to_real($signed(gx), 10), 
//                        sfix_to_real($signed(gy), 10), 
//                        //sfix_to_real($signed(ax), 11), 
//                        sfix_to_real($signed(ay), 11), 
//                        sfix_to_real($signed(az), 11), 
//                        sfix_to_real($signed(uut.ay_global_w), 11),
//                        //sfix_to_real($signed(X_raw_out), 12), 
//                        sfix_to_real($signed(Y_raw_out), 12),
//                        //sfix_to_real($signed(uut.Uest_w), 12), 
//                        sfix_to_real($signed(uut.Vest_w), 12), 
//                        //sfix_to_real($signed(uut.Xest_w), 12), 
//                        sfix_to_real($signed(uut.Yest_w), 12), 
//                        phi_real, // Reusing the already calculated real variables to monitor filter accuracy
//                        U2);
                    
//                    print_counter = 0; // Reset counter
//                end

//            end else begin
//                ce_counter <= ce_counter + 1;
//            end
//        end
//    end

//endmodule


//////original

//`timescale 1 ns / 1 ns

//module tb_closed_loop_fil();

//    // ===================================================================
//    // 1. Clock and Reset Generation
//    // ===================================================================
//    logic clk;
//    logic reset;
    
//    // 10 MHz Clock = 100 ns period
//    initial begin
//        clk = 0;
//        forever #50 clk = ~clk;
//    end

//    // ===================================================================
//    // 2. DUT Input/Output Signals
//    // ===================================================================
    
//    // IMU and Setpoint Inputs (to DUT)
//    logic signed [15:0] gx, gy;
//    logic signed [15:0] ax, ay, az;
//    logic signed [15:0] Psi;
//    logic signed [15:0] x_setpoint, y_setpoint;
//    logic signed [15:0] vx_ff, vy_ff, ax_ff, ay_ff;
    
//    // Position Controller Gains (to DUT)
//    logic signed [15:0] Kp_vel, Ki_vel, inv_g;
//    logic signed [15:0] Kp_pos, Ki_pos, Kw_pos, v_max;
    
//    // Attitude Controller Gains (to DUT)
//    logic signed [15:0] Ki_rate, Kp_rate, Kd_rate, Kang;
    
//    // DUT Outputs
//    wire signed [23:0] U2_out;
//    wire signed [15:0] X_raw_out, Y_raw_out;
//    wire signed [15:0] phi_out, theta_out;
//    wire signed [15:0] p_filt_out, q_filt_out;

//    // ===================================================================
//    // 3. Plant Setup (Real/Floating-Point Physics Model)
//    // ===================================================================
    
//    // Physical Parameters (from MATLAB)
//    real m  = 0.7;       // Mass (kg)
//    real g  = 9.81;      // Gravity (m/s^2)
//    real Ix = 0.0030;    // Roll Inertia (kg*m^2)
//    real Iy = 0.0033;    // Pitch Inertia (kg*m^2)
//    real Iz = 0.0060;    // Yaw Inertia (kg*m^2)
//    real dt = 0.004;     // 250 Hz integration time step
    
//    // Internal States
//    real u_st=0, v_st=0, w_st=0;
//    real p_st=0, q_st=0, r_st=0;
//    real z_st=0;
    
//    // Kinematic rates
//    real u_dot, v_dot, w_dot;
//    real p_dot, q_dot, r_dot;
//    real z_dot;
    
//    // Accelerometer Readings (Specific Force)
//    real ax_real, ay_real, az_real;

//    // System Inputs (Converted from DUT outputs)
//    real U1, U2, U3, U4;
//    real phi_real, theta_real;

//    // ===================================================================
//    // 4. Fixed-Point to Real Conversion Functions
//    // ===================================================================
    
//    function real sfix_to_real(input integer val, input integer frac_bits);
//        sfix_to_real = real'(val) / (2.0 ** frac_bits);
//    endfunction

//    function logic [15:0] real_to_sfix16(input real val, input integer frac_bits);
//        integer scaled;
//        scaled = $rtoi(val * (2.0 ** frac_bits));
//        // Saturation logic to prevent overflow wrapping
//        if (scaled > 32767) scaled = 32767;
//        if (scaled < -32768) scaled = -32768;
//        real_to_sfix16 = scaled[15:0];
//    endfunction

//    // ===================================================================
//    // 5. Instantiate the DUT (Device Under Test)
//    // ===================================================================
//    Attitude_Position_Top uut (
//        .clk(clk),
//        .reset(reset),
//        .gx(gx), .gy(gy),
//        .ax(ax), .ay(ay), .az(az),
//        .Psi(Psi),
//        .x_setpoint(x_setpoint), .y_setpoint(y_setpoint),
//        .vx_ff(vx_ff), .vy_ff(vy_ff),
//        .ax_ff(ax_ff), .ay_ff(ay_ff),
//        .U2(U2_out),
//        .X_raw_out(X_raw_out), .Y_raw_out(Y_raw_out),
//        .phi(phi_out), .theta(theta_out),
//        .p_filt(p_filt_out), .q_filt(q_filt_out),
//        .Kp_vel(Kp_vel), .Ki_vel(Ki_vel), .inv_g(inv_g),
//        .Kp_pos(Kp_pos), .Ki_pos(Ki_pos), .Kw_pos(Kw_pos), .v_max(v_max),
//        .Ki_rate(Ki_rate), .Kp_rate(Kp_rate), .Kd_rate(Kd_rate), .Kang(Kang)
//    );

//    // ===================================================================
//    // 6. Test Sequence and Gains Initialization
//    // ===================================================================
//    initial begin
//        // Initialize all inputs to 0
//        gx = 0; gy = 0; ax = 0; ay = 0; az = 0; Psi = 0;
//        x_setpoint = 0; y_setpoint = 0;
//        vx_ff = 0; vy_ff = 0; ax_ff = 0; ay_ff = 0;
        
//        // Let the simulator compute the hex values dynamically!
//        // Assuming En12 format for Position Gains
//        Kp_vel = real_to_sfix16(0.5097, 12);
//        Ki_vel = real_to_sfix16(0.6371, 12);
//        inv_g  = real_to_sfix16(0.1019, 12);
//        Kp_pos = real_to_sfix16(1.0000, 12);
//        Ki_pos = real_to_sfix16(0.2500, 12);
//        Kw_pos = real_to_sfix16(0.5000, 12);
//        v_max  = real_to_sfix16(5.0000, 12);
        
//        // Assuming En12 format for Attitude Gains
//        Ki_rate = real_to_sfix16(4.7250, 12);
//        Kp_rate = real_to_sfix16(0.3381, 12);
//        Kd_rate = real_to_sfix16(0.0065, 12);
//        Kang    = real_to_sfix16(4.7600, 12);

//        // Apply Reset
//        reset = 1;
//        #1000;
//        reset = 0;

//        // Command a Y-setpoint (Roll Axis) after 10ms
//        // Command 5.0 meters in En12 format
//        #10_000_000; 
//        y_setpoint = real_to_sfix16(0.2, 12); 
//        x_setpoint = 16'h0000; 

//        // Run simulation for 80 seconds 
//        repeat (40) begin
//            #2_000_000_000; 
//        end 
        
//        $display("Simulation complete: 80 seconds elapsed.");
//        $finish;
//    end

//    // ===================================================================
//    // 7. Plant Physics Simulation (Triggers at 250Hz / every 40,000 clocks)
//    // ===================================================================
//    integer ce_counter = 0;
//    integer print_counter = 0; // Counter to manage console printing frequency

//    always @(posedge clk) begin
//        if (reset) begin
//            ce_counter <= 0;
//            print_counter <= 0;
//        end else begin
//            if (ce_counter == 39999) begin // 10MHz / 250Hz = 40,000 cycles
//                ce_counter <= 0;

//                // A. Extract Plant Inputs from DUT
//                // U2 is sfix24_En16. Sign extend to 32 bits, then convert.
//                U2 = sfix_to_real($signed({{8{U2_out[23]}}, U2_out}), 16);
                
//                // Assuming hover throttle to balance gravity so the drone doesn't drop
//                U1 = m * g; 
//                U3 = 0.0; // Pitch control assumed zero for this specific roll-focused test
//                U4 = 0.0; // Yaw control assumed zero
                
//                // Get Current Orientation from DUT Filters (sfix16_En14)
//                phi_real   = sfix_to_real($signed(phi_out), 14);
//                theta_real = sfix_to_real($signed(theta_out), 14);

//                // B. Calculate Translational and Rotational Accelerations (MATLAB Plant)
//                u_dot = (v_st*r_st - w_st*q_st) + g*$sin(theta_real);
//                v_dot = (w_st*p_st - u_st*r_st) - g*$sin(phi_real)*$cos(theta_real);
//                w_dot = (u_st*q_st - v_st*p_st) - g*$cos(phi_real)*$cos(theta_real) + (U1/m);

//                p_dot = ((Iy - Iz)/Ix)*q_st*r_st + (U2/Ix);
//                q_dot = ((Iz - Ix)/Iy)*p_st*r_st + (U3/Iy);
//                r_dot = ((Ix - Iy)/Iz)*p_st*q_st + (U4/Iz);

//                z_dot = -u_st*$sin(theta_real) + v_st*$sin(phi_real)*$cos(theta_real) + w_st*$cos(phi_real)*$cos(theta_real);

//                // C. Forward Euler Integration
//                u_st = u_st + (u_dot * dt);
//                v_st = v_st + (v_dot * dt);
//                w_st = w_st + (w_dot * dt);
//                p_st = p_st + (p_dot * dt);
//                q_st = q_st + (q_dot * dt);
//                r_st = r_st + (r_dot * dt);
//                z_st = z_st + (z_dot * dt);

//                // D. Convert Physics States to IMU Sensor Data for Next Cycle
//                // Gyroscope measures angular rates directly (p, q, r). Format: sfix16_En10
//                gx <= real_to_sfix16(p_st, 10);
//                gy <= real_to_sfix16(q_st, 10);
                
//                // Accelerometer measures Specific Force: a_imu = dv/dt + (w x v) - g_vector
//                ax_real = u_dot - (v_st*r_st - w_st*q_st); // evaluates to g*sin(theta)
//                ay_real = v_dot - (w_st*p_st - u_st*r_st); // evaluates to -g*sin(phi)*cos(theta)
//                az_real = w_dot - (u_st*q_st - v_st*p_st); // evaluates to -g*cos(phi)*cos(theta) + U1/m
                
//                // Convert Accelerometer specific force to sfix16_En11
//                ax <= real_to_sfix16(ax_real, 11);
//                ay <= real_to_sfix16(ay_real, 11);
//                az <= real_to_sfix16(az_real, 11);

//                // E. Console Printing
//                print_counter = print_counter + 1;
//                if (print_counter >= 25) begin // Print every 0.1 seconds
//                    // Print physical quantities by converting the fixed-point wires back to real numbers
//                    $display("Time: %0.3f s | IMU[gx:%0.3f gy:%0.3f ax:%0.3f ay:%0.3f az:%0.3f] | RAW_POS[X:%0.3f Y:%0.3f] | EST_POS[U:%0.3f V:%0.3f X:%0.3f Y:%0.3f] | ATT[phi:%0.3f] | CTRL[U2:%0.3f]", 
//                        $realtime / 1_000_000_000.0, 
//                        sfix_to_real($signed(gx), 10), 
//                        sfix_to_real($signed(gy), 10), 
//                        sfix_to_real($signed(ax), 11), 
//                        sfix_to_real($signed(ay), 11), 
//                        sfix_to_real($signed(az), 11), 
//                        sfix_to_real($signed(X_raw_out), 12), 
//                        sfix_to_real($signed(Y_raw_out), 12),
//                        sfix_to_real($signed(uut.Uest_w), 12), 
//                        sfix_to_real($signed(uut.Vest_w), 12), 
//                        sfix_to_real($signed(uut.Xest_w), 12), 
//                        sfix_to_real($signed(uut.Yest_w), 12), 
//                        phi_real, // Reusing the already calculated real variables
//                        U2);
                    
//                    print_counter = 0; // Reset counter
//                end

//            end else begin
//                ce_counter <= ce_counter + 1;
//            end
//        end
//    end

//endmodule