`timescale 1ns / 1ps

module tb_cascaded_top;

//==========================================================
// CLOCK / TIMING
//==========================================================

parameter integer CLK_FREQ      = 10_000_000;
parameter time    CLK_PERIOD    = 1_000_000_000 / CLK_FREQ;
parameter time    CLK_HIGH_TIME = CLK_PERIOD / 2;
parameter time    RESET_TIME    = 4_000_000;

parameter integer HIST_LEN      = 1024;


//==========================================================
// FRAME CONVENTION
//==========================================================
//
// INERTIAL FRAME:
//     Z-UP INERTIAL FRAME
//
//     X = North
//     Y = East
//     Z = Up
//
// BODY FRAME:
//
//     u = forward
//     v = right
//     w = up
//
// EULER:
//
//     Z-Y-X
//
//     phi   = roll
//     theta = pitch
//     psi   = yaw
//
// All real-valued angles are radians.
//
//==========================================================


//==========================================================
// PLANT PARAMETERS
//==========================================================
//
// U1 is force along +body-z.
//
// Since body-z is UP:
//
//     U1 = +m*g  -> level hover
//
// U2 = roll torque
// U3 = pitch torque
// U4 = yaw torque
//
//==========================================================

parameter real PLANT_MASS = 0.8;
parameter real PLANT_IX   = 0.003;
parameter real PLANT_IY   = 0.0033;
parameter real PLANT_IZ   = 0.006;


//==========================================================
// CLOCK / RESET
//==========================================================

reg clk;
reg reset;


//==========================================================
// LOGGING
//==========================================================

integer log_file;


//==========================================================
// LOG MACROS
//==========================================================

`define LOG0(fmt) \
begin \
    $display(fmt); \
    $fdisplay(log_file,fmt); \
end

`define LOG1(fmt,a1) \
begin \
    $display(fmt,a1); \
    $fdisplay(log_file,fmt,a1); \
end

`define LOG2(fmt,a1,a2) \
begin \
    $display(fmt,a1,a2); \
    $fdisplay(log_file,fmt,a1,a2); \
end

`define LOG3(fmt,a1,a2,a3) \
begin \
    $display(fmt,a1,a2,a3); \
    $fdisplay(log_file,fmt,a1,a2,a3); \
end

`define LOG4(fmt,a1,a2,a3,a4) \
begin \
    $display(fmt,a1,a2,a3,a4); \
    $fdisplay(log_file,fmt,a1,a2,a3,a4); \
end

`define LOG5(fmt,a1,a2,a3,a4,a5) \
begin \
    $display(fmt,a1,a2,a3,a4,a5); \
    $fdisplay(log_file,fmt,a1,a2,a3,a4,a5); \
end

`define LOG6(fmt,a1,a2,a3,a4,a5,a6) \
begin \
    $display(fmt,a1,a2,a3,a4,a5,a6); \
    $fdisplay(log_file,fmt,a1,a2,a3,a4,a5,a6); \
end


//==========================================================
// DUT INPUTS
//==========================================================

reg clk_enable;


//----------------------------------------------------------
// Gyroscope
//----------------------------------------------------------

reg signed [15:0] gx;
reg signed [15:0] gy;
reg signed [15:0] gz;


//----------------------------------------------------------
// Physical accelerometer:
// SPECIFIC FORCE
//----------------------------------------------------------

reg signed [15:0] ax;
reg signed [15:0] ay;
reg signed [15:0] az;


//----------------------------------------------------------
// Acceleration-vector inputs
//
// These are body-frame linear accelerations:
//
//     a_b = dv_body/dt + omega x v_body
//
//==========================================================

reg signed [15:0] ax_rm;
reg signed [15:0] ay_rm;
reg signed [15:0] az_rm;


//----------------------------------------------------------
// dt
//----------------------------------------------------------

reg [11:0] dt;


//----------------------------------------------------------
// Complementary-filter coefficients
//----------------------------------------------------------

reg signed [15:0] Alpha_2;
reg signed [15:0] alpha1_Alpha_2;

reg signed [15:0] Alpha_1;
reg signed [15:0] alpha1_Alpha_1;


//==========================================================
// DUT OUTPUTS
//==========================================================

wire ce_out;

wire signed [15:0] ax_global;
wire signed [15:0] ay_global;

wire signed [15:0] Xest;
wire signed [15:0] Yest;

wire signed [15:0] Uest;
wire signed [15:0] Vest;

wire signed [15:0] p_filt_dbg_en10;
wire signed [15:0] q_filt_dbg_en10;
wire signed [15:0] r_filt_dbg_en10;

wire signed [15:0] phi_filt_dbg_en14;
wire signed [15:0] theta_filt_dbg_en9;
wire signed [15:0] psi_filt_dbg_en12;

wire signed [15:0] theta_cmd;
wire signed [31:0] phi_cmd;

wire signed [15:0] U2;


//==========================================================
// DUT SETPOINTS
//==========================================================

wire signed [15:0] x_setpoint;
wire signed [15:0] y_setpoint;


//==========================================================
// REFERENCE TIMING
//==========================================================

real Ts;
real Ts_dut;
real Ts_position_rtl;
real Ts_controller;
real Ts_rate;


//==========================================================
// PHYSICAL CONSTANTS
//==========================================================

real g;
real alpha;


//==========================================================
// POSITION CONTROLLER GAINS
//==========================================================

real Kp_pos;
real Ki_pos;
real Kw_pos;

real v_max;

real Kp_vel;
real Ki_vel;

real inv_g;


//==========================================================
// ATTITUDE CONTROLLER GAINS
//==========================================================

real Kp_attitude;
real Ki_attitude;


//==========================================================
// RATE CONTROLLER GAINS
//==========================================================

real Kp_rate;
real Ki_rate;
real Kd_rate;


//==========================================================
// POSITION COMMAND
//==========================================================

real X_set;
real Y_set;


//==========================================================
// TRUE PHYSICAL PLANT STATES
//==========================================================

real u_ref;
real v_ref;
real w_ref;

real p_ref;
real q_ref;
real r_ref;

real phi_ref;
real theta_ref;
real psi_ref;

real x_ref;
real y_ref;
real z_ref;


//==========================================================
// PLANT DERIVATIVES
//==========================================================

real u_dot_ref;
real v_dot_ref;
real w_dot_ref;

real p_dot_plant_ref;
real q_dot_plant_ref;
real r_dot_plant_ref;

real phi_dot_ref;
real theta_dot_ref;
real psi_dot_ref;

real x_dot_ref;
real y_dot_ref;
real z_dot_ref;


//==========================================================
// BODY-FRAME TRUE LINEAR ACCELERATION
//==========================================================

real a_bx_ref;
real a_by_ref;
real a_bz_ref;


//==========================================================
// CURRENTLY APPLIED PHYSICAL PLANT INPUTS
//==========================================================

real U1_plant;
real U2_plant;
real U3_plant;
real U4_plant;


//==========================================================
// MATHEMATICAL REFERENCE CONTROLLER OUTPUTS
//==========================================================

real u2_ref;
real u3_ref;


//==========================================================
// ACCELEROMETER SPECIFIC FORCE
//==========================================================

real fx_ref;
real fy_ref;
real fz_ref;


//==========================================================
// TRUE INERTIAL ACCELERATION
//==========================================================

real axg_ref;
real ayg_ref;
real azg_ref;


//==========================================================
// BODY -> Z-UP INERTIAL ROTATION MATRIX
//==========================================================

real R11_ref;
real R12_ref;
real R13_ref;

real R21_ref;
real R22_ref;
real R23_ref;

real R31_ref;
real R32_ref;
real R33_ref;


//==========================================================
// TRUE INERTIAL POSITION / VELOCITY
//==========================================================

real x_raw;
real y_raw;

real vx_raw;
real vy_raw;


//==========================================================
// COMPLEMENTARY FILTER REFERENCE
//==========================================================

real phi_pred_ref;
real theta_pred_ref;

real phi_acc_ref;
real theta_acc_ref;

real phi_cf_ref;
real theta_cf_ref;
real psi_cf_ref;

real phi0_ref;


//==========================================================
// ATTITUDE FILTER HISTORY
//==========================================================

real gyro_sum_ref;
real acc_sum_ref;

real phi_formula_ref;

real phi_dot_hist [0:HIST_LEN-1];
real phi_acc_hist [0:HIST_LEN-1];


//==========================================================
// MATLAB POSITION OBSERVER
//==========================================================

real K1_X_MATLAB;
real K1_Y_MATLAB;
real K2_MATLAB;

real ex_ref;
real ey_ref;

real vx_hat_ref;
real vy_hat_ref;

real x_hat_ref;
real y_hat_ref;


//==========================================================
// RTL-TIMED POSITION OBSERVER REFERENCE
//==========================================================

real K1_X_RTL;
real K1_Y_RTL;
real K2_RTL;

real vx_hat_ref_rtl;
real vy_hat_ref_rtl;

real x_hat_ref_rtl;
real y_hat_ref_rtl;

real ex_ref_rtl;
real ey_ref_rtl;

real x_delay_ref_rtl;
real y_delay_ref_rtl;

real vx_prev_rtl;
real vy_prev_rtl;


//==========================================================
// ACTUAL DUT POSITION-FILTER RAW INPUTS
//
// These are the exact Position_Filter2 X_raw/Y_raw inputs.
//==========================================================

real X_raw_dut;
real Y_raw_dut;

real X_raw_dut_error;
real Y_raw_dut_error;


//==========================================================
// DUT-STATE CONTROLLER REFERENCE
//==========================================================

real Xest_ctrl_dut;
real Yest_ctrl_dut;

real Uest_ctrl_dut;
real Vest_ctrl_dut;

real phi_ctrl_dut;
real theta_ctrl_dut;
real psi_ctrl_dut;

real ex_pos_dut;
real ey_pos_dut;

real pos_int_x_dut;
real pos_int_y_dut;

real Vx_cmd_dut;
real Vy_cmd_dut;

real Vx_cmd_dut_unsat;
real Vy_cmd_dut_unsat;

real evx_dut;
real evy_dut;
real evz_dut;

real eu_dut;
real ev_dut;

real vel_int_u_dut;
real vel_int_v_dut;

real theta_cmd_ref_dutstate;
real phi_cmd_ref_dutstate;


//==========================================================
// TB-STATE CONTROLLER REFERENCE
//==========================================================

real Xest_ctrl_tb;
real Yest_ctrl_tb;

real Uest_ctrl_tb;
real Vest_ctrl_tb;

real phi_ctrl_tb;
real theta_ctrl_tb;
real psi_ctrl_tb;

real ex_pos_tb;
real ey_pos_tb;

real pos_int_x_tb;
real pos_int_y_tb;

real Vx_cmd_tb;
real Vy_cmd_tb;

real Vx_cmd_tb_unsat;
real Vy_cmd_tb_unsat;

real evx_tb;
real evy_tb;
real evz_tb;

real eu_tb;
real ev_tb;

real vel_int_u_tb;
real vel_int_v_tb;

real theta_cmd_ref_tbstates;
real phi_cmd_ref_tbstates;


//==========================================================
// ATTITUDE -> RATE REFERENCE
//==========================================================

real e_theta_att_ref;
real e_phi_att_ref;

real att_int_theta_ref;
real att_int_phi_ref;

real p_cmd_ref;
real q_cmd_ref;


//==========================================================
// RATE CONTROLLER REFERENCE
//==========================================================

real p_meas_ref;
real q_meas_ref;

real p_prev_ref;
real q_prev_ref;

real p_dot_ref;
real q_dot_ref;

real e_p_ref;
real e_q_ref;

real rate_int_p_ref;
real rate_int_q_ref;


//==========================================================
// DUT U2 COMPARISON
//==========================================================

real u2_dut_real;
real u2_ref_error;


//==========================================================
// COMPLETE U2 DIAGNOSTIC STATE
//==========================================================

real diag_p_prev;
real diag_rate_int;
real diag_rate_int_1dly;

integer diag_initialized;

real diag_phi_dut;
real diag_phi_cmd_dut;

real diag_phi_error;
real diag_p_cmd;

real diag_p_meas;
real diag_p_dot;

real diag_ep;
real diag_ep_1dly;

real diag_u2_dutstate;
real diag_u2_dutstate_1dly;

real diag_P;
real diag_I;
real diag_D;

real diag_U2_before_sat;
real diag_U2_sat;
real diag_u2_actual;

real diag_u2_tb_ref;

real diag_err_dut_vs_tb;
real diag_err_dut_vs_dutstate;
real diag_err_dut_vs_dutstate_1dly;


//==========================================================
// COUNTERS
//==========================================================

integer sample_idx;
integer pulse_count;
integer j;


//==========================================================
// FIXED-POINT HELPERS
//==========================================================

function automatic real sfix_to_real;

    input signed [15:0] sfix_val;
    input integer frac_bits;

    begin

        sfix_to_real =
            $itor(sfix_val) /
            (2.0 ** frac_bits);

    end

endfunction


function automatic real sfix32_to_real;

    input signed [31:0] sfix_val;
    input integer frac_bits;

    begin

        sfix32_to_real =
            $itor(sfix_val) /
            (2.0 ** frac_bits);

    end

endfunction


function automatic signed [15:0] real_to_sfix;

    input real real_val;
    input integer frac_bits;

    real scaled;

    begin

        scaled =
            real_val *
            (2.0 ** frac_bits);

        if (scaled > 32767.0)

            real_to_sfix =
                16'sh7FFF;

        else if (scaled < -32768.0)

            real_to_sfix =
                16'sh8000;

        else

            real_to_sfix =
                $rtoi(scaled);

    end

endfunction


//==========================================================
// SETPOINT FIXED-POINT CONVERSION
//==========================================================

assign x_setpoint =
    real_to_sfix(
        X_set,
        10
    );

assign y_setpoint =
    real_to_sfix(
        Y_set,
        10
    );


//==========================================================
// CLOCK
//==========================================================

initial begin

    clk = 1'b0;

end


always #(CLK_HIGH_TIME)
    clk = ~clk;


//==========================================================
// DUT
//==========================================================

Attitude_Position_Top uut (

    .clk(clk),
    .reset(reset),

    .gx(gx),
    .gy(gy),
    .gz(gz),

    .ax(ax),
    .ay(ay),
    .az(az),

    .ax_rm(ax_rm),
    .ay_rm(ay_rm),
    .az_rm(az_rm),

    .dt(dt),

    .Alpha_2(Alpha_2),
    .alpha1_Alpha_2(alpha1_Alpha_2),
    .Alpha_1(Alpha_1),
    .alpha1_Alpha_1(alpha1_Alpha_1),

    .x_setpoint(x_setpoint),
    .y_setpoint(y_setpoint),

    .ce_out(ce_out),

    .ax_global(ax_global),
    .ay_global(ay_global),

    .Xest(Xest),
    .Yest(Yest),

    .Uest(Uest),
    .Vest(Vest),

    .p_filt_dbg_en10(p_filt_dbg_en10),
    .q_filt_dbg_en10(q_filt_dbg_en10),
    .r_filt_dbg_en10(r_filt_dbg_en10),

    .phi_filt_dbg_en14(phi_filt_dbg_en14),
    .theta_filt_dbg_en9(theta_filt_dbg_en9),
    .psi_filt_dbg_en12(psi_filt_dbg_en12),

    .theta_cmd(theta_cmd),
    .phi_cmd(phi_cmd),

    .U2(U2)

);


//==========================================================
// INITIALIZATION
//==========================================================

initial begin

    log_file =
        $fopen(
            "cascaded_debug.log",
            "w"
        );

    if (log_file == 0) begin

        $display(
            "ERROR: Cannot open cascaded_debug.log"
        );

        $finish;

    end


    `LOG0("")
    `LOG0("============================================================")
    `LOG0("                 CASCADED SIMULATION LOG")
    `LOG0("============================================================")
    `LOG0("")


    reset =
        1'b1;

    clk_enable =
        1'b1;


    Ts =
        0.004;

    g =
        9.81;

    alpha =
        0.992;


    Kp_pos =
        1.0;

    Ki_pos =
        0.25;

    Kw_pos =
        0.5;

    v_max =
        5.0;

    Kp_vel =
        0.5097;

    Ki_vel =
        0.6371;

    inv_g =
        1.0 / g;


    Kp_attitude =
        4.76;

    Ki_attitude =
        0.0;


    Kp_rate =
        0.338;

    Ki_rate =
        4.725;

    Kd_rate =
        0.0066;


    K1_X_MATLAB =
        2.25;

    K1_Y_MATLAB =
        2.25;

    K2_MATLAB =
        3.0;

    K1_X_RTL =
        2.25;

    K1_Y_RTL =
        2.25;

    K2_RTL =
        3.0;


    reset_plant_ref();

    reset_controller_ref();


    phi_cf_ref =
        0.0;

    theta_cf_ref =
        0.0;

    psi_cf_ref =
        0.0;

    phi0_ref =
        0.0;


    sample_idx =
        0;

    pulse_count =
        0;


    diag_p_prev =
        0.0;

    diag_rate_int =
        0.0;

    diag_rate_int_1dly =
        0.0;

    diag_initialized =
        0;


    X_raw_dut =
        0.0;

    Y_raw_dut =
        0.0;

    X_raw_dut_error =
        0.0;

    Y_raw_dut_error =
        0.0;


    gx = 16'sd0;
    gy = 16'sd0;
    gz = 16'sd0;

    ax = 16'sd0;
    ay = 16'sd0;
    az = 16'sd0;

    ax_rm = 16'sd0;
    ay_rm = 16'sd0;
    az_rm = 16'sd0;


    dt =
        real_to_sfix(
            Ts,
            12
        );


    Alpha_1 =
        real_to_sfix(
            0.992,
            14
        );

    alpha1_Alpha_1 =
        real_to_sfix(
            1.0 - 0.992,
            13
        );

    Alpha_2 =
        real_to_sfix(
            1.0,
            10
        );

    alpha1_Alpha_2 =
        real_to_sfix(
            0.0,
            10
        );


    Ts_dut =
        $itor(dt) /
        4096.0;

    // Exact generated Position_Filter2 integrator sample time:
    // 131 / 32768 = 0.003997802734375 s
    Ts_position_rtl =
        131.0 /
        32768.0;

    Ts_controller =
        Ts;

    Ts_rate =
        Ts;


    for (
        j = 0;
        j < HIST_LEN;
        j = j + 1
    ) begin

        phi_dot_hist[j] =
            0.0;

        phi_acc_hist[j] =
            0.0;

    end


    #RESET_TIME;

    reset =
        1'b0;


    `LOG0("")

    `LOG1(
        "RESET RELEASED at %0t",
        $time
    )

    `LOG1(
        "Reference Ts       = %0.9f",
        Ts
    )

    `LOG1(
        "DUT represented dt = %0.9f",
        Ts_dut
    )

    `LOG1(
        "RTL position Ts    = %0.12f",
        Ts_position_rtl
    )

    `LOG0("")

end


//==========================================================
// MAIN STIMULUS
//==========================================================

initial begin

    @(negedge reset);


    run_position_command(
        0.0,
        0.5,
        1000
    );


    run_position_command(
        0.0,
        0.0,
        1000
    );


    `LOG0("")
    `LOG0("============================================================")
    `LOG0(" ALL POSITION COMMANDS COMPLETE")
    `LOG0("============================================================")

    $fflush(
        log_file
    );

    $fclose(
        log_file
    );

    $finish;

end


//==========================================================
// SAFETY TIMEOUT
//==========================================================

initial begin

    #2_000_000_000;
    #2_000_000_000;
    #2_000_000_000;
    #2_000_000_000;
    #2_000_000_000;

    `LOG0("")
    `LOG0("============================================================")
    `LOG0(" SIMULATION TIMEOUT")
    `LOG0("============================================================")

    $fflush(
        log_file
    );

    $fclose(
        log_file
    );

    $finish;

end


//==========================================================
// RESET PHYSICAL PLANT
//==========================================================

task automatic reset_plant_ref;

begin

    u_ref = 0.0;
    v_ref = 0.0;
    w_ref = 0.0;

    p_ref = 0.0;
    q_ref = 0.0;
    r_ref = 0.0;

    phi_ref   = 0.0;
    theta_ref = 0.0;
    psi_ref   = 0.0;

    x_ref = 0.0;
    y_ref = 0.0;
    z_ref = 0.0;

    u_dot_ref = 0.0;
    v_dot_ref = 0.0;
    w_dot_ref = 0.0;

    p_dot_plant_ref = 0.0;
    q_dot_plant_ref = 0.0;
    r_dot_plant_ref = 0.0;

    phi_dot_ref   = 0.0;
    theta_dot_ref = 0.0;
    psi_dot_ref   = 0.0;

    x_dot_ref = 0.0;
    y_dot_ref = 0.0;
    z_dot_ref = 0.0;

    a_bx_ref = 0.0;
    a_by_ref = 0.0;
    a_bz_ref = 0.0;

    U1_plant =
        +PLANT_MASS * g;

    U2_plant =
        0.0;

    U3_plant =
        0.0;

    U4_plant =
        0.0;

    x_raw = 0.0;
    y_raw = 0.0;

    vx_raw = 0.0;
    vy_raw = 0.0;

end

endtask


//==========================================================
// RESET CONTROLLER / OBSERVER
//==========================================================

task automatic reset_controller_ref;

begin

    X_set = 0.0;
    Y_set = 0.0;

    Xest_ctrl_dut = 0.0;
    Yest_ctrl_dut = 0.0;
    Uest_ctrl_dut = 0.0;
    Vest_ctrl_dut = 0.0;

    phi_ctrl_dut   = 0.0;
    theta_ctrl_dut = 0.0;
    psi_ctrl_dut   = 0.0;

    ex_pos_dut = 0.0;
    ey_pos_dut = 0.0;

    pos_int_x_dut = 0.0;
    pos_int_y_dut = 0.0;

    Vx_cmd_dut = 0.0;
    Vy_cmd_dut = 0.0;

    Vx_cmd_dut_unsat = 0.0;
    Vy_cmd_dut_unsat = 0.0;

    evx_dut = 0.0;
    evy_dut = 0.0;
    evz_dut = 0.0;

    eu_dut = 0.0;
    ev_dut = 0.0;

    vel_int_u_dut = 0.0;
    vel_int_v_dut = 0.0;

    theta_cmd_ref_dutstate = 0.0;
    phi_cmd_ref_dutstate   = 0.0;


    Xest_ctrl_tb = 0.0;
    Yest_ctrl_tb = 0.0;
    Uest_ctrl_tb = 0.0;
    Vest_ctrl_tb = 0.0;

    phi_ctrl_tb   = 0.0;
    theta_ctrl_tb = 0.0;
    psi_ctrl_tb   = 0.0;

    ex_pos_tb = 0.0;
    ey_pos_tb = 0.0;

    pos_int_x_tb = 0.0;
    pos_int_y_tb = 0.0;

    Vx_cmd_tb = 0.0;
    Vy_cmd_tb = 0.0;

    Vx_cmd_tb_unsat = 0.0;
    Vy_cmd_tb_unsat = 0.0;

    evx_tb = 0.0;
    evy_tb = 0.0;
    evz_tb = 0.0;

    eu_tb = 0.0;
    ev_tb = 0.0;

    vel_int_u_tb = 0.0;
    vel_int_v_tb = 0.0;

    theta_cmd_ref_tbstates = 0.0;
    phi_cmd_ref_tbstates   = 0.0;


    e_theta_att_ref = 0.0;
    e_phi_att_ref   = 0.0;

    att_int_theta_ref = 0.0;
    att_int_phi_ref   = 0.0;

    p_cmd_ref = 0.0;
    q_cmd_ref = 0.0;


    p_meas_ref = 0.0;
    q_meas_ref = 0.0;

    p_prev_ref = 0.0;
    q_prev_ref = 0.0;

    p_dot_ref = 0.0;
    q_dot_ref = 0.0;

    e_p_ref = 0.0;
    e_q_ref = 0.0;

    rate_int_p_ref = 0.0;
    rate_int_q_ref = 0.0;

    u2_ref = 0.0;
    u3_ref = 0.0;


    vx_hat_ref = 0.0;
    vy_hat_ref = 0.0;

    x_hat_ref = 0.0;
    y_hat_ref = 0.0;


    vx_hat_ref_rtl = 0.0;
    vy_hat_ref_rtl = 0.0;

    x_hat_ref_rtl = 0.0;
    y_hat_ref_rtl = 0.0;

    x_delay_ref_rtl = 0.0;
    y_delay_ref_rtl = 0.0;

    vx_prev_rtl = 0.0;
    vy_prev_rtl = 0.0;


    ex_ref = 0.0;
    ey_ref = 0.0;

    ex_ref_rtl = 0.0;
    ey_ref_rtl = 0.0;

end

endtask


//==========================================================
// COMPUTE PHYSICAL PLANT DERIVATIVES
//==========================================================
//
// Z-Y-X Euler kinematics:
//
// phi_dot   = p + (q sin(phi) + r cos(phi)) tan(theta)
// theta_dot = q cos(phi) - r sin(phi)
// psi_dot   = (q sin(phi) + r cos(phi)) / cos(theta)
//
//==========================================================

task automatic compute_plant_derivatives_ref;

begin

    u_dot_ref =
          v_ref * r_ref
        - w_ref * q_ref
        + g * $sin(theta_ref);


    v_dot_ref =
          w_ref * p_ref
        - u_ref * r_ref
        - g
          * $sin(phi_ref)
          * $cos(theta_ref);


    w_dot_ref =
          u_ref * q_ref
        - v_ref * p_ref
        - g
          * $cos(phi_ref)
          * $cos(theta_ref)
        + U1_plant / PLANT_MASS;


    p_dot_plant_ref =
          (
              (PLANT_IY - PLANT_IZ)
              / PLANT_IX
          )
          * q_ref
          * r_ref
        + U2_plant / PLANT_IX;


    q_dot_plant_ref =
          (
              (PLANT_IZ - PLANT_IX)
              / PLANT_IY
          )
          * p_ref
          * r_ref
        + U3_plant / PLANT_IY;


    r_dot_plant_ref =
          (
              (PLANT_IX - PLANT_IY)
              / PLANT_IZ
          )
          * p_ref
          * q_ref
        + U4_plant / PLANT_IZ;


    // Correct Z-Y-X Euler kinematics.
    phi_dot_ref =
          p_ref
        + (
              q_ref * $sin(phi_ref)
            + r_ref * $cos(phi_ref)
          )
          * $tan(theta_ref);


    theta_dot_ref =
          q_ref * $cos(phi_ref)
        - r_ref * $sin(phi_ref);


    psi_dot_ref =
          (
              q_ref * $sin(phi_ref)
            + r_ref * $cos(phi_ref)
          )
          /
          $cos(theta_ref);


    a_bx_ref =
          u_dot_ref
        + w_ref * q_ref
        - v_ref * r_ref;


    a_by_ref =
          v_dot_ref
        + u_ref * r_ref
        - w_ref * p_ref;


    a_bz_ref =
          w_dot_ref
        + v_ref * p_ref
        - u_ref * q_ref;


    x_dot_ref =
          u_ref
          * $cos(theta_ref)
          * $cos(psi_ref)

        + v_ref
          * (
                $sin(phi_ref)
                * $sin(theta_ref)
                * $cos(psi_ref)
              -
                $cos(phi_ref)
                * $sin(psi_ref)
            )

        + w_ref
          * (
                $cos(phi_ref)
                * $sin(theta_ref)
                * $cos(psi_ref)
              +
                $sin(phi_ref)
                * $sin(psi_ref)
            );


    y_dot_ref =
          u_ref
          * $cos(theta_ref)
          * $sin(psi_ref)

        + v_ref
          * (
                $sin(phi_ref)
                * $sin(theta_ref)
                * $sin(psi_ref)
              +
                $cos(phi_ref)
                * $cos(psi_ref)
            )

        + w_ref
          * (
                $cos(phi_ref)
                * $sin(theta_ref)
                * $sin(psi_ref)
              -
                $sin(phi_ref)
                * $cos(psi_ref)
            );


    z_dot_ref =
          -u_ref
          * $sin(theta_ref)

        + v_ref
          * $sin(phi_ref)
          * $cos(theta_ref)

        + w_ref
          * $cos(phi_ref)
          * $cos(theta_ref);

end

endtask


//==========================================================
// COMPUTE TRUE INERTIAL ACCELERATION
//==========================================================

task automatic compute_true_global_acceleration_ref;

begin

    R11_ref =
        $cos(psi_ref)
        * $cos(theta_ref);

    R12_ref =
        $cos(psi_ref)
        * $sin(theta_ref)
        * $sin(phi_ref)
        -
        $sin(psi_ref)
        * $cos(phi_ref);

    R13_ref =
        $cos(psi_ref)
        * $sin(theta_ref)
        * $cos(phi_ref)
        +
        $sin(psi_ref)
        * $sin(phi_ref);


    R21_ref =
        $sin(psi_ref)
        * $cos(theta_ref);

    R22_ref =
        $sin(psi_ref)
        * $sin(theta_ref)
        * $sin(phi_ref)
        +
        $cos(psi_ref)
        * $cos(phi_ref);

    R23_ref =
        $sin(psi_ref)
        * $sin(theta_ref)
        * $cos(phi_ref)
        -
        $cos(psi_ref)
        * $sin(phi_ref);


    R31_ref =
        -$sin(theta_ref);

    R32_ref =
        $cos(theta_ref)
        * $sin(phi_ref);

    R33_ref =
        $cos(theta_ref)
        * $cos(phi_ref);


    axg_ref =
          R11_ref * a_bx_ref
        + R12_ref * a_by_ref
        + R13_ref * a_bz_ref;

    ayg_ref =
          R21_ref * a_bx_ref
        + R22_ref * a_by_ref
        + R23_ref * a_bz_ref;

    azg_ref =
          R31_ref * a_bx_ref
        + R32_ref * a_by_ref
        + R33_ref * a_bz_ref;

end

endtask


//==========================================================
// GENERATE PHYSICAL SENSOR MEASUREMENTS
//==========================================================

task automatic generate_sensor_measurements_ref;

begin

    gx =
        real_to_sfix(
            p_ref,
            10
        );

    gy =
        real_to_sfix(
            q_ref,
            10
        );

    gz =
        real_to_sfix(
            r_ref,
            10
        );


    fx_ref =
          a_bx_ref
        - g * $sin(theta_ref);

    fy_ref =
          a_by_ref
        + g
          * $sin(phi_ref)
          * $cos(theta_ref);

    fz_ref =
          a_bz_ref
        + g
          * $cos(phi_ref)
          * $cos(theta_ref);


    ax =
        real_to_sfix(
            fx_ref,
            10
        );

    ay =
        real_to_sfix(
            fy_ref,
            10
        );

    az =
        real_to_sfix(
            fz_ref,
            10
        );


    ax_rm =
        real_to_sfix(
            a_bx_ref,
            10
        );

    ay_rm =
        real_to_sfix(
            a_by_ref,
            10
        );

    az_rm =
        real_to_sfix(
            a_bz_ref,
            10
        );

end

endtask


//==========================================================
// COMPLEMENTARY FILTER REFERENCE
//==========================================================

task automatic complementary_filter_ref;

begin

    phi_dot_ref =
          p_ref
        + (
              q_ref * $sin(phi_ref)
            + r_ref * $cos(phi_ref)
          )
          * $tan(theta_ref);


    theta_dot_ref =
          q_ref * $cos(phi_ref)
        - r_ref * $sin(phi_ref);


    phi_acc_ref =
        $atan2(
            fy_ref,
            fz_ref
        );


    theta_acc_ref =
        $atan2(
            -fx_ref,
            $sqrt(
                fy_ref * fy_ref
              + fz_ref * fz_ref
            )
        );


    phi_pred_ref =
        phi_cf_ref
        + phi_dot_ref * Ts;


    theta_pred_ref =
        theta_cf_ref
        + theta_dot_ref * Ts;


    phi_cf_ref =
        alpha * phi_pred_ref
        +
        (1.0 - alpha) * phi_acc_ref;


    theta_cf_ref =
        alpha * theta_pred_ref
        +
        (1.0 - alpha) * theta_acc_ref;


    psi_cf_ref =
        psi_ref;


    if (sample_idx < HIST_LEN) begin

        phi_dot_hist[sample_idx] =
            phi_dot_ref;

        phi_acc_hist[sample_idx] =
            phi_acc_ref;

    end


    gyro_sum_ref =
        0.0;

    acc_sum_ref =
        0.0;


    if (sample_idx < HIST_LEN) begin

        for (
            j = 1;
            j <= sample_idx + 1;
            j = j + 1
        ) begin

            gyro_sum_ref =
                gyro_sum_ref
                +
                $pow(
                    alpha,
                    (sample_idx + 1) - j + 1
                )
                *
                phi_dot_hist[j - 1];


            acc_sum_ref =
                acc_sum_ref
                +
                $pow(
                    alpha,
                    (sample_idx + 1) - j
                )
                *
                phi_acc_hist[j - 1];

        end


        phi_formula_ref =
              $pow(
                  alpha,
                  sample_idx + 1
              )
              * phi0_ref

            +
              Ts
              * gyro_sum_ref

            +
              (1.0 - alpha)
              * acc_sum_ref;

    end
    else begin

        phi_formula_ref =
            0.0;

    end


    if (sample_idx < HIST_LEN)

        sample_idx =
            sample_idx + 1;

end

endtask


//==========================================================
// FORWARD-EULER PLANT INTEGRATION
//==========================================================

task automatic integrate_plant_ref;

begin

    u_ref =
        u_ref
        + u_dot_ref * Ts;

    v_ref =
        v_ref
        + v_dot_ref * Ts;

    w_ref =
        w_ref
        + w_dot_ref * Ts;


    p_ref =
        p_ref
        + p_dot_plant_ref * Ts;

    q_ref =
        q_ref
        + q_dot_plant_ref * Ts;

    r_ref =
        r_ref
        + r_dot_plant_ref * Ts;


    phi_ref =
        phi_ref
        + phi_dot_ref * Ts;

    theta_ref =
        theta_ref
        + theta_dot_ref * Ts;

    psi_ref =
        psi_ref
        + psi_dot_ref * Ts;


    x_ref =
        x_ref
        + x_dot_ref * Ts;

    y_ref =
        y_ref
        + y_dot_ref * Ts;

    z_ref =
        z_ref
        + z_dot_ref * Ts;


    x_raw =
        x_ref;

    y_raw =
        y_ref;

    vx_raw =
        x_dot_ref;

    vy_raw =
        y_dot_ref;

end

endtask


//==========================================================
// MATLAB POSITION OBSERVER
//==========================================================

task automatic position_observer_ref;

begin

    ex_ref =
        x_raw
        - x_hat_ref;

    ey_ref =
        y_raw
        - y_hat_ref;


    vx_hat_ref =
        vx_hat_ref
        +
        (
            axg_ref
            +
            K1_X_MATLAB * ex_ref
        )
        * Ts;


    vy_hat_ref =
        vy_hat_ref
        +
        (
            ayg_ref
            +
            K1_Y_MATLAB * ey_ref
        )
        * Ts;


    x_hat_ref =
        x_hat_ref
        +
        (
            vx_hat_ref
            +
            K2_MATLAB * ex_ref
        )
        * Ts;


    y_hat_ref =
        y_hat_ref
        +
        (
            vy_hat_ref
            +
            K2_MATLAB * ey_ref
        )
        * Ts;

end

endtask


//==========================================================
// RTL-TIMED POSITION OBSERVER REFERENCE
//
// Mirrors the generated Position_Filter2.v:
//
//     Saturation(error) occurs BEFORE K2.
//
//     Vx(k+1) = Vx(k)
//              + Ts * (ax_global + K1*x_error)
//
//     X(k+1)  = X(k)
//              + Ts * (Vx(k) + K2*x_error)
//
// The generated RTL explicitly uses the OLD velocity state
// Vx_k/Vy_k for the position update.
//
// The exact generated integrator step is:
//
//     Ts = 131 / 32768
//
//==========================================================

task automatic position_observer_ref_rtl;

begin

    //------------------------------------------------------
    // Read the ACTUAL DUT Position_Filter2 raw inputs
    //------------------------------------------------------

    X_raw_dut =
        sfix_to_real(
            uut.u_Position_Filter2.X_raw,
            10
        );

    Y_raw_dut =
        sfix_to_real(
            uut.u_Position_Filter2.Y_raw,
            10
        );


    //------------------------------------------------------
    // Compare DUT raw inputs with physical truth
    //------------------------------------------------------

    X_raw_dut_error =
        X_raw_dut
        -
        x_raw;

    Y_raw_dut_error =
        Y_raw_dut
        -
        y_raw;


    //------------------------------------------------------
    // Position error
    //
    // Generated RTL:
    //
    //     Sum_rsvd = X_raw - Unit_Delay_out1
    //
    // followed by saturation to +/-20 m.
    //
    // The physical test uses values far below the ±20 m
    // saturation limit, so the reference keeps the same
    // mathematical result.
    //------------------------------------------------------

    ex_ref_rtl =
        X_raw_dut
        -
        x_delay_ref_rtl;

    ey_ref_rtl =
        Y_raw_dut
        -
        y_delay_ref_rtl;


    //------------------------------------------------------
    // Save OLD velocity
    //
    // This MUST happen before updating vx_hat_ref_rtl and
    // vy_hat_ref_rtl because the generated RTL uses Vx_k
    // and Vy_k in the position integrators.
    //------------------------------------------------------

    vx_prev_rtl =
        vx_hat_ref_rtl;

    vy_prev_rtl =
        vy_hat_ref_rtl;


    //------------------------------------------------------
    // Velocity observer state
    //------------------------------------------------------

    vx_hat_ref_rtl =
        vx_hat_ref_rtl
        +
        (
            axg_ref
            +
            K1_X_RTL * ex_ref_rtl
        )
        *
        Ts_position_rtl;


    vy_hat_ref_rtl =
        vy_hat_ref_rtl
        +
        (
            ayg_ref
            +
            K1_Y_RTL * ey_ref_rtl
        )
        *
        Ts_position_rtl;


    //------------------------------------------------------
    // Position observer state
    //
    // OLD velocity is intentional and matches RTL.
    //------------------------------------------------------

    x_hat_ref_rtl =
        x_hat_ref_rtl
        +
        (
            vx_prev_rtl
            +
            K2_RTL * ex_ref_rtl
        )
        *
        Ts_position_rtl;


    y_hat_ref_rtl =
        y_hat_ref_rtl
        +
        (
            vy_prev_rtl
            +
            K2_RTL * ey_ref_rtl
        )
        *
        Ts_position_rtl;


    //------------------------------------------------------
    // Unit-delay state
    //
    // Corresponds to:
    //
    //     Unit_Delay_out1 <= Xk
    //     Unit_Delay1_out1 <= Yk
    //------------------------------------------------------

    x_delay_ref_rtl =
        x_hat_ref_rtl;

    y_delay_ref_rtl =
        y_hat_ref_rtl;

end

endtask


//==========================================================
// DUT-STATE CONTROLLER REFERENCE
//==========================================================

task automatic controller_ref_from_dut;

    real Vx_unsat;
    real Vy_unsat;

begin

    Xest_ctrl_dut =
        sfix_to_real(
            Xest,
            10
        );

    Yest_ctrl_dut =
        sfix_to_real(
            Yest,
            10
        );


    Uest_ctrl_dut =
        sfix_to_real(
            Uest,
            10
        );

    Vest_ctrl_dut =
        sfix_to_real(
            Vest,
            10
        );


    phi_ctrl_dut =
        sfix_to_real(
            phi_filt_dbg_en14,
            14
        );

    theta_ctrl_dut =
        sfix_to_real(
            theta_filt_dbg_en9,
            9
        );

    psi_ctrl_dut =
        sfix_to_real(
            psi_filt_dbg_en12,
            12
        );


    ex_pos_dut =
        X_set
        -
        Xest_ctrl_dut;

    ey_pos_dut =
        Y_set
        -
        Yest_ctrl_dut;


    pos_int_x_dut =
        pos_int_x_dut
        +
        ex_pos_dut
        *
        Ts_controller;

    pos_int_y_dut =
        pos_int_y_dut
        +
        ey_pos_dut
        *
        Ts_controller;


    Vx_unsat =
        Kp_pos
        * ex_pos_dut
        +
        Ki_pos
        * pos_int_x_dut;


    Vy_unsat =
        Kp_pos
        * ey_pos_dut
        +
        Ki_pos
        * pos_int_y_dut;


    Vx_cmd_dut_unsat =
        Vx_unsat;

    Vy_cmd_dut_unsat =
        Vy_unsat;


    Vx_cmd_dut =
        saturate_velocity(
            Vx_unsat
        );

    Vy_cmd_dut =
        saturate_velocity(
            Vy_unsat
        );


    evx_dut =
        Vx_cmd_dut
        -
        Uest_ctrl_dut;

    evy_dut =
        Vy_cmd_dut
        -
        Vest_ctrl_dut;

    evz_dut =
        0.0;


    eu_dut =
          evx_dut
          *
          (
              $cos(theta_ctrl_dut)
              *
              $cos(psi_ctrl_dut)
          )

        + evy_dut
          *
          (
              $cos(theta_ctrl_dut)
              *
              $sin(psi_ctrl_dut)
          )

        + evz_dut
          *
          (
              -$sin(theta_ctrl_dut)
          );


    ev_dut =
          evx_dut
          *
          (
              $sin(phi_ctrl_dut)
              * $sin(theta_ctrl_dut)
              * $cos(psi_ctrl_dut)
              -
              $cos(phi_ctrl_dut)
              * $sin(psi_ctrl_dut)
          )

        + evy_dut
          *
          (
              $sin(phi_ctrl_dut)
              * $sin(theta_ctrl_dut)
              * $sin(psi_ctrl_dut)
              +
              $cos(phi_ctrl_dut)
              * $cos(psi_ctrl_dut)
          )

        + evz_dut
          *
          (
              $sin(phi_ctrl_dut)
              * $cos(theta_ctrl_dut)
          );


    vel_int_u_dut =
        vel_int_u_dut
        +
        eu_dut
        *
        Ts_controller;

    vel_int_v_dut =
        vel_int_v_dut
        +
        ev_dut
        *
        Ts_controller;


    theta_cmd_ref_dutstate =
        Kp_vel
        *
        eu_dut
        +
        Ki_vel
        *
        vel_int_u_dut;


    phi_cmd_ref_dutstate =
        -(
            Kp_vel
            *
            ev_dut
            +
            Ki_vel
            *
            vel_int_v_dut
        );

end

endtask


//==========================================================
// TB-STATE CONTROLLER REFERENCE
//==========================================================

task automatic controller_ref_from_tb;

    real Vx_unsat;
    real Vy_unsat;

begin

    Xest_ctrl_tb =
        x_hat_ref_rtl;

    Yest_ctrl_tb =
        y_hat_ref_rtl;

    Uest_ctrl_tb =
        vx_hat_ref_rtl;

    Vest_ctrl_tb =
        vy_hat_ref_rtl;


    phi_ctrl_tb =
        phi_cf_ref;

    theta_ctrl_tb =
        theta_cf_ref;

    psi_ctrl_tb =
        psi_cf_ref;


    ex_pos_tb =
        X_set
        -
        Xest_ctrl_tb;

    ey_pos_tb =
        Y_set
        -
        Yest_ctrl_tb;


    pos_int_x_tb =
        pos_int_x_tb
        +
        ex_pos_tb
        *
        Ts_controller;

    pos_int_y_tb =
        pos_int_y_tb
        +
        ey_pos_tb
        *
        Ts_controller;


    Vx_unsat =
        Kp_pos
        *
        ex_pos_tb
        +
        Ki_pos
        *
        pos_int_x_tb;


    Vy_unsat =
        Kp_pos
        *
        ey_pos_tb
        +
        Ki_pos
        *
        pos_int_y_tb;


    Vx_cmd_tb_unsat =
        Vx_unsat;

    Vy_cmd_tb_unsat =
        Vy_unsat;


    Vx_cmd_tb =
        saturate_velocity(
            Vx_unsat
        );

    Vy_cmd_tb =
        saturate_velocity(
            Vy_unsat
        );


    evx_tb =
        Vx_cmd_tb
        -
        Uest_ctrl_tb;

    evy_tb =
        Vy_cmd_tb
        -
        Vest_ctrl_tb;

    evz_tb =
        0.0;


    eu_tb =
          evx_tb
          *
          (
              $cos(theta_ctrl_tb)
              *
              $cos(psi_ctrl_tb)
          )

        + evy_tb
          *
          (
              $cos(theta_ctrl_tb)
              *
              $sin(psi_ctrl_tb)
          )

        + evz_tb
          *
          (
              -$sin(theta_ctrl_tb)
          );


    ev_tb =
          evx_tb
          *
          (
              $sin(phi_ctrl_tb)
              * $sin(theta_ctrl_tb)
              * $cos(psi_ctrl_tb)
              -
              $cos(phi_ctrl_tb)
              * $sin(psi_ctrl_tb)
          )

        + evy_tb
          *
          (
              $sin(phi_ctrl_tb)
              * $sin(theta_ctrl_tb)
              * $sin(psi_ctrl_tb)
              +
              $cos(phi_ctrl_tb)
              * $cos(psi_ctrl_tb)
          )

        + evz_tb
          *
          (
              $sin(phi_ctrl_tb)
              * $cos(theta_ctrl_tb)
          );


    vel_int_u_tb =
        vel_int_u_tb
        +
        eu_tb
        *
        Ts_controller;

    vel_int_v_tb =
        vel_int_v_tb
        +
        ev_tb
        *
        Ts_controller;


    theta_cmd_ref_tbstates =
        Kp_vel
        *
        eu_tb
        +
        Ki_vel
        *
        vel_int_u_tb;


    phi_cmd_ref_tbstates =
        -(
            Kp_vel
            *
            ev_tb
            +
            Ki_vel
            *
            vel_int_v_tb
        );

end

endtask


//==========================================================
// ATTITUDE -> BODY-RATE REFERENCE
//==========================================================

task automatic attitude_rate_controller_ref;

begin

    e_theta_att_ref =
        theta_cmd_ref_tbstates
        -
        theta_ctrl_tb;

    e_phi_att_ref =
        phi_cmd_ref_tbstates
        -
        phi_ctrl_tb;


    att_int_theta_ref =
        att_int_theta_ref
        +
        e_theta_att_ref
        *
        Ts_controller;

    att_int_phi_ref =
        att_int_phi_ref
        +
        e_phi_att_ref
        *
        Ts_controller;


    q_cmd_ref =
        Kp_attitude
        *
        e_theta_att_ref
        +
        Ki_attitude
        *
        att_int_theta_ref;


    p_cmd_ref =
        Kp_attitude
        *
        e_phi_att_ref
        +
        Ki_attitude
        *
        att_int_phi_ref;

end

endtask


//==========================================================
// RATE CONTROLLER REFERENCE
//==========================================================

task automatic innermost_rate_controller_ref;

begin

    p_meas_ref =
        sfix_to_real(
            p_filt_dbg_en10,
            10
        );

    q_meas_ref =
        sfix_to_real(
            q_filt_dbg_en10,
            10
        );


    p_dot_ref =
        (
            p_meas_ref
            -
            p_prev_ref
        )
        /
        Ts_rate;


    q_dot_ref =
        (
            q_meas_ref
            -
            q_prev_ref
        )
        /
        Ts_rate;


    p_prev_ref =
        p_meas_ref;

    q_prev_ref =
        q_meas_ref;


    e_p_ref =
        p_cmd_ref
        -
        p_meas_ref;

    e_q_ref =
        q_cmd_ref
        -
        q_meas_ref;


    rate_int_p_ref =
        rate_int_p_ref
        +
        e_p_ref
        *
        Ts_rate;

    rate_int_q_ref =
        rate_int_q_ref
        +
        e_q_ref
        *
        Ts_rate;


    u2_ref =
          Ki_rate
          *
          rate_int_p_ref

        -
          Kp_rate
          *
          p_meas_ref

        -
          Kd_rate
          *
          p_dot_ref;


    u3_ref =
          Ki_rate
          *
          rate_int_q_ref

        -
          Kp_rate
          *
          q_meas_ref

        -
          Kd_rate
          *
          q_dot_ref;

end

endtask


//==========================================================
// VELOCITY SATURATION
//==========================================================

function automatic real saturate_velocity;

    input real v;

    begin

        if (v > v_max)

            saturate_velocity =
                v_max;

        else if (v < -v_max)

            saturate_velocity =
                -v_max;

        else

            saturate_velocity =
                v;

    end

endfunction


//==========================================================
// SET POSITION SETPOINT
//==========================================================

task automatic set_position_setpoint;

    input real x_cmd;
    input real y_cmd;

begin

    X_set =
        x_cmd;

    Y_set =
        y_cmd;


    `LOG0("")
    `LOG0("============================================================")
    `LOG0(" POSITION SETPOINT")
    `LOG1(" X = %0.5f", X_set)
    `LOG1(" Y = %0.5f", Y_set)
    `LOG0("============================================================")

end

endtask


//==========================================================
// RUN POSITION COMMAND
//==========================================================

task automatic run_position_command;

    input real x_cmd;
    input real y_cmd;
    input integer cycles;

    integer k;

begin

    set_position_setpoint(
        x_cmd,
        y_cmd
    );


    `LOG0("")
    `LOG0("============================================================")
    `LOG0(" POSITION COMMAND")
    `LOG1(" X_set  = %0.5f", X_set)
    `LOG1(" Y_set  = %0.5f", Y_set)
    `LOG1(" CYCLES = %0d", cycles)
    `LOG0("============================================================")


    for (
        k = 0;
        k < cycles;
        k = k + 1
    ) begin

        compute_plant_derivatives_ref();

        compute_true_global_acceleration_ref();

        generate_sensor_measurements_ref();


        @(posedge uut.u_Attitude_Roll_Controller_2.ce_out);

        #1;


        complementary_filter_ref();

        position_observer_ref();

        position_observer_ref_rtl();


        controller_ref_from_dut();

        controller_ref_from_tb();


        attitude_rate_controller_ref();

        innermost_rate_controller_ref();


        u2_dut_real =
            sfix_to_real(
                U2,
                10
            );


        u2_ref_error =
            u2_dut_real
            -
            u2_ref;


        diagnose_u2_all();


        U2_plant =
            u2_dut_real;


        U3_plant =
            0.0;


        compute_plant_derivatives_ref();

        integrate_plant_ref();


        pulse_count =
            pulse_count + 1;


        dump_full_chain();

        print_reference();

        print_controller();

        roll_ctrl_debug();

    end


    `LOG0("")
    `LOG0("============================================================")
    `LOG0(" POSITION COMMAND COMPLETE")
    `LOG1(" X_set  = %0.5f", X_set)
    `LOG1(" Y_set  = %0.5f", Y_set)
    `LOG1(" CYCLES = %0d", cycles)
    `LOG0("============================================================")

    $fflush(
        log_file
    );

end

endtask


//==========================================================
// FULL DEBUG DUMP
//==========================================================

task automatic dump_full_chain;

begin

    `LOG0("")
    `LOG0("================================================================================================================")
    `LOG0("                                         FULL CHAIN DEBUG")
    `LOG0("================================================================================================================")


    `LOG0("[TRUE PHYSICAL PLANT]")

    `LOG3(
        "BODY VEL       : u=%0.5f v=%0.5f w=%0.5f",
        u_ref,
        v_ref,
        w_ref
    )

    `LOG3(
        "BODY RATE      : p=%0.5f q=%0.5f r=%0.5f",
        p_ref,
        q_ref,
        r_ref
    )

    `LOG3(
        "ATTITUDE       : phi=%0.5f theta=%0.5f psi=%0.5f",
        phi_ref,
        theta_ref,
        psi_ref
    )

    `LOG3(
        "POSITION       : x=%0.5f y=%0.5f z=%0.5f",
        x_ref,
        y_ref,
        z_ref
    )

    `LOG3(
        "POSITION DOT   : xd=%0.5f yd=%0.5f zd=%0.5f",
        x_dot_ref,
        y_dot_ref,
        z_dot_ref
    )

    `LOG3(
        "BODY ACC       : ax=%0.5f ay=%0.5f az=%0.5f",
        a_bx_ref,
        a_by_ref,
        a_bz_ref
    )


    `LOG0("")
    `LOG0("[APPLIED PLANT CONTROL]")

    `LOG4(
        "U1=%0.5f U2_DUT=%0.5f U3=%0.5f U4=%0.5f",
        U1_plant,
        U2_plant,
        U3_plant,
        U4_plant
    )


    `LOG0("")
    `LOG0("[PHYSICAL SENSOR MODEL]")

    `LOG3(
        "GYRO         : p=%0.5f q=%0.5f r=%0.5f",
        p_ref,
        q_ref,
        r_ref
    )

    `LOG3(
        "SPEC FORCE   : fx=%0.5f fy=%0.5f fz=%0.5f",
        fx_ref,
        fy_ref,
        fz_ref
    )

    `LOG3(
        "ACC RM INPUT : ax=%0.5f ay=%0.5f az=%0.5f",
        a_bx_ref,
        a_by_ref,
        a_bz_ref
    )


    `LOG0("")
    `LOG0("[TRUE GLOBAL ACCELERATION]")

    `LOG3(
        "TRUE Z-UP    : ax=%0.5f ay=%0.5f az=%0.5f",
        axg_ref,
        ayg_ref,
        azg_ref
    )


    `LOG0("")
    `LOG0("[DUT GLOBAL ACCELERATION]")

    `LOG2(
        "DUT          : ax=%0.5f ay=%0.5f",
        sfix_to_real(
            ax_global,
            10
        ),
        sfix_to_real(
            ay_global,
            10
        )
    )


    `LOG0("")
    `LOG0("[ATTITUDE]")

    `LOG3(
        "TRUE         : phi=%0.5f theta=%0.5f psi=%0.5f",
        phi_ref,
        theta_ref,
        psi_ref
    )

    `LOG3(
        "FILTER REF   : phi=%0.5f theta=%0.5f psi=%0.5f",
        phi_cf_ref,
        theta_cf_ref,
        psi_cf_ref
    )

    `LOG3(
        "DUT          : phi=%0.5f theta=%0.5f psi=%0.5f",
        sfix_to_real(
            phi_filt_dbg_en14,
            14
        ),
        sfix_to_real(
            theta_filt_dbg_en9,
            9
        ),
        sfix_to_real(
            psi_filt_dbg_en12,
            12
        )
    )


    `LOG0("")
    `LOG0("[POSITION]")

    `LOG4(
        "TRUE         : x=%0.5f y=%0.5f vx=%0.5f vy=%0.5f",
        x_ref,
        y_ref,
        vx_raw,
        vy_raw
    )

    `LOG4(
        "TB OBS       : x=%0.5f y=%0.5f vx=%0.5f vy=%0.5f",
        x_hat_ref_rtl,
        y_hat_ref_rtl,
        vx_hat_ref_rtl,
        vy_hat_ref_rtl
    )

    `LOG4(
        "DUT          : X=%0.5f Y=%0.5f U=%0.5f V=%0.5f",
        sfix_to_real(
            Xest,
            10
        ),
        sfix_to_real(
            Yest,
            10
        ),
        sfix_to_real(
            Uest,
            10
        ),
        sfix_to_real(
            Vest,
            10
        )
    )


    `LOG0("")
    `LOG0("[CONTROLLER]")

    `LOG2(
        "SETPOINT     : X=%0.5f Y=%0.5f",
        X_set,
        Y_set
    )

    `LOG2(
        "TB ATT CMD   : theta=%0.5f phi=%0.5f",
        theta_cmd_ref_tbstates,
        phi_cmd_ref_tbstates
    )

    `LOG2(
        "DUT ATT CMD  : theta=%0.5f phi=%0.5f",
        sfix_to_real(
            theta_cmd,
            14
        ),
        sfix32_to_real(
            phi_cmd,
            29
        )
    )


    `LOG0("")
    `LOG0("[RATE / TORQUE]")

    `LOG2(
        "RATE CMD     : p=%0.5f q=%0.5f",
        p_cmd_ref,
        q_cmd_ref
    )

    `LOG2(
        "RATE MEAS    : p=%0.5f q=%0.5f",
        p_meas_ref,
        q_meas_ref
    )

    `LOG2(
        "REF TORQUE   : U2=%0.5f U3=%0.5f",
        u2_ref,
        u3_ref
    )

    `LOG1(
        "DUT U2       : U2=%0.5f",
        u2_dut_real
    )

    `LOG1(
        "U2 ERROR     : %0.5f",
        u2_ref_error
    )

    `LOG0(
        "DUT U3       : NOT PRESENT IN CURRENT DUT"
    )


    //------------------------------------------------------
    // IMPORTANT POSITION FILTER INPUT DIAGNOSTIC
    //------------------------------------------------------

    `LOG0("")
    `LOG0("[DUT POSITION FILTER INTERNALS]")

    `LOG1(
        "DUT X_raw       = %0.9f",
        X_raw_dut
    )

    `LOG1(
        "DUT Y_raw       = %0.9f",
        Y_raw_dut
    )

    `LOG1(
        "TRUE x_raw      = %0.9f",
        x_raw
    )

    `LOG1(
        "TRUE y_raw      = %0.9f",
        y_raw
    )

    `LOG1(
        "X_raw DUT-error = %0.9f",
        X_raw_dut_error
    )

    `LOG1(
        "Y_raw DUT-error = %0.9f",
        Y_raw_dut_error
    )

    `LOG1(
        "DUT Xest        = %0.9f",
        sfix_to_real(
            Xest,
            10
        )
    )

    `LOG1(
        "DUT Yest        = %0.9f",
        sfix_to_real(
            Yest,
            10
        )
    )

    `LOG1(
        "DUT Uest        = %0.9f",
        sfix_to_real(
            Uest,
            10
        )
    )

    `LOG1(
        "DUT Vest        = %0.9f",
        sfix_to_real(
            Vest,
            10
        )
    )


    $fflush(
        log_file
    );

end

endtask


//==========================================================
// REFERENCE SUMMARY
//==========================================================

task automatic print_reference;

begin

    `LOG0("")

    `LOG2(
        "Pulse %0d   Time=%0t",
        pulse_count,
        $time
    )

    `LOG3(
        "TRUE ATT : phi=%0.5f theta=%0.5f psi=%0.5f",
        phi_ref,
        theta_ref,
        psi_ref
    )

    `LOG3(
        "TRUE POS : x=%0.5f y=%0.5f z=%0.5f",
        x_ref,
        y_ref,
        z_ref
    )

    `LOG3(
        "SPEC FRC : fx=%0.5f fy=%0.5f fz=%0.5f",
        fx_ref,
        fy_ref,
        fz_ref
    )

    `LOG3(
        "TRUE ACC : ax=%0.5f ay=%0.5f az=%0.5f",
        axg_ref,
        ayg_ref,
        azg_ref
    )

    `LOG4(
        "TB POS   : X=%0.5f Y=%0.5f U=%0.5f V=%0.5f",
        x_hat_ref_rtl,
        y_hat_ref_rtl,
        vx_hat_ref_rtl,
        vy_hat_ref_rtl
    )

    `LOG4(
        "DUT POS  : X=%0.5f Y=%0.5f U=%0.5f V=%0.5f",
        sfix_to_real(Xest,10),
        sfix_to_real(Yest,10),
        sfix_to_real(Uest,10),
        sfix_to_real(Vest,10)
    )

end

endtask


//==========================================================
// CONTROLLER SUMMARY
//==========================================================

task automatic print_controller;

begin

    `LOG0("")

    `LOG0("[CONTROLLER SUMMARY]")

    `LOG2(
        "SETPOINT : X=%0.5f Y=%0.5f",
        X_set,
        Y_set
    )

    `LOG2(
        "REF CMD  : theta=%0.5f phi=%0.5f",
        theta_cmd_ref_tbstates,
        phi_cmd_ref_tbstates
    )

    `LOG2(
        "DUT CMD  : theta=%0.5f phi=%0.5f",
        sfix_to_real(
            theta_cmd,
            14
        ),
        sfix32_to_real(
            phi_cmd,
            29
        )
    )

    `LOG2(
        "RATE CMD : p=%0.5f q=%0.5f",
        p_cmd_ref,
        q_cmd_ref
    )

    `LOG2(
        "REF U2/U3: U2=%0.5f U3=%0.5f",
        u2_ref,
        u3_ref
    )

    `LOG1(
        "DUT U2   : U2=%0.5f",
        u2_dut_real
    )

    `LOG0(
        "DUT U3   : unavailable in current DUT"
    )

end

endtask


//==========================================================
// ROLL CONTROLLER INTERNAL DEBUG
//==========================================================

task automatic roll_ctrl_debug;

begin

    $display("");
    $display("============================================================");
    $display("             ATTITUDE ROLL DUT PIPELINE DEBUG");
    $display("============================================================");

    $display(
        "TIME                  = %0t",
        $time
    );

    $display(
        "s_s                   = %0d",
        uut.u_Attitude_Roll_Controller_2.s_s
    );

    $display("");
    $display("[INPUTS AT DUT OUTPUT EVENT]");

    $display(
        "phi_raw_req           = %0d  (%f)",
        $signed(
            uut.u_Attitude_Roll_Controller_2.phi_raw_req
        ),
        $itor(
            $signed(
                uut.u_Attitude_Roll_Controller_2.phi_raw_req
            )
        ) / 16384.0
    );

    $display(
        "phi_meas              = %0d  (%f)",
        $signed(
            uut.u_Attitude_Roll_Controller_2.phi_meas
        ),
        $itor(
            $signed(
                uut.u_Attitude_Roll_Controller_2.phi_meas
            )
        ) / 16384.0
    );

    $display(
        "p_meas                = %0d  (%f)",
        $signed(
            uut.u_Attitude_Roll_Controller_2.p_meas
        ),
        $itor(
            $signed(
                uut.u_Attitude_Roll_Controller_2.p_meas
            )
        ) / 1024.0
    );


    $display("");
    $display("[ROLL ERROR PATH]");

    $display(
        "phi_error_out1        = %0d  (%f)",
        $signed(
            uut.u_Attitude_Roll_Controller_2.phi_error_out1
        ),
        $itor(
            $signed(
                uut.u_Attitude_Roll_Controller_2.phi_error_out1
            )
        ) / 1024.0
    );

    $display(
        "p_meas_2              = %0d  (%f)",
        $signed(
            uut.u_Attitude_Roll_Controller_2.p_meas_2
        ),
        $itor(
            $signed(
                uut.u_Attitude_Roll_Controller_2.p_meas_2
            )
        ) / 1024.0
    );

    $display(
        "Unit_Delay1_out1      = %0d  (%f)",
        $signed(
            uut.u_Attitude_Roll_Controller_2.Unit_Delay1_out1
        ),
        $itor(
            $signed(
                uut.u_Attitude_Roll_Controller_2.Unit_Delay1_out1
            )
        ) / 1024.0
    );

    $display(
        "Sum_out1              = %0d  (%f)",
        $signed(
            uut.u_Attitude_Roll_Controller_2.Sum_out1
        ),
        $itor(
            $signed(
                uut.u_Attitude_Roll_Controller_2.Sum_out1
            )
        ) / 1024.0
    );

    $display(
        "p_error_out1          = %0d  (%f)",
        $signed(
            uut.u_Attitude_Roll_Controller_2.p_error_out1
        ),
        $itor(
            $signed(
                uut.u_Attitude_Roll_Controller_2.p_error_out1
            )
        ) / 1024.0
    );

    $display(
        "p_error_out1_held     = %0d  (%f)",
        $signed(
            uut.u_Attitude_Roll_Controller_2.p_error_out1_held
        ),
        $itor(
            $signed(
                uut.u_Attitude_Roll_Controller_2.p_error_out1_held
            )
        ) / 1024.0
    );


    $display("");
    $display("[P / I / D INTERNALS]");

    $display(
        "Inner_P_out1_2        = %0d  (%f)",
        $signed(
            uut.u_Attitude_Roll_Controller_2.Inner_P_out1_2
        ),
        $itor(
            $signed(
                uut.u_Attitude_Roll_Controller_2.Inner_P_out1_2
            )
        ) / 1024.0
    );

    $display(
        "Inner_D_out1_1        = %0d  (%f)",
        $signed(
            uut.u_Attitude_Roll_Controller_2.Inner_D_out1_1
        ),
        $itor(
            $signed(
                uut.u_Attitude_Roll_Controller_2.Inner_D_out1_1
            )
        ) / 1024.0
    );

    $display(
        "Discrete_Time_Integrator_x_reg = %0d (%f)",
        $signed(
            uut.u_Attitude_Roll_Controller_2.Discrete_Time_Integrator_x_reg
        ),
        $itor(
            $signed(
                uut.u_Attitude_Roll_Controller_2.Discrete_Time_Integrator_x_reg
            )
        ) / 1024.0
    );

    $display(
        "Discrete_Time_Integrator_u_dtc = %0d (%f)",
        $signed(
            uut.u_Attitude_Roll_Controller_2.Discrete_Time_Integrator_u_dtc
        ),
        $itor(
            $signed(
                uut.u_Attitude_Roll_Controller_2.Discrete_Time_Integrator_u_dtc
            )
        ) / 1024.0
    );


    $display("");
    $display("[FINAL U2 PATH]");

    $display(
        "U2_before_sat         = %0d  (%f)",
        $signed(
            uut.u_Attitude_Roll_Controller_2.U2_before_sat
        ),
        $itor(
            $signed(
                uut.u_Attitude_Roll_Controller_2.U2_before_sat
            )
        ) / 1024.0
    );

    $display(
        "Saturation_out1       = %0d  (%f)",
        $signed(
            uut.u_Attitude_Roll_Controller_2.Saturation_out1
        ),
        $itor(
            $signed(
                uut.u_Attitude_Roll_Controller_2.Saturation_out1
            )
        ) / 1024.0
    );

    $display(
        "t_bypass_reg          = %0d  (%f)",
        $signed(
            uut.u_Attitude_Roll_Controller_2.t_bypass_reg
        ),
        $itor(
            $signed(
                uut.u_Attitude_Roll_Controller_2.t_bypass_reg
            )
        ) / 1024.0
    );

    $display(
        "U2                    = %0d  (%f)",
        $signed(
            uut.u_Attitude_Roll_Controller_2.U2
        ),
        $itor(
            $signed(
                uut.u_Attitude_Roll_Controller_2.U2
            )
        ) / 1024.0
    );

    $display("");
    $display("============================================================");

end

endtask


//==========================================================
// COMPLETE U2 DIAGNOSTIC TASK
//==========================================================

task automatic diagnose_u2_all;

begin

    if (diag_initialized !== 1) begin

        diag_p_prev =
            0.0;

        diag_rate_int =
            0.0;

        diag_rate_int_1dly =
            0.0;

        diag_initialized =
            1;

    end


    diag_u2_tb_ref =
        u2_ref;


    diag_phi_dut =
        sfix_to_real(
            phi_filt_dbg_en14,
            14
        );

    diag_phi_cmd_dut =
        sfix32_to_real(
            phi_cmd,
            29
        );

    diag_p_meas =
        sfix_to_real(
            p_filt_dbg_en10,
            10
        );


    diag_phi_error =
        diag_phi_cmd_dut
        -
        diag_phi_dut;


    diag_p_cmd =
        Kp_attitude
        *
        diag_phi_error
        +
        Ki_attitude
        *
        0.0;


    diag_p_dot =
        (
            diag_p_meas
            -
            diag_p_prev
        )
        /
        Ts_rate;


    diag_ep =
        diag_p_cmd
        -
        diag_p_meas;


    diag_ep_1dly =
        diag_p_cmd
        -
        diag_p_prev;


    diag_rate_int =
        diag_rate_int
        +
        diag_ep
        *
        Ts_rate;


    diag_rate_int_1dly =
        diag_rate_int_1dly
        +
        diag_ep_1dly
        *
        Ts_rate;


    diag_u2_dutstate =
          Ki_rate
          *
          diag_rate_int

        -
          Kp_rate
          *
          diag_p_meas

        -
          Kd_rate
          *
          diag_p_dot;


    diag_u2_dutstate_1dly =
          Ki_rate
          *
          diag_rate_int_1dly

        -
          Kp_rate
          *
          diag_p_prev

        -
          Kd_rate
          *
          diag_p_dot;


    diag_u2_actual =
        sfix_to_real(
            U2,
            10
        );


    diag_P =
        sfix_to_real(
            uut.u_Attitude_Roll_Controller_2.Inner_P_out1_2,
            10
        );


    diag_I =
        sfix_to_real(
            uut.u_Attitude_Roll_Controller_2.Discrete_Time_Integrator_u_dtc,
            10
        );


    diag_D =
        sfix_to_real(
            uut.u_Attitude_Roll_Controller_2.Inner_D_out1_1,
            10
        );


    diag_U2_before_sat =
        sfix_to_real(
            uut.u_Attitude_Roll_Controller_2.U2_before_sat,
            10
        );


    diag_U2_sat =
        sfix_to_real(
            uut.u_Attitude_Roll_Controller_2.Saturation_out1,
            10
        );


    diag_err_dut_vs_tb =
        diag_u2_actual
        -
        diag_u2_tb_ref;


    diag_err_dut_vs_dutstate =
        diag_u2_actual
        -
        diag_u2_dutstate;


    diag_err_dut_vs_dutstate_1dly =
        diag_u2_actual
        -
        diag_u2_dutstate_1dly;


    `LOG0("")
    `LOG0("================================================================")
    `LOG0("                 COMPLETE U2 DIAGNOSTIC")
    `LOG0("================================================================")

    `LOG1(
        "TIME                         = %0t",
        $time
    )

    `LOG1(
        "DUT sample counter s_s      = %0d",
        uut.u_Attitude_Roll_Controller_2.s_s
    )


    `LOG0("")
    `LOG0("[DUT INPUTS]")

    `LOG1(
        "phi_raw_req                 = %0.9f",
        sfix_to_real(
            uut.u_Attitude_Roll_Controller_2.phi_raw_req,
            14
        )
    )

    `LOG1(
        "phi_meas                    = %0.9f",
        sfix_to_real(
            uut.u_Attitude_Roll_Controller_2.phi_meas,
            14
        )
    )

    `LOG1(
        "p_meas                      = %0.9f",
        diag_p_meas
    )


    `LOG0("")
    `LOG0("[EXISTING TB REFERENCE]")

    `LOG1(
        "TB phi command              = %0.9f",
        phi_cmd_ref_tbstates
    )

    `LOG1(
        "TB phi state                = %0.9f",
        phi_ctrl_tb
    )

    `LOG1(
        "TB p command                = %0.9f",
        p_cmd_ref
    )

    `LOG1(
        "TB p measured               = %0.9f",
        p_meas_ref
    )

    `LOG1(
        "TB U2 reference             = %0.9f",
        diag_u2_tb_ref
    )


    `LOG0("")
    `LOG0("[DUT-STATE RECONSTRUCTION]")

    `LOG1(
        "DUT phi command             = %0.9f",
        diag_phi_cmd_dut
    )

    `LOG1(
        "DUT phi state               = %0.9f",
        diag_phi_dut
    )

    `LOG1(
        "DUT phi error               = %0.9f",
        diag_phi_error
    )

    `LOG1(
        "DUT-state p command         = %0.9f",
        diag_p_cmd
    )

    `LOG1(
        "DUT p measured              = %0.9f",
        diag_p_meas
    )

    `LOG1(
        "DUT p previous              = %0.9f",
        diag_p_prev
    )

    `LOG1(
        "DUT p derivative            = %0.9f",
        diag_p_dot
    )

    `LOG1(
        "DUT-state p error           = %0.9f",
        diag_ep
    )

    `LOG1(
        "DUT-state U2 reference      = %0.9f",
        diag_u2_dutstate
    )


    `LOG0("")
    `LOG0("[ONE-SAMPLE-DELAYED DUT REFERENCE]")

    `LOG1(
        "Delayed p error             = %0.9f",
        diag_ep_1dly
    )

    `LOG1(
        "Delayed U2 reference        = %0.9f",
        diag_u2_dutstate_1dly
    )


    `LOG0("")
    `LOG0("[ACTUAL RTL INTERNAL P/I/D]")

    `LOG1(
        "Inner_P_out1_2              = %0.9f",
        diag_P
    )

    `LOG1(
        "Integrator u_dtc            = %0.9f",
        diag_I
    )

    `LOG1(
        "Inner_D_out1_1              = %0.9f",
        diag_D
    )

    `LOG1(
        "RTL U2_before_sat           = %0.9f",
        diag_U2_before_sat
    )

    `LOG1(
        "RTL saturation output       = %0.9f",
        diag_U2_sat
    )


    `LOG0("")
    `LOG0("[ACTUAL DUT OUTPUT]")

    `LOG1(
        "Actual DUT U2               = %0.9f",
        diag_u2_actual
    )


    `LOG0("")
    `LOG0("[U2 DIFFERENCES]")

    `LOG1(
        "DUT - TB reference          = %0.9f",
        diag_err_dut_vs_tb
    )

    `LOG1(
        "DUT - DUT-state reference   = %0.9f",
        diag_err_dut_vs_dutstate
    )

    `LOG1(
        "DUT - DUT+1cycle reference  = %0.9f",
        diag_err_dut_vs_dutstate_1dly
    )


    `LOG0("")
    `LOG0("[RTL INTERNAL CONSISTENCY]")

    `LOG1(
        "U2_before_sat               = %0.9f",
        diag_U2_before_sat
    )

    `LOG1(
        "Saturation_out1             = %0.9f",
        diag_U2_sat
    )

    `LOG1(
        "Actual U2                   = %0.9f",
        diag_u2_actual
    )

    `LOG1(
        "Output - saturation         = %0.9f",
        diag_u2_actual - diag_U2_sat
    )


    diag_p_prev =
        diag_p_meas;


    `LOG0("")
    `LOG0("================================================================")
    `LOG0("              END COMPLETE U2 DIAGNOSTIC")
    `LOG0("================================================================")
    `LOG0("")

end

endtask

endmodule
