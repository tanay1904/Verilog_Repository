`timescale 1 ns / 1 ns

module tb_Position_Controller;

    // ============================================================
    // Inputs
    // ============================================================
    reg clk;
    reg reset;
    reg clk_enable;

    reg signed [15:0] Psi;          // sfix16_En14
    reg signed [15:0] x_setpoint;   // sfix16_En10
    reg signed [15:0] y_setpoint;   // sfix16_En10
    reg signed [15:0] U_est;        // sfix16_En10
    reg signed [15:0] V_est;        // sfix16_En10
    reg signed [15:0] y_est;        // sfix16_En10
    reg signed [15:0] x_est;        // sfix16_En10
    reg signed [15:0] W_est;        // sfix16_En10
    reg signed [15:0] W_cmd;        // sfix16_En10
    reg signed [15:0] phi_k_1;      // sfix16_En14
    reg signed [15:0] theta_k_1;    // sfix16_En14

    // ============================================================
    // Outputs
    // ============================================================
    wire ce_out;
    wire signed [15:0] theta;       // sfix16_En14
    wire signed [31:0] phi;         // sfix32_En29

    // ============================================================
    // Display / debug variables
    // ============================================================
    real time_ms;
    real theta_deg_hdl, phi_deg_hdl;
    real theta_deg_ref, phi_deg_ref;
    real x_est_real, y_est_real;
    real theta_hdl_r, phi_hdl_r;
    real theta_ref_r, phi_ref_r;
    real theta_err_r, phi_err_r;

    integer i;
    real sweep_y;
    wire resetn = ~reset;
    // ============================================================
    // UUT
    // ============================================================
    Position_Controller_2 uut (
        .clk(clk),
        .resetn(resetn),
        .resetn_Position_Controller_2_tc(resetn),
        .clk_enable(clk_enable),
        .Psi(Psi),
        .x_setpoint(x_setpoint),
        .y_setpoint(y_setpoint),
        .U_est(U_est),
        .V_est(V_est),
        .y_est(y_est),
        .x_est(x_est),
        .W_est(W_est),
        .W_cmd(W_cmd),
        .phi_k_1(phi_k_1),
        .theta_k_1(theta_k_1),
        .ce_out(ce_out),
        .theta(theta),
        .phi(phi)
    );

    // ============================================================
    // Fixed-point helpers
    // ============================================================
    function automatic signed [15:0] to_sfix_EnX(input real val, input integer frac_bits);
        integer tmp;
        begin
            tmp = $rtoi(val * (2.0 ** frac_bits));
            if (tmp > 32767)
                to_sfix_EnX = 16'sh7FFF;
            else if (tmp < -32768)
                to_sfix_EnX = 16'sh8000;
            else
                to_sfix_EnX = tmp[15:0];
        end
    endfunction

    function automatic real sfix_to_real(input signed [31:0] v, input integer frac_bits);
        begin
            sfix_to_real = $itor(v) / (2.0 ** frac_bits);
        end
    endfunction

    function automatic real abs_real(input real x);
        begin
            abs_real = (x < 0.0) ? -x : x;
        end
    endfunction

    function automatic real clamp_real(input real x, input real lo, input real hi);
        begin
            if (x < lo)
                clamp_real = lo;
            else if (x > hi)
                clamp_real = hi;
            else
                clamp_real = x;
        end
    endfunction

    // ============================================================
    // Reference model state
    // NOTE: behavioral / unit-test reference, not bit-true
    // ============================================================
    real PI;
    real Ts;

    real POS_AW;
    real VEL_KP;
    real VEL_AW;
    real VEL_KI;
    real POS_LIM;
    real ANG_LIM;

    real x_sp_r, y_sp_r;
    real x_est_r, y_est_r;
    real U_est_r, V_est_r, W_est_r, W_cmd_r;
    real Psi_r, phi_k_1_r, theta_k_1_r;

    real pos_int_x_r, pos_int_y_r;
    real vel_int_x_r, vel_int_y_r;

    real evx_d0, evx_d1, evx_d2, evx_d3, evx_d4, evx_d5;
    real evy_d0, evy_d1, evy_d2, evy_d3, evy_d4, evy_d5;
    real evz_d0, evz_d1, evz_d2, evz_d3, evz_d4, evz_d5;

    real theta_unsat_r, phi_unsat_r;
    real vel_x_des_r, vel_y_des_r;
    real body_x_err, body_y_err;

    // ============================================================
    // Clock generation: 4 ms period
    // ============================================================
    initial begin
        clk = 1'b0;
        forever #50 clk = ~clk;   // 2 ms half-period
    end

    // ============================================================
    // Reference model update on each clock
    // ============================================================
    real ex, ey;
    real vel_x_unsat, vel_y_unsat;
    real evx, evy, evz;
    real cth, sth, cpsi, spsi, cphi, sphi;
    always @(posedge clk or posedge reset) begin


        if (reset) begin
            pos_int_x_r <= 0.0;
            pos_int_y_r <= 0.0;
            vel_int_x_r <= 0.0;
            vel_int_y_r <= 0.0;

            evx_d0 <= 0.0; evx_d1 <= 0.0; evx_d2 <= 0.0; evx_d3 <= 0.0; evx_d4 <= 0.0; evx_d5 <= 0.0;
            evy_d0 <= 0.0; evy_d1 <= 0.0; evy_d2 <= 0.0; evy_d3 <= 0.0; evy_d4 <= 0.0; evy_d5 <= 0.0;
            evz_d0 <= 0.0; evz_d1 <= 0.0; evz_d2 <= 0.0; evz_d3 <= 0.0; evz_d4 <= 0.0; evz_d5 <= 0.0;

            theta_ref_r <= 0.0;
            phi_ref_r   <= 0.0;
        end
        else if (clk_enable) begin
            // Convert current HDL inputs to real
            x_sp_r      = sfix_to_real({{16{x_setpoint[15]}}, x_setpoint}, 10);
            y_sp_r      = sfix_to_real({{16{y_setpoint[15]}}, y_setpoint}, 10);
            x_est_r     = sfix_to_real({{16{x_est[15]}}, x_est}, 10);
            y_est_r     = sfix_to_real({{16{y_est[15]}}, y_est}, 10);
            U_est_r     = sfix_to_real({{16{U_est[15]}}, U_est}, 10);
            V_est_r     = sfix_to_real({{16{V_est[15]}}, V_est}, 10);
            W_est_r     = sfix_to_real({{16{W_est[15]}}, W_est}, 10);
            W_cmd_r     = sfix_to_real({{16{W_cmd[15]}}, W_cmd}, 10);
            Psi_r       = sfix_to_real({{16{Psi[15]}}, Psi}, 14);
            phi_k_1_r   = sfix_to_real({{16{phi_k_1[15]}}, phi_k_1}, 14);
            theta_k_1_r = sfix_to_real({{16{theta_k_1[15]}}, theta_k_1}, 14);

            // Position loop
            ex = x_sp_r - x_est_r;
            ey = y_sp_r - y_est_r;

            vel_x_unsat = ex + pos_int_x_r;
            vel_y_unsat = ey + pos_int_y_r;

            vel_x_des_r = clamp_real(vel_x_unsat, -POS_LIM, POS_LIM);
            vel_y_des_r = clamp_real(vel_y_unsat, -POS_LIM, POS_LIM);

            pos_int_x_r = pos_int_x_r + Ts * (ex + POS_AW * (vel_x_des_r - vel_x_unsat));
            pos_int_y_r = pos_int_y_r + Ts * (ey + POS_AW * (vel_y_des_r - vel_y_unsat));

            // Velocity error
            evx = vel_x_des_r - U_est_r;
            evy = vel_y_des_r - V_est_r;
            evz = W_cmd_r - W_est_r;

            // 6-sample delay lines to reflect HDL structure
            evx_d5 = evx_d4; evx_d4 = evx_d3; evx_d3 = evx_d2; evx_d2 = evx_d1; evx_d1 = evx_d0; evx_d0 = evx;
            evy_d5 = evy_d4; evy_d4 = evy_d3; evy_d3 = evy_d2; evy_d2 = evy_d1; evy_d1 = evy_d0; evy_d0 = evy;
            evz_d5 = evz_d4; evz_d4 = evz_d3; evz_d3 = evz_d2; evz_d2 = evz_d1; evz_d1 = evz_d0; evz_d0 = evz;

            evx = evx_d5;
            evy = evy_d5;
            evz = evz_d5;

            // Earth/body rotation using previous-sample attitude
            cth  = $cos(theta_k_1_r);
            sth  = $sin(theta_k_1_r);
            cpsi = $cos(Psi_r);
            spsi = $sin(Psi_r);
            cphi = $cos(phi_k_1_r);
            sphi = $sin(phi_k_1_r);

            body_x_err = (evx * cth * cpsi) + (evy * cth * spsi) - (evz * sth);

            body_y_err = (evx * (sth * sphi * cpsi - cphi * spsi)) +
                         (evy * (sth * sphi * spsi + cphi * cpsi)) +
                         (evz * (sphi * cth));

            // Velocity loop
            theta_unsat_r = VEL_KI * vel_int_x_r + VEL_KP * body_x_err;
            phi_unsat_r   = VEL_KI * vel_int_y_r + VEL_KP * body_y_err;

            theta_ref_r   = clamp_real(theta_unsat_r, -ANG_LIM, ANG_LIM);
            phi_ref_r     = clamp_real(phi_unsat_r,   -ANG_LIM, ANG_LIM);

            vel_int_x_r   = vel_int_x_r + Ts * (body_x_err + VEL_AW * (theta_ref_r - theta_unsat_r));
            vel_int_y_r   = vel_int_y_r + Ts * (body_y_err + VEL_AW * (phi_ref_r   - phi_unsat_r));

            // HDL outputs: theta direct, phi negated and widened
            phi_ref_r = -phi_ref_r;
        end
    end

    // ============================================================
    // Compare / print task
    // ============================================================
    task automatic report_sample;
        real pi_local;
        begin
            pi_local = PI;

            time_ms = $time / 1000000.0;

            x_est_real = $itor({{16{x_est[15]}}, x_est}) / (2.0 ** 10);
            y_est_real = $itor({{16{y_est[15]}}, y_est}) / (2.0 ** 10);

            theta_hdl_r = $itor({{16{theta[15]}}, theta}) / (2.0 ** 14);
            phi_hdl_r   = $itor(phi) / (2.0 ** 29);

            theta_deg_hdl = theta_hdl_r * (180.0 / pi_local);
            phi_deg_hdl   = phi_hdl_r   * (180.0 / pi_local);

            theta_deg_ref = theta_ref_r * (180.0 / pi_local);
            phi_deg_ref   = phi_ref_r   * (180.0 / pi_local);

            theta_err_r = abs_real(theta_hdl_r - theta_ref_r);
            phi_err_r   = abs_real(phi_hdl_r   - phi_ref_r);

            $display("--------------------------------------------------------------------------------------------");
            $display("Time(ms) | Yest  | Yref  | Phi_hdl  | Phi_ref  | Xest  | Xref  | Theta_hdl | Theta_ref");
$display("%4.0f ms  | %4.2f  | %4.2f  | %8.3f° | %8.3f° | %4.2f  | %4.2f  | %8.3f° | %8.3f°",
         time_ms, y_est_real, sfix_to_real({{16{y_setpoint[15]}}, y_setpoint}, 10),
         phi_deg_hdl, phi_deg_ref, x_est_real,
         sfix_to_real({{16{x_setpoint[15]}}, x_setpoint}, 10),
         theta_deg_hdl, theta_deg_ref);
        $display("time=%0t ce_out=%b phi=%0d theta=%0d", $time, ce_out, phi, theta);
            if ((theta_err_r > 0.05) || (phi_err_r > 0.05)) begin
                $display("COMPARE: FAIL  theta_err=%f rad  phi_err=%f rad", theta_err_r, phi_err_r);
            end
            else begin
                $display("COMPARE: PASS  theta_err=%f rad  phi_err=%f rad", theta_err_r, phi_err_r);
            end
        end
    endtask

    // ============================================================
    // Stimulus
    // Unit test: keep attitude fixed, sweep Y only
    // ============================================================
    initial begin
        // Constants
        PI = 3.14159265358979323846;
        Ts = 0.004;

        // Values inferred from the generated HDL
        POS_AW  = 1.25;
        VEL_KP  = 0.509765625;
        VEL_AW  = 0.32000732421875;
        VEL_KI =  1.0;
        POS_LIM = 10.0;
        ANG_LIM = 0.52099609375;

        // Init inputs
        clk_enable = 1'b1;
        reset      = 1'b1;

        Psi        = 16'sd0;
        x_setpoint = 16'sd0;
        y_setpoint = 16'sd0;
        U_est      = 16'sd0;
        V_est      = 16'sd0;
        x_est      = 16'sd0;
        y_est      = 16'sd0;
        W_est      = 16'sd0;
        W_cmd      = 16'sd0;
        phi_k_1    = 16'sd0;
        theta_k_1  = 16'sd0;

        // Wait one cycle for reset to propagate
        #4000000;
        reset = 1'b0;

        $display("-----------------------------------------------------------------");
        $display("Unit test: sweep Y only, keep Psi/phi/theta fixed at zero");
        $display("-----------------------------------------------------------------");

        // Set target
        x_setpoint = to_sfix_EnX(0.0, 10);
        y_setpoint = to_sfix_EnX(2.0, 10);
        x_est      = to_sfix_EnX(0.0, 10);

        // Initial compare
        #1000000;
        report_sample();

        // Sweep y_est from 0.0 m to 2.0 m in 0.025 m steps (80 steps)
        for (i = 0; i <= 80; i = i + 1) begin
            sweep_y = i * 0.025;
            y_est = to_sfix_EnX(sweep_y, 10);
//            v_est = t_sfix_EnX();
            // Let the controller run for 10 ms at this stimulus point
            #10000000;
            report_sample();
        end

        $display("-----------------------------------------------------------------");
        $stop;
    end

endmodule