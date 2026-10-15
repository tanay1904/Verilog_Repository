`timescale 1 ns / 1 ns

module tb_Complementary_filter;

    // Inputs
    reg clk;
    reg reset;
    reg clk_enable;
    reg signed [15:0] gx;
    reg signed [15:0] gy;
    reg signed [15:0] gz;
    reg signed [15:0] ax;
    reg signed [15:0] ay;
    reg signed [15:0] az;
    reg signed [15:0] Alpha1;
    reg signed [15:0] alpha1_Alpha_1;
    reg signed [15:0] alpha1_Alpha_2;
    reg signed [15:0] Alpha2;
    reg [11:0] dt;

    // Outputs
    wire ce_out;
    wire signed [15:0] phi;
    wire signed [16:0] theta;
    wire signed [15:0] psi_filt;
    wire signed [15:0] p_filt;
    wire signed [15:0] q_filt;
    wire signed [15:0] r_filt;

    wire resetn = ~reset;
    // Display Variables
    real time_s;
    real phi_deg, theta_deg, psi_deg;
    real p_filt_real, q_filt_real;

    // Test phase tracking - set by stimulus process, read by display process
    integer test_phase;
    reg [255:0] phase_name;
    
    reg [15:0] V_dot, U_dot, W_dot;
    initial begin
     U_dot = 0;
     V_dot = 0;
     W_dot = 0;
    end
    // Instantiate the Unit Under Test (UUT)
    Attitude_filter10_Attitude_Filter uut (
        .clk(clk),
        .resetn(resetn),
        .resetn_Attitude_filter10_tc(resetn),
        .clk_enable(clk_enable),
        .gx(gx),
        .gy(gy),
        .gz(gz),
        .ax(ax),
        .ay(ay),
        .az(az),
        .dt(dt),
        .U_dot(U_dot),
        .V_dot(V_dot),
        .W_dot(W_dot),
        .Alpha_1(Alpha1),
        .Alpha_2(Alpha2),
        .alpha1_Alpha_2(alpha1_Alpha_2),
        .alpha1_Alpha_1(alpha1_Alpha_1),
        .ce_out(ce_out),
        .phi(phi),
        .theta(theta),
        .p_filt(p_filt),
        .q_filt(q_filt),
        .psi_filt(psi_filt),
        .r_filt(r_filt)
    );

    // --- Scaling Helper Functions ---
    function signed [15:0] to_sfix16_En10(input real val);
        begin to_sfix16_En10 = val * 1024.0; end
    endfunction

    function signed [15:0] to_sfix16_EnX(input real val, input integer frac_bits);
        begin to_sfix16_EnX = val * (2**frac_bits); end
    endfunction

    function [11:0] to_ufix12_En12(input real val);
        begin to_ufix12_En12 = val * 4096.0; end
    endfunction

    // Clock Generation: 100 ns period -> matches "Model base rate" (1e-7 s)
    initial begin
        clk = 0;
        forever #50 clk = ~clk;
    end

    initial begin
        Alpha1         = to_sfix16_EnX(0.992, 14);
        alpha1_Alpha_1 = to_sfix16_EnX(1 - 0.992, 13);
        Alpha2         = to_sfix16_EnX(1, 10);
        alpha1_Alpha_2 = to_sfix16_EnX(0, 10);
    end

    // Sets the phase label AND prints a marker the instant stimulus changes
    task set_phase(input integer phase_num, input [255:0] name);
        begin
            test_phase = phase_num;
            phase_name = name;
            $display(">> [%7.4f s] Stimulus change -> Phase %0d: %0s",
                      $realtime / 1_000_000_000.0, phase_num, name);
        end
    endtask

    // Stimulus Process
    initial begin
        clk_enable = 1;
        dt = to_sfix16_EnX(0.004, 12); // Constant dt = 4ms

        gx = 0; gy = 0; gz = 0;
        ax = 0; ay = 0; az = 0;
        test_phase = 0;
        phase_name = "Reset";

        $display("----------------------------------------------------------------------------------");
        $display("Time(s) | Test Phase               | Phi (deg) | Theta (deg) | Psi (deg) | p_filt  | q_filt ");
        $display("----------------------------------------------------------------------------------");
        // -----------------------------------------------------------
        // Test 0: Reset Initialization
        // -----------------------------------------------------------
        reset = 1;
        #12_000_000; // Hold reset for 12ms
        reset = 0;

        // -----------------------------------------------------------
        // Test 1: Level (Static Attitude)
        // -----------------------------------------------------------
        set_phase(1, "Level");
        ax = to_sfix16_EnX(0.0, 10);
        ay = to_sfix16_EnX(0.0, 10);
        az = to_sfix16_EnX(9.81, 10);
        gx = to_sfix16_EnX(0.0, 10);
        gy = to_sfix16_EnX(0.0, 10);
        #5_000_000;

//        // -----------------------------------------------------------
//        // Test 2: Roll 30 degrees
//        // -----------------------------------------------------------
//        set_phase(2, "Roll 30 deg");
//        ax = to_sfix16_EnX(0.0, 10);
//        ay = to_sfix16_EnX(0.5, 10);
//        az = to_sfix16_EnX(0.866, 10);
//        #500_000_000;

//        // -----------------------------------------------------------
//        // Test 3: Pitch 20 degrees
//        // -----------------------------------------------------------
//        set_phase(3, "Pitch 20 deg");
//        ax = to_sfix16_EnX(-0.342, 10);
//        ay = to_sfix16_EnX(0.0, 10);
//        az = to_sfix16_EnX(0.939, 10);
//        #500_000_000;

        // -----------------------------------------------------------
        // Test 4: Gyro Integration (Constant gx)
        // -----------------------------------------------------------
        set_phase(4, "Gyro Int (gx=0.1rad/s)");
        ax = to_sfix16_EnX(0.0, 10);
        ay = to_sfix16_EnX(0.0, 10);
        az = to_sfix16_EnX(1.0, 10);
        gx = to_sfix16_EnX(1.0, 10);
        #500_000_000;
        
        set_phase(5, "Gyro Int (gz=0.1rad/s)");
        ax = to_sfix16_EnX(0.0, 10);
        ay = to_sfix16_EnX(0.0, 10);
        az = to_sfix16_EnX(1.0, 10);
        gx = to_sfix16_EnX(-0.1, 10);
        gz = to_sfix16_EnX(0.1, 10);
        #500_000_000;        

        // -----------------------------------------------------------
        // Test 5: Low-pass filter step response
        // -----------------------------------------------------------
        set_phase(5, "LPF Step (gx=30)");
        gx = to_sfix16_EnX(0.523599, 10);
        #500_000_000;

        // -----------------------------------------------------------
        // Test 6: Noise Rejection
        // -----------------------------------------------------------
        set_phase(6, "Noise Rejection");
        gx = to_sfix16_EnX(0.0, 10);

        ax = to_sfix16_EnX(0.02, 10);
        #50_000_000;
        ax = to_sfix16_EnX(-0.02, 10);
        #50_000_000;
        ax = to_sfix16_EnX(0.02, 10);
        #50_000_000;
        ax = to_sfix16_EnX(0.00, 10);
        #50_000_000;

//        // -----------------------------------------------------------
//        // Test 7: Long Term Stability (Hover)
//        // -----------------------------------------------------------
//        set_phase(7, "Long Term Stability");
//        ax = to_sfix16_En10(0.0);
//        ay = to_sfix16_En10(0.0);
//        az = to_sfix16_En10(1.0);
//        #2_000_000_000;

        $display("----------------------------------------------------------------------------------");
        $stop;
    end

    // --- Console Output Logic ---
    // Print only when phi or theta actually change value (4-state compare
    // so X right after reset doesn't cause a spurious first print).
reg signed [15:0] phi_prev;
    reg signed [16:0] theta_prev;
    reg signed [15:0] psi_prev;
    reg first_sample = 1;

    always @(posedge clk) begin
        if (first_sample || phi !== phi_prev || theta !== theta_prev || psi_filt !== psi_prev) begin
            phi_prev     <= phi;
            theta_prev   <= theta;
            psi_prev     <= psi_filt;
            first_sample <= 0;

            time_s = $realtime / 1_000_000_000.0;

            phi_deg     = ($itor(phi)      / $itor(2**14)) * (180.0 / 3.1415926535);
            theta_deg   = ($itor(theta)    / $itor(2**10)) * (180.0 / 3.1415926535);
            psi_deg     = ($itor(psi_filt) / $itor(2**12)) * (180.0 / 3.1415926535);
            p_filt_real = $itor(p_filt) / 1024.0;
            q_filt_real = $itor(q_filt) / 1024.0;

            $display("%7.4f s | %-24s | %9.3f | %11.3f | %9.3f | %7.2f | %6.2f | %0d ",
                      time_s, phase_name, phi_deg, theta_deg, psi_deg, p_filt_real, q_filt_real, psi_filt);
        end
    end    
//    integer hb_count = 0;
//    always @(posedge clk) begin
//        hb_count = hb_count + 1;
//        if (hb_count >= 100) begin   // print every 10us - fine enough to catch a 4ms period cleanly
//            hb_count = 0;
//            $display("[hb %8.5f s] resetn=%b s_s=%0d count40000=%0d ce_out=%b phase_1=%b Add_out1_1=%0d",
//                       $realtime/1_000_000_000.0,
//                       uut.resetn, uut.s_s,
//                       uut.Attitude_filter10_tc.count40000,
//                       uut.ce_out,
//                       uut.Attitude_filter10_tc.phase_1,
//                       uut.Add_out1_1);
//        end
//    end
//    always @(posedge clk) begin
//    if (uut.quotient_2 === 16'bx || uut.Add_out1_1 === 16'sbx) begin
//        $display("[X-trace %0.5f s] validIn=%b num=%0d den=%0d quotient_2=%b  Add_out1_1=%b",
//                   $realtime/1e9,
//                   uut.TmpGroundAtReal_Divide_HDL_Optimized_1,
//                   uut.Trigonometric_Function_out1,
//                   uut.Trigonometric_Function_out2,
//                   uut.quotient_2,
//                   uut.Add_out1_1);
//    end
//end
//    reg last_xtrace_valid = 0;
//    real last_xtrace_time = -1;

//    always @(posedge clk) begin
//        if (uut.Add_out1_1 === 16'sbx) begin
//            if ($realtime/1_000_000_000.0 != last_xtrace_time) begin
//                last_xtrace_time = $realtime/1_000_000_000.0;
//                $display("[X-trace %0.5f s] Product16_out1_1=%b Product17_out1_1=%b | Product16_out1=%b Product18_out1=%b Sum1_out1_3=%b Alpha_1_3=%b | Product17_out1=%b Product17_out1_2=%b phi_acc_out1_2=%b alpha1_Alpha_1_7=%b",
//                           $realtime/1_000_000_000.0,
//                           uut.Product16_out1_1, uut.Product17_out1_1,
//                           uut.Product16_out1, uut.Product18_out1, uut.Sum1_out1_3, uut.Alpha_1_3,
//                           uut.Product17_out1, uut.Product17_out1_2, uut.phi_acc_out1_2, uut.alpha1_Alpha_1_7);
//            end
//        end
//    end
    real last_xtrace2_time = -1;
    always @(posedge clk) begin
        if (uut.alpha1_Alpha_1_7 === 16'sbx) begin
            if ($realtime/1_000_000_000.0 != last_xtrace2_time) begin
                last_xtrace2_time = $realtime/1_000_000_000.0;
                $display("[X2 %0.5f s] counterSig_4=%b | alpha1_Alpha_1_5=%b gz_3=%b gy_5=%b theta_acc_out1_1=%b | Gain_out1=%b Sqrt_out1=%b Add2_out1=%b Add3_out1=%b",
                           $realtime/1_000_000_000.0, uut.counterSig_4,
                           uut.alpha1_Alpha_1_5, uut.gz_3, uut.gy_5, uut.theta_acc_out1_1,
                           uut.Gain_out1, uut.Sqrt_out1, uut.Add2_out1, uut.Add3_out1);
            end
        end
    end
    real last_xtrace3_time = -1;
        always @(posedge clk) begin
        if (uut.Add3_out1 === 32'sbx || uut.Add2_out1 === 32'sbx) begin
            if ($realtime/1_000_000_000.0 != last_xtrace3_time) begin
                last_xtrace3_time = $realtime/1_000_000_000.0;
                $display("[X3 %0.5f s] ax=%b ay=%b az=%b | ax_1=%b ay_1=%b az_1=%b",
                           $realtime/1_000_000_000.0,
                           uut.ax, uut.ay, uut.az,
                           uut.ax_1, uut.ay_1, uut.az_1);
            end
        end
    end
    real last_xtrace4_time = -1;

    always @(posedge clk) begin
        if (uut.Add3_add_temp === 17'sbx) begin
            if ($realtime/1_000_000_000.0 != last_xtrace4_time) begin
                last_xtrace4_time = $realtime/1_000_000_000.0;
                $display("[X4 %0.5f s] counterSig_11=%b | Product12_out1_3=%b Product13_out1_2=%b | Product12_out1_2=%b Product14_out1_1=%b Math_Function1_out1_1=%b | Add3_add_cast=%b Add3_add_cast_1=%b",
                           $realtime/1_000_000_000.0, uut.counterSig_11,
                           uut.Product12_out1_3, uut.Product13_out1_2,
                           uut.Product12_out1_2, uut.Product14_out1_1, uut.Math_Function1_out1_1,
                           uut.Add3_add_cast, uut.Add3_add_cast_1);
            end
        end
    end
real last_xtrace5_time = -1;

always @(posedge clk) begin
    if (uut.Add3_add_temp === 17'sbx) begin
        if ($realtime/1_000_000_000.0 != last_xtrace5_time) begin
            last_xtrace5_time = $realtime/1_000_000_000.0;

                     //"Math Function path"
$display("[X5 %0.5f s] c0_serialOut_0_3[0]=%b c0_serialOut_0_3[1]=%b c0_serialOut_0_3[2]=%b | tappedDelay_reg_2[0]=%b tappedDelay_reg_2[1]=%b tappedDelay_reg_2[2]=%b",
         $realtime/1_000_000_000.0,
         uut.c0_serialOut_0_3[0],
         uut.c0_serialOut_0_3[1],
         uut.c0_serialOut_0_3[2],
         uut.tappedDelay_reg_2[0],
         uut.tappedDelay_reg_2[1],
         uut.tappedDelay_reg_2[2]);         
         end
    end
end
real last_xtrace7_time = -1;

always @(posedge clk) begin
    if (uut.Product8_out1_2 === 16'sbx) begin
        if ($realtime/1_000_000_000.0 != last_xtrace7_time) begin
            last_xtrace7_time = $realtime/1_000_000_000.0;

            $display("[X7 %0.5f s] | Product8_out1=%b | Product8_out1_1=%b | Product8_out1_2=%b",
                     $realtime/1_000_000_000.0,
                     uut.Product8_out1,
                     uut.Product8_out1_1,
                     uut.Product8_out1_2);
        end
    end
end
real last_xtrace8_time = -1;

always @(posedge clk) begin
    if (uut.Product8_out1_2 === 32'sbx) begin
        if ($realtime/1_000_000_000.0 != last_xtrace8_time) begin
            last_xtrace8_time = $realtime/1_000_000_000.0;

$display("[X8 %0.5f s] counterSig_15=%b Sin_out1_3=%b gz_8=%b Product8_out1_2=%b",
         $realtime/1_000_000_000.0,
         uut.counterSig_15,
         uut.Sin_out1_3,
         uut.gz_8,
         uut.Product8_out1_2);
        end
    end
end
real last_xtrace9_time = -1;

always @(posedge clk) begin
    if (uut.Product8_out1_2 === 32'sbx) begin
        if ($realtime/1_000_000_000.0 != last_xtrace9_time) begin
            last_xtrace9_time = $realtime/1_000_000_000.0;

            $display("[X9 %0.5f s] counter=%b Sum7_out1_1=%b SinMuxSrc=%b",
                     $realtime/1_000_000_000.0,
                     uut.counterSig_15,
                     uut.Sum7_out1_1,
                     uut.Sin_out1_3);
        end
    end
   
end
real last_ctrl_time = -1;

//reg last_prod8_x = 0;
//real last_x10_time = -1;

reg last_prod8_x = 0;
real last_x10_time = -1;

always @(posedge clk) begin
    if ((uut.psi_filt === 16'sbx) && !last_prod8_x) begin
        last_prod8_x = 1;
        last_x10_time = $realtime/1_000_000_000.0;

        $display("theta=%h", uut.Saturation_out1);
        $display("cos(theta)=%h", uut.Cos1_out1);
        $display("reciprocal=%h", uut.Math_Function_out1);
        $display("Product8_out1_2=%h", uut.Product8_out1_2);
        $display("[X10 %0.5f s] psi_filt (Sfix16_En12)=%d",
                 last_x10_time,
//                 uut.counterSig_11,
//                 uut.counterSig_15,
//                 uut.crp_temp_shared_enb_phase_2_2,
//                 uut.Sum8_out1,
                 uut.psi_filt);
    end

    if (uut.Product8_out1_2 !== 32'sbx)
        last_prod8_x = 0;
end

real last_xtrace10_time = -1;
real last_xtrace11_time = -1;

always @(posedge clk) begin
    if (uut.Product8_out1_2 === 32'sbx) begin
        if ($realtime/1_000_000_000.0 != last_xtrace11_time) begin
            last_xtrace11_time = $realtime/1_000_000_000.0;

            $display("[X11 %0.5f s] counterSig_6=%b | az_2=%b W_dot_2=%b | Sum8_sub_cast=%b Sum8_sub_cast_1=%b Sum8_sub_temp=%b | Sum8_out1=%b",
                     $realtime/1_000_000_000.0,
                     uut.counterSig_6,
                     uut.az_2,
                     uut.W_dot_2,
                     uut.Sum8_sub_cast,
                     uut.Sum8_sub_cast_1,
                     uut.Sum8_sub_temp,
                     uut.Sum8_out1);
        end
    end
end


always @(posedge clk) begin
    if (uut.Product8_out1_2 === 32'sbx) begin
        if ($realtime/1_000_000_000.0 != last_xtrace10_time) begin
            last_xtrace10_time = $realtime/1_000_000_000.0;

            $display("[X11 %0.5f s] counterSig_6=%b | az_2=%b W_dot_2=%b | Sum8_sub_cast=%b Sum8_sub_cast_1=%b Sum8_sub_temp=%b | Sum8_out1=%b",
                     $realtime/1_000_000_000.0,
                     uut.counterSig_6,
                     uut.az_2,
                     uut.W_dot_2,
                     uut.Sum8_sub_cast,
                     uut.Sum8_sub_cast_1,
                     uut.Sum8_sub_temp,
                     uut.Sum8_out1);
        end
    end
end
endmodule