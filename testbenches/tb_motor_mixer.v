`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 20.09.2026 17:45:58
// Design Name: 
// Module Name: tb_motor_mixer
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

module tb_motor_mixer_direct;

    // Clock and Reset
    reg clk;
    reg rstn;

    // Inputs to DUT
    reg signed [11:0] roll;
    reg signed [11:0] pitch;
    reg signed [11:0] throttle;
    reg signed [11:0] yaw;
    reg               failsafe;

    // Outputs from DUT
    wire [15:0] duty_int_1;
    wire [15:0] duty_int_2;
    wire [15:0] duty_int_3;
    wire [15:0] duty_int_4;

    // Error counter
    integer err_count = 0;
    integer test_num  = 0;

    // -------------------------------------------------------------
    // Instantiate Device Under Test (DUT)
    // -------------------------------------------------------------
    motor_mixer #(
        .DUTY_MIN        (3333),
        .DUTY_MAX        (6667),
        .ROLL_SCALE      (400),
        .PITCH_SCALE     (400),
        .YAW_SCALE       (300),
        .THROTTLE_CUTOFF (-900),
        .DEADBAND        (30)
    ) dut (
        .clk        (clk),
        .rstn       (rstn),
        .roll       (roll),
        .pitch      (pitch),
        .throttle   (throttle),
        .yaw        (yaw),
        .failsafe   (failsafe),
        .duty_int_1 (duty_int_1),
        .duty_int_2 (duty_int_2),
        .duty_int_3 (duty_int_3),
        .duty_int_4 (duty_int_4)
    );

    // -------------------------------------------------------------
    // Clock Generator (100 MHz -> 10 ns period)
    // -------------------------------------------------------------
    always #5 clk = ~clk;

    // -------------------------------------------------------------
    // Verification Helper Task
    // -------------------------------------------------------------
    task check_outputs;
        input [15:0] exp_m1;
        input [15:0] exp_m2;
        input [15:0] exp_m3;
        input [15:0] exp_m4;
        input [255:0] test_name;
    begin
        test_num = test_num + 1;
        @(posedge clk); // Wait 1 cycle for registers to update
        #1;             // Small delta delay to sample after clock edge

        if (duty_int_1 === exp_m1 && 
            duty_int_2 === exp_m2 && 
            duty_int_3 === exp_m3 && 
            duty_int_4 === exp_m4) begin
            $display("[PASS] Test %2d: %0s", test_num, test_name);
            $display("         Outputs: M1=%d, M2=%d, M3=%d, M4=%d\n", 
                     duty_int_1, duty_int_2, duty_int_3, duty_int_4);
        end else begin
            $display("[FAIL] Test %2d: %0s", test_num, test_name);
            $display("         Expected: M1=%d, M2=%d, M3=%d, M4=%d", 
                     exp_m1, exp_m2, exp_m3, exp_m4);
            $display("         Actual:   M1=%d, M2=%d, M3=%d, M4=%d\n", 
                     duty_int_1, duty_int_2, duty_int_3, duty_int_4);
            err_count = err_count + 1;
        end
    end
    endtask

    // -------------------------------------------------------------
    // Test Sequence
    // -------------------------------------------------------------
    initial begin
        // Initialize inputs
        clk      = 0;
        rstn     = 0;
        roll     = 0;
        pitch    = 0;
        throttle = -1000;
        yaw      = 0;
        failsafe = 0;

        $display("======================================================");
        $display("   STARTING DIRECTED TESTBENCH: motor_mixer_direct    ");
        $display("======================================================\n");

        // ---------------------------------------------------------
        // Test 1: Reset State
        // ---------------------------------------------------------
        #20;
        check_outputs(3333, 3333, 3333, 3333, "Reset active (rstn=0)");

        // Release reset
        @(negedge clk);
        rstn = 1;
        #10;

        // ---------------------------------------------------------
        // Test 2: Throttle at Minimum (Idle on ground)
        // ---------------------------------------------------------
        throttle = -1000; roll = 0; pitch = 0; yaw = 0; failsafe = 0;
        check_outputs(3333, 3333, 3333, 3333, "Min Throttle (-1000) / Sticks Centered");

        // ---------------------------------------------------------
        // Test 3: Mid-Throttle Hover (0 on -1000..+1000 scale)
        // Expected base_duty: 3333 + (1000*3334)/2000 = 5000
        // ---------------------------------------------------------
        throttle = 0; roll = 0; pitch = 0; yaw = 0; failsafe = 0;
        check_outputs(5000, 5000, 5000, 5000, "Mid-Throttle (0) / Neutral Hover");

        // ---------------------------------------------------------
        // Test 4: Full Throttle (+1000)
        // Expected base_duty: 3333 + (2000*3334)/2000 = 6667
        // ---------------------------------------------------------
        throttle = 1000; roll = 0; pitch = 0; yaw = 0; failsafe = 0;
        check_outputs(6667, 6667, 6667, 6667, "Full Throttle (+1000) / Max Climb");

        // ---------------------------------------------------------
        // Test 5: Pure Pitch Forward at Mid-Throttle
        // pitch=+1000 -> p_effort=+400
        // M1(RR)=5000+400=5400, M2(FR)=5000-400=4600
        // M3(RL)=5000+400=5400, M4(FL)=5000-400=4600
        // ---------------------------------------------------------
        throttle = 0; roll = 0; pitch = 1000; yaw = 0; failsafe = 0;
        check_outputs(5400, 4600, 5400, 4600, "Pitch Full Forward (+1000)");

        // ---------------------------------------------------------
        // Test 6: Pure Roll Right at Mid-Throttle
        // roll=+1000 -> r_effort=+400
        // M1(RR)=5000-400=4600, M2(FR)=5000+400=5400
        // M3(RL)=5000+400=5400, M4(FL)=5000-400=4600
        // ---------------------------------------------------------
        throttle = 0; roll = 1000; pitch = 0; yaw = 0; failsafe = 0;
        check_outputs(4600, 5400, 5400, 4600, "Roll Full Right (+1000)");

        // ---------------------------------------------------------
        // Test 7: Pure Yaw Clockwise (CW) at Mid-Throttle
        // yaw=+1000 -> y_effort=+300
        // M1=5000+300=5300, M2=5000+300=5300
        // M3=5000-300=4700, M4=5000-300=4700
        // ---------------------------------------------------------
        throttle = 0; roll = 0; pitch = 0; yaw = 1000; failsafe = 0;
        check_outputs(5300, 5300, 4700, 4700, "Yaw Full CW (+1000)");

        // =========================================================
        // EDGE CASES
        // =========================================================

        // ---------------------------------------------------------
        // Test 8: EDGE CASE - Failsafe Active at Full Throttle
        // Failsafe MUST override stick input and force all motors to idle
        // ---------------------------------------------------------
        throttle = 1000; roll = 500; pitch = -300; yaw = 200; failsafe = 1;
        check_outputs(3333, 3333, 3333, 3333, "EDGE: Failsafe Active during flight");

        // ---------------------------------------------------------
        // Test 9: EDGE CASE - Throttle Below Cutoff with Sticks Deflected
        // Throttle=-950 (< -900). Pilot wiggling sticks on ground.
        // Motors MUST stay idle (no accidental spin up on ground).
        // ---------------------------------------------------------
        throttle = -950; roll = 800; pitch = 800; yaw = 800; failsafe = 0;
        check_outputs(3333, 3333, 3333, 3333, "EDGE: Sticks Deflected below Throttle Cutoff");

        // ---------------------------------------------------------
        // Test 10: EDGE CASE - Deadband Filtering around Center
        // Roll=25, Pitch=-20, Yaw=15 (all within DEADBAND=30)
        // Outputs should ignore noise and stay exactly at 5000
        // ---------------------------------------------------------
        throttle = 0; roll = 25; pitch = -20; yaw = 15; failsafe = 0;
        check_outputs(5000, 5000, 5000, 5000, "EDGE: Stick Jitter within Deadband (+/-30)");

        // ---------------------------------------------------------
        // Test 11: EDGE CASE - Upper Saturation Clamping
        // Throttle=+1000 (6667) + Pitch=+1000 (+400) + Yaw=+1000 (+300)
        // Math = 7367. Must saturate cleanly at DUTY_MAX (6667)
        // ---------------------------------------------------------
        throttle = 1000; roll = 0; pitch = 1000; yaw = 1000; failsafe = 0;
        check_outputs(6667, 6567, 6667, 5967, "EDGE: Upper Saturation Clamp (Max 6667)");

        // ---------------------------------------------------------
        // Test 12: EDGE CASE - Lower Saturation Clamping
        // Low throttle (-850) + Full reverse pitch (-1000).
        // Math pushes motors below 3333. Must saturate at DUTY_MIN (3333).
        // ---------------------------------------------------------
        throttle = -850; roll = 0; pitch = -1000; yaw = 0; failsafe = 0;
        // base = 3333 + (150*3334)/2000 = 3583
        // M1(RR) = 3583 - 400 = 3183 -> Clamped to 3333
        // M2(FR) = 3583 + 400 = 3983
        // M3(RL) = 3583 - 400 = 3183 -> Clamped to 3333
        // M4(FL) = 3583 + 400 = 3983
        check_outputs(3333, 3983, 3333, 3983, "EDGE: Lower Saturation Clamp (Min 3333)");

        // ---------------------------------------------------------
        // Summary
        // ---------------------------------------------------------
        #50;
        $display("======================================================");
        if (err_count == 0)
            $display("   TESTBENCH SUCCESSFUL: All %0d tests PASSED!        ", test_num);
        else
            $display("   TESTBENCH FAILED: %0d error(s) detected.          ", err_count);
        $display("======================================================\n");
        $finish;
    end

endmodule