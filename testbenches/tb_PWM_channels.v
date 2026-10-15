`timescale 1ns / 1ps

module tb_PWM_channels;

    parameter integer CLK_FREQ_MHZ = 100;
    localparam real    CLK_PERIOD   = 1000.0 / CLK_FREQ_MHZ; // 10.0 ns

    reg         clk;
    reg         rstn;
    reg         clk_en;

    reg  [15:0] duty_int_1;
    reg  [15:0] duty_int_2;
    reg  [15:0] duty_int_3;
    reg  [15:0] duty_int_4;

    wire        pwm_out_1;
    wire        pwm_out_2;
    wire        pwm_out_3;
    wire        pwm_out_4;

    integer error_count = 0;
    integer pass_count  = 0;
    reg     pause_test  = 0;

    PWM_channels #(
        .CLK_FREQ_MHZ(CLK_FREQ_MHZ)
    ) dut (
        .clk        (clk),
        .rstn       (rstn),
        .clk_en     (clk_en),
        .duty_int_1 (duty_int_1),
        .duty_int_2 (duty_int_2),
        .duty_int_3 (duty_int_3),
        .duty_int_4 (duty_int_4),
        .pwm_out_1  (pwm_out_1),
        .pwm_out_2  (pwm_out_2),
        .pwm_out_3  (pwm_out_3),
        .pwm_out_4  (pwm_out_4)
    );

    initial begin
        clk = 0;
        forever #(CLK_PERIOD / 2.0) clk = ~clk;
    end

    task check_pulse_width(
        input integer channel_num,
        input real    measured_ns,
        input [15:0]  expected_us
    );
        real measured_us;
        real diff_us;
        begin
            measured_us = measured_ns / 1000.0;
            diff_us     = measured_us - expected_us;
            if (diff_us < 0) diff_us = -diff_us;

            if (diff_us <= 0.05) begin
                $display("[PASS] Channel %0d: Expected = %0d us | Measured = %0.3f us", 
                         channel_num, expected_us, measured_us);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] Channel %0d: Expected = %0d us | Measured = %0.3f us (Diff: %0.3f us)", 
                         channel_num, expected_us, measured_us, diff_us);
                error_count = error_count + 1;
            end
        end
    endtask

    real t_rise_1, t_high_1;
    real t_rise_2, t_high_2;
    real t_rise_3, t_high_3;
    real t_rise_4, t_high_4;
    real t_frame_start, t_frame_period;

    // Channel 1 Monitor & 50 Hz Frame Period Check
    always @(posedge pwm_out_1) begin
        if (t_frame_start > 0) begin
            t_frame_period = ($realtime - t_frame_start) / 1000.0;
            $display("--------------------------------------------------------------------------------");
            $display("[INFO] Frame Period Measured: %0.2f us (%0.2f Hz)", 
                     t_frame_period, 1000000.0 / t_frame_period);
            if (pause_test) begin
                if (t_frame_period >= 20049.0 && t_frame_period <= 20051.0) begin
                    $display("[PASS] 50 us clk_en Gating Pause Verified! Period = 20050.00 us");
                    pass_count = pass_count + 1;
                end else begin
                    $display("[FAIL] Unexpected Period during pause test: %0.2f us", t_frame_period);
                    error_count = error_count + 1;
                end
            end else begin
                if (t_frame_period >= 19999.0 && t_frame_period <= 20001.0) begin
                    $display("[PASS] 50 Hz Frame Rate Timing Verified!");
                    pass_count = pass_count + 1;
                end else begin
                    $display("[FAIL] Frame Period deviation! Measured = %0.2f us", t_frame_period);
                    error_count = error_count + 1;
                end
            end
            $display("--------------------------------------------------------------------------------");
        end
        t_frame_start = $realtime;
        t_rise_1 = $realtime;
    end
    always @(negedge pwm_out_1) begin
        if (rstn && t_rise_1 > 0) begin
            t_high_1 = $realtime - t_rise_1;
            check_pulse_width(1, t_high_1, dut.pwm_ch1.duty_us_shadow);
        end
    end

    // Channel 2 Monitor
    always @(posedge pwm_out_2) t_rise_2 = $realtime;
    always @(negedge pwm_out_2) begin
        if (rstn && t_rise_2 > 0) begin
            t_high_2 = $realtime - t_rise_2;
            check_pulse_width(2, t_high_2, dut.pwm_ch2.duty_us_shadow);
        end
    end

    // Channel 3 Monitor
    always @(posedge pwm_out_3) t_rise_3 = $realtime;
    always @(negedge pwm_out_3) begin
        if (rstn && t_rise_3 > 0) begin
            t_high_3 = $realtime - t_rise_3;
            check_pulse_width(3, t_high_3, dut.pwm_ch3.duty_us_shadow);
        end
    end

    // Channel 4 Monitor
    always @(posedge pwm_out_4) t_rise_4 = $realtime;
    always @(negedge pwm_out_4) begin
        if (rstn && t_rise_4 > 0) begin
            t_high_4 = $realtime - t_rise_4;
            check_pulse_width(4, t_high_4, dut.pwm_ch4.duty_us_shadow);
        end
    end

    initial begin
        $display("\n================================================================================");
        $display("          STARTING SIMULATION: PWM_channels 50 Hz ESC Pulse Generator            ");
        $display("================================================================================\n");

        rstn          = 0;
        clk_en        = 0;
        duty_int_1    = 16'd1000;
        duty_int_2    = 16'd1500;
        duty_int_3    = 16'd2000;
        duty_int_4    = 16'd1250;
        t_frame_start = 0;
        t_rise_1      = 0;
        t_rise_2      = 0;
        t_rise_3      = 0;
        t_rise_4      = 0;
        pause_test    = 0;

        #500;
        if (pwm_out_1 !== 0 || pwm_out_2 !== 0 || pwm_out_3 !== 0 || pwm_out_4 !== 0) begin
            $display("[FAIL] Outputs not 0 during reset!");
            error_count = error_count + 1;
        end else begin
            $display("[PASS] Reset state verified: All PWM channels held LOW (0V).");
            pass_count = pass_count + 1;
        end

        @(posedge clk);
        rstn   = 1;
        clk_en = 1;
        $display("[INFO] Released reset and enabled clk_en at t = %0t ns", $time);

        $display("\n--- TEST CASE 1: Standard Endpoints & Hover Setpoints ---");
        #21000000;

        $display("\n--- TEST CASE 2: Dynamic Mid-Flight Setpoint Change ---");
        // Sync setpoint change cleanly to frame boundary
        @(negedge pwm_out_3);
        duty_int_1 = 16'd1100;
        duty_int_2 = 16'd1350;
        duty_int_3 = 16'd1650;
        duty_int_4 = 16'd1900;
        #21000000;

        $display("\n--- TEST CASE 3: Clock Enable (clk_en) Gating ---");
        @(negedge pwm_out_4);
        pause_test = 1;
        clk_en = 0;
        $display("[INFO] Disabled clk_en for 50 us at t = %0t ns", $time);
        #50000;
        @(posedge clk);
        clk_en = 1;
        $display("[INFO] Re-enabled clk_en at t = %0t ns", $time);
        #21000000;
        pause_test = 0;

        $display("\n================================================================================");
        $display("                             TESTBENCH COMPLETE                                ");
        $display("================================================================================");
        $display("  Total Checks Passed : %0d", pass_count);
        $display("  Total Checks Failed : %0d", error_count);
        if (error_count == 0)
            $display("  >>> ALL CHECKS PASSED: 100%% VERIFIED! <<<\n");
        else
            $display("  >>> TESTBENCH FAILED WITH %0d ERRORS! <<<\n", error_count);
        $display("================================================================================\n");

        $finish;
    end

endmodule