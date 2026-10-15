`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// Tier 2 RTL Testbench: Actuator Saturation & Dynamic Range
// Target: Attitude_Roll_Controller_2.v
//////////////////////////////////////////////////////////////////////////////////

module tb_tier2_boundaries;

    reg clk;
    reg resetn;
    reg clk_enable;

    reg signed [15:0] phi_raw_req;
    reg signed [15:0] phi_meas;
    reg signed [15:0] p_meas;

    wire ce_out;
    wire signed [15:0] U2;

    // Clock (10 MHz)
    initial clk = 0;
    always #50 clk = ~clk;

    // Instantiate fixed Roll Controller
    Attitude_Roll_Controller_2 dut_roll_ctrl (
        .clk(clk),
        .resetn(resetn),
        .resetn_Attitude_Roll_Controller_2_tc(resetn),
        .clk_enable(clk_enable),
        .phi_raw_req(phi_raw_req),
        .phi_meas(phi_meas),
        .p_meas(p_meas),
        .ce_out(ce_out),
        .U2(U2)
    );

    integer pass_count = 0;
    integer fail_count = 0;
    integer tick;

    initial begin
        $display("=================================================");
        $display("Starting Tier 2 RTL Test: Saturation & Boundaries");
        $display("=================================================");

        resetn = 0;
        clk_enable = 1;
        phi_raw_req = 0;
        phi_meas = 0;
        p_meas = 0;
        #1000;
        @(posedge clk);
        resetn = 1;
        #1000;

        // Test 1: Zero error condition
        // Wait for 1 sample period (40,000 cycles = 4ms)
        @(posedge ce_out);
        #100;
        if (U2 === 16'sd0) begin
            $display("[PASS] T2.1-RTL: Zero demand produces zero torque U2 = %d", U2);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] T2.1-RTL: Expected zero U2, got %d", U2);
            fail_count = fail_count + 1;
        end

        // Test 2: Huge positive demand (+45 deg = 12868 in sfix16_En14)
        phi_raw_req = 16'sd12868;
        @(posedge ce_out);
        #100;
        $display("After huge positive demand, U2 = %d (sfix16_En10)", U2);
        // Expect clamp at +1024 (1.0 N*m)
        if (U2 <= 16'sd1024 && U2 > 16'sd500) begin
            $display("[PASS] T2.2-RTL: Positive demand saturation bounded (U2 = %d <= 1024)", U2);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] T2.2-RTL: Positive saturation violation (U2 = %d)", U2);
            fail_count = fail_count + 1;
        end

        // Test 3: Huge negative demand (-45 deg = -12868 in sfix16_En14)
        phi_raw_req = -16'sd12868;
        @(posedge ce_out);
        #100;
        $display("After huge negative demand, U2 = %d (sfix16_En10)", U2);
        // Expect clamp at -1024 (-1.0 N*m)
        if (U2 >= -16'sd1024 && U2 < -16'sd500) begin
            $display("[PASS] T2.3-RTL: Negative demand saturation bounded (U2 = %d >= -1024)", U2);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] T2.3-RTL: Negative saturation violation (U2 = %d)", U2);
            fail_count = fail_count + 1;
        end

        $display("=================================================");
        $display("Tier 2 RTL Test Complete: %0d PASSED, %0d FAILED", pass_count, fail_count);
        $display("=================================================");
        $finish;
    end

endmodule
