`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// Tier 1 RTL Testbench: Standalone double_Integrator Unit Test
// Target: Integrator.v
//////////////////////////////////////////////////////////////////////////////////

module tb_tier1_integrator;

    reg clk;
    reg reset;
    reg clk_enable;
    reg [11:0] dt;
    reg signed [15:0] u;
    wire signed [15:0] y;

    // Clock generation (10 MHz)
    initial clk = 0;
    always #50 clk = ~clk;

    // Instantiate DUT
    double_Integrator #(
        .WIDTH(16),
        .DT_WIDTH(12),
        .FRAC_DT(12)
    ) dut (
        .clk(clk),
        .reset(reset),
        .clk_enable(clk_enable),
        .dt(dt),
        .u(u),
        .y(y)
    );

    integer pass_count = 0;
    integer fail_count = 0;
    integer i;

    initial begin
        $display("=================================================");
        $display("Starting Tier 1 RTL Test: double_Integrator");
        $display("=================================================");

        // 1. Reset DUT
        reset = 1;
        clk_enable = 0;
        dt = 12'd16; // 16/4096 = 0.00390625s
        u = 0;
        #200;
        reset = 0;
        #100;

        // Test 1: Zero acceleration check
        @(posedge clk);
        clk_enable = 1;
        u = 16'sd0;
        @(posedge clk);
        clk_enable = 0;
        #100;

        if (y === 16'sd0) begin
            $display("[PASS] T1.1-RTL: Zero input produces zero position (y = %d)", y);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] T1.1-RTL: Expected y=0, got %d", y);
            fail_count = fail_count + 1;
        end

        // Test 2: Step acceleration of 1.0 m/s^2 (1024 in sfix16_En10) for 100 sample pulses
        u = 16'sd1024;
        for (i = 0; i < 100; i = i + 1) begin
            @(posedge clk);
            clk_enable = 1;
            @(posedge clk);
            clk_enable = 0;
            #1000;
        end

        $display("After 100 ticks with u=1024, integrated position y = %d (sfix16_En10)", y);
        // Expected position is around 78 counts (0.076m * 1024)
        if (y > 0) begin
            $display("[PASS] T1.2-RTL: Positive acceleration integrated positively (y = %d)", y);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] T1.2-RTL: Expected positive position, got %d", y);
            fail_count = fail_count + 1;
        end

        $display("=================================================");
        $display("Tier 1 RTL Test Complete: %0d PASSED, %0d FAILED", pass_count, fail_count);
        $display("=================================================");
        $finish;
    end

endmodule
