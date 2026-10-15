`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// Tier 3 RTL Testbench: Integrator + Position Filter Coupling
// Targets: Integrator.v and Position_Filter2.v
//////////////////////////////////////////////////////////////////////////////////

module tb_tier3_coupling;

    reg clk;
    reg reset;
    wire resetn = ~reset;

    reg [11:0] dt;
    reg signed [15:0] ax_global;
    reg signed [15:0] ay_global;

    wire signed [15:0] Xraw, Yraw;
    wire filter_ce_out;
    wire signed [15:0] Uest, Vest, Xest, Yest;

    // Clock (10 MHz)
    initial clk = 0;
    always #50 clk = ~clk;

    // Timing generator for 4ms (40,000 cycles)
    reg [15:0] tick_counter = 0;
    reg ce_pulse = 0;

    always @(posedge clk) begin
        if (reset) begin
            tick_counter <= 0;
            ce_pulse <= 0;
        end else begin
            if (tick_counter == 39999) begin
                tick_counter <= 0;
                ce_pulse <= 1;
            end else begin
                tick_counter <= tick_counter + 1;
                ce_pulse <= 0;
            end
        end
    end

    // Double Integrators
    double_Integrator #(.WIDTH(16), .DT_WIDTH(12), .FRAC_DT(12)) u_int_x (
        .clk(clk),
        .reset(reset),
        .clk_enable(ce_pulse),
        .dt(dt),
        .u(ax_global),
        .y(Xraw)
    );

    double_Integrator #(.WIDTH(16), .DT_WIDTH(12), .FRAC_DT(12)) u_int_y (
        .clk(clk),
        .reset(reset),
        .clk_enable(ce_pulse),
        .dt(dt),
        .u(ay_global),
        .y(Yraw)
    );

    // Position Filter
    Position_Filter2 u_pos_filter (
        .clk(clk),
        .resetn(resetn),
        .resetn_Position_Filter2_tc(resetn),
        .clk_enable(1'b1),
        .X_raw(Xraw),
        .Y_raw(Yraw),
        .ax_global(ax_global),
        .ay_global(ay_global),
        .ce_out(filter_ce_out),
        .Uest(Uest),
        .Vest(Vest),
        .Yest(Yest),
        .Xest(Xest)
    );

    integer pass_count = 0;
    integer fail_count = 0;
    integer pulse_idx;

    initial begin
        $display("=================================================");
        $display("Starting Tier 3 RTL Test: Integrator + Filter Coupling");
        $display("=================================================");

        reset = 1;
        dt = 12'd16; // 0.00390625s
        ax_global = 0;
        ay_global = 0;
        #2000;
        @(posedge clk);
        reset = 0;
        #1000;

        // Apply acceleration pulse ay_global = 1024 (1.0 m/s^2) for 20 sample pulses
        ay_global = 16'sd1024;
        for (pulse_idx = 0; pulse_idx < 20; pulse_idx = pulse_idx + 1) begin
            @(posedge ce_pulse);
        end

        // Wait 5 more sample periods with zero accel
        ay_global = 16'sd0;
        for (pulse_idx = 0; pulse_idx < 5; pulse_idx = pulse_idx + 1) begin
            @(posedge ce_pulse);
        end

        $display("After pulse: Yraw = %d, Yest = %d, Vest = %d", Yraw, Yest, Vest);

        // Verification: Both raw and estimated position must be positive and non-zero
        if (Yraw > 0 && Yest > 0) begin
            $display("[PASS] T3.1-RTL: Integrator and Position Filter coupling active and positive");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] T3.1-RTL: Coupling failed (Yraw=%d, Yest=%d)", Yraw, Yest);
            fail_count = fail_count + 1;
        end

        if (Vest > 0) begin
            $display("[PASS] T3.2-RTL: Velocity state correctly estimated (Vest = %d)", Vest);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] T3.2-RTL: Expected positive Vest, got %d", Vest);
            fail_count = fail_count + 1;
        end

        $display("=================================================");
        $display("Tier 3 RTL Test Complete: %0d PASSED, %0d FAILED", pass_count, fail_count);
        $display("=================================================");
        $finish;
    end

endmodule
