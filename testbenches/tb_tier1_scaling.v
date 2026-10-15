`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// Tier 1 RTL Testbench: Bit-Shift Scaling & Q-Format Converters
// Target: sfix_converter.v and Top-level Shift
//////////////////////////////////////////////////////////////////////////////////

module tb_tier1_scaling;

    // Converter signals
    reg signed [15:0] in_en9;
    wire signed [15:0] out_en10;

    reg signed [15:0] in_en12;
    wire signed [15:0] out_en14;

    sfix_converter #(.EN_IN(9), .EN_OUT(10)) conv_9_10 (
        .din(in_en9),
        .dout(out_en10)
    );

    sfix_converter #(.EN_IN(12), .EN_OUT(14)) conv_12_14 (
        .din(in_en12),
        .dout(out_en14)
    );

    // Signed shift check
    reg signed [31:0] phi_cmd_signed;
    wire signed [15:0] phi_raw_req_arithmetic;
    wire [15:0] phi_raw_req_logical;

    assign phi_raw_req_arithmetic = $signed(phi_cmd_signed) >>> 15;
    assign phi_raw_req_logical    = phi_cmd_signed >> 15;

    integer pass_count = 0;
    integer fail_count = 0;

    initial begin
        $display("=================================================");
        $display("Starting Tier 1 RTL Test: Scaling & Converters");
        $display("=================================================");

        #10;
        in_en9 = 16'sd100;
        in_en12 = 16'sd400;
        phi_cmd_signed = -32'sd107374182; // -0.2 rad in sfix32_En29

        #10;
        // Test 1: En9 -> En10 (shift left 1)
        if (out_en10 === 16'sd200) begin
            $display("[PASS] T1.3-RTL: sfix_converter En9->En10 correct (100 -> %d)", out_en10);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] T1.3-RTL: sfix_converter En9->En10 expected 200, got %d", out_en10);
            fail_count = fail_count + 1;
        end

        // Test 2: En12 -> En14 (shift left 2)
        if (out_en14 === 16'sd1600) begin
            $display("[PASS] T1.4-RTL: sfix_converter En12->En14 correct (400 -> %d)", out_en14);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] T1.4-RTL: sfix_converter En12->En14 expected 1600, got %d", out_en14);
            fail_count = fail_count + 1;
        end

        // Test 3: Arithmetic right shift preservation of negative values
        if (phi_raw_req_arithmetic < 0 && phi_raw_req_arithmetic === -16'sd3277) begin
            $display("[PASS] T1.5-RTL: $signed(phi_cmd) >>> 15 preserves negative sign (val = %d)", phi_raw_req_arithmetic);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] T1.5-RTL: Signed shift failed, got %d", phi_raw_req_arithmetic);
            fail_count = fail_count + 1;
        end

        // Test 4: Verify logical shift fails (produces positive number)
        if (phi_raw_req_logical !== phi_raw_req_arithmetic) begin
            $display("[PASS] T1.6-RTL: Confirmed logical shift produces invalid unsigned value (%h vs %h)", phi_raw_req_logical, phi_raw_req_arithmetic);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] T1.6-RTL: Expected mismatch between logical and arithmetic shifts");
            fail_count = fail_count + 1;
        end

        $display("=================================================");
        $display("Tier 1 Scaling Test Complete: %0d PASSED, %0d FAILED", pass_count, fail_count);
        $display("=================================================");
        $finish;
    end

endmodule
