`timescale 1ns / 1ps

module thrust_to_pwm_channel (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               valid_in,
    input  wire signed [29:0] thrust_in,   // sfix30En16 format
    output reg                valid_out,
    output reg         [15:0] pwm_out      // Integer microseconds (1000 - 2000)
);

    // =========================================================================
    // Stage 1: Segment Classification & Coefficient Lookup
    // =========================================================================
    reg signed [29:0] thrust_r1;
    reg        [25:0] t_base_r1;
    reg        [31:0] pwm_base_r1;
    reg        [17:0] slope_r1;
    reg               underflow_r1;
    reg               overflow_r1;
    reg               vld_r1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            thrust_r1    <= 30'sd0;
            t_base_r1    <= 26'd0;
            pwm_base_r1  <= 32'd0;
            slope_r1     <= 18'd0;
            underflow_r1 <= 1'b0;
            overflow_r1  <= 1'b0;
            vld_r1       <= 1'b0;
        end else begin
            vld_r1    <= valid_in;
            thrust_r1 <= thrust_in;

            if (thrust_in <= 30'sd0) begin
                underflow_r1 <= 1'b1;
                overflow_r1  <= 1'b0;
                t_base_r1    <= 26'd0;
                pwm_base_r1  <= 32'd65536000;  // 1000 * 65536
                slope_r1     <= 18'd0;
            end else if (thrust_in >= 30'sd55050240) begin // 840 N
                underflow_r1 <= 1'b0;
                overflow_r1  <= 1'b1;
                t_base_r1    <= 26'd55050240;
                pwm_base_r1  <= 32'd131072000; // 2000 * 65536
                slope_r1     <= 18'd0;
            end else begin
                underflow_r1 <= 1'b0;
                overflow_r1  <= 1'b0;

                if (thrust_in >= 30'sd46530560) begin      // [710, 840)
                    t_base_r1   <= 26'd46530560;
                    pwm_base_r1 <= 32'd121709714;
                    slope_r1    <= 18'd72018;
                end else if (thrust_in >= 30'sd44564480) begin // [680, 710)
                    t_base_r1   <= 26'd44564480;
                    pwm_base_r1 <= 32'd114688000;
                    slope_r1    <= 18'd234057;
                end else if (thrust_in >= 30'sd43909120) begin // [670, 680)
                    t_base_r1   <= 26'd43909120;
                    pwm_base_r1 <= 32'd112347429;
                    slope_r1    <= 18'd234057;
                end else if (thrust_in >= 30'sd39321600) begin // [600, 670)
                    t_base_r1   <= 26'd39321600;
                    pwm_base_r1 <= 32'd110006857;
                    slope_r1    <= 18'd33437;
                end else if (thrust_in >= 30'sd35192832) begin // [537, 600)
                    t_base_r1   <= 26'd35192832;
                    pwm_base_r1 <= 32'd107666286;
                    slope_r1    <= 18'd37152;
                end else if (thrust_in >= 30'sd30605312) begin // [467, 537)
                    t_base_r1   <= 26'd30605312;
                    pwm_base_r1 <= 32'd105325714;
                    slope_r1    <= 18'd33437;
                end else if (thrust_in >= 30'sd26542080) begin // [405, 467)
                    t_base_r1   <= 26'd26542080;
                    pwm_base_r1 <= 32'd102985143;
                    slope_r1    <= 18'd37751;
                end else if (thrust_in >= 30'sd22806528) begin // [348, 405)
                    t_base_r1   <= 26'd22806528;
                    pwm_base_r1 <= 32'd100644571;
                    slope_r1    <= 18'd41063;
                end else if (thrust_in >= 30'sd19005440) begin // [290, 348)
                    t_base_r1   <= 26'd19005440;
                    pwm_base_r1 <= 32'd98304000;
                    slope_r1    <= 18'd40355;
                end else if (thrust_in >= 30'sd15597568) begin // [238, 290)
                    t_base_r1   <= 26'd15597568;
                    pwm_base_r1 <= 32'd95963429;
                    slope_r1    <= 18'd45011;
                end else if (thrust_in >= 30'sd12320768) begin // [188, 238)
                    t_base_r1   <= 26'd12320768;
                    pwm_base_r1 <= 32'd93622857;
                    slope_r1    <= 18'd46811;
                end else if (thrust_in >= 30'sd9502720) begin  // [145, 188)
                    t_base_r1   <= 26'd9502720;
                    pwm_base_r1 <= 32'd91282286;
                    slope_r1    <= 18'd54432;
                end else if (thrust_in >= 30'sd7208960) begin  // [110, 145)
                    t_base_r1   <= 26'd7208960;
                    pwm_base_r1 <= 32'd88941714;
                    slope_r1    <= 18'd66873;
                end else if (thrust_in >= 30'sd4456448) begin  // [68, 110)
                    t_base_r1   <= 26'd4456448;
                    pwm_base_r1 <= 32'd86601143;
                    slope_r1    <= 18'd55728;
                end else if (thrust_in >= 30'sd2818048) begin  // [43, 68)
                    t_base_r1   <= 26'd2818048;
                    pwm_base_r1 <= 32'd84260571;
                    slope_r1    <= 18'd93623;
                end else if (thrust_in >= 30'sd1507328) begin  // [23, 43)
                    t_base_r1   <= 26'd1507328;
                    pwm_base_r1 <= 32'd81920000;
                    slope_r1    <= 18'd117029;
                end else if (thrust_in >= 30'sd622592) begin   // [9.5, 23)
                    t_base_r1   <= 26'd622592;
                    pwm_base_r1 <= 32'd79579429;
                    slope_r1    <= 18'd173376;
                end else begin                                 // (0, 9.5)
                    t_base_r1   <= 26'd0;
                    pwm_base_r1 <= 32'd77238857;
                    slope_r1    <= 18'd246376;
                end
            end
        end
    end

    // =========================================================================
    // Stage 2: Difference Calculation and Fixed-Point Multiplication
    // =========================================================================
    reg [41:0] prod_r2;
    reg [31:0] pwm_base_r2;
    reg        underflow_r2;
    reg        overflow_r2;
    reg        vld_r2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prod_r2      <= 42'd0;
            pwm_base_r2  <= 32'd0;
            underflow_r2 <= 1'b0;
            overflow_r2  <= 1'b0;
            vld_r2       <= 1'b0;
        end else begin
            vld_r2       <= vld_r1;
            underflow_r2 <= underflow_r1;
            overflow_r2  <= overflow_r1;
            pwm_base_r2  <= pwm_base_r1;
            // Delta thrust * slope (24-bit unsigned x 18-bit unsigned)
            prod_r2      <= (thrust_r1[25:0] - t_base_r1) * slope_r1;
        end
    end

    // =========================================================================
    // Stage 3: Accumulate Base, Convergent Rounding & Clamping
    // =========================================================================
    wire [31:0] pwm_fixed   = pwm_base_r2 + prod_r2[41:16];
    // Adding 0.5 (32768 in Q16) prior to right shift by 16 implements nearest-integer rounding
    wire [31:0] pwm_rounded = pwm_fixed + 32'd32768;
    wire [15:0] pwm_int     = pwm_rounded[31:16];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_out   <= 16'd1000;
            valid_out <= 1'b0;
        end else begin
            valid_out <= vld_r2;
            if (underflow_r2 || (pwm_int < 16'd1000)) begin
                pwm_out <= 16'd1000;
            end else if (overflow_r2 || (pwm_int > 16'd2000)) begin
                pwm_out <= 16'd2000;
            end else begin
                pwm_out <= pwm_int;
            end
        end
    end

endmodule