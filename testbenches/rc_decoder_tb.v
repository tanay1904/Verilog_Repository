`timescale 1ns/1ps
`default_nettype none

module rc_decoder_tb;

    // -------------------------------------------------
    // Clock / Reset
    // -------------------------------------------------
    reg clk = 0;
    always #5 clk = ~clk;   // 100 MHz

    reg rst = 1;

    // -------------------------------------------------
    // PWM input (throttle only for clarity)
    // -------------------------------------------------
    reg pwm_throttle = 0;

    // -------------------------------------------------
    // DUT outputs
    // -------------------------------------------------
    wire signed [11:0] throttle;
    wire failsafe;

    // -------------------------------------------------
    // DUT
    // -------------------------------------------------
    rc_input_decoder DUT (
        .clk(clk),
        .rst(rst),
        .pwm_roll(1'b0),
        .pwm_pitch(1'b0),
        .pwm_throttle(pwm_throttle),
        .pwm_yaw(1'b0),
        .roll(),
        .pitch(),
        .throttle(throttle),
        .yaw(),
        .failsafe(failsafe)
    );

    // -------------------------------------------------
    // Testbench state tracking
    // -------------------------------------------------
    reg permission_to_update;
    reg update_seen_for_pulse;
    reg signed [11:0] last_throttle;

    // -------------------------------------------------
    // Detect PWM edges
    // -------------------------------------------------
    always @(posedge pwm_throttle) begin
        // Rising edge ? measurement starts
        permission_to_update   <= 1'b0;
        update_seen_for_pulse  <= 1'b0;
    end

    always @(negedge pwm_throttle) begin
        // Falling edge ? measurement complete
        permission_to_update  <= 1'b1;
    end

    // -------------------------------------------------
    // Output change monitor (THE CORE CHECK)
    // -------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            last_throttle       <= 0;
            permission_to_update<= 0;
            update_seen_for_pulse <= 0;
        end else begin
            if (throttle !== last_throttle) begin
                // Output changed - check legality

                if (!permission_to_update) begin
                    $display("FAIL: throttle changed BEFORE falling edge @ %0t", $time);
                    $fatal;
                end

                if (update_seen_for_pulse) begin
                    $display("FAIL: throttle changed MORE THAN ONCE per pulse @ %0t", $time);
                    $fatal;
                end

                // First legal update
                update_seen_for_pulse <= 1'b1;
                permission_to_update  <= 1'b0;
            end

            last_throttle <= throttle;
        end
    end

    // -------------------------------------------------
    // Task: send a PWM pulse
    // -------------------------------------------------
    task send_pulse(input integer high_ns, input integer period_ns);
        begin
            pwm_throttle = 1;
            #(high_ns);
            pwm_throttle = 0;
            #(period_ns - high_ns);
        end
    endtask

    // -------------------------------------------------
    // Test sequence
    // -------------------------------------------------
    initial begin
        // Reset
        rst = 1;
        #200;
        rst = 0;

        // Give DUT time
        #1000;

        // --- Valid pulses ---
        send_pulse(1_500_000, 20_000_000);
        send_pulse(1_250_000, 20_000_000);
        send_pulse(1_750_000, 20_000_000);

        // --- Back-to-back pulses ---
        send_pulse(1_500_000, 20_000_000);
        send_pulse(1_500_000, 20_000_000);

        // --- Long idle ? failsafe ---
        #50_000_000;
        if (!failsafe) begin
            $display("FAIL: failsafe did not assert");
            $fatal;
        end

        // --- Recovery pulse ---
        send_pulse(1_500_000, 20_000_000);
        #1_000_000;
        if (failsafe) begin
            $display("FAIL: failsafe did not clear after valid pulse");
            $fatal;
        end

        $display("PASS: all timing and safety rules satisfied");
        $stop;
    end

endmodule

`default_nettype wire
