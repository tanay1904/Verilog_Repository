`timescale 1ns / 1ps
`default_nettype none

module throttle_rc_frontend (
    input wire clk,
    input wire reset,

    input wire pwm_throttle_in,

    output wire signed [11:0] throttle_out,

    output wire failsafe_out,
    
    output wire led
);

    throttle_pwm_decoder lead_decoder (

        .clk(clk),
        .rst(reset),

        .pwm_throttle(pwm_throttle_in),

        .throttle(throttle_out),

        .failsafe(failsafe_out),
        
        .led(led)
    );

endmodule

`default_nettype wire