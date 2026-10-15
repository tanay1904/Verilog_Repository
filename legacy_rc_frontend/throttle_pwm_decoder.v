`timescale 1ns/1ps
`default_nettype none

module throttle_pwm_decoder(

    input wire clk,
    input wire rst,
    input wire pwm_throttle,

    output reg signed [11:0] throttle,

    output reg failsafe,
    
    output reg led
);

    parameter integer FREQ = 10_000_000;

    parameter integer PWM_MIN     = 1000;
    parameter integer PWM_MAX     = 2000;
    parameter integer PWM_NEUTRAL = 1500;

    parameter integer CLOCKS_PER_US = FREQ / 1_000_000;

    localparam integer PWM_MIN_CYCLES = (PWM_MIN - 20)*CLOCKS_PER_US;
    localparam integer PWM_MAX_CYCLES = (PWM_MAX + 20)*CLOCKS_PER_US;

    reg throttle_sync1, throttle_sync2;

    always @(posedge clk) begin
        throttle_sync1 <= pwm_throttle;
        throttle_sync2 <= throttle_sync1;
    end

    reg throttle_prev;

    reg [31:0] count_throttle;

    function signed [11:0] normalize_pwm;
        input [31:0] cycles;
        integer pulse_us;
        integer temp;
    begin

        pulse_us = (cycles + (CLOCKS_PER_US/2))
                 / CLOCKS_PER_US;

        temp = ((pulse_us - PWM_NEUTRAL) * 2000)
             / (PWM_MAX - PWM_MIN);

        // Saturate output to ±1000
        if(temp > 1000)
            temp = 1000;

        if(temp < -1000)
            temp = -1000;

        // Deadband ±50
        if(temp > -50 && temp < 50)
            normalize_pwm = 0;
        else
            normalize_pwm = temp;

    end
    endfunction

    always @(posedge clk) begin

        if(rst) begin

            throttle <= -1000;

            failsafe <= 1;

            count_throttle <= 0;

            throttle_prev <= 0;

        end
        else begin

            // Count pulse width
            if(throttle_sync2 && !throttle_prev)
                count_throttle <= 0;
            else if(throttle_sync2)
                count_throttle <= count_throttle + 1;

            // On falling edge, decode pulse
            if(!throttle_sync2 && throttle_prev) begin

                if((count_throttle >= PWM_MIN_CYCLES) &&
                   (count_throttle <= PWM_MAX_CYCLES))

                    throttle <= normalize_pwm(count_throttle);

            end

            failsafe <= 0;

            throttle_prev <= throttle_sync2;

        end
    end
    always @(posedge clk) begin
    if (throttle_sync2 && !throttle_prev)
        led <= ~led;
    end
endmodule

`default_nettype wire


// code for 50mhz clock

//`timescale 1ns/1ps
//`default_nettype none

//module throttle_pwm_decoder(

//    input wire clk,
//    input wire rst,

//    input wire pwm_throttle,

//    output reg signed [11:0] throttle,

//    output reg failsafe
//);

//    parameter integer FREQ = 10_000_000;

//    parameter integer PWM_MIN     = 1000;
//    parameter integer PWM_MAX     = 2000;
//    parameter integer PWM_NEUTRAL = 1500;

//    parameter integer CLOCKS_PER_US = FREQ / 1_000_000;

//    localparam integer PWM_MIN_CYCLES = (PWM_MIN - 20)*CLOCKS_PER_US;
//    localparam integer PWM_MAX_CYCLES = (PWM_MAX + 20)*CLOCKS_PER_US;

//    reg throttle_sync1, throttle_sync2;

//    always @(posedge clk) begin
//        throttle_sync1 <= pwm_throttle;
//        throttle_sync2 <= throttle_sync1;
//    end

//    reg throttle_prev;

//    reg [31:0] count_throttle;

//    function signed [11:0] normalize_pwm;
//        input [31:0] cycles;
//        integer pulse_us;
//        integer temp;
//    begin

//        pulse_us = (cycles + (CLOCKS_PER_US/2))
//                 / CLOCKS_PER_US;

//        temp = ((pulse_us - PWM_NEUTRAL)*2000)
//              /(PWM_MAX - PWM_MIN);

//        if(temp > 1000)
//            temp = 1000;

//        if(temp < -1000)
//            temp = -1000;

//        if(temp > -20 && temp < 20)
//            normalize_pwm = 0;
//        else
//            normalize_pwm = temp;

//    end
//    endfunction

//    always @(posedge clk) begin

//        if(rst) begin

//            throttle <= -1000;

//            failsafe <= 1;

//            count_throttle <= 0;

//            throttle_prev <= 0;

//        end
//        else begin

//            if(throttle_sync2 && !throttle_prev)
//                count_throttle <= 0;
//            else if(throttle_sync2)
//                count_throttle <= count_throttle + 1;
                
//            if(!throttle_sync2 && throttle_prev) begin
//            // Center the count around neutral (75,000 cycles)
//            // Multiply by 41 and right-shift by 10 (equivalent to dividing by 25)
//            throttle <= (($signed(count_throttle) - 75000) * 41) >>> 10;
//            end
        
////            if(!throttle_sync2 && throttle_prev) begin

////                if((count_throttle >= PWM_MIN_CYCLES) &&
////                   (count_throttle <= PWM_MAX_CYCLES))

////                    throttle <= normalize_pwm(count_throttle);

////            end

            
////                if(!throttle_sync2 && throttle_prev) begin
                
////                    throttle <= count_throttle/50;
                
////                end
                


//            failsafe <= 0;

//            throttle_prev <= throttle_sync2;

//        end
//    end

//endmodule

//`default_nettype wire