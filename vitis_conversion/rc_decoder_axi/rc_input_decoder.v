`timescale 1ns/1ps
`default_nettype none

module rc_input_decoder(
    input  wire clk,
    input  wire rst,

    input  wire pwm_roll,
    input  wire pwm_pitch,
    input  wire pwm_throttle,
    input  wire pwm_yaw,

    output reg  signed [11:0] roll,
    output reg  signed [11:0] pitch,
    output reg  signed [11:0] throttle,
    output reg  signed [11:0] yaw,

    output reg failsafe
);

// -----------------------------------------------------------------------------
// PARAMETERS
// -----------------------------------------------------------------------------
parameter integer PWM_MIN     = 1000;  // us
parameter integer PWM_MAX     = 2000;  // us
parameter integer PWM_NEUTRAL = 1500;  // us

parameter integer FREQ            = 100_000_000;
parameter integer CLOCKS_PER_US   = FREQ / 1_000_000;

parameter integer TIMEOUT_MS      = 50;
parameter integer TIMEOUT_CYCLES  = (FREQ / 1000) * TIMEOUT_MS;

localparam integer PWM_MIN_CYCLES = PWM_MIN * CLOCKS_PER_US;
localparam integer PWM_MAX_CYCLES = PWM_MAX * CLOCKS_PER_US;

// -----------------------------------------------------------------------------
// SYNCHRONIZERS
// -----------------------------------------------------------------------------
reg pwm_roll_sync1=0, pwm_roll_sync2=0;
reg pwm_pitch_sync1=0, pwm_pitch_sync2=0;
reg pwm_throttle_sync1=0, pwm_throttle_sync2=0;
reg pwm_yaw_sync1=0, pwm_yaw_sync2=0;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        pwm_roll_sync1 <= 0; pwm_roll_sync2 <= 0;
        pwm_pitch_sync1 <= 0; pwm_pitch_sync2 <= 0;
        pwm_throttle_sync1 <= 0; pwm_throttle_sync2 <= 0;
        pwm_yaw_sync1 <= 0; pwm_yaw_sync2 <= 0;
    end else begin
        pwm_roll_sync1     <= pwm_roll;
        pwm_roll_sync2     <= pwm_roll_sync1;

        pwm_pitch_sync1    <= pwm_pitch;
        pwm_pitch_sync2    <= pwm_pitch_sync1;

        pwm_throttle_sync1 <= pwm_throttle;
        pwm_throttle_sync2 <= pwm_throttle_sync1;

        pwm_yaw_sync1      <= pwm_yaw;
        pwm_yaw_sync2      <= pwm_yaw_sync1;
    end
end

// -----------------------------------------------------------------------------
// PER-CHANNEL REGISTERS
// -----------------------------------------------------------------------------
reg [19:0] counter_roll=0, counter_pitch=0, counter_throttle=0, counter_yaw=0;
reg [19:0] measured_roll=0, measured_pitch=0, measured_throttle=0, measured_yaw=0;

reg pwm_roll_prev=0, pwm_pitch_prev=0, pwm_throttle_prev=0, pwm_yaw_prev=0;

reg valid_roll=0, valid_pitch=0, valid_throttle=0, valid_yaw=0;

reg [31:0] timeout_counter_roll=0;
reg [31:0] timeout_counter_pitch=0;
reg [31:0] timeout_counter_throttle=0;
reg [31:0] timeout_counter_yaw=0;

reg signed [11:0] roll_next=0, pitch_next=0, yaw_next=0;
reg signed [11:0] throttle_next=-12'sd1000;

// -----------------------------------------------------------------------------
// NORMALIZATION FUNCTION
// -----------------------------------------------------------------------------
function automatic signed [11:0] normalize_pwm;
    input [19:0] pulse_cycles;
    integer pulse_us, offset;
begin
    pulse_us = pulse_cycles / CLOCKS_PER_US;

    if (pulse_us <= PWM_MIN)
        normalize_pwm = -12'sd1000;
    else if (pulse_us >= PWM_MAX)
        normalize_pwm = 12'sd1000;
    else begin
        offset = pulse_us - PWM_NEUTRAL + 1;
        normalize_pwm = (offset * 2000) / (PWM_MAX - PWM_MIN);
    end
end
endfunction

// -----------------------------------------------------------------------------
// MAIN LOGIC - edge-aware timeout, stable valid behaviour
// -----------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        // reset everything
        counter_roll <= 0; counter_pitch <= 0; counter_throttle <= 0; counter_yaw <= 0;
        measured_roll <= 0; measured_pitch <= 0; measured_throttle <= 0; measured_yaw <= 0;

        valid_roll <= 0; valid_pitch <= 0; valid_throttle <= 0; valid_yaw <= 0;

        timeout_counter_roll <= 0;
        timeout_counter_pitch <= 0;
        timeout_counter_throttle <= 0;
        timeout_counter_yaw <= 0;

        roll_next <= 0; 
        pitch_next <= 0; 
        yaw_next <= 0; 
        throttle_next <= -1000;

        failsafe <= 1;

        pwm_roll_prev <= 0;
        pwm_pitch_prev <= 0;
        pwm_throttle_prev <= 0;
        pwm_yaw_prev <= 0;
    end else begin

        // ========================= ROLL =========================
        // Rising edge: reset counter and timeout (edge-protected)
        if (pwm_roll_sync2 && !pwm_roll_prev) begin
            counter_roll <= 0;
            timeout_counter_roll <= 0;
        end
        else if (pwm_roll_sync2) begin
            // count while high
            counter_roll <= counter_roll + 1;
        end
        // Falling edge: capture measurement; set valid if in range; safety-clear timeout
        if (!pwm_roll_sync2 && pwm_roll_prev) begin
            measured_roll <= counter_roll;
            if (counter_roll >= PWM_MIN_CYCLES && counter_roll <= PWM_MAX_CYCLES)
                valid_roll <= 1;
            // DO NOT clear valid here on out-of-range; leave clearing to timeout
            // safety reset timeout so any prior accumulation doesn't persist
            timeout_counter_roll <= 0;
        end
        // Timeout increments only during LOW (idle) phase
        if (!pwm_roll_sync2) begin
            if (timeout_counter_roll < TIMEOUT_CYCLES)
                timeout_counter_roll <= timeout_counter_roll + 1;
        end
        // Expire valid when timeout threshold reached
        if (timeout_counter_roll >= TIMEOUT_CYCLES)
            valid_roll <= 0;

        pwm_roll_prev <= pwm_roll_sync2;


        // ========================= PITCH =========================
        if (pwm_pitch_sync2 && !pwm_pitch_prev) begin
            counter_pitch <= 0;
            timeout_counter_pitch <= 0;
        end
        else if (pwm_pitch_sync2) begin
            counter_pitch <= counter_pitch + 1;
        end

        if (!pwm_pitch_sync2 && pwm_pitch_prev) begin
            measured_pitch <= counter_pitch;
            if (counter_pitch >= PWM_MIN_CYCLES && counter_pitch <= PWM_MAX_CYCLES)
                valid_pitch <= 1;
            timeout_counter_pitch <= 0;
        end

        if (!pwm_pitch_sync2) begin
            if (timeout_counter_pitch < TIMEOUT_CYCLES)
                timeout_counter_pitch <= timeout_counter_pitch + 1;
        end

        if (timeout_counter_pitch >= TIMEOUT_CYCLES)
            valid_pitch <= 0;

        pwm_pitch_prev <= pwm_pitch_sync2;


        // ========================= THROTTLE =========================
        if (pwm_throttle_sync2 && !pwm_throttle_prev) begin
            counter_throttle <= 0;
            timeout_counter_throttle <= 0;
        end
        else if (pwm_throttle_sync2) begin
            counter_throttle <= counter_throttle + 1;
        end

        if (!pwm_throttle_sync2 && pwm_throttle_prev) begin
            measured_throttle <= counter_throttle;
            if (counter_throttle >= PWM_MIN_CYCLES && counter_throttle <= PWM_MAX_CYCLES)
                valid_throttle <= 1;
            timeout_counter_throttle <= 0;
        end

        if (!pwm_throttle_sync2) begin
            if (timeout_counter_throttle < TIMEOUT_CYCLES)
                timeout_counter_throttle <= timeout_counter_throttle + 1;
        end

        if (timeout_counter_throttle >= TIMEOUT_CYCLES)
            valid_throttle <= 0;

        pwm_throttle_prev <= pwm_throttle_sync2;


        // ========================= YAW =========================
        if (pwm_yaw_sync2 && !pwm_yaw_prev) begin
            counter_yaw <= 0;
            timeout_counter_yaw <= 0;
        end
        else if (pwm_yaw_sync2) begin
            counter_yaw <= counter_yaw + 1;
        end

        if (!pwm_yaw_sync2 && pwm_yaw_prev) begin
            measured_yaw <= counter_yaw;
            if (counter_yaw >= PWM_MIN_CYCLES && counter_yaw <= PWM_MAX_CYCLES)
                valid_yaw <= 1;
            timeout_counter_yaw <= 0;
        end

        if (!pwm_yaw_sync2) begin
            if (timeout_counter_yaw < TIMEOUT_CYCLES)
                timeout_counter_yaw <= timeout_counter_yaw + 1;
        end

        if (timeout_counter_yaw >= TIMEOUT_CYCLES)
            valid_yaw <= 0;

        pwm_yaw_prev <= pwm_yaw_sync2;


        // ========================= FAILSAFE =========================
        failsafe <= !(valid_roll && valid_pitch && valid_throttle && valid_yaw);


        // ========================= NEXT-STAGE OUTPUTS =========================
        // note: these use current valid flags; outputs are pipelined one more stage below
        roll_next     <= valid_roll     ? normalize_pwm(measured_roll)      : 0;
        pitch_next    <= valid_pitch    ? normalize_pwm(measured_pitch)     : 0;
        yaw_next      <= valid_yaw      ? normalize_pwm(measured_yaw)       : 0;
        throttle_next <= (!failsafe && valid_throttle)
                        ? normalize_pwm(measured_throttle)
                        : -12'sd1000;
    end
end

// -----------------------------------------------------------------------------
// PIPELINED REGISTER OUTPUTS
// -----------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        roll <= 0;
        pitch <= 0;
        yaw <= 0;
        throttle <= 0;
    end else begin
        roll <= roll_next;
        pitch <= pitch_next;
        yaw <= yaw_next;
        throttle <= throttle_next;
    end
end

endmodule
`default_nettype wire