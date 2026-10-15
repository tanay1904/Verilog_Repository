`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 25.09.2026 18:19:36
// Design Name: 
// Module Name: pwm_gen
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02.07.2026 19:29:07
// Design Name: 
// Module Name: pwm_gen
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


`timescale 1ns / 1ps

module PWM_channels #(
    parameter integer CLK_FREQ_MHZ = 10
)(
    input  clk,
    input  rstn,
    input  clk_en,

    // Direct microsecond values: 1000 to 2000
    input  [15:0] duty_int_1,
    input  [15:0] duty_int_2,
    input  [15:0] duty_int_3,
    input  [15:0] duty_int_4,

    output pwm_out_1,
    output pwm_out_2,
    output pwm_out_3,
    output pwm_out_4
);

    PWM_Custom #(
        .CLK_FREQ_MHZ(CLK_FREQ_MHZ)
    ) pwm_ch1 (
        .clk     (clk),
        .rstn    (rstn),
        .clk_en  (clk_en),
        .duty_us (duty_int_1),
        .pwm_out (pwm_out_1)
    );

    PWM_Custom #(
        .CLK_FREQ_MHZ(CLK_FREQ_MHZ)
    ) pwm_ch2 (
        .clk     (clk),
        .rstn    (rstn),
        .clk_en  (clk_en),
        .duty_us (duty_int_2),
        .pwm_out (pwm_out_2)
    );

    PWM_Custom #(
        .CLK_FREQ_MHZ(CLK_FREQ_MHZ)
    ) pwm_ch3 (
        .clk     (clk),
        .rstn    (rstn),
        .clk_en  (clk_en),
        .duty_us (duty_int_3),
        .pwm_out (pwm_out_3)
    );

    PWM_Custom #(
        .CLK_FREQ_MHZ(CLK_FREQ_MHZ)
    ) pwm_ch4 (
        .clk     (clk),
        .rstn    (rstn),
        .clk_en  (clk_en),
        .duty_us (duty_int_4),
        .pwm_out (pwm_out_4)
    );

endmodule


module PWM_Custom #(
    parameter integer CLK_FREQ_MHZ = 50
)(
    input  wire        clk,
    input  wire        rstn,
    input  wire        clk_en,
    input  wire [15:0] duty_us,
    output wire        pwm_out
);

    localparam integer CLKS_PER_US        = CLK_FREQ_MHZ;
    localparam integer FRAME_US           = 2000;
    localparam integer FRAME_CLKS         = CLK_FREQ_MHZ * FRAME_US;
    localparam integer FRAME_COUNTER_BITS = $clog2(FRAME_CLKS);
    localparam integer US_COUNTER_BITS    = (CLKS_PER_US <= 1) ? 1 : $clog2(CLKS_PER_US);

    reg [US_COUNTER_BITS-1:0]    us_counter;
    wire                         one_us_tick;
    reg [FRAME_COUNTER_BITS-1:0] frame_counter;
    
    // Shadow register: latches new duty synchronously at frame start to prevent glitching
    reg [15:0]                   duty_us_shadow;

    assign one_us_tick = (us_counter == CLKS_PER_US - 1);

    // 1 us tick generator
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            us_counter <= 0;
        end else if (clk_en) begin
            if (one_us_tick)
                us_counter <= 0;
            else
                us_counter <= us_counter + 1'b1;
        end
    end

    // 20 ms frame counter with synchronized shadow latch
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            frame_counter  <= 0;
            duty_us_shadow <= 0;
        end else if (clk_en && one_us_tick) begin
            if (frame_counter == FRAME_US - 1) begin
                frame_counter  <= 0;
                duty_us_shadow <= duty_us; // Latch new duty at start of new frame
            end else begin
                if (frame_counter == 0)
                    duty_us_shadow <= duty_us;
                frame_counter <= frame_counter + 1'b1;
            end
        end
    end

    // Gated with rstn and clk_en so outputs are strictly LOW during reset/idle
    assign pwm_out = rstn && clk_en && (frame_counter < duty_us_shadow);

endmodule