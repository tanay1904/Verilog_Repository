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


module PWM_channels(
    input  clk,
    input  rstn,
    input  clk_en,

    input  [15:0] duty_int_1,
    input  [15:0] duty_int_2,
    input  [15:0] duty_int_3,
    input  [15:0] duty_int_4,

    output pwm_out_1,
    output pwm_out_2,
    output pwm_out_3,
    output pwm_out_4
);

    localparam [1:0] FINAL_VALUE = 2'd2;
    wire rst = rstn;
    PWM_Custom #(
        .R(16),
        .TIMER_BITS(2)
    ) pwm_ch1 (
        .clk(clk),
        .rst(rst),
        .clk_en(clk_en),
        .duty_channel_int(duty_int_1),
        .Final_value(FINAL_VALUE),
        .pwm_out(pwm_out_1)
    );

    PWM_Custom #(
        .R(16),
        .TIMER_BITS(2)
    ) pwm_ch2 (
        .clk(clk),
        .rst(rst),
        .clk_en(clk_en),
        .duty_channel_int(duty_int_2),
        .Final_value(FINAL_VALUE),
        .pwm_out(pwm_out_2)
    );

    PWM_Custom #(
        .R(16),
        .TIMER_BITS(2)
    ) pwm_ch3 (
        .clk(clk),
        .rst(rst),
        .clk_en(clk_en),
        .duty_channel_int(duty_int_3),
        .Final_value(FINAL_VALUE),
        .pwm_out(pwm_out_3)
    );

    PWM_Custom #(
        .R(16),
        .TIMER_BITS(2)
    ) pwm_ch4 (
        .clk(clk),
        .rst(rst),
        .clk_en(clk_en),
        .duty_channel_int(duty_int_4),
        .Final_value(FINAL_VALUE),
        .pwm_out(pwm_out_4)
    );

endmodule


module PWM_Custom #(
parameter R = 16,
parameter TIMER_BITS = 2)
(
  input clk,
  input rst,
  input clk_en,
  input [R-1:0] duty_channel_int, // actual duty is this divided by max count
  input [TIMER_BITS-1:0] Final_value,
  output pwm_out

    );
    
    
    
    wire tick;
    // up counter for channel 1
    // sequential and combinational circuit separate coding style 
    
    // internal reg declaration
    reg [R-1:0] Q_reg;
    reg [R-1:0] Q_next;
    reg d_reg;
    reg d_next;
    //wire need_to_warp;
    
    // state reg with asych reset
    always@(posedge clk, negedge rst)
    begin
         if(~rst)
           begin
            Q_reg <= 1'b0;
            d_reg <= 1'b0;
           end
         else if (tick)
                begin
                  Q_reg <= Q_next;
                  d_reg <= d_next;
                end
              else 
              begin
                  Q_reg <= Q_reg;
                  d_reg <= d_reg;
              end
    end
    
    
    // next state logic
    always@(*)
    begin 
         Q_next = Q_reg + 1'b1;
         d_next = Q_reg < duty_channel_int;
    end
    
    
    // Output logic
     assign pwm_out = d_next ;
     
    // assign need_to_warp = Q_reg == 15'b100111000100000; // reset counter when conter register stores 20,000
     
     
     Timer #(.BITS(TIMER_BITS)) timer_i(
     .clk(clk),
     .rst(rst),
     .enable(1'b1),
     .Final_value(Final_value),
     .done(tick)
     );
    
endmodule



module Timer #(
    parameter BITS = 2
)(
    input clk,
    input rst,
    input enable,
    input [BITS-1:0] Final_value,
    output done
);

    reg [BITS-1:0] Q_reg;
    reg [BITS-1:0] Q_next;
    wire warp;

    always @(posedge clk or negedge rst) begin
        if (~rst|done)
            Q_reg <= 0;
        else if (enable)
            Q_reg <= Q_next;
    end

    always @(*) begin
        if (done)
            Q_next = 0;
        else
            Q_next = Q_reg + 1'b1;
    end

    assign done = (Q_reg == Final_value);
    

endmodule