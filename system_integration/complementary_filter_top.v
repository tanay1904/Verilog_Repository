`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 23.07.2026 17:43:22
// Design Name: 
// Module Name: top
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


module top
          (input clk,
           input reset,
           input signed [15:0] gx,
           input signed [15:0] gy,
           input signed [15:0] ax,
           input signed [15:0] ay,
           input signed [15:0] az,
           input signed [11:0] dt,
           output ce_out,
           output signed [15:0] phi_acc,
           output signed [15:0] phi_gyro,
           output signed [15:0] phi,
           output signed [15:0] theta,
           output signed [15:0] p_filt,
           output signed [15:0] q_filt);
           
           wire rstn;
           wire clk_enable_250Hz;
           assign rstn = ~reset;
           Complementary_filter u_comp_fil
          (.clk             (clk),
           .reset           (reset),
           .clk_enable      (clk_enable_250Hz),
           .gx              (gx),
           .gy              (gy),
           .ax              (ax),
           .ay              (ay),
           .az              (az),
           .dt              (dt),
           .ce_out          (ce_out),
           .phi             (phi),
           .theta           (theta),
           .p_filt          (p_filt),
           .q_filt          (q_filt),
           .phi_gyro        (phi_gyro)
           );
pulse_gen          
#(
    .CLK_FREQ   (10_000_000),
    .PULSE_FREQ (250)
)
u_pulse_gen   (
            .clk            (clk),
            .rstn           (rstn),
            .pulse_out_250Hz (clk_enable_250Hz)
    );
    phi_acc u_phi_accel (
    .x_in                    (az),// sfix16_En11
    .y_in                    (ay),// sfix16_En11
    .angle                  (phi_acc)// sfix16_En13
    );       
endmodule
