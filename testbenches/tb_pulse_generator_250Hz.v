`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09.07.2026 18:56:29
// Design Name: 
// Module Name: tb_pulse_generator_250Hz
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


module tb_pulse_generator_250Hz(

    );
    parameter PERIOD = 100;
    reg clk;
    reg rstn;
    wire pulse_out_250Hz;
    
    initial begin
    clk = 0;
    rstn = 0;
    end
    always #(PERIOD/2) clk = ~clk; 
    
    initial begin
    rstn = 0;
    #5000;
    rstn  = 1;
    end
    
    
    //-------------------------------------------------------
    // Count pulses
    //-------------------------------------------------------
    pulse_gen dut(
    .clk(clk),
    .rstn(rstn),
    .pulse_out_250Hz(pulse_out_250Hz)
    );

    //-------------------------------------------------------
    // Count pulses
    //-------------------------------------------------------
    integer pulse_count = 0;

    always @(posedge clk) begin
        if (pulse_out_250Hz) begin
            pulse_count = pulse_count + 1;
            $display("[%0t ps] Pulse %0d",
                     $time, pulse_count);
        end
    end
    //-------------------------------------------------------
    // Finish simulation
    //-------------------------------------------------------
    initial begin
        #25_000_000;      // 25 us
        $display("--------------------------------");
        $display("Total pulses = %0d", pulse_count);
        $display("--------------------------------");
        $finish;
    end
endmodule
