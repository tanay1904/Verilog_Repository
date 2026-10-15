`timescale 1 ns / 1 ns

module tb_yaw_control_top;

  // -----------------------------------------------------------
  // 1. Simulation & Clock Parameters
  // -----------------------------------------------------------
  parameter CLK_PERIOD = 10;              // 10 ns period = 100 MHz master clock
                                          // Matches your 10 ns simulation steps
  
  // Set to 1 for fast simulation debugging; set to 0 for real 4 ms hardware timing
  parameter SIM_SPEEDUP = 1;              
  
  // Dynamic calculation of the 4 ms interval based on speedup choice
  localparam integer TICKS_4MS = (SIM_SPEEDUP == 1) ? 20 : 400000; 

  // -----------------------------------------------------------
  // 2. Testbench Signals
  // -----------------------------------------------------------
  reg         clk;
  reg         resetn;
  reg         clk_enable;
  reg signed  [15:0] gz_in;
  reg signed  [15:0] mag_yaw_in;
  reg signed  [15:0] yaw_req;

  wire        ce_out;
  wire signed [15:0] U4;

  integer     ce_counter = 0;
  integer     file_handle;  // File descriptor pointer

  // -----------------------------------------------------------
  // 3. Device Under Test (DUT) Instantiation
  // -----------------------------------------------------------
  yaw_control_top dut (
    .clk        (clk),
    .resetn     (resetn),
    .clk_enable (clk_enable),
    .gz_in      (gz_in),
    .mag_yaw_in (mag_yaw_in),
    .yaw_req    (yaw_req),
    .ce_out     (ce_out),
    .U4         (U4)
  );

  // -----------------------------------------------------------
  // 4. Clock Generation
  // -----------------------------------------------------------
  initial begin
    clk = 1'b0;
    forever #(CLK_PERIOD/2) clk = ~clk;
  end

  // -----------------------------------------------------------
  // 5. Periodic clk_enable Strobe Generator (Every 4 ms Sample Time)
  // -----------------------------------------------------------
  always @(posedge clk) begin
    if (resetn == 1'b0) begin
      clk_enable <= 1'b0;
      ce_counter <= 0;
    end else begin
      if (ce_counter == (TICKS_4MS - 1)) begin
        clk_enable <= 1'b1;  // Strobe high for 1 clock cycle
        ce_counter <= 0;
      end else begin
        clk_enable <= 1'b0;
        ce_counter <= ce_counter + 1;
      end
    end
  end

  // -----------------------------------------------------------
  // 6. Real-Time CSV Data Logging Process
  // -----------------------------------------------------------
  initial begin
    // Open the CSV file for writing
    file_handle = $fopen("yaw_control_sim_results.csv", "w");
    if (file_handle == 0) begin
      $display("[ERROR] Failed to open yaw_control_sim_results.csv for writing!");
      $finish;
    end
    
    // Write CSV Column Headers
    $fdisplay(file_handle, "Time_ns,clk_enable,gz_in(Raw),mag_yaw_in(Raw),yaw_req(Raw),ce_out,U4(Raw)");
  end

  // Log signal state values at every single rising clock edge
  always @(posedge clk) begin
    if (resetn === 1'b1) begin
      $fdisplay(file_handle, "%0d,%b,%d,%d,%d,%b,%d", 
                $time, clk_enable, gz_in, mag_yaw_in, yaw_req, ce_out, U4);
    end
  end

  // -----------------------------------------------------------
  // 7. Test Vectors & Stimulus Process
  // -----------------------------------------------------------
  initial begin
    // Initialize inputs
    resetn     = 1'b0;
    gz_in      = 16'sb0;
    mag_yaw_in = 16'sb0;
    yaw_req    = 16'sb0;

    // Hold active-low reset for 5 clock cycles
    #(CLK_PERIOD * 5);
    @(posedge clk);
    resetn = 1'b1;
    $display("[TB] Reset released. Recording data to CSV file...");

    // --- Scenario 1: System is steady at zero ---
    wait_for_ce(2);
    gz_in      = 16'd0;
    mag_yaw_in = 16'd0;
    yaw_req    = 16'd0;

    // --- Scenario 2: Step change in requested target Yaw ---
    wait_for_ce(1);
    yaw_req    = 16'd4096;   
    $display("[TB] Step Input applied: yaw_req = %d", yaw_req);

    // --- Scenario 3: Sensor feedback update ---
    wait_for_ce(3);
    gz_in      = 16'd500;    
    mag_yaw_in = 16'd2048;   
    $display("[TB] Sensor Feedback shifted: gz_in = %d, mag_yaw_in = %d", gz_in, mag_yaw_in);

    // Run for several more sample loops to let the control loop filter/process
    wait_for_ce(15);

    // Clean up and close the file
    $display("[TB] Simulation finished. Closing data file.");
    $fclose(file_handle);
    $finish;
  end

  // -----------------------------------------------------------
  // Helper Task: Pauses stimulus execution until N sample pulses pass
  // -----------------------------------------------------------
  task wait_for_ce(input integer num_cycles);
    integer i;
    begin
      for (i = 0; i < num_cycles; i = i + 1) begin
        @(posedge clk);
        while (clk_enable !== 1'b1) begin
          @(posedge clk);
        end
      end
    end
  endtask

endmodule // tb_yaw_control_top