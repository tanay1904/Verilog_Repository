`timescale 1 ns / 1 ns

module tb_Acceleration_vectirs;

    // Inputs
    reg signed [15:0] ax;
    reg signed [15:0] ay;
    reg signed [15:0] az;
    reg signed [15:0] phi;
    reg signed [15:0] theta;
    reg signed [15:0] psi;

    // Outputs
    wire signed [15:0] ax_global;
    wire signed [15:0] ay_global;

    // Instantiate the Unit Under Test (UUT)
    Acceleration_vectirs uut (
        .ax(ax),
        .ay(ay),
        .az(az),
        .phi(phi),
        .theta(theta),
        .psi(psi),
        .ax_global(ax_global),
        .ay_global(ay_global)
    );

    // Helper function: Convert Real to sfix16_En10
    function signed [15:0] to_sfix(input real val);
        begin
            to_sfix = val * 1024.0; // Multiply by 2^10
        end
    endfunction

    // Helper function: Convert Degrees to Radians, then to sfix16_En10
    function signed [15:0] deg2sfix(input real deg);
        real rad;
        begin
            rad = deg * (3.1415926535 / 180.0);
            deg2sfix = rad * 1024.0; // Multiply by 2^10
        end
    endfunction

    // Task to format and print the results beautifully
    task print_results;
        input integer test_num;
        input [80*8:1] test_name; // String up to 80 chars
        real ax_real, ay_real, az_real;
        real ax_g_real, ay_g_real;
        real phi_deg, theta_deg, psi_deg;
        begin
            // Convert fixed-point back to real numbers for display
            ax_real = $itor(ax) / 1024.0;
            ay_real = $itor(ay) / 1024.0;
            az_real = $itor(az) / 1024.0;
            
            // Convert radians back to degrees
            phi_deg   = ($itor(phi)   / 1024.0) * (180.0 / 3.1415926535);
            theta_deg = ($itor(theta) / 1024.0) * (180.0 / 3.1415926535);
            psi_deg   = ($itor(psi)   / 1024.0) * (180.0 / 3.1415926535);
            
            // Convert global outputs back to real numbers
            ax_g_real = $itor(ax_global) / 1024.0;
            ay_g_real = $itor(ay_global) / 1024.0;

            // Print formatted output
            $display("Test %0d: %s", test_num, test_name);
            $display("  Body Accel : ax = %5.2f, ay = %5.2f, az = %5.2f", ax_real, ay_real, az_real);
            $display("  Euler Ang  : phi = %5.0f deg, theta = %5.0f deg, psi = %5.0f deg", phi_deg, theta_deg, psi_deg);
            $display("  Global Acc : ax_global = %5.2f, ay_global = %5.2f", ax_g_real, ay_g_real);
            $display("-------------------------------------------------------------------------");
        end
    endtask

    // Stimulus Process
    initial begin
        $display("-------------------------------------------------------------------------");
        $display("Starting Acceleration Vectors Testbench");
        $display("-------------------------------------------------------------------------");

        // Test 1: No rotation
        phi = deg2sfix(0.0); theta = deg2sfix(0.0); psi = deg2sfix(0.0);
        ax = to_sfix(5.0); ay = to_sfix(2.0); az = to_sfix(9.81);
        #10; // Wait for combinational logic
        print_results(1, "No rotation (Linear Acceleration)");

        // Test 2: Gravity only
        phi = deg2sfix(0.0); theta = deg2sfix(0.0); psi = deg2sfix(0.0);
        ax = to_sfix(0.0); ay = to_sfix(0.0); az = to_sfix(9.81);
        #10;
        print_results(2, "Gravity only (Hovering)");

        // Test 3: Roll 90 deg
        phi = deg2sfix(90.0); theta = deg2sfix(0.0); psi = deg2sfix(0.0);
        ax = to_sfix(0.0); ay = to_sfix(0.0); az = to_sfix(9.81);
        #10;
        print_results(3, "Roll 90 deg (Gravity rotates into Y)");

        // Test 4: Pitch 90 deg
        phi = deg2sfix(0.0); theta = deg2sfix(90.0); psi = deg2sfix(0.0);
        ax = to_sfix(0.0); ay = to_sfix(0.0); az = to_sfix(9.81);
        #10;
        print_results(4, "Pitch 90 deg (Gravity moves into X)");

        $stop; // Pause the simulator
    end

endmodule