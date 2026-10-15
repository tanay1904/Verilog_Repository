`timescale 1ns/1ps

module tb_roll_reference_ramp_rigidbody;

/////////////////////////////////////////////////
// CLOCK
/////////////////////////////////////////////////

reg clk;

always #5 clk = ~clk;

/////////////////////////////////////////////////
// DUT INPUTS
/////////////////////////////////////////////////

reg reset;
reg clk_enable;

reg signed [15:0] phi_raw_req;
reg signed [15:0] phi_meas;
reg signed [15:0] p_meas;

/////////////////////////////////////////////////
// DUT OUTPUTS
/////////////////////////////////////////////////

wire signed [15:0] U2;
wire ce_out;

/////////////////////////////////////////////////
// REAL VARIABLES
/////////////////////////////////////////////////

real phi_req_real;
real phi_real;
real p_real;
real u2_real;

real Ts;

// Rigid body gain = L/Ix
real K_u;

integer cycle;
integer outfile;

/////////////////////////////////////////////////
// Q16 HELPER
/////////////////////////////////////////////////

function signed [15:0] to_fixed;
input real val;
input integer frac_bits;
begin
    to_fixed = $rtoi(val * (2**frac_bits));
end
endfunction

/////////////////////////////////////////////////
// DUT
/////////////////////////////////////////////////

Attitude_Roll_Controller_2 DUT
(
    .clk(clk),
    .reset(reset),
    .clk_enable(clk_enable),

    .phi_raw_req(phi_raw_req),
    .phi_meas(phi_meas),
    .p_meas(p_meas),

    .ce_out(ce_out),
    .U2(U2)
);

/////////////////////////////////////////////////
// TEST
/////////////////////////////////////////////////

initial
begin

    /////////////////////////////////////////////
    // SETUP
    /////////////////////////////////////////////

    clk = 0;

    reset      = 1;
    clk_enable = 1;

    Ts = 0.004;

    /////////////////////////////////////////////
    // RIGID BODY PLANT
    //
    // Ix = 0.003
    // L  = 0.171
    //
    // p_dot = (L/Ix)*U2
    //
    /////////////////////////////////////////////

    K_u = 1 / 0.003;

    /////////////////////////////////////////////
    // INITIAL CONDITIONS
    /////////////////////////////////////////////

    phi_req_real = 0.0;

    phi_real = 0.0;
    p_real   = 0.0;

    phi_raw_req = 0;
    phi_meas    = 0;
    p_meas      = 0;

    /////////////////////////////////////////////
    // OUTPUT FILE
    /////////////////////////////////////////////

    outfile = $fopen(
        "roll_reference_ramp_rigidbody.txt",
        "w"
    );

    $fwrite(
        outfile,
        "cycle phi_req phi_meas p_meas U2\n"
    );

    /////////////////////////////////////////////
    // RESET
    /////////////////////////////////////////////

    repeat(10) @(posedge clk);

    reset = 0;

    /////////////////////////////////////////////
    // MAIN LOOP
    /////////////////////////////////////////////

    for(cycle=0; cycle<2000; cycle=cycle+1)
    begin

        /////////////////////////////////////////
        // RAMP INPUT
        //
        // 0 deg -> 20 deg
        // over 1000 cycles
        /////////////////////////////////////////

        if(cycle < 100)
        begin
            phi_req_real = 0.0;
        end
        else if(cycle < 1100)
        begin

            phi_req_real =
            (
                (cycle - 100)
                *
                20.0
                /
                1000.0
            )
            *
            3.14159265359
            /
            180.0;

        end
        else
        begin

            phi_req_real =
                20.0 *
                3.14159265359 /
                180.0;

        end

        /////////////////////////////////////////
        // FEEDBACK TO CONTROLLER
        /////////////////////////////////////////

        phi_raw_req = to_fixed(phi_req_real, 14);
        phi_meas    = to_fixed(phi_real, 14);
        p_meas      = to_fixed(p_real, 10);

        /////////////////////////////////////////
        // WAIT FOR CONTROLLER UPDATE
        /////////////////////////////////////////

        @(posedge clk);

        /////////////////////////////////////////
        // CONTROLLER OUTPUT
        /////////////////////////////////////////

        u2_real =
            $itor($signed(U2))
            / (2**10);

        /////////////////////////////////////////
        // RIGID BODY ROLL DYNAMICS
        //
        // p_dot   = (L/Ix)*U2
        // phi_dot = p
        /////////////////////////////////////////

        p_real =
            p_real +
            (K_u * u2_real) * Ts;

        phi_real =
            phi_real +
            p_real * Ts;

        /////////////////////////////////////////
        // STATUS DISPLAY
        /////////////////////////////////////////

        $display(
            "cycle=%0d req=%f phi=%f p=%f U2=%f",
            cycle,
            phi_req_real,
            phi_real,
            p_real,
            u2_real
        );

        /////////////////////////////////////////
        // SAVE FILE
        /////////////////////////////////////////

        $fwrite(
            outfile,
            "%0d %f %f %f %f\n",
            cycle,
            phi_req_real,
            phi_real,
            p_real,
            u2_real
        );

    end

    /////////////////////////////////////////////
    // CLEANUP
    /////////////////////////////////////////////

    $fclose(outfile);

    $display("");
    $display("Ramp Simulation Complete");
    $display("");

    $finish;

end

endmodule