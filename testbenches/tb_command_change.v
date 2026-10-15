`timescale 1ns/1ps

module tb_roll_reference_step_rigidbody;

/////////////////////////////////////////////////
// CLOCK
/////////////////////////////////////////////////

reg clk;

always #50 clk = ~clk;

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

wire resetn = ~reset;
wire resetn_Attitude_Roll_Controller_2_tc = resetn;
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

function automatic signed [15:0] to_sfix16_EnX;
input real val;
input integer frac_bits;
begin
    to_sfix16_EnX = $rtoi(val * (2 ** frac_bits));
end
endfunction

/////////////////////////////////////////////////
// DUT
/////////////////////////////////////////////////

Attitude_Roll_Controller_2 DUT
(
    .clk(clk),
    .resetn(resetn),
    .resetn_Attitude_Roll_Controller_2_tc(resetn_Attitude_Roll_Controller_2_tc),
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

    K_u = 0.171 * 1 / 0.003;     // 57

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
        "test.txt",
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

    for(cycle=0; cycle<1500; cycle=cycle+1)
    begin

        /////////////////////////////////////////
        // STEP TO +10 DEG AT CYCLE 100
        /////////////////////////////////////////

        if(cycle < 100)
        begin
            phi_req_real = 0.0;
        end
        else
        begin
            phi_req_real =
                10.0 *
                3.14159265359 /
                180.0;
        end

        /////////////////////////////////////////
        // FEEDBACK TO CONTROLLER
        /////////////////////////////////////////

        phi_raw_req = to_sfix16_EnX(phi_req_real, 14);
        phi_meas    = to_sfix16_EnX(phi_real, 14);
        p_meas      = to_sfix16_EnX(p_real, 10);

        /////////////////////////////////////////
        // WAIT FOR CONTROLLER UPDATE
        /////////////////////////////////////////

        // wait for the actual 4ms-equivalent update
        @(posedge clk);
        while (!ce_out) @(posedge clk);
        /////////////////////////////////////////
        // CONTROLLER OUTPUT
        /////////////////////////////////////////

        u2_real =
            $itor($signed(U2))/
            (2**10);

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
        // SIMPLE STATUS DISPLAY
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
        // INTERNAL HDL DEBUG
        /////////////////////////////////////////

        $display(
            "cy=%0d phi_req=%0d phi_meas=%0d p_meas=%0d",
            cycle,
            DUT.phi_raw_req,
            DUT.phi_meas,
            DUT.p_meas
        );

//        $display(
//            "phi_raw_phi_cmd=%0d Sum1=%0d p_ff=%0d",
//            DUT.phi_raw_phi_cmd_out1,
//            DUT.Sum1_out1,
//            DUT.p_ff_x_reg
//        );

//        $display(
//            "theta_cmd=%0d phi_err=%0d Gain=%0d",
//            DUT.theta_cmd,
//            DUT.phi_error_out1,
//            DUT.Gain_out1
//        );

//        $display(
//            "qref_b=%0d qref_a=%0d p_err=%0d",
//            DUT.qref_before_delay,
//            DUT.qref_after_delay,
//            DUT.p_error_out1
//        );

        $display(
            "I=%0d P=%0d D=%0d U2pre=%0d U2=%0d",
            DUT.Discrete_Time_Integrator_x_reg,
            DUT.Inner_P_out1,
            DUT.Inner_D_out1,
            DUT.U2_before_sat,
            DUT.U2
        );

        $display("");

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
    $display("Simulation Complete");
    $display("");

    $finish;

end

endmodule