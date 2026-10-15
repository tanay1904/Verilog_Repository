`timescale 1 ns / 1 ns

module tb_Position_Filter;

    //=============================================================
    // Testbench Signals
    //=============================================================
    reg clk;
    reg reset;
    reg clk_enable;

    // Inputs: sfix16_En10
    reg signed [15:0] X_raw;
    reg signed [15:0] Y_raw;
    reg signed [15:0] ax_global;
    reg signed [15:0] ay_global;

    // Outputs: sfix16_En10
    wire ce_out;
    wire signed [15:0] Uest;
    wire signed [15:0] Vest;
    wire signed [15:0] Yest;
    wire signed [15:0] Xest;

    wire resetn = ~reset;


    //=============================================================
    // Reference Model Parameters
    //
    // These are derived from Position_Filter2.v:
    //
    // Integration coefficient:
    //      131 / 2^15
    //
    // X:
    //      K1x = 9216 / 2^12 = 2.25
    //      K2x = 12288 / 2^12 = 3.0
    //
    // Y:
    //      K1y = 576 / 2^8 = 2.25
    //      K2y = 12288 / 2^12 = 3.0
    //=============================================================
    real Ts;

    real K1_X;
    real K2_X;

    real K1_Y;
    real K2_Y;


    //=============================================================
    // Reference Model State
    //=============================================================
    real Xest_ref;
    real Yest_ref;

    real Uest_ref;
    real Vest_ref;

    // Explicit model of:
    //
    //     Unit_Delay_out1
    //     Unit_Delay1_out1
    //
    // These are updated with the PREVIOUS X/Y state.
    real X_delay_ref;
    real Y_delay_ref;


    //=============================================================
    // Temporary values
    //=============================================================
    real X_raw_real;
    real Y_raw_real;

    real ax_real;
    real ay_real;

    real ex_ref;
    real ey_ref;

    real X_prev_ref;
    real Y_prev_ref;

    real U_prev_ref;
    real V_prev_ref;


    //=============================================================
    // Sample counter
    //=============================================================
    integer sample_count;


    //=============================================================
    // sfix16_En10 -> real
    //=============================================================
    function automatic real sfix_to_real10;
        input signed [15:0] val;

        begin
            sfix_to_real10 = $itor(val) / 1024.0;
        end
    endfunction


    //=============================================================
    // real -> sfix16_En10
    //
    // Used only to generate input stimuli.
    //=============================================================
    function automatic signed [15:0] real_to_sfix10;
        input real val;

        real scaled;

        begin
            scaled = val * 1024.0;

            if (scaled > 32767.0)
                real_to_sfix10 = 16'sh7FFF;

            else if (scaled < -32768.0)
                real_to_sfix10 = -16'sd32768;

            else
                real_to_sfix10 = $rtoi(scaled);
        end
    endfunction


    //=============================================================
    // Reset reference model
    //=============================================================
    task automatic reset_ref_model;

        begin
            Xest_ref = 0.0;
            Yest_ref = 0.0;

            Uest_ref = 0.0;
            Vest_ref = 0.0;

            X_delay_ref = 0.0;
            Y_delay_ref = 0.0;

            X_prev_ref = 0.0;
            Y_prev_ref = 0.0;

            U_prev_ref = 0.0;
            V_prev_ref = 0.0;

            ex_ref = 0.0;
            ey_ref = 0.0;
        end

    endtask


    //=============================================================
    // Reference model update
    //
    // This mirrors the STRUCTURE of Position_Filter2.v.
    //
    // RTL ordering:
    //
    //   error uses Unit_Delay_out1 / Unit_Delay1_out1
    //
    //   velocity state uses:
    //       old velocity + acceleration correction
    //
    //   position state uses:
    //       old velocity + position correction
    //
    //   Unit_Delay registers receive the OLD position state.
    //=============================================================
    task automatic position_filter_ref_rtl;

        begin

            //---------------------------------------------------------
            // Convert inputs
            //---------------------------------------------------------
            X_raw_real = sfix_to_real10(X_raw);
            Y_raw_real = sfix_to_real10(Y_raw);

            ax_real = sfix_to_real10(ax_global);
            ay_real = sfix_to_real10(ay_global);


            //---------------------------------------------------------
            // Save CURRENT states before updating them
            //---------------------------------------------------------
            X_prev_ref = Xest_ref;
            Y_prev_ref = Yest_ref;

            U_prev_ref = Uest_ref;
            V_prev_ref = Vest_ref;


            //---------------------------------------------------------
            // IMPORTANT:
            //
            // The generated RTL does NOT use Xk/Yk directly here.
            //
            // It uses:
            //
            //     Unit_Delay_out1
            //     Unit_Delay1_out1
            //
            // which contain the previous Xk/Yk state.
            //---------------------------------------------------------
            ex_ref = X_raw_real - X_delay_ref;
            ey_ref = Y_raw_real - Y_delay_ref;


            //---------------------------------------------------------
            // X velocity
            //
            // U[k+1] =
            //     U[k] + Ts*(ax + 2.25*ex)
            //---------------------------------------------------------
            Uest_ref =
                U_prev_ref +
                Ts * (ax_real + K1_X * ex_ref);


            //---------------------------------------------------------
            // Y velocity
            //
            // V[k+1] =
            //     V[k] + Ts*(ay + 2.25*ey)
            //---------------------------------------------------------
            Vest_ref =
                V_prev_ref +
                Ts * (ay_real + K1_Y * ey_ref);


            //---------------------------------------------------------
            // X position
            //
            // X[k+1] =
            //     X[k] + Ts*(U[k] + 3*ex)
            //
            // NOTE:
            // Uses OLD velocity U_prev_ref.
            //---------------------------------------------------------
            Xest_ref =
                X_prev_ref +
                Ts * (U_prev_ref + K2_X * ex_ref);


            //---------------------------------------------------------
            // Y position
            //---------------------------------------------------------
            Yest_ref =
                Y_prev_ref +
                Ts * (V_prev_ref + K2_Y * ey_ref);


            //---------------------------------------------------------
            // Update explicit delay registers
            //
            // RTL:
            //
            // Unit_Delay_out1 <= Xk;
            // Unit_Delay1_out1 <= Yk;
            //
            // So the delay registers receive the OLD position state
            // at the same state-update event.
            //---------------------------------------------------------
            X_delay_ref = X_prev_ref;
            Y_delay_ref = Y_prev_ref;

        end

    endtask


    //=============================================================
    // Hold current stimulus for N ce_out samples
    //=============================================================
    task automatic hold_for_samples;

        input integer n;

        integer k;

        begin

            for (k = 0; k < n; k = k + 1)
                @(posedge ce_out);

        end

    endtask


    //=============================================================
    // DUT
    //=============================================================
    Position_Filter2 uut (

        .clk(clk),

        .resetn(resetn),

        .resetn_Position_Filter2_tc(resetn),

        .clk_enable(clk_enable),

        .X_raw(X_raw),
        .Y_raw(Y_raw),

        .ax_global(ax_global),
        .ay_global(ay_global),

        .ce_out(ce_out),

        .Uest(Uest),
        .Vest(Vest),

        .Yest(Yest),
        .Xest(Xest)

    );


    //=============================================================
    // Clock
    //
    // 10 ns period = 100 MHz
    //=============================================================
    always #5 clk = ~clk;


    //=============================================================
    // Reference vs DUT comparison
    //
    // #1:
    // Allow DUT nonblocking state assignments to settle before
    // sampling the outputs.
    //=============================================================
    always @(posedge ce_out) begin

        #1;

        //---------------------------------------------------------
        // Update reference
        //---------------------------------------------------------
        position_filter_ref_rtl();


        //---------------------------------------------------------
        // Header every 20 samples
        //---------------------------------------------------------
        if ((sample_count % 20) == 0) begin

            $display("");
            $display("==============================================================================================================");
            $display(" SAMPLE |          X POSITION          |          Y POSITION          |          X VELOCITY          |          Y VELOCITY");
            $display("        |       REF       DUT       ERR |       REF       DUT       ERR |       REF       DUT       ERR |       REF       DUT       ERR");
            $display("--------------------------------------------------------------------------------------------------------------");

        end


        //---------------------------------------------------------
        // IMPORTANT:
        //
        // Use simple %0.5f formatting.
        //
        // XSim was printing the field-width formatting literally
        // with %10.5f / %+10.5f in the previous output.
        //---------------------------------------------------------
        $display(
            "%6d | %0.5f  %0.5f  %0.5f | %0.5f  %0.5f  %0.5f | %0.5f  %0.5f  %0.5f | %0.5f  %0.5f  %0.5f",

            sample_count,

            // X position
            Xest_ref,
            sfix_to_real10(Xest),
            Xest_ref - sfix_to_real10(Xest),

            // Y position
            Yest_ref,
            sfix_to_real10(Yest),
            Yest_ref - sfix_to_real10(Yest),

            // X velocity
            Uest_ref,
            sfix_to_real10(Uest),
            Uest_ref - sfix_to_real10(Uest),

            // Y velocity
            Vest_ref,
            sfix_to_real10(Vest),
            Vest_ref - sfix_to_real10(Vest)
        );


        sample_count = sample_count + 1;

    end


    //=============================================================
    // Stimulus
    //=============================================================
    initial begin

        // --------------------------------------------------------
        // VCD
        // --------------------------------------------------------
        $dumpfile("position_filter_tb.vcd");
        $dumpvars(0, tb_Position_Filter);


        // --------------------------------------------------------
        // Derive constants from the actual generated HDL
        // --------------------------------------------------------

        // The generated integrator multiplies by 131 and
        // subsequently shifts by 15 bits.
        Ts = 131.0 / 32768.0;

        // X:
        // k1_x = 9216 / 4096 = 2.25
        // k2_x = 12288 / 4096 = 3.0
        K1_X = 9216.0 / 4096.0;
        K2_X = 12288.0 / 4096.0;

        // Y:
        // k1_y = 576 / 256 = 2.25
        // k2_y = 12288 / 4096 = 3.0
        K1_Y = 576.0 / 256.0;
        K2_Y = 12288.0 / 4096.0;


        // --------------------------------------------------------
        // Initial conditions
        // --------------------------------------------------------
        clk = 1'b0;

        reset = 1'b1;

        clk_enable = 1'b0;

        X_raw = 16'sd0;
        Y_raw = 16'sd0;

        ax_global = 16'sd0;
        ay_global = 16'sd0;

        sample_count = 0;

        reset_ref_model();


        // --------------------------------------------------------
        // Reset
        // --------------------------------------------------------
        #20;

        reset = 1'b0;

        clk_enable = 1'b1;

        reset_ref_model();

        sample_count = 0;


        // ========================================================
        // TEST CASE 1
        // ZERO INPUTS
        // ========================================================
        $display("");
        $display("============================================================");
        $display(" TEST CASE 1: ZERO INPUTS");
        $display("============================================================");

        hold_for_samples(20);


        // --------------------------------------------------------
        // Change stimulus away from positive clock edge
        // --------------------------------------------------------
        @(negedge clk);


        // ========================================================
        // TEST CASE 2
        // POSITION STEP
        // ========================================================
        $display("");
        $display("============================================================");
        $display(" TEST CASE 2: POSITION STEP");
        $display(" X_raw = +1.0");
        $display(" Y_raw = -1.0");
        $display(" ax    =  0");
        $display(" ay    =  0");
        $display("============================================================");

        X_raw = real_to_sfix10(+1.0);
        Y_raw = real_to_sfix10(-1.0);

        ax_global = 16'sd0;
        ay_global = 16'sd0;

        hold_for_samples(1500);


        // --------------------------------------------------------
        // Next test starts away from active clock edge
        // --------------------------------------------------------
        @(negedge clk);


        // ========================================================
        // TEST CASE 3
        // CONSTANT ACCELERATION
        // ========================================================
        $display("");
        $display("============================================================");
        $display(" TEST CASE 3: CONSTANT ACCELERATION");
        $display(" X_raw =  0");
        $display(" Y_raw =  0");
        $display(" ax    = +0.1");
        $display(" ay    = +0.2");
        $display("============================================================");

        X_raw = 16'sd0;
        Y_raw = 16'sd0;

        ax_global = real_to_sfix10(+0.1);
        ay_global = real_to_sfix10(+0.2);

        hold_for_samples(20);


        // --------------------------------------------------------
        // Next test starts away from active clock edge
        // --------------------------------------------------------
        @(negedge clk);


        // ========================================================
        // TEST CASE 4
        // MIXED INPUTS
        // ========================================================
        $display("");
        $display("============================================================");
        $display(" TEST CASE 4: MIXED INPUTS");
        $display(" X_raw = +0.5");
        $display(" Y_raw = +0.5");
        $display(" ax    = -0.05");
        $display(" ay    = +0.1");
        $display("============================================================");

        X_raw = real_to_sfix10(+0.5);
        Y_raw = real_to_sfix10(+0.5);

        ax_global = real_to_sfix10(-0.05);
        ay_global = real_to_sfix10(+0.1);

        hold_for_samples(20);


        // --------------------------------------------------------
        // End simulation
        // --------------------------------------------------------
        @(negedge clk);

        $display("");
        $display("============================================================");
        $display(" SIMULATION COMPLETE");
        $display("============================================================");

        $finish;

    end

endmodule