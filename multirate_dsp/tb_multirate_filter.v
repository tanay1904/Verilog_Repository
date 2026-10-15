// ============================================================================
// Testbench:     tb_multirate_filter
// Location:      C:\Users\tanay\multirate_dsp\hdl\tb_multirate_filter.v
// Description:   Comprehensive Verification Testbench for Multirate DSP Engine
// Features:
//   1. Generates 120 MHz master clock (period = 8.333 ns).
//   2. Reads 1 MHz Q1.15 stimulus from data/input_signal.txt.
//   3. Captures Decimated output on tvalid (ensuring ZERO repeated samples).
//   4. Captures Interpolated output, serializing the 4 parallel samples/clock
//      in exact chronological order.
// ============================================================================

`timescale 1ns / 1ps

module tb_multirate_filter;

    // ------------------------------------------------------------------------
    // Clock & Reset Definitions (120 MHz -> T = 8.333 ns)
    // ------------------------------------------------------------------------
    reg clk;
    reg rst_n;

    initial begin
        clk = 1'b0;
        forever #4.167 clk = ~clk; // 120 MHz clock period = 8.334 ns
    end

    // ------------------------------------------------------------------------
    // Device Under Test (DUT) Interface Signals
    // ------------------------------------------------------------------------
    reg  [15:0] s_axis_tdata;
    reg         s_axis_tvalid;
    wire        s_axis_tready;

    wire [15:0] m_axis_dec_tdata;
    wire        m_axis_dec_tvalid;

    wire [63:0] m_axis_interp_tdata;
    wire        m_axis_interp_tvalid;

    // Instantiate Top-Level Module
    multirate_filter_top #(
        .DATA_WIDTH(16)
    ) dut (
        .clk                  (clk),
        .rst_n                (rst_n),
        .s_axis_tdata         (s_axis_tdata),
        .s_axis_tvalid        (s_axis_tvalid),
        .s_axis_tready        (s_axis_tready),
        .m_axis_dec_tdata     (m_axis_dec_tdata),
        .m_axis_dec_tvalid    (m_axis_dec_tvalid),
        .m_axis_interp_tdata  (m_axis_interp_tdata),
        .m_axis_interp_tvalid (m_axis_interp_tvalid)
    );

    // ------------------------------------------------------------------------
    // File I/O Handlers
    // ------------------------------------------------------------------------
    integer file_in;
    integer file_dec_out;
    integer file_interp_out;
    integer scan_status;
    integer in_val;
    integer sample_count_in;
    integer sample_count_dec;
    integer sample_count_interp;

    // ------------------------------------------------------------------------
    // Process 1: Stimulus Driver (Feeds input_signal.txt at 120 MSPS)
    // ------------------------------------------------------------------------
    initial begin
        s_axis_tdata     = 16'd0;
        s_axis_tvalid    = 1'b0;
        rst_n            = 1'b0;
        sample_count_in  = 0;

        file_in = $fopen("C:/Users/tanay/multirate_dsp/data/input_signal.txt", "r");
        if (file_in == 0) file_in = $fopen("../data/input_signal.txt", "r");
        if (file_in == 0) file_in = $fopen("input_signal.txt", "r");
        
        if (file_in == 0) begin
            $display("[ERROR] Could not open input_signal.txt!");
            $finish;
        end

        // Apply Reset
        #50;
        @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);

        $display("[TB] Starting input playback @ 120 MSPS...");

        while (!$feof(file_in)) begin
            scan_status = $fscanf(file_in, "%d\n", in_val);
            if (scan_status == 1) begin
                @(posedge clk);
                s_axis_tdata    <= in_val[15:0];
                s_axis_tvalid   <= 1'b1;
                sample_count_in  = sample_count_in + 1;
            end
        end

        @(posedge clk);
        s_axis_tvalid <= 1'b0;
        $fclose(file_in);
        $display("[TB] Finished reading input_signal.txt (%0d samples injected).", sample_count_in);

        // Allow pipeline drain time (10 us)
        #10000;
        $fclose(file_dec_out);
        $fclose(file_interp_out);
        $display("[TB] Verification complete.");
        $display("[TB] Decimated samples logged:   %0d (Expected ~%0d)", sample_count_dec, sample_count_in / 3);
        $display("[TB] Interpolated samples logged: %0d (Expected ~%0d)", sample_count_interp, (sample_count_in / 3) * 12);
        $finish;
    end

    // ------------------------------------------------------------------------
    // Process 2: Decimated Logger (40 MSPS)
    // Strictly logs on m_axis_dec_tvalid to ENSURE NO REPEATED SAMPLES!
    // ------------------------------------------------------------------------
    initial begin
        sample_count_dec = 0;
        file_dec_out = $fopen("C:/Users/tanay/multirate_dsp/data/hdl_output/decimated_signal.txt", "w");
        if (file_dec_out == 0) file_dec_out = $fopen("../data/decimated_signal.txt", "w");
        if (file_dec_out == 0) file_dec_out = $fopen("decimated_signal.txt", "w");
    end

    always @(posedge clk) begin
        if (rst_n && m_axis_dec_tvalid) begin
            $fwrite(file_dec_out, "%d\n", $signed(m_axis_dec_tdata));
            sample_count_dec = sample_count_dec + 1;
        end
    end

    // ------------------------------------------------------------------------
    // Process 3: Interpolated Logger (480 MSPS)
    // Serializes the 4 parallel samples per clock in chronological order:
    // [15:0] -> [31:16] -> [47:32] -> [63:48]
    // ------------------------------------------------------------------------
    initial begin
        sample_count_interp = 0;
        file_interp_out = $fopen("C:/Users/tanay/multirate_dsp/data/hdl_output/interpolated_signal.txt", "w");
        if (file_interp_out == 0) file_interp_out = $fopen("../data/interpolated_signal.txt", "w");
        if (file_interp_out == 0) file_interp_out = $fopen("interpolated_signal.txt", "w");
    end

    always @(posedge clk) begin
        if (rst_n && m_axis_interp_tvalid) begin
            $fwrite(file_interp_out, "%d\n", $signed(m_axis_interp_tdata[15:0]));
            $fwrite(file_interp_out, "%d\n", $signed(m_axis_interp_tdata[31:16]));
            $fwrite(file_interp_out, "%d\n", $signed(m_axis_interp_tdata[47:32]));
            $fwrite(file_interp_out, "%d\n", $signed(m_axis_interp_tdata[63:48]));
            sample_count_interp = sample_count_interp + 4;
        end
    end

endmodule
