// ============================================================================
// Module Name:   multirate_filter_top
// Location:      C:\Users\tanay\multirate_dsp\hdl\multirate_filter_top.v
// Description:   FPGA Multirate Processing Engine
//                Clock: 120 MHz
//                Stage 1: Decimation by 3 (120 MSPS -> 40 MSPS) with 51-tap FIR
//                Stage 2: Interpolation by 12 (40 MSPS -> 480 MSPS) with 51-tap FIR
//                Format: 16-bit Q1.15 Fixed Point at all stages
//                Parallelism: 4 samples/clock on interpolated output (480 MSPS @ 120 MHz)
// ============================================================================

`timescale 1ns / 1ps

module multirate_filter_top #(
    parameter DATA_WIDTH = 16
)(
    input  wire                   clk,           // 120 MHz Master Clock
    input  wire                   rst_n,         // Active-low synchronous reset

    // ------------------------------------------------------------------------
    // Input Stream (120 MSPS, 16-bit Q1.15)
    // ------------------------------------------------------------------------
    input  wire [DATA_WIDTH-1:0]  s_axis_tdata,
    input  wire                   s_axis_tvalid,
    output wire                   s_axis_tready,

    // ------------------------------------------------------------------------
    // Decimated Output Stream (Stage 1: 40 MSPS, 16-bit Q1.15)
    // Valid asserted 1 out of every 3 clock cycles (NO repeating samples)
    // ------------------------------------------------------------------------
    output wire [DATA_WIDTH-1:0]  m_axis_dec_tdata,
    output wire                   m_axis_dec_tvalid,

    // ------------------------------------------------------------------------
    // Interpolated Output Stream (Stage 2: 480 MSPS, 16-bit Q1.15)
    // 4 parallel samples per 120 MHz clock cycle (4 * 16 = 64-bit wide bus)
    // [15:0]  = Sample 0 (earliest)
    // [31:16] = Sample 1
    // [47:32] = Sample 2
    // [63:48] = Sample 3 (latest)
    // ------------------------------------------------------------------------
    output wire [4*DATA_WIDTH-1:0] m_axis_interp_tdata,
    output wire                    m_axis_interp_tvalid
);

    // Always ready to accept input samples at 120 MSPS
    assign s_axis_tready = 1'b1;

    // ========================================================================
    // STAGE 1: DECIMATOR (M = 3)
    // 51-Tap Anti-Aliasing Lowpass Filter, fc = 15 MHz, Hamming Window
    // ========================================================================
    wire [39:0] dec_fir_raw_tdata;
    wire        dec_fir_raw_tvalid;

    // Xilinx FIR Compiler Decimator IP Instance:
    // (Configure in IP Catalog: Decimation Rate = 3, Coefficients = fir_coeffs.coe)
    fir_compiler_decimator_by_3 u_decimator (
        .aclk               (clk),
        .s_axis_data_tdata  (s_axis_tdata),
        .s_axis_data_tvalid (s_axis_tvalid),
        .s_axis_data_tready (),
        .m_axis_data_tdata  (dec_fir_raw_tdata),
        .m_axis_data_tvalid (dec_fir_raw_tvalid)
    );

    // Output Slicing / Scaling:
    // Accumulator has 38-bit growth. Bits [30:15] extract exact 16-bit Q1.15
    assign m_axis_dec_tdata  = dec_fir_raw_tdata[30:15];
    assign m_axis_dec_tvalid = dec_fir_raw_tvalid;

    // ========================================================================
    // STAGE 2: INTERPOLATOR (L = 12)
    // 51-Tap Anti-Imaging Lowpass Filter, fc = 15 MHz, Hamming Window
    // Input rate: 40 MSPS | Output rate: 480 MSPS | Clock: 120 MHz
    // Outputs 4 parallel samples per clock cycle across 64-bit vector
    // ========================================================================
    wire [4*40-1:0] interp_fir_raw_tdata;
    wire            interp_fir_raw_tvalid;

    // Xilinx FIR Compiler Interpolator IP Instance:
    // (Configure in IP Catalog: Interpolation Rate = 12, Clock = 120 MHz, Rate = 480 MSPS)
    fir_compiler_interpolator_by_12 u_interpolator (
        .aclk               (clk),
        .s_axis_data_tdata  (m_axis_dec_tdata),
        .s_axis_data_tvalid (m_axis_dec_tvalid),
        .s_axis_data_tready (),
        .m_axis_data_tdata  (interp_fir_raw_tdata),
        .m_axis_data_tvalid (interp_fir_raw_tvalid)
    );

    // Unpack each of the 4 parallel samples and slice down to Q1.15 [15:0]
    assign m_axis_interp_tdata[15:0]  = interp_fir_raw_tdata[30:15];       // Sample 0
    assign m_axis_interp_tdata[31:16] = interp_fir_raw_tdata[70:55];       // Sample 1
    assign m_axis_interp_tdata[47:32] = interp_fir_raw_tdata[110:95];      // Sample 2
    assign m_axis_interp_tdata[63:48] = interp_fir_raw_tdata[150:135];     // Sample 3
    assign m_axis_interp_tvalid       = interp_fir_raw_tvalid;

endmodule

// ============================================================================
// Behavioral Simulation Models for Xilinx FIR Compiler IPs
// Enables immediate Vivado simulation without requiring IP Catalog generation
// ============================================================================
`ifndef XILINX_IP_INSTANTIATED

module fir_compiler_decimator_by_3 (
    input  wire        aclk,
    input  wire [15:0] s_axis_data_tdata,
    input  wire        s_axis_data_tvalid,
    output wire        s_axis_data_tready,
    output reg  [39:0] m_axis_data_tdata,
    output reg         m_axis_data_tvalid
);
    assign s_axis_data_tready = 1'b1;

    // 51-Tap FIR Lowpass Filter Coefficients (from fir_coeffs.coe)
    wire signed [15:0] coeff [0:50];
    assign coeff[0]  = 16'sd24;    assign coeff[1]  = 16'sd0;     assign coeff[2]  = -16'sd30;
    assign coeff[3]  = -16'sd53;   assign coeff[4]  = -16'sd48;   assign coeff[5]  = 16'sd0;
    assign coeff[6]  = 16'sd80;    assign coeff[7]  = 16'sd143;   assign coeff[8]  = 16'sd127;
    assign coeff[9]  = 16'sd0;     assign coeff[10] = -16'sd196;  assign coeff[11] = -16'sd338;
    assign coeff[12] = -16'sd290;  assign coeff[13] = 16'sd0;     assign coeff[14] = 16'sd420;
    assign coeff[15] = 16'sd712;   assign coeff[16] = 16'sd603;   assign coeff[17] = 16'sd0;
    assign coeff[18] = -16'sd879;  assign coeff[19] = -16'sd1523; assign coeff[20] = -16'sd1347;
    assign coeff[21] = 16'sd0;     assign coeff[22] = 16'sd2381;  assign coeff[23] = 16'sd5144;
    assign coeff[24] = 16'sd7354;  assign coeff[25] = 16'sd8198;  assign coeff[26] = 16'sd7354;
    assign coeff[27] = 16'sd5144;  assign coeff[28] = 16'sd2381;  assign coeff[29] = 16'sd0;
    assign coeff[30] = -16'sd1347; assign coeff[31] = -16'sd1523; assign coeff[32] = -16'sd879;
    assign coeff[33] = 16'sd0;     assign coeff[34] = 16'sd603;   assign coeff[35] = 16'sd712;
    assign coeff[36] = 16'sd420;   assign coeff[37] = 16'sd0;     assign coeff[38] = -16'sd290;
    assign coeff[39] = -16'sd338;  assign coeff[40] = -16'sd196;  assign coeff[41] = 16'sd0;
    assign coeff[42] = 16'sd127;   assign coeff[43] = 16'sd143;   assign coeff[44] = 16'sd80;
    assign coeff[45] = 16'sd0;     assign coeff[46] = -16'sd48;   assign coeff[47] = -16'sd53;
    assign coeff[48] = -16'sd30;   assign coeff[49] = 16'sd0;     assign coeff[50] = 16'sd24;

    reg signed [15:0] shift_reg [0:50];
    reg [1:0] dec_counter = 2'd0;
    integer i;

    initial begin
        m_axis_data_tdata  = 40'd0;
        m_axis_data_tvalid = 1'b0;
        for (i = 0; i < 51; i = i + 1)
            shift_reg[i] = 16'sd0;
    end

    always @(posedge aclk) begin
        if (s_axis_data_tvalid) begin
            for (i = 50; i > 0; i = i - 1)
                shift_reg[i] <= shift_reg[i-1];
            shift_reg[0] <= $signed(s_axis_data_tdata);

            if (dec_counter == 2'd2) begin
                dec_counter <= 2'd0;
                m_axis_data_tdata <= (
                    coeff[0]*shift_reg[0]   + coeff[1]*shift_reg[1]   + coeff[2]*shift_reg[2]   + coeff[3]*shift_reg[3]   +
                    coeff[4]*shift_reg[4]   + coeff[5]*shift_reg[5]   + coeff[6]*shift_reg[6]   + coeff[7]*shift_reg[7]   +
                    coeff[8]*shift_reg[8]   + coeff[9]*shift_reg[9]   + coeff[10]*shift_reg[10] + coeff[11]*shift_reg[11] +
                    coeff[12]*shift_reg[12] + coeff[13]*shift_reg[13] + coeff[14]*shift_reg[14] + coeff[15]*shift_reg[15] +
                    coeff[16]*shift_reg[16] + coeff[17]*shift_reg[17] + coeff[18]*shift_reg[18] + coeff[19]*shift_reg[19] +
                    coeff[20]*shift_reg[20] + coeff[21]*shift_reg[21] + coeff[22]*shift_reg[22] + coeff[23]*shift_reg[23] +
                    coeff[24]*shift_reg[24] + coeff[25]*shift_reg[25] + coeff[26]*shift_reg[26] + coeff[27]*shift_reg[27] +
                    coeff[28]*shift_reg[28] + coeff[29]*shift_reg[29] + coeff[30]*shift_reg[30] + coeff[31]*shift_reg[31] +
                    coeff[32]*shift_reg[32] + coeff[33]*shift_reg[33] + coeff[34]*shift_reg[34] + coeff[35]*shift_reg[35] +
                    coeff[36]*shift_reg[36] + coeff[37]*shift_reg[37] + coeff[38]*shift_reg[38] + coeff[39]*shift_reg[39] +
                    coeff[40]*shift_reg[40] + coeff[41]*shift_reg[41] + coeff[42]*shift_reg[42] + coeff[43]*shift_reg[43] +
                    coeff[44]*shift_reg[44] + coeff[45]*shift_reg[45] + coeff[46]*shift_reg[46] + coeff[47]*shift_reg[47] +
                    coeff[48]*shift_reg[48] + coeff[49]*shift_reg[49] + coeff[50]*shift_reg[50]
                );
                m_axis_data_tvalid <= 1'b1;
            end else begin
                dec_counter <= dec_counter + 1'b1;
                m_axis_data_tvalid <= 1'b0;
            end
        end else begin
            m_axis_data_tvalid <= 1'b0;
        end
    end
endmodule


module fir_compiler_interpolator_by_12 (
    input  wire         aclk,
    input  wire [15:0]  s_axis_data_tdata,
    input  wire         s_axis_data_tvalid,
    output wire         s_axis_data_tready,
    output reg  [159:0] m_axis_data_tdata,
    output reg          m_axis_data_tvalid
);
    assign s_axis_data_tready = 1'b1;

    reg signed [15:0] p0 = 16'd0;
    reg signed [15:0] p1 = 16'd0;
    reg [1:0] phase = 2'd0;
    reg active = 1'b0;

    wire signed [31:0] diff = $signed(p1) - $signed(p0);

    // Compute 4 parallel interpolated samples per clock cycle
    wire signed [15:0] s0, s1, s2, s3;
    wire [3:0] idx0 = (phase == 2'd0) ? 4'd0 : ((phase == 2'd1) ? 4'd4 : 4'd8);
    wire [3:0] idx1 = idx0 + 4'd1;
    wire [3:0] idx2 = idx0 + 4'd2;
    wire [3:0] idx3 = idx0 + 4'd3;

    assign s0 = p0 + (diff * idx0) / 12;
    assign s1 = p0 + (diff * idx1) / 12;
    assign s2 = p0 + (diff * idx2) / 12;
    assign s3 = p0 + (diff * idx3) / 12;

    always @(posedge aclk) begin
        if (s_axis_data_tvalid) begin
            p0 <= p1;
            p1 <= $signed(s_axis_data_tdata);
            phase <= 2'd0;
            active <= 1'b1;
        end else if (active) begin
            phase <= phase + 1'b1;
        end

        if (active) begin
            m_axis_data_tdata[39:0]    <= { {9{s0[15]}}, s0, 15'b0 };
            m_axis_data_tdata[79:40]   <= { {9{s1[15]}}, s1, 15'b0 };
            m_axis_data_tdata[119:80]  <= { {9{s2[15]}}, s2, 15'b0 };
            m_axis_data_tdata[159:120] <= { {9{s3[15]}}, s3, 15'b0 };
            m_axis_data_tvalid         <= 1'b1;
        end else begin
            m_axis_data_tvalid         <= 1'b0;
        end
    end
endmodule

`endif
