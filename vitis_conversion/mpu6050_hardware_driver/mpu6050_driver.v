`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: top_mpu6050_driver (in mpu6050_driver.v)
// Description: Standalone subsystem integrating:
//              1. Vitis HLS MPU-6050 Core (top_mpu6050_iic_0)
//              2. Command & Response FWFT Streaming FIFOs (u_cmd_fifo, u_resp_fifo)
//              3. Pure Verilog FIFO-to-I2C Manager (fifo_i2c_manager)
//              4. Digilent VHDL I2C Master (i2c_master.vhd)
//
// Ports:
//   - clk, rst_n
//   - accel_x/y/z, gyro_x/y/z, temp_raw
//   - scl, sda (inout)
//////////////////////////////////////////////////////////////////////////////////

// Parameterized First-Word Fall-Through (FWFT) Synchronous FIFO
module mpu_fwft_fifo #(
    parameter DATA_WIDTH = 32,
    parameter DEPTH      = 16
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire [DATA_WIDTH-1:0] din,
    input  wire                  wr_en,
    output wire                  full,
    output wire [DATA_WIDTH-1:0] dout,
    input  wire                  rd_en,
    output wire                  empty
);
    localparam ADDR_WIDTH = $clog2(DEPTH);

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_WIDTH-1:0] wr_ptr;
    reg [ADDR_WIDTH-1:0] rd_ptr;
    reg [ADDR_WIDTH:0]   count;

    assign full  = (count == DEPTH);
    assign empty = (count == 0);
    assign dout  = mem[rd_ptr];

    always @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr <= {ADDR_WIDTH{1'b0}};
            rd_ptr <= {ADDR_WIDTH{1'b0}};
            count  <= {(ADDR_WIDTH+1){1'b0}};
        end else begin
            case ({wr_en && !full, rd_en && !empty})
                2'b10: begin // Write only
                    mem[wr_ptr] <= din;
                    wr_ptr      <= wr_ptr + 1'b1;
                    count       <= count + 1'b1;
                end
                2'b01: begin // Read only
                    rd_ptr      <= rd_ptr + 1'b1;
                    count       <= count - 1'b1;
                end
                2'b11: begin // Simultaneous Write and Read
                    mem[wr_ptr] <= din;
                    wr_ptr      <= wr_ptr + 1'b1;
                    rd_ptr      <= rd_ptr + 1'b1;
                end
                default: ;
            endcase
        end
    end
endmodule

module top_mpu6050_driver #(
    parameter integer CLK_FREQ_HZ      = 100_000_000,
    parameter integer I2C_BUS_HZ       = 100_000,
    parameter integer DEFAULT_PIN_SWAP = 1 // 1: P16=SDA, P15=SCL (matches physical board wiring); 0: standard
)(
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK",
       X_INTERFACE_PARAMETER = "ASSOCIATED_RESET rst_n" *)
    input  wire        clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 rst_n RST",
       X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        rst_n,

    // Calibrated sensor outputs from HLS core
    output wire [15:0] accel_x,
    output wire [15:0] accel_y,
    output wire [15:0] accel_z,
    output wire [15:0] gyro_x,
    output wire [15:0] gyro_y,
    output wire [15:0] gyro_z,
    output wire [15:0] temp_raw,

    // I2C physical bus
    inout  wire        scl,
    inout  wire        sda
);

    // Static control lines for clean standalone operation
    wire       vio_rst_override = 1'b0;
    wire       vio_addr_sel     = 1'b0;
    wire [6:0] vio_addr_val     = 7'h68;
    wire       vio_pin_swap     = 1'b0;

    // Effective reset combining external rst_n and VIO manual reset
    wire rst_n_eff = rst_n & ~vio_rst_override;
    wire hls_rst   = ~rst_n_eff;
    wire [9:0] clk_freq_mhz = CLK_FREQ_HZ / 1_000_000;

    // --- Signals between HLS Core and Command FIFO ---
    wire [31:0] hls_cmd_din;
    wire        hls_cmd_wr;
    wire        cmd_fifo_full;

    // --- Signals between Command FIFO and Manager ---
    wire [31:0] mgr_cmd_dout;
    wire        mgr_cmd_rd_en;
    wire        cmd_fifo_empty;

    // --- Signals between Manager and Response FIFO ---
    wire [15:0] mgr_resp_din;
    wire        mgr_resp_wr_en;
    wire        resp_fifo_full;

    // --- Signals between Response FIFO and HLS Core ---
    wire [15:0] hls_resp_dout;
    wire        hls_resp_rd;
    wire        resp_fifo_empty;

    // --- Signals between Manager and Digilent i2c_master ---
    wire        i2c_ena;
    wire [6:0]  mgr_i2c_addr;
    wire [6:0]  i2c_addr = vio_addr_sel ? vio_addr_val : mgr_i2c_addr;
    wire        i2c_rw;
    wire [7:0]  i2c_data_wr;
    wire        i2c_busy;
    wire [7:0]  i2c_data_rd;
    wire        i2c_ack_error;

    // -------------------------------------------------------------------------
    // 1. Instantiate Vitis HLS MPU-6050 Core
    // -------------------------------------------------------------------------
    top_mpu6050_iic_0 u_hls_mpu6050 (
        .ap_clk              (clk),
        .ap_rst              (hls_rst),
        .clk_freq_mhz        (clk_freq_mhz),
        .accel_x             (accel_x),
        .accel_y             (accel_y),
        .accel_z             (accel_z),
        .gyro_x              (gyro_x),
        .gyro_y              (gyro_y),
        .gyro_z              (gyro_z),
        .temp_raw            (temp_raw),
        .cmd_fifo_1_din      (hls_cmd_din),
        .cmd_fifo_1_full_n   (~cmd_fifo_full),
        .cmd_fifo_1_write    (hls_cmd_wr),
        .resp_fifo_1_dout    (hls_resp_dout),
        .resp_fifo_1_empty_n (~resp_fifo_empty),
        .resp_fifo_1_read    (hls_resp_rd)
    );

    // -------------------------------------------------------------------------
    // 2. Command FIFO (32-bit FWFT)
    // -------------------------------------------------------------------------
    mpu_fwft_fifo #(
        .DATA_WIDTH (32),
        .DEPTH      (16)
    ) u_cmd_fifo (
        .clk   (clk),
        .rst_n (rst_n_eff),
        .din   (hls_cmd_din),
        .wr_en (hls_cmd_wr),
        .full  (cmd_fifo_full),
        .dout  (mgr_cmd_dout),
        .rd_en (mgr_cmd_rd_en),
        .empty (cmd_fifo_empty)
    );

    // -------------------------------------------------------------------------
    // 3. Response FIFO (16-bit FWFT)
    // -------------------------------------------------------------------------
    mpu_fwft_fifo #(
        .DATA_WIDTH (16),
        .DEPTH      (16)
    ) u_resp_fifo (
        .clk   (clk),
        .rst_n (rst_n_eff),
        .din   (mgr_resp_din),
        .wr_en (mgr_resp_wr_en),
        .full  (resp_fifo_full),
        .dout  (hls_resp_dout),
        .rd_en (hls_resp_rd),
        .empty (resp_fifo_empty)
    );

    // -------------------------------------------------------------------------
    // 4. Instantiate FIFO-to-I2C Manager
    // -------------------------------------------------------------------------
    fifo_i2c_manager #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ)
    ) u_fifo_i2c_mgr (
        .clk             (clk),
        .rst_n           (rst_n_eff),
        .cmd_fifo_dout   (mgr_cmd_dout),
        .cmd_fifo_empty  (cmd_fifo_empty),
        .cmd_fifo_rd_en  (mgr_cmd_rd_en),
        .resp_fifo_din   (mgr_resp_din),
        .resp_fifo_full  (resp_fifo_full),
        .resp_fifo_wr_en (mgr_resp_wr_en),
        .i2c_ena         (i2c_ena),
        .i2c_addr        (mgr_i2c_addr),
        .i2c_rw          (i2c_rw),
        .i2c_data_wr     (i2c_data_wr),
        .i2c_busy        (i2c_busy),
        .i2c_data_rd     (i2c_data_rd),
        .i2c_ack_error   (i2c_ack_error)
    );

    // -------------------------------------------------------------------------
    // 5. Instantiate Digilent I2C Master (VHDL)
    // -------------------------------------------------------------------------
    
    wire scl_master_i, scl_master_o, scl_master_t;
    wire sda_master_i, sda_master_o, sda_master_t;

    wire pin_scl_i, pin_scl_o, pin_scl_t;
    wire pin_sda_i, pin_sda_o, pin_sda_t;

    // Dynamic Pin Swap multiplexer:
    // Controlled by DEFAULT_PIN_SWAP (default on boot) and VIO probe_out3 (runtime override):
    // If eff_pin_swap == 0: P16 = SCL, P15 = SDA (Standard PYNQ-Z1 mapping)
    // If eff_pin_swap == 1: P16 = SDA, P15 = SCL (Matches physical board wiring)
    wire eff_pin_swap = vio_pin_swap ^ (DEFAULT_PIN_SWAP != 0);

    assign pin_scl_o = eff_pin_swap ? sda_master_o : scl_master_o;
    assign pin_scl_t = eff_pin_swap ? sda_master_t : scl_master_t;
    assign pin_sda_o = eff_pin_swap ? scl_master_o : sda_master_o;
    assign pin_sda_t = eff_pin_swap ? scl_master_t : sda_master_t;

    assign scl_master_i = eff_pin_swap ? pin_sda_i : pin_scl_i;
    assign sda_master_i = eff_pin_swap ? pin_scl_i : pin_sda_i;

    IOBUF u_scl_iobuf (
        .I  (pin_scl_o),
        .O  (pin_scl_i),
        .T  (pin_scl_t),
        .IO (scl)
    );

    IOBUF u_sda_iobuf (
        .I  (pin_sda_o),
        .O  (pin_sda_i),
        .T  (pin_sda_t),
        .IO (sda)
    );

    i2c_master #(
        .input_clk (CLK_FREQ_HZ),
        .bus_clk   (I2C_BUS_HZ)
    ) u_i2c_master (
        .clk       (clk),
        .reset_n   (rst_n_eff),
        .ena       (i2c_ena),
        .addr      (i2c_addr),
        .rw        (i2c_rw),
        .data_wr   (i2c_data_wr),
        .busy      (i2c_busy),
        .data_rd   (i2c_data_rd),
        .ack_error (i2c_ack_error),
        .sda_i     (sda_master_i),
        .sda_o     (sda_master_o),
        .sda_t     (sda_master_t),
        .scl_i     (scl_master_i),
        .scl_o     (scl_master_o),
        .scl_t     (scl_master_t)
    );

    // (Hardware debug cores removed: clean, self-contained standalone RTL)
endmodule

