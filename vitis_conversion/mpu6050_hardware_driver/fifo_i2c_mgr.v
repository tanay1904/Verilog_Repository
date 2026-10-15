`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: fifo_i2c_manager (in fifo_i2c_mgr.v)
// Description: Bridges HLS streaming FIFOs (cmd_fifo, resp_fifo) to Digilent's
//              i2c_master controller (i2c_master.vhd) using the standard
//              busy_cnt edge-tracking handshake.
//
// FIFO Formats:
//   Command FIFO (32-bit):
//     [0]     : is_read (0 = Write, 1 = Read)
//     [14:8]  : dev_addr (7-bit I2C device address, e.g., 7'h68)
//     [23:16] : reg_addr (8-bit sensor register address)
//     [31:24] : wr_data  (8-bit data byte to write)
//
//   Response FIFO (16-bit):
//     [7:0]   : status (8'h00 = ACK / Success, 8'hFF = NACK / Error)
//     [15:8]  : data   (8-bit data byte read from sensor)
//////////////////////////////////////////////////////////////////////////////////

module fifo_i2c_manager #(
    parameter integer CLK_FREQ_HZ = 100_000_000 // System clock frequency in Hz (e.g., 100 MHz)
)(
    input  wire        clk,
    input  wire        rst_n,

    // --- Interface to Command FIFO (Pop) ---
    input  wire [31:0] cmd_fifo_dout,
    input  wire        cmd_fifo_empty,
    output reg         cmd_fifo_rd_en,

    // --- Interface to Response FIFO (Push) ---
    output reg  [15:0] resp_fifo_din,
    input  wire        resp_fifo_full,
    output reg         resp_fifo_wr_en,

    // --- Interface to Digilent i2c_master (i2c_master.vhd) ---
    output reg         i2c_ena,
    output reg  [6:0]  i2c_addr,
    output reg         i2c_rw,
    output reg  [7:0]  i2c_data_wr,
    input  wire        i2c_busy,
    input  wire [7:0]  i2c_data_rd,
    input  wire        i2c_ack_error
);

    // Unpack command fields from 32-bit FIFO word
    wire       is_read  = cmd_fifo_dout[0];
    wire [6:0] dev_addr = cmd_fifo_dout[14:8];
    wire [7:0] reg_addr = cmd_fifo_dout[23:16];
    wire [7:0] wr_data  = cmd_fifo_dout[31:24];

    // Internal latches
    reg       latched_is_read;
    reg [6:0] latched_dev_addr;
    reg [7:0] latched_reg_addr;
    reg [7:0] latched_wr_data;
    reg [7:0] latched_rd_data;
    reg [7:0] latched_status;

    // Edge detector & counter for i2c_busy
    reg       busy_prev;
    reg [2:0] busy_cnt;

    // Watchdog Timeout (10ms): Prevents hardware lockup if I2C bus hangs
    localparam integer TIMEOUT_CYCLES = (CLK_FREQ_HZ / 100);
    reg [31:0] timeout_cnt;

    // FSM States
    localparam MGR_IDLE      = 2'd0;
    localparam MGR_FETCH     = 2'd1;
    localparam MGR_EXEC      = 2'd2;
    localparam MGR_PUSH_RESP = 2'd3;

    reg [1:0] state;

    always @(posedge clk) begin
        if (!rst_n) begin
            state            <= MGR_IDLE;
            cmd_fifo_rd_en   <= 1'b0;
            resp_fifo_wr_en  <= 1'b0;
            resp_fifo_din    <= 16'h0000;
            i2c_ena          <= 1'b0;
            i2c_addr         <= 7'd0;
            i2c_rw           <= 1'b0;
            i2c_data_wr      <= 8'd0;
            latched_is_read  <= 1'b0;
            latched_dev_addr <= 7'd0;
            latched_reg_addr <= 8'd0;
            latched_wr_data  <= 8'd0;
            latched_rd_data  <= 8'd0;
            latched_status   <= 8'h00;
            busy_prev        <= 1'b0;
            busy_cnt         <= 3'd0;
            timeout_cnt      <= 32'd0;
        end else begin
            cmd_fifo_rd_en  <= 1'b0;
            resp_fifo_wr_en <= 1'b0;
            busy_prev       <= i2c_busy;

            case (state)
                // 1. Idle: Wait for incoming command and response FIFO space
                MGR_IDLE: begin
                    i2c_ena  <= 1'b0;
                    busy_cnt <= 3'd0;
                    if (!cmd_fifo_empty && !resp_fifo_full && !i2c_busy) begin
                        state <= MGR_FETCH;
                    end
                end

                // 2. Fetch: Latch command attributes and pop from FWFT FIFO
                MGR_FETCH: begin
                    cmd_fifo_rd_en   <= 1'b1;
                    latched_is_read  <= is_read;
                    latched_dev_addr <= dev_addr;
                    latched_reg_addr <= reg_addr;
                    latched_wr_data  <= wr_data;
                    latched_status   <= 8'h00;
                    latched_rd_data  <= 8'h00;
                    busy_cnt         <= 3'd0;
                    timeout_cnt      <= 32'd0;

                    // Immediately present command 0: Dev Addr + Reg Addr (Write)
                    i2c_addr    <= dev_addr;
                    i2c_rw      <= 1'b0; // Write mode
                    i2c_data_wr <= reg_addr;
                    i2c_ena     <= 1'b1;
                    state       <= MGR_EXEC;
                end

                // 3. Execute I2C Transaction using standard busy_cnt sequence
                MGR_EXEC: begin
                    timeout_cnt <= timeout_cnt + 1'b1;

                    // Count rising edges of i2c_busy
                    if (!busy_prev && i2c_busy) begin
                        busy_cnt <= busy_cnt + 1'b1;
                    end

                    // Check for Watchdog timeout or NACK error
                    if (timeout_cnt >= TIMEOUT_CYCLES) begin
                        latched_status <= 8'hFF; // Timeout Error
                        i2c_ena        <= 1'b0;  // Abort transaction
                        state          <= MGR_PUSH_RESP;
                    end else if (i2c_ack_error && busy_cnt > 0) begin
                        latched_status <= 8'hFF;
                        i2c_ena        <= 1'b0; // Terminate transaction early on NACK
                        if (!i2c_busy) begin
                            state <= MGR_PUSH_RESP;
                        end
                    end else begin
                        case (busy_cnt)
                            3'd0: begin
                                // Command 0 is active: wait for master to latch it (busy goes high)
                                i2c_addr    <= latched_dev_addr;
                                i2c_rw      <= 1'b0;
                                i2c_data_wr <= latched_reg_addr;
                                i2c_ena     <= 1'b1;
                            end

                            3'd1: begin
                                // Command 0 latched and executing. Prepare Command 1:
                                if (latched_is_read) begin
                                    // Read transaction: Repeated Start with Read
                                    i2c_addr <= latched_dev_addr;
                                    i2c_rw   <= 1'b1; // Read
                                    i2c_ena  <= 1'b1;
                                end else begin
                                    // Write transaction: Data byte
                                    i2c_data_wr <= latched_wr_data;
                                    i2c_ena     <= 1'b1;
                                end
                            end

                            3'd2: begin
                                // Command 1 latched and executing.
                                // Deassert ena so master will send STOP after this byte
                                i2c_ena <= 1'b0;

                                // When busy falls back to 0, transaction is complete
                                if (busy_prev && !i2c_busy) begin
                                    if (i2c_ack_error) begin
                                        latched_status <= 8'hFF;
                                    end else begin
                                        latched_status  <= 8'h00;
                                        latched_rd_data <= i2c_data_rd;
                                    end
                                    state <= MGR_PUSH_RESP;
                                end
                            end

                            default: begin
                                i2c_ena <= 1'b0;
                                if (!i2c_busy) begin
                                    state <= MGR_PUSH_RESP;
                                end
                            end
                        endcase
                    end
                end

                // 4. Push response to HLS resp_fifo
                MGR_PUSH_RESP: begin
                    if (!resp_fifo_full) begin
                        // Packet layout: [7:0] = status, [15:8] = data
                        resp_fifo_din   <= {latched_rd_data, latched_status};
                        resp_fifo_wr_en <= 1'b1;
                        state           <= MGR_IDLE;
                    end
                end

                default: begin
                    state <= MGR_IDLE;
                end
            endcase
        end
    end

endmodule
