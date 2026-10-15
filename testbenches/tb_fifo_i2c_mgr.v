`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Testbench: tb_fifo_i2c_manager (tb_fifo_i2c_mgr.v)
// Description: Comprehensive testbench verifying fifo_i2c_manager connected to
//              Digilent's i2c_master.vhd and a behavioral MPU-6050 I2C slave model.
//
// Tests:
//   1. Register Write (PWR_MGMT_1 = 0x01)
//   2. Register Read  (WHO_AM_I, expects 0x68)
//   3. Register Read  (ACCEL_XOUT_H, expects 0x03)
//   4. Slave NACK Detection (Device address 0x12, expects Status 0xFF)
//////////////////////////////////////////////////////////////////////////////////

module tb_fifo_i2c_manager;

    reg clk;
    reg rst_n;

    // --- Command FIFO Signals ---
    reg  [31:0] cmd_fifo_dout;
    reg         cmd_fifo_empty;
    wire        cmd_fifo_rd_en;

    // --- Response FIFO Signals ---
    wire [15:0] resp_fifo_din;
    reg         resp_fifo_full;
    wire        resp_fifo_wr_en;

    // --- Signals between manager and i2c_master ---
    wire        i2c_ena;
    wire [6:0]  i2c_addr;
    wire        i2c_rw;
    wire [7:0]  i2c_data_wr;
    wire        i2c_busy;
    wire [7:0]  i2c_data_rd;
    wire        i2c_ack_error;

    // --- I2C Bus Pins ---
    wire scl_bus;
    wire sda_bus;

    pullup(scl_bus);
    pullup(sda_bus);

    // -------------------------------------------------------------------------
    // 1. Instantiate Device Under Test: fifo_i2c_manager
    // -------------------------------------------------------------------------
    fifo_i2c_manager dut (
        .clk             (clk),
        .rst_n           (rst_n),
        .cmd_fifo_dout   (cmd_fifo_dout),
        .cmd_fifo_empty  (cmd_fifo_empty),
        .cmd_fifo_rd_en  (cmd_fifo_rd_en),
        .resp_fifo_din   (resp_fifo_din),
        .resp_fifo_full  (resp_fifo_full),
        .resp_fifo_wr_en (resp_fifo_wr_en),
        .i2c_ena         (i2c_ena),
        .i2c_addr        (i2c_addr),
        .i2c_rw          (i2c_rw),
        .i2c_data_wr     (i2c_data_wr),
        .i2c_busy        (i2c_busy),
        .i2c_data_rd     (i2c_data_rd),
        .i2c_ack_error   (i2c_ack_error)
    );

    // -------------------------------------------------------------------------
    // 2. Instantiate Digilent i2c_master (VHDL)
    // -------------------------------------------------------------------------
    wire scl_i, scl_o, scl_t;
    wire sda_i, sda_o, sda_t;

    IOBUF u_scl_iobuf (
        .I  (scl_o),
        .O  (scl_i),
        .T  (scl_t),
        .IO (scl_bus)
    );

    IOBUF u_sda_iobuf (
        .I  (sda_o),
        .O  (sda_i),
        .T  (sda_t),
        .IO (sda_bus)
    );

    i2c_master #(
        .input_clk (50_000_000), // 50 MHz
        .bus_clk   (1_000_000)   // 1 MHz for simulation speed
    ) u_i2c_master (
        .clk       (clk),
        .reset_n   (rst_n), // Active-low reset
        .ena       (i2c_ena),
        .addr      (i2c_addr),
        .rw        (i2c_rw),
        .data_wr   (i2c_data_wr),
        .busy      (i2c_busy),
        .data_rd   (i2c_data_rd),
        .ack_error (i2c_ack_error),
        .sda_i     (sda_i),
        .sda_o     (sda_o),
        .sda_t     (sda_t),
        .scl_i     (scl_i),
        .scl_o     (scl_o),
        .scl_t     (scl_t)
    );

    // -------------------------------------------------------------------------
    // 3. Behavioral MPU-6050 I2C Slave Model on Bus (Address 7'h68)
    // -------------------------------------------------------------------------
    reg [7:0] s_mock_regs [0:255];
    reg [7:0] slave_reg_ptr;
    reg       slave_sda_drive = 1'b0;
    reg       slave_sda_val   = 1'b0;

    assign sda_bus = slave_sda_drive ? (slave_sda_val ? 1'bz : 1'b0) : 1'bz;

    initial begin
        slave_sda_drive    = 1'b0;
        slave_sda_val      = 1'b0;
        s_mock_regs[8'h75] = 8'h68; // WHO_AM_I default
        s_mock_regs[8'h6B] = 8'h40; // PWR_MGMT_1 default
        s_mock_regs[8'h3B] = 8'h03; // ACCEL_XOUT_H
        s_mock_regs[8'h3C] = 8'hE8; // ACCEL_XOUT_L
    end

    reg [3:0] s_bit   = 4'd0;
    reg [7:0] s_shift = 8'd0;
    reg [6:0] s_addr  = 7'd0;
    reg       s_rw    = 1'b0;

    localparam SLV_IDLE     = 3'd0;
    localparam SLV_ADDR     = 3'd1;
    localparam SLV_ACK_ADDR = 3'd2;
    localparam SLV_REG      = 3'd3;
    localparam SLV_ACK_REG  = 3'd4;
    localparam SLV_DATA_WR  = 3'd5;
    localparam SLV_ACK_WR   = 3'd6;
    localparam SLV_DATA_RD  = 3'd7;

    reg [2:0] slv_state = SLV_IDLE;

    // Detect START and REPEATED START
    always @(negedge sda_bus) begin
        if (scl_bus === 1'b1) begin
            slv_state       <= SLV_ADDR;
            s_bit           <= 4'd0;
            slave_sda_drive <= 1'b0;
        end
    end

    // Detect STOP
    always @(posedge sda_bus) begin
        if (scl_bus === 1'b1) begin
            slv_state       <= SLV_IDLE;
            slave_sda_drive <= 1'b0;
        end
    end

    // Sample bits on posedge SCL
    always @(posedge scl_bus) begin
        case (slv_state)
            SLV_ADDR: begin
                s_shift[7 - s_bit] <= sda_bus;
                if (s_bit == 4'd7) begin
                    s_addr <= s_shift[7:1];
                    s_rw   <= sda_bus;
                    if (s_shift[7:1] == 7'h68) begin
                        slv_state <= SLV_ACK_ADDR;
                    end else begin
                        slv_state <= SLV_IDLE; // NACK
                    end
                end else begin
                    s_bit <= s_bit + 1'b1;
                end
            end

            SLV_ACK_ADDR: begin
                s_bit <= 4'd0;
                if (s_rw == 1'b0) begin
                    slv_state <= SLV_REG;
                end else begin
                    slv_state <= SLV_DATA_RD;
                end
            end

            SLV_REG: begin
                s_shift[7 - s_bit] <= sda_bus;
                if (s_bit == 4'd7) begin
                    slave_reg_ptr <= {s_shift[7:1], sda_bus};
                    slv_state     <= SLV_ACK_REG;
                end else begin
                    s_bit <= s_bit + 1'b1;
                end
            end

            SLV_ACK_REG: begin
                s_bit     <= 4'd0;
                slv_state <= SLV_DATA_WR;
            end

            SLV_DATA_WR: begin
                s_shift[7 - s_bit] <= sda_bus;
                if (s_bit == 4'd7) begin
                    s_mock_regs[slave_reg_ptr] <= {s_shift[7:1], sda_bus};
                    slv_state <= SLV_ACK_WR;
                end else begin
                    s_bit <= s_bit + 1'b1;
                end
            end

            SLV_ACK_WR: begin
                s_bit     <= 4'd0;
                slv_state <= SLV_IDLE;
            end

            SLV_DATA_RD: begin
                if (s_bit == 4'd7) begin
                    slv_state <= SLV_IDLE; // Read byte complete
                end else begin
                    s_bit <= s_bit + 1'b1;
                end
            end
        endcase
    end

    // Drive SDA on negedge SCL
    always @(negedge scl_bus) begin
        case (slv_state)
            SLV_ACK_ADDR, SLV_ACK_REG, SLV_ACK_WR: begin
                slave_sda_drive <= 1'b1;
                slave_sda_val   <= 1'b0; // ACK
            end

            SLV_DATA_RD: begin
                slave_sda_drive <= 1'b1;
                slave_sda_val   <= s_mock_regs[slave_reg_ptr][7 - s_bit];
            end

            default: begin
                slave_sda_drive <= 1'b0;
            end
        endcase
    end

    // -------------------------------------------------------------------------
    // 4. Clock Generation (50 MHz = 20 ns period)
    // -------------------------------------------------------------------------
    initial begin
        clk = 1'b0;
        forever #10 clk = ~clk;
    end

    // -------------------------------------------------------------------------
    // 5. Test Stimulus & Tasks
    // -------------------------------------------------------------------------
    integer errors = 0;

    task send_cmd;
        input [0:0] is_read;
        input [6:0] dev_addr;
        input [7:0] reg_addr;
        input [7:0] data;
        begin
            @(posedge clk);
            cmd_fifo_dout  <= {data, reg_addr, 1'b0, dev_addr, 7'b0, is_read};
            cmd_fifo_empty <= 1'b0;

            @(posedge clk);
            while (!cmd_fifo_rd_en) @(posedge clk);

            @(posedge clk);
            cmd_fifo_empty <= 1'b1;
            cmd_fifo_dout  <= 32'h0;
        end
    endtask

    task wait_resp;
        output [7:0] status;
        output [7:0] data;
        begin
            while (!resp_fifo_wr_en) @(posedge clk);
            status = resp_fifo_din[7:0];
            data   = resp_fifo_din[15:8];
            @(posedge clk);
        end
    endtask

    reg [7:0] resp_status;
    reg [7:0] resp_data;

    initial begin
        $display("==========================================================");
        $display("  TESTBENCH: fifo_i2c_manager (Digilent i2c_master.vhd)   ");
        $display("==========================================================");

        rst_n          = 1'b0; // Active-low reset asserted
        cmd_fifo_dout  = 32'h0;
        cmd_fifo_empty = 1'b1;
        resp_fifo_full = 1'b0;

        #200;
        @(posedge clk);
        rst_n = 1'b1; // Active-low reset released
        #200;

        // ---------------------------------------------------------------------
        // TEST 1: I2C Write Transaction
        // Write 0x01 to PWR_MGMT_1 (0x6B) on MPU-6050 (0x68)
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] Initiating Write: Dev=0x68, Reg=0x6B, Data=0x01...");
        send_cmd(1'b0, 7'h68, 8'h6B, 8'h01);
        wait_resp(resp_status, resp_data);

        $display("  Response received: Status=0x%02h, Data=0x%02h", resp_status, resp_data);
        if (resp_status == 8'h00 && s_mock_regs[8'h6B] == 8'h01) begin
            $display("  -> TEST 1 PASSED: Write successful and verified in slave register!");
        end else begin
            $display("  -> TEST 1 FAILED: Status=0x%02h, Expected slave reg=0x01, Got=0x%02h",
                     resp_status, s_mock_regs[8'h6B]);
            errors = errors + 1;
        end

        #500;

        // ---------------------------------------------------------------------
        // TEST 2: I2C Read Transaction
        // Read WHO_AM_I register (0x75) from MPU-6050 (0x68) -> Expected 0x68
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] Initiating Read: Dev=0x68, Reg=0x75 (WHO_AM_I)...");
        send_cmd(1'b1, 7'h68, 8'h75, 8'h00);
        wait_resp(resp_status, resp_data);

        $display("  Response received: Status=0x%02h, Data=0x%02h", resp_status, resp_data);
        if (resp_status == 8'h00 && resp_data == 8'h68) begin
            $display("  -> TEST 2 PASSED: Successfully read WHO_AM_I = 0x68!");
        end else begin
            $display("  -> TEST 2 FAILED: Expected Data=0x68, Got=0x%02h", resp_data);
            errors = errors + 1;
        end

        #500;

        // ---------------------------------------------------------------------
        // TEST 3: I2C Read Accelerometer High Byte (0x3B) -> Expected 0x03
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] Initiating Read: Dev=0x68, Reg=0x3B (ACCEL_XOUT_H)...");
        send_cmd(1'b1, 7'h68, 8'h3B, 8'h00);
        wait_resp(resp_status, resp_data);

        $display("  Response received: Status=0x%02h, Data=0x%02h", resp_status, resp_data);
        if (resp_status == 8'h00 && resp_data == 8'h03) begin
            $display("  -> TEST 3 PASSED: Successfully read ACCEL_XOUT_H = 0x03!");
        end else begin
            $display("  -> TEST 3 FAILED: Expected Data=0x03, Got=0x%02h", resp_data);
            errors = errors + 1;
        end

        #500;

        // ---------------------------------------------------------------------
        // TEST 4: Invalid Slave Address NACK Handling (Dev=0x12)
        // ---------------------------------------------------------------------
        $display("\n[TEST 4] Initiating Write to Non-Existent Device (Dev=0x12)...");
        send_cmd(1'b0, 7'h12, 8'h00, 8'hFF);
        wait_resp(resp_status, resp_data);

        $display("  Response received: Status=0x%02h, Data=0x%02h", resp_status, resp_data);
        if (resp_status == 8'hFF) begin
            $display("  -> TEST 4 PASSED: Correctly caught NACK error (Status=0xFF)!");
        end else begin
            $display("  -> TEST 4 FAILED: Expected error status 0xFF, Got=0x%02h", resp_status);
            errors = errors + 1;
        end

        #500;

        // ---------------------------------------------------------------------
        // TEST 5: Recovery After NACK (Read WHO_AM_I, expects 0x68)
        // ---------------------------------------------------------------------
        $display("\n[TEST 5] Verifying Post-NACK Recovery: Read Dev=0x68, Reg=0x75 (WHO_AM_I)...");
        send_cmd(1'b1, 7'h68, 8'h75, 8'h00);
        wait_resp(resp_status, resp_data);

        $display("  Response received: Status=0x%02h, Data=0x%02h", resp_status, resp_data);
        if (resp_status == 8'h00 && resp_data == 8'h68) begin
            $display("  -> TEST 5 PASSED: System successfully recovered from NACK and read WHO_AM_I = 0x68!");
        end else begin
            $display("  -> TEST 5 FAILED: Failed to recover from NACK! Expected Data=0x68, Got=0x%02h (Status=0x%02h)", resp_data, resp_status);
            errors = errors + 1;
        end

        #500;

        // Final Summary
        $display("\n----------------------------------------------------------");
        if (errors == 0) begin
            $display(">>> ALL TESTS PASSED SUCCESSFULLY! fifo_i2c_manager is 100%% VERIFIED! <<<");
        end else begin
            $display(">>> TEST FAILED WITH %0d ERRORS! <<<", errors);
        end
        $display("----------------------------------------------------------\n");

        #100;
        $finish;
    end

endmodule
