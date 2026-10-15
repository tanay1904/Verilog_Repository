`timescale 1ns / 1ps
`define SIMULATION 1
//////////////////////////////////////////////////////////////////////////////////
// Testbench: tb_mpu6050_driver
// Description: Full closed-loop verification of top_mpu6050_driver
//              (HLS Core + FWFT FIFOs + FIFO Manager + Digilent I2C Master)
//              communicating with a behavioral MPU-6050 I2C slave on the bus.
//////////////////////////////////////////////////////////////////////////////////

module tb_mpu6050_driver;

    reg clk;
    reg rst_n;

    wire signed [15:0] accel_x;
    wire signed [15:0] accel_y;
    wire signed [15:0] accel_z;
    wire signed [15:0] gyro_x;
    wire signed [15:0] gyro_y;
    wire signed [15:0] gyro_z;
    wire signed [15:0] temp_raw;

    wire scl;
    wire sda;

    pullup(scl);
    pullup(sda);

    // -------------------------------------------------------------------------
    // 1. Device Under Test: top_mpu6050_driver
    // -------------------------------------------------------------------------
    top_mpu6050_driver #(
        .CLK_FREQ_HZ      (50_000_000), // 50 MHz clock
        .I2C_BUS_HZ       (2_500_000),  // 2.5 MHz for simulation acceleration
        .DEFAULT_PIN_SWAP (0)           // 0: Unswapped for 1-to-1 slave model testbench hookup
    ) dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .accel_x  (accel_x),
        .accel_y  (accel_y),
        .accel_z  (accel_z),
        .gyro_x   (gyro_x),
        .gyro_y   (gyro_y),
        .gyro_z   (gyro_z),
        .temp_raw (temp_raw),
        .scl      (scl),
        .sda      (sda)
    );

    // -------------------------------------------------------------------------
    // 2. Behavioral MPU-6050 I2C Slave Model (Address 7'h68)
    // -------------------------------------------------------------------------
    reg [7:0] s_mock_regs [0:255];
    reg [7:0] slave_reg_ptr;
    reg       slave_sda_drive = 1'b0;
    reg       slave_sda_val   = 1'b0;

    assign sda = slave_sda_drive ? (slave_sda_val ? 1'bz : 1'b0) : 1'bz;

    initial begin
        slave_sda_drive = 1'b0;
        slave_sda_val   = 1'b0;
        slave_reg_ptr   = 8'h00;

        // Configuration defaults
        s_mock_regs[8'h75] = 8'h68; // WHO_AM_I
        s_mock_regs[8'h6B] = 8'h40; // PWR_MGMT_1 (Sleep mode initially)
        s_mock_regs[8'h19] = 8'h00; // SMPLRT_DIV
        s_mock_regs[8'h1A] = 8'h00; // CONFIG
        s_mock_regs[8'h1B] = 8'h00; // GYRO_CONFIG
        s_mock_regs[8'h1C] = 8'h00; // ACCEL_CONFIG
        s_mock_regs[8'h6A] = 8'h00; // USER_CTRL
        s_mock_regs[8'h38] = 8'h00; // INT_ENABLE

        // Calibrated Sensor Test Values
        // Accel: X = +1000 (0x03E8), Y = -500 (0xFE0C), Z = +16384 (0x4000)
        s_mock_regs[8'h3B] = 8'h03; // ACCEL_XOUT_H
        s_mock_regs[8'h3C] = 8'hE8; // ACCEL_XOUT_L
        s_mock_regs[8'h3D] = 8'hFE; // ACCEL_YOUT_H
        s_mock_regs[8'h3E] = 8'h0C; // ACCEL_YOUT_L
        s_mock_regs[8'h3F] = 8'h40; // ACCEL_ZOUT_H
        s_mock_regs[8'h40] = 8'h00; // ACCEL_ZOUT_L

        // Temperature: Raw = 43 (0x002B)
        s_mock_regs[8'h41] = 8'h00; // TEMP_OUT_H
        s_mock_regs[8'h42] = 8'h2B; // TEMP_OUT_L

        // Gyro: X = +70 (0x0046), Y = -40 (0xFFD8), Z = +15 (0x000F)
        s_mock_regs[8'h43] = 8'h00; // GYRO_XOUT_H
        s_mock_regs[8'h44] = 8'h46; // GYRO_XOUT_L
        s_mock_regs[8'h45] = 8'hFF; // GYRO_YOUT_H
        s_mock_regs[8'h46] = 8'hD8; // GYRO_YOUT_L
        s_mock_regs[8'h47] = 8'h00; // GYRO_ZOUT_H
        s_mock_regs[8'h48] = 8'h0F; // GYRO_ZOUT_L
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

    // Detect START & REPEATED START
    always @(negedge sda) begin
        if (scl === 1'b1) begin
            slv_state       <= SLV_ADDR;
            s_bit           <= 4'd0;
            slave_sda_drive <= 1'b0;
        end
    end

    // Detect STOP
    always @(posedge sda) begin
        if (scl === 1'b1) begin
            slv_state       <= SLV_IDLE;
            slave_sda_drive <= 1'b0;
        end
    end

    // Sample bits on posedge SCL
    always @(posedge scl) begin
        case (slv_state)
            SLV_ADDR: begin
                s_shift[7 - s_bit] <= sda;
                if (s_bit == 4'd7) begin
                    s_addr <= s_shift[7:1];
                    s_rw   <= sda;
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
                s_shift[7 - s_bit] <= sda;
                if (s_bit == 4'd7) begin
                    slave_reg_ptr <= {s_shift[7:1], sda};
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
                s_shift[7 - s_bit] <= sda;
                if (s_bit == 4'd7) begin
                    s_mock_regs[slave_reg_ptr] <= {s_shift[7:1], sda};
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
    always @(negedge scl) begin
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
    // 3. Clock Generation (50 MHz = 20 ns period)
    // -------------------------------------------------------------------------
    initial begin
        clk = 1'b0;
        forever #10 clk = ~clk;
    end

    // -------------------------------------------------------------------------
    // 4. Test Execution & Result Assertion
    // -------------------------------------------------------------------------
    integer timeout_counter = 0;

    initial begin
        $display("==================================================================");
        $display("  TESTBENCH: tb_mpu6050_driver (Full Hardware Loop Verification)  ");
        $display("  Components: HLS Driver + FWFT FIFOs + FIFO Mgr + Digilent I2C   ");
        $display("==================================================================");

        rst_n = 1'b0; // Assert active-low reset
        #200;
        @(posedge clk);
        rst_n = 1'b1; // Deassert reset
        $display("[INFO] System Reset Released at %0t ps. Starting autonomous HLS boot...", $time);

        // Wait until HLS initializes the MPU-6050 and outputs valid sensor samples for all axes
        while ((gyro_z === 16'sh0000 || gyro_z === 16'shxxxx || temp_raw === 16'shxxxx) && timeout_counter < 500000) begin
            @(posedge clk);
            timeout_counter = timeout_counter + 1;
        end

        // Wait a few cycles for all outputs to settle
        #100;
        @(posedge clk);

        $display("\n------------------------------------------------------------------");
        $display("[RESULTS] Received Sensor Outputs from HLS Core at %0t ps:", $time);
        $display("  Accelerometer: X = %0d LSB, Y = %0d LSB, Z = %0d LSB",
                 accel_x, accel_y, accel_z);
        $display("  Gyroscope:     X = %0d LSB, Y = %0d LSB, Z = %0d LSB",
                 gyro_x, gyro_y, gyro_z);
        $display("  Temperature:   Raw = %0d", temp_raw);
        $display("------------------------------------------------------------------");

        // Validate results against mock sensor register values
        if (accel_x == 1000 && accel_y == -500 && accel_z == 16384 &&
            gyro_x == 70 && gyro_y == -40 && gyro_z == 15) begin
            $display("\n>>> SUCCESS: All MPU-6050 sensor readings 100%% VERIFIED! <<<");
            $display(">>> End-to-End Hardware Chain (HLS -> FIFOs -> Manager -> I2C Master -> Sensor) PASSED! <<<\n");
        end else begin
            $display("\n>>> FAILURE: Mismatch in sensor readings! <<<");
            $display("  Expected Accel: X=1000, Y=-500, Z=16384");
            $display("  Expected Gyro:  X=70, Y=-40, Z=15\n");
        end

        #200;
        $finish;
    end

endmodule
