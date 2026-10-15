# Verilog Repository: Custom FPGA RTL & Vitis Hardware Datapaths

Target FPGA: **Xilinx Zynq-7000 SoC (XC7Z020-CLG400-1 / PYNQ-Z2)**  
Hardware Design Suite: **Vivado Design Suite 2020.2+ / Vitis Unified Platform**  
Hardware Language: **Verilog HDL (IEEE 1364-2001 / IEEE 1364-2005)**  

---

## 1. Overview

`Verilog_Repository` is the centralized repository containing all custom, hand-crafted Verilog RTL modules, AXI4-Lite IP peripherals, hardware mixers, multi-rate clock enables, ESC PWM generators, and system integration top-wrappers developed for the PYNQ-Z2 autonomous quadcopter flight control system.

> [!IMPORTANT]
> **100% Custom RTL / Zero Simulink Overhead**: This repository strictly contains hand-written, production-optimized Verilog modules designed for real-time FPGA execution and seamless **Vitis Conversion**. All auto-generated MATLAB/Simulink HDL Coder files have been excluded.

---

## 2. Repository Structure

```
Verilog_Repository/
├── .gitignore                          # Clean filters for Vivado runs, caches, bitstreams, and waveforms
├── README.md                           # Master RTL documentation & module reference
│
├── vitis_conversion/                   # Core RTL Modules Developed for Vitis Integration
│   ├── rc_decoder_axi/                 # Complete AXI4-Lite 6-Channel Hardware RC Decoder IP
│   │   ├── rc_decoder_ip_v1_0.v        # Top AXI4-Lite IP wrapper connecting bus to decoder core
│   │   ├── rc_decoder_ip_v1_0_S00_AXI.v# AXI memory-mapped register interface (channels & failsafe)
│   │   ├── rc_input_decoder.v          # 4/6-channel precision microsecond pulse width timer
│   │   └── rc_input_decoder_top.v      # Board-level top module with LED failsafe status
│   │
│   ├── motor_mixing/                   # Hardware Motor Mixer Datapath
│   │   ├── motor_mixer.v               # Quadcopter X-Frame force/moment-to-ESC duty cycle mixer
│   │   └── u2_to_mixer.v               # Roll torque (U2) format adapter (sfix24_En16 -> sfix36_En30)
│   │
│   ├── pwm_generation/                 # Hardware ESC Actuator Drivers
│   │   ├── pwm_gen_new.v               # Multi-rate parameterized PWM generator (10/100 MHz, 50 Hz/400 Hz)
│   │   ├── pwm_gen.v                   # 50 Hz standard PWM servo/ESC generator
│   │   ├── pwm_mux.v                   # Hardware manual RC vs autonomous flight controller PWM multiplexer
│   │   └── pulse_generator.v           # Strobe pulse generator for multi-rate synchronization
│   │
│   ├── multiplexers_and_routing/       # Vitis Flight Core Control Flow Multiplexers
│   │   ├── rc_cmd_mux.v                # RC command multiplexer with auto-failsafe switchover
│   │   ├── U1_mux.v                    # Thrust (U1) manual vs autonomous altitude controller mux
│   │   ├── U4_mux.v                    # Yaw torque (U4) manual vs autonomous heading controller mux
│   │   └── mux_2_input.v               # Parameterized 2-to-1 data multiplexer
│   │
│   ├── clocking_and_enables/           # Sample Rate and Clock Enable Generators
│   │   ├── clk_en.v                    # Sampling rate clock enable generator (e.g., 250 Hz)
│   │   ├── clk_down_10MHz_250Hz.v      # Hardware 10 MHz to 250 Hz synchronous clock divider
│   │   └── ce_generator.v              # Multi-rate clock enable generator for cascaded loops
│   │
│   ├── format_converters/              # Fixed-Point & Precision Bus Adapters
│   │   ├── Sfix_En10_to_En11.v         # Fixed-point format converter (sfix16_En10 -> sfix16_En11)
│   │   ├── sfix_converter.v            # Signed fixed-point precision alignment
│   │   └── vest.v                      # Velocity estimator format adapter (sfix16_En12 -> sfix17_En12)
│   │
│   ├── mpu6050_hardware_driver/        # Pure RTL MPU-6050 Hardware Driver Subsystem
│   │   ├── mpu6050_driver.v            # Top-level subsystem integrating HLS core, FIFOs, and I2C master
│   │   ├── fifo_i2c_mgr.v              # FIFO-to-I2C manager bridging HLS streaming FIFOs to I2C master
│   │   ├── i2c_master.vhd              # Hardware Digilent I2C master core in VHDL
│   │   ├── mpu6050_pynq_constraints.xdc# Pin constraints for PYNQ-Z2 board (SCL/SDA)
│   │   └── mpu6050_driver_hw.bd        # Vivado Block Design wiring PS7, AXI GPIOs, and MPU driver
│   │
│   └── hls_mpu6050_axi_core/           # Vitis HLS Synthesized MPU-6050 AXI I2C Controller Core
│       ├── top_mpu6050_iic.v           # Top-level AXI I2C master state machine
│       ├── top_mpu6050_iic_mpu_init_1.v # InvenSense MPU-6050 hardware initialization engine
│       ├── top_mpu6050_iic_i2c_transaction.v # I2C start/stop/data transaction pipeline
│       ├── top_mpu6050_iic_send_i2c_cmd.v    # I2C command dispatcher
│       ├── top_mpu6050_iic_recv_i2c_resp.v   # I2C response collector
│       ├── top_mpu6050_iic_delay_us.v        # Microsecond hardware delay timer
│       └── top_mpu6050_iic_*.v         # Floating-point/DSP conversion arithmetic units
│
├── system_integration/                 # Full System Hardware Interconnect Top Modules
│   ├── complementary_filter_top.v      # Hardware wrapper combining complementary filter, pulse generator, and phi_acc
│   ├── Attitude_Control_System_Top.v   # Complete FPGA Attitude Control Loop Top Integration
│   ├── Alti_Yaw_top.v                  # Barometric Altitude & Yaw Gyro Heading Top Integration
│   ├── yaw_control_top.v               # Yaw Filter and Cascaded Heading Controller Top Integration
│   ├── top_YAW.v                       # Dedicated Yaw heading stabilizer top module
│   ├── Attitude_Position_Top.v         # Zero-latency wire interconnect top for outer position + attitude loop
│   ├── Integrator.v                    # Double integrator with anti-windup for physical kinematics
│   └── proposed_Integrator.v           # Enhanced anti-windup integrator
│
├── legacy_rc_frontend/                 # Dedicated Single-Channel RC Decoders
│   ├── throttle_pwm_decoder.v          # Standalone throttle PWM pulse width decoder with failsafe & LED
│   └── throttle_rc_frontend.v          # Front-end synchronizer for throttle receiver input
│
└── testbenches/                        # Verification Testbenches
    ├── tb_mpu6050_driver.v             # Full closed-loop MPU-6050 driver + behavioral I2C slave testbench
    ├── tb_fifo_i2c_mgr.v               # FIFO-to-I2C manager read/write handshake testbench
    ├── tb_motor_mixer.v                # Motor mixer testbench
    ├── tb_PWM_channels.v               # 4-channel PWM generator testbench
    ├── tb_pulse_generator_250Hz.v      # 250 Hz timing testbench
    ├── tb_cascaded_top.v               # Full cascaded attitude + rate controller testbench
    ├── tb_command_change.v             # Step response testbench
    ├── tb_ramp.v                       # Ramp response testbench
    ├── tb_Acceleration_vectors.v       # Coordinate transformation testbench
    ├── tb_Complementary_filter.v       # Attitude estimation filter testbench
    ├── tb_Position_Filter.v            # Position filter testbench
    ├── tb_pos_con.v                    # Position controller testbench
    ├── tb_tier1_integrator.v           # Tier 1 numerical integrator test
    ├── tb_tier1_scaling.v              # Tier 1 fixed-point scaling test
    ├── tb_tier2_boundaries.v           # Tier 2 numerical boundary & saturation test
    ├── tb_tier3_coupling.v             # Tier 3 multi-axis cross-coupling test
    └── tb.v                            # Integrated top testbench
```

---

## 3. Module Details & Vitis Conversion Architecture

### 3.1 RC Decoder AXI IP (`vitis_conversion/rc_decoder_axi/`)
- Measures receiver pulse widths in microseconds ($1000\text{--}2000\text{ }\mu\text{s}$) across Roll, Pitch, Throttle, and Yaw.
- Contains an automatic timeout failsafe: if pulses are missing for $> 100\text{ ms}$, the failsafe flag asserts immediately, zeroing the throttle output.
- Memory-mapped into the ARM Cortex-A9 address space via AXI4-Lite slave registers (`slv_reg0` to `slv_reg4`), enabling bare-metal Vitis software to read calibrated stick inputs directly with zero CPU interrupt latency.

### 3.2 Quadcopter Motor Mixer (`vitis_conversion/motor_mixing/`)
- Implements the X-configuration quadcopter mixing equations:
  $$\begin{aligned}
  M_1 &= U_1 - U_2 + U_3 + U_4 \\
  M_2 &= U_1 + U_2 - U_3 + U_4 \\
  M_3 &= U_1 + U_2 + U_3 - U_4 \\
  M_4 &= U_1 - U_2 - U_3 - U_4
  \end{aligned}$$
- Direct hardware scaling ensures zero latency between attitude PID calculation and motor PWM generation.

### 3.3 Multi-Rate PWM Generator (`vitis_conversion/pwm_generation/`)
- Parameterized clock input (`CLK_FREQ_MHZ`, e.g., 10 MHz or 100 MHz).
- Produces $50\text{ Hz}$ or $400\text{ Hz}$ ESC pulse trains with $1\text{ }\mu\text{s}$ tick resolution.
- Integrated safety disarm clamps outputs to $1000\text{ }\mu\text{s}$ or zero on loss of transmitter signal.

### 3.4 Hardware Multiplexing & Overrides (`vitis_conversion/multiplexers_and_routing/`)
- Facilitates autonomous transition between pilot manual control and Vitis software flight control loops.
- `rc_cmd_mux.v`, `U1_mux.v`, and `U4_mux.v` allow dynamic routing of throttle, attitude, and yaw setpoints without recompiling FPGA bitstreams.

---

## 4. Integration with Vivado & Vitis

1. **In Vivado Block Design (BD)**:
   - Add `vitis_conversion/rc_decoder_axi/` to your IP Repository (`Settings -> IP -> Repository`).
   - Instantiate `rc_decoder_ip_v1_0` into the block diagram and connect `S00_AXI` to the Zynq PS AXI Interconnect.
   - Connect PWM receiver pins to the PYNQ-Z2 board constraints (`.xdc`).
2. **In Vitis Software**:
   - Access the RC channels using the generated base address from `xparameters.h`:
     ```c
     #define RC_DECODER_BASEADDR XPAR_RC_DECODER_IP_0_S00_AXI_BASEADDR
     int32_t throttle = Xil_In32(RC_DECODER_BASEADDR + 0x00);
     int32_t roll     = Xil_In32(RC_DECODER_BASEADDR + 0x04);
     int32_t pitch    = Xil_In32(RC_DECODER_BASEADDR + 0x08);
     int32_t yaw      = Xil_In32(RC_DECODER_BASEADDR + 0x0C);
     int32_t failsafe = Xil_In32(RC_DECODER_BASEADDR + 0x10);
     ```

---

## 5. Version Control & Git

- Git Branch: `main`
- Target Remote: `https://github.com/tanay1904/Verilog_Repository.git`
