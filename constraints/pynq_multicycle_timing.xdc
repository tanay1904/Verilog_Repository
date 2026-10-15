## ============================================================================
## PYNQ-Z2 Constraints for Flight Controller (RC_Ctrl_Attitude)
## ============================================================================

## ----------------------------------------------------------------------------
## 1. Failsafe & Status Indicator (Onboard User LED)
## ----------------------------------------------------------------------------
## LD0 on PYNQ-Z2 board
set_property -dict { PACKAGE_PIN R14   IOSTANDARD LVCMOS33 } [get_ports { failsafe_led_0 }];

## ----------------------------------------------------------------------------
## 2. I2C Bus (PMOD A - Edge Connector near SPI)
## ----------------------------------------------------------------------------
## Sensor wiring to PMOD A:
##   PMOD A Pin 6  : 3.3V  -> Sensor VCC
##   PMOD A Pin 5  : GND   -> Sensor GND
##   PMOD A Pin 4  : Y16   -> Sensor SDA (JA2_P)
##   PMOD A Pin 10 : Y17   -> Sensor SCL (JA2_N)
set_property -dict { PACKAGE_PIN Y17   IOSTANDARD LVCMOS33  PULLUP TRUE } [get_ports { IIC_0_scl_io }]; # JA4_N (Pin 10)
set_property -dict { PACKAGE_PIN Y16   IOSTANDARD LVCMOS33  PULLUP TRUE } [get_ports { IIC_0_sda_io }]; # JA4_P (Pin 4)

## ----------------------------------------------------------------------------
## 3. RC Receiver Inputs (Arduino Digital Header AR0 - AR7)
## ----------------------------------------------------------------------------
## Connect 4 RC receiver PWM channel signal pins here:
#set_property -dict { PACKAGE_PIN T14   IOSTANDARD LVCMOS33 } [get_ports { pwm_roll_0 }];     # AR0
#set_property -dict { PACKAGE_PIN U12   IOSTANDARD LVCMOS33 } [get_ports { pwm_pitch_0 }];    # AR1
#set_property -dict { PACKAGE_PIN U13   IOSTANDARD LVCMOS33 } [get_ports { pwm_throttle_0 }]; # AR2
#set_property -dict { PACKAGE_PIN V13   IOSTANDARD LVCMOS33 } [get_ports { pwm_yaw_0 }];      # AR3

## Spare Arduino pins reserved for future RC channels (e.g. Flight Mode, Arm):
## AR4: V15 | AR5: T15 | AR6: R16 | AR7: U17

## ----------------------------------------------------------------------------
## 4a. Motor ESC PWM Outputs (Raspberry Pi 40-Pin Header) For High Power Motors
## ----------------------------------------------------------------------------
## Configured for 3-pin ESC servo connectors [ SIG , NC (middle disconnected) , GND ]
## Placed along the inner (even) row of the RPi header:
##
## Motor 1 (ESC 1):
##   RPi Pin 10 : Y6   -> Signal (pwm_out_1_0)
##   RPi Pin 8  : V6   -> Middle pin (NC / disconnected)
##   RPi Pin 6  : GND  -> ESC Ground
##
## Motor 2 (ESC 2):
##   RPi Pin 18 : Y7   -> Signal (pwm_out_2_0)
##   RPi Pin 16 : W6   -> Middle pin (NC / disconnected)
##   RPi Pin 14 : GND  -> ESC Ground
##
## Motor 3 (ESC 3):
##   RPi Pin 24 : F19  -> Signal (pwm_out_3_0)
##   RPi Pin 22 : F20  -> Middle pin (NC / disconnected)
##   RPi Pin 20 : GND  -> ESC Ground
##
## Motor 4 (ESC 4):
##   RPi Pin 38 : A20  -> Signal (pwm_out_4_0)
##   RPi Pin 36 : B19  -> Middle pin (NC / disconnected)
##   RPi Pin 34 : GND  -> ESC Ground

#set_property -dict { PACKAGE_PIN Y6    IOSTANDARD LVCMOS33 } [get_ports { pwm_out_1_0 }]; # RPi Pin 10
#set_property -dict { PACKAGE_PIN Y7    IOSTANDARD LVCMOS33 } [get_ports { pwm_out_2_0 }]; # RPi Pin 18
#set_property -dict { PACKAGE_PIN F19   IOSTANDARD LVCMOS33 } [get_ports { pwm_out_3_0 }]; # RPi Pin 24
#set_property -dict { PACKAGE_PIN A20   IOSTANDARD LVCMOS33 } [get_ports { pwm_out_4_0 }]; # RPi Pin 38

### ----------------------------------------------------------------------------
### 4b. Motor ESC PWM Outputs (PMOD JA & JB Headers) For Low Power Motors
### ----------------------------------------------------------------------------
### Standard 12-pin (2x6) PMOD Headers JA and JB
### Pin 5 & Pin 11 on each PMOD are common Board Ground (GND).
###
### Motor 1 (ESC 1):
###   PMOD JA Pin 1  : Y18  -> Signal (pwm_out_1_0) [ja_p[1]]
###   PMOD JA Pin 5  : GND  -> ESC Ground (Top row GND)
###
### Motor 2 (ESC 2):
###   PMOD JB Pin 10 : W13  -> Signal (pwm_out_2_0) [jb_n[4]]
###   PMOD JB Pin 11 : GND  -> ESC Ground (Bottom row GND)
###
### Motor 3 (ESC 3):
###   PMOD JA Pin 10 : W19  -> Signal (pwm_out_3_0) [ja_n[4]]
###   PMOD JA Pin 11 : GND  -> ESC Ground (Bottom row GND)
###
### Motor 4 (ESC 4):
###   PMOD JB Pin 1  : W14  -> Signal (pwm_out_4_0) [jb_p[1]]
###   PMOD JB Pin 5  : GND  -> ESC Ground (Top row GND)

#set_property -dict { PACKAGE_PIN Y18  IOSTANDARD LVCMOS33 } [get_ports { pwm_out_1_0 }]; # PMOD JA Pin 1  (ja_p[1]) - Motor 1
#set_property -dict { PACKAGE_PIN W13  IOSTANDARD LVCMOS33 } [get_ports { pwm_out_2_0 }]; # PMOD JB Pin 10 (jb_n[4]) - Motor 2
#set_property -dict { PACKAGE_PIN W19  IOSTANDARD LVCMOS33 } [get_ports { pwm_out_3_0 }]; # PMOD JA Pin 10 (ja_n[4]) - Motor 3
#set_property -dict { PACKAGE_PIN W14  IOSTANDARD LVCMOS33 } [get_ports { pwm_out_4_0 }]; # PMOD JB Pin 1  (jb_p[1]) - Motor 4
### ----------------------------------------------------------------------------
### 5. Timing Exception Constraints (Multicycle Paths)
### ----------------------------------------------------------------------------

### 5a. Software AXI GPIO Registers -> Attitude Filter
#set_multicycle_path 2 -setup -from [get_cells -hier -filter {NAME =~ *gpio*gpio_Data_Out_reg[*]}] -to [get_cells -hier -filter {NAME =~ *Attitude_filter*x_in_reg_out_1_reg[*]}]
#set_multicycle_path 1 -hold  -from [get_cells -hier -filter {NAME =~ *gpio*gpio_Data_Out_reg[*]}] -to [get_cells -hier -filter {NAME =~ *Attitude_filter*x_in_reg_out_1_reg[*]}]

### 5b. Attitude Complementary Filter -> Attitude Pitch Controller (clk_fpga_0 @ 10 MHz)
### Filter updates at 250 Hz (4 ms sample time) via internal clock enable.
### The 156-level combinational datapath (Sqrt + Trig + Euler + PID) takes ~99.5 ns.
### 2 clock cycles (200 ns setup, 1 cycle hold) achieves robust timing closure.
#set_multicycle_path 2 -setup -from [get_cells -hier -filter {NAME =~ *Attitude_filter1_att_0*}] -to [get_cells -hier -filter {NAME =~ *Attitude_pitch_Contr_0*}]
#set_multicycle_path 1 -hold  -from [get_cells -hier -filter {NAME =~ *Attitude_filter1_att_0*}] -to [get_cells -hier -filter {NAME =~ *Attitude_pitch_Contr_0*}]

### 5c. RC Decoder PWM Normalization Dividers (clk_fpga_1 @ 100 MHz)
### Combinational integer divisions (/ 100 and / 1000) take ~39.4 ns.
### RC pulses only arrive at 50 Hz (every 20 ms), so 5 cycles (50 ns) provides full margin.
set_multicycle_path 5 -setup -from [get_cells -hier -filter {NAME =~ *rc_decoder_inst/measured_*_reg[*]}] -to [get_cells -hier -filter {NAME =~ *Sqrt1_1_1/*_reg_reg[*]}]
set_multicycle_path 4 -hold  -from [get_cells -hier -filter {NAME =~ *rc_decoder_inst/measured_*_reg[*]}] -to [get_cells -hier -filter  {NAME =~ *Sqrt1_1_1/*_reg_reg[*]}]



### ============================================================================
### PYNQ-Z2 Constraints for fabric_rc_ctrl 
### ============================================================================

### ----------------------------------------------------------------------------
### 1. RC Receiver Inputs (Arduino Digital Header AR0 - AR3)
### ----------------------------------------------------------------------------
### Connect your 4 RC receiver PWM channel signal pins here:
#set_property -dict { PACKAGE_PIN T14   IOSTANDARD LVCMOS33 } [get_ports { hw_roll_0 }];     # AR0 (Roll)
#set_property -dict { PACKAGE_PIN U12   IOSTANDARD LVCMOS33 } [get_ports { hw_pitch_0 }];    # AR1 (Pitch)
#set_property -dict { PACKAGE_PIN U13   IOSTANDARD LVCMOS33 } [get_ports { hw_throttle_0 }]; # AR2 (Throttle)
#set_property -dict { PACKAGE_PIN V13   IOSTANDARD LVCMOS33 } [get_ports { hw_yaw_0 }];      # AR3 (Yaw)

### ----------------------------------------------------------------------------
### 2. Motor ESC PWM Outputs (PMOD JA & JB Headers)
### ----------------------------------------------------------------------------
### Motor 1 (ESC 1) -> PMOD JA Pin 1 (Y18) | Ground -> JA Pin 5
#set_property -dict { PACKAGE_PIN Y18   IOSTANDARD LVCMOS33 } [get_ports { pwm_out_1_0 }]; 

### Motor 2 (ESC 2) -> PMOD JB Pin 10 (W13) | Ground -> JB Pin 11
#set_property -dict { PACKAGE_PIN W13   IOSTANDARD LVCMOS33 } [get_ports { pwm_out_2_0 }]; 

### Motor 3 (ESC 3) -> PMOD JA Pin 10 (W19) | Ground -> JA Pin 11
#set_property -dict { PACKAGE_PIN W19   IOSTANDARD LVCMOS33 } [get_ports { pwm_out_3_0 }]; 

### Motor 4 (ESC 4) -> PMOD JB Pin 1 (W14)  | Ground -> JB Pin 5
#set_property -dict { PACKAGE_PIN W14   IOSTANDARD LVCMOS33 } [get_ports { pwm_out_4_0 }]; 

### ----------------------------------------------------------------------------
### 3. Timing Exception (Decoder Divider Margin)
### ----------------------------------------------------------------------------
#set_multicycle_path 5 -setup -from [get_cells -hier -filter {NAME =~ *rc_input_decoder*measured_*_reg[*]}] -to [get_cells -hier -filter {NAME =~ *rc_input_decoder*/*_next_reg[*]}]
#set_multicycle_path 4 -hold  -from [get_cells -hier -filter {NAME =~ *rc_input_decoder*measured_*_reg[*]}] -to [get_cells -hier -filter {NAME =~ *rc_input_decoder*/*_next_reg[*]}]\