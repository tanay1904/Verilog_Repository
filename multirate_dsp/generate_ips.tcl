# ==============================================================================
# Tcl Script to Generate Xilinx FIR Compiler IPs in Vivado
# Run this in your Vivado Tcl Console:
# source C:/Users/tanay/multirate_dsp/generate_ips.tcl
# ==============================================================================

# Determine script directory dynamically for Linux / Windows portability
set script_dir [file dirname [file normalize [info script]]]
if {[file exists [file join $script_dir "fir_coeffs.coe"]]} {
    set coe_path [file join $script_dir "fir_coeffs.coe"]
} else {
    set coe_path "C:/Users/tanay/multirate_dsp/ip/fir_coeffs.coe"
}

# 1. Create Decimator IP (Decimation by 3, 120 MSPS -> 40 MSPS)
create_ip -name fir_compiler -vendor xilinx.com -library ip -version 7.2 -module_name fir_compiler_decimator_by_3
set_property -dict [list \
    CONFIG.Filter_Type {Decimation} \
    CONFIG.Decimation_Rate {3} \
    CONFIG.Interpolation_Rate {1} \
    CONFIG.Coefficient_File $coe_path \
    CONFIG.Coefficient_Sets {1} \
    CONFIG.Coefficient_Sign {Signed} \
    CONFIG.Coefficient_Width {16} \
    CONFIG.Quantization {Integer_Coefficients} \
    CONFIG.Data_Width {16} \
    CONFIG.Data_Sign {Signed} \
    CONFIG.Sample_Frequency {120} \
    CONFIG.Clock_Frequency {120} \
] [get_ips fir_compiler_decimator_by_3]
generate_target all [get_ips fir_compiler_decimator_by_3]

# 2. Create Interpolator IP (Interpolation by 12, 40 MSPS -> 480 MSPS)
create_ip -name fir_compiler -vendor xilinx.com -library ip -version 7.2 -module_name fir_compiler_interpolator_by_12
set_property -dict [list \
    CONFIG.Filter_Type {Interpolation} \
    CONFIG.Interpolation_Rate {12} \
    CONFIG.Decimation_Rate {1} \
    CONFIG.Coefficient_File $coe_path \
    CONFIG.Coefficient_Sets {1} \
    CONFIG.Coefficient_Sign {Signed} \
    CONFIG.Coefficient_Width {16} \
    CONFIG.Quantization {Integer_Coefficients} \
    CONFIG.Data_Width {16} \
    CONFIG.Data_Sign {Signed} \
    CONFIG.Sample_Frequency {40} \
    CONFIG.Clock_Frequency {120} \
] [get_ips fir_compiler_interpolator_by_12]
generate_target all [get_ips fir_compiler_interpolator_by_12]

puts "=========================================================="
puts "[SUCCESS] Both FIR Compiler IPs generated successfully!"
puts "Now click: Run Simulation -> Run Behavioral Simulation"
puts "=========================================================="
