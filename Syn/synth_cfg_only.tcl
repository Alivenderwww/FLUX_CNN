# synth_cfg_only.tcl -- OOC synthesize cfg_regs.sv 单独, 估算 LUT 占用变化
# 用法: vivado -mode batch -source synth_cfg_only.tcl -nojournal -nolog

set TOP cfg_regs

# Versal AI Edge xcve2302 (跟 vd100_minimal 一致)
read_verilog -sv "C:/_Project/FLUX_CNN/RTL/cfg_regs.sv"

synth_design -top $TOP -part xcve2302-sfva784-1LP-e-S -mode out_of_context \
             -include_dirs "C:/_Project/FLUX_CNN/RTL"

report_utilization -file "C:/_Project/FLUX_CNN/Syn/cfg_regs_ooc_util.rpt"

puts ""
puts "=========== OOC util report: cfg_regs_ooc_util.rpt ==========="
exit 0
