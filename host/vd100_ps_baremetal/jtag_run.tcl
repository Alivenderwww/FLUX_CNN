# =============================================================================
# jtag_run.tcl  --  通过 JTAG 烧 bitstream + 跑 baremetal ELF (xsct)
#
# 前提: VD100 板 USB JTAG 接 PC, 上电
# 用法: "D:\Xilinx\Vitis\2023.2\bin\xsct.bat" host/vd100_ps_baremetal/jtag_run.tcl
# =============================================================================

set PDI "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.pdi"
set ELF "C:/_Project/FLUX_CNN/host/vd100_ps_baremetal/workspace/resnet11_app/Debug/resnet11_app.elf"

# 1. 连 JTAG
puts "[INFO] Connecting JTAG..."
connect

# 2. 列 target (找 A72 #0)
targets -filter {name =~ "*A72 #0*"}
set a72_target [lindex [targets -filter {name =~ "*A72 #0*"} -target-properties] 0 0]
puts "[INFO] A72 target: $a72_target"

# 3. 下 .pdi (Versal 比 7-series 简单, .pdi 含 PS 配置 + PL bitstream + boot loader)
puts "[INFO] Programming PDI: $PDI"
device program $PDI

# 4. Reset target
puts "[INFO] Reset A72..."
targets -set -filter {name =~ "*A72 #0*"}
rst -system

# 5. Download ELF
puts "[INFO] Downloading ELF: $ELF"
dow $ELF

# 6. Run
puts "[INFO] Continue execution..."
con

puts ""
puts "============================================================"
puts " Baremetal app running on A72."
puts " Open UART terminal (Vitis Serial Terminal or putty 115200 8N1)"
puts " 应该看到: '[srv] ResNet11 server listen on 5000'"
puts "============================================================"
