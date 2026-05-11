# =============================================================================
# jtag_run_elf_only.tcl  --  端到端 JTAG bring-up: program PDI + dow ELF + run
#
# 配套 run_xsct.ps1 跑 (它会启 Vivado hw_server + 注入 connect -url tcp:3121).
# 不要直接跑 .\xsct.bat (Vitis hw_server 看不到 Digilent cable).
#
# 用法:
#   .\run_xsct.ps1 jtag_run_elf_only.tcl
# =============================================================================

set PDI "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.pdi"
set ELF "C:/_Project/FLUX_CNN/host/vd100_ps_baremetal/workspace/resnet11_app/Debug/resnet11_app.elf"

puts "INFO targets:"
targets

puts "INFO Selecting Versal device for PDI program..."
targets -set -filter {name =~ "*xcve2302*" || name =~ "*Versal*"}

puts "INFO rst -por (Versal POR_B via JTAG TAP, 不依赖 PMC 响应; 等价 power-cycle)..."
catch {rst -por}
after 2000

puts "INFO Programming PDI: $PDI"
device program $PDI

puts "INFO Selecting A72 #0..."
targets -set -filter {name =~ "*A72 #0*"}

puts "INFO rst -processor (PMC default subsystem 已 active, A72 释放)..."
rst -processor

puts "INFO Downloading ELF: $ELF"
dow $ELF

puts "INFO Continue execution..."
con

puts ""
puts "============================================================"
puts " A72 baremetal lwIP server running!"
puts " 板 IP 169.254.111.10:5000  (PC 169.254.111.133)"
puts " PC GUI:"
puts "   & C:\\_Project\\FLUX_CNN\\toolchain\\.venv\\Scripts\\python.exe \\"
puts "       C:\\_Project\\FLUX_CNN\\host\\vd100_pc\\resnet11_gui.py"
puts " GUI 里 IP=169.254.111.10, port=5000, 选图 → 连接 → 推理"
puts "============================================================"
