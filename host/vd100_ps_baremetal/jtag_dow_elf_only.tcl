# =============================================================================
# jtag_dow_elf_only.tcl  --  ELF download + run, 不烧 PDI
#
# 前提: PDI 已经通过 program_pdi_via_vivado.tcl 烧过, PMC + a72 已 active.
# xsct 的 `device program` 在 PMC 已 boot 态会 hang, Vivado HW Manager backend 不会.
# =============================================================================

set ELF "C:/_Project/FLUX_CNN/host/vd100_ps_baremetal/workspace2/resnet11_app/Debug/resnet11_app.elf"

puts "INFO targets:"
targets

puts "INFO Selecting A72 #0..."
targets -set -filter {name =~ "*A72 #0*"}

puts "INFO stop A72 (PLM 默认 free-run, dow 前必须 halt 才能用 DAP)..."
catch {stop} stop_err
puts "  stop: $stop_err"

puts "INFO rst -processor -clear-registers (清寄存器, 让 a72 halted 状态等 dow)..."
rst -processor -clear-registers

puts "INFO stop again after rst..."
catch {stop} stop_err
puts "  stop: $stop_err"

puts "INFO Downloading ELF: $ELF"
dow $ELF

puts "INFO Continue execution..."
con

puts ""
puts "============================================================"
puts " A72 baremetal lwIP server running!"
puts " 板 IP 169.254.111.10:5000  (PC 169.254.111.133)"
puts "============================================================"
