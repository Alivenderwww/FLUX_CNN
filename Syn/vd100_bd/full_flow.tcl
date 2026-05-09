# =============================================================================
# full_flow.tcl  --  VD100 BD 完整 batch 流程: synth + impl + bitstream + xsa
#
# 用法 (PowerShell, 用户已关闭 Vivado GUI):
#   "D:\Xilinx\Vivado\2023.2\bin\vivado.bat" -mode batch -source Syn\vd100_bd\full_flow.tcl -nojournal -log Syn\vd100_bd\full_flow.log
# =============================================================================

set PROJ "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr"

puts "============================================================"
puts " VD100 full flow start: synth + impl + bitstream + .xsa"
puts "============================================================"

open_project $PROJ

# -----------------------------------------------------------------------------
# 1. 诊断
# -----------------------------------------------------------------------------
puts ""
puts "=== Diagnostic ==="
puts "  current top : [get_property top [current_fileset]]"
puts "  fileset has [llength [get_files -of_objects [current_fileset]]] files"
puts "  design_1.bd : [get_files -filter {NAME =~ *design_1.bd}]"
puts "  design_1_wrapper.v : [get_files -filter {NAME =~ *design_1_wrapper*}]"
puts "  axi_dm IP   : [get_ips -quiet axi_dm]"
puts "  multicore_top_vd100_bd_v.v : [get_files -filter {NAME =~ *multicore_top_vd100_bd_v*}]"

# -----------------------------------------------------------------------------
# 2. 强制 top = design_1_wrapper, 锁定
# -----------------------------------------------------------------------------
puts ""
puts "=== Force top = design_1_wrapper ==="
# 先确保 BD wrapper 存在, 不存在就 make
if {[get_files -quiet design_1_wrapper.v] eq ""} {
    puts "  design_1_wrapper.v missing, generating..."
    make_wrapper -files [get_files design_1.bd] -top -import -force
    update_compile_order -fileset sources_1
}
set_property top design_1_wrapper [current_fileset]
set_property top_auto_set false [current_fileset]
puts "  top now = [get_property top [current_fileset]]"

# -----------------------------------------------------------------------------
# 3. 确保 axi_dm IP 存在 (如果不在, source gen 脚本)
# -----------------------------------------------------------------------------
if {[get_ips -quiet axi_dm] eq ""} {
    puts ""
    puts "=== axi_dm IP missing, generating ==="
    source C:/_Project/FLUX_CNN/Syn/vd100_bd/gen_axi_dm_vd100.tcl
}

# -----------------------------------------------------------------------------
# 4. Validate BD
# -----------------------------------------------------------------------------
puts ""
puts "=== Validate BD ==="
open_bd_design [get_files design_1.bd]
if {[catch {validate_bd_design} verr]} {
    puts "  ! BD validate had warnings/errors: $verr"
    puts "  continuing anyway..."
}
save_bd_design

# -----------------------------------------------------------------------------
# 5. Reset + relaunch synth
# -----------------------------------------------------------------------------
puts ""
puts "=== Reset + launch synth_1 ==="
reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1

set status [get_property STATUS [get_runs synth_1]]
puts "  synth_1 status = $status"
if {$status ne "synth_design Complete!"} {
    puts "  ! synth_1 NOT complete, abort flow"
    close_project
    exit 1
}

# Post-synth utilization
open_run synth_1
report_utilization -file C:/_Project/FLUX_CNN/Syn/reports_vd100/utilization_synth_bd.rpt
puts "  written: utilization_synth_bd.rpt"
close_design

# -----------------------------------------------------------------------------
# 6. Launch impl + bitstream
# -----------------------------------------------------------------------------
puts ""
puts "=== Launch impl_1 -to_step write_bitstream ==="
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

set status [get_property STATUS [get_runs impl_1]]
puts "  impl_1 status = $status"
if {![string match "*Complete!*" $status]} {
    puts "  ! impl_1 NOT complete, abort"
    close_project
    exit 1
}

# Post-impl utilization + timing
open_run impl_1
report_utilization -file C:/_Project/FLUX_CNN/Syn/reports_vd100/utilization_routed_bd.rpt
report_timing_summary -file C:/_Project/FLUX_CNN/Syn/reports_vd100/timing_routed_bd.rpt -max_paths 10
puts "  written: utilization_routed_bd.rpt + timing_routed_bd.rpt"

# -----------------------------------------------------------------------------
# 7. Write .xsa (含 bitstream) 给 Vitis 用
# -----------------------------------------------------------------------------
puts ""
puts "=== Write hardware platform .xsa ==="
set XSA "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xsa"
write_hw_platform -fixed -include_bit -force $XSA
puts "  written: $XSA"

# -----------------------------------------------------------------------------
# 8. RESULT SUMMARY
# -----------------------------------------------------------------------------
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts ""
puts "============================================================"
puts " RESULT SUMMARY"
puts "============================================================"
puts "  Part  : [get_property PART [current_project]]"
puts "  Top   : [get_property top [current_fileset]]"
puts "  WNS   : $wns ns"
if {$wns >= 0} {
    set fmax [expr {1000.0 / (10.0 - $wns)}]
    puts "  Fmax  : [format %.1f $fmax] MHz @ 100 MHz target (timing MET)"
} else {
    puts "  Fmax  : timing VIOLATED at 100 MHz"
}
puts "  XSA   : $XSA"
puts "============================================================"

close_design
close_project
puts ""
puts "VD100 full flow done!"
exit 0
