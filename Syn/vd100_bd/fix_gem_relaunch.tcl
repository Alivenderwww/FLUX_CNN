# =============================================================================
# fix_gem_relaunch.tcl  --  启用 PS GEM0 (ETH0), 重综合到 .xsa
#
# 跟 ALINX VD100 ETH0 配置一致:
#   PS_ENET0_PERIPHERAL = {ENABLE 1, IO PS_MIO 0..11}    (RGMII)
#   PS_ENET0_MDIO       = {ENABLE 1, IO PMC_MIO 50..51}  (MDIO)
#
# 用法:
#   "D:\Xilinx\Vivado\2023.2\bin\vivado.bat" -mode batch -source Syn\vd100_bd\fix_gem_relaunch.tcl -nojournal -log Syn\vd100_bd\fix_gem.log
# =============================================================================

set PROJ "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr"

open_project $PROJ
open_bd_design [get_files design_1.bd]

puts ""
puts "=== Enable PS GEM0 (ALINX VD100 ETH0 RGMII) ==="
set_property -dict [list \
    CONFIG.PS_PMC_CONFIG.PS_ENET0_PERIPHERAL {{ENABLE 1} {IO {PS_MIO 0 .. 11}}} \
    CONFIG.PS_PMC_CONFIG.PS_ENET0_MDIO       {{ENABLE 1} {IO {PMC_MIO 50 .. 51}}} \
] [get_bd_cells versal_cips_0]
puts "  + PS_ENET0_PERIPHERAL enabled (PS_MIO 0..11 RGMII)"
puts "  + PS_ENET0_MDIO       enabled (PMC_MIO 50..51)"

puts ""
puts "=== Validate BD ==="
if {[catch {validate_bd_design} verr]} {
    puts "  ! BD validate had warnings: $verr"
}
save_bd_design

puts ""
puts "=== Regenerate BD wrapper ==="
make_wrapper -files [get_files design_1.bd] -top -import -force
update_compile_order -fileset sources_1
set_property top design_1_wrapper [current_fileset]
set_property top_auto_set false [current_fileset]

puts ""
puts "=== Reset + launch synth_1 (versal_cips changed, full resynth needed) ==="
reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1

set status [get_property STATUS [get_runs synth_1]]
puts "  synth_1 status = $status"
if {$status ne "synth_design Complete!"} {
    puts "  ! synth_1 failed, abort"
    close_project
    exit 1
}

puts ""
puts "=== launch impl_1 -to_step write_bitstream ==="
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

set status [get_property STATUS [get_runs impl_1]]
puts "  impl_1 status = $status"
if {![string match "*Complete!*" $status]} {
    puts "  ! impl_1 failed, abort"
    close_project
    exit 1
}

puts ""
puts "=== Reports ==="
open_run impl_1
report_utilization -file C:/_Project/FLUX_CNN/Syn/reports_vd100/utilization_routed_gem.rpt
report_timing_summary -file C:/_Project/FLUX_CNN/Syn/reports_vd100/timing_routed_gem.rpt -max_paths 10

set XSA "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xsa"
write_hw_platform -fixed -include_bit -force $XSA
puts "  + .xsa: $XSA"

set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts ""
puts "============================================================"
puts " FIX GEM DONE"
puts "  WNS  : $wns ns"
if {$wns >= 0} {
    set fmax [expr {1000.0 / (10.0 - $wns)}]
    puts "  Fmax : [format %.1f $fmax] MHz"
}
puts "  XSA  : $XSA"
puts "============================================================"

close_design
close_project
exit 0
