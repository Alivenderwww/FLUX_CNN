# =============================================================================
# extract_routed.tcl  --  从 routed_dcp 提取 utilization + timing (不需要 bitstream)
# 用 evaluation license 也能跑 (只要不 write_device_image).
# =============================================================================

set DCP "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper_routed.dcp"

if {![file exists $DCP]} {
    puts "ERROR: routed_dcp not found: $DCP"
    exit 1
}

puts "Opening routed checkpoint..."
open_checkpoint $DCP

puts ""
puts "=== Utilization (post-route) ==="
report_utilization -file C:/_Project/FLUX_CNN/Syn/reports_vd100/utilization_routed_bd.rpt -hierarchical -hierarchical_depth 3
report_utilization

puts ""
puts "=== Timing (post-route) ==="
report_timing_summary -file C:/_Project/FLUX_CNN/Syn/reports_vd100/timing_routed_bd.rpt -max_paths 10
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]

puts ""
puts "============================================================"
puts " ROUTED RESULT"
puts "============================================================"
puts "  WNS  : $wns ns"
if {$wns >= 0} {
    set fmax [expr {1000.0 / (10.0 - $wns)}]
    puts "  Fmax : [format %.1f $fmax] MHz @ 100 MHz target (timing MET)"
} else {
    puts "  Fmax : timing VIOLATED at 100 MHz"
}
puts "============================================================"

close_design
exit 0
