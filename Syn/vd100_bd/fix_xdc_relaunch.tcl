# =============================================================================
# fix_xdc_relaunch.tcl  --  加 ALINX DDR4 xdc 进工程, 重 launch impl
#
# Synth 已 PASS, 但 impl 报 DDR4 IO unplaced. 因 ALINX xdc 没拷进 constrs_1.
# 这个脚本: open + add xdc + launch impl + bitstream + xsa.
# =============================================================================

set PROJ "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr"
set ALINX_XDC_DIR "C:/_Project/Bishe/VD100/VD100_9_16/demo/course_s1/02_pl_rw_ddr/auto_create_project/src/constraints"

open_project $PROJ

# ----- 加 ALINX 提供的 ddr4.xdc + system.xdc 到 constrs_1 -----
puts "=== Adding ALINX xdc to constrs_1 ==="
foreach xdc [list "ddr4.xdc" "system.xdc"] {
    set src "$ALINX_XDC_DIR/$xdc"
    if {[file exists $src]} {
        # 拷到工程 constrs_1/new/, 然后 add
        set dst "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.srcs/constrs_1/new/$xdc"
        file mkdir [file dirname $dst]
        file copy -force $src $dst
        if {[get_files -quiet $xdc] eq ""} {
            add_files -fileset constrs_1 $dst
            set_property USED_IN {synthesis implementation} [get_files $xdc]
            puts "  + added $xdc"
        } else {
            puts "  - $xdc already in fileset, skip"
        }
    } else {
        puts "  ! $src not found"
    }
}

# top_debug.xdc 是 mark_debug 用, 当前不需要
# (保留 ddr4.xdc + system.xdc 即可)

update_compile_order -fileset sources_1

# ----- Synth 已 PASS, 不重跑. 强制 reset impl 重跑 -----
# 注: xdc 加到 USED_IN {synthesis implementation}, 严格说会让 synth out of date.
# 但 Vivado 实际只在 impl 阶段读 PACKAGE_PIN, 我们 set USED_IN 只 implementation
# 让 synth 保持 valid.
set_property USED_IN {implementation} [get_files ddr4.xdc]
set_property USED_IN {implementation} [get_files system.xdc]

puts ""
puts "=== Reset + launch impl_1 (synth_1 reuse cached) ==="
reset_run impl_1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

set status [get_property STATUS [get_runs impl_1]]
puts "  impl_1 status = $status"
if {![string match "*Complete!*" $status]} {
    puts "  ! impl_1 NOT complete, abort"
    close_project
    exit 1
}

# ----- Reports + xsa -----
open_run impl_1
report_utilization -file C:/_Project/FLUX_CNN/Syn/reports_vd100/utilization_routed_bd.rpt
report_timing_summary -file C:/_Project/FLUX_CNN/Syn/reports_vd100/timing_routed_bd.rpt -max_paths 10

set XSA "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xsa"
write_hw_platform -fixed -include_bit -force $XSA

set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts ""
puts "============================================================"
puts " RESULT SUMMARY"
puts "============================================================"
puts "  WNS  : $wns ns"
if {$wns >= 0} {
    set fmax [expr {1000.0 / (10.0 - $wns)}]
    puts "  Fmax : [format %.1f $fmax] MHz @ 100 MHz target (timing MET)"
} else {
    puts "  Fmax : timing VIOLATED at 100 MHz"
}
puts "  XSA  : $XSA"
puts "============================================================"

close_design
close_project
exit 0
