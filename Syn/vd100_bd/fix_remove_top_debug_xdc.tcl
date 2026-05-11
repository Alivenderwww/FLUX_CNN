# =============================================================================
# fix_remove_top_debug_xdc.tcl  --  从现有 vd100 工程移除 top_debug.xdc + 重 impl
# 该文件来自 ALINX 02_pl_rw_ddr 模板, 含 u_ila_0 + M_AXI_* 模板 ILA, 跟我们 BD
# 内的 ila_dbg 冲突让 impl 卡在 Generate Debug Cores. 移除即可.
# =============================================================================

open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr

puts "=== Remove top_debug.xdc from constrs_1 ==="
set bad [get_files -of_objects [get_filesets constrs_1] -filter {NAME =~ "*top_debug*"}]
if {$bad ne ""} {
    remove_files -fileset constrs_1 $bad
    puts "  removed: $bad"
} else {
    puts "  not found, skip"
}

# 也物理删除文件防被重 add
catch {file delete -force C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.srcs/constrs_1/new/top_debug.xdc}

# Reset impl_1 (synth_1 Complete 不动)
puts ""
puts "=== Reset impl_1 ==="
reset_run impl_1

# 重 launch
puts ""
puts "=== launch impl_1 -to_step write_bitstream ==="
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
set status [get_property STATUS [get_runs impl_1]]
puts "  impl_1 status = $status"

# 写 .ltx + .xsa
open_run impl_1
set ltx C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.ltx
write_debug_probes -force $ltx
puts "  wrote ltx"

set XSA C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xsa
write_hw_platform -fixed -include_bit -force $XSA

puts ""
puts "===== FIX TOP_DEBUG DONE ====="
close_project
exit 0
