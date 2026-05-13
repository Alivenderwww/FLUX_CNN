open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

# v36 isolate: mac_pe 乘法 bypass (FLUX_MAC_BYPASS define) 省 768 DSP + 加速 P&R.
# 目的: 隔离测试 ConvCore axi 流水线是否独立 stuck (跟 mac_array 计算无关).
# 实际 layer 0 卡 ODMA S_TX, cmd 序列 + AW address pattern 跟原 design 一致. mac 输出
# garbage 不影响 board stuck 测试.
puts "=== v36 isolate: FLUX_MAC_BYPASS + 删 ILA + axi_arbiter 简化 ==="

# 删 ILA (释放 LUT + 不触发 LUT3 trim cascade)
catch {delete_bd_objs [get_bd_cells ila_dbg]}
catch {delete_bd_objs [get_bd_cells system_ila_axi]}

# 加 verilog_define FLUX_MAC_BYPASS 到 sources_1 fileset
set_property verilog_define {FLUX_MAC_BYPASS} [current_fileset]
puts "  verilog_define FLUX_MAC_BYPASS set"

save_bd_design
generate_target -force all [get_files design_1.bd]

puts "=== reset all runs ==="
reset_runs synth_1
reset_runs impl_1
foreach r [get_runs -filter {NAME =~ "*_synth_*"}] {
    reset_run $r
}

foreach pat {mm2s_arb idma_sg_dispatcher odma_sg_dispatcher axi_arbiter axi_m_mux multicore_top_vd100_bd sequencer cfg_regs core_top mac_pe mac_array} {
    foreach f [get_files -filter "NAME =~ \"*$pat*\""] {
        set p [get_property NAME $f]
        if {[file exists $p]} {
            catch {set fh [open $p a]; close $fh}
        }
    }
}
update_compile_order -fileset sources_1

launch_runs synth_1 -jobs 8
wait_on_run synth_1
puts "synth_1 = [get_property STATUS [get_runs synth_1]]"

# 看 DSP 用量 (应该接近 0)
catch {
    open_run synth_1 -name synth_1
    set rpt [report_utilization -return_string]
    foreach line [split $rpt "\n"] {
        if {[string match "*DSP*" $line]} { puts "  $line" }
    }
}

set_param drc.disableLUTOverUtilError 1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
puts "impl_1 = [get_property STATUS [get_runs impl_1]]"

open_run impl_1
write_debug_probes -force C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.ltx
write_hw_platform -fixed -include_bit -force C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xsa

puts "===== v36 DONE ====="
exit 0
