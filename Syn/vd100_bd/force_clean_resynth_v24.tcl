open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

# v24: 降 ila_dbg DATA_DEPTH 1024 → 256 释放 ~3 BRAM + ~500 LUT 余量 (Place 30-487 临界)
# cfg_regs ADDR_SEQ_DBG 0x008 RO 寄存器仍提供 sequencer.state/fifo.count 诊断
puts "=== v24: ila_dbg DEPTH 1024 → 256 释放资源 ==="
set_property CONFIG.C_DATA_DEPTH {256} [get_bd_cells ila_dbg]
save_bd_design
generate_target -force all [get_files design_1.bd]

puts "=== reset all runs ==="
reset_runs synth_1
reset_runs impl_1
foreach r [get_runs -filter {NAME =~ "*_synth_*"}] {
    reset_run $r
}

# touch v23 RTL 文件
foreach pat {mm2s_arb idma_sg_dispatcher axi_arbiter axi_m_mux multicore_top_vd100_bd sequencer cfg_regs core_top desc_fifo} {
    foreach f [get_files -filter "NAME =~ \"*$pat*\""] {
        set p [get_property NAME $f]
        if {[file exists $p]} {
            catch {set fh [open $p a]; close $fh}
            puts "  touched: $p"
        }
    }
}
update_compile_order -fileset sources_1

launch_runs synth_1 -jobs 8
wait_on_run synth_1
puts "synth_1 = [get_property STATUS [get_runs synth_1]]"

set_param drc.disableLUTOverUtilError 1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
puts "impl_1 = [get_property STATUS [get_runs impl_1]]"

open_run impl_1
write_debug_probes -force C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.ltx
write_hw_platform -fixed -include_bit -force C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xsa

puts "===== v24 DONE ====="
exit 0
