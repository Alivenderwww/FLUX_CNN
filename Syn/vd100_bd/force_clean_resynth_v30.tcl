open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

# v30: mm2s_arb 去 sts_empty/sts_full gate (axi_dm 板上偶丢 sts 让 sts_cnt 累积 → cmd_tvalid 卡).
# + 沿用 v29: IDMA dispatcher 4KB-split, IDMA r_done 不等 sts, ODMA streaming_all_done 用 data tlast.
puts "=== v30: mm2s_arb 去 sts gate + v29 4KB/no-sts fix ==="

puts "=== reset all runs ==="
reset_runs synth_1
reset_runs impl_1
foreach r [get_runs -filter {NAME =~ "*_synth_*"}] {
    reset_run $r
}

foreach pat {mm2s_arb idma_sg_dispatcher odma_sg_dispatcher axi_arbiter axi_m_mux multicore_top_vd100_bd sequencer cfg_regs core_top desc_fifo} {
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

puts "===== v30 DONE ====="
exit 0
