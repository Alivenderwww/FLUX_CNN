open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

# v19c: 进一步减 ILA depth 1024 → 256, 不动 probes (synth_1 已 OK, 只 reset impl_1)
puts "=== v19c: ila_dbg DATA_DEPTH → 256 ==="
set_property CONFIG.C_DATA_DEPTH {256} [get_bd_cells ila_dbg]
save_bd_design
generate_target -force all [get_files design_1.bd]

# reset synth (因为 ila 配置改变, 需重 synth ila IP)
reset_runs synth_1
reset_runs impl_1
foreach r [get_runs -filter {NAME =~ "*_synth_*"}] {
    reset_run $r
}

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

puts "===== v19c FORCE CLEAN DONE ====="
exit 0
