# =============================================================================
# run_synth.tcl  --  vd100_minimal 综合 + impl + bitstream
# =============================================================================

open_project C:/_Project/FLUX_CNN/Syn/vd100_minimal/output/vd100_minimal.xpr

puts "============================================================"
puts " vd100_minimal: launch synth_1 + impl_1 + write_device_image"
puts "============================================================"

reset_runs synth_1
reset_runs impl_1
foreach r [get_runs -filter {NAME =~ "*_synth_*"}] {
    reset_run $r
}

update_compile_order -fileset sources_1

launch_runs synth_1 -jobs 8
wait_on_run synth_1
puts "synth_1 = [get_property STATUS [get_runs synth_1]]"

set_param drc.disableLUTOverUtilError 1
launch_runs impl_1 -to_step write_device_image -jobs 8
wait_on_run impl_1
puts "impl_1 = [get_property STATUS [get_runs impl_1]]"

open_run impl_1
write_hw_platform -fixed -include_bit -force \
    C:/_Project/FLUX_CNN/Syn/vd100_minimal/output/vd100_minimal.xsa

puts "============================================================"
puts " DONE. PDI at:"
puts "   C:/_Project/FLUX_CNN/Syn/vd100_minimal/output/vd100_minimal.runs/impl_1/design_1_wrapper.pdi"
puts "============================================================"
exit 0
