# =============================================================================
# full_resynth_after_rtl_fix.tcl  --  RTL 改了 (axi_arbiter), reset synth_1 +
#   impl_1 + write bitstream + .ltx + .xsa.
# =============================================================================
open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
update_compile_order -fileset sources_1

reset_run impl_1
reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1
puts "synth_1 status = [get_property STATUS [get_runs synth_1]]"

launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
puts "impl_1 status = [get_property STATUS [get_runs impl_1]]"

open_run impl_1
write_debug_probes -force C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.ltx
write_hw_platform -fixed -include_bit -force C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xsa

puts "===== RESYNTH DONE ====="
close_project
exit 0
