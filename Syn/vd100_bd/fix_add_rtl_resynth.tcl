open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr

# 重新 add RTL files (源 add_rtl.tcl)
puts "=== Re-add RTL files ==="
source C:/_Project/FLUX_CNN/Syn/vd100_bd/add_rtl.tcl

# 验证 sources_1 现在有 multicore_top_vd100*
puts ""
puts "=== Verify sources_1 has vd100 RTL ==="
foreach f [get_files -of_objects [get_filesets sources_1]] {
    if {[string match "*vd100*" $f]} { puts "  $f" }
}

# 重新 reset + 综合
update_compile_order -fileset sources_1
reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1
puts "  synth_1 status = [get_property STATUS [get_runs synth_1]]"

launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
puts "  impl_1 status = [get_property STATUS [get_runs impl_1]]"

# 检查 axi_dm 个数
open_run impl_1
puts ""
puts "=== axi_dm count ==="
puts "  [llength [get_cells -hier -filter {REF_NAME =~ *axi_dm*}]] axi_dm cells"

set ltx C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.ltx
write_debug_probes -force $ltx
puts "  wrote ltx"

set XSA "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xsa"
write_hw_platform -fixed -include_bit -force $XSA

puts ""
puts "===== READD RTL DONE ====="
close_project
exit 0
