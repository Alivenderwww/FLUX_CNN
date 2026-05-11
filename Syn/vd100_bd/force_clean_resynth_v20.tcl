open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

# v20: ILA depth 1024, probe12 width 2 (恢复 v18 ILA 配置, RTL 路径暴露 sequencer state)
puts "=== v20: restore ila_dbg to v18 config ==="
set_property CONFIG.C_DATA_DEPTH {1024} [get_bd_cells ila_dbg]
set_property CONFIG.C_PROBE12_WIDTH {2} [get_bd_cells ila_dbg]
save_bd_design
generate_target -force all [get_files design_1.bd]

# 删除 OOC sub-design synth cache + 强制 fresh
puts "=== v20: Reset + force clean ==="
reset_runs synth_1
reset_runs impl_1
foreach r [get_runs -filter {NAME =~ "*_synth_*"}] {
    reset_run $r
}

# touch RTL 让 timestamp 更新
puts "=== touch wrapper RTL ==="
foreach pat {axi_arbiter sequencer cfg_regs core_top multicore_top_vd100_bd} {
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

puts "===== v20 FORCE CLEAN DONE ====="
exit 0
