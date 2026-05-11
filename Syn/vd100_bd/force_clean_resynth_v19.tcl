open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

# v19: 把 dfe_state probe 宽度 2→4 (现在 dbg_dfe_state_0 接 sequencer.state 4 bit)
puts "=== v19: ila_dbg C_PROBE12_WIDTH 2 → 4 ==="
set_property CONFIG.C_PROBE12_WIDTH {4} [get_bd_cells ila_dbg]
set_property CONFIG.C_DATA_DEPTH {1024} [get_bd_cells ila_dbg]
save_bd_design
generate_target -force all [get_files design_1.bd]

# 删除 OOC sub-design synth cache + 强制 fresh
puts "=== v19: Reset + force clean ==="
reset_runs synth_1
reset_runs impl_1
foreach r [get_runs -filter {NAME =~ "*_synth_*"}] {
    puts "  reset OOC: $r"
    reset_run $r
}

# touch 关键 RTL 让 timestamp 更新 (force fresh OOC synth)
puts "=== touch wrapper RTL ==="
foreach pat {axi_arbiter sequencer multicore_top_vd100_bd} {
    foreach f [get_files -filter "NAME =~ \"*$pat*\""] {
        set p [get_property NAME $f]
        if {[file exists $p]} {
            catch {set fh [open $p a]; close $fh}
            puts "  touched: $p"
        }
    }
}
update_compile_order -fileset sources_1

# 强制 fresh synth
launch_runs synth_1 -jobs 8
wait_on_run synth_1
puts "synth_1 = [get_property STATUS [get_runs synth_1]]"

# 关 LUT over-utilization DRC
set_param drc.disableLUTOverUtilError 1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
puts "impl_1 = [get_property STATUS [get_runs impl_1]]"

open_run impl_1
write_debug_probes -force C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.ltx
write_hw_platform -fixed -include_bit -force C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xsa

puts "===== v19 FORCE CLEAN DONE ====="
exit 0
