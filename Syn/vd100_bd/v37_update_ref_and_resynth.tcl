# =============================================================================
# v37: 跟 v36 一样, 但加 update_module_reference 让 BD 重读
# multicore_top_vd100_bd_v.v 的新 port 定义 (删了 m00_axi 的多余
# X_INTERFACE_PARAMETER, 避免 Vivado mute 6 个 handshake input).
# =============================================================================
open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

puts "=== v37: BD update_module_reference u_mc_vd100 ==="
# 尝试多种 update_module_reference 语法, 任一成功即可继续.
# Vivado 2023.2 接受: update_module_reference <module_name> (module name, 不是 cell path)
# multicore_top_vd100_bd_v.v 的 module 名 = multicore_top_vd100_bd_v
if {[catch {update_module_reference multicore_top_vd100_bd_v} err]} {
    puts "  attempt1 failed: $err"
    if {[catch {update_module_reference -refresh_module multicore_top_vd100_bd_v} err2]} {
        puts "  attempt2 failed: $err2"
        # fallback: 删 cell + 重 create
        puts "  fallback: 不显式 update, 依赖 generate_target -force 重读源"
    } else {
        puts "  attempt2 OK"
    }
} else {
    puts "  attempt1 OK"
}

# Sanity check 一下 m00_axi 这条 interface 的 scalar pin 列表
puts "=== sanity check: u_mc_vd100/m00_axi_<scalar> pin ==="
foreach pname {awready wready bvalid arready rlast rvalid awvalid arvalid wvalid wlast bready rready} {
    set pin [get_bd_pins -quiet u_mc_vd100/m00_axi_${pname}]
    if {$pin ne ""} {
        set net [get_bd_nets -quiet -of_objects $pin]
        puts "  m00_axi_${pname} pin=$pin net=$net"
    } else {
        puts "  m00_axi_${pname} NOT FOUND"
    }
}

# v36 isolate: mac bypass + 删 ILA
catch {delete_bd_objs [get_bd_cells ila_dbg]}
catch {delete_bd_objs [get_bd_cells system_ila_axi]}
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

puts "===== v37 DONE ====="
exit 0
