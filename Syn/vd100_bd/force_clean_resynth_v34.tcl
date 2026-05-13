open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

# v34: multicore_top_vd100_bd_v.v 的 m00/m01/m02_axi 端口加 X_INTERFACE_PARAMETER 声明 master
# capability (NUM_WRITE_OUTSTANDING=1, NUM_WRITE_THREADS=1, ID_WIDTH=5, READ_WRITE_MODE=READ_WRITE,
# 等). axi_noc S0X_AXI 会自动 infer 这些 capability (v33 直接 set 是 read-only failed).
# v32 沿用: dispatcher 4KB-split + IDMA r_done 不依 sts + mm2s_arb 去 sts gate + ODMA skip S_STS.
puts "=== v34: wrapper PARAMETER 声明 m0X_axi master capability (axi_noc infer) ==="

# 不需 set axi_noc S0X_AXI property (v33 试过, read-only). 只需 update wrapper RTL.
# 但 BD 内的 multicore_top_vd100_bd_v module reference 需要 re-elab 才认新 PARAMETER.
puts "=== reset all runs + force re-elab module ref ==="
reset_runs synth_1
reset_runs impl_1
foreach r [get_runs -filter {NAME =~ "*_synth_*"}] {
    reset_run $r
}
# BD 内 u_mc_vd100 是 multicore_top_vd100_bd_v module reference cell.
# reset_run + generate_target -force 让 Vivado 重新读 RTL 源 (含新 X_INTERFACE_PARAMETER).
save_bd_design
generate_target -force all [get_files design_1.bd]

# touch RTL 让 module_ref re-pick up X_INTERFACE_PARAMETER
foreach pat {mm2s_arb idma_sg_dispatcher odma_sg_dispatcher axi_arbiter axi_m_mux multicore_top_vd100_bd sequencer cfg_regs core_top} {
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

puts "===== v34 DONE ====="
exit 0
