open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

# v33: BD axi_noc S00..S02_AXI 加 master capability declaration (跟 ALINX demo 02_pl_rw_ddr 对齐).
# Hypothesis: axi_noc 期望 multi-thread multi-outstanding master, axi_dm IP 实际 single-thread
# (ARID/AWID=0 same ID 多 cmd), axi_noc 路由 R/B 错位 → axi_dm W 卡 → ODMA S_TX 卡.
# Demo S_AXI port set: NUM_WRITE_OUTSTANDING=1, NUM_WRITE_THREADS=1, NUM_READ_OUTSTANDING=1,
#                       NUM_READ_THREADS=1, ID_WIDTH=2, READ_WRITE_MODE=READ_WRITE.
# v32 沿用: dispatcher 4KB-split + IDMA r_done 不依 sts + mm2s_arb 去 sts gate + ODMA skip S_STS.
puts "=== v33: axi_noc S00..S02_AXI 加 master capability (single-outstanding/thread, ID_WIDTH=2) ==="

for {set i 0} {$i < 3} {incr i} {
    set port [format "S%02d_AXI" $i]
    set pin [get_bd_intf_pins /axi_noc_0/$port]
    if {$pin eq ""} {
        puts "  WARN: $port not found, skip"
        continue
    }
    set_property -dict [list \
        CONFIG.NUM_WRITE_OUTSTANDING {1} \
        CONFIG.NUM_WRITE_THREADS     {1} \
        CONFIG.NUM_READ_OUTSTANDING  {1} \
        CONFIG.NUM_READ_THREADS      {1} \
        CONFIG.READ_WRITE_MODE       {READ_WRITE} \
        CONFIG.ID_WIDTH              {2} \
        CONFIG.MAX_BURST_LENGTH      {256} \
        CONFIG.SUPPORTS_NARROW_BURST {1} \
        CONFIG.HAS_BURST             {1} \
        CONFIG.HAS_LOCK              {1} \
        CONFIG.HAS_CACHE             {1} \
        CONFIG.HAS_PROT              {1} \
        CONFIG.HAS_QOS               {1} \
        CONFIG.HAS_WSTRB             {1} \
        CONFIG.HAS_BRESP             {1} \
        CONFIG.HAS_RRESP             {1} \
    ] $pin
    puts "  set $port master capability (single-outstanding/thread, ID_WIDTH=2)"
}

save_bd_design
generate_target -force all [get_files design_1.bd]

puts "=== reset all runs ==="
reset_runs synth_1
reset_runs impl_1
foreach r [get_runs -filter {NAME =~ "*_synth_*"}] {
    reset_run $r
}

# touch RTL 让 IP re-elab catch up
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

puts "===== v33 DONE ====="
exit 0
