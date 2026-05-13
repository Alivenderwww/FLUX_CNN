open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

# v25: 删 ila_dbg 整个 + 自动清悬空 dbg net.
#   - 释放 ~1.5K LUT + 3 BRAM (ILA storage + state machine)
#   - wrapper dbg_* port driver 没 consumer, Vivado 综合时 prune hier ref signals
#     → 再释放 ConvCore 内 dbg fanout 占用的 ~500-1000 LUT
#   - 诊断改走 cfg_regs ADDR_SEQ_DBG (0x008) RO 寄存器 PC poll
puts "=== v25: 删 ila_dbg + cfg_regs RO 诊断路径 ==="

# 删 ila_dbg (delete_bd_objs 自动断它的所有 net)
catch {delete_bd_objs [get_bd_cells ila_dbg]}
puts "  ila_dbg deleted"

# 删剩余孤立 net (ila_dbg/probeN 的 net 应该已被 delete_bd_objs 清掉)
foreach n [get_bd_nets -quiet] {
    set pins [get_bd_pins -of $n -quiet]
    if {[llength $pins] < 2} {
        catch {delete_bd_objs $n}
    }
}

save_bd_design
generate_target -force all [get_files design_1.bd]

puts "=== reset all runs ==="
reset_runs synth_1
reset_runs impl_1
foreach r [get_runs -filter {NAME =~ "*_synth_*"}] {
    reset_run $r
}

foreach pat {mm2s_arb idma_sg_dispatcher axi_arbiter axi_m_mux multicore_top_vd100_bd sequencer cfg_regs core_top desc_fifo} {
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

puts "===== v25 DONE ====="
exit 0
