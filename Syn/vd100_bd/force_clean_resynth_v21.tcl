open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

# v21: 三项 RTL fix — mm2s_arb 严格 single-outstanding + idma_sg_dispatcher S_DONE 等 sts_drained
#                     + axi_arbiter R lock state machine (+ wrapper bus 端口补 R 通道字段)
# 流程照 v20 (已验证 force_clean.log = write_device_image Complete), 只换 touch RTL 列表.
# 关键: 必须保留 generate_target -force all, 让 IPCACHE 一次性建立完整 cache;
# 否则 launch_runs synth_1 阶段 IPCACHE thread pool (8 threads) 边检查边补 cache
# 触发 race condition → EXCEPTION_ACCESS_VIOLATION in finishWork (v21 前 4 次崩的根因)
puts "=== v22 (in-place rebuild): + dbg_arlen_0 = {fifo.count[3:0], seq.state[3:0]} ==="

# BD 配置不动 (v20 ila_dbg DATA_DEPTH=1024 / PROBE12_WIDTH=2 保留)
# generate_target -force 让 BD 内 IP 全部 (re)generate, IPCACHE 在此阶段稳态写盘
generate_target -force all [get_files design_1.bd]

# 重置 all runs (含 BD 每个 IP 的 *_synth_1)
puts "=== reset all runs ==="
reset_runs synth_1
reset_runs impl_1
foreach r [get_runs -filter {NAME =~ "*_synth_*"}] {
    reset_run $r
}

# touch v21 三项 RTL fix + wrapper (touch 强制 timestamp 更新, 走完整 synth)
puts "=== touch v21 RTL fix files ==="
foreach pat {mm2s_arb idma_sg_dispatcher axi_arbiter axi_m_mux multicore_top_vd100_bd} {
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

puts "===== v21 DONE ====="
exit 0
