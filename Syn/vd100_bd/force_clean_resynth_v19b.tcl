open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

# v19b: 进一步减 ILA 容量 — depth 1024 → 512, probe 30 → 24 (去掉 master_arvalid/arready/rvalid)
puts "=== v19b: shrink ila_dbg ==="
set_property CONFIG.C_DATA_DEPTH {512} [get_bd_cells ila_dbg]

# 去掉 master_arvalid_0/arready_0/rvalid_0 三个 4-bit probe (probe21/22/23)
# 重新配置 NUM_OF_PROBES = 27 (保留前 21 个, 加回最后 6 个 (probe24..29))
# 即: 0-20 保留, 21=start_dfe_pulse_0, 22=dfe_busy_0, 23=dfe_done_0, 24=idma_busy_0, 25=layer_busy_0, 26=csr_aw_fire_0
set_property -dict [list \
  CONFIG.C_NUM_OF_PROBES {27} \
  CONFIG.C_PROBE21_WIDTH {1} \
  CONFIG.C_PROBE22_WIDTH {1} \
  CONFIG.C_PROBE23_WIDTH {1} \
  CONFIG.C_PROBE24_WIDTH {1} \
  CONFIG.C_PROBE25_WIDTH {1} \
  CONFIG.C_PROBE26_WIDTH {1} \
] [get_bd_cells ila_dbg]

# 重连 probes 21-26 到新信号 (因为去掉了 master_*)
# 删旧连接
foreach p [list probe21 probe22 probe23 probe24 probe25 probe26 probe27 probe28 probe29] {
    catch {
        set net [get_bd_intf_nets -of [get_bd_pins ila_dbg/$p] 2>/dev/null]
        if {$net ne ""} { delete_bd_objs $net }
    }
    # net 也可能是 普通 bd net
    catch {
        set nets [get_bd_nets -of_objects [get_bd_pins ila_dbg/$p]]
        foreach n $nets { delete_bd_objs $n }
    }
}

# 重连 probe21..26 (端口名顺序对齐 wrapper 端口)
set probes_remap {
    dbg_start_dfe_pulse_0   21
    dbg_dfe_busy_0          22
    dbg_dfe_done_0          23
    dbg_idma_busy_0         24
    dbg_layer_busy_0        25
    dbg_csr_aw_fire_0       26
}
foreach {sig probe} $probes_remap {
    set src [get_bd_pins multicore_top_vd100_bd_0/$sig]
    set dst [get_bd_pins ila_dbg/probe$probe]
    if {[get_bd_nets -of $dst] eq ""} {
        connect_bd_net $src $dst
        puts "  reconnect probe$probe ← $sig"
    } else {
        puts "  probe$probe already connected"
    }
}

save_bd_design
generate_target -force all [get_files design_1.bd]

puts "=== v19b: Reset + force clean impl_1 only (keep synth_1) ==="
reset_runs impl_1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
puts "impl_1 = [get_property STATUS [get_runs impl_1]]"

open_run impl_1
write_debug_probes -force C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.ltx
write_hw_platform -fixed -include_bit -force C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xsa

puts "===== v19b FORCE CLEAN DONE ====="
exit 0
