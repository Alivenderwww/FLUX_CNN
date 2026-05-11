# =============================================================================
# patch_add_ila.tcl  --  在 BD 上加 system_ila 抓 PL master AXI 信号
#
# ILA 抓点: smartconnect_pl_0.M00_AXI (即真喂给 axi_noc.S00_AXI 的信号).
# 这能定位 ConvCore -> smartconnect -> axi_noc 三段中, axi_noc 是否真接收 transaction.
#
# 信号: AXI4 全套 (aw/w/b/ar/r), 加 awid/arid/awaddr/araddr/awlen 等关键 control.
# 抓到 arvalid 拉了但 arready 不来, 就是 axi_noc 锁死的证据.
# 抓到 arvalid 始终为 0, 就是 ConvCore axi master 没真出 transaction.
#
# 用法: vivado -mode batch -source patch_add_ila.tcl
#
# 综合后烧 PDI, 用 Vivado HW Manager open hw_target → 看到 ILA → run trigger
# (设 trigger = arvalid==1 OR awvalid==1) → 拖 waveform 看时序.
# =============================================================================

open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr

# 强制 Vivado 重读 RTL (multicore_top_vd100.sv + multicore_top_vd100_bd.sv 改了)
update_compile_order -fileset sources_1
puts "=== Refresh sources OK ==="

open_bd_design [get_files design_1.bd]

puts "=== Manually instantiate axis_ila:1.2 with probes ==="
# 删除之前可能存在的 ila_pl0
catch {delete_bd_objs [get_bd_cells -quiet ila_pl0]}
catch {delete_bd_objs [get_bd_cells -quiet system_ila_pl0]}

# 用 axis_ila:1.2 standalone (probes 模式), Versal 兼容
set ila [create_bd_cell -type ip -vlnv xilinx.com:ip:axis_ila:1.2 ila_pl0]
set_property -dict [list \
    CONFIG.C_NUM_OF_PROBES {10} \
    CONFIG.C_DATA_DEPTH {2048} \
    CONFIG.C_INPUT_PIPE_STAGES {1} \
    CONFIG.C_TRIGOUT_EN {false} \
    CONFIG.C_TRIGIN_EN {false} \
    CONFIG.C_PROBE0_WIDTH {1}  \
    CONFIG.C_PROBE1_WIDTH {1}  \
    CONFIG.C_PROBE2_WIDTH {1}  \
    CONFIG.C_PROBE3_WIDTH {1}  \
    CONFIG.C_PROBE4_WIDTH {1}  \
    CONFIG.C_PROBE5_WIDTH {1}  \
    CONFIG.C_PROBE6_WIDTH {32} \
    CONFIG.C_PROBE7_WIDTH {32} \
    CONFIG.C_PROBE8_WIDTH {5}  \
    CONFIG.C_PROBE9_WIDTH {5}  \
] $ila

# clk + reset
connect_bd_net [get_bd_pins clk_wizard_0/clk_out1] [get_bd_pins ila_pl0/clk]
connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] \
               [get_bd_pins ila_pl0/resetn]

# probes 接 smartconnect_pl_0.M00_AXI 信号 (用 BD interface pin bit-select)
# 但 BD intf_pin 不能直接 bit-select; 用 xlslice 或 直接接 net (intf_net 内单 bit signal)
puts "=== Wire probes to smartconnect_pl_0/M00_AXI signals ==="

# 创建 xlslice 不可行因为 intf 是 pack 信号. 改用 直接拍 RTL signal.
# axis_ila 的每个 probe 都是 input pin, 接 BD net.
# 用 hdl_attribute 的方式 mark 也可, 但这次手动接.

# 简单方案: 把 smartconnect_pl_0.M00_AXI 的 awvalid/awready 等 break out 后接 probe.
# 但 BD 内部 intf_net 的子信号没暴露 pin. 用 set_property MARK_DEBUG on net 不可行 (BD 内部)
#
# 改用 Tcl 创建独立 wire pin 然后 net.
# 实际上最简方案: 在 multicore_top_vd100_bd.sv 内手动 wire 关键信号到 debug_* 输出端口,
# BD wrapper 接 axis_ila probes. 但这要改 wrapper.
#
# 既然 mark_debug 综合层面没生成 ltx, 我尝试更原生的: 给 net 加 NEEDS_KEEP 让综合保留,
# 然后在 implementation 阶段手动加 debug core. 太复杂.
#
# 直接在 multicore_top_vd100_bd_v.v 加 debug 信号输出端口, BD wrapper 接 probe.
# 但这要改 BD wrapper RTL + 重生成 BD module reference.
#
# 简化: 直接在 BD 上用 mark_debug + implement_debug_core 显式跑 (上次没跑).
puts "  fallback: re-mark_debug + implement_debug_core in synth"
delete_bd_objs $ila

set debug_net [get_bd_intf_nets -of_objects \
                  [get_bd_intf_pins smartconnect_pl_0/M00_AXI]]
puts "  intf_net: $debug_net"
set_property HDL_ATTRIBUTE.DEBUG true $debug_net
puts "  marked DEBUG=true"

# Validate + save
if {[catch {validate_bd_design} verr]} {
    puts "  ! BD validate had warnings: $verr"
}
save_bd_design

# regen + reset_run + relaunch
generate_target all [get_files design_1.bd]
reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1
set status [get_property STATUS [get_runs synth_1]]
puts "  synth_1 status = $status"
if {$status ne "synth_design Complete!"} { close_project; exit 1 }

launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
puts "  impl_1 status = [get_property STATUS [get_runs impl_1]]"

set XSA "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xsa"
write_hw_platform -fixed -include_bit -force $XSA
puts "  written: $XSA"

puts ""
puts "===== ILA PATCH DONE ====="
puts "下一步: bootgen + program PDI, 然后 Vivado HW Manager 抓 ILA waveform"
close_project
exit 0
