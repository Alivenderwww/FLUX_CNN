# =============================================================================
# patch_inline_ila.tcl  --  内联式 ILA: BD wrapper 加 dbg_* 输出端口, BD 加
#                            axis_ila + xlconcat 接 probes.
#
# RTL 改动 (已做):
#   - multicore_top_vd100_bd.sv: 加 dbg_awvalid_0/.../dbg_arlen_0 expose iw_*[0]
#   - multicore_top_vd100_bd_v.v: 加同名端口透传
#
# BD 改动 (本脚本):
#   - 删除可能存在的旧 ila_pl0 / system_ila
#   - 加 axis_ila:1.2 with 12 probes
#   - 接 wrapper 的 dbg_* 输出到 ILA probes
# =============================================================================

open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
update_compile_order -fileset sources_1
puts "=== Refresh sources OK ==="

open_bd_design [get_files design_1.bd]

# Force BD module-ref refresh: 用 update_module_reference 是 Vivado 标准命令,
# 但有时不刷新; 加 close/open BD + delete + recreate u_mc_vd100 cell 全套强制.
puts "=== Force refresh module reference (delete/recreate u_mc_vd100) ==="
if {[catch {
    # 先记下连接关系 (intf_net + net), 后面恢复
    set u_axil_net  [get_bd_intf_nets -quiet -of_objects [get_bd_intf_pins u_mc_vd100/csr_axil]]
    set u_irq_net   [get_bd_nets       -quiet -of_objects [get_bd_pins      u_mc_vd100/irq_done]]
    set u_clk_net   [get_bd_nets       -quiet -of_objects [get_bd_pins      u_mc_vd100/clk]]
    set u_rst_net   [get_bd_nets       -quiet -of_objects [get_bd_pins      u_mc_vd100/rst_n]]
    set m00_net     [get_bd_intf_nets  -quiet -of_objects [get_bd_intf_pins u_mc_vd100/m00_axi]]
    set m01_net     [get_bd_intf_nets  -quiet -of_objects [get_bd_intf_pins u_mc_vd100/m01_axi]]
    set m02_net     [get_bd_intf_nets  -quiet -of_objects [get_bd_intf_pins u_mc_vd100/m02_axi]]
    puts "  csr_axil net: $u_axil_net"
    puts "  m00_axi net:  $m00_net"

    # Delete cell. 端口上的 net 自动断开但 net 本身保留(连到对端).
    delete_bd_objs [get_bd_cells u_mc_vd100]

    # update_module_reference 重读 .v wrapper 端口
    update_module_reference multicore_top_vd100_bd_v

    # Recreate u_mc_vd100, BD 会自动跟 .v wrapper 新端口对齐 (含 dbg_*)
    create_bd_cell -type module -reference multicore_top_vd100_bd_v u_mc_vd100

    # 重接基础连线 (复用残留 nets)
    if {$u_clk_net  ne ""} { connect_bd_net  -net $u_clk_net  [get_bd_pins u_mc_vd100/clk] }
    if {$u_rst_net  ne ""} { connect_bd_net  -net $u_rst_net  [get_bd_pins u_mc_vd100/rst_n] }
    if {$u_axil_net ne ""} { connect_bd_intf_net -net $u_axil_net [get_bd_intf_pins u_mc_vd100/csr_axil] }
    if {$u_irq_net  ne ""} { connect_bd_net  -net $u_irq_net  [get_bd_pins u_mc_vd100/irq_done] }
    if {$m00_net    ne ""} { connect_bd_intf_net -net $m00_net [get_bd_intf_pins u_mc_vd100/m00_axi] }
    if {$m01_net    ne ""} { connect_bd_intf_net -net $m01_net [get_bd_intf_pins u_mc_vd100/m01_axi] }
    if {$m02_net    ne ""} { connect_bd_intf_net -net $m02_net [get_bd_intf_pins u_mc_vd100/m02_axi] }

    # 重新 assign address (CSR + 3 PL master)
    assign_bd_address -offset 0xA4000000 -range 0x4000 \
        -target_address_space [get_bd_addr_spaces versal_cips_0/M_AXI_FPD] \
        [get_bd_addr_segs u_mc_vd100/csr_axil/Reg] -force
    for {set i 0} {$i < 3} {incr i} {
        set port [format "S%02d_AXI" $i]
        catch {
            assign_bd_address -offset 0x00000000 -range 0x80000000 \
                -target_address_space [get_bd_addr_spaces u_mc_vd100/$port] \
                [get_bd_addr_segs axi_noc_0/${port}/C0_DDR_LOW0] -force
        }
    }
    puts "  u_mc_vd100 recreated, dbg_* pins should now exist"
} err]} {
    puts "  WARN refresh err: $err"
}

# 0. 清理旧的 ILA cells (所有可能名字)
puts "=== Cleanup old ILA cells ==="
foreach n {ila_pl0 system_ila_pl0 ila_dbg ila_test} {
    catch {delete_bd_objs [get_bd_cells -quiet $n]}
}
foreach n [get_bd_intf_nets -quiet -filter {HDL_ATTRIBUTE.DEBUG == 1}] {
    set_property HDL_ATTRIBUTE.DEBUG false $n
    puts "  cleared mark_debug on: $n"
}

# 1. 创建 axis_ila with 12 probes
puts "=== Create axis_ila:1.2 with 12 probes ==="
set ila [create_bd_cell -type ip -vlnv xilinx.com:ip:axis_ila:1.2 ila_dbg]
set_property -dict [list \
    CONFIG.C_NUM_OF_PROBES {12} \
    CONFIG.C_DATA_DEPTH {2048} \
    CONFIG.C_INPUT_PIPE_STAGES {1} \
    CONFIG.C_TRIGOUT_EN {false} \
    CONFIG.C_TRIGIN_EN {false} \
    CONFIG.C_PROBE0_WIDTH {1}  CONFIG.C_PROBE1_WIDTH {1} \
    CONFIG.C_PROBE2_WIDTH {1}  CONFIG.C_PROBE3_WIDTH {1} \
    CONFIG.C_PROBE4_WIDTH {1}  CONFIG.C_PROBE5_WIDTH {1} \
    CONFIG.C_PROBE6_WIDTH {32} CONFIG.C_PROBE7_WIDTH {32} \
    CONFIG.C_PROBE8_WIDTH {4}  CONFIG.C_PROBE9_WIDTH {4} \
    CONFIG.C_PROBE10_WIDTH {8} CONFIG.C_PROBE11_WIDTH {8} \
] $ila

# 2. clk (axis_ila 不需要 resetn)
connect_bd_net [get_bd_pins clk_wizard_0/clk_out1] [get_bd_pins ila_dbg/clk]

# 3. 接 wrapper dbg_* 输出到 ila probes
puts "=== Wire dbg_* to ILA probes ==="
set probes {
    awvalid_0 0  awready_0 1  arvalid_0 2  arready_0 3
    wvalid_0  4  rvalid_0  5  awaddr_0  6  araddr_0  7
    awid_0    8  arid_0    9  awlen_0  10  arlen_0  11
}
foreach {sig probe} $probes {
    set wpin [get_bd_pins -quiet u_mc_vd100/dbg_$sig]
    if {$wpin eq ""} {
        puts "  WARN: u_mc_vd100/dbg_$sig not found"
        continue
    }
    set ipin [get_bd_pins ila_dbg/probe$probe]
    connect_bd_net $wpin $ipin
    puts "  dbg_$sig → probe$probe"
}

# 4. validate + save
if {[catch {validate_bd_design} verr]} {
    puts "  ! BD validate had warnings: $verr"
}
save_bd_design

# 5. regen + 重综合 (delete OOC sub-design synth cache 强制 fresh re-synth)
generate_target all [get_files design_1.bd]
foreach run [get_runs -filter {NAME =~ "*_synth_*"}] {
    puts "  resetting $run"
    reset_run $run
}
reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1
set status [get_property STATUS [get_runs synth_1]]
puts "  synth_1 status = $status"
if {$status ne "synth_design Complete!"} { close_project; exit 1 }

launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
puts "  impl_1 status = [get_property STATUS [get_runs impl_1]]"

# 6. 生成 .ltx
open_run impl_1
set ltx C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.ltx
write_debug_probes -force $ltx
puts "  wrote ltx: $ltx"

set XSA "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xsa"
write_hw_platform -fixed -include_bit -force $XSA
puts "  written: $XSA"

puts ""
puts "===== INLINE ILA DONE ====="
close_project
exit 0
