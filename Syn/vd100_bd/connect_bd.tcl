# =============================================================================
# connect_bd.tcl  --  BD 一次性连线 (跳过 GUI 拖线)
#
# 前提: 已经 source build_bd_ips_only.tcl, 所有 IP 都在 BD canvas 上.
# 用法 (Vivado GUI Tcl Console):
#   source C:/_Project/FLUX_CNN/Syn/vd100_bd/connect_bd.tcl
#
# 跑完: F6 Validate Design 应该 0 errors.
# =============================================================================

# ---- 工具: 安全连接, 避免重复连 ----
proc safe_connect_intf {src dst} {
    set src_pin [get_bd_intf_pins -quiet $src]
    set dst_pin [get_bd_intf_pins -quiet $dst]
    if {$src_pin eq "" || $dst_pin eq ""} {
        # 也试 port (顶层 port)
        if {$src_pin eq ""} { set src_pin [get_bd_intf_ports -quiet [file tail $src]] }
        if {$dst_pin eq ""} { set dst_pin [get_bd_intf_ports -quiet [file tail $dst]] }
    }
    if {$src_pin eq "" || $dst_pin eq ""} {
        puts "  ! intf miss: $src or $dst"
        return
    }
    if {[catch {connect_bd_intf_net $src_pin $dst_pin} err]} {
        puts "  - intf $src -> $dst: $err"
    } else {
        puts "  + intf $src -> $dst"
    }
}

proc safe_connect_net {src dst} {
    set src_pin [get_bd_pins -quiet $src]
    set dst_pin [get_bd_pins -quiet $dst]
    if {$src_pin eq ""} { set src_pin [get_bd_ports -quiet [file tail $src]] }
    if {$dst_pin eq ""} { set dst_pin [get_bd_ports -quiet [file tail $dst]] }
    if {$src_pin eq "" || $dst_pin eq ""} {
        puts "  ! pin miss: $src or $dst"
        return
    }
    if {[catch {connect_bd_net $src_pin $dst_pin} err]} {
        puts "  - net $src -> $dst: $err"
    } else {
        puts "  + net $src -> $dst"
    }
}

puts "=== Connecting BD ==="

# ---- 1. Clock ----
safe_connect_intf  /sys                                /util_ds_buf_0/CLK_IN_D
safe_connect_net   /util_ds_buf_0/IBUF_OUT             /clk_wizard_0/clk_in1
safe_connect_net   /util_ds_buf_0/IBUF_OUT             /axi_noc_0/sys_clk0
safe_connect_net   /clk_wizard_0/clk_out1              /axi_noc_0/aclk0
safe_connect_net   /clk_wizard_0/clk_out1              /proc_sys_reset_0/slowest_sync_clk
safe_connect_net   /clk_wizard_0/clk_out1              /smartconnect_0/aclk
safe_connect_net   /clk_wizard_0/clk_out1              /u_mc_vd100/clk
safe_connect_net   /clk_wizard_0/clk_out1              /versal_cips_0/m_axi_fpd_aclk

# ---- 2. Reset (用 active LOW peripheral_aresetn, 不要 peripheral_reset) ----
safe_connect_net   /versal_cips_0/pl0_resetn           /proc_sys_reset_0/ext_reset_in
safe_connect_net   /proc_sys_reset_0/peripheral_aresetn /u_mc_vd100/rst_n
safe_connect_net   /proc_sys_reset_0/peripheral_aresetn /smartconnect_0/aresetn

# ---- 3. AXI Lite CSR (PS GP → smartconnect → u_mc_vd100.csr_axil) ----
safe_connect_intf  /versal_cips_0/M_AXI_FPD            /smartconnect_0/S00_AXI
safe_connect_intf  /smartconnect_0/M00_AXI             /u_mc_vd100/csr_axil

# ---- 4. AXI Master x3 (u_mc_vd100 → axi_noc_0) — ★ 关键, 之前漏拖 ★ ----
safe_connect_intf  /u_mc_vd100/m00_axi                 /axi_noc_0/S00_AXI
safe_connect_intf  /u_mc_vd100/m01_axi                 /axi_noc_0/S01_AXI
safe_connect_intf  /u_mc_vd100/m02_axi                 /axi_noc_0/S02_AXI

# ---- 5. DDR4 (axi_noc 出板载) ----
safe_connect_intf  /axi_noc_0/CH0_DDR4_0               /DDR4

# ---- 6. IRQ (PL → PS, 走 3 个 IRQ pin: pl_ps_irq8/9/10 FPD) ----
# 注: 之前 IRQ 启用了 pl_ps_irq8, 9, 10. xlconcat 删了, 用 xlslice 直接接.
# u_mc_vd100/irq_done [2:0] -> 3 个 xlslice 取单 bit -> versal_cips pl_ps_irq8/9/10
foreach i {0 1 2} {
    set pl_irq [expr 8 + $i]
    safe_connect_net /u_mc_vd100/irq_done            /irq_s$i/Din
    set src "/irq_s$i/Dout"
    set dst "/versal_cips_0/pl_ps_irq$pl_irq"
    if {[get_bd_pins -quiet $dst] ne ""} {
        safe_connect_net $src $dst
    } else {
        # IRQ 端口名可能不一样, 试用其他风格
        set dst2 "/versal_cips_0/pl_ps_irq\[$pl_irq\]"
        if {[get_bd_pins -quiet $dst2] ne ""} { safe_connect_net $src $dst2 }
        # 或者 vector 风格 versal_cips_0/pl_ps_irq, 留 user 手动接
        puts "  ? versal_cips pl_ps_irq$pl_irq pin not found, may need manual connect"
    }
}

regenerate_bd_layout
save_bd_design
puts ""
puts "============================================================"
puts " BD connections done. Now press F6 to Validate."
puts "============================================================"
