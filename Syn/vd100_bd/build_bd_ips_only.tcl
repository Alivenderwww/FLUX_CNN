# =============================================================================
# build_bd_ips_only.tcl  --  只创建所有 BD IP + 配置, 不做连线
#
# 设计意图: 之前 create_project.tcl 自动连线时撞 axi_noc 端口名 corner case
# 失败. 这个脚本只创建 IP, 让 user 在 GUI 手动拖线连接, 避免脚本端口名硬编码.
#
# 用法 (在 Vivado 2023.2 GUI Tcl Console 里):
#   pwd            ;# 确认当前在 vd100_resnet11.xpr 工程目录 (output/)
#   cd <project_dir>  ;# 必要时 cd 到工程目录
#   source ../build_bd_ips_only.tcl
#
# 之后 GUI Block Design canvas 应该出现所有 IP (无连线), 按 BRINGUP.md 手动连.
# =============================================================================

# 假设当前已经在 BD design_1 (Vivado GUI 已 open_bd_design)
# 否则: open_bd_design design_1.bd

# -----------------------------------------------------------------------------
# 1. Versal CIPS (PS A72)
# -----------------------------------------------------------------------------
if {[get_bd_cells -quiet versal_cips_0] eq ""} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:versal_cips:3.4 versal_cips_0
    set_property -dict [list \
        CONFIG.DESIGN_MODE {0} \
        CONFIG.PS_PMC_CONFIG { \
          DESIGN_MODE {0} \
          PS_BOARD_INTERFACE {Custom} \
          PS_NUM_FABRIC_RESETS {1} \
          PS_USE_M_AXI_FPD {1} \
          PS_USE_PMCPL_CLK0 {1} \
          PS_PL_CONNECTIVITY_MODE {Custom} \
          SMON_ALARMS {Set_Alarms_On} \
          SMON_ENABLE_TEMP_AVERAGING {0} \
          SMON_TEMP_AVERAGING_SAMPLES {0} \
        } \
      ] [get_bd_cells versal_cips_0]
    puts "  + versal_cips_0 created"
} else { puts "  - versal_cips_0 already exists, skipped" }

# -----------------------------------------------------------------------------
# 2. clk_wizard (200 MHz → 100 MHz axi_clk)
# -----------------------------------------------------------------------------
if {[get_bd_cells -quiet clk_wizard_0] eq ""} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wizard:1.0 clk_wizard_0
    set_property -dict [list \
      CONFIG.CLKOUT_DRIVES {BUFG,BUFG,BUFG,BUFG,BUFG,BUFG,BUFG} \
      CONFIG.CLKOUT_PORT {clk_out1,clk_out2,clk_out3,clk_out4,clk_out5,clk_out6,clk_out7} \
      CONFIG.CLKOUT_REQUESTED_OUT_FREQUENCY {100.000,100.000,100.000,100.000,100.000,100.000,100.000} \
      CONFIG.CLKOUT_USED {true,false,false,false,false,false,false} \
    ] [get_bd_cells clk_wizard_0]
    puts "  + clk_wizard_0 created"
}

# -----------------------------------------------------------------------------
# 3. util_ds_buf (200 MHz diff IBUFDS)
# -----------------------------------------------------------------------------
if {[get_bd_cells -quiet util_ds_buf_0] eq ""} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf:2.2 util_ds_buf_0
    puts "  + util_ds_buf_0 created"
}

# -----------------------------------------------------------------------------
# 4. proc_sys_reset
# -----------------------------------------------------------------------------
if {[get_bd_cells -quiet proc_sys_reset_0] eq ""} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_0
    puts "  + proc_sys_reset_0 created"
}

# -----------------------------------------------------------------------------
# 5. axi_noc (3 SI 128-bit + 1 DDRMC)
# -----------------------------------------------------------------------------
if {[get_bd_cells -quiet axi_noc_0] eq ""} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:axi_noc:1.0 axi_noc_0
    set_property -dict [list \
      CONFIG.MC_COMPONENT_WIDTH {x16} \
      CONFIG.MC_INPUTCLK0_PERIOD {5000} \
      CONFIG.MC_MEMORY_SPEEDGRADE {DDR4-3200AA(22-22-22)} \
      CONFIG.MC_SYSTEM_CLOCK {No_Buffer} \
      CONFIG.NUM_MC {1} \
      CONFIG.NUM_SI {3} \
      CONFIG.NUM_MI {0} \
    ] [get_bd_cells axi_noc_0]
    puts "  + axi_noc_0 created (3 SI 128-bit + 1 DDRMC)"
    puts "    NOTE: double-click axi_noc_0 -> Inputs page,"
    puts "          set S00/S01/S02_AXI 'Connected To' = MC_0,"
    puts "          DATA_WIDTH = 128, CATEGORY = pl, then Save"
}

# -----------------------------------------------------------------------------
# 6. smartconnect (PS GP → multicore_top_vd100_bd.csr_axil)
# -----------------------------------------------------------------------------
if {[get_bd_cells -quiet smartconnect_0] eq ""} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 smartconnect_0
    set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] [get_bd_cells smartconnect_0]
    puts "  + smartconnect_0 created"
}

# -----------------------------------------------------------------------------
# 7. xlconcat (3-bit IRQ → PS pl_ps_irq0)
# -----------------------------------------------------------------------------
if {[get_bd_cells -quiet xlconcat_irq] eq ""} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat:2.1 xlconcat_irq
    set_property -dict [list \
      CONFIG.NUM_PORTS {3} \
      CONFIG.IN0_WIDTH {1} CONFIG.IN1_WIDTH {1} CONFIG.IN2_WIDTH {1} \
    ] [get_bd_cells xlconcat_irq]
    puts "  + xlconcat_irq created"
}

# -----------------------------------------------------------------------------
# 8. xlslice × 3 (拆 3-bit irq_done 给 xlconcat)
# -----------------------------------------------------------------------------
foreach i {0 1 2} {
    set name "irq_s$i"
    if {[get_bd_cells -quiet $name] eq ""} {
        create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice:1.0 $name
        set_property -dict [list \
          CONFIG.DIN_WIDTH {3} CONFIG.DIN_FROM $i CONFIG.DIN_TO $i \
        ] [get_bd_cells $name]
        puts "  + $name created (slice bit $i)"
    }
}

# -----------------------------------------------------------------------------
# 9. multicore_top_vd100_bd (我们的 PL 加速器)
#    要求 RTL 已经 add_files 进 sources_1
# -----------------------------------------------------------------------------
if {[get_bd_cells -quiet u_mc_vd100] eq ""} {
    if {[catch {create_bd_cell -type module -reference multicore_top_vd100_bd u_mc_vd100} err]} {
        puts "  ! Failed to add multicore_top_vd100_bd: $err"
        puts "    Run: source C:/_Project/FLUX_CNN/Syn/vd100_bd/add_rtl.tcl first"
    } else {
        puts "  + u_mc_vd100 (multicore_top_vd100_bd) added as BD module"
    }
}

# -----------------------------------------------------------------------------
# 10. Interface ports (BD 顶层 IO)
# -----------------------------------------------------------------------------
if {[get_bd_intf_ports -quiet sys] eq ""} {
    create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_clock_rtl:1.0 sys
    set_property CONFIG.FREQ_HZ {200000000} [get_bd_intf_ports sys]
    puts "  + sys port (200 MHz diff clk)"
}
if {[get_bd_intf_ports -quiet DDR4] eq ""} {
    create_bd_intf_port -mode Master -vlnv xilinx.com:interface:ddr4_rtl:1.0 DDR4
    puts "  + DDR4 port (board DDR4)"
}

regenerate_bd_layout
save_bd_design
puts ""
puts "============================================================"
puts " BD IP-only build done. All IPs on canvas, no connections yet."
puts " Next: drag wires per Syn/vd100_bd/STEP_BY_STEP.md Step 2-7"
puts "============================================================"
