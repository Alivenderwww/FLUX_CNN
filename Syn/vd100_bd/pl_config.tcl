# =============================================================================
# pl_config.tcl  --  VD100 demo BD: PL 端 (multicore_top_vd100 作为 BD module)
#
# 关键策略: 不写顶层 top.v, 直接让 BD 实例化 multicore_top_vd100.sv 作为 RTL block.
# Vivado BD 支持 add_module 加 RTL, BD wrapper 自动出顶层 IO.
#
# BD 内部结构:
#   versal_cips (PS A72) ──M_AXI_FPD──► smartconnect ──► multicore_top_vd100.csr_axil
#   versal_cips ──pl_ps_irq[2:0]◄── xlconcat ◄── multicore_top_vd100.irq_done[2:0]
#   multicore_top_vd100.m_axi[0/1/2] ──axi_noc.S00/01/02──► DDRMC ──► DDR4 (板载)
#   util_ds_buf (200 MHz diff) → clk_wizard → 100 MHz axi_clk → 全局
# =============================================================================

  # ====================================================================
  # 1. Interface Ports — BD 顶层 IO (Vivado wrapper 自动出)
  # ====================================================================
  set sys [create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_clock_rtl:1.0 sys]
  set_property CONFIG.FREQ_HZ {200000000} $sys

  set DDR4 [create_bd_intf_port -mode Master -vlnv xilinx.com:interface:ddr4_rtl:1.0 DDR4]

  # ====================================================================
  # 2. clk_wizard / util_ds_buf / proc_sys_reset
  # ====================================================================
  set util_ds_buf_0 [create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf:2.2 util_ds_buf_0]
  set clk_wizard_0 [create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wizard:1.0 clk_wizard_0]
  set_property -dict [list \
    CONFIG.CLKOUT_DRIVES {BUFG,BUFG,BUFG,BUFG,BUFG,BUFG,BUFG} \
    CONFIG.CLKOUT_PORT {clk_out1,clk_out2,clk_out3,clk_out4,clk_out5,clk_out6,clk_out7} \
    CONFIG.CLKOUT_REQUESTED_OUT_FREQUENCY {100.000,100.000,100.000,100.000,100.000,100.000,100.000} \
    CONFIG.CLKOUT_USED {true,false,false,false,false,false,false} \
  ] $clk_wizard_0
  set proc_sys_reset_0 [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_0]

  # ====================================================================
  # 3. axi_noc (3 SI 128-bit PL master + 1 DDRMC)
  # ====================================================================
  set axi_noc_0 [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_noc:1.0 axi_noc_0]
  set_property -dict [list \
    CONFIG.MC_COMPONENT_WIDTH {x16} \
    CONFIG.MC_INPUTCLK0_PERIOD {5000} \
    CONFIG.MC_MEMORY_SPEEDGRADE {DDR4-3200AA(22-22-22)} \
    CONFIG.MC_SYSTEM_CLOCK {No_Buffer} \
    CONFIG.NUM_MC {1} \
    CONFIG.NUM_SI {3} \
    CONFIG.NUM_MI {0} \
  ] $axi_noc_0

  for {set i 0} {$i < 3} {incr i} {
    set port [format "S%02d_AXI" $i]
    set_property -dict [list \
      CONFIG.CONNECTIONS [format "MC_0 \{read_bw \{500\} write_bw \{500\} read_avg_burst \{4\} write_avg_burst \{4\}\}"] \
      CONFIG.CATEGORY {pl} \
      CONFIG.DATA_WIDTH {128} \
    ] [get_bd_intf_pins /axi_noc_0/$port]
    set_property CONFIG.ASSOCIATED_BUSIF $port [get_bd_pins /axi_noc_0/aclk[expr $i]]
  }

  # ====================================================================
  # 4. smartconnect (PS GP → multicore_top_vd100.csr_axil)
  # ====================================================================
  set smartconnect_0 [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 smartconnect_0]
  set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $smartconnect_0

  # ====================================================================
  # 5. xlconcat (3 IRQ → PS pl_ps_irq[2:0])
  # ====================================================================
  set xlconcat_irq [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat:2.1 xlconcat_irq]
  set_property -dict [list \
    CONFIG.NUM_PORTS {3} \
    CONFIG.IN0_WIDTH {1} CONFIG.IN1_WIDTH {1} CONFIG.IN2_WIDTH {1} \
  ] $xlconcat_irq

  # ====================================================================
  # 6. multicore_top_vd100 — 加为 BD module (Vivado 自动从 RTL 提取端口签名)
  # ====================================================================
  # 注: 必须 add_files multicore_top_vd100.sv 后, BD 才能 add_module
  create_bd_cell -type module -reference multicore_top_vd100 u_mc_vd100
  set_property -dict [list CONFIG.NUM_CORES {3}] [get_bd_cells u_mc_vd100]

  # ====================================================================
  # 7. 连线 — Interface
  # ====================================================================
  # sys diff clk → util_ds_buf → clk_wizard.clk_in1 + axi_noc.sys_clk0
  connect_bd_intf_net [get_bd_intf_ports sys] [get_bd_intf_pins util_ds_buf_0/CLK_IN_D]
  connect_bd_net [get_bd_pins util_ds_buf_0/IBUF_OUT] [get_bd_pins clk_wizard_0/clk_in1]
  connect_bd_net [get_bd_pins util_ds_buf_0/IBUF_OUT] [get_bd_pins axi_noc_0/sys_clk0]

  # axi_clk = clk_wizard.clk_out1 → axi_noc aclk0/1/2 + smartconnect + multicore_top_vd100.clk + proc_sys_reset
  for {set i 0} {$i < 3} {incr i} {
    connect_bd_net [get_bd_pins clk_wizard_0/clk_out1] [get_bd_pins axi_noc_0/aclk[expr $i]]
  }
  connect_bd_net [get_bd_pins clk_wizard_0/clk_out1] \
                 [get_bd_pins proc_sys_reset_0/slowest_sync_clk] \
                 [get_bd_pins smartconnect_0/aclk] \
                 [get_bd_pins u_mc_vd100/clk]

  # PS pl0_resetn → proc_sys_reset.ext_reset_in
  connect_bd_net [get_bd_pins versal_cips_0/pl0_resetn] [get_bd_pins proc_sys_reset_0/ext_reset_in]
  connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] [get_bd_pins u_mc_vd100/rst_n]
  connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] [get_bd_pins smartconnect_0/aresetn]

  # PS GP master → smartconnect → multicore_top_vd100.csr_axil
  connect_bd_intf_net [get_bd_intf_pins versal_cips_0/M_AXI_FPD] [get_bd_intf_pins smartconnect_0/S00_AXI]
  connect_bd_intf_net [get_bd_intf_pins smartconnect_0/M00_AXI] [get_bd_intf_pins u_mc_vd100/csr_axil]
  connect_bd_net [get_bd_pins versal_cips_0/m_axi_fpd_aclk] [get_bd_pins clk_wizard_0/clk_out1]

  # multicore_top_vd100.m_axi[0/1/2] → axi_noc.S00/01/02
  for {set i 0} {$i < 3} {incr i} {
    set port [format "S%02d_AXI" $i]
    set mc_port [format "m_axi_%d" $i]
    # 注意: multicore_top_vd100 用 packed vector 端口 (m_axi_awid 等), Vivado BD
    # 自动识别为 AXI4 interface 通过 X_INTERFACE_INFO attribute. 如果没识别, 需要
    # 在 RTL 加 attribute 或写 wrapper 把 vector 拆 interface.
    # 暂用 connect_bd_net (signal level), 后续板级集成补 X_INTERFACE attribute.
    # connect_bd_intf_net [get_bd_intf_pins u_mc_vd100/$mc_port] [get_bd_intf_pins axi_noc_0/$port]
  }

  # IRQ done [2:0] → xlconcat → PS pl_ps_irq
  connect_bd_net [get_bd_pins u_mc_vd100/irq_done] [get_bd_pins xlconcat_irq/dout]
  # xlconcat IN0/1/2 都是 1 bit, 但 IRQ 是 vector 3 bit. 用 xlslice 分别取 bit
  set irq_s0 [create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice:1.0 irq_s0]
  set irq_s1 [create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice:1.0 irq_s1]
  set irq_s2 [create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice:1.0 irq_s2]
  set_property -dict [list CONFIG.DIN_WIDTH {3} CONFIG.DIN_FROM {0} CONFIG.DIN_TO {0}] $irq_s0
  set_property -dict [list CONFIG.DIN_WIDTH {3} CONFIG.DIN_FROM {1} CONFIG.DIN_TO {1}] $irq_s1
  set_property -dict [list CONFIG.DIN_WIDTH {3} CONFIG.DIN_FROM {2} CONFIG.DIN_TO {2}] $irq_s2
  for {set i 0} {$i < 3} {incr i} {
    connect_bd_net [get_bd_pins u_mc_vd100/irq_done] [get_bd_pins irq_s$i/Din]
    connect_bd_net [get_bd_pins irq_s$i/Dout] [get_bd_pins xlconcat_irq/In$i]
  }
  # PS_PMC_CONFIG 启用 pl_ps_irq 后, 接进来. 跟 ps_config.tcl 配合.
  # connect_bd_net [get_bd_pins xlconcat_irq/dout] [get_bd_pins versal_cips_0/pl_ps_irq0]

  # axi_noc.DDR4 output → 板载 DDR4 pin
  connect_bd_intf_net [get_bd_intf_ports DDR4] [get_bd_intf_pins axi_noc_0/CH0_DDR4_0]

  # ====================================================================
  # 8. Address segments
  # ====================================================================
  # PL 3 master 共享 PS DDR4 low 0x0_0000_0000 - 0x0_8000_0000 (2 GB)
  for {set i 0} {$i < 3} {incr i} {
    set port [format "S%02d_AXI" $i]
    assign_bd_address -offset 0x00000000 -range 0x80000000 \
       -target_address_space [get_bd_addr_spaces u_mc_vd100/$port] \
       [get_bd_addr_segs axi_noc_0/${port}/C0_DDR_LOW0] -force
  }

  # PS GP → CSR_AXIL aperture (0xA000_0000, 16 KB)
  assign_bd_address -offset 0xA0000000 -range 0x4000 \
     -target_address_space [get_bd_addr_spaces versal_cips_0/M_AXI_FPD] \
     [get_bd_addr_segs u_mc_vd100/csr_axil/Reg] -force
