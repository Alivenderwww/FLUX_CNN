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
  # 3. axi_noc (NUM_SI = 3 PL + 6 PS NMU = 9; 1 DDRMC, 4 MC channels)
  #    ALINX 06_ps_hello A72 baremetal demo 对齐: PS 必须把 FPD_CCI×4 +
  #    LPD_AXI + PMC_NOC 6 个 NMU 接进来, A72 才能走 NoC 到 DDR4.
  #    PL master (3 ConvCore) 占 S00..S02, PS NMU 占 S03..S08.
  # ====================================================================
  set axi_noc_0 [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_noc:1.0 axi_noc_0]
  set_property -dict [list \
    CONFIG.MC_COMPONENT_WIDTH {x16} \
    CONFIG.MC_INPUTCLK0_PERIOD {5000} \
    CONFIG.MC_MEMORY_SPEEDGRADE {DDR4-3200AA(22-22-22)} \
    CONFIG.MC_SYSTEM_CLOCK {No_Buffer} \
    CONFIG.NUM_MC {1} \
    CONFIG.NUM_MCP {4} \
    CONFIG.NUM_SI {9} \
    CONFIG.NUM_MI {0} \
    CONFIG.NUM_CLKS {9} \
  ] $axi_noc_0

  # 3a. S00..S02: PL master (ConvCore m0/m1/m2), 走 MC_0
  for {set i 0} {$i < 3} {incr i} {
    set port [format "S%02d_AXI" $i]
    set_property -dict [list \
      CONFIG.CONNECTIONS [format "MC_0 \{read_bw \{500\} write_bw \{500\} read_avg_burst \{4\} write_avg_burst \{4\}\}"] \
      CONFIG.CATEGORY {pl} \
      CONFIG.DATA_WIDTH {128} \
    ] [get_bd_intf_pins /axi_noc_0/$port]
    set_property CONFIG.ASSOCIATED_BUSIF $port [get_bd_pins /axi_noc_0/aclk$i]
  }

  # 3b. S03..S06: PS FPD CCI NoC ×4 (A72 cluster cache-coherent → DDR), 分摊到 MC_3/2/0/1
  set ps_nmu_specs {
    {S03 ps_cci MC_3}
    {S04 ps_cci MC_2}
    {S05 ps_cci MC_0}
    {S06 ps_cci MC_1}
    {S07 ps_rpu MC_3}
    {S08 ps_pmc MC_2}
  }
  set i 3
  foreach spec $ps_nmu_specs {
    lassign $spec port category mc
    set_property -dict [list \
      CONFIG.REGION {0} \
      CONFIG.CONNECTIONS [format "%s \{read_bw \{100\} write_bw \{100\} read_avg_burst \{4\} write_avg_burst \{4\}\}" $mc] \
      CONFIG.NOC_PARAMS {} \
      CONFIG.CATEGORY $category \
    ] [get_bd_intf_pins /axi_noc_0/${port}_AXI]
    set_property CONFIG.ASSOCIATED_BUSIF ${port}_AXI [get_bd_pins /axi_noc_0/aclk$i]
    incr i
  }

  # ====================================================================
  # 4. smartconnect_0 (PS GP → multicore_top_vd100.csr_axil)
  # ====================================================================
  set smartconnect_0 [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 smartconnect_0]
  set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $smartconnect_0

  # 4b. smartconnect_pl_0/1/2 + axis_ila debug:
  for {set i 0} {$i < 3} {incr i} {
    set sc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 [format "smartconnect_pl_%0d" $i]]
    set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1} CONFIG.NUM_CLKS {1}] $sc
  }

  # axis_ila_dbg: 抓 ConvCore 0 m_axi 关键信号 (12 probes)
  # u_mc_vd100 wrapper 已 expose dbg_* 端口 (multicore_top_vd100_bd_v.v).
  set ila [create_bd_cell -type ip -vlnv xilinx.com:ip:axis_ila:1.2 ila_dbg]
  set_property -dict [list \
      CONFIG.C_NUM_OF_PROBES {30} \
      CONFIG.C_DATA_DEPTH {1024} \
      CONFIG.C_INPUT_PIPE_STAGES {1} \
      CONFIG.C_TRIGOUT_EN {false} \
      CONFIG.C_TRIGIN_EN {false} \
      CONFIG.C_PROBE0_WIDTH {1}  CONFIG.C_PROBE1_WIDTH {1} \
      CONFIG.C_PROBE2_WIDTH {1}  CONFIG.C_PROBE3_WIDTH {1} \
      CONFIG.C_PROBE4_WIDTH {1}  CONFIG.C_PROBE5_WIDTH {1} \
      CONFIG.C_PROBE6_WIDTH {32} CONFIG.C_PROBE7_WIDTH {32} \
      CONFIG.C_PROBE8_WIDTH {4}  CONFIG.C_PROBE9_WIDTH {4} \
      CONFIG.C_PROBE10_WIDTH {8} CONFIG.C_PROBE11_WIDTH {8} \
      CONFIG.C_PROBE12_WIDTH {2} CONFIG.C_PROBE13_WIDTH {1} \
      CONFIG.C_PROBE14_WIDTH {1} CONFIG.C_PROBE15_WIDTH {1} \
      CONFIG.C_PROBE16_WIDTH {1} CONFIG.C_PROBE17_WIDTH {16} \
      CONFIG.C_PROBE18_WIDTH {1} CONFIG.C_PROBE19_WIDTH {2} \
      CONFIG.C_PROBE20_WIDTH {2} CONFIG.C_PROBE21_WIDTH {4} \
      CONFIG.C_PROBE22_WIDTH {4} CONFIG.C_PROBE23_WIDTH {4} \
      CONFIG.C_PROBE24_WIDTH {1} CONFIG.C_PROBE25_WIDTH {1} \
      CONFIG.C_PROBE26_WIDTH {1} CONFIG.C_PROBE27_WIDTH {1} \
      CONFIG.C_PROBE28_WIDTH {1} CONFIG.C_PROBE29_WIDTH {1} \
  ] $ila

  # ====================================================================
  # 5. xlconcat (3 IRQ → PS pl_ps_irq[2:0])
  # ====================================================================
  set xlconcat_irq [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat:2.1 xlconcat_irq]
  set_property -dict [list \
    CONFIG.NUM_PORTS {3} \
    CONFIG.IN0_WIDTH {1} CONFIG.IN1_WIDTH {1} CONFIG.IN2_WIDTH {1} \
  ] $xlconcat_irq

  # ====================================================================
  # 6. multicore_top_vd100_bd — BD 友好 wrapper (X_INTERFACE_INFO attribute)
  #    自动识别 csr_axil + m00_axi/m01_axi/m02_axi + irq_done 为 AXI/IRQ interface
  # ====================================================================
  # 注: 必须 add_files multicore_top_vd100_bd.sv 后, BD 才能 add_module
  # 用 _v 版本 (.v wrapper, BD-friendly), 它内部 instantiate _bd.sv
  create_bd_cell -type module -reference multicore_top_vd100_bd_v u_mc_vd100

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
                 [get_bd_pins smartconnect_pl_0/aclk] \
                 [get_bd_pins smartconnect_pl_1/aclk] \
                 [get_bd_pins smartconnect_pl_2/aclk] \
                 [get_bd_pins ila_dbg/clk] \
                 [get_bd_pins u_mc_vd100/clk]

  # PS pl0_resetn → proc_sys_reset.ext_reset_in
  connect_bd_net [get_bd_pins versal_cips_0/pl0_resetn] [get_bd_pins proc_sys_reset_0/ext_reset_in]
  connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] [get_bd_pins u_mc_vd100/rst_n]
  connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] [get_bd_pins smartconnect_0/aresetn]
  for {set i 0} {$i < 3} {incr i} {
    connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] [get_bd_pins smartconnect_pl_${i}/aresetn]
  }

  # PS GP master → smartconnect → multicore_top_vd100.csr_axil
  connect_bd_intf_net [get_bd_intf_pins versal_cips_0/M_AXI_FPD] [get_bd_intf_pins smartconnect_0/S00_AXI]
  connect_bd_intf_net [get_bd_intf_pins smartconnect_0/M00_AXI] [get_bd_intf_pins u_mc_vd100/csr_axil]
  connect_bd_net [get_bd_pins versal_cips_0/m_axi_fpd_aclk] [get_bd_pins clk_wizard_0/clk_out1]

  # multicore_top_vd100_bd.m0X_axi → smartconnect_pl_X.S00 → smartconnect_pl_X.M00 → axi_noc.S0X_AXI
  # smartconnect 中间层做 AXI4 协议规范化 + ID 宽度匹配 (ConvCore 5-bit ID → axi_noc 期望 ID).
  for {set i 0} {$i < 3} {incr i} {
    connect_bd_intf_net [get_bd_intf_pins u_mc_vd100/m0${i}_axi] \
                        [get_bd_intf_pins smartconnect_pl_${i}/S00_AXI]
    connect_bd_intf_net [get_bd_intf_pins smartconnect_pl_${i}/M00_AXI] \
                        [get_bd_intf_pins axi_noc_0/S0${i}_AXI]
  }

  # ILA probes: 30 个 (m_axi 出口 + dfe FSM 全状态 + arbiter + 4 master + CSR)
  set probes {
      awvalid_0            0
      awready_0            1
      arvalid_0            2
      arready_0            3
      wvalid_0             4
      rvalid_0             5
      awaddr_0             6
      araddr_0             7
      awid_0               8
      arid_0               9
      awlen_0             10
      arlen_0             11
      dfe_state_0         12
      dfe_m_arvalid_0     13
      dfe_m_arready_0     14
      dfe_r_phase_0       15
      dfe_r_done_0        16
      dfe_r_beats_rcvd_0  17
      arb_rd_lock_0       18
      arb_rd_sel_0        19
      arb_cu_rd_sel_0     20
      master_arvalid_0    21
      master_arready_0    22
      master_rvalid_0     23
      start_dfe_pulse_0   24
      dfe_busy_0          25
      dfe_done_0          26
      idma_busy_0         27
      layer_busy_0        28
      csr_aw_fire_0       29
  }
  foreach {sig probe} $probes {
      connect_bd_net [get_bd_pins u_mc_vd100/dbg_$sig] \
                     [get_bd_pins ila_dbg/probe$probe]
  }

  # PS FPD CCI / LPD / PMC NoC outputs → axi_noc.S03..S08 + 各自 aclk
  connect_bd_intf_net [get_bd_intf_pins versal_cips_0/FPD_CCI_NOC_0] [get_bd_intf_pins axi_noc_0/S03_AXI]
  connect_bd_intf_net [get_bd_intf_pins versal_cips_0/FPD_CCI_NOC_1] [get_bd_intf_pins axi_noc_0/S04_AXI]
  connect_bd_intf_net [get_bd_intf_pins versal_cips_0/FPD_CCI_NOC_2] [get_bd_intf_pins axi_noc_0/S05_AXI]
  connect_bd_intf_net [get_bd_intf_pins versal_cips_0/FPD_CCI_NOC_3] [get_bd_intf_pins axi_noc_0/S06_AXI]
  connect_bd_intf_net [get_bd_intf_pins versal_cips_0/LPD_AXI_NOC_0] [get_bd_intf_pins axi_noc_0/S07_AXI]
  connect_bd_intf_net [get_bd_intf_pins versal_cips_0/PMC_NOC_AXI_0] [get_bd_intf_pins axi_noc_0/S08_AXI]
  connect_bd_net [get_bd_pins versal_cips_0/fpd_cci_noc_axi0_clk] [get_bd_pins axi_noc_0/aclk3]
  connect_bd_net [get_bd_pins versal_cips_0/fpd_cci_noc_axi1_clk] [get_bd_pins axi_noc_0/aclk4]
  connect_bd_net [get_bd_pins versal_cips_0/fpd_cci_noc_axi2_clk] [get_bd_pins axi_noc_0/aclk5]
  connect_bd_net [get_bd_pins versal_cips_0/fpd_cci_noc_axi3_clk] [get_bd_pins axi_noc_0/aclk6]
  connect_bd_net [get_bd_pins versal_cips_0/lpd_axi_noc_clk]      [get_bd_pins axi_noc_0/aclk7]
  connect_bd_net [get_bd_pins versal_cips_0/pmc_axi_noc_axi0_clk] [get_bd_pins axi_noc_0/aclk8]

  # IRQ done [2:0] → 3 xlslice → xlconcat → PS pl_ps_irq
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

  # PS NMU 6 个 (S03..S08) → DDR_LOW0 (4 MC channel 分摊). A72 看 0x0..0x7FFFFFFF 是 DDR4 低 2GB.
  set ps_addr_spaces {
    {FPD_CCI_NOC_0 S03 C3_DDR_LOW0}
    {FPD_CCI_NOC_1 S04 C2_DDR_LOW0}
    {FPD_CCI_NOC_2 S05 C0_DDR_LOW0}
    {FPD_CCI_NOC_3 S06 C1_DDR_LOW0}
    {LPD_AXI_NOC_0 S07 C3_DDR_LOW0}
    {PMC_NOC_AXI_0 S08 C2_DDR_LOW0}
  }
  foreach spec $ps_addr_spaces {
    lassign $spec aspace si chan
    assign_bd_address -offset 0x00000000 -range 0x80000000 \
       -target_address_space [get_bd_addr_spaces versal_cips_0/$aspace] \
       [get_bd_addr_segs axi_noc_0/${si}_AXI/$chan] -force
  }

  # PS GP → CSR_AXIL aperture (0xA400_0000, 16 KB).
  # 试 Reg 跟 reg0 两种 segment 名 (BD module reference 通常 reg0).
  set csr_seg ""
  foreach segname {Reg reg0} {
    set s [get_bd_addr_segs -quiet u_mc_vd100/csr_axil/$segname]
    if {$s ne ""} { set csr_seg $s; break }
  }
  if {$csr_seg ne ""} {
    catch {
      assign_bd_address -offset 0xA4000000 -range 0x4000 \
          -target_address_space [get_bd_addr_spaces versal_cips_0/M_AXI_FPD] \
          $csr_seg -force
      puts "  csr_axil seg $csr_seg → 0xA4000000 16KB"
    }
  } else {
    puts "  WARN: csr_axil seg deferred to apply_config retry"
  }
