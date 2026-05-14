# =============================================================================
# build_bd.tcl  --  最小 BD 创建 + 连线
#
# 架构 (PL-only, no PS DDR, no axi_noc):
#
#   sys (200 MHz diff) ─► util_ds_buf ─► clk_wizard ─► 100 MHz axi_clk
#                                                          │
#   versal_cips_pmc_only (PDI boot 用, 不启 A72/PS DDR)     │
#       └ pl_resetn0 ─► proc_sys_reset ─► axi_aresetn       │
#                                                          │
#                       ┌──────────────────────────────────┘
#                       │
#                       ▼
#   ┌────────────────────────────────────────────────────────────────┐
#   │ jtag_to_axi_csr (AXI-Lite, 32-bit)                              │
#   │   └► u_mc_minimal.csr_axil (slave)                              │
#   │                                                                 │
#   │ jtag_to_axi_bulk (AXI4, 128-bit, 32-bit addr)                   │
#   │   └► smartconnect ─► axi_bram_ctrl ─► BRAM (1 MB)               │
#   │                                                                 │
#   │ u_mc_minimal.m_axi (AXI4, 128-bit)                              │
#   │   └► smartconnect ─► (同上 BRAM)                                │
#   └────────────────────────────────────────────────────────────────┘
#
# 地址 map (jtag_to_axi 一次 set_property 配死):
#   jtag_to_axi_csr.M00_AXI  → u_mc_minimal/csr_axil  base 0x80000000 size 16KB
#   jtag_to_axi_bulk.M00_AXI → smartconnect/S00 → BRAM base 0x10000000 size 1MB
#   u_mc_minimal.m_axi       → smartconnect/S01 → BRAM 同上
# =============================================================================

# =============================================================================
# 1. Versal CIPS (PMC-only mode, 不启 A72/PS DDR, 仅做 PDI boot)
# =============================================================================
puts "  + versal_cips_0 (PMC-only, A72 disabled)"
create_bd_cell -type ip -vlnv xilinx.com:ip:versal_cips:3.4 versal_cips_0
# PMC-only: 必要的最小配置. pl0_resetn 用作 PL reset (PDI boot 完后 release).
# PS_USE_PMCPL_CLK0=1 保留 pl0_ref_clk 输出 (虽然我们不用, 但避免端口缺失).
# 不用 M_AXI_FPD, 不启 A72, 没 PS DDR.
set_property -dict [list \
    CONFIG.DESIGN_MODE {0} \
    CONFIG.PS_PMC_CONFIG { \
        DESIGN_MODE {0} \
        PS_BOARD_INTERFACE {Custom} \
        PS_NUM_FABRIC_RESETS {1} \
        PS_USE_PMCPL_CLK0 {1} \
        PS_PL_CONNECTIVITY_MODE {Custom} \
        SMON_ALARMS {Set_Alarms_On} \
        SMON_ENABLE_TEMP_AVERAGING {0} \
        SMON_TEMP_AVERAGING_SAMPLES {0} \
    } \
] [get_bd_cells versal_cips_0]

# =============================================================================
# 2. clk_wizard (200 MHz diff → 100 MHz)
# =============================================================================
puts "  + clk_wizard_0 (200MHz → 100MHz)"
create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wizard:1.0 clk_wizard_0
set_property -dict [list \
    CONFIG.CLKOUT_DRIVES {BUFG,BUFG,BUFG,BUFG,BUFG,BUFG,BUFG} \
    CONFIG.CLKOUT_PORT {clk_out1,clk_out2,clk_out3,clk_out4,clk_out5,clk_out6,clk_out7} \
    CONFIG.CLKOUT_REQUESTED_OUT_FREQUENCY {100.000,100.000,100.000,100.000,100.000,100.000,100.000} \
    CONFIG.CLKOUT_USED {true,false,false,false,false,false,false} \
    CONFIG.CLK_OUT1_PORT {clk_out1} \
    CONFIG.PRIMITIVE {Auto} \
    CONFIG.PRIM_IN_FREQ {200.000} \
    CONFIG.PRIM_SOURCE {Differential_clock_capable_pin} \
] [get_bd_cells clk_wizard_0]

# =============================================================================
# 3. util_ds_buf (200 MHz diff IBUFDS)
# =============================================================================
puts "  + util_ds_buf_0 (200MHz diff buf)"
create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf:2.2 util_ds_buf_0

# =============================================================================
# 4. proc_sys_reset
# =============================================================================
puts "  + proc_sys_reset_0"
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_0

# =============================================================================
# 5. jtag_to_axi_csr (AXI-Lite, host → ConvCore CSR)
# =============================================================================
puts "  + jtag_to_axi_csr"
create_bd_cell -type ip -vlnv xilinx.com:ip:jtag_axi:1.2 jtag_to_axi_csr
set_property -dict [list \
    CONFIG.PROTOCOL {2} \
    CONFIG.WRITE_BURST_LENGTH {1} \
    CONFIG.READ_BURST_LENGTH {1} \
] [get_bd_cells jtag_to_axi_csr]
# PROTOCOL: 0=AXI4, 1=AXI3, 2=AXI4-LITE

# =============================================================================
# 6. jtag_to_axi_bulk (AXI4 128-bit, host → BRAM)
# =============================================================================
puts "  + jtag_to_axi_bulk"
create_bd_cell -type ip -vlnv xilinx.com:ip:jtag_axi:1.2 jtag_to_axi_bulk
set_property -dict [list \
    CONFIG.PROTOCOL {0} \
    CONFIG.DATA_WIDTH {128} \
    CONFIG.ADDR_WIDTH {32} \
    CONFIG.WRITE_BURST_LENGTH {16} \
    CONFIG.READ_BURST_LENGTH {16} \
    CONFIG.HAS_BURST {1} \
] [get_bd_cells jtag_to_axi_bulk]

# =============================================================================
# 7. smartconnect (2 SI → 1 MI; jtag_to_axi_bulk + u_mc_minimal.m_axi → BRAM)
# =============================================================================
puts "  + smartconnect_bulk (2 SI → 1 MI)"
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 smartconnect_bulk
set_property -dict [list CONFIG.NUM_SI {2} CONFIG.NUM_MI {1} CONFIG.NUM_CLKS {1}] [get_bd_cells smartconnect_bulk]

# =============================================================================
# 8. axi_bram_ctrl + BRAM (1 MB; 64K word × 128-bit)
# =============================================================================
puts "  + axi_bram_ctrl_0"
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_bram_ctrl:4.1 axi_bram_ctrl_0
set_property -dict [list \
    CONFIG.DATA_WIDTH {128} \
    CONFIG.SINGLE_PORT_BRAM {1} \
    CONFIG.ECC_TYPE {0} \
] [get_bd_cells axi_bram_ctrl_0]

puts "  + blk_mem_gen_0 (1 MB BRAM)"
create_bd_cell -type ip -vlnv xilinx.com:ip:blk_mem_gen:8.4 blk_mem_gen_0
set_property -dict [list \
    CONFIG.Memory_Type {True_Dual_Port_RAM} \
    CONFIG.Enable_32bit_Address {true} \
    CONFIG.Use_Byte_Write_Enable {true} \
    CONFIG.Byte_Size {8} \
    CONFIG.Assume_Synchronous_Clk {true} \
] [get_bd_cells blk_mem_gen_0]

# =============================================================================
# 9. multicore_top_minimal (我们的 PL 加速器)
# =============================================================================
puts "  + u_mc_minimal (multicore_top_minimal)"
create_bd_cell -type module -reference multicore_top_minimal u_mc_minimal

# =============================================================================
# 10. 顶层 IO 端口 (sys diff clock)
# =============================================================================
puts "  + sys (200 MHz diff clock)"
create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_clock_rtl:1.0 sys
set_property CONFIG.FREQ_HZ {200000000} [get_bd_intf_ports sys]

# =============================================================================
# 11. 连线
# =============================================================================
puts "=== Connecting BD ==="

# sys (差分时钟) → util_ds_buf → clk_wizard.clk_in1
connect_bd_intf_net [get_bd_intf_ports sys] [get_bd_intf_pins util_ds_buf_0/CLK_IN_D]
connect_bd_net [get_bd_pins util_ds_buf_0/IBUF_OUT] [get_bd_pins clk_wizard_0/clk_in1]

# clk_wizard.clk_out1 → 所有时钟域 (100 MHz)
set CLK [get_bd_pins clk_wizard_0/clk_out1]
connect_bd_net $CLK [get_bd_pins proc_sys_reset_0/slowest_sync_clk]
connect_bd_net $CLK [get_bd_pins jtag_to_axi_csr/aclk]
connect_bd_net $CLK [get_bd_pins jtag_to_axi_bulk/aclk]
connect_bd_net $CLK [get_bd_pins smartconnect_bulk/aclk]
connect_bd_net $CLK [get_bd_pins axi_bram_ctrl_0/s_axi_aclk]
connect_bd_net $CLK [get_bd_pins u_mc_minimal/clk]

# Reset:
#   CIPS.pl0_resetn (PDI boot 完成才 release) → proc_sys_reset.ext_reset_in
#   clk_wizard.locked → proc_sys_reset.dcm_locked (PLL 稳定才 release)
# CIPS.pl0_ref_clk 我们用不到 (PS_USE_PMCPL_CLK0=1 保留 pin 但悬空, BD 可能 warn).
# 不接 CIPS clk - 我们 sys_clk 走 util_ds_buf, 跟 PS 时钟域无关.
connect_bd_net [get_bd_pins clk_wizard_0/locked] [get_bd_pins proc_sys_reset_0/dcm_locked]
connect_bd_net [get_bd_pins versal_cips_0/pl0_resetn] [get_bd_pins proc_sys_reset_0/ext_reset_in]

set RST [get_bd_pins proc_sys_reset_0/peripheral_aresetn]
connect_bd_net $RST [get_bd_pins jtag_to_axi_csr/aresetn]
connect_bd_net $RST [get_bd_pins jtag_to_axi_bulk/aresetn]
connect_bd_net $RST [get_bd_pins smartconnect_bulk/aresetn]
connect_bd_net $RST [get_bd_pins axi_bram_ctrl_0/s_axi_aresetn]
connect_bd_net $RST [get_bd_pins u_mc_minimal/rst_n]

# AXI 连接:
# jtag_to_axi_csr.M_AXI → u_mc_minimal.csr_axil
connect_bd_intf_net [get_bd_intf_pins jtag_to_axi_csr/M_AXI] [get_bd_intf_pins u_mc_minimal/csr_axil]
# jtag_to_axi_bulk.M_AXI → smartconnect.S00_AXI
connect_bd_intf_net [get_bd_intf_pins jtag_to_axi_bulk/M_AXI] [get_bd_intf_pins smartconnect_bulk/S00_AXI]
# u_mc_minimal.m_axi → smartconnect.S01_AXI
connect_bd_intf_net [get_bd_intf_pins u_mc_minimal/m_axi] [get_bd_intf_pins smartconnect_bulk/S01_AXI]
# smartconnect.M00_AXI → axi_bram_ctrl_0.S_AXI
connect_bd_intf_net [get_bd_intf_pins smartconnect_bulk/M00_AXI] [get_bd_intf_pins axi_bram_ctrl_0/S_AXI]
# axi_bram_ctrl_0.BRAM_PORTA → blk_mem_gen.BRAM_PORTA
connect_bd_intf_net [get_bd_intf_pins axi_bram_ctrl_0/BRAM_PORTA] [get_bd_intf_pins blk_mem_gen_0/BRAM_PORTA]

# =============================================================================
# 12. 地址 segment assign
# =============================================================================
puts "=== Assigning addresses ==="
# jtag_to_axi_csr → u_mc_minimal.csr_axil: base 0x80000000, size 16KB
assign_bd_address -offset 0x80000000 -range 4K \
    -target_address_space [get_bd_addr_spaces jtag_to_axi_csr/Data] \
    [get_bd_addr_segs u_mc_minimal/csr_axil/Reg] -force

# jtag_to_axi_bulk → BRAM: base 0x10000000, size 1MB
assign_bd_address -offset 0x10000000 -range 1M \
    -target_address_space [get_bd_addr_spaces jtag_to_axi_bulk/Data] \
    [get_bd_addr_segs axi_bram_ctrl_0/S_AXI/Mem0] -force

# u_mc_minimal.m_axi → BRAM: base 0x10000000, size 1MB
assign_bd_address -offset 0x10000000 -range 1M \
    -target_address_space [get_bd_addr_spaces u_mc_minimal/m_axi] \
    [get_bd_addr_segs axi_bram_ctrl_0/S_AXI/Mem0] -force

puts "============================================================"
puts " BD build done. Layout regenerated, will save in create_project.tcl"
puts "============================================================"
