# =============================================================================
# build_bd.tcl  --  最小 BD 创建 + 部分连线
#
# 架构 (PL 内不用 axi_noc/DDR4/mesh; host 经 CIPS A72 lwIP 已验证工作):
#
#   sys (200 MHz diff) ─► util_ds_buf ─► clk_wizard ─► 100 MHz axi_clk
#                                                          │
#   [USER ADD: versal_cips_0] (用户 IDE 自己配, 原 DDR 配置不能直接用)         │
#     ├ pl0_resetn ─► proc_sys_reset (用户连)                                  │
#     ├ m_axi_fpd_aclk ◄── (用户连 clk_wizard.clk_out1)                        │
#     └ M_AXI_FPD ─► smartconnect.S00_AXI (用户连)                            │
#                                                                             │
#       u_mc_minimal.m_axi ──► smartconnect (2SI→2MI) [✓ tcl 已连]            │
#                              ├ M00 ─► u_mc_minimal.csr_axil [✓ tcl 已连]    │
#                              └ M01 ─► axi_bram_ctrl ─► emb_mem_gen [✓ 已连] │
#
# 地址 map (用户 IDE 后 assign_bd_address):
#   CIPS M_AXI_FPD → u_mc_minimal.csr_axil    0xA400_0000 4KB  ← 跟 vd100_rpc_server.c CSR_BASE 一致, 免改 C 代码
#   CIPS M_AXI_FPD → BRAM                     0xA000_0000 1MB  ← 跟 ConvCore m_axi 视图同地址 (host load IFM ConvCore 读出来一致)
#   u_mc_minimal.m_axi → BRAM                 0xA000_0000 1MB  ← (tcl 已 assign)
#
# 用户 IDE 操作 SOP (创建完工程 + open BD 后):
#   1. 双击 versal_cips_0 (如果没有先 add IP):
#      - DESIGN_MODE: 1
#      - 关闭 DDR connectivity / NoC: 不用 NoC
#      - PS-PMC > PS-PMC IO Mapping: 配 boot (eMMC/QSPI), Ethernet, UART
#      - PS-PMC > Clock: enable PMC PL CLK0 (PS_USE_PMCPL_CLK0=1)
#      - PS-PMC > AXI Interfaces: enable M_AXI_FPD only (其他都关)
#      - PS-PMC > Fabric Resets: PS_NUM_FABRIC_RESETS=1
#   2. Run Block Automation (默认): 自动连 CIPS clk/reset
#   3. 手动连: CIPS.M_AXI_FPD → smartconnect_0.S00_AXI
#   4. 手动连: CIPS.pl0_resetn → proc_sys_reset_0.ext_reset_in (如果 automation 没连)
#   5. 手动连: clk_wizard_0.clk_out1 → versal_cips_0.m_axi_fpd_aclk
#   6. Address Editor → 手动配 (不要 Auto Assign, vivado 默认会给奇怪地址):
#        - versal_cips_0/M_AXI_FPD → u_mc_minimal/csr_axil/Reg   offset 0xA4000000  range 4K
#        - versal_cips_0/M_AXI_FPD → axi_bram_ctrl_0/S_AXI/Mem0  offset 0xA0000000  range 1M
#        (CSR_BASE 跟 host/vd100_ps_baremetal/vd100_rpc_server.c 写死的 0xA4000000 一致)
#   7. validate_bd_design + save_bd_design
# =============================================================================

# =============================================================================
# 1. clk_wizard (200 MHz diff → 100 MHz)
# =============================================================================
puts "  + clk_wizard_0 (200MHz → 100MHz)"
create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wizard:1.0 clk_wizard_0
set_property -dict [list \
    CONFIG.CLKOUT_DRIVES {BUFG,BUFG,BUFG,BUFG,BUFG,BUFG,BUFG} \
    CONFIG.CLKOUT_PORT {clk_out1,clk_out2,clk_out3,clk_out4,clk_out5,clk_out6,clk_out7} \
    CONFIG.CLKOUT_REQUESTED_OUT_FREQUENCY {100.000,100.000,100.000,100.000,100.000,100.000,100.000} \
    CONFIG.CLKOUT_USED {true,false,false,false,false,false,false} \
] [get_bd_cells clk_wizard_0]

# =============================================================================
# 2. util_ds_buf (200 MHz diff IBUFDS)
# =============================================================================
puts "  + util_ds_buf_0 (200MHz diff buf)"
create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf:2.2 util_ds_buf_0

# =============================================================================
# 3. proc_sys_reset
# =============================================================================
puts "  + proc_sys_reset_0"
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_0

# =============================================================================
# 4. smartconnect (2 SI → 2 MI):
#    SI: [USER 连 CIPS M_AXI_FPD] + u_mc_minimal.m_axi
#    MI: u_mc_minimal.csr_axil + axi_bram_ctrl
# =============================================================================
puts "  + smartconnect_0 (2 SI → 2 MI, auto width converter)"
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 smartconnect_0
set_property -dict [list CONFIG.NUM_SI {2} CONFIG.NUM_MI {2} CONFIG.NUM_CLKS {1}] [get_bd_cells smartconnect_0]

# =============================================================================
# 5. axi_bram_ctrl + BRAM (1 MB; 64K word × 128-bit)
# =============================================================================
puts "  + axi_bram_ctrl_0 (128-bit data)"
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_bram_ctrl:4.1 axi_bram_ctrl_0
set_property -dict [list \
    CONFIG.DATA_WIDTH {128} \
    CONFIG.SINGLE_PORT_BRAM {1} \
    CONFIG.ECC_TYPE {0} \
] [get_bd_cells axi_bram_ctrl_0]

puts "  + emb_mem_gen_0 (Versal BRAM, 跟 axi_bram_ctrl 配对 auto-propagate)"
create_bd_cell -type ip -vlnv xilinx.com:ip:emb_mem_gen:1.0 emb_mem_gen_0

# =============================================================================
# 6. multicore_top_minimal (我们的 PL 加速器)
# =============================================================================
puts "  + u_mc_minimal (multicore_top_minimal_v Verilog wrapper)"
# BD -type module -reference 不接受 SystemVerilog 顶层 (filemgmt 56-195).
# 用 .v wrapper multicore_top_minimal_v 内部实例化 SV module multicore_top_minimal.
create_bd_cell -type module -reference multicore_top_minimal_v u_mc_minimal

# =============================================================================
# 7. 顶层 IO 端口 (sys diff clock)
# =============================================================================
puts "  + sys (200 MHz diff clock port)"
create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_clock_rtl:1.0 sys
set_property CONFIG.FREQ_HZ {200000000} [get_bd_intf_ports sys]

# =============================================================================
# 8. 连线 (CIPS-related 留给用户 IDE)
# =============================================================================
puts "=== Connecting BD (excluding CIPS-related, user does in IDE) ==="

# sys (差分时钟) → util_ds_buf → clk_wizard.clk_in1
connect_bd_intf_net [get_bd_intf_ports sys] [get_bd_intf_pins util_ds_buf_0/CLK_IN_D]
connect_bd_net [get_bd_pins util_ds_buf_0/IBUF_OUT] [get_bd_pins clk_wizard_0/clk_in1]

# clk_wizard.clk_out1 → 所有时钟域 (100 MHz, CIPS m_axi_fpd_aclk 用户自己连)
set CLK [get_bd_pins clk_wizard_0/clk_out1]
connect_bd_net $CLK [get_bd_pins proc_sys_reset_0/slowest_sync_clk]
connect_bd_net $CLK [get_bd_pins smartconnect_0/aclk]
connect_bd_net $CLK [get_bd_pins axi_bram_ctrl_0/s_axi_aclk]
connect_bd_net $CLK [get_bd_pins u_mc_minimal/clk]

# Reset: Versal clk_wizard:1.0 没 'locked' pin (不像 7-series clk_wiz).
#   proc_sys_reset.dcm_locked 用 const 1 (PLL 是否 locked 不进 reset chain).
#   CIPS.pl0_resetn → proc_sys_reset.ext_reset_in (用户 IDE 连)
if {[get_bd_cells -quiet const_one] eq ""} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 const_one
    set_property -dict [list CONFIG.CONST_VAL {1} CONFIG.CONST_WIDTH {1}] [get_bd_cells const_one]
}
connect_bd_net [get_bd_pins const_one/dout] [get_bd_pins proc_sys_reset_0/dcm_locked]

set RST [get_bd_pins proc_sys_reset_0/peripheral_aresetn]
connect_bd_net $RST [get_bd_pins smartconnect_0/aresetn]
connect_bd_net $RST [get_bd_pins axi_bram_ctrl_0/s_axi_aresetn]
connect_bd_net $RST [get_bd_pins u_mc_minimal/rst_n]

# AXI 连接 (smartconnect 内部, 不依赖 CIPS):
# u_mc_minimal.m_axi → smartconnect.S01_AXI (S00 留给 CIPS M_AXI_FPD, 用户连)
connect_bd_intf_net [get_bd_intf_pins u_mc_minimal/m_axi] [get_bd_intf_pins smartconnect_0/S01_AXI]
# smartconnect.M00_AXI → u_mc_minimal.csr_axil
connect_bd_intf_net [get_bd_intf_pins smartconnect_0/M00_AXI] [get_bd_intf_pins u_mc_minimal/csr_axil]
# smartconnect.M01_AXI → axi_bram_ctrl_0.S_AXI
connect_bd_intf_net [get_bd_intf_pins smartconnect_0/M01_AXI] [get_bd_intf_pins axi_bram_ctrl_0/S_AXI]
# axi_bram_ctrl_0.BRAM_PORTA → emb_mem_gen_0.BRAM_PORTA
connect_bd_intf_net [get_bd_intf_pins axi_bram_ctrl_0/BRAM_PORTA] [get_bd_intf_pins emb_mem_gen_0/BRAM_PORTA]

# =============================================================================
# 9. 部分地址 segment assign (u_mc_minimal.m_axi → BRAM)
#    CIPS M_AXI_FPD 地址 由用户 IDE 配 CIPS 后 Auto Assign
# =============================================================================
puts "=== Assigning addresses (u_mc_minimal.m_axi 部分, CIPS 部分用户自己配) ==="
catch {
    assign_bd_address -offset 0xA0000000 -range 1M \
        -target_address_space [get_bd_addr_spaces u_mc_minimal/m_axi] \
        [get_bd_addr_segs axi_bram_ctrl_0/S_AXI/Mem0] -force
}

puts "============================================================"
puts " BD 构建完成 (CIPS 缺, 等用户 IDE 操作)."
puts " Next:"
puts "   1. open vd100_minimal.xpr in Vivado GUI"
puts "   2. 双击 design_1.bd → IDE 加 versal_cips_0 + 配置"
puts "   3. 连 CIPS.M_AXI_FPD → smartconnect_0.S00_AXI"
puts "   4. 连 CIPS clk/reset (Run Block Automation 或手动)"
puts "   5. Address Editor Auto Assign"
puts "   6. validate + save BD"
puts "   7. run synth_1 + impl_1 + write_device_image"
puts "============================================================"
