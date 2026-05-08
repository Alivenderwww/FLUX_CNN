# =============================================================================
# ps_config.tcl  --  Versal CIPS 配置 (基于 ALINX 02_pl_rw_ddr 扩展)
#
# 比 ALINX 02_pl_rw_ddr 多启用:
#   - PS_USE__M_AXI_FPD: PS A72 → PL CSR (GP master), 给 multicore_top_vd100 配 CSR
#   - PS_USE__S_AXI_FPD_INST_x: PL → PS DDR (HP slave), Versal 走 NoC 不需要单独配
#   - PS_PL_INTC: 接收 PL IRQ (3-bit done from multicore_top_vd100.irq_done)
#   - GEM0 / GEM1: 千兆以太网 PHY (host runtime 用)
# =============================================================================
proc set_ps_config {bd_cell_name} {
set_property -dict [list \
    CONFIG.DESIGN_MODE {0} \
    CONFIG.PS_PMC_CONFIG { \
      DESIGN_MODE {0} \
      PS_BOARD_INTERFACE {Custom} \
      PS_NUM_FABRIC_RESETS {1} \
      SMON_ALARMS {Set_Alarms_On} \
      SMON_ENABLE_TEMP_AVERAGING {0} \
      SMON_TEMP_AVERAGING_SAMPLES {0} \
      \
      PS_USE_M_AXI_FPD {1}              \
      PS_USE_FPD_AXI_NOC0 {0}           \
      PS_USE_FPD_AXI_NOC1 {0}           \
      PS_USE_NOC_LPD_AXI0 {0}           \
      PS_USE_PMCPL_CLK0 {1}             \
      PS_USE_PMCPL_CLK1 {0}             \
      \
      PS_GEM_EMIO_ENABLE {0}            \
      PS_GEN_IPI0_ENABLE {0}            \
      PS_PL_CONNECTIVITY_MODE {Custom}  \
    } \
  ] [get_bd_cells $bd_cell_name]
}
