# =============================================================================
# ps_config.tcl  --  Versal CIPS 完整 PS_PMC_CONFIG (对齐 ALINX 06_ps_hello A72 baremetal demo)
#
# 关键启用项 (含 A72 baremetal 必需的所有"系统级"配置):
#   - FPD/CCI/NOC: PS_USE_FPD_CCI_NOC + _NOC0 + LPD/PMC NoC AXI (A72 → DDR4 路径)
#   - IPI ch0..6 全启 (default subsystem 激活, A72 拿 firewall 通行证)
#   - TTC0..3 (lwIP echo 时基, BSP 自动装 xttcps)
#   - UART0 PS_MIO 16-17 (ALINX VD100 USB-UART 桥接位置)
#   - GEM0 PS_MIO 0-11 + MDIO PMC_MIO 50-51 (千兆以太网)
#   - DDR_MEMORY_MODE Connectivity to DDR via NOC (PMC cdo 跑 DDR init)
#   - DESIGN_MODE 1 (embedded design, 触发 PSM firmware + PLM cdo)
#   - SD0 (eMMC) / SD1 / QSPI / USB3 (板硬件外设, 不启会被警告)
#
# 使用: 在 BD 已存在的工程里 apply 新 PS config 后 reset_run + 重综合.
# =============================================================================

proc set_ps_config {bd_cell_name} {
    set_property -dict [list \
        CONFIG.BOOT_MODE {Custom} \
        CONFIG.DDR_MEMORY_MODE {Enable} \
        CONFIG.DESIGN_MODE {1} \
        CONFIG.IO_CONFIG_MODE {Custom} \
        CONFIG.PS_PMC_CONFIG { \
            BOOT_MODE {Custom} \
            DDR_MEMORY_MODE {Connectivity to DDR via NOC} \
            DESIGN_MODE {1} \
            IO_CONFIG_MODE {Custom} \
            PMC_QSPI_PERIPHERAL_ENABLE {1} \
            PMC_QSPI_PERIPHERAL_MODE {Dual Parallel} \
            PMC_REF_CLK_FREQMHZ {33.333333} \
            PMC_SD0 {{CD_ENABLE 0} {CD_IO {PMC_MIO 24}} {POW_ENABLE 0} {POW_IO {PMC_MIO 17}} {RESET_ENABLE 1} {RESET_IO {PMC_MIO 49}} {WP_ENABLE 0} {WP_IO {PMC_MIO 25}}} \
            PMC_SD0_DATA_TRANSFER_MODE {8Bit} \
            PMC_SD0_PERIPHERAL {{CLK_100_SDR_OTAP_DLY 0x00} {CLK_200_SDR_OTAP_DLY 0x2} {CLK_50_DDR_ITAP_DLY 0x1E} {CLK_50_DDR_OTAP_DLY 0x5} {CLK_50_SDR_ITAP_DLY 0x2C} {CLK_50_SDR_OTAP_DLY 0x5} {ENABLE 1} {IO {PMC_MIO 37 .. 49}}} \
            PMC_SD0_SLOT_TYPE {eMMC} \
            PMC_SD1 {{CD_ENABLE 1} {CD_IO {PMC_MIO 28}} {POW_ENABLE 0} {POW_IO {PMC_MIO 12}} {RESET_ENABLE 0} {RESET_IO {PMC_MIO 12}} {WP_ENABLE 0} {WP_IO {PMC_MIO 1}}} \
            PMC_SD1_PERIPHERAL {{CLK_100_SDR_OTAP_DLY 0x00} {CLK_200_SDR_OTAP_DLY 0x00} {CLK_50_DDR_ITAP_DLY 0x00} {CLK_50_DDR_OTAP_DLY 0x00} {CLK_50_SDR_ITAP_DLY 0x2C} {CLK_50_SDR_OTAP_DLY 0x4} {ENABLE 1} {IO {PMC_MIO 26 .. 36}}} \
            PMC_USE_PMC_NOC_AXI0 {1} \
            PS_BOARD_INTERFACE {Custom} \
            PS_ENET0_MDIO {{ENABLE 1} {IO {PMC_MIO 50 .. 51}}} \
            PS_ENET0_PERIPHERAL {{ENABLE 1} {IO {PS_MIO 0 .. 11}}} \
            PS_GEN_IPI0_ENABLE {1} \
            PS_GEN_IPI1_ENABLE {1} \
            PS_GEN_IPI2_ENABLE {1} \
            PS_GEN_IPI3_ENABLE {1} \
            PS_GEN_IPI4_ENABLE {1} \
            PS_GEN_IPI5_ENABLE {1} \
            PS_GEN_IPI6_ENABLE {1} \
            PS_GPIO_EMIO_PERIPHERAL_ENABLE {1} \
            PS_MIO24 {{AUX_IO 0} {DIRECTION in} {DRIVE_STRENGTH 8mA} {OUTPUT_DATA default} {PULL pullup} {SCHMITT 0} {SLEW slow} {USAGE GPIO}} \
            PS_MIO25 {{AUX_IO 0} {DIRECTION out} {DRIVE_STRENGTH 8mA} {OUTPUT_DATA default} {PULL pullup} {SCHMITT 0} {SLEW slow} {USAGE GPIO}} \
            PS_NUM_FABRIC_RESETS {1} \
            PS_PL_CONNECTIVITY_MODE {Custom} \
            PS_TTC0_PERIPHERAL_ENABLE {1} \
            PS_TTC1_PERIPHERAL_ENABLE {1} \
            PS_TTC2_PERIPHERAL_ENABLE {1} \
            PS_TTC3_PERIPHERAL_ENABLE {1} \
            PS_UART0_PERIPHERAL {{ENABLE 1} {IO {PS_MIO 16 .. 17}}} \
            PS_USB3_PERIPHERAL {{ENABLE 1} {IO {PMC_MIO 13 .. 25}}} \
            PS_USE_FPD_CCI_NOC {1} \
            PS_USE_FPD_CCI_NOC0 {1} \
            PS_USE_M_AXI_FPD {1} \
            PS_USE_NOC_LPD_AXI0 {1} \
            PS_USE_PMCPL_CLK0 {1} \
            SMON_ALARMS {Set_Alarms_On} \
            SMON_ENABLE_TEMP_AVERAGING {0} \
            SMON_TEMP_AVERAGING_SAMPLES {0} \
        } \
    ] [get_bd_cells $bd_cell_name]
}
