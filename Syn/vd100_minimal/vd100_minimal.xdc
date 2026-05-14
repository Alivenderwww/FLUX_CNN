# =============================================================================
# vd100_minimal.xdc  --  最小 board 约束
#
# 只需要 sys_clk diff (200 MHz). DDR4/Ethernet/USB 全部不用, 不绑定.
# =============================================================================

# 板载 200 MHz 差分时钟 (LVDS15)
set_property PACKAGE_PIN AB23 [get_ports {sys_clk_p[0]}]
set_property PACKAGE_PIN AC23 [get_ports {sys_clk_n[0]}]
set_property IOSTANDARD LVDS15 [get_ports {sys_clk_p[0]}]
set_property IOSTANDARD LVDS15 [get_ports {sys_clk_n[0]}]
create_clock -period 5.000 -name sys_clk -waveform {0.000 2.500} [get_ports {sys_clk_p[0]}]

set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
