# =============================================================================
# gen_axi_dm_vd100.tcl  --  VD100 工程内 native 创建 axi_dm IP (不依赖 K325T)
#
# 跟 Syn/gen_axi_datamover.tcl (K325T 版) 配置完全一致, 但创建在当前 VD100 工程
# (vd100_resnet11.xpr) 内, native targeting VE2302, 一遍综合到位.
#
# 用法 (Vivado GUI Tcl Console, 当前已 open VD100 工程):
#   source C:/_Project/FLUX_CNN/Syn/vd100_bd/gen_axi_dm_vd100.tcl
#
# 跑完: axi_dm IP 加进 sources_1, 已 generate + synth, 顶层综合直接用.
# =============================================================================

set IP_DM axi_dm

# 如果已存在 (可能是 K325T retarget 的, 不能用), 先 delete
if {[llength [get_ips -quiet $IP_DM]] > 0} {
    set is_locked [get_property IS_LOCKED [get_ips $IP_DM]]
    set ip_part   [get_property PART [get_ips $IP_DM]]
    set cur_part  [get_property PART [current_project]]
    if {$is_locked || $ip_part ne $cur_part} {
        puts "  - existing axi_dm IP locked or wrong part ($ip_part vs $cur_part), deleting"
        delete_ips $IP_DM
    } else {
        puts "  - axi_dm already exists for VE2302, reusing"
    }
}

# 创建 (如果还没有)
if {[llength [get_ips -quiet $IP_DM]] == 0} {
    create_ip -name axi_datamover -vendor xilinx.com -library ip \
              -module_name $IP_DM
    puts "  + IP $IP_DM created (axi_datamover, native VE2302)"
}

# 配置 IP 参数 (跟 K325T gen_axi_datamover.tcl 完全一致)
# 双向 MM2S+S2MM Full, 128-bit data, 32-bit addr, BTT 23-bit, DRE on
set_property -dict {
    CONFIG.c_include_mm2s            Full
    CONFIG.c_include_s2mm            Full
    CONFIG.c_addr_width              32
    CONFIG.c_m_axi_mm2s_addr_width   32
    CONFIG.c_m_axi_mm2s_data_width   128
    CONFIG.c_m_axi_mm2s_id_width     2
    CONFIG.c_m_axis_mm2s_tdata_width 128
    CONFIG.c_mm2s_btt_used           23
    CONFIG.c_include_mm2s_dre        true
    CONFIG.c_mm2s_burst_size         256
    CONFIG.c_mm2s_include_sf         false
    CONFIG.c_m_axi_s2mm_addr_width   32
    CONFIG.c_m_axi_s2mm_data_width   128
    CONFIG.c_m_axi_s2mm_id_width     2
    CONFIG.c_s_axis_s2mm_tdata_width 128
    CONFIG.c_s2mm_btt_used           23
    CONFIG.c_include_s2mm_dre        true
    CONFIG.c_s2mm_burst_size         256
    CONFIG.c_s2mm_include_sf         false
} [get_ips $IP_DM]

# Generate synth + sim outputs (native VE2302, 不需要 retarget)
puts "  + generating axi_dm output products (synth + sim)..."
generate_target -force {synthesis instantiation_template} [get_ips $IP_DM]

# Synth IP (5-10 min)
puts "  + synth_ip axi_dm (this takes 5-10 min)..."
synth_ip [get_ips $IP_DM]

puts ""
puts "============================================================"
puts " axi_dm IP ready in VD100 project. Now relaunch synth_1:"
puts "   reset_run synth_1"
puts "   launch_runs synth_1 -jobs 8"
puts "   wait_on_run synth_1"
puts "============================================================"
