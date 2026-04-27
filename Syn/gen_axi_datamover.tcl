# =============================================================================
# gen_axi_datamover.tcl  --  生成 1 个 AXI DataMover IP 实例 (双向)
#
# Vivado 2023.1 的 axi_datamover v5.1 不允许完全禁用某一通道
# (c_include_mm2s / c_include_s2mm 合法值只有 Full / Basic, 无 Omit), 所以
# 用一个双向实例代替原计划的两个单向实例:
#   - MM2S 通道由 idma + wdma 共用 (外部 cmd 仲裁, data 按 tag 分流)
#   - S2MM 通道给 odma 用
#   - 内部两个通道状态机独立, 互不干扰
#
# 在 Vivado 里跑:
#   vivado -mode batch -source gen_axi_datamover.tcl -nojournal -nolog
# 或 Tcl Console:
#   source gen_axi_datamover.tcl
#
# 输出:
#   Syn/ip_managed/                                                - 托管 IP 工程
#   Syn/ip_managed/ip_managed.gen/sources_1/ip/axi_dm/             - IP 实例
#   Syn/ip_managed_summary.txt                                     - 文件清单
# =============================================================================

set PART        xc7k325tffg900-2
set PROJ_DIR    C:/_Project/FLUX_CNN/Syn/ip_managed
set SIMLIB_DIR  C:/_Project/FLUX_CNN/Syn/sim_libs
set RPT_FILE    C:/_Project/FLUX_CNN/Syn/ip_managed_summary.txt

# -----------------------------------------------------------------------------
# 创建 (或重用) 托管 IP 工程
# -----------------------------------------------------------------------------
if {[file exists "$PROJ_DIR/ip_managed.xpr"]} {
    puts "==> reusing existing project at $PROJ_DIR"
    open_project "$PROJ_DIR/ip_managed.xpr"
} else {
    puts "==> creating fresh project at $PROJ_DIR"
    create_project -force -part $PART ip_managed $PROJ_DIR -ip
    set_property target_simulator       ModelSim          [current_project]
    set_property simulator_language     Mixed             [current_project]
    set_property compxlib.modelsim_compiled_library_dir $SIMLIB_DIR [current_project]
}

# -----------------------------------------------------------------------------
# 清理旧的单向实例 (gen_axi_datamover.tcl v1 留下的 axi_dm_mm2s / axi_dm_s2mm)
# 它们 Omit 不生效, 实际两通道都 Full, 与新 axi_dm 重复
# -----------------------------------------------------------------------------
foreach old [list axi_dm_mm2s axi_dm_s2mm] {
    if {[llength [get_ips -quiet $old]] > 0} {
        puts "==> removing stale IP $old"
        remove_files -quiet [get_files -of_objects [get_ips $old]]
        export_ip_user_files -of_objects [get_ips $old] -no_script -reset -force -quiet
    }
}

# -----------------------------------------------------------------------------
# IP: axi_dm   (双向, MM2S + S2MM 都 Full)
#   - MM2S 通道: idma + wdma 共用 (外部仲裁)
#   - S2MM 通道: odma 用
#   - AXI4 MM data 128b, addr 32b
#   - AXI4-Stream data 128b (mm2s_tdata + s2mm_tdata 各 128)
#   - BTT = 23 bit  (max 8MB / cmd, 够 540×960×16 = 8.3MB 整图)
#   - DRE 双向 on (允许非对齐起始地址)
# -----------------------------------------------------------------------------
set IP_DM axi_dm
if {[llength [get_ips -quiet $IP_DM]] == 0} {
    create_ip -name axi_datamover -vendor xilinx.com -library ip \
              -module_name $IP_DM
}

# -dict 原子设置 (避免 IP 内部依赖关系把单条 set 反弹回去)
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
    CONFIG.c_mm2s_burst_size         16
    CONFIG.c_m_axi_s2mm_addr_width   32
    CONFIG.c_m_axi_s2mm_data_width   128
    CONFIG.c_m_axi_s2mm_id_width     2
    CONFIG.c_s_axis_s2mm_tdata_width 128
    CONFIG.c_s2mm_btt_used           23
    CONFIG.c_include_s2mm_dre        true
    CONFIG.c_s2mm_burst_size         16
} [get_ips $IP_DM]

# -----------------------------------------------------------------------------
# 生成 synth + sim 输出
# -----------------------------------------------------------------------------
foreach ip [list $IP_DM] {
    puts "==> generate_target for $ip ..."
    generate_target {synthesis simulation instantiation_template} [get_ips $ip]
}

# -----------------------------------------------------------------------------
# 让 Vivado 直接吐出一份 ModelSim 编译脚本 (.do)
# 路径里会包含完整 vcom/vlog 命令序列 + 正确库映射, 我们 run.tcl 里 source 就行
# -----------------------------------------------------------------------------
set EXPORT_DIR C:/_Project/FLUX_CNN/Syn/ip_sim_export
puts "==> export_simulation -> $EXPORT_DIR"
export_simulation \
    -simulator         modelsim                       \
    -lib_map_path      $SIMLIB_DIR                    \
    -directory         $EXPORT_DIR                    \
    -of_objects        [get_ips $IP_DM]               \
    -force

# -----------------------------------------------------------------------------
# 写一份摘要给 run.tcl 用 (列出 .xci 路径 + sim include 目录)
# -----------------------------------------------------------------------------
set fp [open $RPT_FILE w]
puts $fp "# AXI DataMover IP generation summary"
puts $fp "# Generated: [clock format [clock seconds] -format {%Y-%m-%d %H:%M:%S}]"
puts $fp ""
foreach ip [list $IP_DM] {
    set xci_file [get_property IP_FILE [get_ips $ip]]
    # .xci 在 .srcs/.../ip/<ip>/, sim/synth 实际产物在 .gen/.../ip/<ip>/
    set src_dir  [file dirname $xci_file]
    set gen_dir  [string map {.srcs .gen} $src_dir]
    puts $fp "## $ip"
    puts $fp "xci  : $xci_file"
    puts $fp "src  : $src_dir"
    puts $fp "gen  : $gen_dir"
    puts $fp ""
    foreach sub [list sim synth simulation hdl] {
        if {[file isdirectory "$gen_dir/$sub"]} {
            puts $fp "  $sub/:"
            foreach f [lsort [glob -nocomplain "$gen_dir/$sub/*.{v,sv,vhd,vhdl}"]] {
                puts $fp "    $f"
            }
        }
    }
    puts $fp ""
}
close $fp
puts "==> summary at $RPT_FILE"

# -----------------------------------------------------------------------------
# 报告 IP 的实际 CONFIG (debug 用, 看 set_property 落实情况)
# -----------------------------------------------------------------------------
foreach ip [list $IP_DM] {
    puts ""
    puts "==> $ip CONFIG:"
    foreach prop [lsort [list_property [get_ips $ip] CONFIG.*]] {
        if {[regexp {include|btt|data_width|addr_width|burst_size|dre} $prop]} {
            puts "   $prop = [get_property $prop [get_ips $ip]]"
        }
    }
}

puts "==> done."
