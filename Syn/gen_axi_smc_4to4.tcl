# =============================================================================
# gen_axi_smc_4to4.tcl  --  生成 multicore_top_smc.sv 用的 4M↔4S AXI Crossbar IP
#
# Phase 7 SMC + NUMA: 4 ConvCore (axi master) 通过 axi_smc_4to4 IP 路由到 4 mem
# (axi slave). IP 按 awaddr/araddr 高 8 bit 解码到 mem_id, 跟 sim model
# axi_crossbar_4to4_sim.sv 等价 (但综合质量高, 真硬件能用).
#
# 地址映射 (跟 multicore_top_smc.sv 内 SMC layout 一致):
#   M00  mem[0]:  0x0000_0000 - 0x00FF_FFFF  (16 MB, ADDR_WIDTH=24)
#   M01  mem[1]:  0x0100_0000 - 0x01FF_FFFF
#   M02  mem[2]:  0x0200_0000 - 0x02FF_FFFF
#   M03  mem[3]:  0x0300_0000 - 0x03FF_FFFF
#
# 跑法 (Windows PowerShell):
#   cd c:\_Project\FLUX_CNN
#   vivado -mode batch -source Syn/gen_axi_smc_4to4.tcl -nojournal -nolog
#
# 输出:
#   Syn/ip_managed/ip_managed.gen/sources_1/ip/axi_smc_4to4/  (跟 axi_4to5 共用工程)
#   Syn/ip_smc_summary.txt                                     (xci 路径摘要)
# =============================================================================

set NUM_CORES   4
set PART        xc7k325tffg900-2
set PROJ_DIR    C:/_Project/FLUX_CNN/Syn/ip_managed
set RPT_FILE    C:/_Project/FLUX_CNN/Syn/ip_smc_summary.txt
set IP_NAME     axi_smc_4to4

# -----------------------------------------------------------------------------
# 复用已存在的 ip_managed 工程 (axi_4to5 / axi_dm 也在里面)
# -----------------------------------------------------------------------------
set _need_fresh 0
if {[file exists "$PROJ_DIR/ip_managed.xpr"]} {
    puts "==> reusing project at $PROJ_DIR"
    if {[catch {open_project "$PROJ_DIR/ip_managed.xpr"} err]} {
        puts "==> open_project failed: $err"
        puts "==> recreating project"
        set _need_fresh 1
    } else {
        # 清理 stale file refs
        foreach f [get_files] {
            if {![file exists $f]} {
                puts "==> removing stale ref $f"
                remove_files -quiet $f
            }
        }
    }
} else {
    set _need_fresh 1
}
if {$_need_fresh} {
    puts "==> creating fresh project at $PROJ_DIR"
    create_project -force -part $PART ip_managed $PROJ_DIR -ip
    set_property target_simulator       ModelSim          [current_project]
    set_property simulator_language     Mixed             [current_project]
}

# -----------------------------------------------------------------------------
# 删除所有 axi_smc_4to4* IP 实例 (含 stale _1 _2 等, 避免 Vivado 创建重复)
# -----------------------------------------------------------------------------
foreach old_ip [get_ips -quiet axi_smc_4to4*] {
    puts "==> removing existing IP: $old_ip"
    set xci [get_property IP_FILE $old_ip]
    export_ip_user_files -of_objects $old_ip -no_script -reset -force -quiet
    remove_files -quiet $xci
}
# 删 IP 目录 (vivado 不会自动清)
foreach pat {axi_smc_4to4 axi_smc_4to4_1 axi_smc_4to4_2 axi_smc_4to4_3} {
    foreach d [list "$PROJ_DIR/ip_managed.srcs/sources_1/ip/$pat" \
                    "$PROJ_DIR/ip_managed.gen/sources_1/ip/$pat"] {
        if {[file exists $d]} {
            puts "==> rm -rf $d"
            file delete -force $d
        }
    }
}

# =============================================================================
# IP: axi_smc_4to4 = AXI4 4 SI : 4 MI Crossbar (SAMD)
#   - SI 端: 4 ConvCore axi master (CORE_BUS_ID = 4 bit, 但 ID_WIDTH 设 6 含核 tag)
#   - MI 端: 4 mem axi slave (各 16 MB region by addr[25:24]=mem_id)
#   - 跟 multicore_top.sv 用的 axi_4to5 IP 同 family, 但 NUM_MI=4 + 不同 region
# =============================================================================
puts "==> creating $IP_NAME (4 SI : 4 MI, 4 × 16 MB region)"
create_ip -name axi_crossbar -vendor xilinx.com -library ip \
          -module_name $IP_NAME

# ID_WIDTH = CORE_BUS_ID + log2(NUM_SI) = 4 + 2 = 6
set ID_W [expr {4 + int(ceil(log($NUM_CORES) / log(2)))}]

set smc_dict [list \
    CONFIG.NUM_SI            $NUM_CORES \
    CONFIG.NUM_MI            $NUM_CORES \
    CONFIG.PROTOCOL          AXI4 \
    CONFIG.CONNECTIVITY_MODE SAMD \
    CONFIG.ADDR_WIDTH        32 \
    CONFIG.DATA_WIDTH        128 \
    CONFIG.ID_WIDTH          $ID_W \
    CONFIG.STRATEGY          2 \
    CONFIG.R_REGISTER        8 \
]
# STRATEGY=2 = MaxPerformance (vs default 0 = AreaOptimized)
# R_REGISTER=8 = SRL FIFO with full throughput (vs 0=passthrough, 1=register slice)

# 4 个 MI region 各 16 MB + 加大 outstanding depth (默认 2/4 太小限制并发)
#   S<i>_WRITE/READ_ACCEPTANCE: SI 端能 outstanding 数 (默认 2 → 16)
#   M<i>_WRITE/READ_ISSUING:    MI 端 issuing 数      (默认 4 → 16)
for {set i 0} {$i < $NUM_CORES} {incr i} {
    set base [format 0x%013X [expr {$i * 0x01000000}]]
    set m_id [format "%02d" $i]
    set s_id [format "%02d" $i]
    lappend smc_dict CONFIG.M${m_id}_A00_BASE_ADDR  $base
    lappend smc_dict CONFIG.M${m_id}_A00_ADDR_WIDTH 24
    # MI 端 outstanding: 让 axi_dm.MM2S/S2MM cmd FIFO 内多 cmd 真并发
    lappend smc_dict CONFIG.M${m_id}_WRITE_ISSUING   16
    lappend smc_dict CONFIG.M${m_id}_READ_ISSUING    16
    # SI 端 outstanding: 让 ConvCore 内多 outstanding 真并发
    lappend smc_dict CONFIG.S${s_id}_WRITE_ACCEPTANCE 16
    lappend smc_dict CONFIG.S${s_id}_READ_ACCEPTANCE  16
}

set_property -dict $smc_dict [get_ips $IP_NAME]

# =============================================================================
# 生成 synth + sim 输出
# =============================================================================
puts "==> generate_target for $IP_NAME"
generate_target {synthesis simulation instantiation_template} [get_ips $IP_NAME]

# =============================================================================
# 写摘要 (给 sim run.tcl 用 -- 包括 xci 跟 sim source 路径)
# =============================================================================
set xci_file [get_property IP_FILE [get_ips $IP_NAME]]
set src_dir  [file dirname $xci_file]
set gen_dir  [string map {.srcs .gen} $src_dir]

set fp [open $RPT_FILE w]
puts $fp "# axi_smc_4to4 IP 生成摘要"
puts $fp "# Generated: [clock format [clock seconds] -format {%Y-%m-%d %H:%M:%S}]"
puts $fp ""
puts $fp "IP_NAME      = $IP_NAME"
puts $fp "NUM_SI       = $NUM_CORES"
puts $fp "NUM_MI       = $NUM_CORES"
puts $fp "ADDR_WIDTH   = 32"
puts $fp "DATA_WIDTH   = 128"
puts $fp "ID_WIDTH     = $ID_W"
puts $fp ""
puts $fp "xci_file     = $xci_file"
puts $fp "src_dir      = $src_dir"
puts $fp "gen_dir      = $gen_dir"
puts $fp ""
puts $fp "# 地址映射:"
for {set i 0} {$i < $NUM_CORES} {incr i} {
    set base [format 0x%08X [expr {$i * 0x01000000}]]
    set high [format 0x%08X [expr {($i + 1) * 0x01000000 - 1}]]
    puts $fp "   M${i}: $base - $high  (16 MB → mem\[$i\])"
}
close $fp

# 打印关键 CONFIG 确认
puts ""
puts "==> $IP_NAME CONFIG:"
foreach prop [lsort [list_property [get_ips $IP_NAME] CONFIG.*]] {
    if {[regexp {NUM_|PROTOCOL|ADDR_WIDTH|DATA_WIDTH|ID_WIDTH|BASE_ADDR} $prop]} {
        puts "   $prop = [get_property $prop [get_ips $IP_NAME]]"
    }
}

puts ""
puts "==> done. Summary at $RPT_FILE"
puts "==> sim source file 在: $gen_dir/$IP_NAME/sim/$IP_NAME.v"
