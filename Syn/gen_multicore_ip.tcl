# =============================================================================
# gen_multicore_ip.tcl  --  生成多核 wrapper 用的两个 IP
#
#   1. axi_NtoM       : AXI4 N SI : (N+1) MI crossbar
#                       SI: N 个核的 AXI4 master 出口 (axi_m_mux 聚合后)
#                       MI: 1 个 DDR slave + N 个核 IFB AXI4 slave (跨核 push 入口)
#                       地址映射:
#                         MI[0]   DDR        0x0000_0000 - 0x7FFF_FFFF (31 bit, 2 GB)
#                         MI[i+1] Core[i] IFB 0x8000_0000 + i*0x1000_0000 (28 bit, 256 MB region)
#                       (M2 跨核 SRAM 直送: producer ODMA_DST_BASE = 0x8000_0000+consumer_id*0x1000_0000)
#   2. axi_lite_1toN  : AXI4-Lite 1xN crossbar, 1 SI : N MI
#                       host 单 AXI-Lite 分发到 N 核的 CSR
#                       地址布局: core[i] CSR 在 [i × 0x1000, i × 0x1000 + 0xFFF]
#
# 想换核数, 改 NUM_CORES 重跑即可 (会覆盖现有 IP 实例).
#
# 跑法:
#   vivado -mode batch -source gen_multicore_ip.tcl -nojournal -nolog
#   (或 Tcl Console 里 source)
#
# 输出: 跟 axi_dm 共用同一个 ip_managed/ 工程
# =============================================================================

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#   核数 (改这里就行, 然后重跑)
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
set NUM_CORES 2

set PART        xc7k325tffg900-2
set PROJ_DIR    C:/_Project/FLUX_CNN/Syn/ip_managed
set RPT_FILE    C:/_Project/FLUX_CNN/Syn/ip_multicore_summary.txt

# -----------------------------------------------------------------------------
# 复用已存在的 ip_managed 工程 (axi_dm 也在里面)
# -----------------------------------------------------------------------------
if {[file exists "$PROJ_DIR/ip_managed.xpr"]} {
    puts "==> reusing project at $PROJ_DIR"
    open_project "$PROJ_DIR/ip_managed.xpr"
} else {
    puts "==> creating fresh project at $PROJ_DIR"
    create_project -force -part $PART ip_managed $PROJ_DIR -ip
    set_property target_simulator       ModelSim          [current_project]
    set_property simulator_language     Mixed             [current_project]
}

# -----------------------------------------------------------------------------
# 删除旧的 IP 实例 (各种 NUM_CORES 历史版本 + 旧拓扑 axi_Nto1)
# -----------------------------------------------------------------------------
set stale_ips [list]
foreach n {2 3 4 8} {
    lappend stale_ips axi_${n}to1
    lappend stale_ips axi_${n}to[expr {$n + 1}]
    lappend stale_ips axi_lite_1to${n}
}
foreach old $stale_ips {
    if {[llength [get_ips -quiet $old]] > 0} {
        puts "==> removing stale IP $old"
        set xci [get_property IP_FILE [get_ips $old]]
        export_ip_user_files -of_objects [get_ips $old] -no_script -reset -force -quiet
        remove_files -quiet $xci
    }
}

# =============================================================================
# IP 1: axi_${N}to${N+1}  -- AXI4 N SI : (N+1) MI crossbar
#   - SI 端: N 个核 axi_m_mux 出口 (BUS_DATA_W=128, ID=4)
#   - MI 端: MI[0]=DDR (28 bit BASE 0x80000000-0xFFFFFFFF 给 IFB region 让出空间;
#           DDR 实际 0x0000_0000-0x7FFF_FFFF, 31 bit 容量足够)
#           MI[i+1]=Core[i] IFB AXI4 slave (28 bit region @ 0x8000_0000+i*0x1000_0000)
#   - 拓扑必须 SAMD (Shared-Address Multi-Data); SASD 不支持多 MI
#   - ID_WIDTH 5 = 核内 4 + 1 bit slave tag
# =============================================================================
set NUM_MI [expr {$NUM_CORES + 1}]
set IP_AGG axi_${NUM_CORES}to${NUM_MI}
puts "==> create $IP_AGG (NUM_SI=$NUM_CORES NUM_MI=$NUM_MI)"
create_ip -name axi_crossbar -vendor xilinx.com -library ip \
          -module_name $IP_AGG

set agg_dict [list \
    CONFIG.NUM_SI         $NUM_CORES \
    CONFIG.NUM_MI         $NUM_MI \
    CONFIG.PROTOCOL       AXI4 \
    CONFIG.CONNECTIVITY_MODE SAMD \
    CONFIG.ADDR_WIDTH     32 \
    CONFIG.DATA_WIDTH     128 \
    CONFIG.ID_WIDTH       5 \
]

# MI[0] = DDR: BASE 0x00000000, 31 bit (2 GB) - 给 IFB region 让出 0x80000000+
lappend agg_dict CONFIG.M00_A00_BASE_ADDR 0x0000000000000000
lappend agg_dict CONFIG.M00_A00_ADDR_WIDTH 31

# MI[i+1] = Core[i] IFB: BASE 0x80000000 + i*0x10000000, 28 bit (256 MB region/core)
for {set i 0} {$i < $NUM_CORES} {incr i} {
    set mi_idx [expr {$i + 1}]
    set base   [format 0x%013X [expr {0x80000000 + $i * 0x10000000}]]
    lappend agg_dict CONFIG.M[format "%02d" $mi_idx]_A00_BASE_ADDR $base
    lappend agg_dict CONFIG.M[format "%02d" $mi_idx]_A00_ADDR_WIDTH 28
}

set_property -dict $agg_dict [get_ips $IP_AGG]

# =============================================================================
# IP 2: axi_lite_1to${N}  -- AXI4-Lite 1:N crossbar
#   - 1 SI (host AXI-Lite, (12+log2 N) bit 地址)
#   - N MI (各核 cfg_regs / axi_lite_csr 入口, 12-bit 地址)
#   - 数据 32 bit
#   - 路由: 高 (log2 N) 位选核, 低 12 位选寄存器
# =============================================================================
set IP_CSR axi_lite_1to${NUM_CORES}

# 算 host 端地址位宽: 12 + log2(NUM_CORES)
set host_addr_w [expr {12 + int(ceil(log($NUM_CORES) / log(2)))}]
if {$NUM_CORES == 1} { set host_addr_w 12 }

puts "==> create $IP_CSR (NUM_SI=1 NUM_MI=$NUM_CORES, host_addr=${host_addr_w}b)"
create_ip -name axi_crossbar -vendor xilinx.com -library ip \
          -module_name $IP_CSR

set csr_dict [list \
    CONFIG.NUM_SI         1 \
    CONFIG.NUM_MI         $NUM_CORES \
    CONFIG.PROTOCOL       AXI4LITE \
    CONFIG.CONNECTIVITY_MODE SAMD \
    CONFIG.ADDR_WIDTH     $host_addr_w \
    CONFIG.DATA_WIDTH     32 \
    CONFIG.ID_WIDTH       0 \
]

# 各 MI 端口地址段: core[i] 在 i*0x1000, 12-bit 宽
for {set i 0} {$i < $NUM_CORES} {incr i} {
    set base [format 0x%013X [expr {$i * 0x1000}]]
    lappend csr_dict CONFIG.M[format "%02d" $i]_A00_BASE_ADDR $base
    lappend csr_dict CONFIG.M[format "%02d" $i]_A00_ADDR_WIDTH 12
}
set_property -dict $csr_dict [get_ips $IP_CSR]

# =============================================================================
# 生成 synth + sim 输出
# =============================================================================
foreach ip [list $IP_AGG $IP_CSR] {
    puts "==> generate_target for $ip ..."
    generate_target {synthesis simulation instantiation_template} [get_ips $ip]
}

# =============================================================================
# 写摘要给 run_syn.tcl 用
# =============================================================================
set fp [open $RPT_FILE w]
puts $fp "# Multicore IP generation summary"
puts $fp "# Generated: [clock format [clock seconds] -format {%Y-%m-%d %H:%M:%S}]"
puts $fp "# NUM_CORES = $NUM_CORES"
puts $fp ""
foreach ip [list $IP_AGG $IP_CSR] {
    set xci_file [get_property IP_FILE [get_ips $ip]]
    set src_dir  [file dirname $xci_file]
    set gen_dir  [string map {.srcs .gen} $src_dir]
    puts $fp "## $ip"
    puts $fp "xci  : $xci_file"
    puts $fp "src  : $src_dir"
    puts $fp "gen  : $gen_dir"
    puts $fp ""
}
close $fp
puts "==> summary at $RPT_FILE"

# 打印关键 CONFIG 确认
foreach ip [list $IP_AGG $IP_CSR] {
    puts ""
    puts "==> $ip CONFIG:"
    foreach prop [lsort [list_property [get_ips $ip] CONFIG.*]] {
        if {[regexp {NUM_|PROTOCOL|ADDR_WIDTH|DATA_WIDTH|ID_WIDTH|BASE_ADDR} $prop]} {
            puts "   $prop = [get_property $prop [get_ips $ip]]"
        }
    }
}

puts "==> done."
