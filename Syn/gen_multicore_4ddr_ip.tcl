# =============================================================================
# gen_multicore_4ddr_ip.tcl  --  PoC: 4 SI / 8 MI crossbar 测多 DDR 端口加速比
#
#   axi_4to8 拓扑 (实验性, 不替换 axi_4to5):
#     SI[0..3] = 4 个核 axi_m_mux 出口
#     MI[0..3] = 4 个独立 DDR slot   (各 256MB region @ 0x0/0x10000000/0x20000000/0x30000000)
#     MI[4..7] = 4 个 Core IFB push  (各 256MB region @ 0x80000000+i*0x10000000)
#
#   PoC 目的: 验证 Patch s2d (DDR-bw bound) 在 4 个独立 DDR mem 下能不能 ~4× 加速.
#     如果加速明显 → 上 multi-DDR 真机 (VCU118 4×DDR4 / U280 HBM2) 有回报.
#     如果加速不明显 → 别投, 退回 SDP 流水线/DSP attribute 等保守优化.
#
# 跑法:
#   先确保 ip_managed 工程 + axi_dm 跟 axi_4to5/axi_lite_1to4 都已生成
#   (gen_axi_datamover.tcl + gen_multicore_ip.tcl).
#   再:
#     vivado -mode batch -source gen_multicore_4ddr_ip.tcl -nojournal -nolog
# =============================================================================

set PART        xc7k325tffg900-2
set PROJ_DIR    C:/_Project/FLUX_CNN/Syn/ip_managed

if {![file exists "$PROJ_DIR/ip_managed.xpr"]} {
    puts "ERROR: ip_managed project not found at $PROJ_DIR. Run gen_axi_datamover.tcl first."
    exit 1
}

puts "==> reusing project at $PROJ_DIR"
if {[catch {open_project "$PROJ_DIR/ip_managed.xpr"} err]} {
    puts "ERROR: open_project failed: $err"
    exit 1
}
foreach f [get_files] {
    if {![file exists $f]} {
        puts "==> removing stale file ref $f"
        remove_files -quiet $f
    }
}

# 删旧 axi_4to8 (若存在), create_ip 才不会 _1 后缀
foreach old {axi_4to8} {
    if {[llength [get_ips -quiet $old]] > 0} {
        puts "==> removing stale IP $old"
        set xci [get_property IP_FILE [get_ips $old]]
        export_ip_user_files -of_objects [get_ips $old] -no_script -reset -force -quiet
        remove_files -quiet $xci
    }
}

# =============================================================================
# axi_4to8: 4 SI / 8 MI crossbar
# =============================================================================
set IP_NAME axi_4to8
set NUM_SI 4
set NUM_MI 8
puts "==> create $IP_NAME (NUM_SI=$NUM_SI NUM_MI=$NUM_MI)"
create_ip -name axi_crossbar -vendor xilinx.com -library ip -module_name $IP_NAME

set agg_dict [list \
    CONFIG.NUM_SI         $NUM_SI \
    CONFIG.NUM_MI         $NUM_MI \
    CONFIG.PROTOCOL       AXI4 \
    CONFIG.CONNECTIVITY_MODE SAMD \
    CONFIG.ADDR_WIDTH     32 \
    CONFIG.DATA_WIDTH     128 \
    CONFIG.ID_WIDTH       6 \
]

# MI[0..3] = 4 个 DDR slot (各 256 MB, BASE 0x0/0x10/0x20/0x30 _0000_0000)
for {set i 0} {$i < 4} {incr i} {
    set base [format 0x%013X [expr {$i * 0x10000000}]]
    lappend agg_dict CONFIG.M[format "%02d" $i]_A00_BASE_ADDR $base
    lappend agg_dict CONFIG.M[format "%02d" $i]_A00_ADDR_WIDTH 28
}
# MI[4..7] = 4 个 Core IFB region (各 256 MB, BASE 0x80/0x90/0xA0/0xB0 _0000_0000)
for {set i 0} {$i < 4} {incr i} {
    set mi_idx [expr {$i + 4}]
    set base [format 0x%013X [expr {0x80000000 + $i * 0x10000000}]]
    lappend agg_dict CONFIG.M[format "%02d" $mi_idx]_A00_BASE_ADDR $base
    lappend agg_dict CONFIG.M[format "%02d" $mi_idx]_A00_ADDR_WIDTH 28
}

set_property -dict $agg_dict [get_ips $IP_NAME]

puts "==> generate_target for $IP_NAME ..."
generate_target {synthesis simulation instantiation_template} [get_ips $IP_NAME]

puts "==> $IP_NAME CONFIG:"
foreach prop [lsort [list_property [get_ips $IP_NAME] CONFIG.*]] {
    if {[regexp {NUM_|PROTOCOL|ADDR_WIDTH|DATA_WIDTH|ID_WIDTH|BASE_ADDR} $prop]} {
        puts "   $prop = [get_property $prop [get_ips $IP_NAME]]"
    }
}

puts "==> done."
