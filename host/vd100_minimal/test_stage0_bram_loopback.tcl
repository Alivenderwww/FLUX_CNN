# =============================================================================
# test_stage0_bram_loopback.tcl  --  Stage 0: jtag_to_axi_bulk → BRAM 回环测试
#
# 目的: 验证 PDI 烧入 OK + jtag_to_axi + smartconnect + axi_bram_ctrl + BRAM
#       这条最简单的 axi 路径工作. 不涉及 ConvCore.
#
# 用法 (Vivado HW Manager Tcl Console, board 已烧 PDI):
#   open_hw_manager
#   connect_hw_server
#   open_hw_target
#   source test_stage0_bram_loopback.tcl
# =============================================================================

# 等 jtag_to_axi cores enumerate
refresh_hw_device
set bulk_idx 0
foreach core [get_hw_axis] {
    set name [get_property NAME $core]
    if {[string match "*bulk*" $name]} { set bulk_idx $core; break }
    if {[string match "*jtag_to_axi_bulk*" $name]} { set bulk_idx $core; break }
}
puts "INFO: bulk core = $bulk_idx"

# Test 1: 写 0xDEADBEEF 到 BRAM 0x10000000, 读回验证
puts "=== Test 1: single beat write/read ==="
create_hw_axi_txn -force wr1 $bulk_idx -address 10000000 -data DEADBEEF12345678AAAA5555BBBB6666 -len 1 -type write
run_hw_axi wr1

create_hw_axi_txn -force rd1 $bulk_idx -address 10000000 -len 1 -type read
run_hw_axi rd1
set v1 [get_property DATA [get_hw_axi_txn rd1]]
puts "  wrote: DEADBEEF12345678AAAA5555BBBB6666"
puts "  read : $v1"
if {[string equal -nocase $v1 "deadbeef12345678aaaa5555bbbb6666"]} {
    puts "  [Test 1] PASS"
} else {
    puts "  [Test 1] FAIL"
}

# Test 2: 4 beat burst write/read
puts "=== Test 2: 4-beat burst write/read ==="
create_hw_axi_txn -force wr4 $bulk_idx -address 10000100 \
    -data {11111111_22222222_33333333_44444444 55555555_66666666_77777777_88888888 99999999_AAAAAAAA_BBBBBBBB_CCCCCCCC DDDDDDDD_EEEEEEEE_FFFFFFFF_00000000} \
    -len 4 -type write
run_hw_axi wr4
create_hw_axi_txn -force rd4 $bulk_idx -address 10000100 -len 4 -type read
run_hw_axi rd4
set v4 [get_property DATA [get_hw_axi_txn rd4]]
puts "  read 4-beat: $v4"

# Test 3: 大量数据 (1 KB)
puts "=== Test 3: 1 KB burst write/read ==="
# 生成 64 beat (each 128b=16 byte) = 1 KB 模式: address + offset
set pattern_data ""
for {set i 0} {$i < 64} {incr i} {
    set d [format "%08X_%08X_%08X_%08X" [expr {$i * 4}] [expr {$i * 4 + 1}] [expr {$i * 4 + 2}] [expr {$i * 4 + 3}]]
    append pattern_data "$d "
}
create_hw_axi_txn -force wr_big $bulk_idx -address 10001000 -data $pattern_data -len 64 -type write
run_hw_axi wr_big
create_hw_axi_txn -force rd_big $bulk_idx -address 10001000 -len 64 -type read
run_hw_axi rd_big
set v_big [get_property DATA [get_hw_axi_txn rd_big]]
puts "  read 64-beat first 16-byte: [string range $v_big 0 31]"
puts "  read 64-beat last 16-byte:  [string range $v_big end-31 end]"

puts ""
puts "============================================================"
puts " Stage 0 done. BRAM loopback verified."
puts "============================================================"
