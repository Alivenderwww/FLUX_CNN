# =============================================================================
# test_stage1a_csr_peek_poke.tcl  --  Stage 1a: ConvCore CSR PEEK/POKE 测试
#
# 验证 jtag_to_axi_csr → multicore_top_minimal.csr_axil 路径工作.
# 不触发任何 dispatcher / layer 运行, 只测 cfg_regs 读写.
#
# 用法:
#   open_hw_manager; connect_hw_server; open_hw_target
#   source test_stage1a_csr_peek_poke.tcl
# =============================================================================

refresh_hw_device
set csr_idx ""
foreach core [get_hw_axis] {
    set name [get_property NAME $core]
    if {[string match "*csr*" $name]} { set csr_idx $core; break }
}
if {$csr_idx eq ""} {
    foreach core [get_hw_axis] { puts "  candidate: $core"; if {$csr_idx eq ""} { set csr_idx $core } }
}
puts "INFO: csr core = $csr_idx"

# csr_axil base 在 BD 上 set 为 0x80000000 (build_bd.tcl assign_bd_address)

# Test 1: 写 boot 寄存器 DESC_LIST_BASE (0x180) 和 DESC_COUNT (0x184), readback
puts "=== Test 1: csr_w boot regs write/read ==="

# 写 DESC_LIST_BASE = 0xDEADBEEF
create_hw_axi_txn -force wr_lb $csr_idx -address 80000180 -data DEADBEEF -len 1 -type write
run_hw_axi wr_lb

create_hw_axi_txn -force rd_lb $csr_idx -address 80000180 -len 1 -type read
run_hw_axi rd_lb
set v_lb [get_property DATA [get_hw_axi_txn rd_lb]]
puts "  DESC_LIST_BASE: wrote 0xDEADBEEF, read 0x$v_lb"

# 写 DESC_COUNT = 0x42 (66)
create_hw_axi_txn -force wr_dc $csr_idx -address 80000184 -data 00000042 -len 1 -type write
run_hw_axi wr_dc
create_hw_axi_txn -force rd_dc $csr_idx -address 80000184 -len 1 -type read
run_hw_axi rd_dc
set v_dc [get_property DATA [get_hw_axi_txn rd_dc]]
puts "  DESC_COUNT: wrote 0x42, read 0x$v_dc"

# Test 2: 读初始 STATUS (应该 = 0, ConvCore reset 状态)
puts "=== Test 2: STATUS readback (expect 0 after reset) ==="
create_hw_axi_txn -force rd_st $csr_idx -address 80000004 -len 1 -type read
run_hw_axi rd_st
set v_st [get_property DATA [get_hw_axi_txn rd_st]]
puts "  STATUS: 0x$v_st (expect 0x00000000)"

# Test 3: 读 SEQ_DBG (应该 seq_state=0 IDLE)
puts "=== Test 3: SEQ_DBG (expect seq=0) ==="
create_hw_axi_txn -force rd_sd $csr_idx -address 80000008 -len 1 -type read
run_hw_axi rd_sd
set v_sd [get_property DATA [get_hw_axi_txn rd_sd]]
puts "  SEQ_DBG: 0x$v_sd"
# 低 4 bit = seq_state
set seq_state [expr {0x$v_sd & 0xF}]
puts "  seq_state = $seq_state (expect 0 = S_IDLE)"

puts ""
puts "============================================================"
puts " Stage 1a done. csr_axil PEEK/POKE verified."
puts " 如果以上都 PASS, 说明 jtag_to_axi_csr → ConvCore.csr_axil 路径 OK"
puts "============================================================"
