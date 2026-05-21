# 查 BD 里 axi_noc_0 的 SI/MI 配置 + 当前接线 + 地址映射
open_project C:/_Project/FLUX_CNN/Syn/vd100_minimal/output/ps_hello.xpr
open_bd_design [get_files design_1.bd]

puts "===== axi_noc_0 cell properties ====="
foreach prop {NUM_SI NUM_MI NUM_NSI NUM_NMI NUM_CLKS} {
    set v [get_property -quiet "CONFIG.$prop" [get_bd_cells axi_noc_0]]
    puts "  $prop = $v"
}

puts ""
puts "===== axi_noc_0 intf pins ====="
foreach p [get_bd_intf_pins -of [get_bd_cells axi_noc_0]] {
    set net [get_bd_intf_nets -quiet -of $p]
    puts "  $p  net=$net"
}

puts ""
puts "===== addr_spaces on CIPS / ConvCore reachable ====="
foreach as [get_bd_addr_spaces -quiet] {
    puts "  $as"
}

puts ""
puts "===== DDR addr_segs (slave) ====="
foreach seg [get_bd_addr_segs -quiet -filter {USAGE == memory}] {
    set sp  [get_property OFFSET $seg]
    set rng [get_property RANGE $seg]
    puts "  $seg  off=$sp  range=$rng"
}

puts ""
puts "===== 当前 design 顶层 cells ====="
foreach c [get_bd_cells] {
    puts "  $c  ([get_property VLNV $c])"
}

exit 0
