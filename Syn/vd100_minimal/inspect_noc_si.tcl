# 查 axi_noc_0 每个 SI 的 CONNECTIONS 属性 + clk pin
open_project C:/_Project/FLUX_CNN/Syn/vd100_minimal/output/ps_hello.xpr
open_bd_design [get_files design_1.bd]

puts "===== axi_noc_0 SI properties ====="
foreach si {S00 S01 S02 S03 S04 S05} {
    set pin [get_bd_intf_pins -quiet "axi_noc_0/${si}_AXI"]
    if {$pin eq ""} { continue }
    puts ""
    puts "--- $si ---"
    foreach p {CONNECTIONS ASSOCIATED_BUSIF FREQ_HZ PROTOCOL DATA_WIDTH ADDR_WIDTH} {
        set v [get_property -quiet "CONFIG.$p" $pin]
        puts "  CONFIG.$p = $v"
    }
}

puts ""
puts "===== axi_noc_0 clock pins ====="
foreach p [get_bd_pins -of [get_bd_cells axi_noc_0] -filter {TYPE == clk}] {
    set net [get_bd_nets -quiet -of $p]
    puts "  $p  net=$net"
}

puts ""
puts "===== axi_noc_0 MC controller properties ====="
foreach p {MC_CHAN_REGION0 NUM_MC NUM_CLKS NUM_SI MC_0_FREQ MC_0_CONNECTIONS} {
    set v [get_property -quiet "CONFIG.$p" [get_bd_cells axi_noc_0]]
    if {$v ne ""} { puts "  $p = $v" }
}

exit 0
