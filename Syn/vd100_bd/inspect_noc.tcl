open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

puts ""
puts "=== axi_noc_0 PL S00..S02 properties ==="
for {set i 0} {$i < 3} {incr i} {
    set port [format "S%02d_AXI" $i]
    set pin [get_bd_intf_pins -quiet /axi_noc_0/$port]
    if {$pin eq ""} {
        puts "$port: pin not found"
        continue
    }
    puts "--- /axi_noc_0/$port ---"
    foreach prop {CONFIG.CATEGORY CONFIG.CONNECTIONS CONFIG.DATA_WIDTH CONFIG.READ_BW CONFIG.WRITE_BW CONFIG.NOC_PARAMS} {
        set v [get_property -quiet $prop $pin]
        if {$v ne ""} { puts "  $prop = $v" }
    }
}

puts ""
puts "=== axi_noc_0 ASSOCIATED_BUSIF for aclk0..2 ==="
for {set i 0} {$i < 3} {incr i} {
    set p [get_bd_pins /axi_noc_0/aclk$i]
    set v [get_property -quiet CONFIG.ASSOCIATED_BUSIF $p]
    puts "  aclk$i: $v"
}

puts ""
puts "=== Address segments u_mc_vd100/S00..S02 ==="
foreach as [get_bd_addr_spaces -quiet u_mc_vd100/*] {
    puts "  $as"
    foreach seg [get_bd_addr_segs -of_objects [get_bd_addr_spaces $as]] {
        set off [get_property OFFSET $seg]
        set rng [get_property RANGE $seg]
        puts "    seg: $seg  offset=$off range=$rng"
    }
}

close_project
exit
