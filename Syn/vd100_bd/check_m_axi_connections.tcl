# 检查 BD 上 u_mc_vd100 / m00..m02_axi 的实际连接, 找 m00_axi_* 没源的原因
open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

puts "=== smartconnect_pl_* cells ==="
foreach c [get_bd_cells -filter {NAME =~ "smartconnect_pl_*"}] {
    puts "  cell: $c"
}

puts "=== u_mc_vd100/m0X_axi connectivity ==="
for {set i 0} {$i < 3} {incr i} {
    set m_intf [get_bd_intf_pins u_mc_vd100/m0${i}_axi]
    set nets   [get_bd_intf_nets -of_objects $m_intf]
    puts "  m0${i}_axi nets = $nets"
    if {$nets ne ""} {
        foreach n $nets {
            set pins [get_bd_intf_pins -of_objects $n]
            puts "    net $n pins = $pins"
        }
    }
}

puts "=== axi_noc_0/S00..S02_AXI connectivity ==="
for {set i 0} {$i < 3} {incr i} {
    set s_intf [get_bd_intf_pins axi_noc_0/S0${i}_AXI]
    set nets   [get_bd_intf_nets -of_objects $s_intf]
    puts "  S0${i}_AXI nets = $nets"
}

puts "=== 单独检查 m00_axi_awready/wready/arready/rvalid 这 6 个标量 pin ==="
foreach pname {awready wready bvalid arready rlast rvalid} {
    set pin [get_bd_pins u_mc_vd100/m00_axi_${pname}]
    if {$pin ne ""} {
        set net [get_bd_nets -of_objects $pin]
        puts "  m00_axi_${pname} pin=$pin net=$net"
    } else {
        puts "  m00_axi_${pname} NOT FOUND as scalar pin"
    }
}

exit 0
