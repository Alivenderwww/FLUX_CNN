open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]
puts "=== Try create axis_ila_intf cell ==="
catch {delete_bd_objs [get_bd_cells -quiet ila_pl0]}
set ila [create_bd_cell -type ip -vlnv xilinx.com:ip:axis_ila_intf:1.0 ila_test]
puts "=== ports ==="
foreach p [get_bd_pins -of_objects $ila] { puts "  pin: $p" }
foreach p [get_bd_intf_pins -of_objects $ila] { puts "  intf: $p" }
puts "=== params ==="
foreach kv [list_property $ila] {
    if {[string match "CONFIG.*" $kv]} {
        puts "  $kv = [get_property $kv $ila]"
    }
}
catch {delete_bd_objs $ila}
exit
