open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
puts "=== sources_1 fileset files ==="
foreach f [get_files -of_objects [get_filesets sources_1]] {
    if {[string match "*vd100*" $f] || [string match "*multicore*" $f]} {
        puts "  $f"
    }
}
open_bd_design [get_files design_1.bd]
puts ""
puts "=== BD cells ==="
foreach c [get_bd_cells] { puts "  $c [get_property VLNV $c]" }
puts ""
puts "=== u_mc_vd100 pins (if exists) ==="
foreach p [get_bd_pins -quiet u_mc_vd100/*] { puts "  $p" }
foreach p [get_bd_intf_pins -quiet u_mc_vd100/*] { puts "  intf: $p" }
exit
