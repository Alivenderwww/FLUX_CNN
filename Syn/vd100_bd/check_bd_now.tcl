open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]
puts ""
puts "=== ALL BD cells ==="
foreach c [get_bd_cells] { puts "  $c [get_property VLNV $c]" }
puts ""
puts "=== u_mc_vd100 ==="
set u [get_bd_cells -quiet u_mc_vd100]
if {$u eq ""} { puts "  NOT FOUND" } else {
    puts "  vlnv: [get_property VLNV $u]"
    foreach p [get_bd_pins -of_objects $u]      { puts "  pin:  $p" }
    foreach p [get_bd_intf_pins -of_objects $u] { puts "  intf: $p" }
}
exit
