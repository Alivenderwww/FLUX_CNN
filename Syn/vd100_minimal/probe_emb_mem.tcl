# probe emb_mem_gen IP available properties
open_project C:/_Project/FLUX_CNN/Syn/vd100_minimal/output/ps_hello.xpr
open_bd_design [get_files design_1.bd]

puts "===== emb_mem_gen_0 CONFIG properties ====="
foreach p [list_property [get_bd_cells emb_mem_gen_0] CONFIG.*] {
    set val [get_property $p [get_bd_cells emb_mem_gen_0]]
    puts "  $p = $val"
}

puts ""
puts "===== axi_bram_ctrl_0 CONFIG properties (auto-prop from emb_mem_gen) ====="
foreach p [list_property [get_bd_cells axi_bram_ctrl_0] CONFIG.*] {
    set val [get_property $p [get_bd_cells axi_bram_ctrl_0]]
    puts "  $p = $val"
}
exit 0
