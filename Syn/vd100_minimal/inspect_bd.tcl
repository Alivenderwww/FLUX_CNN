# 检查现有 BD 看 clk/reset source + remaining cells
open_project C:/_Project/FLUX_CNN/Syn/vd100_minimal/output/ps_hello.xpr
open_bd_design [get_files design_1.bd]
puts "===== BD cells (after migrate failed) ====="
foreach c [get_bd_cells] {
    puts "  $c"
}
puts ""
puts "===== Find clk source ====="
foreach c [get_bd_cells] {
    foreach p [get_bd_pins -of $c -filter {DIR == O && TYPE == clk}] {
        puts "  CLK out: $p"
    }
}
puts ""
puts "===== Find reset source ====="
foreach c [get_bd_cells] {
    foreach p [get_bd_pins -of $c -filter {DIR == O && TYPE == rst}] {
        puts "  RST out: $p"
    }
}
exit 0
