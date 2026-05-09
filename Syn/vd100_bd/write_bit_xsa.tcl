# =============================================================================
# write_bit_xsa.tcl  --  License 已解决, 直接出 bitstream + .xsa (复用 routed_dcp)
# =============================================================================

set DCP "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper_routed.dcp"
set XSA "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xsa"
set PDI "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.pdi"

if {![file exists $DCP]} {
    puts "ERROR: routed_dcp not found, run impl first"
    exit 1
}

puts "Opening routed checkpoint..."
open_checkpoint $DCP

puts ""
puts "=== Writing device image (.pdi for Versal) ==="
write_device_image -force $PDI
if {[file exists $PDI]} {
    set pdi_size [file size $PDI]
    puts "  + $PDI ([expr $pdi_size / 1024] KB)"
} else {
    puts "  ! .pdi generation failed (license issue?)"
    close_design
    exit 1
}

puts ""
puts "=== Writing hardware platform .xsa ==="
write_hw_platform -fixed -include_bit -force $XSA
if {[file exists $XSA]} {
    set xsa_size [file size $XSA]
    puts "  + $XSA ([expr $xsa_size / 1024 / 1024] MB)"
}

puts ""
puts "============================================================"
puts " DONE"
puts "  PDI : $PDI"
puts "  XSA : $XSA"
puts "============================================================"

close_design
exit 0
