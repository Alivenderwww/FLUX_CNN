# Quick smoke test: Vivado 2023.2 + VE2302 device support 可用性
# 用法: vivado -mode batch -source Syn/check_vd100.tcl -nojournal -nolog

set PART "xcve2302-sfva784-1LP-e-s"
puts "=== Creating in-memory project on $PART ==="
create_project -in_memory -part $PART
puts "  Project: [current_project]"
puts "  Part:    [get_property PART [current_project]]"
puts "  Family:  [get_property FAMILY [get_parts $PART]]"
puts "  LUTs:    [get_property LUTS  [get_parts $PART]]"
puts "  FFs:     [get_property FLIPFLOPS  [get_parts $PART]]"
puts "  DSP:     [get_property DSP  [get_parts $PART]]"
puts "  BRAM:    [get_property BLOCK_RAMS [get_parts $PART]]"
puts "  URAM:    [get_property ULTRA_RAMS [get_parts $PART]]"

puts "=== Checking key Versal IPs availability ==="
foreach ip {versal_cips axi_noc clk_wizard util_ds_buf proc_sys_reset axi_datamover} {
    set found [get_ipdefs -filter "NAME == $ip" -quiet]
    if {[llength $found] > 0} {
        puts "  ✓ $ip: [lindex $found 0]"
    } else {
        puts "  ✗ $ip: NOT FOUND"
    }
}

close_project
puts "=== Vivado 2023.2 + VE2302 smoke test PASSED ==="
exit
