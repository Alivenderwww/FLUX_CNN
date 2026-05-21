open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

puts "===== step A: update_compile_order ====="
update_compile_order -fileset sources_1

puts "===== step B: reset_target all (clear BD cache) ====="
if {[catch {reset_target all [get_files design_1.bd]} e]} {
    puts "ERR reset_target: $e"
} else {
    puts "OK reset_target"
}

puts "===== step C: update_module_reference ====="
if {[catch {update_module_reference {multicore_top_vd100_bd_v}} e]} {
    puts "ERR update_module_reference: $e"
} else {
    puts "OK update_module_reference"
}

puts "===== step D: validate BD ====="
catch {validate_bd_design -force} v
puts "validate result: $v"

puts "===== step E: m00_axi_<scalar> pin net check ====="
foreach pname {awready wready bvalid arready rlast rvalid awvalid arvalid wvalid wlast bready rready} {
    set pin [get_bd_pins -quiet u_mc_vd100/m00_axi_${pname}]
    if {$pin ne ""} {
        set net [get_bd_nets -quiet -of_objects $pin]
        puts "  m00_axi_${pname} pin=$pin net=$net"
    } else {
        puts "  m00_axi_${pname} NOT FOUND (good if it's input bundled in intf only)"
    }
}

puts "===== step F: list all /Net* dummy nets ====="
foreach n [get_bd_nets -quiet -filter {NAME =~ "Net*"}] {
    puts "  dummy net: $n"
}

save_bd_design
exit 0
