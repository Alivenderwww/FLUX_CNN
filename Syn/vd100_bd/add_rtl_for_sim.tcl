# 检查 BD sources_1 是否 include ConvCore RTL, 不完整则补加
open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr

puts "=== sources_1 fileset 当前 file count ==="
puts "  count = [llength [get_files -of_objects [get_filesets sources_1]]]"

puts ""
puts "=== ConvCore RTL files in sources_1 ==="
foreach pat {multicore_top_vd100_bd_v core_top sequencer cfg_regs mac_array dispatcher mm2s_arb axi_arbiter} {
    set found [get_files -quiet -filter "NAME =~ \"*$pat*\""]
    if {[llength $found] > 0} {
        puts "  $pat: [llength $found] file(s) - first: [lindex $found 0]"
    } else {
        puts "  $pat: NOT in sources"
    }
}

puts ""
puts "=== sim_1 fileset top + file count ==="
puts "  sim_1 top = '[get_property top [get_filesets sim_1]]'"
puts "  sim_1 file count = [llength [get_files -of_objects [get_filesets sim_1]]]"

exit 0
