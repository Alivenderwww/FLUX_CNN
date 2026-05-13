# Check 1: BD IP 是否有 behavioral sim model 可用
# Check 2: Vivado xsim 能否 elab 当前工程的 design_1_wrapper
open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

puts ""
puts "=== sim_1 fileset properties ==="
puts "  top = '[get_property top [get_filesets sim_1]]'"
puts "  source_mgmt_mode = [get_property source_mgmt_mode [current_project]]"

puts ""
puts "=== BD IP sim model 类型 ==="
foreach cell {axi_noc_0 versal_cips_0 smartconnect_pl_0 clk_wizard_0} {
    set ip [get_bd_cells -quiet $cell]
    if {$ip eq ""} { continue }
    set vlnv [get_property VLNV $ip]
    puts "  $cell ($vlnv):"
    # 看 IP 是否提供 behavioral sim model 文件
    set sim_files [get_files -quiet -of_objects [get_filesets sim_1] -filter "PARENT_COMPOSITE_FILE =~ \"*$cell*\""]
    if {$sim_files ne ""} {
        puts "    has [llength $sim_files] sim files"
        puts "    first: [lindex $sim_files 0]"
    } else {
        # 试 PARENT 模式
        set sim_files [get_files -quiet -of_objects [get_filesets sources_1] -filter "PARENT_COMPOSITE_FILE =~ \"*$cell*\""]
        puts "    via sources_1: [llength $sim_files] files"
    }
}

puts ""
puts "=== 找 design_1_wrapper.v (BD top wrapper) ==="
set wrap [get_files -quiet design_1_wrapper.v]
if {$wrap ne ""} {
    puts "  found: $wrap"
} else {
    puts "  NOT found - 需 make_wrapper [get_files design_1.bd]"
}

puts ""
puts "=== sim_1 中所有 file count ==="
puts "  sources_1: [llength [get_files -of_objects [get_filesets sources_1]]]"
puts "  sim_1: [llength [get_files -of_objects [get_filesets sim_1]]]"

exit 0
