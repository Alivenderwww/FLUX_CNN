# 把 sources_1 内所有 ConvCore + AXI + DMA RTL 加到 sim_1 fileset, 让 export_simulation
# 把它们包进 compile.do
open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

puts "=== copy sources_1 RTL → sim_1 fileset ==="
set src_files [get_files -of_objects [get_filesets sources_1] \
    -filter "FILE_TYPE == \"Verilog\" || FILE_TYPE == \"SystemVerilog\" || FILE_TYPE == \"Verilog Header\""]
puts "  sources_1 RTL: [llength $src_files]"

# 过滤掉 BD-generated 的 (它们 export_simulation 已经从 .bd 拿), 只 add 我们手写的 RTL
set our_rtl [list]
foreach f $src_files {
    set p [get_property NAME $f]
    if {[string match "*sources_1/bd/*" $p]} { continue }
    if {[string match "*ip_managed/*" $p]} { continue }
    lappend our_rtl $f
}
puts "  our RTL (non-BD): [llength $our_rtl]"

# Add to sim_1 (Vivado tcl 不支持 lmap, 用 foreach)
set rtl_paths [list]
foreach f $our_rtl {
    lappend rtl_paths [get_property NAME $f]
}
add_files -fileset sim_1 $rtl_paths
puts "  added [llength $rtl_paths] files to sim_1"

puts ""
puts "=== sim_1 fileset 现在 ==="
puts "  top = '[get_property top [get_filesets sim_1]]'"
puts "  count = [llength [get_files -of_objects [get_filesets sim_1]]]"

save_bd_design

puts ""
puts "=== 重新 export_simulation 给 ModelSim (含 ConvCore RTL) ==="
set EXPORT_DIR C:/_Project/FLUX_CNN/sim/tb_bd_full
file mkdir $EXPORT_DIR

set rc [catch {
    export_simulation \
        -force \
        -directory $EXPORT_DIR \
        -simulator modelsim \
        -of_objects [get_files design_1.bd] \
        -export_source_files \
        -ip_user_files_dir C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.ip_user_files \
        -ipstatic_source_dir C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.ip_user_files/ipstatic \
        -lib_map_path C:/_Project/FLUX_CNN/Syn/sim_libs
} err]
puts "  export rc=$rc, err='$err'"

# Also export with sim_1 fileset (to grab our RTL)
puts ""
puts "=== additional export_simulation -of_objects sim_1 (含 our RTL) ==="
set EXPORT_DIR2 C:/_Project/FLUX_CNN/sim/tb_bd_full/sim_only
file mkdir $EXPORT_DIR2

set rc [catch {
    export_simulation \
        -force \
        -directory $EXPORT_DIR2 \
        -simulator modelsim \
        -of_objects [get_filesets sim_1] \
        -export_source_files \
        -lib_map_path C:/_Project/FLUX_CNN/Syn/sim_libs
} err]
puts "  export sim_1 rc=$rc, err='$err'"

exit 0
