# Quick check: 能否从当前 BD 工程 export_simulation 出 axi_noc + axi_dm + smartconnect 完整 sim
open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

# 看 BD IP list (确认 sim model 可用)
puts "=== BD cells ==="
foreach cell [get_bd_cells] {
    puts "  [get_property NAME $cell] : [get_property VLNV $cell]"
}

# 看 sim_1 fileset 内容
puts ""
puts "=== sim_1 fileset (current top) ==="
puts "  top = [get_property top [get_filesets sim_1]]"

# 试 export simulation 到一个临时目录, 用 modelsim simulator
puts ""
puts "=== export_simulation (ModelSim, 仅 elab, 不真跑) ==="
set EXPORT_DIR C:/_Project/FLUX_CNN/sim/tb_bd_full
file mkdir $EXPORT_DIR

# 只 export, 不 launch sim. 看生成的 sim files 是否完整.
catch {
    export_simulation -force -directory $EXPORT_DIR \
        -simulator modelsim \
        -of_objects [get_filesets sim_1] \
        -export_source_files
} err
puts "export_simulation result: $err"

# list 生成的关键文件
puts ""
puts "=== generated sim files ==="
foreach f [glob -nocomplain $EXPORT_DIR/modelsim/*] {
    puts "  $f"
}

exit 0
