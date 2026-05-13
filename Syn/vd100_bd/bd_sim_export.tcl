# BD-level ModelSim 仿真导出: 把 design_1 (BD) 的全部 IP behavioral sim model
# + 顶层 wrapper 导出给 ModelSim, 让我们能 sim 真实 axi_noc + smartconnect_pl + axi_dm
# + versal_cips, 复现 board 行为.
#
# Pre-requisite: Vivado simlib 已 compile (Syn/compile_simlib.tcl 跑过一次)

open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

puts "=== Step 1: generate_target simulation for BD (per-IP sim source) ==="
generate_target -force simulation [get_files design_1.bd]

puts "=== Step 2: 生成 simulation HDL wrapper (default BD wrapper) ==="
catch {
    make_wrapper -force -files [get_files design_1.bd] -top -import
} mk_err
puts "  make_wrapper: $mk_err"

puts "=== Step 3: 检查 simlib 路径 ==="
set SIMLIB C:/_Project/FLUX_CNN/Syn/compile_simlib_output
if {[file isdirectory $SIMLIB]} {
    puts "  simlib dir found: $SIMLIB"
} else {
    puts "  simlib dir NOT found at $SIMLIB - 看其它可能位置"
    foreach try {output/modelsim_lib output/sim_libs sim/modelsim_lib} {
        if {[file isdirectory C:/_Project/FLUX_CNN/$try]} {
            puts "  found candidate: C:/_Project/FLUX_CNN/$try"
            set SIMLIB C:/_Project/FLUX_CNN/$try
        }
    }
}

puts ""
puts "=== Step 4: export_simulation (modelsim, BD sim) ==="
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
        -lib_map_path $SIMLIB
} err]
puts "  export rc=$rc, err='$err'"

puts ""
puts "=== Step 5: 列出生成的 sim 文件 ==="
foreach pat {modelsim/*.sh modelsim/*.bat modelsim/*.do} {
    foreach f [glob -nocomplain $EXPORT_DIR/$pat] {
        puts "  $f"
    }
}

puts ""
puts "===== bd_sim_export DONE ====="
exit 0
