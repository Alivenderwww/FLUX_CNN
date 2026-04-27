# =============================================================================
# compile_simlib.tcl  --  把 Vivado 仿真库编译给 ModelSim 用 (一次性)
#
# 在 Vivado Tcl Console 里 source 这个脚本, 或:
#   vivado -mode batch -source compile_simlib.tcl
# 编完后把输出 dir 下的 modelsim.ini 路径 (或库映射) 接进 sim/tb_core_dma/run.tcl.
#
# 目标: Kintex-7 XC7K325T (kintex7 family)
# ModelSim: C:/modeltech64_2020.4/win64
# 输出 dir: C:/_Project/FLUX_CNN/Syn/sim_libs/
# =============================================================================

set MODELSIM_BIN  {C:/modeltech64_2020.4/win64}
set SIMLIB_OUTDIR {C:/_Project/FLUX_CNN/Syn/sim_libs}

if {![file exists $SIMLIB_OUTDIR]} {
    file mkdir $SIMLIB_OUTDIR
}

puts "==> compile_simlib for ModelSim"
puts "    simulator_exec_path : $MODELSIM_BIN"
puts "    family              : kintex7  (XC7K325T)"
puts "    output dir          : $SIMLIB_OUTDIR"

compile_simlib \
    -simulator        modelsim                \
    -simulator_exec_path $MODELSIM_BIN        \
    -family           kintex7                 \
    -language         all                     \
    -library          all                     \
    -dir              $SIMLIB_OUTDIR          \
    -force

puts "==> done. modelsim.ini 在 $SIMLIB_OUTDIR/modelsim.ini"
puts "    把它的库映射 (vmap) 拿到 sim/tb_core_dma/run.tcl 顶部, 或用 -modelsimini 参数."
