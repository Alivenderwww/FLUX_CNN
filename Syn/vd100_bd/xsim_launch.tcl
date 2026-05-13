# Vivado xsim 跑 BD sim: 跟 ModelSim 不同, xsim 是 Vivado 自家 simulator, 知道 cips_vip 私有
# boot sequence + axi_noc NMU BFM 的 sim-only 设置.
#
# 工程内 sim_1 fileset 已 add tb_bd_top.sv (来自 setup_sim_fileset.tcl).
# 这里设 sim_1 top + launch_simulation.

open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

# 加 tb_bd_top.sv 到 sim_1 (如果还没加)
set tb_path C:/_Project/FLUX_CNN/sim/tb_bd_full/tb_bd_top.sv
if {[get_files -quiet $tb_path] eq ""} {
    add_files -fileset sim_1 $tb_path
    puts "  added tb_bd_top.sv to sim_1"
}

# 设 sim_1 top + simulator runtime
set_property top tb_bd_top [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {10ms} -objects [get_filesets sim_1]
set_property -name {xsim.simulate.log_all_signals} -value {false} -objects [get_filesets sim_1]

puts "=== launch_simulation (xsim, behavioral, 非 batch 试避 broken pipe) ==="
# 跳过 launch_simulation 内部 compile/elab — 直接调 xsim flow scripts
# 让 Vivado 仅 generate scripts, 我们手动 run

# 先 generate sim scripts
update_compile_order -fileset sim_1
puts "=== sim_1 fileset top = [get_property top [get_filesets sim_1]] ==="

# 生成 .prj + scripts 不真 launch
launch_simulation -scripts_only -simset sim_1 -mode behavioral
puts "=== scripts generated. run manually via xsim_run.bat ==="

exit 0
