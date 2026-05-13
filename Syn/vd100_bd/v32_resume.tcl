# v32 resume: 重启后 v32 impl_1 死在 write_bitstream 之前 (route phase 10 完成 但 routed.dcp 没写).
# placed.dcp 跟 physopt.dcp 还在. launch_runs impl_1 让 Vivado 自动判断从 route_design 继续.
open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr

# 让 Vivado 重置 impl_1 但保留 synth_1 + placed/physopt 中间结果
# launch_runs 默认会接续到 -to_step
set_param drc.disableLUTOverUtilError 1
reset_runs impl_1 -from_step route_design
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
puts "impl_1 = [get_property STATUS [get_runs impl_1]]"

open_run impl_1
write_debug_probes -force C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.ltx
write_hw_platform -fixed -include_bit -force C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xsa

puts "===== v32 RESUME DONE ====="
exit 0
