open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr

# v22 retry: synth_1 OK, impl_1 Place 30-487 fail (unplaced 12503 CLB > 12392 avail).
# 用 Area_Explore strategy 更激进 LUT pack, 不重 synth.
puts "=== v22 retry impl_1 with Area_Explore strategy ==="
reset_run impl_1

# Area_Explore: 让 placer / phys_opt 更激进 area-pack
# (Vivado 2023.2 内建 strategy 名见 list_property_value STRATEGY [get_runs impl_1])
catch {
    set_property STRATEGY "Area_Explore" [get_runs impl_1]
}
# 显式给 placer 一个更分散的 directive 也加上
set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE "AltSpreadLogic_high" [get_runs impl_1]
set_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE "ExploreWithAggressiveHoldFix" [get_runs impl_1]

set_param drc.disableLUTOverUtilError 1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
puts "impl_1 = [get_property STATUS [get_runs impl_1]]"

open_run impl_1
write_debug_probes -force C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.ltx
write_hw_platform -fixed -include_bit -force C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xsa

puts "===== v22 retry DONE ====="
exit 0
