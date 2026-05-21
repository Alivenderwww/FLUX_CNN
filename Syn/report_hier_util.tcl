# report_hier_util.tcl -- 出按 hierarchy 拆的 utilization
# 用法: vivado -mode batch -source report_hier_util.tcl -nojournal -nolog

set DCP "C:/_Project/FLUX_CNN/Syn/vd100_minimal/output/ps_hello.runs/impl_1/design_1_wrapper_routed.dcp"
set OUT "C:/_Project/FLUX_CNN/Syn/util_hier_post_route.rpt"

open_checkpoint $DCP

# 整体 hierarchical, 含子模块拆分
report_utilization -hierarchical -hierarchical_depth 8 -file $OUT

puts ""
puts "==================== Util report ===================="
puts "Output: $OUT"
puts "====================================================="

close_design
exit 0
