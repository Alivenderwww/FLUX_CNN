# =============================================================================
# rebuild_all.tcl  --  完整重建 BD: create_project + apply_config + full_flow
#
# 当前 BD design 损坏 (v6 cell delete 引发 cascade), 必须从头重建.
# 这次 pl_config.tcl 已含 axis_ila + dbg_* probe 接线, BD 直接含 ILA.
# =============================================================================

set OUTDIR C:/_Project/FLUX_CNN/Syn/vd100_bd/output

# 1. 删除旧 output (干净环境)
puts "=== Delete old output ==="
if {[file exists $OUTDIR]} {
    if {[catch {file delete -force $OUTDIR} err]} {
        puts "  WARN: $err"
    } else {
        puts "  deleted $OUTDIR"
    }
}

# 2. create_project (BD + PL config + ILA)
puts ""
puts "=== source create_project.tcl ==="
cd C:/_Project/FLUX_CNN/Syn/vd100_bd
source C:/_Project/FLUX_CNN/Syn/vd100_bd/create_project.tcl

# 3. apply_config (PS NMU + axi_noc PS NMU + run full_flow synth+impl+bitstream+xsa)
puts ""
puts "=== source apply_config.tcl ==="
source C:/_Project/FLUX_CNN/Syn/vd100_bd/apply_config.tcl

# 4. 生成 .ltx (含 ILA probes)
puts ""
puts "=== Write .ltx debug probes ==="
open_run impl_1
set ltx C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.ltx
write_debug_probes -force $ltx
puts "  wrote: $ltx"

puts ""
puts "===== REBUILD ALL DONE ====="
exit 0
