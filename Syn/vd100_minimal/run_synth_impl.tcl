# =============================================================================
# run_synth_impl.tcl  --  不动 BD 配置, 跑 synth_1 + impl_1 + write_device_image
# 用法:
#   D:\Xilinx\Vivado\2023.2\bin\vivado.bat -mode batch -source run_synth_impl.tcl
# =============================================================================

open_project C:/_Project/FLUX_CNN/Syn/vd100_minimal/output/ps_hello.xpr

puts "============================================================"
puts " vd100_minimal: synth + impl + write_device_image (N=3+DDR/NoC)"
puts "============================================================"

# 确保 FLUX_MAC_SIMD enable (BD 没动, 但 verilog_define 可能被 reset)
set existing_defs [get_property verilog_define [current_fileset]]
if {[lsearch $existing_defs "FLUX_MAC_SIMD*"] < 0} {
    set_property verilog_define {FLUX_MAC_SIMD=1} [current_fileset]
    puts "  + verilog_define FLUX_MAC_SIMD=1 ensured"
}

# Generate output products (BD)
puts ""
puts ">>> STAGE: Generate output products"
set bd_file [get_files design_1.bd]
catch {generate_target all [get_files $bd_file]}
catch {make_wrapper -files [get_files $bd_file] -top -force -import}
update_compile_order -fileset sources_1

# Reset 所有 runs 让综合用新 BD
puts ""
puts ">>> STAGE: Reset runs"
foreach r [get_runs -filter {NAME =~ "*_synth_*"}] {
    catch {reset_run $r}
    puts "  - reset $r"
}
catch {reset_runs synth_1}
catch {reset_runs impl_1}

# Synth
puts ""
puts ">>> STAGE: launch_runs synth_1 (16 jobs)"
launch_runs synth_1 -jobs 16
wait_on_run synth_1
set ss [get_property STATUS [get_runs synth_1]]
puts "synth_1 STATUS = $ss"
if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
    puts "ERROR: synth_1 did not finish at 100%"
    exit 1
}

# Impl + write_device_image
puts ""
puts ">>> STAGE: launch_runs impl_1 -to_step write_device_image (16 jobs)"
set_param drc.disableLUTOverUtilError 1
launch_runs impl_1 -to_step write_device_image -jobs 16
wait_on_run impl_1
set si [get_property STATUS [get_runs impl_1]]
puts "impl_1 STATUS = $si"
set ip [get_property PROGRESS [get_runs impl_1]]
puts "impl_1 PROGRESS = $ip"

# PDI 位置
set pdi C:/_Project/FLUX_CNN/Syn/vd100_minimal/output/ps_hello.runs/impl_1/design_1_wrapper.pdi
if {[file exists $pdi]} {
    puts ""
    puts "============================================================"
    puts " DONE. PDI ready:"
    puts "   $pdi"
    puts " Size = [file size $pdi] bytes"
    puts " Next: bootgen (生成 BOOT.BIN) 或 JTAG 直接烧 PDI"
    puts "============================================================"
} else {
    puts ""
    puts "ERROR: PDI not generated, check impl_1 status above"
    exit 1
}

exit 0
