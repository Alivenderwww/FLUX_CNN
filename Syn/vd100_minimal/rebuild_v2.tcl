# =============================================================================
# rebuild_v2.tcl  --  vd100_minimal v2 重综合 + bitgen
#
# v2 改动:
#   1. RTL: cfg_regs 加 CTRL.bit7 软 reset, core_top 加 reset stretcher + 16
#          sub-module rst_n 改 core_rst_n
#   2. BD : axi_bram_ctrl MEM_DEPTH 32768 (512KB), assign addr range 512K
#
# 用法:
#   D:\Xilinx\Vivado\2023.2\bin\vivado.bat -mode batch -source rebuild_v2.tcl
#
# 之后跑 bootgen 生成新 PDI:
#   D:\Xilinx\Vitis\2023.2\bin\bootgen.bat -arch versal -image vd100_minimal_with_elf.bif \
#       -o vd100_minimal_with_elf.pdi -w on
# =============================================================================

open_project C:/_Project/FLUX_CNN/Syn/vd100_minimal/output/ps_hello.xpr

puts "============================================================"
puts " vd100_minimal v2: update BD (BRAM 512KB) + resynth"
puts "============================================================"

# -----------------------------------------------------------------------------
# 1. open BD + 更新 axi_bram_ctrl 容量 + 重 assign address
# -----------------------------------------------------------------------------
open_bd_design [get_files design_1.bd]

# axi_bram_ctrl.MEM_DEPTH 是 read-only (auto-prop from emb_mem_gen). 改 emb_mem_gen.
# emb_mem_gen WRITE_DEPTH_A = 32768 × 16 byte = 512 KB.
set_property -dict [list \
    CONFIG.MEMORY_PRIMITIVE  {bram} \
    CONFIG.WRITE_DEPTH_A     {32768} \
] [get_bd_cells emb_mem_gen_0]
puts "  + emb_mem_gen_0 WRITE_DEPTH_A = 32768 (512KB, auto-prop 给 axi_bram_ctrl)"

# 重 assign address: 用 0xA4100000 (在 M_AXI_FPD aperture 0xA4000000 [448M] 内, 0xA4000000 4KB
# 已给 CSR). 跟之前 GUI 配的一致, host 测试脚本 BRAM_BASE 不用改.
catch {assign_bd_address -offset 0xA4100000 -range 512K \
    -target_address_space [get_bd_addr_spaces versal_cips_0/M_AXI_FPD] \
    [get_bd_addr_segs axi_bram_ctrl_0/S_AXI/Mem0] -force}
catch {assign_bd_address -offset 0xA4100000 -range 512K \
    -target_address_space [get_bd_addr_spaces u_mc_minimal/m_axi] \
    [get_bd_addr_segs axi_bram_ctrl_0/S_AXI/Mem0] -force}
puts "  + assign_bd_address 0xA4100000 512K (both CIPS.M_AXI_FPD + u_mc_minimal.m_axi)"

validate_bd_design
save_bd_design
puts "  + BD validated + saved"

# -----------------------------------------------------------------------------
# 2. reset + launch synth + impl + write_device_image
# -----------------------------------------------------------------------------
reset_runs synth_1
reset_runs impl_1
foreach r [get_runs -filter {NAME =~ "*_synth_*"}] {
    catch {reset_run $r}
}

update_compile_order -fileset sources_1

puts ""
puts "  > launch synth_1"
launch_runs synth_1 -jobs 8
wait_on_run synth_1
puts "synth_1 = [get_property STATUS [get_runs synth_1]]"

set_param drc.disableLUTOverUtilError 1
puts ""
puts "  > launch impl_1 -to_step write_device_image"
launch_runs impl_1 -to_step write_device_image -jobs 8
wait_on_run impl_1
puts "impl_1 = [get_property STATUS [get_runs impl_1]]"

# -----------------------------------------------------------------------------
# 3. 提取 PDI 路径方便外部 bootgen
# -----------------------------------------------------------------------------
set pdi_path C:/_Project/FLUX_CNN/Syn/vd100_minimal/output/ps_hello.runs/impl_1/design_1_wrapper.pdi
if {[file exists $pdi_path]} {
    puts ""
    puts "============================================================"
    puts " DONE. PDI at:"
    puts "   $pdi_path"
    puts ""
    puts " Next: bootgen 生成 PDI w/ A72 ELF"
    puts "   D:\\Xilinx\\Vitis\\2023.2\\bin\\bootgen.bat -arch versal \\"
    puts "       -image vd100_minimal_with_elf.bif \\"
    puts "       -o vd100_minimal_with_elf.pdi -w on"
    puts "============================================================"
} else {
    puts ""
    puts " WARN: PDI not found at $pdi_path"
    puts "       Check impl_1 status, possibly failed."
}

exit 0
