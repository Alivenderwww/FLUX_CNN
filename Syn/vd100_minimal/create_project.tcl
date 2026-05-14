# =============================================================================
# create_project.tcl  --  仅创建 vd100_minimal Vivado 工程 + 加 RTL/约束
#
# 注意: 不在这里 create BD. Vivado 2023.2.1 batch 模式 fresh project
#       create_bd_design 撞 roe_framer / busdef IP init Tcl bug 触发
#       Common 17-39 ERROR, 后续 create_bd_cell 全 fail. workaround 验证
#       过多种均无效. 转用 GUI 模式: GUI 启动时 vivado init 已完整, BD
#       创建不撞 bug.
#
# 用法 (准备工程):
#   vivado.bat -mode batch -source create_project.tcl
#
# 用户后续 GUI 操作 (open vd100_minimal.xpr 后 Tcl Console):
#   source C:/_Project/FLUX_CNN/Syn/vd100_minimal/build_bd.tcl
#   (会 create_bd_design + 创建所有 IP + 连线, 留 CIPS 待手动添加)
# =============================================================================

set PROJ_DIR "C:/_Project/FLUX_CNN/Syn/vd100_minimal/output"
set PROJ_NAME "vd100_minimal"
set PART      "xcve2302-sfva784-1LP-e-S"
set RTL_DIR   "C:/_Project/FLUX_CNN/RTL"

file mkdir $PROJ_DIR
create_project -force $PROJ_NAME $PROJ_DIR -part $PART
set_property target_language Verilog [current_project]

puts "============================================================"
puts " vd100_minimal project created at $PROJ_DIR"
puts " Part: $PART"
puts "============================================================"

# -----------------------------------------------------------------------------
# 1. axi_dm IP (ConvCore 内部依赖)
# -----------------------------------------------------------------------------
puts "\[1/3\] create axi_dm IP ..."
if {[llength [get_ips -quiet axi_dm]] == 0} {
    create_ip -name axi_datamover -vendor xilinx.com -library ip -module_name axi_dm
    set_property -dict {
        CONFIG.c_include_mm2s            Full
        CONFIG.c_include_s2mm            Full
        CONFIG.c_addr_width              32
        CONFIG.c_m_axi_mm2s_addr_width   32
        CONFIG.c_m_axi_mm2s_data_width   128
        CONFIG.c_m_axi_mm2s_id_width     2
        CONFIG.c_m_axis_mm2s_tdata_width 128
        CONFIG.c_mm2s_btt_used           23
        CONFIG.c_include_mm2s_dre        true
        CONFIG.c_mm2s_burst_size         256
        CONFIG.c_mm2s_include_sf         false
        CONFIG.c_m_axi_s2mm_addr_width   32
        CONFIG.c_m_axi_s2mm_data_width   128
        CONFIG.c_m_axi_s2mm_id_width     2
        CONFIG.c_s_axis_s2mm_tdata_width 128
        CONFIG.c_s2mm_btt_used           23
        CONFIG.c_include_s2mm_dre        true
        CONFIG.c_s2mm_burst_size         256
        CONFIG.c_s2mm_include_sf         false
    } [get_ips axi_dm]
    generate_target -force {synthesis instantiation_template} [get_ips axi_dm]
    puts "  + axi_dm IP created"
}

# -----------------------------------------------------------------------------
# 2. 加 RTL (svh + sv)
# -----------------------------------------------------------------------------
puts "\[2/3\] add RTL ..."

add_files -norecurse "$RTL_DIR/flux_cnn_params.svh"
set_property file_type {Verilog Header} [get_files flux_cnn_params.svh]
set_property is_global_include true [get_files flux_cnn_params.svh]

set rtl_files [list \
    "$RTL_DIR/std_rf.sv" \
    "$RTL_DIR/sram_model.sv" \
    "$RTL_DIR/sdp.sv" \
    "$RTL_DIR/AXI4/axi_lite_csr.sv" \
    "$RTL_DIR/AXI4/axi_m_mux.sv" \
    "$RTL_DIR/AXI4/axi_arbiter.sv" \
    "$RTL_DIR/AXI4/ifb_axi_slave.sv" \
    "$RTL_DIR/DMA/idma_ctrl.sv" \
    "$RTL_DIR/DMA/idma_sg_dispatcher.sv" \
    "$RTL_DIR/DMA/wdma_ctrl.sv" \
    "$RTL_DIR/DMA/odma_ctrl.sv" \
    "$RTL_DIR/DMA/odma_sg_dispatcher.sv" \
    "$RTL_DIR/DMA/rdma_ctrl.sv" \
    "$RTL_DIR/DMA/mm2s_arb.sv" \
    "$RTL_DIR/DMA/dfe.sv" \
    "$RTL_DIR/desc_fifo.sv" \
    "$RTL_DIR/sequencer.sv" \
    "$RTL_DIR/cfg_regs.sv" \
    "$RTL_DIR/mac_pe.sv" \
    "$RTL_DIR/mac_col.sv" \
    "$RTL_DIR/mac_array.sv" \
    "$RTL_DIR/mac_simd_pair.sv" \
    "$RTL_DIR/mac_array_simd.sv" \
    "$RTL_DIR/parf_col.sv" \
    "$RTL_DIR/parf_accum.sv" \
    "$RTL_DIR/line_buffer.sv" \
    "$RTL_DIR/wgt_buffer.sv" \
    "$RTL_DIR/bias_rf.sv" \
    "$RTL_DIR/ofb_writer.sv" \
    "$RTL_DIR/core_top.sv" \
    "$RTL_DIR/Versal/multicore_top_minimal.sv" \
    "$RTL_DIR/Versal/multicore_top_minimal_v.v" \
]
foreach f $rtl_files {
    if {[file exists $f]} { add_files -norecurse $f }
}
set_property FILE_TYPE {SystemVerilog} [get_files *.sv]
# .v 文件保持 Verilog 类型 (不要改 SV), BD module reference 必须是 Verilog 顶
set_property include_dirs $RTL_DIR [current_fileset]
set_property include_dirs $RTL_DIR [get_filesets sim_1]
set_property source_mgmt_mode All [current_project]
update_compile_order -fileset sources_1
puts "  + [llength $rtl_files] RTL files added"

# -----------------------------------------------------------------------------
# 3. 加约束
# -----------------------------------------------------------------------------
puts "\[3/3\] add constraints ..."
add_files -fileset constrs_1 -norecurse C:/_Project/FLUX_CNN/Syn/vd100_minimal/vd100_minimal.xdc
puts "  + vd100_minimal.xdc added"

puts ""
puts "============================================================"
puts " vd100_minimal 工程准备完成 (无 BD - 用户 GUI 创建)"
puts ""
puts " batch 模式无法稳定创建 BD (roe_framer IP init bug + create_bd_cell"
puts " silent skip). GUI 模式 vivado init 完整, 跑 build_bd.tcl 一次过."
puts ""
puts " 下一步 (用户 GUI 操作):"
puts "   1. 打开 Vivado GUI: vivado.bat &"
puts "   2. Open Project: $PROJ_DIR/$PROJ_NAME.xpr"
puts "   3. 在 Tcl Console 跑:"
puts "        create_bd_design design_1"
puts "        source C:/_Project/FLUX_CNN/Syn/vd100_minimal/build_bd.tcl"
puts "      会创建所有非 CIPS IP + 连线"
puts "   4. 双击 design_1.bd 打开 BD canvas, 手动加 versal_cips_0 IP"
puts "      + 配置 + 连线 (按 build_bd.tcl 头注释 SOP)"
puts "   5. validate_bd_design + save_bd_design + make_wrapper"
puts "   6. Run synth + impl + write_device_image"
puts "============================================================"
exit 0
