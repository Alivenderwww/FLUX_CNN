# =============================================================================
# add_rtl.tcl  --  把所有 multicore_top_vd100_bd 依赖 RTL 加进当前 Vivado 工程
#
# 用法 (Vivado GUI Tcl Console):
#   source C:/_Project/FLUX_CNN/Syn/vd100_bd/add_rtl.tcl
#
# 完成后再跑:
#   source C:/_Project/FLUX_CNN/Syn/vd100_bd/build_bd_ips_only.tcl
# 这次 u_mc_vd100 应该能成功 add 进 BD.
# =============================================================================

set RTL_DIR "C:/_Project/FLUX_CNN/RTL"

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
    "$RTL_DIR/Versal/multicore_top_vd100.sv" \
    "$RTL_DIR/Versal/multicore_top_vd100_bd.sv" \
]

# add 所有 RTL (跳过已存在的)
set added 0
foreach f $rtl_files {
    if {![file exists $f]} {
        puts "  ! file not found: $f"
        continue
    }
    set basename [file tail $f]
    if {[get_files -quiet $basename] eq ""} {
        add_files -norecurse $f
        incr added
    }
}
puts "  added $added new RTL files"

# 设 SystemVerilog 类型 (svh 不需要 add, include 路径搞定)
set_property FILE_TYPE {SystemVerilog} [get_files *.sv]

# 加 include path 让 RTL 能找到 RTL/flux_cnn_params.svh
set inc_path [list $RTL_DIR]
set_property include_dirs $inc_path [current_fileset]

# 也要给 BD 用的 fileset 加 include path (BD wrapper 编译时也要)
set_property include_dirs $inc_path [get_filesets sim_1]

# 切到 automatic compile order (Vivado 自己排顺序)
set_property source_mgmt_mode All [current_project]
update_compile_order -fileset sources_1

puts ""
puts "============================================================"
puts " RTL added. 现在可以再 source build_bd_ips_only.tcl 创建 u_mc_vd100"
puts "============================================================"
