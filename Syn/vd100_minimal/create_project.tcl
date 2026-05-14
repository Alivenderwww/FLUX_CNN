# =============================================================================
# create_project.tcl  --  创建 vd100_minimal Vivado 工程 + 加 RTL + 创建 BD
#
# 目的: bring-up 用最小系统, 完全绕开 axi_noc + PL DDR4 + multicore_top_vd100_bd_v
#       历史 BD 包袱. 用 jtag_to_axi (vendor IP) 当 host, BRAM 当 m_axi slave.
#
# 用法: vivado -mode batch -source create_project.tcl
# =============================================================================

set PROJ_DIR "C:/_Project/FLUX_CNN/Syn/vd100_minimal/output"
set PROJ_NAME "vd100_minimal"
set PART      "xcve2302-sfva784-1LP-e-S"   ;# VD100 (Versal AI Edge VE2302)
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
puts "[1/4] gen axi_dm IP ..."
source C:/_Project/FLUX_CNN/Syn/vd100_bd/gen_axi_dm_vd100.tcl

# -----------------------------------------------------------------------------
# 2. 加 RTL (svh + sv + v)
# -----------------------------------------------------------------------------
puts "[2/4] add RTL ..."

set svh_files [list \
    "$RTL_DIR/flux_cnn_params.svh" \
]
foreach f $svh_files {
    add_files -norecurse $f
    set_property file_type {Verilog Header} [get_files [file tail $f]]
    set_property is_global_include true [get_files [file tail $f]]
}

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
]
foreach f $rtl_files {
    if {[file exists $f]} {
        add_files -norecurse $f
    } else {
        puts "  ! file not found: $f"
    }
}
set_property FILE_TYPE {SystemVerilog} [get_files *.sv]
set_property include_dirs $RTL_DIR [current_fileset]
set_property include_dirs $RTL_DIR [get_filesets sim_1]
set_property source_mgmt_mode All [current_project]
update_compile_order -fileset sources_1

puts "  added [llength $rtl_files] RTL files"

# 加约束 (XDC)
add_files -fileset constrs_1 -norecurse C:/_Project/FLUX_CNN/Syn/vd100_minimal/vd100_minimal.xdc
puts "  + constraint: vd100_minimal.xdc"

# -----------------------------------------------------------------------------
# 3. 创建 Block Design + 调 build_bd.tcl 创建所有 IP + 连线
# -----------------------------------------------------------------------------
puts "[3/4] create block design ..."
create_bd_design design_1
source C:/_Project/FLUX_CNN/Syn/vd100_minimal/build_bd.tcl

# -----------------------------------------------------------------------------
# 4. 生成 wrapper + 设顶层
# -----------------------------------------------------------------------------
puts "[4/4] generate wrapper ..."
make_wrapper -files [get_files design_1.bd] -top -import -force
set_property top design_1_wrapper [current_fileset]

regenerate_bd_layout
save_bd_design
validate_bd_design
generate_target -force all [get_files design_1.bd]

puts ""
puts "============================================================"
puts " vd100_minimal project ready. Next: run_synth.tcl"
puts "============================================================"
exit 0
