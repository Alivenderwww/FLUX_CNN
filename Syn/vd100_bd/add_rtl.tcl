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

# ---- Xilinx IP: axi_dm (axi_datamover) — core_top.sv 实例化它 ----
# VD100 native 版本: 用 gen_axi_dm_vd100.tcl 在当前 VE2302 工程内 native 创建,
# 不依赖 K325T 工程, 一遍综合到位.
if {[get_ips -quiet axi_dm] eq ""} {
    set GEN_AXI_DM_TCL "C:/_Project/FLUX_CNN/Syn/vd100_bd/gen_axi_dm_vd100.tcl"
    if {[file exists $GEN_AXI_DM_TCL]} {
        source $GEN_AXI_DM_TCL
    } else {
        puts "  ! gen_axi_dm_vd100.tcl not found, axi_dm IP missing"
    }
}

# Verilog header (svh): BD module 集成时 Vivado 要求 svh 也 add 进工程, 不仅 include path
set svh_files [list \
    "$RTL_DIR/flux_cnn_params.svh" \
]
foreach f $svh_files {
    set basename [file tail $f]
    if {[get_files -quiet $basename] eq ""} {
        add_files -norecurse $f
        set_property file_type {Verilog Header} [get_files $basename]
        set_property is_global_include true [get_files $basename]
        puts "  + svh: $basename added (global include)"
    }
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
    "$RTL_DIR/Versal/multicore_top_vd100.sv" \
    "$RTL_DIR/Versal/multicore_top_vd100_bd.sv" \
    "$RTL_DIR/Versal/multicore_top_vd100_bd_v.v" \
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
puts "  added $added new RTL files (skipped existing ones)"

# 设 SystemVerilog 类型 (svh 不需要 add, include 路径搞定)
set_property FILE_TYPE {SystemVerilog} [get_files *.sv]
# .v 文件保持 Verilog (默认), 不要设成 SV — 因为 BD module reference 顶必须是 Verilog

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
puts " RTL added. Now source build_bd_ips_only.tcl to create u_mc_vd100"
puts "============================================================"
