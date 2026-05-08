# =============================================================================
# create_project.tcl  --  VD100 demo Vivado 工程创建 (基于 ALINX 02_pl_rw_ddr)
#
# 用法:
#   cd C:\_Project\FLUX_CNN\Syn\vd100_bd
#   "D:\Xilinx\Vivado\2023.2\bin\vivado.bat" -mode batch -source create_project.tcl -nojournal -log create.log
# =============================================================================
set tclpath [pwd]
cd $tclpath

source $tclpath/project_info.tcl

# 工程输出根目录: vd100_bd/output (跟脚本同级, 避免污染 RTL/)
set projpath "$tclpath/output"
file mkdir $projpath
cd $projpath

# 创建 in-disk project
create_project -force $projName $projpath -part $devicePart
set_property TARGET_LANGUAGE Verilog [current_project]

# Filesets
if {[string equal [get_filesets -quiet sources_1] ""]} { create_fileset -srcset sources_1 }
if {[string equal [get_filesets -quiet constrs_1] ""]} { create_fileset -constrset constrs_1 }
file mkdir $projpath/$projName.srcs/sources_1/new
file mkdir $projpath/$projName.srcs/constrs_1/new

# ===== Block Design =====
create_bd_design $bdname

# Versal CIPS
create_bd_cell -type ip -vlnv xilinx.com:ip:versal_cips:3.4 versal_cips_0
source $tclpath/ps_config.tcl
set_ps_config versal_cips_0

# PL 端 (NoC + DDRMC + 3 S_AXI + GP smartconnect + IRQ + clk_wizard)
source $tclpath/pl_config.tcl

regenerate_bd_layout
validate_bd_design
save_bd_design

# BD wrapper 自动生成
make_wrapper -files [get_files $projpath/$projName.srcs/sources_1/bd/$bdname/$bdname.bd] -top
add_files -norecurse [glob -nocomplain $projpath/$projName.gen/sources_1/bd/$bdname/hdl/*.v]

# ===== 加自定义 RTL: multicore_top_vd100 + 所有依赖 =====
set RTL_DIR  "$tclpath/../../RTL"
set rtl_files [list \
    "$RTL_DIR/std_rf.sv"             "$RTL_DIR/sram_model.sv"          "$RTL_DIR/sdp.sv" \
    "$RTL_DIR/AXI4/axi_lite_csr.sv"  "$RTL_DIR/AXI4/axi_m_mux.sv" \
    "$RTL_DIR/AXI4/axi_arbiter.sv"   "$RTL_DIR/AXI4/ifb_axi_slave.sv" \
    "$RTL_DIR/DMA/idma_ctrl.sv"      "$RTL_DIR/DMA/idma_sg_dispatcher.sv" \
    "$RTL_DIR/DMA/wdma_ctrl.sv"      "$RTL_DIR/DMA/odma_ctrl.sv" \
    "$RTL_DIR/DMA/odma_sg_dispatcher.sv" \
    "$RTL_DIR/DMA/rdma_ctrl.sv"      "$RTL_DIR/DMA/mm2s_arb.sv" \
    "$RTL_DIR/DMA/dfe.sv"            "$RTL_DIR/desc_fifo.sv" \
    "$RTL_DIR/sequencer.sv"          "$RTL_DIR/cfg_regs.sv" \
    "$RTL_DIR/mac_pe.sv"             "$RTL_DIR/mac_col.sv"             "$RTL_DIR/mac_array.sv" \
    "$RTL_DIR/mac_simd_pair.sv"      "$RTL_DIR/mac_array_simd.sv" \
    "$RTL_DIR/parf_col.sv"           "$RTL_DIR/parf_accum.sv" \
    "$RTL_DIR/line_buffer.sv"        "$RTL_DIR/wgt_buffer.sv" \
    "$RTL_DIR/bias_rf.sv"            "$RTL_DIR/ofb_writer.sv" \
    "$RTL_DIR/core_top.sv" \
    "$RTL_DIR/Versal/multicore_top_vd100.sv" \
    "$RTL_DIR/Versal/multicore_top_vd100_bd.sv" \
]
add_files $rtl_files
set_property FILE_TYPE {SystemVerilog} [get_files *.sv]
set_property include_dirs $RTL_DIR [current_fileset]

# Top = BD wrapper (Vivado 自动从 BD 生成 design_1_wrapper.v 当顶层)
# 不需要手写 top.v: BD 内部 multicore_top_vd100_bd 跟 PS/NoC 直接连

# axi_dm IP (复用 K325T 的 xci, 跨设备 retarget)
set IP_DIR  "$tclpath/../ip_managed/ip_managed.srcs/sources_1/ip"
read_ip "$IP_DIR/axi_dm/axi_dm.xci"
generate_target -force synthesis [get_ips axi_dm]
synth_ip [get_ips axi_dm]

# Constraints: 复用 ALINX 02_pl_rw_ddr 的 ddr4.xdc + system.xdc (板级 IO 一致)
set ALINX_XDC_DIR "C:/_Project/Bishe/VD100/VD100_9_16/demo/course_s1/02_pl_rw_ddr/auto_create_project/src/constraints"
if {[file exists $ALINX_XDC_DIR]} {
    foreach xdc [glob -nocomplain $ALINX_XDC_DIR/*.xdc] {
        add_files -fileset constrs_1 -copy_to $projpath/$projName.srcs/constrs_1/new -force $xdc
    }
    puts "  使用 ALINX 02_pl_rw_ddr 的 xdc: $ALINX_XDC_DIR"
} else {
    puts "  WARNING: ALINX xdc dir not found, 板级 IO 约束需手动加"
}

update_compile_order -fileset sources_1

puts "=== Project created: $projpath/$projName.xpr ==="
puts "下一步:"
puts "  1. 打开 GUI 检查 BD 是否合法: vivado $projpath/$projName.xpr"
puts "  2. 板级 P&R + bitstream:"
puts "     launch_runs synth_1 -jobs $runs_jobs ; wait_on_run synth_1"
puts "     launch_runs impl_1 -to_step write_bitstream -jobs $runs_jobs ; wait_on_run impl_1"
