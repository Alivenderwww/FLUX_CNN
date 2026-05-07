# =============================================================================
# tb_smc/run_chain.tcl  --  Phase 7 SMC + NUMA chain sim 共享 prelude
#   跑 toolchain/run_multicore_chain.py --smc 生成的 case 目录
#   DUT: multicore_top_smc (4 ConvCore + axi_smc_4to4 IP + 4 mem stub)
#
#   Case-specific tcl (run_resnet11/wslice1/wslice5.tcl) 设 CASE_DIR_REL 后 source 本文件.
# =============================================================================

if {![info exists CASE_DIR_REL]} {
    set CASE_DIR_REL "cases/smc_wslice1"     ;# 默认 case
}

set XLNX_INI    {C:/_Project/FLUX_CNN/Syn/ip_sim_export/axi_dm/modelsim/modelsim.ini}
set IP_DM_DIR   {C:/_Project/FLUX_CNN/Syn/ip_managed/ip_managed.gen/sources_1/ip/axi_dm}
set IP_CSR4_DIR {C:/_Project/FLUX_CNN/Syn/ip_managed/ip_managed.gen/sources_1/ip/axi_lite_1to4}
set IP_SMC_DIR  {C:/_Project/FLUX_CNN/Syn/ip_managed/ip_managed.gen/sources_1/ip/axi_smc_4to4}
set IP_SMC_XCI  {C:/_Project/FLUX_CNN/Syn/ip_managed/ip_managed.srcs/sources_1/ip/axi_smc_4to4/axi_smc_4to4.xci}

if {![file exists $IP_SMC_XCI]} {
    error "axi_smc_4to4 IP not generated. Run:\n  cd c:\\_Project\\FLUX_CNN\n  vivado -mode batch -source Syn/gen_axi_smc_4to4.tcl"
}

file copy -force $XLNX_INI ./modelsim.ini

vlib work
vmap work work
vlib xil_defaultlib
vmap xil_defaultlib xil_defaultlib

# axi_dm IP (VHDL)
vcom -work xil_defaultlib -93 \
    "$IP_DM_DIR/sim/axi_dm.vhd"
vlog -work xil_defaultlib \
    "C:/_Project/FLUX_CNN/Syn/ip_sim_export/axi_dm/modelsim/glbl.v"

# axi_lite_1to4 IP
vlog -work xil_defaultlib \
    "$IP_CSR4_DIR/sim/axi_lite_1to4.v"

# axi_smc_4to4 IP (Vivado axi_crossbar 配 4M↔4S)
vlog -work xil_defaultlib \
    "$IP_SMC_DIR/sim/axi_smc_4to4.v"

# RTL
vlog -sv -work work -mfcu -suppress 2902,13314 \
    +incdir+../../RTL \
    ../../RTL/std_rf.sv \
    ../../RTL/sdp.sv \
    ../../RTL/sram_model.sv \
    ../../RTL/AXI4/axi_arbiter.sv \
    ../../RTL/AXI4/axi_m_mux.sv \
    ../../RTL/AXI4/axi_lite_csr.sv \
    ../../RTL/AXI4/ifb_axi_slave.sv \
    ../tb_axi_m_mux/axi_slave_mem.sv \
    ../../RTL/desc_fifo.sv \
    ../../RTL/sequencer.sv \
    ../../RTL/cfg_regs.sv \
    ../../RTL/mac_pe.sv \
    ../../RTL/mac_col.sv \
    ../../RTL/mac_array.sv \
    ../../RTL/parf_col.sv \
    ../../RTL/parf_accum.sv \
    ../../RTL/line_buffer.sv \
    ../../RTL/wgt_buffer.sv \
    ../../RTL/bias_rf.sv \
    ../../RTL/ofb_writer.sv \
    ../../RTL/DMA/idma_ctrl.sv \
    ../../RTL/DMA/idma_sg_dispatcher.sv \
    ../../RTL/DMA/wdma_ctrl.sv \
    ../../RTL/DMA/odma_ctrl.sv \
    ../../RTL/DMA/odma_sg_dispatcher.sv \
    ../../RTL/DMA/rdma_ctrl.sv \
    ../../RTL/DMA/mm2s_arb.sv \
    ../../RTL/DMA/dfe.sv \
    ../../RTL/core_top.sv \
    ../../RTL/multicore_top_smc.sv \
    tb_smc_chain.sv

vsim -c -voptargs="+acc" \
    -L work -L xil_defaultlib \
    -L axi_datamover_v5_1_30 \
    -L fifo_generator_v13_2_8 \
    -L lib_pkg_v1_0_2 -L lib_cdc_v1_0_2 \
    -L lib_fifo_v1_0_17 -L lib_srl_fifo_v1_0_2 \
    -L blk_mem_gen_v8_4_6 \
    -L axi_crossbar_v2_1_29 -L axi_data_fifo_v2_1_27 \
    -L axi_infrastructure_v1_1_0 -L axi_register_slice_v2_1_28 \
    -L generic_baseblocks_v2_1_0 \
    -L unisims_ver \
    "+CASE_DIR=$CASE_DIR_REL" work.tb_smc_chain xil_defaultlib.glbl
run -all
quit -f
