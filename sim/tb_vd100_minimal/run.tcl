# =============================================================================
# tb_vd100_minimal/run.tcl — 严格复刻 board vd100_minimal sim
#   DUT = multicore_top_minimal_v (board RTL wrapper)
#   BRAM = axi_slave_mem
#   case 来自 gen_case_hex.py 生成 (跑前先跑它)
# =============================================================================

set XLNX_INI    {C:/_Project/FLUX_CNN/Syn/ip_sim_export/axi_dm/modelsim/modelsim.ini}
set IP_DM_DIR   {C:/_Project/FLUX_CNN/Syn/ip_managed/ip_managed.gen/sources_1/ip/axi_dm}

if {![file exists "bram_init.hex"]} {
    error "bram_init.hex 不存在. 先跑 gen_case_hex.py --k 3 --h 200 --w 16"
}
if {![file exists "params.f"]} {
    error "params.f 不存在. 跑 gen_case_hex.py"
}

file copy -force $XLNX_INI ./modelsim.ini

vlib work
vmap work work
vlib xil_defaultlib
vmap xil_defaultlib xil_defaultlib

# axi_dm IP (VHDL)
vcom -work xil_defaultlib -93 "$IP_DM_DIR/sim/axi_dm.vhd"
vlog -work xil_defaultlib "C:/_Project/FLUX_CNN/Syn/ip_sim_export/axi_dm/modelsim/glbl.v"

# RTL
vlog -sv -work work -mfcu -suppress 2902,13314 \
    -f params.f \
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
    ../../RTL/mac_simd_pair.sv \
    ../../RTL/mac_array_simd.sv \
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
    ../../RTL/Versal/multicore_top_minimal.sv \
    ../../RTL/Versal/multicore_top_minimal_v.v \
    tb_vd100_minimal.sv

vsim -c -voptargs="+acc" \
    -L work -L xil_defaultlib \
    -L axi_datamover_v5_1_30 \
    -L fifo_generator_v13_2_8 \
    -L lib_pkg_v1_0_2 -L lib_cdc_v1_0_2 \
    -L lib_fifo_v1_0_17 -L lib_srl_fifo_v1_0_2 \
    -L blk_mem_gen_v8_4_6 \
    -L axi_infrastructure_v1_1_0 -L axi_register_slice_v2_1_28 \
    -L generic_baseblocks_v2_1_0 \
    -L unisims_ver \
    work.tb_vd100_minimal xil_defaultlib.glbl
run -all
quit -f
