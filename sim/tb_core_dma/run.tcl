# =============================================================================
# tb_core_dma/run.tcl
# =============================================================================

set XLNX_INI    {C:/_Project/FLUX_CNN/Syn/ip_sim_export/axi_dm/modelsim/modelsim.ini}
set IP_GEN_DIR  {C:/_Project/FLUX_CNN/Syn/ip_managed/ip_managed.gen/sources_1/ip/axi_dm}
file copy -force $XLNX_INI ./modelsim.ini

vlib work
vmap work work
vlib xil_defaultlib
vmap xil_defaultlib xil_defaultlib

# ---- axi_dm IP wrapper (VHDL) + Vivado glbl ----
vcom -work xil_defaultlib -93 \
    "$IP_GEN_DIR/sim/axi_dm.vhd"
vlog -work xil_defaultlib \
    "C:/_Project/FLUX_CNN/Syn/ip_sim_export/axi_dm/modelsim/glbl.v"

# ---- 我们的 RTL + TB ----
vlog -sv -work work -mfcu -suppress 2902,13314 \
    +incdir+../../RTL \
    ../../RTL/std_rf.sv \
    ../../RTL/sdp.sv \
    ../../RTL/sram_model.sv \
    ../../RTL/AXI4/axi_arbiter.sv \
    ../../RTL/AXI4/axi_m_mux.sv \
    ../../RTL/AXI4/axi_lite_csr.sv \
    ../../RTL/DMA/idma_ctrl.sv \
    ../../RTL/DMA/wdma_ctrl.sv \
    ../../RTL/DMA/odma_ctrl.sv \
    ../../RTL/DMA/mm2s_arb.sv \
    ../../RTL/DMA/dfe.sv \
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
    ../../RTL/ofb_writer.sv \
    ../../RTL/core_top.sv \
    ../tb_axi_m_mux/axi_slave_mem.sv \
    tb_core_dma.sv

vsim -c -voptargs="+acc" -sva -f sim_params.f \
    -L work -L xil_defaultlib \
    -L axi_datamover_v5_1_30 \
    -L fifo_generator_v13_2_8 \
    -L lib_pkg_v1_0_2 \
    -L lib_cdc_v1_0_2 \
    -L lib_fifo_v1_0_17 \
    -L lib_srl_fifo_v1_0_2 \
    -L blk_mem_gen_v8_4_6 \
    -L unisims_ver \
    work.tb_core_dma xil_defaultlib.glbl
run -all
quit -f
