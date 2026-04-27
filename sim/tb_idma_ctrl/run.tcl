# =============================================================================
# tb_idma_ctrl/run.tcl  --  idma_ctrl + axi_dm DataMover 联合自测
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
vlog -sv -work work -mfcu \
    +incdir+../../RTL \
    ../../RTL/sram_model.sv \
    ../../RTL/DMA/idma_ctrl.sv \
    ../tb_axi_m_mux/axi_slave_mem.sv \
    tb_idma_ctrl.sv

vsim -c -voptargs="+acc" \
    -L xil_defaultlib \
    -L axi_datamover_v5_1_30 \
    -L fifo_generator_v13_2_8 \
    -L lib_pkg_v1_0_2 \
    -L lib_cdc_v1_0_2 \
    -L lib_fifo_v1_0_17 \
    -L lib_srl_fifo_v1_0_2 \
    -L blk_mem_gen_v8_4_6 \
    -L unisims_ver \
    work.tb_idma_ctrl xil_defaultlib.glbl

run -all
quit -f
