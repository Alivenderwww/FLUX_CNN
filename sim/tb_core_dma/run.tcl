vlib work
vmap work work
vlog -sv -work work -mfcu -suppress 2902,13314 \
    +incdir+../../RTL \
    ../../RTL/std_rf.sv \
    ../../RTL/sdp.sv \
    ../../RTL/sram_model.sv \
    ../../RTL/AXI4/axi_arbiter.sv \
    ../../RTL/AXI4/axi_m_mux.sv \
    ../../RTL/AXI4/axi_lite_csr.sv \
    ../../RTL/DMA/idma.sv \
    ../../RTL/DMA/wdma.sv \
    ../../RTL/DMA/odma.sv \
    ../../RTL/cfg_regs.sv \
    ../../RTL/mac_pe.sv \
    ../../RTL/mac_col.sv \
    ../../RTL/mac_array.sv \
    ../../RTL/parf_accum.sv \
    ../../RTL/line_buffer.sv \
    ../../RTL/wgt_buffer.sv \
    ../../RTL/ofb_writer.sv \
    ../../RTL/core_top.sv \
    ../tb_axi_m_mux/axi_slave_mem.sv \
    tb_core_dma.sv
vsim -c -sva -f ../tb_core_isa/sim_params.f -L work work.tb_core_dma
run -all
quit -f
