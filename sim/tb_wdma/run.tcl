vlib work
vmap work work
vlog -sv -work work -mfcu -suppress 2902,13314 \
    ../../RTL/sram_model.sv \
    ../../RTL/DMA/wdma.sv \
    ../tb_axi_m_mux/axi_slave_mem.sv \
    tb_wdma.sv
vsim -c -sva -L work work.tb_wdma
run -all
quit -f
