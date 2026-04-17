vlib work
vmap work work
vlog -sv -work work -mfcu -suppress 2902,13314 \
    ../../RTL/sram_model.sv \
    ../../RTL/DMA/odma.sv \
    ../tb_axi_m_mux/axi_slave_mem.sv \
    tb_odma.sv
vsim -c -sva -L work work.tb_odma
run -all
quit -f
