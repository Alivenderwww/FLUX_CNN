vlib work
vmap work work
vlog -sv -work work -mfcu -suppress 2902,13314 \
    ../../RTL/AXI4/axi_arbiter.sv \
    ../../RTL/AXI4/axi_m_mux.sv \
    axi_slave_mem.sv \
    tb_axi_m_mux.sv
vsim -c -sva -L work work.tb_axi_m_mux
run -all
quit -f
