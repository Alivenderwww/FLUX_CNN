vlib work
vmap work work
vlog -sv -work work -mfcu -suppress 2902,13314 \
    ../../RTL/AXI4/axi_lite_csr.sv \
    tb_axi_lite_csr.sv
vsim -c -sva -L work work.tb_axi_lite_csr
run -all
quit -f
