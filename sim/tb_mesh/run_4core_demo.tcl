# =============================================================================
# tb_mesh/run_4core_demo.tcl  --  4 ConvCore + 1 Mem Core 5x1 mesh demo
# =============================================================================

vlib work
vmap work work

vlog -sv -work work -mfcu -suppress 2902,13314 \
    +incdir+../../RTL \
    ../../RTL/Mesh/router_node.sv \
    ../../RTL/Mesh/mesh_5x1.sv \
    ../../RTL/Mesh/axis_packet_rx.sv \
    ../../RTL/Mesh/axis_packet_tx.sv \
    ../../RTL/Mesh/conv_core_stub.sv \
    ../../RTL/Mesh/mem_core_stub.sv \
    tb_mesh_4core_demo.sv

vsim -c -voptargs="+acc" \
    work.tb_mesh_4core_demo
run -all
quit -f
