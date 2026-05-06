# =============================================================================
# tb_mesh/run_4x2_chain.tcl  --  4x2 mesh (4 Mem + 4 Conv) chain profile
# =============================================================================

vlib work
vmap work work

vlog -sv -work work -mfcu -suppress 2902,13314 \
    +incdir+../../RTL \
    ../../RTL/Mesh/router_node.sv \
    ../../RTL/Mesh/mesh_4x2.sv \
    ../../RTL/Mesh/axis_packet_rx.sv \
    ../../RTL/Mesh/axis_packet_tx.sv \
    ../../RTL/Mesh/conv_core_stub.sv \
    ../../RTL/Mesh/mem_core_stub.sv \
    tb_mesh_4x2_chain.sv

vsim -c -voptargs="+acc" \
    work.tb_mesh_4x2_chain
run -all
quit -f
