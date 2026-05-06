# =============================================================================
# tb_mesh/run_packet_io.tcl  --  axis_packet_rx/tx + router 端到端
# =============================================================================

vlib work
vmap work work

vlog -sv -work work -mfcu -suppress 2902,13314 \
    +incdir+../../RTL \
    ../../RTL/Mesh/router_node.sv \
    ../../RTL/Mesh/axis_packet_rx.sv \
    ../../RTL/Mesh/axis_packet_tx.sv \
    tb_packet_io.sv

vsim -c -voptargs="+acc" \
    work.tb_packet_io
run -all
quit -f
