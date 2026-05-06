# =============================================================================
# tb_mesh/run_mesh_2x2.tcl  --  2×2 mesh integration test
# =============================================================================

vlib work
vmap work work

vlog -sv -work work -mfcu -suppress 2902,13314 \
    +incdir+../../RTL \
    ../../RTL/Mesh/router_node.sv \
    ../../RTL/Mesh/mesh_2x2.sv \
    tb_mesh_2x2.sv

vsim -c -voptargs="+acc" \
    work.tb_mesh_2x2
run -all
quit -f
