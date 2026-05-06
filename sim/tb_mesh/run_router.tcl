# =============================================================================
# tb_mesh/run_router.tcl  --  router_node 单元测试
# =============================================================================

vlib work
vmap work work

vlog -sv -work work -mfcu -suppress 2902,13314 \
    +incdir+../../RTL \
    ../../RTL/Mesh/router_node.sv \
    ../../RTL/Mesh/mesh_2x2.sv \
    tb_router_node.sv \
    tb_mesh_2x2.sv

vsim -c -voptargs="+acc" \
    work.tb_router_node
run -all

# 跑完 router 单元后接 2x2 mesh 测试
vsim -c -voptargs="+acc" \
    work.tb_mesh_2x2
run -all
quit -f
