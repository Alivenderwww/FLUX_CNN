# =============================================================================
# tb_mesh/run_mesh_to_ifb.tcl  --  Phase 6 端到端: mesh axis → axi → IFB SRAM
# =============================================================================

vlib work
vmap work work

vlog -sv -work work -mfcu -suppress 2902,13314 \
    +incdir+../../RTL \
    ../../RTL/Mesh/axis_to_axi_writer.sv \
    ../../RTL/AXI4/ifb_axi_slave.sv \
    ../../RTL/sram_model.sv \
    tb_mesh_to_ifb.sv

vsim -c -voptargs="+acc" \
    work.tb_mesh_to_ifb
run -all
quit -f
