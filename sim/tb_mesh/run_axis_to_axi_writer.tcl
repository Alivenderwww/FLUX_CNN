# =============================================================================
# tb_mesh/run_axis_to_axi_writer.tcl  --  axis_to_axi_writer 单元测试
# =============================================================================

vlib work
vmap work work

vlog -sv -work work -mfcu -suppress 2902,13314 \
    +incdir+../../RTL \
    ../../RTL/Mesh/axis_to_axi_writer.sv \
    tb_axis_to_axi_writer.sv

vsim -c -voptargs="+acc" \
    work.tb_axis_to_axi_writer
run -all
quit -f
