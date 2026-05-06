vlib work
vmap work work

vlog -sv -work work -mfcu -suppress 2902,13314 \
    +incdir+../../RTL \
    ../../RTL/Mesh/axi_writer_to_axis.sv \
    tb_axi_writer_to_axis.sv

vsim -c -voptargs="+acc" \
    work.tb_axi_writer_to_axis
run -all
quit -f
