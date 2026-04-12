vlib work
vmap work work
vlog -sv -f sim_file_list.f
vsim -voptargs="+acc" work.tb_core_isa
add wave -position insertpoint sim:/tb_core_isa/u_core_top/clk
add wave -position insertpoint sim:/tb_core_isa/u_core_top/rst_n
add wave -position insertpoint sim:/tb_core_isa/u_core_top/start
add wave -position insertpoint sim:/tb_core_isa/u_core_top/done
add wave -position insertpoint sim:/tb_core_isa/u_core_top/u_ctrl/current_state
add wave -position insertpoint sim:/tb_core_isa/u_core_top/u_ctrl/pc
add wave -position insertpoint sim:/tb_core_isa/u_core_top/u_ctrl/inst_reg
add wave -position insertpoint sim:/tb_core_isa/u_core_top/u_mac_array/compute_en
add wave -position insertpoint sim:/tb_core_isa/u_core_top/u_mac_array/act_in_vec
add wave -position insertpoint sim:/tb_core_isa/u_core_top/u_mac_array/psum_out_vec
run -all