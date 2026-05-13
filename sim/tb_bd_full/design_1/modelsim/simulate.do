onbreak {quit -f}
onerror {quit -f}

vsim -voptargs="+acc" -onfinish stop -L smartconnect_v1_0 -L axi_infrastructure_v1_1_0 -L axi_register_slice_v2_1_30 -L axi_vip_v1_1_16 -L lib_cdc_v1_0_2 -L proc_sys_reset_v5_0_14 -L xlconstant_v1_1_8 -L lib_pkg_v1_0_3 -L axi_apb_bridge_v3_0_19 -L versal_cips_ps_vip_v1_0_8 -L xil_defaultlib -L cpm4_v1_0_15 -L cpm5_v1_0_15 -L xlconcat_v2_1_5 -L xlslice_v1_0_3 -L xilinx_vip -L unisims_ver -L unimacro_ver -L secureip -L xpm -L axi_datamover_v5_1_30 -lib xil_defaultlib xil_defaultlib.tb_bd_top xil_defaultlib.glbl

set NumericStdNoWarnings 1
set StdArithNoWarnings 1

do {wave.do}

view wave
view structure
view signals

do {design_1.udo}

run -all

quit -force
