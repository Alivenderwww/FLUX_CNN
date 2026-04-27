# =============================================================================
# tb_axi_dm_smoke / run.tcl  --  AXI DataMover smoke test
#
# 验证: ModelSim 能否正确挂载 Vivado 仿真库 + 跨语言例化 axi_dm (VHDL IP)
#
# 跑法:
#   cd sim/tb_axi_dm_smoke
#   vsim -c -do run.tcl
# =============================================================================

# Vivado 导出的 modelsim.ini 含所有 axi_datamover_v5_1_30 / fifo_generator_v13_2_8 /
# lib_pkg_v1_0_2 等依赖库的映射, 拷过来当本地 ini base
set XLNX_INI    {C:/_Project/FLUX_CNN/Syn/ip_sim_export/axi_dm/modelsim/modelsim.ini}
set IP_GEN_DIR  {C:/_Project/FLUX_CNN/Syn/ip_managed/ip_managed.gen/sources_1/ip/axi_dm}

file copy -force $XLNX_INI ./modelsim.ini

vlib work
vmap work work
vlib xil_defaultlib
vmap xil_defaultlib xil_defaultlib

# ---- 编 axi_dm 顶层 wrapper (VHDL) 进 xil_defaultlib ----
# 它的依赖 (axi_datamover_v5_1, fifo_generator_v13_2, lib_pkg, ...) 已在 sim_libs/
# 预编好, 通过 modelsim.ini 自动找到
vcom -work xil_defaultlib -93 \
    "$IP_GEN_DIR/sim/axi_dm.vhd"

# ---- 编 Vivado glbl (Xilinx primitives 全局信号) ----
vlog -work xil_defaultlib \
    "C:/_Project/FLUX_CNN/Syn/ip_sim_export/axi_dm/modelsim/glbl.v"

# ---- 编 SV testbench ----
vlog -sv -work work -mfcu \
    tb_axi_dm_smoke.sv

# ---- 启动 vsim, 拉 xil_defaultlib + glbl ----
vsim -c -voptargs="+acc" \
    -L xil_defaultlib \
    -L axi_datamover_v5_1_30 \
    -L fifo_generator_v13_2_8 \
    -L lib_pkg_v1_0_2 \
    -L lib_cdc_v1_0_2 \
    -L lib_fifo_v1_0_17 \
    -L lib_srl_fifo_v1_0_2 \
    -L blk_mem_gen_v8_4_6 \
    -L unisims_ver \
    work.tb_axi_dm_smoke xil_defaultlib.glbl

run -all
quit -f
