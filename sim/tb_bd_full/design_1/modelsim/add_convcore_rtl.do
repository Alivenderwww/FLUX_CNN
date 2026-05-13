# 在 compile.do 跑完后 (BD sim source 已编), 补编 ConvCore RTL (multicore_top_vd100_bd_v + 全 deps)
# multicore_top_vd100_bd_v.v 是 BD module reference, BD sim 不知道它的 underlying RTL,
# 需要手动 add.
#
# ConvCore RTL 编译顺序参考 sim/tb_smc/run.tcl, 但跳过 sim-only 的 hack (axi_crossbar_4to4_sim,
# axi_slave_mem) — BD sim 有真 axi_noc + DDR4 model.

# 编 ConvCore RTL 到 xil_defaultlib (跟 BD sim source 同 lib)
set FLUX_RTL ../../../../RTL
vlog -sv -work xil_defaultlib -mfcu -suppress 2902,13314 \
    +incdir+$FLUX_RTL \
    $FLUX_RTL/std_rf.sv \
    $FLUX_RTL/sdp.sv \
    $FLUX_RTL/sram_model.sv \
    $FLUX_RTL/AXI4/axi_arbiter.sv \
    $FLUX_RTL/AXI4/axi_m_mux.sv \
    $FLUX_RTL/AXI4/axi_lite_csr.sv \
    $FLUX_RTL/AXI4/ifb_axi_slave.sv \
    $FLUX_RTL/desc_fifo.sv \
    $FLUX_RTL/sequencer.sv \
    $FLUX_RTL/cfg_regs.sv \
    $FLUX_RTL/mac_pe.sv \
    $FLUX_RTL/mac_col.sv \
    $FLUX_RTL/mac_array.sv \
    $FLUX_RTL/mac_simd_pair.sv \
    $FLUX_RTL/mac_array_simd.sv \
    $FLUX_RTL/parf_col.sv \
    $FLUX_RTL/parf_accum.sv \
    $FLUX_RTL/line_buffer.sv \
    $FLUX_RTL/wgt_buffer.sv \
    $FLUX_RTL/bias_rf.sv \
    $FLUX_RTL/ofb_writer.sv \
    $FLUX_RTL/DMA/idma_ctrl.sv \
    $FLUX_RTL/DMA/idma_sg_dispatcher.sv \
    $FLUX_RTL/DMA/wdma_ctrl.sv \
    $FLUX_RTL/DMA/odma_ctrl.sv \
    $FLUX_RTL/DMA/odma_sg_dispatcher.sv \
    $FLUX_RTL/DMA/rdma_ctrl.sv \
    $FLUX_RTL/DMA/mm2s_arb.sv \
    $FLUX_RTL/DMA/dfe.sv \
    $FLUX_RTL/core_top.sv

# axi_dm IP (VHDL): top entity 加载, axi_datamover_v5_1_32 + fifo_gen + lib_* 都来自 simlib
set IP_DM_DIR ../../../../Syn/ip_managed/ip_managed.gen/sources_1/ip/axi_dm
vcom -work xil_defaultlib -93 \
    $IP_DM_DIR/sim/axi_dm.vhd

# multicore_top_vd100.sv (N-core wrapper) + multicore_top_vd100_bd.sv (BD wrapper)
vlog -sv -work xil_defaultlib -mfcu -suppress 2902,13314 \
    +incdir+$FLUX_RTL \
    $FLUX_RTL/Versal/multicore_top_vd100.sv \
    $FLUX_RTL/Versal/multicore_top_vd100_bd.sv

# multicore_top_vd100_bd_v.v 是 Verilog wrapper for BD instantiation
vlog -work xil_defaultlib -mfcu \
    +incdir+$FLUX_RTL \
    $FLUX_RTL/Versal/multicore_top_vd100_bd_v.v

# design_1_wrapper.v: BD top wrapper (实例化 design_1 + DDR4 ports)
vlog -work xil_defaultlib -mfcu \
    ../../../../Syn/vd100_bd/output/vd100_resnet11.gen/sources_1/bd/design_1/hdl/design_1_wrapper.v

# tb_bd_top.sv: testbench top (drive sys_clk + dangling DDR4 ports + hier-ref monitor)
vlog -sv -work xil_defaultlib -mfcu \
    +incdir+$FLUX_RTL \
    ../../tb_bd_top.sv

puts "===== ConvCore RTL compile OK ====="
