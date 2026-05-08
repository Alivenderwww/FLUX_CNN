# =============================================================================
# run_syn_vd100.tcl  --  Synthesis (OOC) for multicore_top_vd100 on VE2302
#
# Target  : xcve2302-sfva784-1LP-e-s (Versal AI Edge, ALINX VD100 板)
# Top     : multicore_top_vd100 (3 ConvCore, 不含 axi_smc IP, NoC 由 Vitis BD 提供)
# 用法    : vivado -mode batch -source Syn/run_syn_vd100.tcl -nojournal -log Syn/syn_vd100.log
#
# 跟 K325T (run_syn_smc.tcl) 区别:
#   - PART = xcve2302 (vs xc7k325t)
#   - Top = multicore_top_vd100 (vs multicore_top_smc), N=3
#   - 不需要 axi_smc_4to4 IP (直接 expose 3 个 AXI master 给 Vitis NoC)
#   - 不需要 axi_lite_1to4 IP (RTL 内部 1to3 demux)
#   - 仍需要 axi_dm IP (跨设备 IP, 但要 generate for VE2302)
# =============================================================================

set SCRIPT_DIR [file dirname [file normalize [info script]]]
set RTL_DIR    [file normalize "$SCRIPT_DIR/../RTL"]
set RPT_DIR    "$SCRIPT_DIR/reports_vd100"

set TOP_NAME           multicore_top_vd100_bd
set NUM_CORES          3

file mkdir $RPT_DIR

# -----------------------------------------------------------------------------
# 1. Create in-memory project for VE2302
# -----------------------------------------------------------------------------
set PART xcve2302-sfva784-1LP-e-s
puts "=== Target part: $PART ==="
create_project -in_memory -part $PART
set_property TARGET_LANGUAGE Verilog [current_project]

# -----------------------------------------------------------------------------
# 2. Read & re-generate axi_dm IP for VE2302
#    axi_datamover IP 是跨设备通用 IP, read_ip 时 Vivado 用当前 project 的 part
#    自动 retarget. 直接复用 K325T 的 axi_dm.xci 配置.
# -----------------------------------------------------------------------------
set IP_DIR     "$SCRIPT_DIR/ip_managed/ip_managed.srcs/sources_1/ip"
set ip_xci "$IP_DIR/axi_dm/axi_dm.xci"
if {![file exists $ip_xci]} {
    error "missing axi_dm xci: $ip_xci (run K325T IP gen first?)"
}
read_ip $ip_xci
generate_target -force synthesis [get_ips axi_dm]
foreach ip [get_ips] {
    puts "=== synth_ip $ip (retarget to VE2302) ==="
    synth_ip [get_ips $ip]
}
puts "=== axi_dm IP generated for VE2302 ==="

# -----------------------------------------------------------------------------
# 3. Add RTL sources (跟 sim/tb_smc/run.tcl 一致, 但 top 换 vd100)
# -----------------------------------------------------------------------------
set rtl_files [list \
    "$RTL_DIR/std_rf.sv"                          \
    "$RTL_DIR/sram_model.sv"                      \
    "$RTL_DIR/sdp.sv"                             \
    "$RTL_DIR/AXI4/axi_lite_csr.sv"               \
    "$RTL_DIR/AXI4/axi_m_mux.sv"                  \
    "$RTL_DIR/AXI4/axi_arbiter.sv"                \
    "$RTL_DIR/AXI4/ifb_axi_slave.sv"              \
    "$RTL_DIR/DMA/idma_ctrl.sv"                   \
    "$RTL_DIR/DMA/idma_sg_dispatcher.sv"          \
    "$RTL_DIR/DMA/wdma_ctrl.sv"                   \
    "$RTL_DIR/DMA/odma_ctrl.sv"                   \
    "$RTL_DIR/DMA/odma_sg_dispatcher.sv"          \
    "$RTL_DIR/DMA/rdma_ctrl.sv"                   \
    "$RTL_DIR/DMA/mm2s_arb.sv"                    \
    "$RTL_DIR/DMA/dfe.sv"                         \
    "$RTL_DIR/desc_fifo.sv"                       \
    "$RTL_DIR/sequencer.sv"                       \
    "$RTL_DIR/cfg_regs.sv"                        \
    "$RTL_DIR/mac_pe.sv"                          \
    "$RTL_DIR/mac_col.sv"                         \
    "$RTL_DIR/mac_array.sv"                       \
    "$RTL_DIR/mac_simd_pair.sv"                   \
    "$RTL_DIR/mac_array_simd.sv"                  \
    "$RTL_DIR/parf_col.sv"                        \
    "$RTL_DIR/parf_accum.sv"                      \
    "$RTL_DIR/line_buffer.sv"                     \
    "$RTL_DIR/wgt_buffer.sv"                      \
    "$RTL_DIR/bias_rf.sv"                         \
    "$RTL_DIR/ofb_writer.sv"                      \
    "$RTL_DIR/core_top.sv"                        \
    "$RTL_DIR/Versal/multicore_top_vd100.sv"      \
    "$RTL_DIR/Versal/multicore_top_vd100_bd.sv"   \
]
add_files $rtl_files
set_property FILE_TYPE {SystemVerilog} [get_files *.sv]
# 加 include path 让 RTL/Versal/multicore_top_vd100.sv 能找到 RTL/flux_cnn_params.svh
set_property include_dirs $RTL_DIR [current_fileset]

# axi_dm IP 暂时不 generate, set as black-box (USE_AXI_DM_IP=1 时 core_top 实例化它)
# 也可以加 generic USE_AXI_DM_IP=0 跳过 (需要 core_top.sv 改, 暂保留默认)

# -----------------------------------------------------------------------------
# 4. Clock constraint
# -----------------------------------------------------------------------------
set CLK_PERIOD_NS 10.0  ;# 100 MHz target (跟 ALINX 02_pl_rw_ddr 一致)

set xdc_file "$SCRIPT_DIR/cnn_vd100_ooc.xdc"
set fd [open $xdc_file w]
puts $fd "create_clock -name clk -period ${CLK_PERIOD_NS} \[get_ports clk\]"
close $fd
add_files -fileset constrs_1 $xdc_file
set_property USED_IN {synthesis implementation} [get_files $xdc_file]

# -----------------------------------------------------------------------------
# 5. OOC Synthesis (跳过 axi_dm IP, 把 IP 实例当 black-box 估算)
# -----------------------------------------------------------------------------
puts "=== Starting OOC Synthesis: $TOP_NAME, NUM_CORES=$NUM_CORES, clk=${CLK_PERIOD_NS}ns ==="

# 强制 axi_dm 模块 black-box (没 IP definition, OOC 模式 Vivado 会 warn 但不报错)
synth_design \
    -top              $TOP_NAME \
    -part             $PART     \
    -mode             out_of_context \
    -flatten_hierarchy rebuilt

puts "=== Synthesis Done ==="

# Post-synth utilization
report_utilization -file "$RPT_DIR/utilization_synth.rpt" \
                   -hierarchical -hierarchical_depth 4
puts "Written: $RPT_DIR/utilization_synth.rpt"

report_timing_summary -file "$RPT_DIR/timing_synth.rpt" -max_paths 10
puts "Written: $RPT_DIR/timing_synth.rpt"

puts ""
puts "=== Synth utilization snapshot ==="
report_utilization

# WNS summary
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts ""
puts "============================================================"
puts "  RESULT SUMMARY  ($PART, $TOP_NAME N=$NUM_CORES, clk = ${CLK_PERIOD_NS} ns)"
puts "============================================================"
puts "  WNS  : $wns ns"
if {$wns >= 0} {
    set fmax [expr {1000.0 / ($CLK_PERIOD_NS - $wns)}]
    puts "  Fmax : [format %.1f $fmax] MHz  (timing MET)"
} else {
    puts "  Fmax : timing VIOLATED at ${CLK_PERIOD_NS} ns"
}
puts "============================================================"
