# =============================================================================
# run_syn_smc.tcl  --  Synthesis for multicore_top_smc (Phase 7 SMC + NUMA)
#
# Target : Xilinx xc7k325tffg900-2 (Kintex-7)
# Top    : multicore_top_smc (4 ConvCore + axi_smc_4to4 IP + 4 axi_slave_mem stub)
#
# 注意: multicore_top_smc 内的 axi_slave_mem 是 sim 行为级 mem.
#   - 真硬件 mem 是 board 上 DDR + axi_dma 控制器, 不上 FPGA
#   - 综合时把 SLAVE_MEM_DEPTH 设小 (4096 words = 64 KB / mem) 仅占少量 BRAM
#   - 资源报告反映: ConvCore × 4 + axi_smc_4to4 IP + axi_lite_1to4 IP + 256 KB BRAM stub
#
# Usage  : vivado -mode batch -source Syn/run_syn_smc.tcl -nojournal -log Syn/syn_smc.log
# =============================================================================

set SCRIPT_DIR [file dirname [file normalize [info script]]]
set RTL_DIR    [file normalize "$SCRIPT_DIR/../RTL"]
set RPT_DIR    "$SCRIPT_DIR/reports_smc"
set IP_DIR     "$SCRIPT_DIR/ip_managed/ip_managed.srcs/sources_1/ip"

set TOP_NAME           multicore_top_smc
set NUM_CORES          4
set SLAVE_MEM_DEPTH    0              ;# 0 = synth mode 不实例化 axi_slave_mem
                                       ;# m_axi_* 出口 expose, 真硬件接 board DDR
                                       ;# (sim 时用 1M words 装 ResNet11 数据)

file mkdir $RPT_DIR

# -----------------------------------------------------------------------------
# 1. Create in-memory project
# -----------------------------------------------------------------------------
set PART xc7k325tffg900-2
create_project -in_memory -part $PART
set_property TARGET_LANGUAGE Verilog [current_project]

# -----------------------------------------------------------------------------
# 2. Read & synth IPs (SMC: axi_dm + axi_smc_4to4 + axi_lite_1to4)
# -----------------------------------------------------------------------------
set ip_list [list \
    "$IP_DIR/axi_dm/axi_dm.xci" \
    "$IP_DIR/axi_smc_4to4/axi_smc_4to4.xci" \
    "$IP_DIR/axi_lite_1to4/axi_lite_1to4.xci" \
]
foreach xci $ip_list {
    if {![file exists $xci]} { error "missing IP: $xci" }
    puts "=== read_ip $xci ==="
    read_ip $xci
}
generate_target -force synthesis [get_ips]
foreach ip [get_ips] {
    puts "=== synth_ip $ip ==="
    synth_ip [get_ips $ip]
}
puts "=== All IPs synthesized ==="

# -----------------------------------------------------------------------------
# 3. Add RTL source files (跟 sim/tb_smc/run_chain.tcl 一致, 加 SMC SG dispatcher)
# -----------------------------------------------------------------------------
set rtl_files [list \
    "$RTL_DIR/std_rf.sv"                          \
    "$RTL_DIR/sram_model.sv"                      \
    "$RTL_DIR/sdp.sv"                             \
    "$RTL_DIR/AXI4/axi_lite_csr.sv"               \
    "$RTL_DIR/AXI4/axi_m_mux.sv"                  \
    "$RTL_DIR/AXI4/axi_arbiter.sv"                \
    "$RTL_DIR/AXI4/ifb_axi_slave.sv"              \
    "$RTL_DIR/AXI4/axi_crossbar_4to4_sim.sv"      \
    "$SCRIPT_DIR/../sim/tb_axi_m_mux/axi_slave_mem.sv" \
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
    "$RTL_DIR/parf_col.sv"                        \
    "$RTL_DIR/parf_accum.sv"                      \
    "$RTL_DIR/line_buffer.sv"                     \
    "$RTL_DIR/wgt_buffer.sv"                      \
    "$RTL_DIR/bias_rf.sv"                         \
    "$RTL_DIR/ofb_writer.sv"                      \
    "$RTL_DIR/core_top.sv"                        \
    "$RTL_DIR/multicore_top_smc.sv"               \
]
add_files $rtl_files
set_property FILE_TYPE {SystemVerilog} [get_files *.sv]

# +define USE_AXI_SMC_IP: 让 multicore_top_smc 走 IP 路径 (vs sim model crossbar)
set_property verilog_define USE_AXI_SMC_IP=1 [current_fileset]

# -----------------------------------------------------------------------------
# 4. Clock constraint
# -----------------------------------------------------------------------------
set CLK_PERIOD_NS 10.0   ;# 100 MHz target

set xdc_file "$SCRIPT_DIR/cnn_smc_ooc.xdc"
set fd [open $xdc_file w]
puts $fd "create_clock -name clk -period ${CLK_PERIOD_NS} \[get_ports clk\]"
close $fd

add_files -fileset constrs_1 $xdc_file
set_property USED_IN {synthesis implementation} [get_files $xdc_file]

# -----------------------------------------------------------------------------
# 5. Synthesis (OOC)
# -----------------------------------------------------------------------------
puts "=== Starting Synthesis: $TOP_NAME, NUM_CORES=$NUM_CORES, SLAVE_MEM_DEPTH=$SLAVE_MEM_DEPTH, clk=${CLK_PERIOD_NS}ns ==="

synth_design \
    -top              $TOP_NAME \
    -part             $PART     \
    -mode             out_of_context \
    -flatten_hierarchy rebuilt   \
    -generic [list NUM_CORES=$NUM_CORES \
                   SLAVE_MEM_DEPTH=$SLAVE_MEM_DEPTH]
    # 不用 -generic 覆盖 SRAM_DEPTH / NUM_COL / NUM_PE 等, 让 svh 默认值生效
    # (params.py trim 后 IFB=1024 / SHB=2560 必须靠 svh 才能 take effect)

puts "=== Synthesis Done ==="

# Post-synth utilization
report_utilization -file "$RPT_DIR/utilization_synth.rpt" \
                   -hierarchical -hierarchical_depth 4
puts "Written: $RPT_DIR/utilization_synth.rpt"

report_timing_summary -file "$RPT_DIR/timing_synth.rpt" -max_paths 5
puts "Written: $RPT_DIR/timing_synth.rpt"

# Console snapshot of utilization
puts ""
puts "=== Synth utilization snapshot ==="
report_utilization

# -----------------------------------------------------------------------------
# 6. Implementation (P&R)
# -----------------------------------------------------------------------------
puts "=== Starting Implementation ==="
opt_design
place_design
phys_opt_design
route_design
puts "=== Implementation Done ==="

# -----------------------------------------------------------------------------
# 7. Post-impl reports
# -----------------------------------------------------------------------------
report_utilization -file "$RPT_DIR/utilization.rpt" \
                   -hierarchical -hierarchical_depth 4
puts "Written: $RPT_DIR/utilization.rpt"

report_timing_summary -file "$RPT_DIR/timing_summary.rpt" -max_paths 10
puts "Written: $RPT_DIR/timing_summary.rpt"

# -----------------------------------------------------------------------------
# 8. Console summary
# -----------------------------------------------------------------------------
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
puts ""
puts "  Full reports: $RPT_DIR/"
puts "============================================================"
