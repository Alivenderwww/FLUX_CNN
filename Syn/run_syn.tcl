# =============================================================================
# run_syn.tcl  --  CNN Accelerator Out-of-Context Synthesis Script
#
# Target : Xilinx xc7k325tffg900-2 (Kintex-7 XC7K325T)
#          (匹配 axi_dm IP 生成时用的 part — gen_axi_datamover.tcl)
# Flow   : in-memory project -> add RTL + IP -> synth_design (OOC) -> P&R -> reports
#
# Usage  : vivado -mode tcl -source run_syn.tcl -log syn_run.log -nojournal
# =============================================================================

set SCRIPT_DIR [file dirname [file normalize [info script]]]
set RTL_DIR    [file normalize "$SCRIPT_DIR/../RTL"]
set RPT_DIR    "$SCRIPT_DIR/reports"
set IP_DIR     "$SCRIPT_DIR/ip_managed/ip_managed.srcs/sources_1/ip"

# 顶层 = multicore_top (含 NUM_CORES 个 core_top + 2 个 IP crossbar)
# 跑 single-core 的话改回 core_top 即可 (只 read axi_dm IP)
set TOP_NAME   multicore_top
set NUM_CORES  2

file mkdir $RPT_DIR

# -----------------------------------------------------------------------------
# 1. Create in-memory project
# -----------------------------------------------------------------------------
set PART xc7k325tffg900-2
create_project -in_memory -part $PART
set_property TARGET_LANGUAGE Verilog [current_project]

# -----------------------------------------------------------------------------
# 2. Read & synth IPs
#    axi_dm:        DataMover (DMA backbone, MM2S+S2MM)
#    axi_${N}to1:   N-core master 聚合到单 DDR 端口
#    axi_lite_1to${N}: host CSR 1:N 分发
# -----------------------------------------------------------------------------
set ip_list [list \
    "$IP_DIR/axi_dm/axi_dm.xci" \
]
if {$TOP_NAME eq "multicore_top"} {
    # 找 axi_${NUM_CORES}to1 (Vivado 可能在 _2 / _3 后缀目录)
    set agg_xci [glob -nocomplain "$IP_DIR/axi_${NUM_CORES}to1*/axi_${NUM_CORES}to1.xci"]
    set csr_xci [glob -nocomplain "$IP_DIR/axi_lite_1to${NUM_CORES}*/axi_lite_1to${NUM_CORES}.xci"]
    if {[llength $agg_xci] == 0} { error "missing axi_${NUM_CORES}to1 IP, run gen_multicore_ip.tcl first" }
    if {[llength $csr_xci] == 0} { error "missing axi_lite_1to${NUM_CORES} IP, run gen_multicore_ip.tcl first" }
    lappend ip_list [lindex $agg_xci 0]
    lappend ip_list [lindex $csr_xci 0]
}
foreach xci $ip_list {
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
# 3. Add RTL source files (按 leaf → top 顺序, 跟 sim/run.tcl 一致)
# -----------------------------------------------------------------------------
set rtl_files [list \
    "$RTL_DIR/std_rf.sv"               \
    "$RTL_DIR/sram_model.sv"           \
    "$RTL_DIR/sdp.sv"                  \
    "$RTL_DIR/AXI4/axi_lite_csr.sv"    \
    "$RTL_DIR/AXI4/axi_m_mux.sv"       \
    "$RTL_DIR/AXI4/axi_arbiter.sv"     \
    "$RTL_DIR/DMA/idma_ctrl.sv"        \
    "$RTL_DIR/DMA/wdma_ctrl.sv"        \
    "$RTL_DIR/DMA/odma_ctrl.sv"        \
    "$RTL_DIR/DMA/rdma_ctrl.sv"        \
    "$RTL_DIR/DMA/mm2s_arb.sv"         \
    "$RTL_DIR/DMA/dfe.sv"              \
    "$RTL_DIR/desc_fifo.sv"            \
    "$RTL_DIR/sequencer.sv"            \
    "$RTL_DIR/cfg_regs.sv"             \
    "$RTL_DIR/mac_pe.sv"               \
    "$RTL_DIR/mac_col.sv"              \
    "$RTL_DIR/mac_array.sv"            \
    "$RTL_DIR/parf_col.sv"             \
    "$RTL_DIR/parf_accum.sv"           \
    "$RTL_DIR/line_buffer.sv"          \
    "$RTL_DIR/wgt_buffer.sv"           \
    "$RTL_DIR/bias_rf.sv"              \
    "$RTL_DIR/ofb_writer.sv"           \
    "$RTL_DIR/core_top.sv"             \
    "$RTL_DIR/multicore_top.sv"        \
]
add_files $rtl_files
set_property FILE_TYPE {SystemVerilog} [get_files *.sv]

# -----------------------------------------------------------------------------
# 4. Clock constraint
# -----------------------------------------------------------------------------
set CLK_PERIOD_NS 10.0

set xdc_file "$SCRIPT_DIR/cnn_ooc.xdc"
set fd [open $xdc_file w]
puts $fd "create_clock -name clk -period ${CLK_PERIOD_NS} \[get_ports clk\]"
close $fd

add_files -fileset constrs_1 $xdc_file
set_property USED_IN {synthesis implementation} [get_files $xdc_file]

# -----------------------------------------------------------------------------
# 5. Synthesis (OOC, flatten_hierarchy=full for accurate resource count)
# -----------------------------------------------------------------------------
puts "=== Starting Synthesis (clk = ${CLK_PERIOD_NS} ns) ==="

if {$TOP_NAME eq "multicore_top"} {
    synth_design \
        -top              $TOP_NAME  \
        -part             $PART      \
        -mode             out_of_context \
        -flatten_hierarchy rebuilt   \
        -generic [list NUM_CORES=$NUM_CORES \
                       NUM_COL=16 NUM_PE=16 DATA_WIDTH=8 PSUM_WIDTH=32 \
                       WRF_DEPTH=32 ARF_DEPTH=32 PARF_DEPTH=32 \
                       SRAM_DEPTH=8192]
} else {
    synth_design \
        -top              $TOP_NAME  \
        -part             $PART      \
        -mode             out_of_context \
        -flatten_hierarchy rebuilt   \
        -generic [list NUM_COL=16 NUM_PE=16 DATA_WIDTH=8 PSUM_WIDTH=32 \
                       WRF_DEPTH=32 ARF_DEPTH=32 PARF_DEPTH=32 \
                       SRAM_DEPTH=8192]
}

puts "=== Synthesis Done ==="

# Post-synth utilization
report_utilization \
    -file "$RPT_DIR/utilization_synth.rpt" \
    -hierarchical -hierarchical_depth 4
puts "Written: utilization_synth.rpt"

# Post-synth timing
report_timing_summary \
    -file "$RPT_DIR/timing_synth.rpt" \
    -max_paths 5
puts "Written: timing_synth.rpt"

# -----------------------------------------------------------------------------
# 6. Implementation (Place & Route)
# -----------------------------------------------------------------------------
puts "=== Starting Implementation ==="
opt_design
place_design
phys_opt_design
route_design
puts "=== Implementation Done ==="

# -----------------------------------------------------------------------------
# 7. Post-implementation reports
# -----------------------------------------------------------------------------
report_utilization \
    -file "$RPT_DIR/utilization.rpt" \
    -hierarchical -hierarchical_depth 4
puts "Written: utilization.rpt"

report_timing_summary \
    -file "$RPT_DIR/timing_summary.rpt" \
    -max_paths 10
puts "Written: timing_summary.rpt"

# -----------------------------------------------------------------------------
# 8. Console summary
# -----------------------------------------------------------------------------
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]

puts ""
puts "============================================================"
puts "  RESULT SUMMARY  ($PART, clk = ${CLK_PERIOD_NS} ns)"
puts "============================================================"
puts "  WNS  : $wns ns"
if {$wns >= 0} {
    set fmax [expr {1000.0 / ($CLK_PERIOD_NS - $wns)}]
    puts "  Fmax : [format %.1f $fmax] MHz  (timing MET)"
} else {
    puts "  Fmax : timing VIOLATED at ${CLK_PERIOD_NS} ns  (need longer period)"
}
puts ""
puts "  Full reports: $RPT_DIR/"
puts "============================================================"
