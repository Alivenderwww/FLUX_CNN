# =============================================================================
# run_syn.tcl  --  CNN Accelerator Out-of-Context Synthesis Script
#
# Target : Xilinx xcku060-ffva1156-2-e (UltraScale)
# Flow   : in-memory project -> add RTL -> add XDC clock constraint
#          -> synth_design (OOC) -> opt -> P&R -> reports
#
# Usage  : vivado -mode tcl -source run_syn.tcl -log syn_run.log -nojournal
# =============================================================================

set SCRIPT_DIR [file dirname [file normalize [info script]]]
set RTL_DIR    [file normalize "$SCRIPT_DIR/../RTL"]
set RPT_DIR    "$SCRIPT_DIR/reports"

file mkdir $RPT_DIR

# -----------------------------------------------------------------------------
# 1. Create in-memory project
# -----------------------------------------------------------------------------
create_project -in_memory -part xcku060-ffva1156-2-e
set_property TARGET_LANGUAGE Verilog [current_project]

# -----------------------------------------------------------------------------
# 2. Add RTL source files
# -----------------------------------------------------------------------------
set rtl_files [list \
    "$RTL_DIR/sram_model.sv"   \
    "$RTL_DIR/std_rf.sv"       \
    "$RTL_DIR/sdp.sv"          \
    "$RTL_DIR/cfg_regs.sv"     \
    "$RTL_DIR/mac_pe.sv"       \
    "$RTL_DIR/mac_col.sv"      \
    "$RTL_DIR/mac_array.sv"    \
    "$RTL_DIR/parf_accum.sv"   \
    "$RTL_DIR/line_buffer.sv"  \
    "$RTL_DIR/wgt_buffer.sv"   \
    "$RTL_DIR/ofb_writer.sv"   \
    "$RTL_DIR/core_top.sv"     \
]
add_files $rtl_files
set_property FILE_TYPE {SystemVerilog} [get_files *.sv]

# -----------------------------------------------------------------------------
# 3. Clock constraint (XDC inline) -- MUST be set before synth for OOC
#    Target: 100 MHz (10 ns).  Adjust CLK_PERIOD_NS to explore timing.
# -----------------------------------------------------------------------------
set CLK_PERIOD_NS 10.0

set xdc_file "$SCRIPT_DIR/cnn_ooc.xdc"
set fd [open $xdc_file w]
puts $fd "create_clock -name clk -period ${CLK_PERIOD_NS} \[get_ports clk\]"
puts $fd "set_property HD.CLK_SRC BUFGCTRL_X0Y0 \[get_ports clk\]"
close $fd

add_files -fileset constrs_1 $xdc_file
set_property USED_IN {synthesis implementation} [get_files $xdc_file]

# -----------------------------------------------------------------------------
# 4. Synthesis (OOC, flatten_hierarchy=full for accurate resource count)
# -----------------------------------------------------------------------------
puts "=== Starting Synthesis (clk = ${CLK_PERIOD_NS} ns) ==="

synth_design \
    -top              core_top               \
    -part             xcku060-ffva1156-2-e   \
    -mode             out_of_context         \
    -flatten_hierarchy full                  \
    -generic [list NUM_COL=16 NUM_PE=16 DATA_WIDTH=8 PSUM_WIDTH=32 \
                   WRF_DEPTH=32 ARF_DEPTH=32 PARF_DEPTH=32 \
                   SRAM_DEPTH=8192]

puts "=== Synthesis Done ==="

# Post-synth utilization (before P&R)
report_utilization \
    -file "$RPT_DIR/utilization_synth.rpt" \
    -hierarchical -hierarchical_depth 4
puts "Written: utilization_synth.rpt"

# Post-synth timing estimate
report_timing_summary \
    -file "$RPT_DIR/timing_synth.rpt" \
    -max_paths 5
puts "Written: timing_synth.rpt"

# -----------------------------------------------------------------------------
# 5. Implementation (Place & Route)
# -----------------------------------------------------------------------------
puts "=== Starting Implementation ==="
opt_design
place_design
phys_opt_design
route_design
puts "=== Implementation Done ==="

# -----------------------------------------------------------------------------
# 6. Post-implementation reports
# -----------------------------------------------------------------------------

# Utilization (final)
report_utilization \
    -file "$RPT_DIR/utilization.rpt" \
    -hierarchical -hierarchical_depth 4
puts "Written: utilization.rpt"

# Timing (final, with actual routing delays)
report_timing_summary \
    -file "$RPT_DIR/timing_summary.rpt" \
    -max_paths 10
puts "Written: timing_summary.rpt"

# Power (vector-less, 12.5% default toggle rate)
report_power \
    -file "$RPT_DIR/power.rpt" \
    -hierarchical_depth 4
puts "Written: power.rpt"

# -----------------------------------------------------------------------------
# 7. Console summary
# -----------------------------------------------------------------------------
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
set lut_used  [llength [get_cells -hierarchical -filter {REF_NAME =~ LUT*}]]

puts ""
puts "============================================================"
puts "  RESULT SUMMARY  (xcku060, clk = ${CLK_PERIOD_NS} ns)"
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
