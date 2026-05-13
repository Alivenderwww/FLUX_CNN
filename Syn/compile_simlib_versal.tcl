# =============================================================================
# compile_simlib_versal.tcl  --  Versal VE2302 (VD100) ModelSim simlib
#
# 跟 compile_simlib.tcl 同, 但 -family versal 编 Versal secureip / unisims / unimacro
# (SIP_XRAM, NoC NMU 等 Versal-specific cell).
# =============================================================================

set MODELSIM_BIN  {C:/modeltech64_2020.4/win64}
set SIMLIB_OUTDIR {C:/_Project/FLUX_CNN/Syn/sim_libs}

if {![file exists $SIMLIB_OUTDIR]} {
    file mkdir $SIMLIB_OUTDIR
}

puts "==> compile_simlib for ModelSim (Versal family)"
puts "    simulator_exec_path : $MODELSIM_BIN"
puts "    family              : versal  (VE2302/VD100)"
puts "    output dir          : $SIMLIB_OUTDIR"

compile_simlib \
    -simulator        modelsim                \
    -simulator_exec_path $MODELSIM_BIN        \
    -family           versal                  \
    -language         all                     \
    -library          all                     \
    -dir              $SIMLIB_OUTDIR          \
    -force

puts "==> done."
