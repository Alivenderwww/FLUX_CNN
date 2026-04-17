vlib work
vmap work work
vlog -sv -work work -mfcu -incr -suppress 2902 -f sim_file_list.f
# -novopt removed (+acc) → full optimizer enabled; -wlf removed → no waveform dump
# Both are major speedups for regression (batch -c mode needs neither)
vsim -suppress 3486,3680,3781 -voptargs="+acc" +nowarn1 -c -sva -f sim_params.f -L work work.tb_core_isa
# add wave calls omitted: irrelevant in -c mode, waste time building signal DB
run -all
# quit -f