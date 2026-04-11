quit -sim                  
                           
if {[file exists tb_core_top/work]} {
  file delete -force tb_core_top/work  
}                          
                           
vlib tb_core_top/work              
vmap work tb_core_top/work         
                           
vlog -sv -work tb_core_top/work -mfcu -incr -suppress 2902 -f sim_file_list.f
vsim -suppress 3486,3680,3781 -voptargs="+acc" +nowarn1 -c -sva \
     -L tb_core_top/work\
     tb_core_top
run -all
quit
