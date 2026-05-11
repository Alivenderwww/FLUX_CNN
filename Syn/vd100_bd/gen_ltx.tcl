open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_run impl_1
set ltx C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.ltx
write_debug_probes -force $ltx
puts "wrote $ltx"
exit
