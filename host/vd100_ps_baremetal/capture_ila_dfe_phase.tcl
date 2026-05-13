# Trigger on start_dfe pulse → 抓 phase 1 dfe 拉 desc 全过程 (1024 拍 = 10us 装得下)
set LTX C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.ltx
open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target
set dev [lindex [get_hw_devices xcve*] 0]
current_hw_device $dev
set_property PROBES.FILE $LTX $dev
set_property FULL_PROBES.FILE $LTX $dev
refresh_hw_device $dev
set ila [lindex [get_hw_ilas] 0]
current_hw_ila $ila

set probe_start [get_hw_probes -of_objects $ila design_1_i/u_mc_vd100_dbg_start_dfe_pulse_0]
set_property TRIGGER_COMPARE_VALUE eq1'b1 $probe_start
set_property CONTROL.TRIGGER_POSITION 32 $ila
set_property CONTROL.WINDOW_COUNT 1 $ila
set_property CONTROL.DATA_DEPTH 1024 $ila

run_hw_ila $ila
puts "ILA armed on dbg_start_dfe_pulse_0==1..."
if {[catch {wait_on_hw_ila -timeout 60 $ila} werr]} {puts "wait err: $werr"}
upload_hw_ila_data $ila
set data [current_hw_ila_data]
write_hw_ila_data -force -csv_file C:/_Project/FLUX_CNN/Syn/vd100_bd/output/ila_dfe_phase.csv $data
puts "  CSV: ila_dfe_phase.csv"
close_hw_target
disconnect_hw_server
close_hw_manager
exit 0
