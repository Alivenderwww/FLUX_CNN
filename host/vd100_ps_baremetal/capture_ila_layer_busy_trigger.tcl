# =============================================================================
# capture_ila_layer_busy_trigger.tcl
# 设 ILA trigger on dbg_layer_busy_0 EQ 1 (sequencer 接 start_layer 进 S_FETCH 那拍)
# armed + RUN, 等 PC test_layer0_real.py 触发. capture 后 dump csv.
# =============================================================================
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
puts "Using: $ila"

# Trigger: dbg_layer_busy_0 == 1 (上升沿: ILA armed 时 layer_busy=0, 等 0→1)
set probe_lb [get_hw_probes -of_objects $ila design_1_i/u_mc_vd100_dbg_layer_busy_0]
set_property TRIGGER_COMPARE_VALUE eq1'b1 $probe_lb
puts "Trigger: dbg_layer_busy_0 == 1'b1 (sequencer S_IDLE→S_FETCH 那拍 + 之后)"

# Pre-trigger 64, post-trigger 960 → 看 layer_busy 拉高前 ~0.6us + 之后 ~9.6us
set_property CONTROL.TRIGGER_POSITION 64 $ila
set_property CONTROL.WINDOW_COUNT 1 $ila
set_property CONTROL.DATA_DEPTH 1024 $ila

run_hw_ila $ila
puts "ILA armed, waiting for trigger (PC test_layer0_real.py 触发)..."

# 等 30s, PC 端有时间烧/启 a72/preload/跑 phase 1 + 2
if {[catch {wait_on_hw_ila -timeout 60 $ila} werr]} {
    puts "  wait err: $werr (ILA 可能没 trigger, dump 当前 buffer)"
}

upload_hw_ila_data $ila
set data [current_hw_ila_data]

set csv C:/_Project/FLUX_CNN/Syn/vd100_bd/output/ila_layer_busy.csv
write_hw_ila_data -force -csv_file $csv $data
puts "  CSV: $csv"

set vcd C:/_Project/FLUX_CNN/Syn/vd100_bd/output/ila_layer_busy.vcd
write_hw_ila_data -force -vcd_file $vcd $data
puts "  VCD: $vcd"

close_hw_target
disconnect_hw_server
close_hw_manager
exit 0
