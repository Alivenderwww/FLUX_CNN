# =============================================================================
# vivado_capture_ila.tcl  --  Vivado HW Manager 自动: load ltx → set trigger → run
#
# 用 vivado -mode batch -source 调用. 注意: 必须跟 RPC trigger 同步.
# 流程: 这个脚本只设置 ILA armed, 真正的 RPC trigger 由 Python 平行跑.
#
# 用法:
#   1. 此脚本 background 跑 (vivado batch armed ILA)
#   2. Python 端 RPC poke CTRL=0x10 触发 ConvCore start_dfe
#   3. ILA 触发, 抓 waveform, dump
# =============================================================================
set LTX C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.ltx

open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target

# 选 Versal device
set dev [lindex [get_hw_devices xcve*] 0]
puts "Target: $dev"
current_hw_device $dev

# 加载 probes
puts "=== Load probes from $LTX ==="
set_property PROBES.FILE $LTX $dev
set_property FULL_PROBES.FILE $LTX $dev
refresh_hw_device $dev

puts "=== Available ILAs ==="
foreach ila [get_hw_ilas] { puts "  $ila" }
set ila [lindex [get_hw_ilas] 0]
if {$ila eq ""} { puts "no ILA"; exit 1 }
current_hw_ila $ila
puts "Using: $ila"

puts "=== ILA probes ==="
foreach p [get_hw_probes -of_objects $ila] { puts "  $p" }

# 用 trigger_now 抓 ConvCore stuck 当前一拍 (ConvCore 已经 dfe_busy=1 stuck)
puts "=== Run ILA with trigger_now (capture current stuck state) ==="
set_property CONTROL.TRIGGER_POSITION 512 $ila
set_property CONTROL.WINDOW_COUNT 1 $ila
set_property CONTROL.DATA_DEPTH 1024 $ila
run_hw_ila -trigger_now $ila
if {[catch {wait_on_hw_ila -timeout 10 $ila} werr]} { puts "  wait err: $werr" }

# Upload data
upload_hw_ila_data $ila
set data [current_hw_ila_data]

# Dump VCD (易解析 + 用户能用)
set vcd C:/_Project/FLUX_CNN/Syn/vd100_bd/output/ila_capture.vcd
write_hw_ila_data -force -vcd_file $vcd $data
puts "  VCD written: $vcd"

set ila_file C:/_Project/FLUX_CNN/Syn/vd100_bd/output/ila_capture.ila
write_hw_ila_data -force $ila_file $data
puts "  ILA  written: $ila_file"

# 关键: 用 read_hw_ila_data + dump 单 bit/多 bit probe 当前 sample 值
puts ""
puts "=== Probe sample values (around trigger) ==="
foreach p [get_hw_probes -of_objects $ila] {
    set pname [get_property NAME $p]
    if {[catch {set vals [get_hw_ila_data -of_objects $data -include_radix_information false]} qe]} {
        # 备用: 用 hw_ila_data 直接 query
        catch { set vals [list_property_value VALUE $p] }
    }
    # 用更直接方法: get_hw_ila_data 取 sample column
    if {[catch {
        set probe_short [string range $pname [expr {[string last "/" $pname] + 1}] end]
    } err]} { set probe_short $pname }
    puts "  $pname"
}

# 改用 export ILA data CSV (Vivado 2023.2 支持)
catch {
    set csv C:/_Project/FLUX_CNN/Syn/vd100_bd/output/ila_capture.csv
    write_hw_ila_data -force -csv_file $csv $data
    puts "  CSV  written: $csv"
}

close_hw_target
disconnect_hw_server
close_hw_manager
exit 0
