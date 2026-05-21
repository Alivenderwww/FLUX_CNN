# =============================================================================
# arm_ila_and_wait.tcl  v2  --  arm + 等触发 + 直接 dump pc_status binary
# 用法:
#   D:\Xilinx\Vivado\2023.2\bin\vivado.bat -mode batch -source arm_ila_and_wait.tcl \
#       [-tclargs <timeout_min>] -nojournal -nolog
# =============================================================================

set TIMEOUT_MIN 2
if {[llength $argv] >= 1} { set TIMEOUT_MIN [lindex $argv 0] }
puts "INFO timeout = $TIMEOUT_MIN min"

open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target
set dev [lindex [get_hw_devices xcve*] 0]
current_hw_device $dev
refresh_hw_device -update_hw_probes false $dev

set ila [lindex [get_hw_ilas -of $dev -quiet] 0]
puts "INFO ILA: $ila"

set_property CONTROL.TRIGGER_POSITION 512 $ila

# Trigger on pc_asserted == 1
set pc_asserted_probe ""
foreach p [get_hw_probes -of $ila] {
    if {[string match -nocase "*pc_asserted*" $p]} { set pc_asserted_probe $p; break }
}
if {$pc_asserted_probe eq ""} { error "pc_asserted probe not found" }
puts "INFO using probe: $pc_asserted_probe"
set_property TRIGGER_COMPARE_VALUE eq1'b1 $pc_asserted_probe

run_hw_ila $ila
puts ""
puts "INFO ILA armed, waiting up to $TIMEOUT_MIN min for trigger..."

if {[catch {wait_on_hw_ila -timeout $TIMEOUT_MIN $ila} err]} {
    puts "TIMEOUT: $err"
    puts "  pc_asserted 没拉高 → 协议没违规 / 没触发 bug"
    close_hw_target
    disconnect_hw_server
    close_hw_manager
    exit 0
}

set data [upload_hw_ila_data $ila]
puts ""
puts "===== ILA triggered + captured ====="
puts "INFO data obj: $data"

current_hw_ila_data $data

# trigger 在 sample index = 512 (TRIGGER_POSITION). 我们读 trigger 周围多个 sample 看 pc_status.
puts ""
puts "===== pc_status values around trigger (samples 510..520) ====="

# 列出所有 pc_status 相关 probe + dump
set status_probes [list]
foreach p [get_hw_probes -of $ila] {
    if {[string match -nocase "*pc_status*" $p] || [string match -nocase "*pc_asserted*" $p]} {
        lappend status_probes $p
    }
}
puts "INFO [llength $status_probes] probes:"

# 直接 dump 整个 data obj 到文件 (Vivado 默认 wave 格式可读) + 单独读 trigger sample
foreach s {500 505 510 511 512 513 515 520 530} {
    puts "--- sample $s ---"
    foreach p $status_probes {
        # get_hw_probe value at specific sample
        if {[catch {set v [get_hw_probe_value $p -input -radix BIN -sample $s]} cerr]} {
            # 备用: trigger value compare
            puts "  $p -- (sample API err: $cerr)"
        } else {
            puts "  $p = $v"
        }
    }
}

# 也用 write_hw_ila_data 试试输出 wave file (csv 之前不 work, 改 wave 格式)
catch {
    write_hw_ila_data -force C:/_Project/FLUX_CNN/Syn/vd100_minimal/ila_capture.wcfg $data
    puts "INFO wave saved (wcfg)"
}

close_hw_target
disconnect_hw_server
close_hw_manager
exit 0
