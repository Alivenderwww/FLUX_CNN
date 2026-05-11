# =============================================================================
# program_pdi_via_vivado.tcl  --  用 Vivado tcl 烧 PDI (跟 HW Manager GUI 同 backend)
#
# xsct 的 device program 在 PMC 已 boot 态会 hang. Vivado tcl 的 program_hw_devices
# 跟 HW Manager GUI 是同一个 backend, 处理 sideband reset 完整, 重烧不需要 power-cycle.
#
# 用法 (PowerShell, 由 run_xsct.ps1 调):
#   "D:\Xilinx\Vivado\2023.2\bin\vivado.bat" -mode batch \
#       -source program_pdi_via_vivado.tcl \
#       -tclargs <PDI_PATH> -nojournal -nolog
# =============================================================================

set PDI [lindex $argv 0]
if {$PDI eq ""} {
    error "Usage: -tclargs <PDI_PATH>"
}
puts "INFO Program PDI via Vivado HW Manager backend: $PDI"

open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target

# 找 Versal device (跳过 arm_dap_0 这种不可烧的 debug port)
set dev [lindex [get_hw_devices xcve*] 0]
if {$dev eq ""} {
    puts "INFO no xcve* match; trying any non-DAP device:"
    foreach d [get_hw_devices] {
        puts "    candidate: $d"
        if {[regexp {^xc} $d]} { set dev $d; break }
    }
}
if {$dev eq ""} { error "No programmable Versal device found" }
puts "INFO target device: $dev"

current_hw_device $dev

# 烧 PDI (Vivado HW Manager 内部会自动 sideband reset, 不需 power-cycle)
set_property PROGRAM.FILE $PDI $dev
if {[catch {program_hw_devices $dev} prog_err]} {
    puts ""
    puts "============================================================"
    puts " PROGRAM FAILED: $prog_err"
    puts "============================================================"
    puts " PROGRAM.LOG (PLM boot trace):"
    puts "------------------------------------------------------------"
    catch {
        set plog [get_property PROGRAM.LOG $dev]
        puts $plog
    } log_err
    if {$log_err ne ""} { puts "  (cannot fetch PROGRAM.LOG: $log_err)" }
    puts "============================================================"
    close_hw_target
    disconnect_hw_server
    close_hw_manager
    exit 1
}

puts "INFO PDI programmed successfully via Vivado backend"

close_hw_target
disconnect_hw_server
close_hw_manager
exit 0
