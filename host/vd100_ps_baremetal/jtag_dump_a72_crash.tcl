# 假设 a72 已 hang, stop a72 + dump PC/LR/ESR/FAR 看 crash 状态
catch {targets -set -filter {name =~ "*A72 #0*"}}
catch {stop}
puts ""
puts "==== A72 #0 crash dump ===="
catch {puts "rd pc:  [rrd pc]"}
catch {puts "rd elr_el3: [rrd elr_el3]"}
catch {puts "rd esr_el3: [rrd esr_el3]"}
catch {puts "rd far_el3: [rrd far_el3]"}
catch {puts "rd elr_el1: [rrd elr_el1]"}
catch {puts "rd esr_el1: [rrd esr_el1]"}
catch {puts "rd far_el1: [rrd far_el1]"}
catch {puts "rd lr:   [rrd lr]"}
catch {puts "rd sp:   [rrd sp]"}
puts ""
# read csr_axil (0xA4000004) directly via DAP
puts "==== mrd 0xA4000004 (csr STATUS via DAP) ===="
if {[catch {set v [mrd -force 0xA4000004 1]} err]} {
    puts "  FAIL: $err"
} else {
    puts "  $v"
}
puts ""
# don't con — leave a72 stopped for inspection
exit
