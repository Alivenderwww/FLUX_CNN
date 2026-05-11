#  jtag_probe_csr.tcl  --  通过 a72 DAP 读 csr_axil 0xA0000004 测 PL csr 端是否响应
puts "INFO: targets:"
targets

# 选 A72 #0 (跟 jtag_run_elf_only.tcl 同样)
catch {targets -set -filter {name =~ "*A72 #0*"}}

puts "INFO: stop A72 to free DAP"
catch {stop}

# mrd 试读 4 byte at 0xA0000004 (CSR core0 STATUS reg)
puts "INFO: mrd 0xA0000004 (core0 STATUS):"
if {[catch {set v [mrd -force 0xA0000004 1]} err]} {
    puts "  FAIL: $err"
} else {
    puts "  status[0] = $v"
}

puts "INFO: mrd 0xA0000000 (core0 CTRL):"
if {[catch {set v [mrd -force 0xA0000000 1]} err]} {
    puts "  FAIL: $err"
} else {
    puts "  ctrl[0]  = $v"
}

puts "INFO: continue A72"
catch {con}
exit
