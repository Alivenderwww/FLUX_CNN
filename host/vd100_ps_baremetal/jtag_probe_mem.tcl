# =============================================================================
# jtag_probe_mem.tcl  --  探针: 不同地址 / 不同 master 写测试
#
# 用法: .\run_xsct.ps1 jtag_probe_mem.tcl
# (假设 .pdi 已烧好, 板已 power-cycle)
# =============================================================================

set PDI "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.pdi"

puts "INFO targets list:"
targets

# 1. 烧 PDI (PMC 起来, default subsystem 激活)
puts ""
puts "INFO Programming PDI..."
targets -set -filter {name =~ "*xcve2302*" || name =~ "*Versal*"}
puts "INFO rst -por (POR_B via JTAG TAP, 等价 power-cycle)..."
catch {rst -por}
after 2000
device program $PDI

puts ""
puts "INFO All targets after PDI program:"
targets

# 2. 用 PMC 写 OCM / DDR (PMC 有 root 权限, 都该 work)
proc try_write {label addr val} {
    if {[catch {mwr $addr $val} err]} {
        puts "  $label  mwr $addr 0x[format %X $val]: FAIL ($err)"
        return 0
    }
    if {[catch {mrd $addr 1} val_read]} {
        puts "  $label  mrd $addr: FAIL ($val_read)"
        return 0
    }
    set readback [lindex [split $val_read] end]
    puts "  $label  $addr <- 0x[format %X $val]; readback = $val_read"
    return 1
}

puts ""
puts "INFO ===== PMC target write tests ====="
if {![catch {targets -set -filter {name =~ "*MicroBlaze PMC*" || name =~ "*PMC*"}} terr]} {
    puts "  PMC target selected"
    try_write "PMC->OCM "  0xFFFC0000 0x12345678
    try_write "PMC->XRAM"  0xFE800000 0xCAFEBABE
    try_write "PMC->DDR " 0x40000000 0xDEADBEEF
} else {
    puts "  no PMC target: $terr"
}

puts ""
puts "INFO ===== A72 #0 target write tests ====="
puts "  Selecting A72 #0..."
targets -set -filter {name =~ "*A72 #0*"}
puts "  rst -processor..."
catch {rst -processor}

try_write "A72->OCM " 0xFFFC0000 0xAA55AA55
try_write "A72->XRAM" 0xFE800000 0xBEEFCAFE
try_write "A72->DDR " 0x40000000 0xC0DECAFE

puts ""
puts "============================================================"
puts " Probe done. 看哪些 OK 哪些 FAIL — 区分 firewall vs DAP 问题"
puts "============================================================"
