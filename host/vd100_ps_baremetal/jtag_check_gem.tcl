# 看 GEM0 寄存器 + PHY 状态 — 诊断 ping 不通是 link 没起还是 lwIP 没跑
# GEM0 base = 0xFF0C0000 (Versal psv_ethernet_0)

puts "INFO Selecting A72 #0 + stopping..."
targets -set -filter {name =~ "*A72 #0*"}
catch {stop} _

after 200

set GEM0 0xFF0C0000

proc dump_reg {name addr} {
    global GEM0
    set full [expr $GEM0 + $addr]
    if {[catch {mrd $full 1} v]} {
        puts "  $name @ 0x[format %X $full]: FAIL ($v)"
    } else {
        puts "  $name @ 0x[format %X $full]: $v"
    }
}

puts ""
puts "INFO GEM0 registers:"
dump_reg "Net Control"     0x000
dump_reg "Net Config"      0x004
dump_reg "Net Status"      0x008
dump_reg "DMA Config"      0x010
dump_reg "Tx Status"       0x014
dump_reg "Rx Status"       0x020
dump_reg "Int Status"      0x024
dump_reg "PHY Maintenance" 0x034
dump_reg "RX Q ptr"        0x018
dump_reg "TX Q ptr"        0x01C

puts ""
puts "Net Status bit decode:"
puts "  bit 2 (PHY MGMT IDLE) = 1 表示 MDIO 空闲 OK"
puts "  bit 0 (PCS link state) = 1 表示 link UP"

puts ""
puts "INFO Continue A72..."
catch {con} _
