# stop A72 + dump 状态: PC, RVBAR, ELR, SPSR
puts "INFO Selecting A72 #0..."
targets -set -filter {name =~ "*A72 #0*"}

puts "INFO stop A72..."
catch {stop} st
puts "  stop: $st"

after 500

puts ""
puts "INFO Registers:"
foreach reg {pc lr cpsr sp_el3 elr_el3 spsr_el3 esr_el3 far_el3 vbar_el3 sctlr_el3} {
    catch {rrd $reg} val
    puts "  $reg = $val"
}

puts ""
puts "INFO ELF link 地址 0x40000000 测 read after stop..."
catch {mrd 0x40000000 4} v
puts "  $v"

puts ""
puts "INFO 0x0 (default reset vector / OCM alias)..."
catch {mrd 0x00000000 4} v
puts "  $v"
