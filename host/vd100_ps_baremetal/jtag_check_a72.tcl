# 看 A72 是否在跑 / 卡在哪
puts "INFO targets list:"
targets

puts ""
puts "INFO Selecting A72 #0..."
targets -set -filter {name =~ "*A72 #0*"}

puts "INFO A72 state:"
catch {state} st
puts "  state: $st"

puts ""
puts "INFO A72 PC + LR (current execution location):"
catch {rrd pc} pc
puts "  PC: $pc"
catch {rrd lr} lr
puts "  LR: $lr"

puts ""
puts "INFO Sample memory at PC:"
catch {mrd $pc 8} mem
puts "  $mem"

puts ""
puts "INFO Sample DDR 0x40000000 (ELF entry):"
catch {mrd 0x40000000 8} mem
puts "  $mem"

puts ""
puts "INFO Sample DDR 0x40080000 (.bss area, should not all be 0 if A72 wrote there):"
catch {mrd 0x40080000 8} mem
puts "  $mem"
