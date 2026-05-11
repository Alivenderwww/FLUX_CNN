# =============================================================================
# patch_pl_aclk_assoc.tcl  --  修 axi_noc PL master ASSOCIATED_BUSIF
#
# 现在 aclk0 关联 S00_AXI:S01_AXI:S02_AXI 三个, aclk1/aclk2 空. axi_noc 内部 NoC
# 路由表识别 S01/S02 的 clock domain 失败, 导致 ConvCore 1/2 的 arready 不响应.
# 修法: 让 aclk0/1/2 各自关联 S00/S01/S02_AXI, 互不重叠.
# =============================================================================

open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

puts "=== Before fix: aclk0..2 ASSOCIATED_BUSIF ==="
for {set i 0} {$i < 3} {incr i} {
    set p [get_bd_pins /axi_noc_0/aclk$i]
    puts "  aclk$i: [get_property -quiet CONFIG.ASSOCIATED_BUSIF $p]"
}

puts ""
puts "=== Apply: each aclk_i associated with S0i_AXI only ==="
for {set i 0} {$i < 3} {incr i} {
    set port [format "S%02d_AXI" $i]
    set p [get_bd_pins /axi_noc_0/aclk$i]
    set_property CONFIG.ASSOCIATED_BUSIF $port $p
    puts "  aclk$i ↔ $port"
}

# 同时确保 aclk1/aclk2 真的接到 clk_wizard_0/clk_out1 (aclk0 已接)
# 看看现在 aclk1/aclk2 接什么
puts ""
puts "=== Check aclk1/aclk2 net source ==="
for {set i 1} {$i < 3} {incr i} {
    set net [get_bd_nets -of_objects [get_bd_pins /axi_noc_0/aclk$i]]
    puts "  aclk$i net: $net"
}

# 如果没接 (空 net), 接 clk_wizard_0/clk_out1
for {set i 1} {$i < 3} {incr i} {
    set p [get_bd_pins /axi_noc_0/aclk$i]
    set nets [get_bd_nets -quiet -of_objects $p]
    if {[llength $nets] == 0} {
        connect_bd_net [get_bd_pins clk_wizard_0/clk_out1] $p
        puts "  connected aclk$i ← clk_wizard_0/clk_out1"
    }
}

puts ""
puts "=== After fix: aclk0..2 ASSOCIATED_BUSIF ==="
for {set i 0} {$i < 3} {incr i} {
    set p [get_bd_pins /axi_noc_0/aclk$i]
    puts "  aclk$i: [get_property -quiet CONFIG.ASSOCIATED_BUSIF $p]"
}

if {[catch {validate_bd_design} verr]} {
    puts "  ! BD validate had warnings/errors: $verr"
}
save_bd_design

# Regen + reset_run + relaunch
generate_target all [get_files design_1.bd]
reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1
set status [get_property STATUS [get_runs synth_1]]
puts "  synth_1 status = $status"
if {$status ne "synth_design Complete!"} {
    puts "  ! synth_1 NOT complete, abort"
    close_project
    exit 1
}

launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
set status [get_property STATUS [get_runs impl_1]]
puts "  impl_1 status = $status"

set XSA "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xsa"
write_hw_platform -fixed -include_bit -force $XSA
puts "  written: $XSA"

puts ""
puts "===== ACLK PATCH DONE ====="
close_project
exit 0
