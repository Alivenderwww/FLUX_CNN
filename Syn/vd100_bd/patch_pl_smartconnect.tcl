# =============================================================================
# patch_pl_smartconnect.tcl  --  在现有 BD 上加 smartconnect_pl_0/1/2 中间层
#
# 修复: ConvCore m_axi 直接接 axi_noc.S00..S02 时 NoC 不响应 (DFE 永 stuck S_AR).
# 加 3 个 smartconnect (1S→1M each) 做 AXI4 协议规范化 + ID 宽度匹配.
#
# 用法: vivado -mode batch -source patch_pl_smartconnect.tcl
# =============================================================================

set PROJ "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr"
puts "open_project $PROJ"
open_project $PROJ
open_bd_design [get_files design_1.bd]

# 1. Disconnect 现有 u_mc_vd100/m0X_axi <-> axi_noc_0/S0X_AXI 连线
puts "=== Disconnect existing m_axi <-> axi_noc connections ==="
for {set i 0} {$i < 3} {incr i} {
    set net_name [format "u_mc_vd100_m%02d_axi" $i]
    if {[get_bd_intf_nets -quiet $net_name] ne ""} {
        delete_bd_objs [get_bd_intf_nets $net_name]
        puts "  disconnected $net_name"
    }
}

# 2. 创建 3 个 smartconnect
puts "=== Create smartconnect_pl_0/1/2 ==="
for {set i 0} {$i < 3} {incr i} {
    set name "smartconnect_pl_$i"
    set sc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 $name]
    set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1} CONFIG.NUM_CLKS {1}] $sc
    puts "  created $name"
}

# 3. 接 clock + reset (跟现有 smartconnect_0 一样源)
puts "=== Connect clk/reset ==="
for {set i 0} {$i < 3} {incr i} {
    connect_bd_net [get_bd_pins clk_wizard_0/clk_out1] [get_bd_pins smartconnect_pl_${i}/aclk]
    connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] [get_bd_pins smartconnect_pl_${i}/aresetn]
}

# 4. 重接 AXI: ConvCore.m0X_axi → SC_pl_X.S00 → SC_pl_X.M00 → axi_noc.S0X
puts "=== Connect AXI through smartconnect ==="
for {set i 0} {$i < 3} {incr i} {
    connect_bd_intf_net [get_bd_intf_pins u_mc_vd100/m0${i}_axi] \
                        [get_bd_intf_pins smartconnect_pl_${i}/S00_AXI]
    connect_bd_intf_net [get_bd_intf_pins smartconnect_pl_${i}/M00_AXI] \
                        [get_bd_intf_pins axi_noc_0/S0${i}_AXI]
    puts "  ConvCore m0${i}_axi → smartconnect_pl_${i} → axi_noc.S0${i}_AXI"
}

# 5. assign address (smartconnect 引入新 master/slave segment, 必须 reassign).
#    PL master 看到的地址范围保持原样 (0x0-0x80000000 → DDR_LOW0 via NoC).
puts "=== Reassign addresses ==="
for {set i 0} {$i < 3} {incr i} {
    set port [format "S%02d_AXI" $i]
    if {[catch {
        assign_bd_address -offset 0x00000000 -range 0x80000000 \
            -target_address_space [get_bd_addr_spaces u_mc_vd100/$port] \
            [get_bd_addr_segs axi_noc_0/${port}/C0_DDR_LOW0] -force
    } err]} {
        puts "  WARN reassign $port: $err"
    } else {
        puts "  reassigned u_mc_vd100/$port → C0_DDR_LOW0"
    }
}

# 6. Validate
puts "=== Validate BD ==="
if {[catch {validate_bd_design} verr]} {
    puts "  ! BD validate had warnings/errors: $verr"
}
save_bd_design

# 7. Force regen + reset_run + relaunch
puts "=== Regenerate BD output products + reset runs ==="
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
if {![string match "*Complete!*" $status]} {
    puts "  ! impl_1 NOT complete, abort"
    close_project
    exit 1
}

set XSA "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xsa"
write_hw_platform -fixed -include_bit -force $XSA
puts "  written: $XSA"

puts ""
puts "===== PATCH DONE ====="
close_project
exit 0
