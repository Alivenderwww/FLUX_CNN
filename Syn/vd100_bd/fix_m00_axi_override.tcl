# =============================================================================
# fix_m00_axi_override.tcl
# 删除 BD 上 m00_axi_<scalar> pin 的 user override (dummy /Net /Net1.../Net5).
# 这些 override 覆盖了 interface bundle 的隐式连接, 让 14 个 scalar pin
# tie 0 → CRITICAL [BD 41-759] + 综合 LUT3 trim cascade + 板上 ODMA stuck.
#
# 删 dummy net 后, interface bundle u_mc_vd100/m00_axi <-> smartconnect_pl_0/S00_AXI
# 会自动覆盖所有 14 个 scalar pin (跟 m01/m02_axi 一样工作方式).
# =============================================================================
open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

puts "===== Step 1: 列出当前 dummy /Net* 与 m00_axi_<scalar> pin 上的 override ====="
foreach pname {awready wready bvalid arready rlast rvalid awvalid arvalid wvalid wlast bready rready awaddr awlen} {
    set pin [get_bd_pins -quiet u_mc_vd100/m00_axi_${pname}]
    if {$pin ne ""} {
        set net [get_bd_nets -quiet -of_objects $pin]
        if {$net ne ""} {
            puts "  m00_axi_${pname} → net $net (will delete)"
        }
    }
}

puts "===== Step 2: 删 m00_axi_<scalar> pin 上的 net 连接 (override) ====="
# 直接删 net /Net /Net1 .. /Net5 (这些是 dummy)
foreach netname {Net Net1 Net2 Net3 Net4 Net5} {
    set net [get_bd_nets -quiet $netname]
    if {$net ne ""} {
        puts "  deleting net: $net"
        if {[catch {delete_bd_objs $net} e]} {
            puts "    delete failed: $e"
        } else {
            puts "    deleted OK"
        }
    }
}

# 双保险: 找任何接 m00_axi_* scalar pin 的 net, 都删 (不是 interface net)
foreach pname {awready wready bvalid arready rlast rvalid awvalid arvalid wvalid wlast bready rready awaddr awlen} {
    set pin [get_bd_pins -quiet u_mc_vd100/m00_axi_${pname}]
    if {$pin ne ""} {
        set net [get_bd_nets -quiet -of_objects $pin]
        if {$net ne ""} {
            puts "  still has net on m00_axi_${pname}: $net, deleting..."
            catch {delete_bd_objs $net}
        }
    }
}

puts "===== Step 3: validate ====="
catch {validate_bd_design -force} v
puts "validate result done"

puts "===== Step 4: post-fix m00_axi_<scalar> pin net check ====="
foreach pname {awready wready bvalid arready rlast rvalid awvalid arvalid wvalid wlast bready rready awaddr awlen} {
    set pin [get_bd_pins -quiet u_mc_vd100/m00_axi_${pname}]
    if {$pin ne ""} {
        set net [get_bd_nets -quiet -of_objects $pin]
        puts "  m00_axi_${pname} pin=$pin net=$net"
    } else {
        puts "  m00_axi_${pname} NOT FOUND as scalar pin"
    }
}

puts "===== Step 5: 列出剩余 dummy net ====="
foreach n [get_bd_nets -quiet -filter {NAME =~ "Net*"}] {
    puts "  remaining dummy net: $n"
}

save_bd_design
puts "===== DONE ====="
exit 0
