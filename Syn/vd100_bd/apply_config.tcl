# =============================================================================
# apply_config.tcl  --  在已有 .xpr 上 reapply PS_PMC_CONFIG + axi_noc PS NMU
#                       然后调 full_flow.tcl 跑综合 + impl + xsa.
#
# 替代之前的 fix_gem / fix_uart / fix_full / fix_xdc 一堆 patch 脚本.
#
# 用法:
#   "D:\Xilinx\Vivado\2023.2\bin\vivado.bat" -mode batch \
#       -source Syn\vd100_bd\apply_config.tcl \
#       -nojournal -log Syn\vd100_bd\apply_config.log
#
# 改动来源:
#   ps_config.tcl  : 完整 PS_PMC_CONFIG (FPD/CCI/IPI/TTC/UART/GEM, ALINX 06_ps_hello 对齐)
#   pl_config.tcl  : axi_noc_0 NUM_SI 3→9, 加 6 个 PS NMU (FPD_CCI×4 + LPD + PMC)
# =============================================================================

set PROJ "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr"
set BD_DIR "C:/_Project/FLUX_CNN/Syn/vd100_bd"

# 仅当 project 未 open 时才 open (rebuild_all.tcl 内 source create_project 后已 open)
if {[catch {current_project} cur] || $cur eq ""} {
    open_project $PROJ
}
if {[catch {current_bd_design} cur_bd] || $cur_bd eq ""} {
    open_bd_design [get_files design_1.bd]
}

# -----------------------------------------------------------------------------
# 1. Apply PS_PMC_CONFIG (从 ps_config.tcl 取 set_ps_config proc)
# -----------------------------------------------------------------------------
puts ""
puts "=== Apply PS_PMC_CONFIG (full ALINX-aligned) ==="
source $BD_DIR/ps_config.tcl
set_ps_config versal_cips_0
puts "  PS config applied"

# -----------------------------------------------------------------------------
# 2. Patch axi_noc_0: 加 6 个 PS NMU port (S03..S08) + 连 versal_cips + assign addr
# -----------------------------------------------------------------------------
puts ""
puts "=== Patch axi_noc_0 to add 6 PS NMU ports ==="

set_property -dict [list \
    CONFIG.CONTROLLERTYPE {DDR4_SDRAM} \
    CONFIG.MC_CHAN_REGION1 {NONE} \
    CONFIG.NUM_SI {9} \
    CONFIG.NUM_MCP {4} \
    CONFIG.NUM_CLKS {9} \
] [get_bd_cells axi_noc_0]

# NUM_CLKS 扩展后, aclk1/aclk2 (原 PL master 时钟) 连接可能丢. 重新接 clk_wizard_0/clk_out1
foreach idx {0 1 2} {
    if {[get_bd_nets -quiet -of_objects [get_bd_pins axi_noc_0/aclk$idx]] eq ""} {
        connect_bd_net [get_bd_pins clk_wizard_0/clk_out1] [get_bd_pins axi_noc_0/aclk$idx]
        puts "  + aclk$idx ← clk_wizard_0/clk_out1 (re-connected)"
    }
}

# 6 个 PS NMU: 4 FPD_CCI + 1 LPD + 1 PMC, 分摊 4 个 MC channel
set ps_nmu_specs {
    {S03 ps_cci MC_3 FPD_CCI_NOC_0 fpd_cci_noc_axi0_clk C3_DDR_LOW0}
    {S04 ps_cci MC_2 FPD_CCI_NOC_1 fpd_cci_noc_axi1_clk C2_DDR_LOW0}
    {S05 ps_cci MC_0 FPD_CCI_NOC_2 fpd_cci_noc_axi2_clk C0_DDR_LOW0}
    {S06 ps_cci MC_1 FPD_CCI_NOC_3 fpd_cci_noc_axi3_clk C1_DDR_LOW0}
    {S07 ps_rpu MC_3 LPD_AXI_NOC_0 lpd_axi_noc_clk      C3_DDR_LOW0}
    {S08 ps_pmc MC_2 PMC_NOC_AXI_0 pmc_axi_noc_axi0_clk C2_DDR_LOW0}
}
set i 3
foreach spec $ps_nmu_specs {
    lassign $spec port category mc cips_intf cips_clk addr_chan
    set port_full ${port}_AXI
    set conn_str [format "%s {read_bw {100} write_bw {100} read_avg_burst {4} write_avg_burst {4}}" $mc]

    # 配 NMU port 属性
    set_property -dict [list \
        CONFIG.REGION {0} \
        CONFIG.CONNECTIONS $conn_str \
        CONFIG.NOC_PARAMS {} \
        CONFIG.CATEGORY $category \
    ] [get_bd_intf_pins /axi_noc_0/$port_full]
    set_property CONFIG.ASSOCIATED_BUSIF $port_full [get_bd_pins /axi_noc_0/aclk$i]

    # 连 versal_cips_0 → axi_noc_0
    if {[get_bd_intf_nets -quiet -of_objects [get_bd_intf_pins versal_cips_0/$cips_intf]] eq ""} {
        connect_bd_intf_net [get_bd_intf_pins versal_cips_0/$cips_intf] \
                            [get_bd_intf_pins axi_noc_0/$port_full]
    }
    if {[get_bd_nets -quiet -of_objects [get_bd_pins versal_cips_0/$cips_clk]] eq ""} {
        connect_bd_net [get_bd_pins versal_cips_0/$cips_clk] \
                       [get_bd_pins axi_noc_0/aclk$i]
    }

    # assign DDR4 low 2 GB 给该 NMU
    assign_bd_address -offset 0x00000000 -range 0x80000000 \
        -target_address_space [get_bd_addr_spaces versal_cips_0/$cips_intf] \
        [get_bd_addr_segs axi_noc_0/${port_full}/$addr_chan] -force

    puts "  + $port_full ($category) ← versal_cips_0/$cips_intf, addr $addr_chan assigned"
    incr i
}

# -----------------------------------------------------------------------------
# 3. Save BD + regenerate output products
# -----------------------------------------------------------------------------
puts ""
puts "=== Save BD + generate output products ==="
save_bd_design
generate_target -force all [get_files design_1.bd]

if {[catch {validate_bd_design} verr]} {
    puts "  ! BD validate: $verr"
}

# Retry csr_axil address assign (validate_bd_design 后 register segment 应该出现)
if {[get_bd_addr_segs -of_objects [get_bd_addr_spaces versal_cips_0/M_AXI_FPD] -filter "NAME =~ *u_mc_vd100*"] eq ""} {
    puts "=== Retry assign csr_axil address (post validate) ==="
    set csr_seg ""
    foreach segname {Reg reg0} {
        set s [get_bd_addr_segs -quiet u_mc_vd100/csr_axil/$segname]
        if {$s ne ""} { set csr_seg $s; break }
    }
    if {$csr_seg ne ""} {
        if {[catch {
            assign_bd_address -offset 0xA4000000 -range 0x4000 \
               -target_address_space [get_bd_addr_spaces versal_cips_0/M_AXI_FPD] \
               $csr_seg -force
            puts "  csr_axil seg $csr_seg → 0xA4000000 16KB OK"
        } err2]} {
            puts "  WARN retry fail: $err2"
        }
    } else {
        puts "  WARN: csr_axil seg still missing (Reg/reg0 both not found)"
    }
}
save_bd_design

puts ""
puts "=== Regenerate BD wrapper ==="
make_wrapper -files [get_files design_1.bd] -top -import -force
update_compile_order -fileset sources_1
set_property top design_1_wrapper [current_fileset]
set_property top_auto_set false [current_fileset]

# -----------------------------------------------------------------------------
# 4. Reset runs (PS config / NoC topology 改了, 必须全综合)
# -----------------------------------------------------------------------------
puts ""
puts "=== Reset synth_1 + impl_1 ==="
reset_run synth_1
reset_run impl_1

# -----------------------------------------------------------------------------
# 5. 调 full_flow.tcl 跑 synth + impl + bitstream + xsa
# -----------------------------------------------------------------------------
puts ""
puts "=== Hand off to full_flow.tcl ==="
close_bd_design [get_bd_designs design_1]
close_project

source $BD_DIR/full_flow.tcl
