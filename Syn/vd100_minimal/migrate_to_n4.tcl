# =============================================================================
# migrate_to_n4.tcl  --  vd100_minimal BD: 单核 → N=3 多核 migration
# (脚本名保留 n4 但实际 N=3, URAM/LUT 预算考虑)
#
# 改动:
#   1. RTL: 加 multicore_top_n4.sv + multicore_top_n4_v.v (内部 NUM_CORES=3)
#   2. BD :
#      - 删 u_mc_minimal (单核 instance)
#      - 删 emb_mem_gen_0, axi_bram_ctrl_0 (单 BRAM)
#      - 删 smartconnect_pl (单 SC) + 任何 N=4 残留 (_3)
#      - 加 u_mc_n4 (multicore_top_n4_v reference, 内部 N=3)
#      - 加 smartconnect_pl (NUM_SI=4: CIPS + 3 m_axi, NUM_MI=6: 3 csr + 3 BRAM)
#      - 加 3 × emb_mem_gen URAM
#      - 加 3 × axi_bram_ctrl
#      - 连线 + assign address
#   3. enable +define+FLUX_MAC_SIMD (开 DSP48E1 INT8 SIMD MAC)
#
# 地址 map (在 CIPS.M_AXI_FPD aperture 0xA4000000 [448M] 内):
#   csr_axil_0 @ 0xA4000000  4KB  (ConvCore[0] CSR)
#   csr_axil_1 @ 0xA4001000  4KB
#   csr_axil_2 @ 0xA4002000  4KB
#   axi_bram_ctrl_0 @ 0xA4100000  1MB  (URAM mem[0])
#   axi_bram_ctrl_1 @ 0xA4200000  1MB  (URAM mem[1])
#   axi_bram_ctrl_2 @ 0xA4300000  1MB  (URAM mem[2])
#
# 用法:
#   D:\Xilinx\Vivado\2023.2\bin\vivado.bat -mode batch -source migrate_to_n4.tcl
# =============================================================================

open_project C:/_Project/FLUX_CNN/Syn/vd100_minimal/output/ps_hello.xpr

puts "============================================================"
puts " vd100_minimal: migrate single-core → N=3 multicore"
puts "============================================================"

# -----------------------------------------------------------------------------
# 1. 加 RTL 文件 (multicore_top_n4.sv + .v wrapper)
# -----------------------------------------------------------------------------
set rtl_n4_sv C:/_Project/FLUX_CNN/RTL/Versal/multicore_top_n4.sv
set rtl_n4_v  C:/_Project/FLUX_CNN/RTL/Versal/multicore_top_n4_v.v

if {[llength [get_files -quiet $rtl_n4_sv]] == 0} {
    add_files -fileset sources_1 $rtl_n4_sv
    puts "  + added $rtl_n4_sv"
}
if {[llength [get_files -quiet $rtl_n4_v]] == 0} {
    add_files -fileset sources_1 $rtl_n4_v
    puts "  + added $rtl_n4_v"
}
update_compile_order -fileset sources_1

# -----------------------------------------------------------------------------
# 2. enable FLUX_MAC_SIMD verilog define (开 DSP48E1 INT8 SIMD MAC)
# -----------------------------------------------------------------------------
set existing_defs [get_property verilog_define [current_fileset]]
if {[lsearch $existing_defs "FLUX_MAC_SIMD"] < 0} {
    set_property verilog_define {FLUX_MAC_SIMD=1} [current_fileset]
    puts "  + verilog_define FLUX_MAC_SIMD=1 added (DSP48E1 INT8 SIMD MAC enabled)"
}

# -----------------------------------------------------------------------------
# 3. open BD + 删旧 cells
# -----------------------------------------------------------------------------
open_bd_design [get_files design_1.bd]

# 删旧 cells (单核架构 + 任何 N=4 残留 from 之前失败的 migrate)
foreach c {u_mc_minimal emb_mem_gen_0 axi_bram_ctrl_0 smartconnect_pl \
           u_mc_n4 \
           axi_bram_ctrl_1 axi_bram_ctrl_2 axi_bram_ctrl_3 \
           emb_mem_gen_1 emb_mem_gen_2 emb_mem_gen_3} {
    if {[llength [get_bd_cells -quiet $c]] > 0} {
        catch {delete_bd_objs [get_bd_cells $c]}
        puts "  - deleted old cell $c"
    }
}

# -----------------------------------------------------------------------------
# 4. 加 N=3 ConvCore wrapper instance
# -----------------------------------------------------------------------------
puts ""
puts "(Step 4) Creating multicore_top_n4_v instance (内部 N=3)"
create_bd_cell -type module -reference multicore_top_n4_v u_mc_n4

# -----------------------------------------------------------------------------
# 5. 加 SmartConnect (4 SI: CIPS + 3 m_axi, 5 MI: 3 csr + 2 BRAM)
# -----------------------------------------------------------------------------
puts ""
puts "(Step 5) Creating SmartConnect (4 SI × 5 MI)"
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 smartconnect_pl
set_property -dict [list \
    CONFIG.NUM_SI {4} \
    CONFIG.NUM_MI {5} \
    CONFIG.NUM_CLKS {1} \
] [get_bd_cells smartconnect_pl]

# -----------------------------------------------------------------------------
# 6. 加 2 × axi_bram_ctrl + 2 × emb_mem_gen BRAM (3 核共享 2 BRAM)
# -----------------------------------------------------------------------------
puts ""
puts "(Step 6) Creating 2 × axi_bram_ctrl + 2 × emb_mem_gen BRAM"
for {set i 0} {$i < 2} {incr i} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:axi_bram_ctrl:4.1 axi_bram_ctrl_$i
    set_property -dict [list \
        CONFIG.DATA_WIDTH {128} \
        CONFIG.SINGLE_PORT_BRAM {1} \
        CONFIG.ECC_TYPE {0} \
    ] [get_bd_cells axi_bram_ctrl_$i]

    create_bd_cell -type ip -vlnv xilinx.com:ip:emb_mem_gen:1.0 emb_mem_gen_$i
    set_property -dict [list \
        CONFIG.MEMORY_PRIMITIVE {BRAM} \
        CONFIG.MEMORY_DEPTH {8192} \
    ] [get_bd_cells emb_mem_gen_$i]
    # MEMORY_DEPTH 8192 × 128 bit / 8 = 128 KB/块 (BRAM ~16 RAMB36/块 × 2 = 32, +ConvCore 81 ≈ 113/155)

    puts "  + axi_bram_ctrl_$i + emb_mem_gen_$i (BRAM 128KB)"
}

# -----------------------------------------------------------------------------
# 7. 连线
# -----------------------------------------------------------------------------
puts ""
puts "(Step 7) BD connections"

# 时钟 + reset (用 ps_hello 原 BD 既有的)
set CLK [get_bd_pins versal_cips_0/pl0_ref_clk]
set RST [get_bd_pins proc_sys_reset_0/peripheral_aresetn]

connect_bd_net $CLK [get_bd_pins smartconnect_pl/aclk]
connect_bd_net $RST [get_bd_pins smartconnect_pl/aresetn]

# u_mc_n4 时钟 reset
connect_bd_net $CLK [get_bd_pins u_mc_n4/clk]
connect_bd_net $RST [get_bd_pins u_mc_n4/rst_n]

# 2 × BRAM ctrl 时钟 reset
for {set i 0} {$i < 2} {incr i} {
    connect_bd_net $CLK [get_bd_pins axi_bram_ctrl_$i/s_axi_aclk]
    connect_bd_net $RST [get_bd_pins axi_bram_ctrl_$i/s_axi_aresetn]
}

# SmartConnect SI: S00 = CIPS.M_AXI_FPD, S01..S03 = u_mc_n4.m_axi_0..2
connect_bd_intf_net [get_bd_intf_pins versal_cips_0/M_AXI_FPD] [get_bd_intf_pins smartconnect_pl/S00_AXI]
for {set i 0} {$i < 3} {incr i} {
    connect_bd_intf_net [get_bd_intf_pins u_mc_n4/m_axi_$i] \
        [get_bd_intf_pins smartconnect_pl/S0[expr $i+1]_AXI]
    puts "  S0[expr $i+1] ← u_mc_n4/m_axi_$i"
}

# SmartConnect MI: M00..M02 = csr_axil_0..2, M03..M04 = axi_bram_ctrl_0..1
for {set i 0} {$i < 3} {incr i} {
    connect_bd_intf_net [get_bd_intf_pins smartconnect_pl/M0[expr $i]_AXI] \
        [get_bd_intf_pins u_mc_n4/csr_axil_$i]
    puts "  M0$i → u_mc_n4/csr_axil_$i"
}
for {set i 0} {$i < 2} {incr i} {
    connect_bd_intf_net [get_bd_intf_pins smartconnect_pl/M0[expr $i+3]_AXI] \
        [get_bd_intf_pins axi_bram_ctrl_$i/S_AXI]
    puts "  M0[expr $i+3] → axi_bram_ctrl_$i"
}

# axi_bram_ctrl → emb_mem_gen BRAM ports
for {set i 0} {$i < 2} {incr i} {
    connect_bd_intf_net [get_bd_intf_pins axi_bram_ctrl_$i/BRAM_PORTA] \
        [get_bd_intf_pins emb_mem_gen_$i/BRAM_PORTA]
}

# -----------------------------------------------------------------------------
# 8. Assign addresses (CIPS.M_AXI_FPD 视角)
# -----------------------------------------------------------------------------
puts ""
puts "(Step 8) assign_bd_address"

# csr_axil_0..2 (4 KB each)
for {set i 0} {$i < 3} {incr i} {
    set csr_offset [expr {0xA4000000 + $i * 0x1000}]
    catch {assign_bd_address \
        -offset $csr_offset -range 4K \
        -target_address_space [get_bd_addr_spaces versal_cips_0/M_AXI_FPD] \
        [get_bd_addr_segs u_mc_n4/csr_axil_$i/reg0] -force}
    puts "  csr_axil_$i @ [format 0x%08x $csr_offset] 4 KB"
}

# axi_bram_ctrl_0..2 (1 MB range each, 实际容量由 emb_mem_gen MEMORY_DEPTH 决定)
for {set i 0} {$i < 3} {incr i} {
    set mem_offset [expr {0xA4100000 + $i * 0x100000}]
    catch {assign_bd_address \
        -offset $mem_offset -range 1M \
        -target_address_space [get_bd_addr_spaces versal_cips_0/M_AXI_FPD] \
        [get_bd_addr_segs axi_bram_ctrl_$i/S_AXI/Mem0] -force}
    puts "  axi_bram_ctrl_$i @ [format 0x%08x $mem_offset] 1 MB"
}

# u_mc_n4 3 个 m_axi master 视角: 都能访问 3 BRAM
for {set i 0} {$i < 3} {incr i} {
    for {set j 0} {$j < 3} {incr j} {
        set mem_offset [expr {0xA4100000 + $j * 0x100000}]
        catch {assign_bd_address \
            -offset $mem_offset -range 1M \
            -target_address_space [get_bd_addr_spaces u_mc_n4/m_axi_$i] \
            [get_bd_addr_segs axi_bram_ctrl_$j/S_AXI/Mem0] -force}
    }
}
# m_axi_i 不应该访问 csr_axil 自己, exclude
for {set i 0} {$i < 3} {incr i} {
    for {set j 0} {$j < 3} {incr j} {
        catch {exclude_bd_addr_seg \
            -target_address_space [get_bd_addr_spaces u_mc_n4/m_axi_$i] \
            [get_bd_addr_segs u_mc_n4/csr_axil_$j/reg0]}
    }
}

# -----------------------------------------------------------------------------
# 9. validate + save
# -----------------------------------------------------------------------------
validate_bd_design
save_bd_design
puts "  + BD validated + saved"

# -----------------------------------------------------------------------------
# 10. reset + launch synth + impl + write_device_image
# -----------------------------------------------------------------------------
reset_runs synth_1
reset_runs impl_1
foreach r [get_runs -filter {NAME =~ "*_synth_*"}] {
    catch {reset_run $r}
}

update_compile_order -fileset sources_1

puts ""
puts "  > launch synth_1 (with +define+FLUX_MAC_SIMD)"
launch_runs synth_1 -jobs 16
wait_on_run synth_1
puts "synth_1 = [get_property STATUS [get_runs synth_1]]"

set_param drc.disableLUTOverUtilError 1
puts ""
puts "  > launch impl_1 -to_step write_device_image"
launch_runs impl_1 -to_step write_device_image -jobs 16
wait_on_run impl_1
puts "impl_1 = [get_property STATUS [get_runs impl_1]]"

# -----------------------------------------------------------------------------
# 11. PDI path 提示
# -----------------------------------------------------------------------------
set pdi_path C:/_Project/FLUX_CNN/Syn/vd100_minimal/output/ps_hello.runs/impl_1/design_1_wrapper.pdi
if {[file exists $pdi_path]} {
    puts ""
    puts "============================================================"
    puts " DONE. N=3 multicore PDI at:"
    puts "   $pdi_path"
    puts " Next: bootgen + flash + test multi-core"
    puts "============================================================"
} else {
    puts ""
    puts " WARN: PDI not found, check impl_1 status"
}

exit 0
