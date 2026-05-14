# =============================================================================
# add_minimal_to_ps_hello.tcl
#   在 ps_hello 基础工程 (VD100 demo, CIPS + NoC + DDR4 + lwIP echo 已 work)
#   上 add ConvCore + BRAM 用于 bring-up.
#
# 输入 BD 状态: design_1 已含 versal_cips_0 + axi_noc_0 + smartconnect_0 + ddr4
#               跑过 lwIP echo (host PING/ECHO PASS).
#
# 添加项 (PL 内, 不动 PS / NoC / DDR4):
#   - 加 RTL: multicore_top_minimal.sv + .v wrapper + 所有 ConvCore 依赖
#   - 加 IP: axi_dm (datamover), smartconnect_pl (PL 内, 不是已存在 smartconnect_0)
#            axi_bram_ctrl, emb_mem_gen
#   - 加 BD cell:
#       u_mc_minimal (multicore_top_minimal_v Verilog wrapper)
#       smartconnect_pl (2 SI → 2 MI, 仲裁 CIPS + ConvCore.m_axi → CSR + BRAM)
#       axi_bram_ctrl_0 + emb_mem_gen_0
#   - 连线: 用现有 clock/reset, 用 CIPS.M_AXI_FPD (lwIP echo server 已用过)
#   - assign address: 0xA4000000 csr_axil, 0xA0000000 BRAM
#
# 用法 (Vivado GUI open ps_hello.xpr 后 Tcl Console):
#   source C:/_Project/FLUX_CNN/Syn/vd100_minimal/add_minimal_to_ps_hello.tcl
# =============================================================================

set RTL_DIR "C:/_Project/FLUX_CNN/RTL"

# =============================================================================
# 1. 加 ConvCore RTL (svh + sv + v)
# =============================================================================
puts "\[1/4\] add ConvCore RTL ..."
if {[get_files -quiet flux_cnn_params.svh] eq ""} {
    add_files -norecurse "$RTL_DIR/flux_cnn_params.svh"
    set_property file_type {Verilog Header} [get_files flux_cnn_params.svh]
    set_property is_global_include true [get_files flux_cnn_params.svh]
}

set rtl_files [list \
    "$RTL_DIR/std_rf.sv" \
    "$RTL_DIR/sram_model.sv" \
    "$RTL_DIR/sdp.sv" \
    "$RTL_DIR/AXI4/axi_lite_csr.sv" \
    "$RTL_DIR/AXI4/axi_m_mux.sv" \
    "$RTL_DIR/AXI4/axi_arbiter.sv" \
    "$RTL_DIR/AXI4/ifb_axi_slave.sv" \
    "$RTL_DIR/DMA/idma_ctrl.sv" \
    "$RTL_DIR/DMA/idma_sg_dispatcher.sv" \
    "$RTL_DIR/DMA/wdma_ctrl.sv" \
    "$RTL_DIR/DMA/odma_ctrl.sv" \
    "$RTL_DIR/DMA/odma_sg_dispatcher.sv" \
    "$RTL_DIR/DMA/rdma_ctrl.sv" \
    "$RTL_DIR/DMA/mm2s_arb.sv" \
    "$RTL_DIR/DMA/dfe.sv" \
    "$RTL_DIR/desc_fifo.sv" \
    "$RTL_DIR/sequencer.sv" \
    "$RTL_DIR/cfg_regs.sv" \
    "$RTL_DIR/mac_pe.sv" \
    "$RTL_DIR/mac_col.sv" \
    "$RTL_DIR/mac_array.sv" \
    "$RTL_DIR/mac_simd_pair.sv" \
    "$RTL_DIR/mac_array_simd.sv" \
    "$RTL_DIR/parf_col.sv" \
    "$RTL_DIR/parf_accum.sv" \
    "$RTL_DIR/line_buffer.sv" \
    "$RTL_DIR/wgt_buffer.sv" \
    "$RTL_DIR/bias_rf.sv" \
    "$RTL_DIR/ofb_writer.sv" \
    "$RTL_DIR/core_top.sv" \
    "$RTL_DIR/Versal/multicore_top_minimal.sv" \
    "$RTL_DIR/Versal/multicore_top_minimal_v.v" \
]
set added 0
foreach f $rtl_files {
    if {[file exists $f] && [get_files -quiet [file tail $f]] eq ""} {
        add_files -norecurse $f
        incr added
    }
}
set_property FILE_TYPE {SystemVerilog} [get_files *.sv]
set_property include_dirs $RTL_DIR [current_fileset]
set_property source_mgmt_mode All [current_project]
update_compile_order -fileset sources_1
puts "  + added $added RTL files"

# =============================================================================
# 2. 创建 axi_dm IP (ConvCore 内部依赖) - 跟 RTL 一起加进 sources_1
# =============================================================================
puts "\[2/4\] create axi_dm IP ..."
if {[llength [get_ips -quiet axi_dm]] == 0} {
    create_ip -name axi_datamover -vendor xilinx.com -library ip -module_name axi_dm
    set_property -dict {
        CONFIG.c_include_mm2s            Full
        CONFIG.c_include_s2mm            Full
        CONFIG.c_addr_width              32
        CONFIG.c_m_axi_mm2s_addr_width   32
        CONFIG.c_m_axi_mm2s_data_width   128
        CONFIG.c_m_axi_mm2s_id_width     2
        CONFIG.c_m_axis_mm2s_tdata_width 128
        CONFIG.c_mm2s_btt_used           23
        CONFIG.c_include_mm2s_dre        true
        CONFIG.c_mm2s_burst_size         256
        CONFIG.c_mm2s_include_sf         false
        CONFIG.c_m_axi_s2mm_addr_width   32
        CONFIG.c_m_axi_s2mm_data_width   128
        CONFIG.c_m_axi_s2mm_id_width     2
        CONFIG.c_s_axis_s2mm_tdata_width 128
        CONFIG.c_s2mm_btt_used           23
        CONFIG.c_include_s2mm_dre        true
        CONFIG.c_s2mm_burst_size         256
        CONFIG.c_s2mm_include_sf         false
    } [get_ips axi_dm]
    generate_target -force {synthesis instantiation_template} [get_ips axi_dm]
    puts "  + axi_dm IP created"
}

# =============================================================================
# 3. open BD design_1, add IPs
# =============================================================================
puts "\[3/4\] add BD cells (smartconnect_pl, axi_bram_ctrl, emb_mem_gen, u_mc_minimal) ..."
open_bd_design [get_files design_1.bd]

# smartconnect_pl (区别于 PS 内部 smartconnect_0): 2 SI → 2 MI
#   SI: CIPS.M_AXI_FPD + u_mc_minimal.m_axi
#   MI: u_mc_minimal.csr_axil + axi_bram_ctrl_0
if {[get_bd_cells -quiet smartconnect_pl] eq ""} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 smartconnect_pl
    set_property -dict [list CONFIG.NUM_SI {2} CONFIG.NUM_MI {2} CONFIG.NUM_CLKS {1}] [get_bd_cells smartconnect_pl]
    puts "  + smartconnect_pl (2 SI → 2 MI)"
}

if {[get_bd_cells -quiet axi_bram_ctrl_0] eq ""} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:axi_bram_ctrl:4.1 axi_bram_ctrl_0
    set_property -dict [list CONFIG.DATA_WIDTH {128} CONFIG.SINGLE_PORT_BRAM {1} CONFIG.ECC_TYPE {0}] [get_bd_cells axi_bram_ctrl_0]
    puts "  + axi_bram_ctrl_0"
}

if {[get_bd_cells -quiet emb_mem_gen_0] eq ""} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:emb_mem_gen:1.0 emb_mem_gen_0
    puts "  + emb_mem_gen_0"
}

if {[get_bd_cells -quiet u_mc_minimal] eq ""} {
    create_bd_cell -type module -reference multicore_top_minimal_v u_mc_minimal
    puts "  + u_mc_minimal (multicore_top_minimal_v)"
}

# =============================================================================
# 4. 连线 (复用 ps_hello 已有 clock/reset)
# =============================================================================
puts "\[4/4\] connect BD nets ..."

# 找现有 100MHz clock 跟 reset (ps_hello demo 默认 clk_wizard 输出 clk_out1, proc_sys_reset peripheral_aresetn)
set CLK [get_bd_pins clk_wizard_0/clk_out1]
set RST [get_bd_pins proc_sys_reset_0/peripheral_aresetn]

if {$CLK eq ""} { error "Cannot find clk_wizard_0/clk_out1, BD 不是 ps_hello 标准 layout" }

# 连 clock 给新加 cell
connect_bd_net $CLK [get_bd_pins smartconnect_pl/aclk]
connect_bd_net $CLK [get_bd_pins axi_bram_ctrl_0/s_axi_aclk]
connect_bd_net $CLK [get_bd_pins u_mc_minimal/clk]

# 连 reset
connect_bd_net $RST [get_bd_pins smartconnect_pl/aresetn]
connect_bd_net $RST [get_bd_pins axi_bram_ctrl_0/s_axi_aresetn]
connect_bd_net $RST [get_bd_pins u_mc_minimal/rst_n]

# AXI 连接:
# CIPS.M_AXI_FPD → smartconnect_pl.S00_AXI
#   但 ps_hello 的 M_AXI_FPD 可能已经接到别处 (lwIP demo 不用 M_AXI_FPD 默认?).
#   先 disconnect 原连线再连我们的.
set existing_intf_net [get_bd_intf_nets -of_objects [get_bd_intf_pins versal_cips_0/M_AXI_FPD]]
if {$existing_intf_net ne ""} {
    puts "  ! M_AXI_FPD 已接到 $existing_intf_net, disconnect 后重连 smartconnect_pl"
    delete_bd_objs $existing_intf_net
}
connect_bd_intf_net [get_bd_intf_pins versal_cips_0/M_AXI_FPD] [get_bd_intf_pins smartconnect_pl/S00_AXI]

# u_mc_minimal.m_axi → smartconnect_pl.S01_AXI
connect_bd_intf_net [get_bd_intf_pins u_mc_minimal/m_axi] [get_bd_intf_pins smartconnect_pl/S01_AXI]

# smartconnect_pl.M00_AXI → u_mc_minimal.csr_axil
connect_bd_intf_net [get_bd_intf_pins smartconnect_pl/M00_AXI] [get_bd_intf_pins u_mc_minimal/csr_axil]

# smartconnect_pl.M01_AXI → axi_bram_ctrl_0.S_AXI
connect_bd_intf_net [get_bd_intf_pins smartconnect_pl/M01_AXI] [get_bd_intf_pins axi_bram_ctrl_0/S_AXI]

# axi_bram_ctrl_0.BRAM_PORTA → emb_mem_gen_0.BRAM_PORTA
connect_bd_intf_net [get_bd_intf_pins axi_bram_ctrl_0/BRAM_PORTA] [get_bd_intf_pins emb_mem_gen_0/BRAM_PORTA]

# CIPS m_axi_fpd_aclk: 接 100 MHz clock
catch {connect_bd_net [get_bd_pins versal_cips_0/m_axi_fpd_aclk] $CLK}

# =============================================================================
# 5. assign address (跟 vd100_rpc_server.c CSR_BASE 一致)
# =============================================================================
puts "\[5/5\] assign addresses ..."

# CIPS.M_AXI_FPD 视图:
#   - u_mc_minimal/csr_axil/Reg @ 0xA4000000 size 4K
#   - axi_bram_ctrl_0/S_AXI/Mem0 @ 0xA0000000 size 1M
catch {assign_bd_address -offset 0xA4000000 -range 4K \
    -target_address_space [get_bd_addr_spaces versal_cips_0/M_AXI_FPD] \
    [get_bd_addr_segs u_mc_minimal/csr_axil/Reg] -force}
catch {assign_bd_address -offset 0xA0000000 -range 1M \
    -target_address_space [get_bd_addr_spaces versal_cips_0/M_AXI_FPD] \
    [get_bd_addr_segs axi_bram_ctrl_0/S_AXI/Mem0] -force}

# u_mc_minimal.m_axi 视图 (PL 内 ConvCore 看 BRAM):
catch {assign_bd_address -offset 0xA0000000 -range 1M \
    -target_address_space [get_bd_addr_spaces u_mc_minimal/m_axi] \
    [get_bd_addr_segs axi_bram_ctrl_0/S_AXI/Mem0] -force}

puts ""
puts "============================================================"
puts " minimal IP added to ps_hello BD. Next:"
puts "   1. regenerate_bd_layout (整理 canvas)"
puts "   2. validate_bd_design"
puts "   3. save_bd_design"
puts "   4. make_wrapper -files [get_files design_1.bd] -top -import -force"
puts "   5. Run synth + impl + write_device_image"
puts "============================================================"
