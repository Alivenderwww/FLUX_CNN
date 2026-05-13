open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
open_bd_design [get_files design_1.bd]

# v35b: BD 加 axis_ila probe ConvCore[0] m00_axi 关键 handshake signals.
# board layer 0 卡 ODMA S_TX 等 axi_dm s2mm 不接 W data, ILA 抓 stuck 瞬间看
# axi_dm 实际在等什么 (AWVALID/AWREADY/WVALID/WREADY/BVALID 协议级证据).
puts "=== v35: BD 加 axis_ila probe ConvCore[0] m00_axi AW/W/B/AR/R signals ==="

# delete 老 ila if exists
catch {delete_bd_objs [get_bd_cells system_ila_axi]}
catch {delete_bd_objs [get_bd_cells ila_dbg]}

# 创建 axis_ila with 14 probes
set ila [create_bd_cell -type ip -vlnv xilinx.com:ip:axis_ila ila_dbg]
set_property -dict [list \
    CONFIG.C_NUM_OF_PROBES {12} \
    CONFIG.C_DATA_DEPTH {1024} \
    CONFIG.C_INPUT_PIPE_STAGES {1} \
    CONFIG.C_TRIGOUT_EN {false} \
    CONFIG.C_TRIGIN_EN {false} \
    CONFIG.C_EN_STRG_QUAL {1} \
    CONFIG.C_PROBE0_WIDTH {1} \
    CONFIG.C_PROBE1_WIDTH {1} \
    CONFIG.C_PROBE2_WIDTH {1} \
    CONFIG.C_PROBE3_WIDTH {1} \
    CONFIG.C_PROBE4_WIDTH {1} \
    CONFIG.C_PROBE5_WIDTH {1} \
    CONFIG.C_PROBE6_WIDTH {1} \
    CONFIG.C_PROBE7_WIDTH {1} \
    CONFIG.C_PROBE8_WIDTH {1} \
    CONFIG.C_PROBE9_WIDTH {1} \
    CONFIG.C_PROBE10_WIDTH {1} \
    CONFIG.C_PROBE11_WIDTH {1} \
] $ila

# Connect ila clk
connect_bd_net [get_bd_pins clk_wizard_0/clk_out1] [get_bd_pins ila_dbg/clk]

# Connect 14 probes to ConvCore[0] m00_axi signals
# u_mc_vd100/m00_axi_* 是 BD interface 拆出的 sub-pin
set probes [list \
    {probe0  m00_axi_awvalid} \
    {probe1  m00_axi_awready} \
    {probe2  m00_axi_wvalid} \
    {probe3  m00_axi_wready} \
    {probe4  m00_axi_wlast} \
    {probe5  m00_axi_bvalid} \
    {probe6  m00_axi_bready} \
    {probe7  m00_axi_arvalid} \
    {probe8  m00_axi_arready} \
    {probe9  m00_axi_rvalid} \
    {probe10 m00_axi_rready} \
    {probe11 m00_axi_rlast} \
]
foreach pair $probes {
    lassign $pair probe sig
    set src_pin [get_bd_pins -quiet u_mc_vd100/$sig]
    set dst_pin [get_bd_pins -quiet ila_dbg/$probe]
    if {$src_pin eq "" || $dst_pin eq ""} {
        puts "  WARN: $probe ← $sig 找不到 pin (src=$src_pin dst=$dst_pin)"
        continue
    }
    # 连 ila probe 到 现有 net (复用 net 不重连)
    set existing [get_bd_nets -of_objects $src_pin -quiet]
    if {$existing ne ""} {
        connect_bd_net -net $existing $dst_pin
        puts "  $probe ← $sig (reuse net $existing)"
    } else {
        connect_bd_net $src_pin $dst_pin
        puts "  $probe ← $sig (new net)"
    }
}

save_bd_design
generate_target -force all [get_files design_1.bd]

puts "=== reset all runs ==="
reset_runs synth_1
reset_runs impl_1
foreach r [get_runs -filter {NAME =~ "*_synth_*"}] {
    reset_run $r
}

foreach pat {mm2s_arb idma_sg_dispatcher odma_sg_dispatcher axi_arbiter axi_m_mux multicore_top_vd100_bd sequencer cfg_regs core_top} {
    foreach f [get_files -filter "NAME =~ \"*$pat*\""] {
        set p [get_property NAME $f]
        if {[file exists $p]} {
            catch {set fh [open $p a]; close $fh}
        }
    }
}
update_compile_order -fileset sources_1

launch_runs synth_1 -jobs 8
wait_on_run synth_1
puts "synth_1 = [get_property STATUS [get_runs synth_1]]"

set_param drc.disableLUTOverUtilError 1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
puts "impl_1 = [get_property STATUS [get_runs impl_1]]"

open_run impl_1
write_debug_probes -force C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.ltx
write_hw_platform -fixed -include_bit -force C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xsa

puts "===== v35 DONE ====="
exit 0
