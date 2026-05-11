# =============================================================================
# jtag_capture_ila.tcl  --  xsdb 自动加载 ILA + 设 trigger + run + dump waveform
#
# 用法: 由 run_xsct.ps1 调用 (启动 hw_server + connect)
# 触发: probe arvalid_0=1 (任意 ConvCore 0 m_axi read 出现就抓)
# =============================================================================
set LTX C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.ltx

puts "=== Targets ==="
targets

# 选 Versal device (DPC = Debug Packet Controller, 用作 ILA 通信入口)
catch {targets -set -filter {name =~ "*Versal xcve2302*"}}

# 加载 hw_probes (ltx)
puts "=== Loading ILA probes from $LTX ==="
if {[catch {hw_probes -load $LTX} err]} {
    puts "  load fail: $err"
    exit 1
}

puts "=== Available ILAs ==="
foreach ila [hw_ilas] {
    puts "  $ila"
}

# 期望有 1 个 ILA: hw_ila_1 (auto-named by Vivado)
set ila [lindex [hw_ilas] 0]
if {$ila eq ""} {
    puts "  no ILA found"; exit 1
}
puts "Using: $ila"

# 设 trigger: probe10 = dbg_arvalid_0 (probe index 2 in our 12-probe ILA)
# 我们的 probes 顺序: 0=awvalid_0, 1=awready_0, 2=arvalid_0, 3=arready_0, ...
puts "=== List probes ==="
foreach p [hw_probes -of_objects $ila] {
    puts "  $p"
}

# 设 trigger condition: arvalid_0 == 1
# Note: probe name 含 hierarchy
set arvalid_probe [lindex [hw_probes "*arvalid_0*" -of_objects $ila] 0]
if {$arvalid_probe ne ""} {
    puts "=== Set trigger: $arvalid_probe == 1 ==="
    set_property TRIGGER_COMPARE_VALUE eq1'b1 $arvalid_probe
    set_property CONTROL.TRIGGER_POSITION 100 $ila
    set_property CONTROL.WINDOW_COUNT 1 $ila
    set_property CONTROL.DATA_DEPTH 2048 $ila
}

# 启动 ILA armed
puts "=== Run ILA (armed, waiting for trigger) ==="
run_hw_ila $ila

# 给 host 时间触发
puts "  ILA armed, sleeping 2s (host should trigger ConvCore start_dfe now)"
after 2000

# Wait + check status
puts "=== Wait_on_hw_ila ==="
if {[catch {wait_on_hw_ila $ila -timeout 10} werr]} {
    puts "  wait err: $werr"
}

set st [get_property CONTROL.STATUS $ila]
puts "  ILA status: $st"

# 若已 captured, dump waveform
if {[string match "*Idle*" $st]} {
    puts "=== Upload waveform ==="
    upload_hw_ila_data $ila
    set wave_file C:/_Project/FLUX_CNN/Syn/vd100_bd/output/ila_capture.wcfg
    write_hw_ila_data -force C:/_Project/FLUX_CNN/Syn/vd100_bd/output/ila_capture.ila $ila
    puts "  waveform written to ila_capture.ila"
}
exit
