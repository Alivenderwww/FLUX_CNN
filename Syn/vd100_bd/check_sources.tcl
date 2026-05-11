open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
puts "=== ALL sources_1 files ==="
foreach f [get_files -of_objects [get_filesets sources_1]] {
    set match 0
    foreach k {vd100 multicore core_top mac_ cfg_regs dfe odma idma wdma line_buffer wgt_buffer ofb_writer parf_ sequencer rdma sram_model std_rf bias_rf desc_fifo flux_cnn_params sdp axi_lite axi_m_mux axi_arbiter mm2s} {
        if {[string match -nocase "*${k}*" $f]} { set match 1; break }
    }
    if {$match} { puts "  RTL: $f" }
}
exit
