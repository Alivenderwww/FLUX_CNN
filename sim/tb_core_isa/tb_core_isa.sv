`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// tb_core_isa.sv  --  Testbench for configuration-register driven CNN core
// -----------------------------------------------------------------------------
// Changes vs ISA-era TB:
//   • No inst.txt loading (INST_SRAM removed)
//   • Reads config.txt (key=value) and pokes u_ctrl.cfg_* via hierarchical refs
//   • Everything else (IFB/WB loading, OFB verification, perf counters) kept
// -----------------------------------------------------------------------------

module tb_core_isa;

    localparam NUM_COL    = 16;
    localparam NUM_PE     = 16;
    localparam DATA_WIDTH = 8;
    localparam PSUM_WIDTH = 32;
    parameter  SRAM_DEPTH = 8192;

    localparam IFB_WIDTH = NUM_PE * DATA_WIDTH;
    localparam WB_WIDTH  = NUM_COL * NUM_PE * DATA_WIDTH;
    localparam OFB_WIDTH = NUM_COL * DATA_WIDTH;

    logic clk;
    logic rst_n;
    logic start;
    logic done;
    logic signed [NUM_COL*PSUM_WIDTH-1:0] psum_out_vec;
    logic [OFB_WIDTH-1:0] ofb_rdata_ext;

    core_top #(
        .NUM_COL(NUM_COL),
        .NUM_PE(NUM_PE),
        .DATA_WIDTH(DATA_WIDTH),
        .PSUM_WIDTH(PSUM_WIDTH),
        .WRF_DEPTH(32),
        .ARF_DEPTH(32),
        .PARF_DEPTH(32),
        .SRAM_DEPTH(SRAM_DEPTH)
    ) u_core_top (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .ifb_we_ext(1'b0),
        .ifb_waddr_ext('0),
        .ifb_wdata_ext('0),
        .wb_we_ext(1'b0),
        .wb_waddr_ext('0),
        .wb_wdata_ext('0),
        .ofb_re_ext(1'b0),
        .ofb_raddr_ext('0),
        .ofb_rdata_ext(ofb_rdata_ext),
        .psum_out_vec(psum_out_vec),
        .done(done)
    );

    // Clock
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Load SRAM contents
    initial begin
        $readmemh("ifb.txt", u_core_top.u_ifb.mem);
        $readmemh("wb.txt",  u_core_top.u_wb.mem);
        for (int i = 0; i < SRAM_DEPTH; i++) u_core_top.u_ofb.mem[i] = '0;
    end

    // Expected OFM
    logic [OFB_WIDTH-1:0] expected_ofm_arr [0:SRAM_DEPTH-1];

    // Cycle counter
    longint total_cycles = 0;
    logic   is_running   = 0;
    always_ff @(posedge clk) begin
        if (rst_n) begin
            if (start) is_running <= 1;
            if (done)  is_running <= 0;
            if (is_running) total_cycles++;
        end else begin
            total_cycles <= 0;
            is_running   <= 0;
        end
    end

    // Perf arrays (mac_array 顶层 counter 负责 real MAC ops; 这里只保留 WRF 写次数统计)
    int pe_wrf_write_arr [0:NUM_COL-1][0:NUM_PE-1];

    genvar gc, gp;
    generate
        for (gc = 0; gc < NUM_COL; gc++) begin : perf_col
            for (gp = 0; gp < NUM_PE; gp++) begin : perf_pe
                assign pe_wrf_write_arr[gc][gp]  = u_core_top.u_mac_array.gen_col[gc].u_col.gen_pe[gp].u_pe.u_wrf.total_write_ops;
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Config loader: parse config.txt and poke u_ctrl.cfg_* registers
    // -------------------------------------------------------------------------
    task automatic load_config();
        int      fd;
        string   line;
        string   key;
        int      val;
        int      count;
        fd = $fopen("config.txt", "r");
        if (fd == 0) begin
            $display("FATAL: cannot open config.txt");
            $stop;
        end
        while (!$feof(fd)) begin
            void'($fgets(line, fd));
            if (line.len() == 0) continue;
            // Accept "KEY = VALUE" (possibly with leading whitespace)
            count = $sscanf(line, "%s = %d", key, val);
            if (count < 2) continue;
            case (key)
                "H_OUT"        : u_core_top.u_cfg.r_h_out        = val[15:0];
                "W_OUT"        : u_core_top.u_cfg.r_w_out        = val[15:0];
                "W_IN"         : u_core_top.u_cfg.r_w_in         = val[15:0];
                "K"            : u_core_top.u_cfg.r_k            = val[3:0];
                "STRIDE"       : u_core_top.u_cfg.r_stride       = val[2:0];
                "CIN_SLICES"   : u_core_top.u_cfg.r_cin_slices   = val[5:0];
                "COUT_SLICES"  : u_core_top.u_cfg.r_cout_slices  = val[5:0];
                "TILE_W"       : u_core_top.u_cfg.r_tile_w       = val[5:0];
                "TOTAL_WRF"      : u_core_top.u_cfg.r_total_wrf      = val[9:0];
                "WRF_PACKED"     : u_core_top.u_cfg.r_wrf_packed     = val[0];
                "KK"             : u_core_top.u_cfg.r_kk             = val[9:0];
                "ROUNDS_PER_CINS": u_core_top.u_cfg.r_rounds_per_cins= val[2:0];
                "ROUND_LEN_LAST" : u_core_top.u_cfg.r_round_len_last = val[5:0];
                "IFB_BASE"     : u_core_top.u_cfg.r_ifb_base     = val[19:0];
                "WB_BASE"      : u_core_top.u_cfg.r_wb_base      = val[19:0];
                "OFB_BASE"     : u_core_top.u_cfg.r_ofb_base     = val[19:0];
                "IFB_CIN_STEP" : u_core_top.u_cfg.r_ifb_cin_step = val[19:0];
                "IFB_ROW_STEP" : u_core_top.u_cfg.r_ifb_row_step = val[19:0];
                "WB_CIN_STEP"  : u_core_top.u_cfg.r_wb_cin_step  = val[19:0];
                "WB_COUT_STEP" : u_core_top.u_cfg.r_wb_cout_step = val[19:0];
                "OFB_COUT_STEP": u_core_top.u_cfg.r_ofb_cout_step= val[19:0];
                "NUM_TILES"    : u_core_top.u_cfg.r_num_tiles    = val[7:0];
                "LAST_VALID_W" : u_core_top.u_cfg.r_last_valid_w = val[5:0];
                "TILE_IN_STEP" : u_core_top.u_cfg.r_tile_in_step = val[19:0];
                "SDP_SHIFT"    : u_core_top.u_cfg.r_sdp_shift    = val[4:0];
                "SDP_RELU_EN"  : u_core_top.u_cfg.r_sdp_relu_en  = val[0];
                default        : $display("WARN: unknown config key '%s'", key);
            endcase
        end
        $fclose(fd);
        $display("Config loaded from config.txt");
    endtask

    // -------------------------------------------------------------------------
    // Watchdog
    // -------------------------------------------------------------------------
    initial begin
        #50000000;
        $display("========================================");
        $display("FATAL: simulation timeout (50ms)");
        $display("========================================");
        $stop;
    end

    // -------------------------------------------------------------------------
    // Main sequence
    // -------------------------------------------------------------------------
    initial begin
        rst_n = 0;
        start = 0;
        #20;
        rst_n = 1;
        #10;

        // Load expected OFM
        $readmemh("expected_ofm.txt", expected_ofm_arr);

        // Poke configuration registers
        load_config();
        #10;

        $display("========================================");
        $display("Starting config-driven CNN Core...");
        $display("========================================");

        start = 1;
        #10;
        start = 0;

        wait (done == 1);
        #50;

        $display("========================================");
        $display("Core execution finished. Verifying OFB...");

        begin
            automatic int mismatch_cnt = 0;
            int h_out_val, w_out_val, cout_slices_val, total_ofm;
            if (!$value$plusargs("H_OUT=%d",        h_out_val))       h_out_val = 66;
            if (!$value$plusargs("W_OUT=%d",        w_out_val))       w_out_val = 118;
            if (!$value$plusargs("COUT_SLICES=%d",  cout_slices_val)) cout_slices_val = 1;
            total_ofm = h_out_val * w_out_val * cout_slices_val;
            $display("OFM dims: H_OUT=%0d W_OUT=%0d COUT_SLICES=%0d TOTAL=%0d",
                     h_out_val, w_out_val, cout_slices_val, total_ofm);

            for (int addr = 0; addr < total_ofm; addr++) begin
                automatic logic [OFB_WIDTH-1:0] actual   = u_core_top.u_ofb.mem[addr];
                automatic logic [OFB_WIDTH-1:0] expected = expected_ofm_arr[addr];
                if (actual !== expected) begin
                    if (mismatch_cnt < 10)
                        $display("FAIL: OFB[%0d] expected=%0h got=%0h", addr, expected, actual);
                    if (mismatch_cnt == 10)
                        $display("... (further mismatches suppressed)");
                    mismatch_cnt++;
                end
            end

            if (mismatch_cnt == 0)
                $display("Check Complete! 0 mismatches found. Simulation PASSED.");
            else
                $display("Check Complete! Found %0d mismatches. Simulation FAILED.", mismatch_cnt);

            $display("========================================");
            $display(" PERFORMANCE & RESOURCE UTILIZATION STATS");
            $display("========================================");
            $display("Total Active Cycles:  %0d", total_cycles);
            $display("");
            $display("--- SRAM Traffic ---");
            $display("IFB: Reads=%0d  Writes=%0d", u_core_top.u_ifb.sram_read_cnt, u_core_top.u_ifb.sram_write_cnt);
            $display("WB : Reads=%0d  Writes=%0d", u_core_top.u_wb.sram_read_cnt,  u_core_top.u_wb.sram_write_cnt);
            $display("OFB: Reads=%0d  Writes=%0d", u_core_top.u_ofb.sram_read_cnt, u_core_top.u_ofb.sram_write_cnt);
            $display("");

            begin
                longint mac_fire_cycles;
                longint total_mac_ops;
                longint total_wrf_writes;
                mac_fire_cycles = u_core_top.u_mac_array.mac_fire_cnt;
                total_mac_ops   = mac_fire_cycles * NUM_COL * NUM_PE;
                total_wrf_writes = 0;
                for (int c = 0; c < NUM_COL; c++) begin
                    for (int p = 0; p < NUM_PE; p++) begin
                        total_wrf_writes += pe_wrf_write_arr[c][p];
                    end
                end
                $display("--- PE Array Utilization ---");
                $display("MAC Fire Cycles:      %0d", mac_fire_cycles);
                $display("Total MAC Ops:        %0d (= real_cycles * %0d PEs)", total_mac_ops, NUM_COL * NUM_PE);
                $display("Theoretical Max MACs: %0d (= cycles * %0d PEs)",
                         total_cycles * NUM_COL * NUM_PE, NUM_COL * NUM_PE);
                $display("MAC Utilization:      %.2f %%",
                         (real'(total_mac_ops) / (real'(total_cycles) * real'(NUM_COL * NUM_PE))) * 100.0);
                $display("");
                $display("--- RF Traffic ---");
                $display("WRF  Writes: %0d", total_wrf_writes);
                $display("");
                $display("--- Handshake Stats (fire / stall=V&!R / idle=!V&R) ---");
                $display("ACT  (lb  -> mac): fire=%0d stall=%0d idle=%0d",
                         u_core_top.u_mac_array.hs_act_fire,
                         u_core_top.u_mac_array.hs_act_stall,
                         u_core_top.u_mac_array.hs_act_idle);
                $display("WGT  (wb  -> mac): fire=%0d stall=%0d idle=%0d",
                         u_core_top.u_mac_array.hs_wgt_fire,
                         u_core_top.u_mac_array.hs_wgt_stall,
                         u_core_top.u_mac_array.hs_wgt_idle);
                $display("PSUM (mac -> prf): fire=%0d stall=%0d idle=%0d",
                         u_core_top.u_mac_array.hs_psum_fire,
                         u_core_top.u_mac_array.hs_psum_stall,
                         u_core_top.u_mac_array.hs_psum_idle);
                $display("ACC  (prf -> ofb): fire=%0d stall=%0d idle=%0d",
                         u_core_top.u_ofb_writer.hs_acc_fire,
                         u_core_top.u_ofb_writer.hs_acc_stall,
                         u_core_top.u_ofb_writer.hs_acc_idle);
            end
        end
        $display("========================================");
        $stop;
    end

endmodule
