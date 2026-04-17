`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// tb_core_isa.sv  --  Testbench for AXI-Lite-driven CNN core
// -----------------------------------------------------------------------------
// 流程：
//   1) $readmemh 把 ifb.txt / wb.txt 灌到 u_ifb.mem / u_wb.mem (hier 直写)
//   2) 解析 config.txt，通过 AXI-Lite S 写到 cfg_regs 对应地址
//   3) 写 CTRL=0x1 触发 start_pulse，核内 feeder 启动
//   4) wait (done)，hier 读 u_ofb.mem 做 OFB 校验
// -----------------------------------------------------------------------------

module tb_core_isa;

    localparam NUM_COL    = 16;
    localparam NUM_PE     = 16;
    localparam DATA_WIDTH = 8;
    localparam PSUM_WIDTH = 32;
    parameter  SRAM_DEPTH = 8192;
    localparam CSR_ADDR_W = 12;
    localparam CSR_DATA_W = 32;

    localparam IFB_WIDTH = NUM_PE * DATA_WIDTH;
    localparam WB_WIDTH  = NUM_COL * NUM_PE * DATA_WIDTH;
    localparam OFB_WIDTH = NUM_COL * DATA_WIDTH;

    // -------------------------------------------------------------------------
    // cfg_regs 地址常量（与 RTL/cfg_regs.sv 内部 localparam 一致）
    // -------------------------------------------------------------------------
    localparam [11:0] ADDR_CTRL             = 12'h000;
    localparam [11:0] ADDR_H_OUT            = 12'h100;
    localparam [11:0] ADDR_W_OUT            = 12'h104;
    localparam [11:0] ADDR_W_IN             = 12'h108;
    localparam [11:0] ADDR_K                = 12'h10C;
    localparam [11:0] ADDR_STRIDE           = 12'h110;
    localparam [11:0] ADDR_CIN_SLICES       = 12'h114;
    localparam [11:0] ADDR_COUT_SLICES      = 12'h118;
    localparam [11:0] ADDR_TILE_W           = 12'h11C;
    localparam [11:0] ADDR_NUM_TILES        = 12'h120;
    localparam [11:0] ADDR_LAST_VALID_W     = 12'h124;
    localparam [11:0] ADDR_TOTAL_WRF        = 12'h128;
    localparam [11:0] ADDR_WRF_PACKED       = 12'h12C;
    localparam [11:0] ADDR_KK               = 12'h130;
    localparam [11:0] ADDR_ROUNDS_PER_CINS  = 12'h134;
    localparam [11:0] ADDR_ROUND_LEN_LAST   = 12'h138;
    localparam [11:0] ADDR_IFB_BASE         = 12'h13C;
    localparam [11:0] ADDR_WB_BASE          = 12'h140;
    localparam [11:0] ADDR_OFB_BASE         = 12'h144;
    localparam [11:0] ADDR_IFB_CIN_STEP     = 12'h148;
    localparam [11:0] ADDR_IFB_ROW_STEP     = 12'h14C;
    localparam [11:0] ADDR_WB_CIN_STEP      = 12'h150;
    localparam [11:0] ADDR_WB_COUT_STEP     = 12'h154;
    localparam [11:0] ADDR_OFB_COUT_STEP    = 12'h158;
    localparam [11:0] ADDR_TILE_IN_STEP     = 12'h15C;
    localparam [11:0] ADDR_SDP_SHIFT        = 12'h160;
    localparam [11:0] ADDR_SDP_RELU_EN      = 12'h164;

    logic clk;
    logic rst_n;
    logic done;
    logic signed [NUM_COL*PSUM_WIDTH-1:0] psum_out_vec;
    logic [OFB_WIDTH-1:0] ofb_rdata_ext;

    // AXI-Lite S 驱动信号
    logic [CSR_ADDR_W-1:0]    csr_awaddr;
    logic                     csr_awvalid;
    logic                     csr_awready;
    logic [CSR_DATA_W-1:0]    csr_wdata;
    logic [CSR_DATA_W/8-1:0]  csr_wstrb;
    logic                     csr_wvalid;
    logic                     csr_wready;
    logic [1:0]               csr_bresp;
    logic                     csr_bvalid;
    logic                     csr_bready;
    logic [CSR_ADDR_W-1:0]    csr_araddr = 0;
    logic                     csr_arvalid = 0;
    logic                     csr_arready;
    logic [CSR_DATA_W-1:0]    csr_rdata;
    logic [1:0]               csr_rresp;
    logic                     csr_rvalid;
    logic                     csr_rready = 0;

    core_top #(
        .NUM_COL(NUM_COL),
        .NUM_PE(NUM_PE),
        .DATA_WIDTH(DATA_WIDTH),
        .PSUM_WIDTH(PSUM_WIDTH),
        .WRF_DEPTH(32),
        .ARF_DEPTH(32),
        .PARF_DEPTH(32),
        .SRAM_DEPTH(SRAM_DEPTH),
        .CSR_ADDR_W(CSR_ADDR_W),
        .CSR_DATA_W(CSR_DATA_W)
    ) u_core_top (
        .clk(clk),
        .rst_n(rst_n),
        .csr_awaddr(csr_awaddr),   .csr_awvalid(csr_awvalid), .csr_awready(csr_awready),
        .csr_wdata(csr_wdata),     .csr_wstrb(csr_wstrb),
        .csr_wvalid(csr_wvalid),   .csr_wready(csr_wready),
        .csr_bresp(csr_bresp),     .csr_bvalid(csr_bvalid),   .csr_bready(csr_bready),
        .csr_araddr(csr_araddr),   .csr_arvalid(csr_arvalid), .csr_arready(csr_arready),
        .csr_rdata(csr_rdata),     .csr_rresp(csr_rresp),
        .csr_rvalid(csr_rvalid),   .csr_rready(csr_rready),
        // 外部 AXI M 在本 TB 不用 DMA，全 tie 为 slave side "always ready, never valid"
        .bus_awid(), .bus_awaddr(), .bus_awlen(), .bus_awburst(),
        .bus_awvalid(), .bus_awready(1'b1),
        .bus_wdata(), .bus_wstrb(), .bus_wlast(), .bus_wvalid(), .bus_wready(1'b1),
        .bus_bid('0), .bus_bresp(2'b00), .bus_bvalid(1'b0), .bus_bready(),
        .bus_arid(), .bus_araddr(), .bus_arlen(), .bus_arburst(),
        .bus_arvalid(), .bus_arready(1'b1),
        .bus_rid('0), .bus_rdata('0), .bus_rresp(2'b00),
        .bus_rlast(1'b0), .bus_rvalid(1'b0), .bus_rready(),
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

    // Cycle counter：利用 cfg_regs 内部 r_core_busy（start_pulse 置位、core_done 清零）
    longint total_cycles = 0;
    always_ff @(posedge clk) begin
        if (!rst_n)                            total_cycles <= 0;
        else if (u_core_top.u_cfg.r_core_busy) total_cycles <= total_cycles + 1;
        else                                    total_cycles <= total_cycles;
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
    // AXI-Lite S 写单拍 task（用 nonblocking 驱动规避 Active region race；详见
    // tb_axi_lite_csr 的 debug 过程）
    // -------------------------------------------------------------------------
    task automatic axi_lite_write(input [CSR_ADDR_W-1:0] addr,
                                   input [CSR_DATA_W-1:0] data);
        csr_awaddr  <= addr;
        csr_awvalid <= 1'b1;
        csr_wdata   <= data;
        csr_wstrb   <= '1;
        csr_wvalid  <= 1'b1;
        do @(posedge clk); while (!(csr_awvalid && csr_awready));
        csr_awvalid <= 1'b0;
        while (!(csr_wvalid && csr_wready)) @(posedge clk);
        csr_wvalid  <= 1'b0;
        csr_bready  <= 1'b1;
        do @(posedge clk); while (!(csr_bvalid && csr_bready));
        csr_bready  <= 1'b0;
    endtask

    // -------------------------------------------------------------------------
    // Config loader：解析 config.txt，逐字段 AXI-Lite 写入 cfg_regs
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
            count = $sscanf(line, "%s = %d", key, val);
            if (count < 2) continue;
            case (key)
                "H_OUT"          : axi_lite_write(ADDR_H_OUT,           val);
                "W_OUT"          : axi_lite_write(ADDR_W_OUT,           val);
                "W_IN"           : axi_lite_write(ADDR_W_IN,            val);
                "K"              : axi_lite_write(ADDR_K,               val);
                "STRIDE"         : axi_lite_write(ADDR_STRIDE,          val);
                "CIN_SLICES"     : axi_lite_write(ADDR_CIN_SLICES,      val);
                "COUT_SLICES"    : axi_lite_write(ADDR_COUT_SLICES,     val);
                "TILE_W"         : axi_lite_write(ADDR_TILE_W,          val);
                "TOTAL_WRF"      : axi_lite_write(ADDR_TOTAL_WRF,       val);
                "WRF_PACKED"     : axi_lite_write(ADDR_WRF_PACKED,      val);
                "KK"             : axi_lite_write(ADDR_KK,              val);
                "ROUNDS_PER_CINS": axi_lite_write(ADDR_ROUNDS_PER_CINS, val);
                "ROUND_LEN_LAST" : axi_lite_write(ADDR_ROUND_LEN_LAST,  val);
                "IFB_BASE"       : axi_lite_write(ADDR_IFB_BASE,        val);
                "WB_BASE"        : axi_lite_write(ADDR_WB_BASE,         val);
                "OFB_BASE"       : axi_lite_write(ADDR_OFB_BASE,        val);
                "IFB_CIN_STEP"   : axi_lite_write(ADDR_IFB_CIN_STEP,    val);
                "IFB_ROW_STEP"   : axi_lite_write(ADDR_IFB_ROW_STEP,    val);
                "WB_CIN_STEP"    : axi_lite_write(ADDR_WB_CIN_STEP,     val);
                "WB_COUT_STEP"   : axi_lite_write(ADDR_WB_COUT_STEP,    val);
                "OFB_COUT_STEP"  : axi_lite_write(ADDR_OFB_COUT_STEP,   val);
                "NUM_TILES"      : axi_lite_write(ADDR_NUM_TILES,       val);
                "LAST_VALID_W"   : axi_lite_write(ADDR_LAST_VALID_W,    val);
                "TILE_IN_STEP"   : axi_lite_write(ADDR_TILE_IN_STEP,    val);
                "SDP_SHIFT"      : axi_lite_write(ADDR_SDP_SHIFT,       val);
                "SDP_RELU_EN"    : axi_lite_write(ADDR_SDP_RELU_EN,     val);
                default          : $display("WARN: unknown config key '%s'", key);
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
        rst_n       = 0;
        csr_awaddr  = 0; csr_awvalid = 0;
        csr_wdata   = 0; csr_wstrb   = 0; csr_wvalid = 0;
        csr_bready  = 0;
        csr_araddr  = 0; csr_arvalid = 0; csr_rready = 0;
        #20;
        rst_n = 1;
        #10;

        // Load expected OFM
        $readmemh("expected_ofm.txt", expected_ofm_arr);

        // 通过 AXI-Lite 写入配置
        load_config();
        #10;

        $display("========================================");
        $display("Starting AXI-Lite driven CNN Core...");
        $display("========================================");

        // 写 CTRL=0x1 触发 start_pulse
        axi_lite_write(ADDR_CTRL, 32'h0000_0001);

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
