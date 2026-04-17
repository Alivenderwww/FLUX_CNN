`timescale 1ns/1ps

// =============================================================================
// tb_core_dma.sv  --  End-to-end DMA TB
//
// 完整流程：
//   1. $readmemh 把 ifb.txt (128-bit)、wb.txt (2048-bit) 预填到 DDR stub 不同区域
//   2. AXI-Lite 写所有 cfg 寄存器 + 3 个 DMA descriptor
//   3. 写 CTRL=0x6 同时触发 IDMA+WDMA；等 busy 双双归 0
//   4. 写 CTRL=0x1 触发核；wait core done
//   5. 写 CTRL=0x8 触发 ODMA；等 busy 归 0
//   6. 读 DDR OFB 区对比 expected_ofm.txt
// =============================================================================

module tb_core_dma;

    // ---------------- 参数 ----------------
    localparam NUM_COL    = 16;
    localparam NUM_PE     = 16;
    localparam DATA_WIDTH = 8;
    localparam PSUM_WIDTH = 32;
    parameter  SRAM_DEPTH = 8192;
    localparam CSR_ADDR_W = 12;
    localparam CSR_DATA_W = 32;
    localparam BUS_ADDR_W = 32;
    localparam BUS_DATA_W = 128;
    localparam AXI_M_ID   = 2;
    localparam AXI_M_W    = 2;
    localparam BUS_ID_W   = AXI_M_ID + AXI_M_W;   // 4
    localparam DDR_DEPTH  = 65536;                 // 1 MB，128-bit words

    localparam IFB_WIDTH  = NUM_PE * DATA_WIDTH;       // 128
    localparam WB_WIDTH   = NUM_COL*NUM_PE*DATA_WIDTH; // 2048
    localparam OFB_WIDTH  = NUM_COL * DATA_WIDTH;      // 128

    // DDR 布局（byte 地址）
    localparam [31:0] DDR_IFB_BASE = 32'h0000_0000;
    localparam [31:0] DDR_WB_BASE  = 32'h0002_0000;   // 128 KB offset
    localparam [31:0] DDR_OFB_BASE = 32'h0004_0000;   // 256 KB offset

    // cfg_regs 地址（和 RTL 对齐）
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
    localparam [11:0] ADDR_IFB_BASE_R       = 12'h13C;
    localparam [11:0] ADDR_WB_BASE_R        = 12'h140;
    localparam [11:0] ADDR_OFB_BASE_R       = 12'h144;
    localparam [11:0] ADDR_IFB_CIN_STEP     = 12'h148;
    localparam [11:0] ADDR_IFB_ROW_STEP     = 12'h14C;
    localparam [11:0] ADDR_WB_CIN_STEP      = 12'h150;
    localparam [11:0] ADDR_WB_COUT_STEP     = 12'h154;
    localparam [11:0] ADDR_OFB_COUT_STEP    = 12'h158;
    localparam [11:0] ADDR_TILE_IN_STEP     = 12'h15C;
    localparam [11:0] ADDR_SDP_SHIFT        = 12'h160;
    localparam [11:0] ADDR_SDP_RELU_EN      = 12'h164;
    localparam [11:0] ADDR_IDMA_SRC_BASE    = 12'h200;
    localparam [11:0] ADDR_IDMA_BYTE_LEN    = 12'h204;
    localparam [11:0] ADDR_WDMA_SRC_BASE    = 12'h210;
    localparam [11:0] ADDR_WDMA_BYTE_LEN    = 12'h214;
    localparam [11:0] ADDR_ODMA_DST_BASE    = 12'h220;
    localparam [11:0] ADDR_ODMA_BYTE_LEN    = 12'h224;

    logic clk  = 0;
    logic rst_n = 0;
    always #5 clk = ~clk;

    // ----------------- core_top 信号 -----------------
    logic signed [NUM_COL*PSUM_WIDTH-1:0] psum_out_vec;
    logic                                 done;

    // AXI-Lite S
    logic [CSR_ADDR_W-1:0]    csr_awaddr;
    logic                     csr_awvalid, csr_awready;
    logic [CSR_DATA_W-1:0]    csr_wdata;
    logic [CSR_DATA_W/8-1:0]  csr_wstrb;
    logic                     csr_wvalid, csr_wready;
    logic [1:0]               csr_bresp;
    logic                     csr_bvalid, csr_bready;
    logic [CSR_ADDR_W-1:0]    csr_araddr = 0;
    logic                     csr_arvalid = 0, csr_arready;
    logic [CSR_DATA_W-1:0]    csr_rdata;
    logic [1:0]               csr_rresp;
    logic                     csr_rvalid, csr_rready = 0;

    // AXI M (core → DDR)
    logic [BUS_ID_W-1:0]      bus_awid, bus_bid, bus_arid, bus_rid;
    logic [BUS_ADDR_W-1:0]    bus_awaddr, bus_araddr;
    logic [7:0]               bus_awlen, bus_arlen;
    logic [1:0]               bus_awburst, bus_arburst, bus_bresp, bus_rresp;
    logic                     bus_awvalid, bus_awready, bus_wvalid, bus_wready, bus_wlast;
    logic                     bus_bvalid, bus_bready, bus_arvalid, bus_arready;
    logic                     bus_rvalid, bus_rready, bus_rlast;
    logic [BUS_DATA_W-1:0]    bus_wdata, bus_rdata;
    logic [BUS_DATA_W/8-1:0]  bus_wstrb;

    core_top #(
        .NUM_COL(NUM_COL), .NUM_PE(NUM_PE), .DATA_WIDTH(DATA_WIDTH),
        .PSUM_WIDTH(PSUM_WIDTH), .WRF_DEPTH(32), .ARF_DEPTH(32), .PARF_DEPTH(32),
        .SRAM_DEPTH(SRAM_DEPTH), .CSR_ADDR_W(CSR_ADDR_W), .CSR_DATA_W(CSR_DATA_W),
        .BUS_ADDR_W(BUS_ADDR_W), .BUS_DATA_W(BUS_DATA_W),
        .AXI_M_ID(AXI_M_ID), .AXI_M_WIDTH(AXI_M_W)
    ) u_core (
        .clk(clk), .rst_n(rst_n),
        .csr_awaddr(csr_awaddr), .csr_awvalid(csr_awvalid), .csr_awready(csr_awready),
        .csr_wdata(csr_wdata), .csr_wstrb(csr_wstrb),
        .csr_wvalid(csr_wvalid), .csr_wready(csr_wready),
        .csr_bresp(csr_bresp), .csr_bvalid(csr_bvalid), .csr_bready(csr_bready),
        .csr_araddr(csr_araddr), .csr_arvalid(csr_arvalid), .csr_arready(csr_arready),
        .csr_rdata(csr_rdata), .csr_rresp(csr_rresp),
        .csr_rvalid(csr_rvalid), .csr_rready(csr_rready),
        .bus_awid(bus_awid), .bus_awaddr(bus_awaddr), .bus_awlen(bus_awlen), .bus_awburst(bus_awburst),
        .bus_awvalid(bus_awvalid), .bus_awready(bus_awready),
        .bus_wdata(bus_wdata), .bus_wstrb(bus_wstrb), .bus_wlast(bus_wlast),
        .bus_wvalid(bus_wvalid), .bus_wready(bus_wready),
        .bus_bid(bus_bid), .bus_bresp(bus_bresp), .bus_bvalid(bus_bvalid), .bus_bready(bus_bready),
        .bus_arid(bus_arid), .bus_araddr(bus_araddr), .bus_arlen(bus_arlen), .bus_arburst(bus_arburst),
        .bus_arvalid(bus_arvalid), .bus_arready(bus_arready),
        .bus_rid(bus_rid), .bus_rdata(bus_rdata), .bus_rresp(bus_rresp),
        .bus_rlast(bus_rlast), .bus_rvalid(bus_rvalid), .bus_rready(bus_rready),
        .ifb_we_ext(1'b0), .ifb_waddr_ext('0), .ifb_wdata_ext('0),
        .wb_we_ext(1'b0), .wb_waddr_ext('0), .wb_wdata_ext('0),
        .ofb_re_ext(1'b0), .ofb_raddr_ext('0), .ofb_rdata_ext(),
        .psum_out_vec(psum_out_vec), .done(done)
    );

    // DDR stub（outside core）
    axi_slave_mem #(
        .ADDR_W(BUS_ADDR_W), .DATA_W(BUS_DATA_W), .ID_W(BUS_ID_W), .DEPTH(DDR_DEPTH)
    ) u_ddr (
        .clk(clk), .rstn(rst_n),
        .AWID(bus_awid), .AWADDR(bus_awaddr), .AWLEN(bus_awlen), .AWBURST(bus_awburst),
        .AWVALID(bus_awvalid), .AWREADY(bus_awready),
        .WDATA(bus_wdata), .WSTRB(bus_wstrb), .WLAST(bus_wlast),
        .WVALID(bus_wvalid), .WREADY(bus_wready),
        .BID(bus_bid), .BRESP(bus_bresp), .BVALID(bus_bvalid), .BREADY(bus_bready),
        .ARID(bus_arid), .ARADDR(bus_araddr), .ARLEN(bus_arlen), .ARBURST(bus_arburst),
        .ARVALID(bus_arvalid), .ARREADY(bus_arready),
        .RID(bus_rid), .RDATA(bus_rdata), .RRESP(bus_rresp), .RLAST(bus_rlast),
        .RVALID(bus_rvalid), .RREADY(bus_rready)
    );

    // ================== 预填 DDR ==================
    logic [BUS_DATA_W-1:0] ifb_arr [0:65535];
    logic [WB_WIDTH-1:0]   wb_arr  [0:2047];
    logic [OFB_WIDTH-1:0]  exp_arr [0:65535];

    task automatic preload_ddr(input int ifb_words, input int wb_words);
        int ifb_base_w = DDR_IFB_BASE / 16;
        int wb_base_w  = DDR_WB_BASE  / 16;
        $readmemh("../tb_core_isa/ifb.txt", ifb_arr);
        $readmemh("../tb_core_isa/wb.txt",  wb_arr);

        // IFB: 128-bit lines 直接填 DDR
        for (int i = 0; i < ifb_words; i++) u_ddr.mem[ifb_base_w + i] = ifb_arr[i];

        // WB: 2048-bit lines 拆 16 个 128-bit 填 DDR（LSB 先）
        for (int i = 0; i < wb_words; i++)
            for (int b = 0; b < 16; b++)
                u_ddr.mem[wb_base_w + i*16 + b] = wb_arr[i][b*BUS_DATA_W +: BUS_DATA_W];
    endtask

    // ================== AXI-Lite 驱动 ==================
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

    task automatic load_config();
        int      fd;
        string   line;
        string   key;
        int      val;
        int      count;
        fd = $fopen("../tb_core_isa/config.txt", "r");
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
                "IFB_BASE"       : axi_lite_write(ADDR_IFB_BASE_R,      val);
                "WB_BASE"        : axi_lite_write(ADDR_WB_BASE_R,       val);
                "OFB_BASE"       : axi_lite_write(ADDR_OFB_BASE_R,      val);
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
                default          : ;
            endcase
        end
        $fclose(fd);
    endtask

    // ================== Core cycle counter & perf ==================
    // r_core_busy 来自 cfg_regs，在 start_core_pulse 置位、core_done 清零，
    // 正好框定 "core 开始接收 IFB 数据 → 整个核跑完" 的窗口。
    longint core_cycles = 0;
    always_ff @(posedge clk) begin
        if      (!rst_n)                         core_cycles <= 0;
        else if (u_core.u_cfg.r_core_busy)       core_cycles <= core_cycles + 1;
        else                                      core_cycles <= core_cycles;
    end

    int pe_wrf_write_arr [0:NUM_COL-1][0:NUM_PE-1];
    genvar gc, gp;
    generate
        for (gc = 0; gc < NUM_COL; gc++) begin : perf_col
            for (gp = 0; gp < NUM_PE; gp++) begin : perf_pe
                assign pe_wrf_write_arr[gc][gp] =
                    u_core.u_mac_array.gen_col[gc].u_col.gen_pe[gp].u_pe.u_wrf.total_write_ops;
            end
        end
    endgenerate

    // ================== Watchdog + main ==================
    initial begin
        #500_000_000;
        $display("FATAL: timeout");
        $stop;
    end

    initial begin
        int h_out_val, w_out_val, cout_slices_val;
        int ifb_words, wb_words, ofb_words;
        int ifb_bytes, wb_bytes, ofb_bytes;
        int mismatch_cnt;
        logic [OFB_WIDTH-1:0] expected, got;

        csr_awaddr  = 0; csr_awvalid = 0;
        csr_wdata   = 0; csr_wstrb   = 0; csr_wvalid = 0;
        csr_bready  = 0;
        csr_araddr  = 0; csr_arvalid = 0; csr_rready = 0;
        #20 rst_n = 1;
        #10;

        // 从 plusargs 拿尺寸；sim_params.f 已有 +H_OUT/+W_OUT/+COUT_SLICES
        if (!$value$plusargs("H_OUT=%d",        h_out_val))       h_out_val = 66;
        if (!$value$plusargs("W_OUT=%d",        w_out_val))       w_out_val = 118;
        if (!$value$plusargs("COUT_SLICES=%d",  cout_slices_val)) cout_slices_val = 1;

        // 按 gen_isa_test.py 的算法反推大小。此 TB 只关心 K=3 C8C8 规模验证；
        // 其他尺寸得外部传入 IFB_WORDS/WB_WORDS（这里写死简单版）
        if (!$value$plusargs("IFB_WORDS=%d", ifb_words)) ifb_words = 8160;
        if (!$value$plusargs("WB_WORDS=%d",  wb_words))  wb_words  = 9;
        ofb_words = h_out_val * w_out_val * cout_slices_val;

        ifb_bytes = ifb_words * (BUS_DATA_W/8);
        wb_bytes  = wb_words  * (WB_WIDTH/8);      // WB word = 256 B
        ofb_bytes = ofb_words * (BUS_DATA_W/8);

        $display("== tb_core_dma ==  ifb=%0d words (%0d B) wb=%0d words (%0d B) ofb=%0d words (%0d B)",
                 ifb_words, ifb_bytes, wb_words, wb_bytes, ofb_words, ofb_bytes);

        // DDR 预填
        preload_ddr(ifb_words, wb_words);
        $readmemh("../tb_core_isa/expected_ofm.txt", exp_arr);

        // cfg
        load_config();

        // DMA descriptors
        axi_lite_write(ADDR_IDMA_SRC_BASE, DDR_IFB_BASE);
        axi_lite_write(ADDR_IDMA_BYTE_LEN, ifb_bytes);
        axi_lite_write(ADDR_WDMA_SRC_BASE, DDR_WB_BASE);
        axi_lite_write(ADDR_WDMA_BYTE_LEN, wb_bytes);
        axi_lite_write(ADDR_ODMA_DST_BASE, DDR_OFB_BASE);
        axi_lite_write(ADDR_ODMA_BYTE_LEN, ofb_bytes);

        // Phase 1: IDMA + WDMA
        $display("== Phase 1: trigger IDMA + WDMA ==");
        axi_lite_write(ADDR_CTRL, 32'h0000_0006);   // CTRL[1]=IDMA, [2]=WDMA
        wait (u_core.idma_busy == 1'b1);
        wait (u_core.wdma_busy == 1'b1);
        wait (u_core.idma_busy == 1'b0);
        wait (u_core.wdma_busy == 1'b0);
        @(posedge clk);
        $display("   IDMA+WDMA done at t=%0t", $time);

        // Phase 2: core
        $display("== Phase 2: trigger core ==");
        axi_lite_write(ADDR_CTRL, 32'h0000_0001);
        wait (done == 1'b1);
        @(posedge clk);
        $display("   Core done at t=%0t", $time);

        // Phase 3: ODMA
        $display("== Phase 3: trigger ODMA ==");
        axi_lite_write(ADDR_CTRL, 32'h0000_0008);
        wait (u_core.odma_busy == 1'b1);
        wait (u_core.odma_busy == 1'b0);
        @(posedge clk);
        $display("   ODMA done at t=%0t", $time);

        // 对比 DDR 的 OFB 区 vs expected
        mismatch_cnt = 0;
        for (int i = 0; i < ofb_words; i++) begin
            expected = exp_arr[i];
            got      = u_ddr.mem[(DDR_OFB_BASE / 16) + i];
            if (got !== expected) begin
                if (mismatch_cnt < 5)
                    $display("  FAIL OFB[%0d]: expect=%h got=%h", i, expected, got);
                mismatch_cnt++;
            end
        end

        if (mismatch_cnt == 0) $display("TB_CORE_DMA PASSED. %0d OFB words checked.", ofb_words);
        else                   $display("TB_CORE_DMA FAILED. %0d mismatches.", mismatch_cnt);

        // ================== Perf stats (仅核运行窗口) ==================
        begin
            longint mac_fire_cycles;
            longint total_mac_ops;
            longint total_wrf_writes;
            mac_fire_cycles  = u_core.u_mac_array.mac_fire_cnt;
            total_mac_ops    = mac_fire_cycles * NUM_COL * NUM_PE;
            total_wrf_writes = 0;
            for (int c = 0; c < NUM_COL; c++)
                for (int p = 0; p < NUM_PE; p++)
                    total_wrf_writes += pe_wrf_write_arr[c][p];

            $display("========================================");
            $display(" CORE PERFORMANCE (start_core → done)");
            $display("========================================");
            $display("Core Active Cycles:   %0d", core_cycles);
            $display("MAC Fire Cycles:      %0d", mac_fire_cycles);
            $display("Total MAC Ops:        %0d (= fire * %0d PEs)", total_mac_ops, NUM_COL * NUM_PE);
            $display("Theoretical Max MACs: %0d (= cycles * %0d PEs)",
                     core_cycles * NUM_COL * NUM_PE, NUM_COL * NUM_PE);
            $display("MAC Utilization:      %.2f %%",
                     (real'(total_mac_ops) / (real'(core_cycles) * real'(NUM_COL * NUM_PE))) * 100.0);
            $display("WRF Writes:           %0d", total_wrf_writes);
            $display("--- Handshake Stats (fire / stall / idle) ---");
            $display("ACT  (lb  -> mac): fire=%0d stall=%0d idle=%0d",
                     u_core.u_mac_array.hs_act_fire,
                     u_core.u_mac_array.hs_act_stall,
                     u_core.u_mac_array.hs_act_idle);
            $display("WGT  (wb  -> mac): fire=%0d stall=%0d idle=%0d",
                     u_core.u_mac_array.hs_wgt_fire,
                     u_core.u_mac_array.hs_wgt_stall,
                     u_core.u_mac_array.hs_wgt_idle);
            $display("PSUM (mac -> prf): fire=%0d stall=%0d idle=%0d",
                     u_core.u_mac_array.hs_psum_fire,
                     u_core.u_mac_array.hs_psum_stall,
                     u_core.u_mac_array.hs_psum_idle);
            $display("ACC  (prf -> ofb): fire=%0d stall=%0d idle=%0d",
                     u_core.u_ofb_writer.hs_acc_fire,
                     u_core.u_ofb_writer.hs_acc_stall,
                     u_core.u_ofb_writer.hs_acc_idle);
            $display("========================================");
        end
        $stop;
    end

endmodule
