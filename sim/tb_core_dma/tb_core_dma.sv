`timescale 1ns/1ps

// =============================================================================
// tb_core_dma.sv  --  End-to-end descriptor-driven TB
//
// 流程：
//   1. $readmemh 把 ifb.txt / wb.txt / desc_list.hex 预填到 DDR stub 各区域
//   2. AXI-Lite 写 layer-level cfg (K/stride/...、DMA base、SDP 等)
//   3. AXI-Lite 写 DESC_LIST_BASE + DESC_COUNT
//   4. 写 CTRL[4]=1 启动 DFE 拉 descriptor 到片内 FIFO
//   5. 写 CTRL[5]=1 启动 Sequencer 消费 descriptor，驱动核流水 + 所有 DMA
//   6. Poll STATUS[11] = layer_done
//   7. 读 DDR OFB 区对比 expected_ofm.txt
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
    localparam DDR_DEPTH  = 1048576;               // 16 MB，128-bit words (支持 480x640 流式 double)

    localparam IFB_WIDTH  = NUM_PE * DATA_WIDTH;       // 128
    localparam WB_WIDTH   = NUM_COL*NUM_PE*DATA_WIDTH; // 2048
    localparam OFB_WIDTH  = NUM_COL * DATA_WIDTH;      // 128

    // DDR 布局（byte 地址）
    //   IFB:  [0, 8MB - 64KB)  — 480x640x16 = 4.9 MB
    //   DESC: [8MB - 64KB, 8MB) — 64KB 足以放 ~2000 条 descriptor
    //   WB:   [8MB, 9MB) — 固定小空间
    //   OFB:  [9MB, 16MB) — 7 MB，放 480x640 OFM
    localparam [31:0] DDR_IFB_BASE  = 32'h0000_0000;
    localparam [31:0] DDR_DESC_BASE = 32'h007F_0000;
    localparam [31:0] DDR_WB_BASE   = 32'h0080_0000;
    localparam [31:0] DDR_OFB_BASE  = 32'h0090_0000;

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
    localparam [11:0] ADDR_IFB_ROW_STEP     = 12'h14C;
    localparam [11:0] ADDR_WB_COUT_STEP     = 12'h154;
    localparam [11:0] ADDR_TILE_IN_STEP     = 12'h15C;
    localparam [11:0] ADDR_SDP_SHIFT        = 12'h160;
    localparam [11:0] ADDR_SDP_RELU_EN      = 12'h164;
    localparam [11:0] ADDR_SDP_MULT         = 12'h188;
    localparam [11:0] ADDR_SDP_ZP_OUT       = 12'h18C;
    localparam [11:0] ADDR_SDP_CLIP_MIN     = 12'h190;
    localparam [11:0] ADDR_SDP_CLIP_MAX     = 12'h194;
    localparam [11:0] ADDR_SDP_ROUND_EN     = 12'h198;
    localparam [11:0] ADDR_H_IN_TOTAL       = 12'h168;
    localparam [11:0] ADDR_IFB_STRIP_ROWS   = 12'h16C;
    localparam [11:0] ADDR_OFB_STRIP_ROWS   = 12'h170;
    localparam [11:0] ADDR_DDR_IFM_ROW_STR  = 12'h174;
    localparam [11:0] ADDR_DDR_OFM_ROW_STR  = 12'h178;
    localparam [11:0] ADDR_DMA_MODE         = 12'h17C;
    localparam [11:0] ADDR_DESC_LIST_BASE   = 12'h180;
    localparam [11:0] ADDR_DESC_COUNT       = 12'h184;
    localparam [11:0] ADDR_IFB_RING_WORDS   = 12'h1A0;
    localparam [11:0] ADDR_OFB_ROW_WORDS    = 12'h1A4;
    localparam [11:0] ADDR_OFB_RING_WORDS   = 12'h1A8;
    localparam [11:0] ADDR_IFB_ISS_STEP     = 12'h1AC;
    localparam [11:0] ADDR_IFB_KY_STEP      = 12'h1B0;
    localparam [11:0] ADDR_TILE_PIX_STEP    = 12'h1B4;
    localparam [11:0] ADDR_ARF_REUSE_EN     = 12'h1B8;
    localparam [11:0] ADDR_STATUS           = 12'h004;
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
    logic [BUS_DATA_W-1:0] ifb_arr  [0:524287];
    logic [WB_WIDTH-1:0]   wb_arr   [0:2047];
    logic [OFB_WIDTH-1:0]  exp_arr  [0:524287];
    logic [BUS_DATA_W-1:0] desc_arr [0:4095];     // 2 beats/desc × up to 2048 desc

    // Phase G: 支持 per-layer DDR base + 可选跳过 IFB preload / OFB clear（多层场景）
    task automatic preload_ddr(input string case_dir,
                                input int ifb_words, input int wb_words,
                                input int desc_beats, input int ofb_words_to_clear,
                                input [31:0] ifb_base_byte, input [31:0] wb_base_byte,
                                input [31:0] desc_base_byte, input [31:0] ofb_base_byte,
                                input bit    skip_ifb_preload, input bit skip_ofb_clear);
        int ifb_base_w  = ifb_base_byte  / 16;
        int wb_base_w   = wb_base_byte   / 16;
        int desc_base_w = desc_base_byte / 16;
        if (!skip_ifb_preload) begin
            $readmemh($sformatf("%s/ifb.txt", case_dir), ifb_arr);
            for (int i = 0; i < ifb_words; i++) u_ddr.mem[ifb_base_w + i] = ifb_arr[i];
        end
        $readmemh($sformatf("%s/wb.txt",        case_dir), wb_arr);
        $readmemh($sformatf("%s/desc_list.hex", case_dir), desc_arr);

        // WB: 2048-bit lines 拆 16 个 128-bit 填 DDR（LSB 先）
        for (int i = 0; i < wb_words; i++)
            for (int b = 0; b < 16; b++)
                u_ddr.mem[wb_base_w + i*16 + b] = wb_arr[i][b*BUS_DATA_W +: BUS_DATA_W];

        // Descriptor list: 每 descriptor 2 × 128-bit beat (little-endian)
        for (int i = 0; i < desc_beats; i++) u_ddr.mem[desc_base_w + i] = desc_arr[i];

        // 清零 OFB 区（防止上一 case 残留）；多层中间层跳过以保留下层 IFB
        if (!skip_ofb_clear) begin
            for (int i = 0; i < ofb_words_to_clear; i++) u_ddr.mem[(ofb_base_byte/16) + i] = '0;
        end
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

    // TB 侧记录的 DMA_MODE（load_config 里更新）——用于主流程分流 batch / streaming
    logic [1:0] dma_mode_cfg = 2'b00;
    // 从 config.txt 读的 META 字段（TB 运行期用，不写 cfg_regs）
    string case_name_cfg;
    int    ifb_words_cfg, wb_words_cfg, ofb_words_cfg;
    int    num_cin_cfg, num_cout_cfg;
    // Phase G: 多层支持。缺失 → 用 TB hard-coded 默认
    logic [31:0] ddr_ifb_base_cfg, ddr_wb_base_cfg, ddr_ofb_base_cfg, ddr_desc_base_cfg;
    bit          skip_ifb_preload_cfg, skip_ofb_clear_cfg;

    task automatic load_config(input string case_dir);
        int      fd;
        string   line;
        string   key;
        int      val;
        int      count;
        string   path;
        // Phase G: META DDR base 默认值 = TB hard-coded localparam（单层回归兼容）
        ddr_ifb_base_cfg     = DDR_IFB_BASE;
        ddr_wb_base_cfg      = DDR_WB_BASE;
        ddr_ofb_base_cfg     = DDR_OFB_BASE;
        ddr_desc_base_cfg    = DDR_DESC_BASE;
        skip_ifb_preload_cfg = 1'b0;
        skip_ofb_clear_cfg   = 1'b0;
        path = $sformatf("%s/config.txt", case_dir);
        fd = $fopen(path, "r");
        if (fd == 0) begin
            $display("FATAL: cannot open %s", path);
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
                "IFB_BASE"       : axi_lite_write(ADDR_IFB_BASE_R,      val);
                "WB_BASE"        : axi_lite_write(ADDR_WB_BASE_R,       val);
                "OFB_BASE"       : axi_lite_write(ADDR_OFB_BASE_R,      val);
                "IFB_ROW_STEP"   : axi_lite_write(ADDR_IFB_ROW_STEP,    val);
                "WB_COUT_STEP"   : axi_lite_write(ADDR_WB_COUT_STEP,    val);
                "IFB_RING_WORDS" : axi_lite_write(ADDR_IFB_RING_WORDS,  val);
                "OFB_ROW_WORDS"  : axi_lite_write(ADDR_OFB_ROW_WORDS,   val);
                "OFB_RING_WORDS" : axi_lite_write(ADDR_OFB_RING_WORDS,  val);
                "IFB_ISS_STEP"   : axi_lite_write(ADDR_IFB_ISS_STEP,    val);
                "IFB_KY_STEP"    : axi_lite_write(ADDR_IFB_KY_STEP,    val);
                "TILE_PIX_STEP"  : axi_lite_write(ADDR_TILE_PIX_STEP,  val);
                "ARF_REUSE_EN"   : axi_lite_write(ADDR_ARF_REUSE_EN,   val);
                "NUM_TILES"      : axi_lite_write(ADDR_NUM_TILES,       val);
                "LAST_VALID_W"   : axi_lite_write(ADDR_LAST_VALID_W,    val);
                "TILE_IN_STEP"   : axi_lite_write(ADDR_TILE_IN_STEP,    val);
                "SDP_SHIFT"      : axi_lite_write(ADDR_SDP_SHIFT,       val);
                "SDP_RELU_EN"    : axi_lite_write(ADDR_SDP_RELU_EN,     val);
                "SDP_MULT"       : axi_lite_write(ADDR_SDP_MULT,        val);
                "SDP_ZP_OUT"     : axi_lite_write(ADDR_SDP_ZP_OUT,      val);
                "SDP_CLIP_MIN"   : axi_lite_write(ADDR_SDP_CLIP_MIN,    val);
                "SDP_CLIP_MAX"   : axi_lite_write(ADDR_SDP_CLIP_MAX,    val);
                "SDP_ROUND_EN"   : axi_lite_write(ADDR_SDP_ROUND_EN,    val);
                "H_IN_TOTAL"     : axi_lite_write(ADDR_H_IN_TOTAL,      val);
                "IFB_STRIP_ROWS" : axi_lite_write(ADDR_IFB_STRIP_ROWS,  val);
                "OFB_STRIP_ROWS" : axi_lite_write(ADDR_OFB_STRIP_ROWS,  val);
                "DDR_IFM_ROW_STRIDE" : axi_lite_write(ADDR_DDR_IFM_ROW_STR, val);
                "DDR_OFM_ROW_STRIDE" : axi_lite_write(ADDR_DDR_OFM_ROW_STR, val);
                "DMA_MODE"       : begin
                                      axi_lite_write(ADDR_DMA_MODE,     val);
                                      dma_mode_cfg = val[1:0];
                                   end
                "DESC_COUNT"     : axi_lite_write(ADDR_DESC_COUNT,      val);
                // META fields (不写 cfg_regs，TB 自用)
                "_META_IFB_WORDS" : ifb_words_cfg = val;
                "_META_WB_WORDS"  : wb_words_cfg  = val;
                "_META_OFB_WORDS" : ofb_words_cfg = val;
                "_META_NUM_CIN"   : num_cin_cfg   = val;
                "_META_NUM_COUT"  : num_cout_cfg  = val;
                // Phase G 多层：META DDR base + skip flags
                "_META_DDR_IFB_BASE"    : ddr_ifb_base_cfg     = val;
                "_META_DDR_WB_BASE"     : ddr_wb_base_cfg      = val;
                "_META_DDR_OFB_BASE"    : ddr_ofb_base_cfg     = val;
                "_META_DDR_DESC_BASE"   : ddr_desc_base_cfg    = val;
                "_META_SKIP_IFB_PRELOAD": skip_ifb_preload_cfg = (val != 0);
                "_META_SKIP_OFB_CLEAR"  : skip_ofb_clear_cfg   = (val != 0);
                default          : ;
            endcase
        end
        $fclose(fd);
        // CASE_NAME 单独解析（再开一次，sscanf %d 主循环拿不到字符串）
        fd = $fopen(path, "r");
        while (!$feof(fd)) begin
            void'($fgets(line, fd));
            if ($sscanf(line, "_META_CASE_NAME = %s", case_name_cfg) == 1) break;
        end
        $fclose(fd);
    endtask


    // ================== Layer cycle counter & perf ==================
    // layer_busy 覆盖 Sequencer 从 start_layer_pulse 到 layer_done 之间整个
    // 跨 strip 执行期；反映整 layer 端到端时间（含 DMA + core 流水）。
    longint core_cycles = 0;
    always_ff @(posedge clk) begin
        if      (!rst_n)                 core_cycles <= 0;
        else if (u_core.layer_busy)      core_cycles <= core_cycles + 1;
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

    // ------ SRAM read/write counters (E-3) ------
    // IFB/WB/OFB 在 core_top 里是 single-port SRAM 信号；这里用层级引用直接数
    // we/re 的 posedge 次数。仅在 layer_busy 期间计数（与 core_cycles 对齐）。
    longint ifb_we_cnt = 0, ifb_re_cnt = 0;
    longint wb_we_cnt  = 0, wb_re_cnt  = 0;
    longint ofb_we_cnt = 0, ofb_re_cnt = 0;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            ifb_we_cnt <= 0; ifb_re_cnt <= 0;
            wb_we_cnt  <= 0; wb_re_cnt  <= 0;
            ofb_we_cnt <= 0; ofb_re_cnt <= 0;
        end else if (u_core.layer_busy) begin
            if (u_core.ifb_we_mux) ifb_we_cnt <= ifb_we_cnt + 1;
            if (u_core.ifb_re)     ifb_re_cnt <= ifb_re_cnt + 1;
            if (u_core.wb_we_mux)  wb_we_cnt  <= wb_we_cnt  + 1;
            if (u_core.wb_re)      wb_re_cnt  <= wb_re_cnt  + 1;
            if (u_core.ofb_we)     ofb_we_cnt <= ofb_we_cnt + 1;
            if (u_core.ofb_re_mux) ofb_re_cnt <= ofb_re_cnt + 1;
        end
    end

    // ================== Watchdog + main ==================
    initial begin
        #2_000_000_000;
        $display("FATAL: timeout");
        $stop;
    end

    // ---- 每 case 跑一层：preload → cfg → DFE → start_layer → wait done → 比对 OFB → perf ----
    //   核之间不复位 rst_n；Sequencer 的 start_layer_pulse 会把 r_layer_done 清 0
    //   perf 计数器是 monotonic cumulative，每 case 打快照算 delta
    task automatic run_one_case(input int c);
        int ifb_bytes, wb_bytes, ofb_bytes;
        int desc_beats;
        int mismatch_cnt;
        logic [OFB_WIDTH-1:0] expected, got;
        string case_dir;
        // 每 case 开始前的 perf 快照
        longint snap_core_cycles;
        longint snap_mac_fire;
        int snap_act_fire, snap_act_stall, snap_act_idle;
        int snap_wgt_fire, snap_wgt_stall, snap_wgt_idle;
        int snap_psum_fire, snap_psum_stall, snap_psum_idle;
        int snap_acc_fire, snap_acc_stall, snap_acc_idle;
        longint snap_ifb_we, snap_ifb_re, snap_wb_we, snap_wb_re, snap_ofb_we, snap_ofb_re;
        int snap_arf_w, snap_arf_r, snap_arf_pad;
        int snap_parf_f, snap_parf_d;
        int snap_wrf_writes;
        longint delta_cycles, delta_mac_fire, delta_wrf_writes;
        int active_pe_per_fire;

        case_dir = (c < 10) ? $sformatf("cases/case0%0d", c)
                            : $sformatf("cases/case%0d",  c);

        // 快照 perf
        snap_core_cycles = core_cycles;
        snap_mac_fire    = u_core.u_mac_array.mac_fire_cnt;
        snap_act_fire   = u_core.u_mac_array.hs_act_fire;
        snap_act_stall  = u_core.u_mac_array.hs_act_stall;
        snap_act_idle   = u_core.u_mac_array.hs_act_idle;
        snap_wgt_fire   = u_core.u_mac_array.hs_wgt_fire;
        snap_wgt_stall  = u_core.u_mac_array.hs_wgt_stall;
        snap_wgt_idle   = u_core.u_mac_array.hs_wgt_idle;
        snap_psum_fire  = u_core.u_mac_array.hs_psum_fire;
        snap_psum_stall = u_core.u_mac_array.hs_psum_stall;
        snap_psum_idle  = u_core.u_mac_array.hs_psum_idle;
        snap_acc_fire   = u_core.u_ofb_writer.hs_acc_fire;
        snap_acc_stall  = u_core.u_ofb_writer.hs_acc_stall;
        snap_acc_idle   = u_core.u_ofb_writer.hs_acc_idle;
        snap_ifb_we = ifb_we_cnt; snap_ifb_re = ifb_re_cnt;
        snap_wb_we  = wb_we_cnt;  snap_wb_re  = wb_re_cnt;
        snap_ofb_we = ofb_we_cnt; snap_ofb_re = ofb_re_cnt;
        snap_arf_w   = u_core.u_line_buffer.arf_write_cnt;
        snap_arf_r   = u_core.u_line_buffer.arf_read_cnt;
        snap_arf_pad = u_core.u_line_buffer.pad_skip_cnt;
        snap_parf_f  = u_core.u_parf_accum.fill_fire_cnt;
        snap_parf_d  = u_core.u_parf_accum.drain_fire_cnt;
        snap_wrf_writes = 0;
        for (int cc = 0; cc < NUM_COL; cc++)
            for (int pp = 0; pp < NUM_PE; pp++)
                snap_wrf_writes += pe_wrf_write_arr[cc][pp];

        // ---- 加载 cfg (会解析 meta + 写 cfg_regs AXI-Lite) ----
        load_config(case_dir);

        desc_beats = 0;   // 先估算；实际 descriptor 数从 config.txt 已经 axi_lite_write 到 cfg_regs
        // 从 cfg_regs 读回 DESC_COUNT 粗略不便；简化：按最大 2048 desc 预填 DDR 足够
        // 精确计算：DESC_COUNT 也在 config.txt 里读，load_config 时已写 cfg_regs；TB 这里我们直接从 config 解析需再多加
        // 简单起见 preload 按配置算：从 ifb_words_cfg 等推出
        ifb_bytes = ifb_words_cfg * (BUS_DATA_W/8);
        wb_bytes  = wb_words_cfg  * (WB_WIDTH/8);
        ofb_bytes = ofb_words_cfg * (BUS_DATA_W/8);
        // desc_beats: preload 必须给个数；按 2048 max 足够
        desc_beats = 4096;   // 2048 desc × 2 beat; 实际使用按 DESC_COUNT

        // 载入 IFB/WB/DESC 到 DDR + 清 OFB 区
        preload_ddr(case_dir, ifb_words_cfg, wb_words_cfg, desc_beats, ofb_words_cfg,
                    ddr_ifb_base_cfg, ddr_wb_base_cfg,
                    ddr_desc_base_cfg, ddr_ofb_base_cfg,
                    skip_ifb_preload_cfg, skip_ofb_clear_cfg);
        // 中间层（skip_ofb_clear=1，OFB 是下层 IFB）没有 expected_ofm.txt，跳过读
        if (!skip_ofb_clear_cfg)
            $readmemh($sformatf("%s/expected_ofm.txt", case_dir), exp_arr);

        // DMA base (layer-level)
        axi_lite_write(ADDR_IDMA_SRC_BASE, ddr_ifb_base_cfg);
        axi_lite_write(ADDR_IDMA_BYTE_LEN, ifb_bytes);
        axi_lite_write(ADDR_WDMA_SRC_BASE, ddr_wb_base_cfg);
        axi_lite_write(ADDR_WDMA_BYTE_LEN, wb_bytes);
        axi_lite_write(ADDR_ODMA_DST_BASE, ddr_ofb_base_cfg);
        axi_lite_write(ADDR_ODMA_BYTE_LEN, ofb_bytes);
        axi_lite_write(ADDR_DESC_LIST_BASE, ddr_desc_base_cfg);
        // DESC_COUNT 已由 load_config 写入

        $display("=== CASE %0d: %s === t=%0t", c, case_name_cfg, $time);

        // DFE 拉 descriptor
        axi_lite_write(ADDR_CTRL, 32'h0000_0010);
        wait (u_core.dfe_busy == 1'b1);
        wait (u_core.dfe_busy == 1'b0);

        // Start layer
        axi_lite_write(ADDR_CTRL, 32'h0000_0020);
        wait (u_core.layer_busy == 1'b1);
        wait (u_core.layer_done == 1'b1);
        @(posedge clk);

        // 比对 DDR OFB vs expected；中间层无 expected_ofm → 不比对
        mismatch_cnt = 0;
        if (!skip_ofb_clear_cfg) begin
            for (int i = 0; i < ofb_words_cfg; i++) begin
                expected = exp_arr[i];
                got      = u_ddr.mem[(ddr_ofb_base_cfg / 16) + i];
                if (got !== expected) begin
                    if (mismatch_cnt < 3)
                        $display("  CASE %0d FAIL OFB[%0d]: expect=%h got=%h", c, i, expected, got);
                    mismatch_cnt++;
                end
            end
        end

        // Perf delta
        delta_cycles      = core_cycles - snap_core_cycles;
        delta_mac_fire    = u_core.u_mac_array.mac_fire_cnt - snap_mac_fire;
        delta_wrf_writes  = 0;
        for (int cc = 0; cc < NUM_COL; cc++)
            for (int pp = 0; pp < NUM_PE; pp++)
                delta_wrf_writes += pe_wrf_write_arr[cc][pp];
        delta_wrf_writes -= snap_wrf_writes;
        active_pe_per_fire = ((num_cin_cfg  > NUM_PE)  ? NUM_PE  : num_cin_cfg) *
                             ((num_cout_cfg > NUM_COL) ? NUM_COL : num_cout_cfg);

        // 单 case 结果（一行精简）+ 若干关键指标给 run_regression parser 用
        if (mismatch_cnt == 0)
            $display("CASE_RESULT %0d PASS  cycles=%0d  mac_fire=%0d  mac_util=%.2f%%  arf_w=%0d  arf_r=%0d  parf_f=%0d  parf_d=%0d  ifb_r=%0d  wb_r=%0d  ofb_w=%0d  name=%s",
                     c, delta_cycles, delta_mac_fire,
                     (real'(delta_mac_fire * active_pe_per_fire) / (real'(delta_cycles) * real'(NUM_COL * NUM_PE))) * 100.0,
                     u_core.u_line_buffer.arf_write_cnt - snap_arf_w,
                     u_core.u_line_buffer.arf_read_cnt  - snap_arf_r,
                     u_core.u_parf_accum.fill_fire_cnt  - snap_parf_f,
                     u_core.u_parf_accum.drain_fire_cnt - snap_parf_d,
                     ifb_re_cnt - snap_ifb_re,
                     wb_re_cnt  - snap_wb_re,
                     ofb_we_cnt - snap_ofb_we,
                     case_name_cfg);
        else
            $display("CASE_RESULT %0d FAIL  cycles=%0d  mismatches=%0d  name=%s",
                     c, delta_cycles, mismatch_cnt, case_name_cfg);
    endtask

    initial begin
        int n_cases;
        int pass_cnt = 0, fail_cnt = 0;

        csr_awaddr  = 0; csr_awvalid = 0;
        csr_wdata   = 0; csr_wstrb   = 0; csr_wvalid = 0;
        csr_bready  = 0;
        csr_araddr  = 0; csr_arvalid = 0; csr_rready = 0;
        #20 rst_n = 1;
        #10;

        if (!$value$plusargs("N_CASES=%d", n_cases)) n_cases = 1;
        $display("== tb_core_dma multi-case run: N_CASES=%0d ==", n_cases);

        for (int c = 0; c < n_cases; c++) begin
            run_one_case(c);
        end

        $display("== ALL %0d CASES DONE ==", n_cases);
        $stop;
    end

endmodule
