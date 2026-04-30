`timescale 1ns/1ps

// =============================================================================
// tb_multicore.sv  --  M1.5 smoke test for multicore_top (NUM_CORES=2)
//
// 验证目标:
//   1. AXI-Lite 1:2 路由通畅 (host 写 14-bit 地址, 高 1 bit 选核)
//   2. AXI4 2:1 聚合通畅 (两核 master 共享一个 DDR slave)
//   3. 两核同时跑同一个 case (各自独立 DDR 区, 各自 done, 各自 OFB 都对)
//
// 简化:
//   - 跑独立小 case (cases/case00/, K=3 C8C16 30x30, 不依赖 chain)
//   - 两核都跑同一份 desc/data, 但分别在 DDR 的 2 个 base 区
//   - Core 0 base = 0x0000_0000, Core 1 base = 0x0100_0000 (16 MB 偏移已超 case 大小)
//   - 不做 perf profiling, 只验证 done + OFB 一致性
// =============================================================================

module tb_multicore;
    timeunit 1ns; timeprecision 1ps;

    // ----------------- 参数 -----------------
    localparam int NUM_COL    = 16;
    localparam int NUM_PE     = 16;
    localparam int DATA_WIDTH = 8;
    localparam int PSUM_WIDTH = 32;
    localparam int SRAM_DEPTH = 8192;
    localparam int CSR_DATA_W = 32;
    localparam int BUS_ADDR_W = 32;
    localparam int BUS_DATA_W = 128;
    localparam int AXI_M_ID   = 2;
    localparam int AXI_M_W    = 2;

    localparam int NUM_CORES   = 2;
    localparam int CORE_ID_W   = 1;                          // log2(2)
    localparam int HOST_CSR_AW = 12 + CORE_ID_W;             // 13 bit
    localparam int CORE_BUS_ID = AXI_M_ID + AXI_M_W;         // 4
    localparam int EXT_BUS_ID  = CORE_BUS_ID + CORE_ID_W;    // 5
    localparam int DDR_DEPTH   = 1048576*2;                  // 32 MB (够装 2 份 case)

    // 每核独立 DDR 区
    localparam [31:0] CORE0_BASE_OFFSET = 32'h0000_0000;
    localparam [31:0] CORE1_BASE_OFFSET = 32'h0100_0000;     // 16 MB 偏移

    // case00 默认 DDR layout (跟 hw_files 一致, 单 case 从 base 0 起)
    localparam [31:0] CASE_IFB_BASE  = 32'h0000_0000;
    localparam [31:0] CASE_DESC_BASE = 32'h007F_0000;
    localparam [31:0] CASE_WB_BASE   = 32'h0080_0000;
    localparam [31:0] CASE_OFB_BASE  = 32'h0090_0000;
    localparam [31:0] CASE_RDMA_BASE = 32'h00A0_0000;

    // cfg_regs 关键地址
    localparam [11:0] ADDR_CTRL             = 12'h000;
    localparam [11:0] ADDR_STATUS           = 12'h004;
    localparam [11:0] ADDR_DESC_LIST_BASE   = 12'h180;
    localparam [11:0] ADDR_DESC_COUNT       = 12'h184;
    localparam [11:0] ADDR_DMA_MODE         = 12'h17C;

    // ----------------- 时钟复位 -----------------
    logic clk = 0;  always #5 clk = ~clk;
    logic rst_n = 0;

    // ----------------- multicore_top 信号 -----------------
    // host AXI-Lite (13-bit 地址)
    logic [HOST_CSR_AW-1:0] csr_awaddr;
    logic                   csr_awvalid, csr_awready;
    logic [CSR_DATA_W-1:0]  csr_wdata;
    logic [CSR_DATA_W/8-1:0]csr_wstrb;
    logic                   csr_wvalid, csr_wready;
    logic [1:0]             csr_bresp;
    logic                   csr_bvalid, csr_bready;
    logic [HOST_CSR_AW-1:0] csr_araddr  = 0;
    logic                   csr_arvalid = 0, csr_arready;
    logic [CSR_DATA_W-1:0]  csr_rdata;
    logic [1:0]             csr_rresp;
    logic                   csr_rvalid, csr_rready = 0;

    // 外部 AXI4 master
    logic [EXT_BUS_ID-1:0]    bus_awid, bus_bid, bus_arid, bus_rid;
    logic [BUS_ADDR_W-1:0]    bus_awaddr, bus_araddr;
    logic [7:0]               bus_awlen, bus_arlen;
    logic [1:0]               bus_awburst, bus_arburst, bus_bresp, bus_rresp;
    logic                     bus_awvalid, bus_awready, bus_wvalid, bus_wready, bus_wlast;
    logic                     bus_bvalid, bus_bready, bus_arvalid, bus_arready;
    logic                     bus_rvalid, bus_rready, bus_rlast;
    logic [BUS_DATA_W-1:0]    bus_wdata, bus_rdata;
    logic [BUS_DATA_W/8-1:0]  bus_wstrb;

    logic [NUM_CORES-1:0]     done_per_core;

    // ----------------- DUT -----------------
    multicore_top #(
        .NUM_CORES(NUM_CORES),
        .NUM_COL(NUM_COL), .NUM_PE(NUM_PE),
        .DATA_WIDTH(DATA_WIDTH), .PSUM_WIDTH(PSUM_WIDTH),
        .SRAM_DEPTH(SRAM_DEPTH),
        .BUS_ADDR_W(BUS_ADDR_W), .BUS_DATA_W(BUS_DATA_W),
        .AXI_M_ID(AXI_M_ID), .AXI_M_WIDTH(AXI_M_W)
    ) u_dut (
        .clk(clk), .rst_n(rst_n),
        .csr_awaddr(csr_awaddr), .csr_awvalid(csr_awvalid), .csr_awready(csr_awready),
        .csr_wdata(csr_wdata), .csr_wstrb(csr_wstrb), .csr_wvalid(csr_wvalid), .csr_wready(csr_wready),
        .csr_bresp(csr_bresp), .csr_bvalid(csr_bvalid), .csr_bready(csr_bready),
        .csr_araddr(csr_araddr), .csr_arvalid(csr_arvalid), .csr_arready(csr_arready),
        .csr_rdata(csr_rdata), .csr_rresp(csr_rresp), .csr_rvalid(csr_rvalid), .csr_rready(csr_rready),

        .bus_awid(bus_awid), .bus_awaddr(bus_awaddr), .bus_awlen(bus_awlen),
        .bus_awburst(bus_awburst), .bus_awvalid(bus_awvalid), .bus_awready(bus_awready),
        .bus_wdata(bus_wdata), .bus_wstrb(bus_wstrb), .bus_wlast(bus_wlast),
        .bus_wvalid(bus_wvalid), .bus_wready(bus_wready),
        .bus_bid(bus_bid), .bus_bresp(bus_bresp), .bus_bvalid(bus_bvalid), .bus_bready(bus_bready),
        .bus_arid(bus_arid), .bus_araddr(bus_araddr), .bus_arlen(bus_arlen),
        .bus_arburst(bus_arburst), .bus_arvalid(bus_arvalid), .bus_arready(bus_arready),
        .bus_rid(bus_rid), .bus_rdata(bus_rdata), .bus_rresp(bus_rresp), .bus_rlast(bus_rlast),
        .bus_rvalid(bus_rvalid), .bus_rready(bus_rready),

        .done_per_core(done_per_core)
    );

    // ----------------- DDR mock -----------------
    axi_slave_mem #(
        .ADDR_W(BUS_ADDR_W), .DATA_W(BUS_DATA_W), .ID_W(EXT_BUS_ID), .DEPTH(DDR_DEPTH)
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

    // ----------------- Preload buffers -----------------
    logic [BUS_DATA_W-1:0]   ifb_arr  [0:32767];
    logic [NUM_COL*NUM_PE*DATA_WIDTH-1:0] wb_arr [0:2047];
    logic [NUM_COL*DATA_WIDTH-1:0]        exp_arr [0:32767];
    logic [BUS_DATA_W-1:0]   desc_arr [0:4095];
    logic [BUS_DATA_W-1:0]   rdma_arr [0:1023];

    int ifb_words, wb_words, ofb_words, desc_count, rdma_words;
    int desc_beats;

    // case 解析: 只取需要的几个字段
    task automatic parse_case_meta(input string case_dir);
        int    fd;
        string line, key;
        int    val;
        int    cnt;
        string path;
        path = $sformatf("%s/config.txt", case_dir);
        fd = $fopen(path, "r");
        if (fd == 0) begin $display("FATAL: cannot open %s", path); $stop; end
        ifb_words = 0; wb_words = 0; ofb_words = 0; desc_count = 0; rdma_words = 0;
        while (!$feof(fd)) begin
            void'($fgets(line, fd));
            cnt = $sscanf(line, "%s = %d", key, val);
            if (cnt < 2) continue;
            case (key)
                "_META_IFB_WORDS" : ifb_words = val;
                "_META_WB_WORDS"  : wb_words  = val;
                "_META_OFB_WORDS" : ofb_words = val;
                "_META_RDMA_WORDS": rdma_words = val;
                "DESC_COUNT"      : desc_count = val;
                default           : ;
            endcase
        end
        $fclose(fd);
        desc_beats = desc_count * 2;
    endtask

    // 把 case 数据 preload 到 DDR (offset = base_offset + per-section)
    task automatic preload_one_core(input string case_dir, input [31:0] base_offset);
        int ifb_base_w  = (base_offset + CASE_IFB_BASE)  / 16;
        int wb_base_w   = (base_offset + CASE_WB_BASE)   / 16;
        int desc_base_w = (base_offset + CASE_DESC_BASE) / 16;
        int rdma_base_w = (base_offset + CASE_RDMA_BASE) / 16;
        int ofb_base_w  = (base_offset + CASE_OFB_BASE)  / 16;

        $readmemh($sformatf("%s/ifb.txt",       case_dir), ifb_arr);
        $readmemh($sformatf("%s/wb.txt",        case_dir), wb_arr);
        $readmemh($sformatf("%s/desc_list.hex", case_dir), desc_arr);
        if (rdma_words > 0)
            $readmemh($sformatf("%s/rdma_data.txt", case_dir), rdma_arr);

        for (int i = 0; i < ifb_words; i++)
            u_ddr.mem[ifb_base_w + i] = ifb_arr[i];
        for (int i = 0; i < wb_words; i++)
            for (int b = 0; b < 16; b++)
                u_ddr.mem[wb_base_w + i*16 + b] = wb_arr[i][b*BUS_DATA_W +: BUS_DATA_W];
        for (int i = 0; i < desc_beats; i++)
            u_ddr.mem[desc_base_w + i] = desc_arr[i];
        for (int i = 0; i < rdma_words; i++)
            u_ddr.mem[rdma_base_w + i] = rdma_arr[i];
        // 清 OFB 区
        for (int i = 0; i < ofb_words; i++)
            u_ddr.mem[ofb_base_w + i] = '0;
    endtask

    // ----------------- AXI-Lite write helper -----------------
    // 13-bit 地址: [12]=core_id, [11:0]=reg
    task automatic axi_lite_write(input bit core_id, input [11:0] reg_addr, input [31:0] data);
        csr_awaddr  <= {core_id, reg_addr};
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

    // ----------------- 启动一个核 (boot regs 写 + 触发) -----------------
    // 因 generate 块的索引必须是常数, 把 core 0 / core 1 的 dfe poll 拆开
    task automatic start_core(input bit core_id, input [31:0] base_offset);
        axi_lite_write(core_id, ADDR_DESC_LIST_BASE, base_offset + CASE_DESC_BASE);
        axi_lite_write(core_id, ADDR_DESC_COUNT,     desc_count);
        axi_lite_write(core_id, ADDR_CTRL, 32'h0000_0010);   // start_dfe
        if (core_id == 1'b0) begin
            wait (u_dut.gen_core[0].u_core.dfe_busy == 1'b1);
            wait (u_dut.gen_core[0].u_core.dfe_busy == 1'b0);
        end else begin
            wait (u_dut.gen_core[1].u_core.dfe_busy == 1'b1);
            wait (u_dut.gen_core[1].u_core.dfe_busy == 1'b0);
        end
        axi_lite_write(core_id, ADDR_CTRL, 32'h0000_0020);   // start_layer
    endtask

    // 按 base_offset + CASE_*_BASE 重写 CFG_WRITE descriptor 中的 IDMA/WDMA/ODMA/RDMA 基址
    //   case 自带的 desc 用 ddr_ofb_base=None 默认 0 生成 (相对地址), TB 在这里
    //   按 CASE_*_BASE layout (IFB@0, WB@0x800000, OFB@0x900000, RDMA@0xA00000)
    //   叠加 base_offset (核间区分) 重写到绝对 DDR 地址.
    task automatic patch_desc_addrs(input [31:0] base_offset);
        // CFG_WRITE descriptor 字段:
        //   word 0 [3:0]  type=0x3
        //   word 0 [15:4] cfg_reg_addr (12-bit)
        //   word 0 [63:32] cfg_reg_data (32-bit, 跟 sequencer hd_cfg_data 解码一致)
        int desc_base_w = (base_offset + CASE_DESC_BASE) / 16;
        for (int i = 0; i < desc_beats; i++) begin
            if ((i & 1) == 0) begin
                logic [3:0]  ty   = u_ddr.mem[desc_base_w + i][3:0];
                logic [11:0] addr = u_ddr.mem[desc_base_w + i][15:4];
                if (ty == 4'h3) begin
                    case (addr)
                        12'h200: u_ddr.mem[desc_base_w + i][63:32] = base_offset + CASE_IFB_BASE;
                        12'h210: u_ddr.mem[desc_base_w + i][63:32] = base_offset + CASE_WB_BASE;
                        12'h220: u_ddr.mem[desc_base_w + i][63:32] = base_offset + CASE_OFB_BASE;
                        12'h230: u_ddr.mem[desc_base_w + i][63:32] = base_offset + CASE_RDMA_BASE;
                        default: ;
                    endcase
                end
            end
        end
    endtask

    // ----------------- OFB 比对 -----------------
    task automatic check_ofb(input string case_dir, input [31:0] base_offset,
                              input bit core_id, output int mismatches);
        int ofb_base_w = (base_offset + CASE_OFB_BASE) / 16;
        logic [NUM_COL*DATA_WIDTH-1:0] expected, got;
        $readmemh($sformatf("%s/expected_ofm.txt", case_dir), exp_arr);
        mismatches = 0;
        for (int i = 0; i < ofb_words; i++) begin
            expected = exp_arr[i];
            got      = u_ddr.mem[ofb_base_w + i][NUM_COL*DATA_WIDTH-1:0];
            if (got !== expected) begin
                if (mismatches < 3)
                    $display("  CORE %0d FAIL OFB[%0d]: expect=%h got=%h",
                             core_id, i, expected, got);
                mismatches++;
            end
        end
    endtask

    // ----------------- 主流程 -----------------
    initial begin
        string case_dir;
        int    mismatches_c0, mismatches_c1;
        longint timeout_ns;
        longint t_start;

        case_dir = "cases/case00";
        timeout_ns = 100_000_000;   // 100 ms 仿真上限

        csr_awaddr  = 0; csr_awvalid = 0;
        csr_wdata   = 0; csr_wstrb   = 0; csr_wvalid = 0;
        csr_bready  = 0;
        csr_araddr  = 0; csr_arvalid = 0; csr_rready = 0;
        #20 rst_n = 1;
        #10;

        $display("== tb_multicore: 2-core smoke test ==");
        $display("== case = %s, 2 cores running concurrently in DDR base [0x%08x, 0x%08x] ==",
                 case_dir, CORE0_BASE_OFFSET, CORE1_BASE_OFFSET);

        // --- 解析 case meta + preload 两份 DDR ---
        parse_case_meta(case_dir);
        $display("  meta: ifb=%0d wb=%0d ofb=%0d desc=%0d rdma=%0d",
                 ifb_words, wb_words, ofb_words, desc_count, rdma_words);

        preload_one_core(case_dir, CORE0_BASE_OFFSET);
        preload_one_core(case_dir, CORE1_BASE_OFFSET);
        // 两核 desc list 都要按 CASE_*_BASE + base_offset 重写 base addr
        // (case desc 默认 base=0, 必须 patch 才能跟 preload 区域对齐)
        patch_desc_addrs(CORE0_BASE_OFFSET);
        patch_desc_addrs(CORE1_BASE_OFFSET);

        $display("  DDR preload done, starting both cores...");

        // --- 启动两核 (顺序启动, 但跑在并发期, AXI/CSR 都共享一条总线) ---
        t_start = $time;
        start_core(0, CORE0_BASE_OFFSET);
        $display("  Core 0 started @ t=%0t", $time);
        start_core(1, CORE1_BASE_OFFSET);
        $display("  Core 1 started @ t=%0t", $time);

        // --- 等两核都完成 ---
        wait ((done_per_core[0] == 1'b1) && (done_per_core[1] == 1'b1));
        $display("  BOTH cores done @ t=%0t (cycles=%0d)",
                 $time, ($time - t_start) / 10);

        @(posedge clk); @(posedge clk);

        // --- 比对各核 OFB ---
        check_ofb(case_dir, CORE0_BASE_OFFSET, 1'b0, mismatches_c0);
        check_ofb(case_dir, CORE1_BASE_OFFSET, 1'b1, mismatches_c1);

        $display("");
        $display("============================================================");
        if (mismatches_c0 == 0 && mismatches_c1 == 0) begin
            $display("  RESULT: PASS  (Core 0 OFB matches, Core 1 OFB matches)");
        end else begin
            $display("  RESULT: FAIL  Core 0 mismatches=%0d  Core 1 mismatches=%0d",
                     mismatches_c0, mismatches_c1);
        end
        $display("  Wall time: %0d ns (%0d cycles @ 10 ns)",
                 $time - t_start, ($time - t_start)/10);
        $display("============================================================");

        $finish;
    end

    // 总 watchdog (12 ms 仿真上限)
    initial begin
        #12_000_000;
        $display("FATAL: watchdog timeout @ %0t", $time);
        $stop;
    end

endmodule
