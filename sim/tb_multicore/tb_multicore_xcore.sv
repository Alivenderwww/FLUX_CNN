`timescale 1ns/1ps

// =============================================================================
// tb_multicore_xcore.sv  --  M2 跨核流水线 smoke (NUM_CORES=2)
//
// 验证目标:
//   1. Core 0 (producer) 算 layer 0, ODMA 不写 DDR 而是 push 到 0x9000_0000+
//      (Core 1 IFB AXI region), 经 axi_2to3 crossbar 路由到 ifb_axi_slave
//   2. Core 1 (consumer) SKIP_IDMA=1 不启本地 IDMA, IFB 等远端 push 进来
//      算完 layer 1 写 DDR OFB, host 比对 golden
//   3. ring 反压自动同步: Core 1 line_buffer 慢→IFB 满→ifb_axi_slave awready=0
//      →Core 0 axi_dm S2MM stall→Core 0 全流水线反压
//
// case 由 toolchain/gen_cross_core_test.py 生成:
//   cases/<name>/core0/  layer 0 (producer): ifb.txt + wb.txt + desc_list.hex + config.txt
//   cases/<name>/core1/  layer 1 (consumer):           wb.txt + desc_list.hex + config.txt
//                        (没 ifb.txt — IFB 不预加载, 等远端核 push)
//
// 启动顺序: consumer 先启动 (CTRL[5]) → producer 后启动
//   理由: ifb_axi_slave 的 wptr/rows_pushed 在 cfg_start_layer_pulse 时清零,
//   必须在 producer 开始 push 之前 reset, 否则上一 layer 残留计数让反压算错.
//
// DDR layout (跟 gen_cross_core_test.py 对齐, 单核 base 0):
//   0x0000_0000  Layer 0 IFB
//   0x0080_0000  Layer 0 WB
//   0x0090_0000  Layer 1 WB
//   0x00A0_0000  Layer 1 OFB (golden)
//   0x00B0_0000  Layer 0 desc list
//   0x00C0_0000  Layer 1 desc list
// =============================================================================

module tb_multicore_xcore;
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
    localparam int CORE_ID_W   = 1;
    localparam int HOST_CSR_AW = 12 + CORE_ID_W;
    localparam int CORE_BUS_ID = AXI_M_ID + AXI_M_W;
    localparam int EXT_BUS_ID  = CORE_BUS_ID + CORE_ID_W;
    localparam int DDR_DEPTH   = 1048576*2;

    // DDR layout
    localparam [31:0] DDR_L0_IFB_BASE   = 32'h0000_0000;
    localparam [31:0] DDR_L0_WB_BASE    = 32'h0080_0000;
    localparam [31:0] DDR_L1_WB_BASE    = 32'h0090_0000;
    localparam [31:0] DDR_L1_OFB_BASE   = 32'h00A0_0000;
    localparam [31:0] DDR_L0_DESC_BASE  = 32'h00B0_0000;
    localparam [31:0] DDR_L1_DESC_BASE  = 32'h00C0_0000;
    localparam [31:0] DDR_L0_RDMA_BASE  = 32'h00D0_0000;
    localparam [31:0] DDR_L1_RDMA_BASE  = 32'h00E0_0000;

    // cfg_regs 关键地址
    localparam [11:0] ADDR_CTRL             = 12'h000;
    localparam [11:0] ADDR_DESC_LIST_BASE   = 12'h180;
    localparam [11:0] ADDR_DESC_COUNT       = 12'h184;

    // ----------------- 时钟复位 -----------------
    logic clk = 0;  always #5 clk = ~clk;
    logic rst_n = 0;

    // ----------------- multicore_top 信号 -----------------
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

    // 每层 meta
    int l0_ifb_w, l0_wb_w, l0_ofb_w, l0_desc_count, l0_desc_beats, l0_rdma_w;
    int l1_ifb_w, l1_wb_w, l1_ofb_w, l1_desc_count, l1_desc_beats, l1_rdma_w;

    // ----------------- 解析 config.txt -----------------
    task automatic parse_meta(input string case_dir,
                               output int ifb_w, output int wb_w,
                               output int ofb_w, output int desc_cnt,
                               output int rdma_w);
        int    fd;
        string line, key;
        int    val;
        int    cnt;
        string path;
        path = $sformatf("%s/config.txt", case_dir);
        fd = $fopen(path, "r");
        if (fd == 0) begin $display("FATAL: cannot open %s", path); $stop; end
        ifb_w = 0; wb_w = 0; ofb_w = 0; desc_cnt = 0; rdma_w = 0;
        while (!$feof(fd)) begin
            void'($fgets(line, fd));
            cnt = $sscanf(line, "%s = %d", key, val);
            if (cnt < 2) continue;
            case (key)
                "_META_IFB_WORDS" : ifb_w   = val;
                "_META_WB_WORDS"  : wb_w    = val;
                "_META_OFB_WORDS" : ofb_w   = val;
                "_META_RDMA_WORDS": rdma_w  = val;
                "DESC_COUNT"      : desc_cnt = val;
                default           : ;
            endcase
        end
        $fclose(fd);
    endtask

    // ----------------- Preload one section to DDR -----------------
    task automatic preload_ifb(input string case_dir, input [31:0] base, input int n_words);
        int base_w = base / 16;
        $readmemh($sformatf("%s/ifb.txt", case_dir), ifb_arr);
        for (int i = 0; i < n_words; i++) u_ddr.mem[base_w + i] = ifb_arr[i];
    endtask

    task automatic preload_wb(input string case_dir, input [31:0] base, input int n_words);
        int base_w = base / 16;
        $readmemh($sformatf("%s/wb.txt", case_dir), wb_arr);
        for (int i = 0; i < n_words; i++)
            for (int b = 0; b < 16; b++)
                u_ddr.mem[base_w + i*16 + b] = wb_arr[i][b*BUS_DATA_W +: BUS_DATA_W];
    endtask

    task automatic preload_desc(input string case_dir, input [31:0] base, input int n_beats);
        int base_w = base / 16;
        $readmemh($sformatf("%s/desc_list.hex", case_dir), desc_arr);
        for (int i = 0; i < n_beats; i++) u_ddr.mem[base_w + i] = desc_arr[i];
    endtask

    task automatic clear_ofb(input [31:0] base, input int n_words);
        int base_w = base / 16;
        for (int i = 0; i < n_words; i++) u_ddr.mem[base_w + i] = '0;
    endtask

    logic [BUS_DATA_W-1:0] rdma_arr [0:1023];
    task automatic preload_rdma(input string case_dir, input [31:0] base, input int n_words);
        int base_w = base / 16;
        if (n_words == 0) return;
        $readmemh($sformatf("%s/rdma_data.txt", case_dir), rdma_arr);
        for (int i = 0; i < n_words; i++) u_ddr.mem[base_w + i] = rdma_arr[i];
    endtask

    // ----------------- AXI-Lite write helper -----------------
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

    // 启核: 写 boot regs + start_dfe + 等 desc 灌完 + start_layer
    task automatic start_core(input bit core_id, input [31:0] desc_base, input int desc_count);
        axi_lite_write(core_id, ADDR_DESC_LIST_BASE, desc_base);
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

    // ----------------- OFB 比对 (Layer 1, golden 在 core1/expected_ofm.txt) -----------------
    task automatic check_ofb_l1(input string l1_dir, output int mismatches);
        int ofb_base_w = DDR_L1_OFB_BASE / 16;
        logic [NUM_COL*DATA_WIDTH-1:0] expected, got;
        $readmemh($sformatf("%s/expected_ofm.txt", l1_dir), exp_arr);
        mismatches = 0;
        for (int i = 0; i < l1_ofb_w; i++) begin
            expected = exp_arr[i];
            got      = u_ddr.mem[ofb_base_w + i][NUM_COL*DATA_WIDTH-1:0];
            if (got !== expected) begin
                if (mismatches < 5)
                    $display("  L1.OFB[%0d]: expect=%h got=%h", i, expected, got);
                mismatches++;
            end
        end
    endtask

    // ----------------- 主流程 -----------------
    initial begin
        string case_root, l0_dir, l1_dir;
        int    mismatches;
        longint t_start;

        case_root = "cases/cross00";
        l0_dir    = $sformatf("%s/core0", case_root);
        l1_dir    = $sformatf("%s/core1", case_root);

        csr_awaddr  = 0; csr_awvalid = 0;
        csr_wdata   = 0; csr_wstrb   = 0; csr_wvalid = 0;
        csr_bready  = 0;
        csr_araddr  = 0; csr_arvalid = 0; csr_rready = 0;
        #20 rst_n = 1;
        #10;

        $display("== tb_multicore_xcore: cross-core 2-layer chain ==");
        $display("  case_root=%s", case_root);

        // --- 解析两层 meta ---
        parse_meta(l0_dir, l0_ifb_w, l0_wb_w, l0_ofb_w, l0_desc_count, l0_rdma_w);
        l0_desc_beats = l0_desc_count * 2;
        parse_meta(l1_dir, l1_ifb_w, l1_wb_w, l1_ofb_w, l1_desc_count, l1_rdma_w);
        l1_desc_beats = l1_desc_count * 2;
        $display("  L0: ifb=%0d wb=%0d ofb=%0d desc=%0d rdma=%0d", l0_ifb_w, l0_wb_w, l0_ofb_w, l0_desc_count, l0_rdma_w);
        $display("  L1: ifb=%0d wb=%0d ofb=%0d desc=%0d rdma=%0d", l1_ifb_w, l1_wb_w, l1_ofb_w, l1_desc_count, l1_rdma_w);

        // --- DDR preload ---
        preload_ifb (l0_dir, DDR_L0_IFB_BASE,  l0_ifb_w);
        preload_wb  (l0_dir, DDR_L0_WB_BASE,   l0_wb_w);
        preload_wb  (l1_dir, DDR_L1_WB_BASE,   l1_wb_w);
        preload_desc(l0_dir, DDR_L0_DESC_BASE, l0_desc_beats);
        preload_desc(l1_dir, DDR_L1_DESC_BASE, l1_desc_beats);
        preload_rdma(l0_dir, DDR_L0_RDMA_BASE, l0_rdma_w);   // bias/shortcut for L0
        preload_rdma(l1_dir, DDR_L1_RDMA_BASE, l1_rdma_w);   // bias/shortcut for L1
        clear_ofb   (DDR_L1_OFB_BASE, l1_ofb_w);

        $display("  DDR preload done. Starting consumer (Core 1) FIRST, then producer (Core 0)...");

        t_start = $time;

        // --- 启动顺序: consumer (Core 1) 先启 (ifb_axi_slave reset wptr/rows_pushed),
        //              然后 producer (Core 0) 启动开始 push ---
        start_core(1, DDR_L1_DESC_BASE, l1_desc_count);
        $display("  Core 1 (consumer) started @ t=%0t", $time);
        start_core(0, DDR_L0_DESC_BASE, l0_desc_count);
        $display("  Core 0 (producer) started @ t=%0t", $time);

        // 等两核 done
        wait ((done_per_core[0] == 1'b1) && (done_per_core[1] == 1'b1));
        $display("  BOTH cores done @ t=%0t (cycles=%0d)", $time, ($time - t_start) / 10);

        @(posedge clk); @(posedge clk);

        // --- 比对 Layer 1 OFB (golden) ---
        check_ofb_l1(l1_dir, mismatches);

        $display("");
        $display("============================================================");
        if (mismatches == 0)
            $display("  RESULT: PASS  (Layer 1 OFB matches golden, %0d words checked)", l1_ofb_w);
        else
            $display("  RESULT: FAIL  Layer 1 OFB mismatches=%0d / %0d", mismatches, l1_ofb_w);
        $display("  Wall time: %0d ns (%0d cycles @ 10 ns)",
                 $time - t_start, ($time - t_start)/10);
        $display("============================================================");

        $finish;
    end

    // 总 watchdog
    initial begin
        #20_000_000;
        $display("FATAL: watchdog timeout @ %0t", $time);
        $stop;
    end

endmodule
