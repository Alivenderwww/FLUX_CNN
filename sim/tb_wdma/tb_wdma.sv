`timescale 1ns/1ps

// =============================================================================
// tb_wdma.sv  --  Smoke test for WDMA (DDR → WB 128→2048 打包)
//
// 拓扑：
//   axi_slave_mem (fake DDR, 128-bit word)
//         │ AXI4 M (AR/R)
//         ▼
//       wdma
//         │ wb_we / waddr / wdata (2048-bit)
//         ▼
//   sram_model (fake WB, 2048-bit word)
//
// 每个测试：DDR 预填 128-bit pattern（beat i 的数据 = seed + i），WDMA 搬运。
// 校验 WB[k] == {ddr[k*16+15], ddr[k*16+14], ..., ddr[k*16+0]}。
// =============================================================================

module tb_wdma;
    localparam int ADDR_W       = 32;
    localparam int DATA_W       = 128;
    localparam int WB_DATA_W    = 2048;
    localparam int M_ID         = 2;
    localparam int DDR_DEPTH    = 8192;    // 128-bit words
    localparam int WB_DEPTH     = 1024;    // 2048-bit words
    localparam int DDR_ADDR_W   = $clog2(DDR_DEPTH);
    localparam int WB_ADDR_W    = $clog2(WB_DEPTH);
    localparam int LEN_W        = 24;
    localparam int BEATS_PER_WORD = WB_DATA_W / DATA_W;  // 16

    logic clk  = 0;
    logic rst_n = 0;
    always #5 clk = ~clk;

    // Control
    logic                 start;
    logic                 done;
    logic                 busy;
    logic [ADDR_W-1:0]    src_base;
    logic [LEN_W-1:0]     byte_len;

    // AXI M
    logic [M_ID-1:0]      AWID, BID, ARID, RID;
    logic [ADDR_W-1:0]    AWADDR, ARADDR;
    logic [7:0]           AWLEN, ARLEN;
    logic [1:0]           AWBURST, ARBURST, BRESP, RRESP;
    logic                 AWVALID, AWREADY, WVALID, WREADY, WLAST;
    logic                 BVALID, BREADY, ARVALID, ARREADY;
    logic                 RVALID, RREADY, RLAST;
    logic [DATA_W-1:0]    WDATA, RDATA;
    logic [DATA_W/8-1:0]  WSTRB;

    // WB SRAM write port
    logic                   wb_we;
    logic [WB_ADDR_W-1:0]   wb_waddr;
    logic [WB_DATA_W-1:0]   wb_wdata;
    logic [WB_DATA_W-1:0]   wb_rdata_dummy;

    wdma #(
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .WB_DATA_W(WB_DATA_W), .M_ID(M_ID),
        .SRAM_ADDR_W(WB_ADDR_W), .LEN_W(LEN_W)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .start(start), .done(done), .busy(busy),
        .src_base(src_base), .byte_len(byte_len),
        .M_AWID(AWID), .M_AWADDR(AWADDR), .M_AWLEN(AWLEN), .M_AWBURST(AWBURST),
        .M_AWVALID(AWVALID), .M_AWREADY(AWREADY),
        .M_WDATA(WDATA), .M_WSTRB(WSTRB), .M_WLAST(WLAST),
        .M_WVALID(WVALID), .M_WREADY(WREADY),
        .M_BID(BID), .M_BRESP(BRESP), .M_BVALID(BVALID), .M_BREADY(BREADY),
        .M_ARID(ARID), .M_ARADDR(ARADDR), .M_ARLEN(ARLEN), .M_ARBURST(ARBURST),
        .M_ARVALID(ARVALID), .M_ARREADY(ARREADY),
        .M_RID(RID), .M_RDATA(RDATA), .M_RRESP(RRESP), .M_RLAST(RLAST),
        .M_RVALID(RVALID), .M_RREADY(RREADY),
        .wb_we(wb_we), .wb_waddr(wb_waddr), .wb_wdata(wb_wdata)
    );

    axi_slave_mem #(
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .ID_W(M_ID), .DEPTH(DDR_DEPTH)
    ) u_ddr (
        .clk(clk), .rstn(rst_n),
        .AWID(AWID), .AWADDR(AWADDR), .AWLEN(AWLEN), .AWBURST(AWBURST),
        .AWVALID(AWVALID), .AWREADY(AWREADY),
        .WDATA(WDATA), .WSTRB(WSTRB), .WLAST(WLAST),
        .WVALID(WVALID), .WREADY(WREADY),
        .BID(BID), .BRESP(BRESP), .BVALID(BVALID), .BREADY(BREADY),
        .ARID(ARID), .ARADDR(ARADDR), .ARLEN(ARLEN), .ARBURST(ARBURST),
        .ARVALID(ARVALID), .ARREADY(ARREADY),
        .RID(RID), .RDATA(RDATA), .RRESP(RRESP), .RLAST(RLAST),
        .RVALID(RVALID), .RREADY(RREADY)
    );

    sram_model #(.DEPTH(WB_DEPTH), .DATA_WIDTH(WB_DATA_W)) u_wb (
        .clk(clk),
        .we(wb_we), .waddr(wb_waddr), .wdata(wb_wdata),
        .re(1'b0), .raddr('0), .rdata(wb_rdata_dummy)
    );

    // =========================================================================
    // 测试逻辑
    // =========================================================================
    int mismatch_cnt;
    int total_mismatches;

    task automatic fill_ddr_pattern(input int addr_word_start, input int n_beats, input [31:0] seed);
        for (int i = 0; i < n_beats; i++)
            u_ddr.mem[addr_word_start + i] = {96'h0, seed + i};
    endtask

    task automatic clear_wb();
        for (int i = 0; i < WB_DEPTH; i++) u_wb.mem[i] = '0;
    endtask

    task automatic run_test(input string label,
                             input [ADDR_W-1:0] addr,
                             input [LEN_W-1:0] len,
                             input [31:0] seed);
        automatic int n_beats = len / (DATA_W/8);
        automatic int n_wb_words = n_beats / BEATS_PER_WORD;
        automatic int addr_word = addr / (DATA_W/8);
        automatic logic [WB_DATA_W-1:0] expect_word, got_word;

        $display("=== %s : addr=0x%08h len=%0d bytes (%0d beats, %0d WB words) ===",
                 label, addr, len, n_beats, n_wb_words);

        fill_ddr_pattern(addr_word, n_beats, seed);
        clear_wb();
        $display("  [pre] u_ddr.mem[%0d]=%h  u_wb.mem[0][127:0]=%h",
                 addr_word, u_ddr.mem[addr_word], u_wb.mem[0][127:0]);

        src_base <= addr;
        byte_len <= len;
        @(posedge clk);
        start    <= 1'b1;
        @(posedge clk);
        start    <= 1'b0;

        wait (busy == 1'b1);
        wait (busy == 1'b0);
        @(posedge clk);

        mismatch_cnt = 0;
        for (int k = 0; k < n_wb_words; k++) begin
            // 期望：WB[k] 低 128-bit = DDR beat k*16+0，MSB 128-bit = DDR beat k*16+15
            expect_word = '0;
            for (int b = 0; b < BEATS_PER_WORD; b++) begin
                expect_word[b*DATA_W +: DATA_W] = {96'h0, seed + (k*BEATS_PER_WORD) + b};
            end
            got_word = u_wb.mem[k];
            if (got_word !== expect_word) begin
                if (mismatch_cnt < 3)
                    $display("  FAIL WB word %0d:\n    expect[127:0]=%h ... expect[2047:1920]=%h\n    got   [127:0]=%h ... got   [2047:1920]=%h",
                             k,
                             expect_word[127:0],
                             expect_word[2047:1920],
                             got_word[127:0],
                             got_word[2047:1920]);
                mismatch_cnt++;
            end
        end
        if (mismatch_cnt == 0) $display("  PASS  (%0d WB words)", n_wb_words);
        else                   $display("  FAIL  (%0d mismatches of %0d words)", mismatch_cnt, n_wb_words);
        total_mismatches += mismatch_cnt;
    endtask

    initial begin
        start = 0; src_base = 0; byte_len = 0;
        total_mismatches = 0;

        #20 rst_n = 1;
        #10;

        // 每次 len 必须是 256 字节倍数（1 个 WB 字）
        run_test("T1 small (1 WB word, 16 beats)",    32'h0000_0000, 24'd256,   32'h7000_0000);
        run_test("T2 medium (16 WB words, 256 beats)",32'h0000_1000, 24'd4096,  32'h8000_0000);
        run_test("T3 large (32 WB words, 512 beats)", 32'h0000_3000, 24'd8192,  32'h9000_0000);

        if (total_mismatches == 0) $display("SMOKE TEST PASSED.");
        else                       $display("SMOKE TEST FAILED. %0d total mismatches.", total_mismatches);
        $stop;
    end

    initial begin
        #5_000_000;
        $display("FATAL: timeout");
        $stop;
    end

endmodule
