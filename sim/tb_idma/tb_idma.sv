`timescale 1ns/1ps

// =============================================================================
// tb_idma.sv  --  Smoke test for IDMA (DDR → IFB)
//
// 拓扑：
//   axi_slave_mem (fake DDR, 8K word × 128-bit = 128KB)
//          │
//          │ AXI4 M (AR/R)
//          ▼
//        idma
//          │ ifb_we / waddr / wdata
//          ▼
//   sram_model (fake IFB, 8K word × 128-bit)
//
// 测试：
//   T1 small: 256 B（16 beat，1 burst，LEN=15）
//   T2 med  : 4096 B（256 beat，1 burst，LEN=255）
//   T3 large: 8192 B（512 beat，2 bursts）
//   每次预填 DDR 已知 pattern，IDMA 完成后对比 IFB 内容
// =============================================================================

module tb_idma;
    localparam int ADDR_W      = 32;
    localparam int DATA_W      = 128;
    localparam int M_ID        = 2;
    localparam int SRAM_DEPTH  = 8192;
    localparam int SRAM_ADDR_W = $clog2(SRAM_DEPTH);
    localparam int LEN_W       = 24;

    logic clk  = 0;
    logic rst_n = 0;
    always #5 clk = ~clk;

    // Control
    logic                 start;
    logic                 done;
    logic                 busy;
    logic [ADDR_W-1:0]    src_base;
    logic [LEN_W-1:0]     byte_len;

    // AXI M bus (idma → axi_slave_mem)
    logic [M_ID-1:0]      AWID, BID, ARID, RID;
    logic [ADDR_W-1:0]    AWADDR, ARADDR;
    logic [7:0]           AWLEN, ARLEN;
    logic [1:0]           AWBURST, ARBURST, BRESP, RRESP;
    logic                 AWVALID, AWREADY, WVALID, WREADY, WLAST;
    logic                 BVALID, BREADY, ARVALID, ARREADY;
    logic                 RVALID, RREADY, RLAST;
    logic [DATA_W-1:0]    WDATA, RDATA;
    logic [DATA_W/8-1:0]  WSTRB;

    // IFB SRAM write port
    logic                    ifb_we;
    logic [SRAM_ADDR_W-1:0]  ifb_waddr;
    logic [DATA_W-1:0]       ifb_wdata;
    logic [DATA_W-1:0]       ifb_rdata_dummy;

    idma #(
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .M_ID(M_ID),
        .SRAM_ADDR_W(SRAM_ADDR_W), .LEN_W(LEN_W)
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
        .ifb_we(ifb_we), .ifb_waddr(ifb_waddr), .ifb_wdata(ifb_wdata)
    );

    axi_slave_mem #(
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .ID_W(M_ID), .DEPTH(SRAM_DEPTH)
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

    sram_model #(.DEPTH(SRAM_DEPTH), .DATA_WIDTH(DATA_W)) u_ifb (
        .clk(clk),
        .we(ifb_we), .waddr(ifb_waddr), .wdata(ifb_wdata),
        .re(1'b0), .raddr('0), .rdata(ifb_rdata_dummy)
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

    task automatic clear_ifb();
        for (int i = 0; i < SRAM_DEPTH; i++) u_ifb.mem[i] = '0;
    endtask

    task automatic run_test(input string label,
                             input [ADDR_W-1:0] addr,
                             input [LEN_W-1:0] len,
                             input [31:0] seed);
        automatic int n_beats = len / (DATA_W/8);
        automatic int addr_word = addr / (DATA_W/8);
        automatic logic [DATA_W-1:0] expect_val, got_val;

        $display("=== %s : addr=0x%08h len=%0d bytes (%0d beats) ===", label, addr, len, n_beats);

        fill_ddr_pattern(addr_word, n_beats, seed);
        clear_ifb();
        $display("  [pre] u_ddr.mem[%0d]=%h  u_ifb.mem[0]=%h", addr_word, u_ddr.mem[addr_word], u_ifb.mem[0]);

        src_base <= addr;
        byte_len <= len;
        @(posedge clk);
        start    <= 1'b1;
        @(posedge clk);
        start    <= 1'b0;

        // 等 busy 升起再落下；用 busy 跳变比 wait(done) 安全（done 是锁存、
        // 可能从上一次测试残留，start 清零 NBA 晚于 wait 的 Active 采样）
        wait (busy == 1'b1);
        wait (busy == 1'b0);
        @(posedge clk);
        $display("  [post] u_ifb.mem[0]=%h  u_ifb.mem[%0d]=%h  idma.wr_ptr=%0d  beats_remaining=%0d",
                 u_ifb.mem[0], n_beats-1, u_ifb.mem[n_beats-1], dut.wr_ptr, dut.beats_remaining);

        mismatch_cnt = 0;
        for (int i = 0; i < n_beats; i++) begin
            expect_val = {96'h0, seed + i};
            got_val    = u_ifb.mem[i];
            if (got_val !== expect_val) begin
                if (mismatch_cnt < 5)
                    $display("  FAIL beat %0d: expect=%h got=%h", i, expect_val, got_val);
                mismatch_cnt++;
            end
        end
        if (mismatch_cnt == 0) $display("  PASS  (%0d beats)", n_beats);
        else                   $display("  FAIL  (%0d mismatches of %0d beats)", mismatch_cnt, n_beats);
        total_mismatches += mismatch_cnt;
    endtask

    initial begin
        start = 0; src_base = 0; byte_len = 0;
        total_mismatches = 0;

        #20 rst_n = 1;
        #10;

        run_test("T1 small (1 burst, 16 beats)",   32'h0000_0000, 24'd256,  32'h1000_0000);
        run_test("T2 medium (1 burst, 256 beats)", 32'h0000_1000, 24'd4096, 32'h2000_0000);
        run_test("T3 large (2 bursts, 512 beats)", 32'h0000_3000, 24'd8192, 32'h3000_0000);

        if (total_mismatches == 0) $display("SMOKE TEST PASSED. All transfers OK.");
        else                       $display("SMOKE TEST FAILED. %0d total mismatches.", total_mismatches);
        $stop;
    end

    initial begin
        #5_000_000;
        $display("FATAL: simulation timeout");
        $stop;
    end

endmodule
