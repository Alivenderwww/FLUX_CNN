`timescale 1ns/1ps

// =============================================================================
// tb_odma.sv  --  Smoke test for ODMA (OFB SRAM → DDR)
//
// 拓扑：
//   sram_model (fake OFB)
//       │ ofb_re / raddr / rdata
//       ▼
//      odma
//       │ AXI4 M (AW/W/B)
//       ▼
//   axi_slave_mem (fake DDR)
//
// 每个 test case：预填 OFB 已知 pattern、触发 ODMA、等 busy 0→1→0、读 DDR
// mem 校验内容。
// =============================================================================

module tb_odma;
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
    logic [ADDR_W-1:0]    dst_base;
    logic [LEN_W-1:0]     byte_len;

    // AXI M bus (odma → axi_slave_mem)
    logic [M_ID-1:0]      AWID, BID, ARID, RID;
    logic [ADDR_W-1:0]    AWADDR, ARADDR;
    logic [7:0]           AWLEN, ARLEN;
    logic [1:0]           AWBURST, ARBURST, BRESP, RRESP;
    logic                 AWVALID, AWREADY, WVALID, WREADY, WLAST;
    logic                 BVALID, BREADY, ARVALID, ARREADY;
    logic                 RVALID, RREADY, RLAST;
    logic [DATA_W-1:0]    WDATA, RDATA;
    logic [DATA_W/8-1:0]  WSTRB;

    // OFB SRAM read port
    logic                    ofb_re;
    logic [SRAM_ADDR_W-1:0]  ofb_raddr;
    logic [DATA_W-1:0]       ofb_rdata;

    odma #(
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .M_ID(M_ID),
        .SRAM_ADDR_W(SRAM_ADDR_W), .LEN_W(LEN_W)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .start(start), .done(done), .busy(busy),
        .dst_base(dst_base), .byte_len(byte_len),
        .M_AWID(AWID), .M_AWADDR(AWADDR), .M_AWLEN(AWLEN), .M_AWBURST(AWBURST),
        .M_AWVALID(AWVALID), .M_AWREADY(AWREADY),
        .M_WDATA(WDATA), .M_WSTRB(WSTRB), .M_WLAST(WLAST),
        .M_WVALID(WVALID), .M_WREADY(WREADY),
        .M_BID(BID), .M_BRESP(BRESP), .M_BVALID(BVALID), .M_BREADY(BREADY),
        .M_ARID(ARID), .M_ARADDR(ARADDR), .M_ARLEN(ARLEN), .M_ARBURST(ARBURST),
        .M_ARVALID(ARVALID), .M_ARREADY(ARREADY),
        .M_RID(RID), .M_RDATA(RDATA), .M_RRESP(RRESP), .M_RLAST(RLAST),
        .M_RVALID(RVALID), .M_RREADY(RREADY),
        .ofb_re(ofb_re), .ofb_raddr(ofb_raddr), .ofb_rdata(ofb_rdata)
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

    sram_model #(.DEPTH(SRAM_DEPTH), .DATA_WIDTH(DATA_W)) u_ofb (
        .clk(clk),
        .we(1'b0), .waddr('0), .wdata('0),
        .re(ofb_re), .raddr(ofb_raddr), .rdata(ofb_rdata)
    );

    // =========================================================================
    // 测试逻辑
    // =========================================================================
    int mismatch_cnt;
    int total_mismatches;

    task automatic fill_ofb_pattern(input int n_beats, input [31:0] seed);
        for (int i = 0; i < n_beats; i++) u_ofb.mem[i] = {96'h0, seed + i};
    endtask

    task automatic clear_ddr();
        for (int i = 0; i < SRAM_DEPTH; i++) u_ddr.mem[i] = '0;
    endtask

    task automatic run_test(input string label,
                             input [ADDR_W-1:0] addr,
                             input [LEN_W-1:0] len,
                             input [31:0] seed);
        automatic int n_beats = len / (DATA_W/8);
        automatic int addr_word = addr / (DATA_W/8);
        automatic logic [DATA_W-1:0] expect_val, got_val;

        $display("=== %s : addr=0x%08h len=%0d bytes (%0d beats) ===", label, addr, len, n_beats);

        fill_ofb_pattern(n_beats, seed);
        clear_ddr();
        $display("  [pre] u_ofb.mem[0]=%h  u_ddr.mem[%0d]=%h", u_ofb.mem[0], addr_word, u_ddr.mem[addr_word]);

        dst_base <= addr;
        byte_len <= len;
        @(posedge clk);
        start    <= 1'b1;
        @(posedge clk);
        start    <= 1'b0;

        // busy 跳变监测（参考 IDMA 的 debug 教训）
        wait (busy == 1'b1);
        wait (busy == 1'b0);
        @(posedge clk);
        $display("  [post] u_ddr.mem[%0d]=%h  u_ddr.mem[%0d]=%h  rd_ptr=%0d",
                 addr_word, u_ddr.mem[addr_word], addr_word+n_beats-1, u_ddr.mem[addr_word+n_beats-1], dut.rd_ptr);

        mismatch_cnt = 0;
        for (int i = 0; i < n_beats; i++) begin
            expect_val = {96'h0, seed + i};
            got_val    = u_ddr.mem[addr_word + i];
            if (got_val !== expect_val) begin
                if (mismatch_cnt < 5)
                    $display("  FAIL beat %0d (DDR addr_word %0d): expect=%h got=%h",
                             i, addr_word+i, expect_val, got_val);
                mismatch_cnt++;
            end
        end
        if (mismatch_cnt == 0) $display("  PASS  (%0d beats)", n_beats);
        else                   $display("  FAIL  (%0d mismatches of %0d beats)", mismatch_cnt, n_beats);
        total_mismatches += mismatch_cnt;
    endtask

    initial begin
        start = 0; dst_base = 0; byte_len = 0;
        total_mismatches = 0;

        #20 rst_n = 1;
        #10;

        run_test("T1 small (1 burst, 16 beats)",   32'h0000_0000, 24'd256,  32'h4000_0000);
        run_test("T2 medium (1 burst, 256 beats)", 32'h0000_1000, 24'd4096, 32'h5000_0000);
        run_test("T3 large (2 bursts, 512 beats)", 32'h0000_3000, 24'd8192, 32'h6000_0000);

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
