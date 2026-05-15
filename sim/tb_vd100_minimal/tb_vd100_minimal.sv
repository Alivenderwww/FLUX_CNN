`timescale 1ns/1ps
// =============================================================================
// tb_vd100_minimal — 严格复刻 board vd100_minimal 工程的 sim
//   DUT = multicore_top_minimal_v (board RTL wrapper, 单核 N=1)
//   BRAM model = axi_slave_mem (跟 board axi_bram_ctrl + emb_mem_gen 等价)
//   driver = AXI-Lite poke + load BRAM init 跟 board host RPC 同 byte stream
//
//   case 来自 gen_case_hex.py 生成 (跑 host setup_case_for_board 同函数), 写
//   bram_init.hex (256KB BRAM init) + expected_ofm.hex (比对 golden) + params.f
//   (DESC_LIST_BASE/COUNT/OFM_BASE/BYTES).
// =============================================================================
`include "flux_cnn_params.svh"

`ifndef DESC_COUNT
  `define DESC_COUNT     63
`endif
`ifndef OFM_WORDS
  `define OFM_WORDS      64
`endif

module tb_vd100_minimal;
    localparam logic [31:0] BRAM_BASE_ADDR = 32'hA4100000;
    localparam logic [31:0] DESC_BASE      = BRAM_BASE_ADDR + 32'h00000;
    localparam logic [31:0] OFM_BASE_ADDR  = BRAM_BASE_ADDR + 32'h30000;
    localparam BRAM_DEPTH     = 16384;  // 256 KB / 16 byte

    logic clk;
    logic rst_n;

    initial clk = 0;
    always #5 clk = ~clk;   // 100 MHz

    initial begin
        rst_n = 0;
        repeat (20) @(posedge clk);
        rst_n = 1;
    end

    // AXI-Lite (csr_axil)
    logic [11:0] csr_awaddr; logic csr_awvalid, csr_awready;
    logic [31:0] csr_wdata;  logic [3:0] csr_wstrb;
    logic csr_wvalid, csr_wready;
    logic [1:0]  csr_bresp;  logic csr_bvalid, csr_bready;
    logic [11:0] csr_araddr; logic csr_arvalid, csr_arready;
    logic [31:0] csr_rdata;  logic [1:0] csr_rresp;
    logic csr_rvalid, csr_rready;

    // AXI4 master (m_axi)
    logic [3:0]   m_awid; logic [31:0] m_awaddr; logic [7:0] m_awlen;
    logic [2:0]   m_awsize; logic [1:0] m_awburst;
    logic         m_awlock; logic [3:0] m_awcache; logic [2:0] m_awprot; logic [3:0] m_awqos;
    logic         m_awvalid, m_awready;
    logic [127:0] m_wdata;  logic [15:0] m_wstrb;
    logic         m_wlast, m_wvalid, m_wready;
    logic [3:0]   m_bid;    logic [1:0]  m_bresp;
    logic         m_bvalid, m_bready;
    logic [3:0]   m_arid;   logic [31:0] m_araddr; logic [7:0] m_arlen;
    logic [2:0]   m_arsize; logic [1:0]  m_arburst;
    logic         m_arlock; logic [3:0] m_arcache; logic [2:0] m_arprot; logic [3:0] m_arqos;
    logic         m_arvalid, m_arready;
    logic [3:0]   m_rid;    logic [127:0] m_rdata;
    logic [1:0]   m_rresp;  logic m_rlast, m_rvalid, m_rready;

    // ========================================================================
    // DUT: multicore_top_minimal_v (board 用的 RTL wrapper)
    // ========================================================================
    multicore_top_minimal_v u_dut (
        .clk(clk), .rst_n(rst_n),

        .csr_axil_awaddr(csr_awaddr), .csr_axil_awvalid(csr_awvalid), .csr_axil_awready(csr_awready),
        .csr_axil_wdata(csr_wdata),   .csr_axil_wstrb(csr_wstrb),
        .csr_axil_wvalid(csr_wvalid), .csr_axil_wready(csr_wready),
        .csr_axil_bresp(csr_bresp),   .csr_axil_bvalid(csr_bvalid), .csr_axil_bready(csr_bready),
        .csr_axil_araddr(csr_araddr), .csr_axil_arvalid(csr_arvalid), .csr_axil_arready(csr_arready),
        .csr_axil_rdata(csr_rdata),   .csr_axil_rresp(csr_rresp),
        .csr_axil_rvalid(csr_rvalid), .csr_axil_rready(csr_rready),

        .m_axi_awid(m_awid), .m_axi_awaddr(m_awaddr), .m_axi_awlen(m_awlen),
        .m_axi_awsize(m_awsize), .m_axi_awburst(m_awburst), .m_axi_awlock(m_awlock),
        .m_axi_awcache(m_awcache), .m_axi_awprot(m_awprot), .m_axi_awqos(m_awqos),
        .m_axi_awvalid(m_awvalid), .m_axi_awready(m_awready),
        .m_axi_wdata(m_wdata), .m_axi_wstrb(m_wstrb),
        .m_axi_wlast(m_wlast), .m_axi_wvalid(m_wvalid), .m_axi_wready(m_wready),
        .m_axi_bid(m_bid), .m_axi_bresp(m_bresp), .m_axi_bvalid(m_bvalid), .m_axi_bready(m_bready),
        .m_axi_arid(m_arid), .m_axi_araddr(m_araddr), .m_axi_arlen(m_arlen),
        .m_axi_arsize(m_arsize), .m_axi_arburst(m_arburst), .m_axi_arlock(m_arlock),
        .m_axi_arcache(m_arcache), .m_axi_arprot(m_arprot), .m_axi_arqos(m_arqos),
        .m_axi_arvalid(m_arvalid), .m_axi_arready(m_arready),
        .m_axi_rid(m_rid), .m_axi_rdata(m_rdata),
        .m_axi_rresp(m_rresp), .m_axi_rlast(m_rlast),
        .m_axi_rvalid(m_rvalid), .m_axi_rready(m_rready)
    );

    // ========================================================================
    // BRAM model — board axi_bram_ctrl + emb_mem_gen 等价
    //   ADDR_W=32, DATA_W=128, DEPTH=16384 (= 256 KB / 16 byte)
    //   axi_slave_mem 内部 mem[BRAM_DEPTH-1:0] 用 ADDR offset (byte addr / 16)
    //   接收 byte addr, mem 内部直接按 word 索引
    // ========================================================================
    // axi_slave_mem 期望 byte addr 转 word offset = ADDR >> 4. 但它内部用
    // ADDR[ADDR_W-1:0] 直接当 mem index, 所以需要把 m_axi awaddr/araddr 偏到
    // BRAM_BASE 起点然后 >> 4. 用 wrapper 信号。
    logic [31:0] mem_awaddr_byte, mem_araddr_byte;
    assign mem_awaddr_byte = m_awaddr - BRAM_BASE_ADDR;   // byte offset 0..256KB
    assign mem_araddr_byte = m_araddr - BRAM_BASE_ADDR;

    axi_slave_mem #(
        .ADDR_W(32), .DATA_W(128), .ID_W(4), .DEPTH(BRAM_DEPTH)
    ) u_bram (
        .clk(clk), .rstn(rst_n),
        .AWID(m_awid), .AWADDR(mem_awaddr_byte), .AWLEN(m_awlen), .AWBURST(m_awburst),
        .AWVALID(m_awvalid), .AWREADY(m_awready),
        .WDATA(m_wdata), .WSTRB(m_wstrb), .WLAST(m_wlast),
        .WVALID(m_wvalid), .WREADY(m_wready),
        .BID(m_bid), .BRESP(m_bresp), .BVALID(m_bvalid), .BREADY(m_bready),
        .ARID(m_arid), .ARADDR(mem_araddr_byte), .ARLEN(m_arlen), .ARBURST(m_arburst),
        .ARVALID(m_arvalid), .ARREADY(m_arready),
        .RID(m_rid), .RDATA(m_rdata), .RRESP(m_rresp), .RLAST(m_rlast),
        .RVALID(m_rvalid), .RREADY(m_rready)
    );

    // ========================================================================
    // BRAM init: load bram_init.hex 进 u_bram.mem (backdoor, 跟 board host
    // load_ddr 等价)
    // ========================================================================
    initial begin
        @(posedge rst_n);
        @(posedge clk);
        $readmemh("bram_init.hex", u_bram.mem);
        $display("[TB] BRAM init loaded from bram_init.hex");
    end

    // ========================================================================
    // AXI-Lite driver task — 跟 board host RPC poke_csr 一致
    // ========================================================================
    task automatic poke_csr(input [11:0] addr, input [31:0] data);
        @(posedge clk);
        csr_awaddr  <= addr; csr_awvalid <= 1'b1;
        csr_wdata   <= data; csr_wstrb   <= 4'hF; csr_wvalid <= 1'b1;
        csr_bready  <= 1'b1;
        @(posedge clk);
        while (!csr_awready || !csr_wready) @(posedge clk);
        csr_awvalid <= 1'b0; csr_wvalid <= 1'b0;
        while (!csr_bvalid) @(posedge clk);
        @(posedge clk);
        csr_bready  <= 1'b0;
    endtask

    task automatic peek_csr(input [11:0] addr, output [31:0] data);
        @(posedge clk);
        csr_araddr  <= addr; csr_arvalid <= 1'b1; csr_rready <= 1'b1;
        @(posedge clk);
        while (!csr_arready) @(posedge clk);
        csr_arvalid <= 1'b0;
        while (!csr_rvalid) @(posedge clk);
        data = csr_rdata;
        @(posedge clk);
        csr_rready  <= 1'b0;
    endtask

    // ========================================================================
    // Main test
    // ========================================================================
    integer mismatches;
    logic [127:0] expected_ofm [(`OFM_WORDS)-1:0];
    logic [31:0]  status_reg, seq_dbg;
    integer i, timeout_us;

    initial begin
        // init signals
        csr_awaddr=0; csr_awvalid=0; csr_wdata=0; csr_wstrb=0; csr_wvalid=0; csr_bready=0;
        csr_araddr=0; csr_arvalid=0; csr_rready=0;

        $readmemh("expected_ofm.hex", expected_ofm);

        // Wait reset done + BRAM loaded
        @(posedge rst_n);
        repeat (10) @(posedge clk);

        $display("[TB t=%0t] Test start: DESC_LIST_BASE=0x%h DESC_COUNT=%0d",
                 $time, `DESC_LIST_BASE, `DESC_COUNT);

        // 1. poke DESC_LIST_BASE / DESC_COUNT
        poke_csr(12'h180, `DESC_LIST_BASE);
        poke_csr(12'h184, `DESC_COUNT);

        // 2. start_dfe (CTRL bit 4)
        poke_csr(12'h000, 32'h10);

        // 3. wait dfe_done (STATUS bit 9)
        timeout_us = 0;
        forever begin
            peek_csr(12'h004, status_reg);
            if (status_reg[9]) begin
                $display("[TB t=%0t] dfe_done STATUS=0x%h", $time, status_reg);
                break;
            end
            if (timeout_us > 100000) begin
                $display("[TB t=%0t] TIMEOUT dfe_done STATUS=0x%h", $time, status_reg);
                $finish;
            end
            timeout_us++;
            repeat (100) @(posedge clk);
        end

        // 4. start_layer (CTRL bit 5)
        poke_csr(12'h000, 32'h20);

        // 5. wait layer_done (STATUS bit 11)
        timeout_us = 0;
        forever begin
            peek_csr(12'h004, status_reg);
            if (status_reg[11]) begin
                $display("[TB t=%0t] layer_done STATUS=0x%h", $time, status_reg);
                break;
            end
            // 每 1000 us peek 一次 SEQ_DBG (跟 board 一致)
            if (timeout_us % 100 == 0 && timeout_us > 0) begin
                peek_csr(12'h008, seq_dbg);
                $display("[TB t=%0t monitor] STATUS=0x%h SEQ_DBG=0x%h  IDMA cmd_idx=%0d rows_pushed=%0d  ODMA cmd_idx=%0d rows_drained=%0d  ofb rows_written=%0d ring_full=%b",
                         $time, status_reg, seq_dbg,
                         u_dut.u_mc_minimal_sv.u_core.g_idma_sg.u_idma_sg.r_cmd_idx,
                         u_dut.u_mc_minimal_sv.u_core.g_idma_sg.u_idma_sg.r_rows_pushed,
                         u_dut.u_mc_minimal_sv.u_core.g_odma_sg.u_odma_sg.r_cmd_idx,
                         u_dut.u_mc_minimal_sv.u_core.g_odma_sg.u_odma_sg.r_rows_drained,
                         u_dut.u_mc_minimal_sv.u_core.u_ofb_writer.rows_written,
                         u_dut.u_mc_minimal_sv.u_core.u_ofb_writer.ring_full_d);
            end
            if (timeout_us > 50000) begin   // 5 s @ 100 MHz
                peek_csr(12'h008, seq_dbg);
                $display("[TB t=%0t] LAYER STUCK STATUS=0x%h SEQ_DBG=0x%h", $time, status_reg, seq_dbg);
                // Hier signal dump (deadlock root cause analysis)
                $display("  === IDMA SG dispatcher ===");
                $display("    st=%0d  r_cmd_idx=%0d / cmd_count=%0d",
                    u_dut.u_mc_minimal_sv.u_core.g_idma_sg.u_idma_sg.st,
                    u_dut.u_mc_minimal_sv.u_core.g_idma_sg.u_idma_sg.r_cmd_idx,
                    u_dut.u_mc_minimal_sv.u_core.g_idma_sg.u_idma_sg.cfg_cmd_count);
                $display("    r_rows_pushed=%0d  rows_consumed=%0d  ring_diff=%0d  ring_strip_rows=%0d",
                    u_dut.u_mc_minimal_sv.u_core.g_idma_sg.u_idma_sg.r_rows_pushed,
                    u_dut.u_mc_minimal_sv.u_core.g_idma_sg.u_idma_sg.rows_consumed,
                    u_dut.u_mc_minimal_sv.u_core.g_idma_sg.u_idma_sg.rows_diff,
                    u_dut.u_mc_minimal_sv.u_core.g_idma_sg.u_idma_sg.cfg_ifb_strip_rows);
                $display("  === ODMA SG dispatcher ===");
                $display("    state=%0d  r_cmd_idx=%0d / cmd_count=%0d  r_cmds_done=%0d  r_rows_drained=%0d",
                    u_dut.u_mc_minimal_sv.u_core.g_odma_sg.u_odma_sg.state,
                    u_dut.u_mc_minimal_sv.u_core.g_odma_sg.u_odma_sg.r_cmd_idx,
                    u_dut.u_mc_minimal_sv.u_core.g_odma_sg.u_odma_sg.cfg_cmd_count,
                    u_dut.u_mc_minimal_sv.u_core.g_odma_sg.u_odma_sg.r_cmds_done,
                    u_dut.u_mc_minimal_sv.u_core.g_odma_sg.u_odma_sg.r_rows_drained);
                $display("    r_rows_consumed_from_writer=%0d  has_writer_data_ready=%b",
                    u_dut.u_mc_minimal_sv.u_core.g_odma_sg.u_odma_sg.r_rows_consumed_from_writer,
                    u_dut.u_mc_minimal_sv.u_core.g_odma_sg.u_odma_sg.has_writer_data_ready);
                $display("  === ofb_writer ===");
                $display("    rows_written=%0d  rows_drained=%0d  ring_full_d=%b  acc_out_ready=%b",
                    u_dut.u_mc_minimal_sv.u_core.u_ofb_writer.rows_written,
                    u_dut.u_mc_minimal_sv.u_core.u_ofb_writer.rows_drained,
                    u_dut.u_mc_minimal_sv.u_core.u_ofb_writer.ring_full_d,
                    u_dut.u_mc_minimal_sv.u_core.u_ofb_writer.acc_out_ready);
                $display("    cfg_ofb_strip_rows=%0d  cfg_ofb_ring_words=%0d  cfg_ofb_row_words=%0d",
                    u_dut.u_mc_minimal_sv.u_core.u_ofb_writer.cfg_ofb_strip_rows,
                    u_dut.u_mc_minimal_sv.u_core.u_ofb_writer.cfg_ofb_ring_words,
                    u_dut.u_mc_minimal_sv.u_core.u_ofb_writer.cfg_ofb_row_words);
                $display("  === line_buffer ===");
                $display("    rows_consumed=%0d  rows_available=%0d  streaming_rows_ready=%b",
                    u_dut.u_mc_minimal_sv.u_core.u_line_buffer.rows_consumed,
                    u_dut.u_mc_minimal_sv.u_core.u_line_buffer.rows_available,
                    u_dut.u_mc_minimal_sv.u_core.u_line_buffer.streaming_rows_ready);
                $display("  === mm2s_arb ===");
                $display("    cmd_owner=%0d  data_cnt=%0d  data_head=%0d  data_empty=%b  data_full=%b",
                    u_dut.u_mc_minimal_sv.u_core.u_mm2s_arb.cmd_owner,
                    u_dut.u_mc_minimal_sv.u_core.u_mm2s_arb.data_cnt,
                    u_dut.u_mc_minimal_sv.u_core.u_mm2s_arb.data_head,
                    u_dut.u_mc_minimal_sv.u_core.u_mm2s_arb.data_empty,
                    u_dut.u_mc_minimal_sv.u_core.u_mm2s_arb.data_full);
                $display("    idma_cmd_tvalid=%b idma_cmd_tready=%b",
                    u_dut.u_mc_minimal_sv.u_core.u_mm2s_arb.idma_cmd_tvalid,
                    u_dut.u_mc_minimal_sv.u_core.u_mm2s_arb.idma_cmd_tready);
                $display("    ocmd_cmd_tvalid=%b ocmd_cmd_tready=%b ocmd_starve=%b ocmd_wait_cnt=%0d",
                    u_dut.u_mc_minimal_sv.u_core.u_mm2s_arb.ocmd_cmd_tvalid,
                    u_dut.u_mc_minimal_sv.u_core.u_mm2s_arb.ocmd_cmd_tready,
                    u_dut.u_mc_minimal_sv.u_core.u_mm2s_arb.ocmd_starve,
                    u_dut.u_mc_minimal_sv.u_core.u_mm2s_arb.r_ocmd_wait_cnt);
                $display("    mm2s_data_tvalid=%b mm2s_data_tready=%b idma_data_tvalid=%b idma_data_tready=%b ocmd_data_tvalid=%b ocmd_data_tready=%b",
                    u_dut.u_mc_minimal_sv.u_core.u_mm2s_arb.mm2s_data_tvalid,
                    u_dut.u_mc_minimal_sv.u_core.u_mm2s_arb.mm2s_data_tready,
                    u_dut.u_mc_minimal_sv.u_core.u_mm2s_arb.idma_data_tvalid,
                    u_dut.u_mc_minimal_sv.u_core.u_mm2s_arb.idma_data_tready,
                    u_dut.u_mc_minimal_sv.u_core.u_mm2s_arb.ocmd_data_tvalid,
                    u_dut.u_mc_minimal_sv.u_core.u_mm2s_arb.ocmd_data_tready);
                $display("  === IDMA SG internals ===");
                $display("    st=%0d r_btt=%0d cap_btt=%0d mm2s_data_tlast=%b ifb_we=%b",
                    u_dut.u_mc_minimal_sv.u_core.g_idma_sg.u_idma_sg.st,
                    u_dut.u_mc_minimal_sv.u_core.g_idma_sg.u_idma_sg.r_btt,
                    u_dut.u_mc_minimal_sv.u_core.g_idma_sg.u_idma_sg.cap_btt,
                    u_dut.u_mc_minimal_sv.u_core.g_idma_sg.u_idma_sg.mm2s_data_tlast,
                    u_dut.u_mc_minimal_sv.u_core.g_idma_sg.u_idma_sg.ifb_we);
                $finish;
            end
            timeout_us++;
            repeat (100) @(posedge clk);
        end

        // 6. 比对 OFM
        mismatches = 0;
        for (i = 0; i < `OFM_WORDS; i++) begin
            logic [127:0] got, exp_w;
            // u_bram.mem 索引 byte_off / 16. OFM_BASE - BRAM_BASE = 0x30000 / 16 = 0x3000
            got = u_bram.mem[((`OFM_BASE - BRAM_BASE_ADDR) >> 4) + i];
            exp_w = expected_ofm[i];
            if (got !== exp_w) begin
                if (mismatches < 5)
                    $display("[FAIL] word %0d: got=0x%032h  exp=0x%032h", i, got, exp_w);
                mismatches++;
            end
        end

        if (mismatches == 0)
            $display("\n=== PASS  All %0d words bit-exact ===\n", `OFM_WORDS);
        else
            $display("\n=== FAIL  Mismatches=%0d / %0d words ===\n", mismatches, `OFM_WORDS);

        $finish;
    end

endmodule
