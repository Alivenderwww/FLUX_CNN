`timescale 1ns/1ps

// =============================================================================
// tb_mesh_to_ifb.sv  --  端到端: AXIS packet → axis_to_axi_writer → ifb_axi_slave → IFB SRAM
//
// 验证 Phase 6 数据链路: mesh packet 通过 axi 桥写入真 ConvCore IFB SRAM (M2 路径).
//
// 拓扑:
//   axis source (TB) → axis_to_axi_writer (我们写的桥)
//                    → ifb_axi_slave (project 现有 RTL, M2 push 已用)
//                    → sram_model (project 现有 RTL, IFB SRAM mock)
//
// 测试: TB 发 1 个 packet (16 word), 检查 SRAM[0..15] 是否对.
// =============================================================================

module tb_mesh_to_ifb;
    timeunit 1ns; timeprecision 1ps;

    localparam int DATA_W   = 128;
    localparam int DEST_W   = 8;
    localparam int ADDR_W   = 32;
    localparam int ID_W     = 6;
    localparam int SRAM_AW  = 13;     // log2(8192)
    localparam int SRAM_DEPTH = 8192;

    logic clk = 0; always #5 clk = ~clk;
    logic rst_n = 0;

    // ----- TB → writer (AXIS) -----
    logic                s_tvalid;
    logic                s_tready;
    logic [DATA_W-1:0]   s_tdata;
    logic                s_tlast;
    logic [DEST_W-1:0]   s_tdest = 0;

    // ----- writer → ifb_axi_slave (AXI4) -----
    logic [ID_W-1:0]     awid;
    logic [ADDR_W-1:0]   awaddr;
    logic [7:0]          awlen;
    logic [2:0]          awsize;
    logic [1:0]          awburst;
    logic                awlock;
    logic [3:0]          awcache;
    logic [2:0]          awprot;
    logic [3:0]          awqos;
    logic                awvalid;
    logic                awready;
    logic [DATA_W-1:0]   wdata;
    logic [DATA_W/8-1:0] wstrb;
    logic                wlast;
    logic                wvalid;
    logic                wready;
    logic [ID_W-1:0]     bid;
    logic [1:0]          bresp;
    logic                bvalid;
    logic                bready;

    axis_to_axi_writer #(
        .DATA_W(DATA_W), .DEST_W(DEST_W), .ADDR_W(ADDR_W), .ID_W(ID_W)
    ) u_writer (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tvalid(s_tvalid), .s_axis_tready(s_tready),
        .s_axis_tdata(s_tdata),   .s_axis_tlast(s_tlast),
        .s_axis_tdest(s_tdest),
        .cfg_base_addr(32'h0),
        .m_axi_awid(awid), .m_axi_awaddr(awaddr),
        .m_axi_awlen(awlen), .m_axi_awsize(awsize), .m_axi_awburst(awburst),
        .m_axi_awlock(awlock), .m_axi_awcache(awcache),
        .m_axi_awprot(awprot), .m_axi_awqos(awqos),
        .m_axi_awvalid(awvalid), .m_axi_awready(awready),
        .m_axi_wdata(wdata), .m_axi_wstrb(wstrb),
        .m_axi_wlast(wlast), .m_axi_wvalid(wvalid), .m_axi_wready(wready),
        .m_axi_bid(bid), .m_axi_bresp(bresp),
        .m_axi_bvalid(bvalid), .m_axi_bready(bready)
    );

    // ----- ifb_axi_slave → SRAM 写口 -----
    logic                ifb_we;
    logic [SRAM_AW-1:0]  ifb_waddr;
    logic [DATA_W-1:0]   ifb_wdata;
    logic [15:0]         rows_pushed;
    logic [15:0]         rows_consumed = 16'd0;     // 不消费, ring 不会满 (足够大 strip)

    // tie ar/r (push-only)
    logic [ID_W-1:0]     arid = 0;
    logic [ADDR_W-1:0]   araddr = 0;
    logic [7:0]          arlen = 0;
    logic [2:0]          arsize = 0;
    logic [1:0]          arburst = 0;
    logic                arlock = 0;
    logic [3:0]          arcache = 0;
    logic [2:0]          arprot = 0;
    logic [3:0]          arqos = 0;
    logic                arvalid = 0;
    logic                arready;
    logic [ID_W-1:0]     rid;
    logic [DATA_W-1:0]   rdata;
    logic [1:0]          rresp;
    logic                rlast;
    logic                rvalid;
    logic                rready = 0;

    ifb_axi_slave #(
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .ID_W(ID_W),
        .SRAM_AW(SRAM_AW), .IFB_W(DATA_W)
    ) u_ifb_slv (
        .clk(clk), .rstn(rst_n),
        .AWID(awid), .AWADDR(awaddr), .AWLEN(awlen), .AWSIZE(awsize),
        .AWBURST(awburst), .AWLOCK(awlock), .AWCACHE(awcache),
        .AWPROT(awprot), .AWQOS(awqos),
        .AWVALID(awvalid), .AWREADY(awready),
        .WDATA(wdata), .WSTRB(wstrb), .WLAST(wlast),
        .WVALID(wvalid), .WREADY(wready),
        .BID(bid), .BRESP(bresp),
        .BVALID(bvalid), .BREADY(bready),
        .ARID(arid), .ARADDR(araddr), .ARLEN(arlen), .ARSIZE(arsize),
        .ARBURST(arburst), .ARLOCK(arlock), .ARCACHE(arcache),
        .ARPROT(arprot), .ARQOS(arqos),
        .ARVALID(arvalid), .ARREADY(arready),
        .RID(rid), .RDATA(rdata), .RRESP(rresp),
        .RLAST(rlast), .RVALID(rvalid), .RREADY(rready),
        .evt_start_layer(1'b0),                // 不重置 rows
        .cfg_skip_idma(1'b1),                  // consumer 模式接收 push
        .rows_consumed(rows_consumed),
        .cfg_ifb_strip_rows(8'd200),           // ring 大, 不会满
        .cfg_ifb_ring_words(SRAM_AW'(8192)),
        .ifb_we(ifb_we), .ifb_waddr(ifb_waddr), .ifb_wdata(ifb_wdata),
        .rows_pushed_out(rows_pushed)
    );

    // ----- IFB SRAM mock -----
    logic [DATA_W-1:0]   sram_re_dummy;
    sram_model #(.DEPTH(SRAM_DEPTH), .DATA_WIDTH(DATA_W)) u_sram (
        .clk(clk),
        .we(ifb_we), .waddr(ifb_waddr), .wdata(ifb_wdata),
        .re(1'b0), .raddr({SRAM_AW{1'b0}}), .rdata(sram_re_dummy)
    );

    // ----- TB drive -----
    int errors = 0;
    logic [DATA_W-1:0] expected_data [16];

    initial begin
        s_tvalid = 0; s_tdata = 0; s_tlast = 0;
        for (int i = 0; i < 16; i++) expected_data[i] = {{32'hC0FFEE}, {32'hDEADBEEF}, {32'(i*2)}, {32'(i)}};
        #20 rst_n = 1;
        #10;

        $display("== tb_mesh_to_ifb: AXIS packet → AXI4 writer → ifb_axi_slave → SRAM ==");

        // 发 1 个 packet (16 word, addr=0): header + 16 body, tail=last
        // header: opcode=0, addr=0x0, burst_len=16
        @(posedge clk);
        s_tvalid = 1'b1;
        s_tdata  = {4'h0, 20'h00000, 16'd16, 88'd0};
        s_tlast  = 1'b0;
        do @(posedge clk); while (!s_tready);

        for (int b = 0; b < 16; b++) begin
            s_tdata = expected_data[b];
            s_tlast = (b == 15) ? 1'b1 : 1'b0;
            do @(posedge clk); while (!s_tready);
        end
        s_tvalid = 1'b0;
        s_tlast  = 1'b0;

        // 等 axi b 完成 + 一些拍
        repeat (10) @(posedge clk);

        // 检查 IFB SRAM[0..15]
        $display("\n[Check IFB SRAM contents]");
        for (int i = 0; i < 16; i++) begin
            automatic logic [DATA_W-1:0] got = u_sram.mem[i];
            automatic logic [DATA_W-1:0] exp = expected_data[i];
            if (got !== exp) begin
                if (errors < 3)
                    $display("  FAIL sram[%0d]=%h expect %h", i, got, exp);
                errors++;
            end
        end
        if (errors == 0)
            $display("  PASS: IFB SRAM[0..15] all 16 words match");

        $display("\n  rows_pushed = %0d (expect 1)", rows_pushed);

        $display("\n============================================================");
        if (errors == 0)
            $display("  RESULT: PASS  (Phase 6 mesh→axi→IFB chain end-to-end)");
        else
            $display("  RESULT: FAIL  errors=%0d", errors);
        $display("============================================================");
        $finish;
    end

    initial begin
        #10000 $display("WATCHDOG TIMEOUT"); $finish;
    end
endmodule
