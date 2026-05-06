`timescale 1ns/1ps

// =============================================================================
// tb_axis_to_axi_writer.sv  --  axis_to_axi_writer 单元测试
//
// 模拟流程: TB 发 1 个 axis packet (header + 4 body + tail)
//          → axis_to_axi_writer 转 → axi4 master burst write
//          → fake axi slave 收 + count beats
//
// 验证: axi awaddr / awlen 跟 axi w 的 4 beat 数据正确.
// =============================================================================

module tb_axis_to_axi_writer;
    timeunit 1ns; timeprecision 1ps;

    localparam int DATA_W = 128;
    localparam int DEST_W = 8;
    localparam int ADDR_W = 32;
    localparam int ID_W   = 6;

    logic clk = 0; always #5 clk = ~clk;
    logic rst_n = 0;

    // ----- TB → writer (AXIS) -----
    logic                s_tvalid;
    logic                s_tready;
    logic [DATA_W-1:0]   s_tdata;
    logic                s_tlast;
    logic [DEST_W-1:0]   s_tdest = 0;

    // ----- writer → fake AXI slave -----
    logic [ID_W-1:0]     m_awid;
    logic [ADDR_W-1:0]   m_awaddr;
    logic [7:0]          m_awlen;
    logic [2:0]          m_awsize;
    logic [1:0]          m_awburst;
    logic                m_awlock;
    logic [3:0]          m_awcache;
    logic [2:0]          m_awprot;
    logic [3:0]          m_awqos;
    logic                m_awvalid;
    logic                m_awready;

    logic [DATA_W-1:0]   m_wdata;
    logic [DATA_W/8-1:0] m_wstrb;
    logic                m_wlast;
    logic                m_wvalid;
    logic                m_wready;

    logic [ID_W-1:0]     m_bid;
    logic [1:0]          m_bresp;
    logic                m_bvalid;
    logic                m_bready;

    axis_to_axi_writer #(
        .DATA_W(DATA_W), .DEST_W(DEST_W), .ADDR_W(ADDR_W), .ID_W(ID_W)
    ) u_writer (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tvalid(s_tvalid), .s_axis_tready(s_tready),
        .s_axis_tdata(s_tdata),   .s_axis_tlast(s_tlast),
        .s_axis_tdest(s_tdest),
        .cfg_base_addr(32'h0000_0000),  // 不加 base, packet header addr 直接用
        .m_axi_awid(m_awid), .m_axi_awaddr(m_awaddr),
        .m_axi_awlen(m_awlen), .m_axi_awsize(m_awsize), .m_axi_awburst(m_awburst),
        .m_axi_awlock(m_awlock), .m_axi_awcache(m_awcache),
        .m_axi_awprot(m_awprot), .m_axi_awqos(m_awqos),
        .m_axi_awvalid(m_awvalid), .m_axi_awready(m_awready),
        .m_axi_wdata(m_wdata), .m_axi_wstrb(m_wstrb),
        .m_axi_wlast(m_wlast), .m_axi_wvalid(m_wvalid), .m_axi_wready(m_wready),
        .m_axi_bid(m_bid), .m_axi_bresp(m_bresp),
        .m_axi_bvalid(m_bvalid), .m_axi_bready(m_bready)
    );

    // ----- Fake AXI slave: 立即 ready, latch awaddr/awlen, count w beats, send b -----
    assign m_awready = 1'b1;
    assign m_wready  = 1'b1;
    assign m_bid     = '0;
    assign m_bresp   = 2'b00;
    logic        b_pending;
    assign m_bvalid  = b_pending;

    int captured_awaddr;
    int captured_awlen;
    int w_beat_count;
    logic [DATA_W-1:0] w_data_log [0:15];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            captured_awaddr <= 0;
            captured_awlen  <= 0;
            w_beat_count    <= 0;
            b_pending       <= 1'b0;
        end else begin
            if (m_awvalid && m_awready) begin
                captured_awaddr <= m_awaddr;
                captured_awlen  <= m_awlen;
            end
            if (m_wvalid && m_wready) begin
                w_data_log[w_beat_count] <= m_wdata;
                w_beat_count <= w_beat_count + 1;
                if (m_wlast) b_pending <= 1'b1;
            end
            if (m_bvalid && m_bready) b_pending <= 1'b0;
        end
    end

    // ----- TB drive -----
    int errors = 0;

    initial begin
        s_tvalid = 0;
        s_tdata  = 0;
        s_tlast  = 0;
        #20 rst_n = 1;
        #10;

        $display("== tb_axis_to_axi_writer: AXIS packet → AXI4 burst write ==");

        // 发 1 个 packet: header (opcode=0, addr=0x100, burst_len=4) + 4 body + tail
        // header flit: data[127:124]=opcode, data[123:104]=addr, data[103:88]=burst_len
        @(posedge clk);
        s_tvalid = 1'b1;
        s_tdata  = {4'h0, 20'h00100, 16'd4, 88'd0};
        s_tlast  = 1'b0;
        do @(posedge clk); while (!s_tready);

        // body 0
        s_tdata = 128'hAAAA_AAAA_AAAA_AAAA_AAAA_AAAA_AAAA_AAAA;
        s_tlast = 1'b0;
        do @(posedge clk); while (!s_tready);

        // body 1
        s_tdata = 128'hBBBB_BBBB_BBBB_BBBB_BBBB_BBBB_BBBB_BBBB;
        do @(posedge clk); while (!s_tready);

        // body 2
        s_tdata = 128'hCCCC_CCCC_CCCC_CCCC_CCCC_CCCC_CCCC_CCCC;
        do @(posedge clk); while (!s_tready);

        // body 3 (tail)
        s_tdata = 128'hDDDD_DDDD_DDDD_DDDD_DDDD_DDDD_DDDD_DDDD;
        s_tlast = 1'b1;
        do @(posedge clk); while (!s_tready);

        s_tvalid = 1'b0;
        s_tlast  = 1'b0;

        // 等 b 完成
        repeat (10) @(posedge clk);

        // 验证
        $display("\n[Captured AXI signals]");
        $display("  awaddr = 0x%0h (expect 0x1000 = 0x100 word << 4)", captured_awaddr);
        $display("  awlen  = %0d (expect 3 = burst-1)", captured_awlen);
        $display("  w_beat_count = %0d (expect 4)", w_beat_count);
        for (int i = 0; i < 4; i++)
            $display("  wdata[%0d] = %h", i, w_data_log[i]);

        if (captured_awaddr != 32'h1000) begin
            $display("  FAIL: awaddr"); errors++;
        end
        if (captured_awlen != 8'd3) begin
            $display("  FAIL: awlen"); errors++;
        end
        if (w_beat_count != 4) begin
            $display("  FAIL: w_beat_count"); errors++;
        end
        if (w_data_log[0] != 128'hAAAA_AAAA_AAAA_AAAA_AAAA_AAAA_AAAA_AAAA) begin
            $display("  FAIL: wdata[0]"); errors++;
        end
        if (w_data_log[3] != 128'hDDDD_DDDD_DDDD_DDDD_DDDD_DDDD_DDDD_DDDD) begin
            $display("  FAIL: wdata[3]"); errors++;
        end

        $display("\n============================================================");
        if (errors == 0)
            $display("  RESULT: PASS  (axis → axi writer)");
        else
            $display("  RESULT: FAIL  errors=%0d", errors);
        $display("============================================================");
        $finish;
    end

    initial begin
        #5000 $display("WATCHDOG TIMEOUT"); $finish;
    end
endmodule
