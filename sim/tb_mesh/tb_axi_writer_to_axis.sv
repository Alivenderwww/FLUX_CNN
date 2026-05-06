`timescale 1ns/1ps

// =============================================================================
// tb_axi_writer_to_axis.sv  --  axi_writer_to_axis 单元测试
//
// TB 模拟 axi master burst write 输入 → 桥转 axis packet 输出
// 验证: header 内容 (opcode/addr/burst_len) + body 数据 + tlast.
// =============================================================================

module tb_axi_writer_to_axis;
    timeunit 1ns; timeprecision 1ps;

    localparam int DATA_W = 128;
    localparam int DEST_W = 8;
    localparam int ADDR_W = 32;
    localparam int ID_W   = 6;

    logic clk = 0; always #5 clk = ~clk;
    logic rst_n = 0;

    // ----- TB → bridge AXI slave -----
    logic [ID_W-1:0]     s_awid;
    logic [ADDR_W-1:0]   s_awaddr;
    logic [7:0]          s_awlen;
    logic [2:0]          s_awsize;
    logic [1:0]          s_awburst;
    logic                s_awvalid;
    logic                s_awready;
    logic [DATA_W-1:0]   s_wdata;
    logic [DATA_W/8-1:0] s_wstrb;
    logic                s_wlast;
    logic                s_wvalid;
    logic                s_wready;
    logic [ID_W-1:0]     s_bid;
    logic [1:0]          s_bresp;
    logic                s_bvalid;
    logic                s_bready;

    // ----- bridge → TB AXIS master -----
    logic                m_tvalid;
    logic                m_tready = 1'b1;     // TB 总是 ready
    logic [DATA_W-1:0]   m_tdata;
    logic                m_tlast;
    logic [DEST_W-1:0]   m_tdest;

    axi_writer_to_axis #(
        .DATA_W(DATA_W), .DEST_W(DEST_W), .ADDR_W(ADDR_W), .ID_W(ID_W)
    ) u_bridge (
        .clk(clk), .rst_n(rst_n),
        .s_axi_awid(s_awid), .s_axi_awaddr(s_awaddr),
        .s_axi_awlen(s_awlen), .s_axi_awsize(s_awsize), .s_axi_awburst(s_awburst),
        .s_axi_awvalid(s_awvalid), .s_axi_awready(s_awready),
        .s_axi_wdata(s_wdata), .s_axi_wstrb(s_wstrb),
        .s_axi_wlast(s_wlast), .s_axi_wvalid(s_wvalid), .s_axi_wready(s_wready),
        .s_axi_bid(s_bid), .s_axi_bresp(s_bresp),
        .s_axi_bvalid(s_bvalid), .s_axi_bready(s_bready),
        .cfg_opcode(4'h5),                         // WRITE_DDR_OFB
        .cfg_tdest(8'h00),                         // dst (0,0)
        .m_axis_tvalid(m_tvalid), .m_axis_tready(m_tready),
        .m_axis_tdata(m_tdata),   .m_axis_tlast(m_tlast),
        .m_axis_tdest(m_tdest)
    );

    // ----- AXIS 接收记录 -----
    int                axis_count;
    logic [DATA_W-1:0] axis_log [0:31];
    logic              axis_tlast_log [0:31];
    logic              b_received = 1'b0;

    initial axis_count = 0;

    always @(posedge clk) begin
        if (rst_n && s_bvalid && s_bready) b_received <= 1'b1;
    end

    always @(posedge clk) begin
        if (rst_n && m_tvalid && m_tready) begin
            axis_log[axis_count]       = m_tdata;
            axis_tlast_log[axis_count] = m_tlast;
            axis_count                 = axis_count + 1;
        end
    end

    // ----- TB drive: 模拟 1 个 axi burst write (4 beat, awaddr=0x1000, awlen=3) -----
    int errors = 0;

    initial begin
        s_awid = 0; s_awaddr = 0; s_awlen = 0; s_awsize = 4; s_awburst = 2'b01;
        s_awvalid = 0;
        s_wdata = 0; s_wstrb = '1; s_wlast = 0; s_wvalid = 0;
        s_bready = 1;
        #20 rst_n = 1;
        #10;

        $display("== tb_axi_writer_to_axis: AXI write burst → AXIS packet ==");

        // AW: awaddr=0x1000, awlen=3 (4 beat)
        @(posedge clk);
        s_awid    = 6'd1;
        s_awaddr  = 32'h1000;
        s_awlen   = 8'd3;
        s_awsize  = 3'd4;
        s_awburst = 2'b01;
        s_awvalid = 1'b1;
        do @(posedge clk); while (!s_awready);
        s_awvalid = 1'b0;

        // W beats × 4
        for (int b = 0; b < 4; b++) begin
            s_wdata  = {32'hABCD, 32'hEFAB, 32'(b * 16), 32'(b)};
            s_wlast  = (b == 3) ? 1'b1 : 1'b0;
            s_wvalid = 1'b1;
            do @(posedge clk); while (!s_wready);
        end
        s_wvalid = 1'b0;
        s_wlast  = 1'b0;

        // B
        do @(posedge clk); while (!s_bvalid);
        repeat (5) @(posedge clk);

        // 验证 AXIS 输出
        $display("\n[AXIS output, %0d flits]", axis_count);
        for (int i = 0; i < axis_count; i++)
            $display("  flit[%0d] tdata=%h tlast=%b", i, axis_log[i], axis_tlast_log[i]);

        if (axis_count != 5) begin   // 1 header + 4 body
            $display("  FAIL: expect 5 flits, got %0d", axis_count);
            errors++;
        end

        // header 内容: opcode=5, addr=0x100 (=0x1000>>4), burst_len=4
        begin
            automatic logic [3:0]  op   = axis_log[0][127:124];
            automatic logic [19:0] addr = axis_log[0][123:104];
            automatic logic [15:0] len  = axis_log[0][103:88];
            $display("  header: opcode=%h addr=0x%h burst_len=%0d", op, addr, len);
            if (op != 4'h5)        begin $display("  FAIL: opcode"); errors++; end
            if (addr != 20'h00100) begin $display("  FAIL: addr"); errors++; end
            if (len != 16'd4)      begin $display("  FAIL: burst_len"); errors++; end
            if (axis_tlast_log[0] != 1'b0) begin $display("  FAIL: header tlast=1"); errors++; end
        end

        // body 数据 (跟 wdata 一致)
        for (int b = 0; b < 4; b++) begin
            automatic logic [DATA_W-1:0] expected = {32'hABCD, 32'hEFAB, 32'(b * 16), 32'(b)};
            if (axis_log[b + 1] !== expected) begin
                $display("  FAIL body[%0d]=%h expect %h", b, axis_log[b + 1], expected);
                errors++;
            end
        end
        // tail flit (last body)
        if (axis_tlast_log[4] != 1'b1) begin
            $display("  FAIL: body[3] should have tlast=1");
            errors++;
        end

        // axi b 完成
        if (!b_received) begin
            $display("  FAIL: bvalid not raised");
            errors++;
        end else $display("  PASS: axi b received");

        $display("\n============================================================");
        if (errors == 0)
            $display("  RESULT: PASS  (axi → axis writer bridge)");
        else
            $display("  RESULT: FAIL  errors=%0d", errors);
        $display("============================================================");
        $finish;
    end

    initial begin
        #5000 $display("WATCHDOG TIMEOUT"); $finish;
    end
endmodule
