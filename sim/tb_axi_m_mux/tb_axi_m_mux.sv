`timescale 1ns/1ps

// =============================================================================
// tb_axi_m_mux.sv  --  smoke test for axi_m_mux
//
// 拓扑：
//   Master[0] (task-driven) ┐
//                           ├── axi_m_mux ── axi_slave_mem (4KB)
//   Master[1] (task-driven) ┘
//
// 测试：
//   Phase 1：M0 写 burst @ 0x000, M1 写 burst @ 0x800；并发发起，观察仲裁
//   Phase 2：M0 读回 @ 0x000, M1 读回 @ 0x800；检查数据
// =============================================================================
module tb_axi_m_mux;

    localparam int M_WIDTH = 1;               // 2 个 master
    localparam int M_ID    = 2;
    localparam int ADDR_W  = 32;
    localparam int DATA_W  = 128;
    localparam int N       = 2**M_WIDTH;

    logic clk  = 0;
    logic rstn = 0;
    always #5 clk = ~clk;

    // ---- master side (打包 2D) ----
    logic [N-1:0] [M_ID-1:0]     M_AWID;
    logic [N-1:0] [ADDR_W-1:0]   M_AWADDR;
    logic [N-1:0] [7:0]          M_AWLEN;
    logic [N-1:0] [1:0]          M_AWBURST;
    logic [N-1:0]                M_AWVALID;
    logic [N-1:0]                M_AWREADY;
    logic [N-1:0] [DATA_W-1:0]   M_WDATA;
    logic [N-1:0] [DATA_W/8-1:0] M_WSTRB;
    logic [N-1:0]                M_WLAST;
    logic [N-1:0]                M_WVALID;
    logic [N-1:0]                M_WREADY;
    logic [N-1:0] [M_ID-1:0]     M_BID;
    logic [N-1:0] [1:0]          M_BRESP;
    logic [N-1:0]                M_BVALID;
    logic [N-1:0]                M_BREADY;
    logic [N-1:0] [M_ID-1:0]     M_ARID;
    logic [N-1:0] [ADDR_W-1:0]   M_ARADDR;
    logic [N-1:0] [7:0]          M_ARLEN;
    logic [N-1:0] [1:0]          M_ARBURST;
    logic [N-1:0]                M_ARVALID;
    logic [N-1:0]                M_ARREADY;
    logic [N-1:0] [M_ID-1:0]     M_RID;
    logic [N-1:0] [DATA_W-1:0]   M_RDATA;
    logic [N-1:0] [1:0]          M_RRESP;
    logic [N-1:0]                M_RLAST;
    logic [N-1:0]                M_RVALID;
    logic [N-1:0]                M_RREADY;

    // ---- bus side ----
    logic [M_ID+M_WIDTH-1:0] B_AWID, B_BID, B_ARID, B_RID;
    logic [ADDR_W-1:0]       B_AWADDR, B_ARADDR;
    logic [7:0]              B_AWLEN, B_ARLEN;
    logic [1:0]              B_AWBURST, B_ARBURST, B_BRESP, B_RRESP;
    logic                    B_AWVALID, B_AWREADY, B_WVALID, B_WREADY, B_WLAST;
    logic                    B_BVALID, B_BREADY, B_ARVALID, B_ARREADY;
    logic                    B_RVALID, B_RREADY, B_RLAST;
    logic [DATA_W-1:0]       B_WDATA, B_RDATA;
    logic [DATA_W/8-1:0]     B_WSTRB;

    axi_m_mux #(
        .M_WIDTH(M_WIDTH),
        .M_ID   (M_ID),
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W)
    ) u_mux (
        .clk(clk), .rstn(rstn),
        .M_AWID(M_AWID), .M_AWADDR(M_AWADDR), .M_AWLEN(M_AWLEN), .M_AWBURST(M_AWBURST),
        .M_AWVALID(M_AWVALID), .M_AWREADY(M_AWREADY),
        .M_WDATA(M_WDATA), .M_WSTRB(M_WSTRB), .M_WLAST(M_WLAST),
        .M_WVALID(M_WVALID), .M_WREADY(M_WREADY),
        .M_BID(M_BID), .M_BRESP(M_BRESP), .M_BVALID(M_BVALID), .M_BREADY(M_BREADY),
        .M_ARID(M_ARID), .M_ARADDR(M_ARADDR), .M_ARLEN(M_ARLEN), .M_ARBURST(M_ARBURST),
        .M_ARVALID(M_ARVALID), .M_ARREADY(M_ARREADY),
        .M_RID(M_RID), .M_RDATA(M_RDATA), .M_RRESP(M_RRESP), .M_RLAST(M_RLAST),
        .M_RVALID(M_RVALID), .M_RREADY(M_RREADY),
        .B_AWID(B_AWID), .B_AWADDR(B_AWADDR), .B_AWLEN(B_AWLEN), .B_AWBURST(B_AWBURST),
        .B_AWVALID(B_AWVALID), .B_AWREADY(B_AWREADY),
        .B_WDATA(B_WDATA), .B_WSTRB(B_WSTRB), .B_WLAST(B_WLAST),
        .B_WVALID(B_WVALID), .B_WREADY(B_WREADY),
        .B_BID(B_BID), .B_BRESP(B_BRESP), .B_BVALID(B_BVALID), .B_BREADY(B_BREADY),
        .B_ARID(B_ARID), .B_ARADDR(B_ARADDR), .B_ARLEN(B_ARLEN), .B_ARBURST(B_ARBURST),
        .B_ARVALID(B_ARVALID), .B_ARREADY(B_ARREADY),
        .B_RID(B_RID), .B_RDATA(B_RDATA), .B_RRESP(B_RRESP), .B_RLAST(B_RLAST),
        .B_RVALID(B_RVALID), .B_RREADY(B_RREADY)
    );

    axi_slave_mem #(
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .ID_W(M_ID+M_WIDTH), .DEPTH(1024)
    ) u_slave (
        .clk(clk), .rstn(rstn),
        .AWID(B_AWID), .AWADDR(B_AWADDR), .AWLEN(B_AWLEN), .AWBURST(B_AWBURST),
        .AWVALID(B_AWVALID), .AWREADY(B_AWREADY),
        .WDATA(B_WDATA), .WSTRB(B_WSTRB), .WLAST(B_WLAST),
        .WVALID(B_WVALID), .WREADY(B_WREADY),
        .BID(B_BID), .BRESP(B_BRESP), .BVALID(B_BVALID), .BREADY(B_BREADY),
        .ARID(B_ARID), .ARADDR(B_ARADDR), .ARLEN(B_ARLEN), .ARBURST(B_ARBURST),
        .ARVALID(B_ARVALID), .ARREADY(B_ARREADY),
        .RID(B_RID), .RDATA(B_RDATA), .RRESP(B_RRESP), .RLAST(B_RLAST),
        .RVALID(B_RVALID), .RREADY(B_RREADY)
    );

    // =========================================================================
    // Master 驱动 task（per-master，并发）
    // =========================================================================
    task automatic drive_write(input int m, input [ADDR_W-1:0] addr,
                                input [7:0] len, input [DATA_W-1:0] seed);
        automatic int i;
        M_AWID   [m] = m[M_ID-1:0] + M_ID'('ha);
        M_AWADDR [m] = addr;
        M_AWLEN  [m] = len;
        M_AWBURST[m] = 2'b01;
        M_AWVALID[m] = 1'b1;
        @(posedge clk);
        while (!M_AWREADY[m]) @(posedge clk);
        M_AWVALID[m] = 1'b0;

        for (i = 0; i <= len; i++) begin
            M_WDATA [m] = seed + DATA_W'(i);
            M_WSTRB [m] = '1;
            M_WLAST [m] = (i == len);
            M_WVALID[m] = 1'b1;
            @(posedge clk);
            while (!M_WREADY[m]) @(posedge clk);
        end
        M_WVALID[m] = 1'b0;
        M_WLAST [m] = 1'b0;

        M_BREADY[m] = 1'b1;
        @(posedge clk);
        while (!M_BVALID[m]) @(posedge clk);
        $display("  [M%0d] B received, BID=%0d, BRESP=%0d", m, M_BID[m], M_BRESP[m]);
        M_BREADY[m] = 1'b0;
    endtask

    task automatic drive_read(input int m, input [ADDR_W-1:0] addr,
                               input [7:0] len, input [DATA_W-1:0] seed,
                               output int mismatch_cnt);
        automatic int i;
        automatic logic [DATA_W-1:0] expect_data;
        mismatch_cnt = 0;
        M_ARID   [m] = m[M_ID-1:0] + M_ID'('h5);
        M_ARADDR [m] = addr;
        M_ARLEN  [m] = len;
        M_ARBURST[m] = 2'b01;
        M_ARVALID[m] = 1'b1;
        @(posedge clk);
        while (!M_ARREADY[m]) @(posedge clk);
        M_ARVALID[m] = 1'b0;

        M_RREADY[m] = 1'b1;
        for (i = 0; i <= len; i++) begin
            @(posedge clk);
            while (!M_RVALID[m]) @(posedge clk);
            expect_data = seed + DATA_W'(i);
            if (M_RDATA[m] !== expect_data) begin
                $display("  [M%0d] FAIL beat %0d: expect=%h got=%h",
                         m, i, expect_data, M_RDATA[m]);
                mismatch_cnt++;
            end
            if (i == len && !M_RLAST[m])
                $display("  [M%0d] FAIL: RLAST missing at last beat", m);
        end
        M_RREADY[m] = 1'b0;
    endtask

    // =========================================================================
    // 主流程
    // =========================================================================
    int total_mismatches = 0;
    int cnt;

    initial begin
        M_AWID = '0; M_AWADDR = '0; M_AWLEN = '0; M_AWBURST = '0; M_AWVALID = '0;
        M_WDATA = '0; M_WSTRB = '0; M_WLAST = '0; M_WVALID = '0;
        M_BREADY = '0;
        M_ARID = '0; M_ARADDR = '0; M_ARLEN = '0; M_ARBURST = '0; M_ARVALID = '0;
        M_RREADY = '0;

        #20 rstn = 1;
        #10;

        $display("=== Phase 1: concurrent writes from M0 and M1 ===");
        fork
            drive_write(0, 32'h0000_0000, 8'd7,  128'h1000);
            drive_write(1, 32'h0000_0800, 8'd11, 128'h2000);
        join

        #20;
        $display("=== Phase 2: concurrent reads from M0 and M1 ===");
        fork
            begin drive_read(0, 32'h0000_0000, 8'd7,  128'h1000, cnt);
                  total_mismatches += cnt; end
            begin drive_read(1, 32'h0000_0800, 8'd11, 128'h2000, cnt);
                  total_mismatches += cnt; end
        join

        #20;
        if (total_mismatches == 0)
            $display("SMOKE TEST PASSED. 0 mismatches.");
        else
            $display("SMOKE TEST FAILED. %0d mismatches.", total_mismatches);
        $stop;
    end

    initial begin
        #200000;
        $display("FATAL: smoke test timeout");
        $stop;
    end

endmodule
