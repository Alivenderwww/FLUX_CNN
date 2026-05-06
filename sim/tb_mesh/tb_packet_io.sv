`timescale 1ns/1ps

// =============================================================================
// tb_packet_io.sv  --  packet rx/tx + router 端到端 sim
//
// 拓扑:
//   axis_packet_tx (Mem Core 模拟) → router_node @ (0,0) → axis_packet_rx (ConvCore 模拟)
//
// 简化场景: tx 节点 LOCAL → router LOCAL → router LOCAL → rx 节点 LOCAL
//   实际上单 router 把 LOCAL in 路由到 LOCAL out (loopback) 因为 dst=(0,0).
//   = TX 端发 packet, 经 router 路由后从 router 的 LOCAL out 出来给 RX 端.
//
// 测试:
//   T1: 单 packet 4 word 写 IFB, 验证 RX 端 SRAM 接到正确数据
//   T2: 多 packet 连发 (写 IFB + 写 WB), 验证 opcode 区分
// =============================================================================

module tb_packet_io;
    timeunit 1ns; timeprecision 1ps;

    localparam int DATA_W = 128;
    localparam int DEST_W = 8;
    localparam int ADDR_W = 20;

    logic clk = 0; always #5 clk = ~clk;
    logic rst_n = 0;

    // ----- TX 端 -----
    logic                tx_start;
    logic [3:0]          tx_opcode;
    logic [ADDR_W-1:0]   tx_sram_addr;
    logic [15:0]         tx_burst_len;
    logic [DEST_W-1:0]   tx_tdest;
    logic                tx_busy;
    logic                tx_done;

    // TX 端模拟 SRAM (上层提供 read data)
    logic                tx_sram_re;
    logic [ADDR_W-1:0]   tx_sram_raddr;
    logic [DATA_W-1:0]   tx_sram_rdata;

    // mock SRAM 数据 (用 array 模拟)
    logic [DATA_W-1:0]   tx_mem [0:1023];
    initial for (int i = 0; i < 1024; i++) tx_mem[i] = {96'hC0FFEE, 32'(i)};
    always_ff @(posedge clk) tx_sram_rdata <= tx_mem[tx_sram_raddr[9:0]];

    // TX → router 的 AXIS link
    logic                tx_axis_tvalid;
    logic                tx_axis_tready;
    logic [DATA_W-1:0]   tx_axis_tdata;
    logic                tx_axis_tlast;
    logic [DEST_W-1:0]   tx_axis_tdest;

    axis_packet_tx #(
        .DATA_W(DATA_W), .DEST_W(DEST_W), .ADDR_W(ADDR_W)
    ) u_tx (
        .clk(clk), .rst_n(rst_n),
        .start(tx_start),
        .cfg_opcode(tx_opcode),
        .cfg_sram_addr(tx_sram_addr),
        .cfg_burst_len(tx_burst_len),
        .cfg_tdest(tx_tdest),
        .busy(tx_busy), .done(tx_done),
        .sram_re(tx_sram_re), .sram_raddr(tx_sram_raddr), .sram_rdata(tx_sram_rdata),
        .m_axis_tvalid(tx_axis_tvalid), .m_axis_tready(tx_axis_tready),
        .m_axis_tdata(tx_axis_tdata),   .m_axis_tlast(tx_axis_tlast),
        .m_axis_tdest(tx_axis_tdest)
    );

    // ----- Router @ (0,0) -----
    // 5 input/output, 我们只用 LOCAL (idx 0)
    logic [4:0]            r_in_tvalid;
    logic [4:0]            r_in_tready;
    logic [4:0]            r_in_tlast;
    logic [DATA_W-1:0]     r_in_tdata  [5];
    logic [DEST_W-1:0]     r_in_tdest  [5];

    logic [4:0]            r_out_tvalid;
    logic [4:0]            r_out_tready;
    logic [4:0]            r_out_tlast;
    logic [DATA_W-1:0]     r_out_tdata  [5];
    logic [DEST_W-1:0]     r_out_tdest  [5];

    // TX → router LOCAL in
    assign r_in_tvalid[0] = tx_axis_tvalid;
    assign r_in_tlast[0]  = tx_axis_tlast;
    assign r_in_tdata[0]  = tx_axis_tdata;
    assign r_in_tdest[0]  = tx_axis_tdest;
    assign tx_axis_tready = r_in_tready[0];

    // 其他 4 ports tie 0 (没邻居)
    genvar gi;
    generate
        for (gi = 1; gi < 5; gi++) begin : g_tie
            assign r_in_tvalid[gi] = 1'b0;
            assign r_in_tlast[gi]  = 1'b0;
            assign r_in_tdata[gi]  = '0;
            assign r_in_tdest[gi]  = '0;
            assign r_out_tready[gi] = 1'b1;
        end
    endgenerate

    router_node #(.X_POS(0), .Y_POS(0), .DATA_W(DATA_W), .DEST_W(DEST_W)) u_router (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tvalid(r_in_tvalid),  .s_axis_tready(r_in_tready),
        .s_axis_tlast(r_in_tlast),    .s_axis_tdata(r_in_tdata),
        .s_axis_tdest(r_in_tdest),
        .m_axis_tvalid(r_out_tvalid), .m_axis_tready(r_out_tready),
        .m_axis_tlast(r_out_tlast),   .m_axis_tdata(r_out_tdata),
        .m_axis_tdest(r_out_tdest)
    );

    // ----- RX 端 (router LOCAL out → packet rx) -----
    logic                rx_axis_tvalid;
    logic                rx_axis_tready;
    logic [DATA_W-1:0]   rx_axis_tdata;
    logic                rx_axis_tlast;
    logic [DEST_W-1:0]   rx_axis_tdest;

    assign rx_axis_tvalid = r_out_tvalid[0];
    assign rx_axis_tlast  = r_out_tlast[0];
    assign rx_axis_tdata  = r_out_tdata[0];
    assign rx_axis_tdest  = r_out_tdest[0];
    assign r_out_tready[0] = rx_axis_tready;

    logic                rx_sram_we;
    logic [3:0]          rx_sram_target;
    logic [ADDR_W-1:0]   rx_sram_waddr;
    logic [DATA_W-1:0]   rx_sram_wdata;
    logic                rx_packet_done;
    logic [3:0]          rx_last_opcode;
    logic [15:0]         rx_last_burst_len;
    logic [ADDR_W-1:0]   rx_last_addr_offset;

    axis_packet_rx #(
        .DATA_W(DATA_W), .DEST_W(DEST_W), .ADDR_W(ADDR_W)
    ) u_rx (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tvalid(rx_axis_tvalid), .s_axis_tready(rx_axis_tready),
        .s_axis_tlast(rx_axis_tlast),   .s_axis_tdata(rx_axis_tdata),
        .s_axis_tdest(rx_axis_tdest),
        .sram_we(rx_sram_we), .sram_target(rx_sram_target),
        .sram_waddr(rx_sram_waddr), .sram_wdata(rx_sram_wdata),
        .packet_done(rx_packet_done),
        .last_opcode(rx_last_opcode),
        .last_burst_len(rx_last_burst_len),
        .last_addr_offset(rx_last_addr_offset)
    );

    // 模拟 RX 端目标 SRAM (按 opcode 分 4 个区, 简化: 都写到同一 array, key by addr)
    // target 0=IFB, 1=WB, 2=SB, 3=RDMA
    logic [DATA_W-1:0]   rx_mem [0:3][0:1023];
    always_ff @(posedge clk) begin
        if (rx_sram_we) begin
            rx_mem[rx_sram_target][rx_sram_waddr[9:0]] <= rx_sram_wdata;
        end
    end

    // ----- Test sequence -----
    int errors = 0;

    task automatic do_packet(input logic [3:0] op, input int addr, len, int dst);
        @(posedge clk);
        tx_start     = 1'b1;
        tx_opcode    = op;
        tx_sram_addr = ADDR_W'(addr);
        tx_burst_len = 16'(len);
        tx_tdest     = 8'(dst);   // 这个 sim 里固定 dst=(0,0)=0x00
        @(posedge clk);
        tx_start = 1'b0;
        // 等 tx_done
        do @(posedge clk); while (!tx_done);
        // 多等几拍 让 rx 收完
        repeat (4) @(posedge clk);
    endtask

    initial begin
        tx_start = 0;
        tx_opcode = 0;
        tx_sram_addr = 0;
        tx_burst_len = 0;
        tx_tdest = 0;
        #20 rst_n = 1;
        #10;

        $display("== tb_packet_io: TX → router (loopback) → RX ==");

        // T1: 写 IFB, 4 word, addr_offset=0x100
        $display("\n[T1] WRITE_IFB 4 word, addr=0x100");
        do_packet(4'h0, 32'h100, 4, 8'h00);
        // 检查 rx_last_*
        if (rx_last_opcode != 4'h0) begin
            $display("  FAIL T1: opcode=%0h expect 0", rx_last_opcode);
            errors++;
        end
        if (rx_last_burst_len != 16'd4) begin
            $display("  FAIL T1: burst_len=%0d expect 4", rx_last_burst_len);
            errors++;
        end
        if (rx_last_addr_offset != 20'h100) begin
            $display("  FAIL T1: addr=%0h expect 100", rx_last_addr_offset);
            errors++;
        end
        // 检查 RX SRAM 内容: rx_mem[IFB][0x100..0x103] 应该 = tx_mem[0x100..0x103]
        // (TX 端从 sram_addr=0x100 开始读)
        for (int i = 0; i < 4; i++) begin
            if (rx_mem[0][20'h100 + i] !== tx_mem[20'h100 + i]) begin
                $display("  FAIL T1: rx_mem[IFB][%0h]=%h expect tx_mem[%0h]=%h",
                         20'h100+i, rx_mem[0][20'h100 + i],
                         20'h100+i, tx_mem[20'h100 + i]);
                errors++;
            end
        end
        if (errors == 0) $display("  PASS T1: IFB write 4 word OK");

        // T2: 写 WB, 8 word, addr_offset=0x200
        $display("\n[T2] WRITE_WB 8 word, addr=0x200");
        do_packet(4'h1, 32'h200, 8, 8'h00);
        if (rx_last_opcode != 4'h1) begin
            $display("  FAIL T2: opcode=%0h expect 1", rx_last_opcode);
            errors++;
        end
        if (rx_last_burst_len != 16'd8) begin
            $display("  FAIL T2: burst_len=%0d expect 8", rx_last_burst_len);
            errors++;
        end
        for (int i = 0; i < 8; i++) begin
            if (rx_mem[1][20'h200 + i] !== tx_mem[20'h200 + i]) begin
                $display("  FAIL T2: rx_mem[WB][%0h]=%h expect tx_mem[%0h]=%h",
                         20'h200+i, rx_mem[1][20'h200 + i],
                         20'h200+i, tx_mem[20'h200 + i]);
                errors++;
            end
        end
        if (errors == 0) $display("  PASS T2: WB write 8 word OK");

        $display("\n============================================================");
        if (errors == 0)
            $display("  RESULT: PASS  (packet_io end-to-end)");
        else
            $display("  RESULT: FAIL  errors=%0d", errors);
        $display("============================================================");
        $finish;
    end

    initial begin
        #50000 $display("WATCHDOG TIMEOUT"); $finish;
    end
endmodule
