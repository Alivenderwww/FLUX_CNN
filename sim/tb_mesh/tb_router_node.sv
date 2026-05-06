`timescale 1ns/1ps

// =============================================================================
// tb_router_node.sv  --  单 router_node 单元测试 (AXI4-Stream 协议)
//
// 路由器 @ (2, 2). 测试 5-port XY routing.
// =============================================================================

module tb_router_node;
    timeunit 1ns; timeprecision 1ps;

    localparam int DATA_W = 128;
    localparam int DEST_W = 8;
    localparam int X_POS = 2;
    localparam int Y_POS = 2;

    localparam int LOCAL = 0;
    localparam int NORTH = 1;
    localparam int SOUTH = 2;
    localparam int EAST  = 3;
    localparam int WEST  = 4;

    logic clk = 0;  always #5 clk = ~clk;
    logic rst_n = 0;

    logic [4:0]            s_axis_tvalid;
    logic [4:0]            s_axis_tready;
    logic [4:0]            s_axis_tlast;
    logic [DATA_W-1:0]     s_axis_tdata  [5];
    logic [DEST_W-1:0]     s_axis_tdest  [5];

    logic [4:0]            m_axis_tvalid;
    logic [4:0]            m_axis_tready = 5'b11111;
    logic [4:0]            m_axis_tlast;
    logic [DATA_W-1:0]     m_axis_tdata  [5];
    logic [DEST_W-1:0]     m_axis_tdest  [5];

    router_node #(
        .X_POS(X_POS), .Y_POS(Y_POS), .DATA_W(DATA_W), .DEST_W(DEST_W)
    ) u_router (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tvalid(s_axis_tvalid), .s_axis_tready(s_axis_tready),
        .s_axis_tlast(s_axis_tlast),   .s_axis_tdata(s_axis_tdata),
        .s_axis_tdest(s_axis_tdest),
        .m_axis_tvalid(m_axis_tvalid), .m_axis_tready(m_axis_tready),
        .m_axis_tlast(m_axis_tlast),   .m_axis_tdata(m_axis_tdata),
        .m_axis_tdest(m_axis_tdest)
    );

    // ----- Helpers -----
    task automatic init_inputs();
        s_axis_tvalid = 5'd0;
        s_axis_tlast  = 5'd0;
        for (int i = 0; i < 5; i++) begin
            s_axis_tdata[i] = '0;
            s_axis_tdest[i] = '0;
        end
    endtask

    function automatic logic [DEST_W-1:0] make_dest(input logic [3:0] dy, dx);
        return {dy, dx};
    endfunction

    // 单 flit (tlast=1)
    task automatic send_single(input int port, input logic [3:0] dy, dx,
                                input logic [127:0] data);
        @(posedge clk);
        s_axis_tvalid[port] = 1'b1;
        s_axis_tlast[port]  = 1'b1;
        s_axis_tdata[port]  = data;
        s_axis_tdest[port]  = make_dest(dy, dx);
        do @(posedge clk); while (!s_axis_tready[port]);
        s_axis_tvalid[port] = 1'b0;
        s_axis_tlast[port]  = 1'b0;
    endtask

    // 多 flit packet (每个 flit fire 后 deassert valid 一拍, 防 valid 跨拍重发)
    task automatic send_packet(input int port, input logic [3:0] dy, dx,
                                input int n_flits, input logic [127:0] payload[]);
        for (int b = 0; b < n_flits; b++) begin
            @(posedge clk);
            s_axis_tvalid[port] = 1'b1;
            s_axis_tlast[port]  = (b == n_flits - 1) ? 1'b1 : 1'b0;
            s_axis_tdata[port]  = payload[b];
            s_axis_tdest[port]  = make_dest(dy, dx);
            do @(posedge clk); while (!s_axis_tready[port]);
            s_axis_tvalid[port] = 1'b0;
            s_axis_tlast[port]  = 1'b0;
        end
    endtask

    // Output observers
    int out_count [5];
    initial for (int p = 0; p < 5; p++) out_count[p] = 0;

    always @(posedge clk) begin
        if (rst_n) begin
            for (int p = 0; p < 5; p++) begin
                if (m_axis_tvalid[p] && m_axis_tready[p])
                    out_count[p] = out_count[p] + 1;
            end
        end
    end

    // Verdict
    int errors = 0;
    string port_name [5] = '{"LOCAL", "NORTH", "SOUTH", "EAST", "WEST"};

    task automatic check_count(input int port, input int expected, input string tag);
        if (out_count[port] != expected) begin
            $display("  FAIL %s: %s got %0d, expect %0d",
                     tag, port_name[port], out_count[port], expected);
            errors++;
        end else begin
            $display("  PASS %s: %s got %0d flit", tag, port_name[port], out_count[port]);
        end
    endtask

    task automatic clear_counts();
        for (int p = 0; p < 5; p++) out_count[p] = 0;
    endtask

    initial begin
        init_inputs();
        #20 rst_n = 1;
        #10;

        $display("== tb_router_node: AXI4-Stream router @ (%0d,%0d) ==", X_POS, Y_POS);

        // T1: L → E (dst=(3,2))
        $display("\n[T1] L → E (dst=(3,2))");
        clear_counts();
        send_single(LOCAL, 4'd2, 4'd3, 128'hAA);
        repeat (4) @(posedge clk);
        check_count(EAST,  1, "T1");
        check_count(WEST,  0, "T1");

        // T2: L → W (dst=(1,2))
        $display("\n[T2] L → W (dst=(1,2))");
        clear_counts();
        send_single(LOCAL, 4'd2, 4'd1, 128'hBB);
        repeat (4) @(posedge clk);
        check_count(WEST, 1, "T2");

        // T3: L → N (dst=(2,3))
        $display("\n[T3] L → N (dst=(2,3))");
        clear_counts();
        send_single(LOCAL, 4'd3, 4'd2, 128'hCC);
        repeat (4) @(posedge clk);
        check_count(NORTH, 1, "T3");

        // T4: L → S (dst=(2,1))
        $display("\n[T4] L → S (dst=(2,1))");
        clear_counts();
        send_single(LOCAL, 4'd1, 4'd2, 128'hDD);
        repeat (4) @(posedge clk);
        check_count(SOUTH, 1, "T4");

        // T5: L → L (dst=(2,2))
        $display("\n[T5] L → L (dst=(2,2))");
        clear_counts();
        send_single(LOCAL, 4'd2, 4'd2, 128'hEE);
        repeat (4) @(posedge clk);
        check_count(LOCAL, 1, "T5");

        // T6: 多 flit packet L → E (4 flit, 都带 tdest)
        $display("\n[T6] L → E multi-flit (4 flit packet)");
        clear_counts();
        begin
            logic [127:0] payload [4];
            payload[0] = 128'h01;
            payload[1] = 128'h02;
            payload[2] = 128'h03;
            payload[3] = 128'h04;
            send_packet(LOCAL, 4'd2, 4'd3, 4, payload);
        end
        repeat (8) @(posedge clk);
        check_count(EAST, 4, "T6");

        // T7: N→S 同时 W→E
        $display("\n[T7] N → S 同时 W → E (无冲突)");
        clear_counts();
        fork
            send_single(NORTH, 4'd1, 4'd2, 128'h11);
            send_single(WEST,  4'd2, 4'd3, 128'h22);
        join
        repeat (5) @(posedge clk);
        check_count(SOUTH, 1, "T7");
        check_count(EAST,  1, "T7");

        // T8: L 跟 N 都送到 E
        $display("\n[T8] L 和 N 同时 → E (round-robin)");
        clear_counts();
        fork
            send_single(LOCAL, 4'd2, 4'd3, 128'h33);
            send_single(NORTH, 4'd2, 4'd3, 128'h44);
        join
        repeat (8) @(posedge clk);
        check_count(EAST, 2, "T8");

        $display("\n============================================================");
        if (errors == 0)
            $display("  RESULT: PASS  (8/8 tests)");
        else
            $display("  RESULT: FAIL  errors=%0d", errors);
        $display("============================================================");
        $finish;
    end

    initial begin
        #50000 $display("WATCHDOG TIMEOUT"); $finish;
    end
endmodule
