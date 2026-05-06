`timescale 1ns/1ps

// =============================================================================
// tb_mesh_2x2.sv  --  2×2 mesh 集成测试 (AXI4-Stream)
// =============================================================================

module tb_mesh_2x2;
    timeunit 1ns; timeprecision 1ps;

    localparam int DATA_W = 128;
    localparam int DEST_W = 8;

    localparam int R00 = 0;
    localparam int R10 = 1;
    localparam int R01 = 2;
    localparam int R11 = 3;

    string r_name [4] = '{"R00", "R10", "R01", "R11"};
    int    r_x    [4] = '{0, 1, 0, 1};
    int    r_y    [4] = '{0, 0, 1, 1};

    logic clk = 0; always #5 clk = ~clk;
    logic rst_n = 0;

    logic [3:0]            s_axis_tvalid;
    logic [3:0]            s_axis_tready;
    logic [3:0]            s_axis_tlast;
    logic [DATA_W-1:0]     s_axis_tdata  [4];
    logic [DEST_W-1:0]     s_axis_tdest  [4];

    logic [3:0]            m_axis_tvalid;
    logic [3:0]            m_axis_tready = 4'b1111;
    logic [3:0]            m_axis_tlast;
    logic [DATA_W-1:0]     m_axis_tdata  [4];
    logic [DEST_W-1:0]     m_axis_tdest  [4];

    mesh_2x2 #(.DATA_W(DATA_W), .DEST_W(DEST_W)) u_mesh (
        .clk(clk), .rst_n(rst_n),
        .s_axis_local_tvalid(s_axis_tvalid),
        .s_axis_local_tready(s_axis_tready),
        .s_axis_local_tlast (s_axis_tlast),
        .s_axis_local_tdata (s_axis_tdata),
        .s_axis_local_tdest (s_axis_tdest),
        .m_axis_local_tvalid(m_axis_tvalid),
        .m_axis_local_tready(m_axis_tready),
        .m_axis_local_tlast (m_axis_tlast),
        .m_axis_local_tdata (m_axis_tdata),
        .m_axis_local_tdest (m_axis_tdest)
    );

    function automatic logic [DEST_W-1:0] make_dest(input logic [3:0] dy, dx);
        return {dy, dx};
    endfunction

    task automatic init_inputs();
        s_axis_tvalid = 4'd0;
        s_axis_tlast  = 4'd0;
        for (int i = 0; i < 4; i++) begin
            s_axis_tdata[i] = '0;
            s_axis_tdest[i] = '0;
        end
    endtask

    task automatic send_single_at(input int src_r, input int dst_r, input logic [127:0] data);
        @(posedge clk);
        s_axis_tvalid[src_r] = 1'b1;
        s_axis_tlast[src_r]  = 1'b1;
        s_axis_tdata[src_r]  = data;
        s_axis_tdest[src_r]  = make_dest(r_y[dst_r][3:0], r_x[dst_r][3:0]);
        do @(posedge clk); while (!s_axis_tready[src_r]);
        s_axis_tvalid[src_r] = 1'b0;
        s_axis_tlast[src_r]  = 1'b0;
    endtask

    task automatic send_packet_at(input int src_r, input int dst_r,
                                    input int n_flits, input logic [127:0] payload[]);
        for (int b = 0; b < n_flits; b++) begin
            @(posedge clk);
            s_axis_tvalid[src_r] = 1'b1;
            s_axis_tlast[src_r]  = (b == n_flits - 1) ? 1'b1 : 1'b0;
            s_axis_tdata[src_r]  = payload[b];
            s_axis_tdest[src_r]  = make_dest(r_y[dst_r][3:0], r_x[dst_r][3:0]);
            do @(posedge clk); while (!s_axis_tready[src_r]);
            s_axis_tvalid[src_r] = 1'b0;
            s_axis_tlast[src_r]  = 1'b0;
        end
    endtask

    int out_count [4];
    initial for (int i = 0; i < 4; i++) out_count[i] = 0;

    always @(posedge clk) begin
        if (rst_n) begin
            for (int p = 0; p < 4; p++) begin
                if (m_axis_tvalid[p] && m_axis_tready[p])
                    out_count[p] = out_count[p] + 1;
            end
        end
    end

    int errors = 0;
    task automatic check_count(input int r, input int expected, input string tag);
        if (out_count[r] != expected) begin
            $display("  FAIL %s: %s got %0d, expect %0d",
                     tag, r_name[r], out_count[r], expected);
            errors++;
        end else begin
            $display("  PASS %s: %s got %0d flit", tag, r_name[r], out_count[r]);
        end
    endtask

    task automatic clear_counts();
        for (int i = 0; i < 4; i++) out_count[i] = 0;
    endtask

    initial begin
        init_inputs();
        #20 rst_n = 1;
        #10;

        $display("== tb_mesh_2x2: AXI4-Stream 2x2 mesh ==");

        // T1: R00 → R10 (1 hop east)
        $display("\n[T1] R00.L → R10.L (1 hop east)");
        clear_counts();
        send_single_at(R00, R10, 128'h11);
        repeat (8) @(posedge clk);
        check_count(R10, 1, "T1");

        // T2: R00 → R01 (1 hop north)
        $display("\n[T2] R00.L → R01.L (1 hop north)");
        clear_counts();
        send_single_at(R00, R01, 128'h22);
        repeat (8) @(posedge clk);
        check_count(R01, 1, "T2");

        // T3: R00 → R11 (2 hop, XY)
        $display("\n[T3] R00.L → R11.L (2 hop, XY)");
        clear_counts();
        send_single_at(R00, R11, 128'h33);
        repeat (10) @(posedge clk);
        check_count(R11, 1, "T3");

        // T4: R11 → R00 (2 hop reverse)
        $display("\n[T4] R11.L → R00.L (2 hop reverse)");
        clear_counts();
        send_single_at(R11, R00, 128'h44);
        repeat (10) @(posedge clk);
        check_count(R00, 1, "T4");

        // T5: 4 LOCAL 顺时针环
        $display("\n[T5] 4 LOCAL 顺时针环 (R00→R10, R10→R11, R11→R01, R01→R00)");
        clear_counts();
        fork
            send_single_at(R00, R10, 128'h55);
            send_single_at(R10, R11, 128'h66);
            send_single_at(R11, R01, 128'h77);
            send_single_at(R01, R00, 128'h88);
        join
        repeat (15) @(posedge clk);
        check_count(R10, 1, "T5");
        check_count(R11, 1, "T5");
        check_count(R01, 1, "T5");
        check_count(R00, 1, "T5");

        // T6: 多 flit packet R00 → R11
        $display("\n[T6] R00 → R11 multi-flit (4 flit)");
        clear_counts();
        begin
            logic [127:0] payload [4];
            payload[0] = 128'h01;
            payload[1] = 128'hAAAAAAAA_BBBBBBBB_CCCCCCCC_DDDDDDDD;
            payload[2] = 128'h11111111_22222222_33333333_44444444;
            payload[3] = 128'hDEADBEEF_DEADBEEF_DEADBEEF_DEADBEEF;
            send_packet_at(R00, R11, 4, payload);
        end
        repeat (15) @(posedge clk);
        check_count(R11, 4, "T6");

        $display("\n============================================================");
        if (errors == 0)
            $display("  RESULT: PASS  (all mesh routing tests passed)");
        else
            $display("  RESULT: FAIL  errors=%0d", errors);
        $display("============================================================");
        $finish;
    end

    initial begin
        #50000 $display("WATCHDOG TIMEOUT"); $finish;
    end
endmodule
