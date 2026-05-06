`timescale 1ns/1ps

// =============================================================================
// tb_mesh_4x2_chain.sv  --  4 Mem + 4 Conv 4x2 mesh chain profile
//
// 拓扑 (mesh_4x2):
//   y=1   Conv0  Conv1  Conv2  Conv3      (idx 4..7)
//           │      │      │      │
//   y=0   Mem0   Mem1   Mem2   Mem3       (idx 0..3)
//
// 数据流 (per-layer):
//   Mem[c] → Conv[c] (1 hop south, 各核独立通道) → compute → Conv[c] → Mem[c]
//
// 这等价于 4-DDR PoC 的硬件实现:
//   - 4 个 Mem 各自独立带宽 (mesh 上不共享 link, 1 hop 直达)
//   - 期望整网 cycles ≈ max(per-core cycles), 跟 4-DDR PoC sim ~196K 一致
//
// 测试: 跟 5x1 同样 3 层配置, 对比 cycles + per-router fire.
// =============================================================================

module tb_mesh_4x2_chain;
    timeunit 1ns; timeprecision 1ps;

    localparam int DATA_W = 128;
    localparam int DEST_W = 8;
    localparam int ADDR_W = 20;
    localparam int N_CORES = 4;

    logic clk = 0; always #5 clk = ~clk;
    logic rst_n = 0;

    // ----- mesh 4x2 -----
    logic [7:0]            mesh_in_tvalid;
    logic [7:0]            mesh_in_tready;
    logic [7:0]            mesh_in_tlast;
    logic [DATA_W-1:0]     mesh_in_tdata  [8];
    logic [DEST_W-1:0]     mesh_in_tdest  [8];
    logic [7:0]            mesh_out_tvalid;
    logic [7:0]            mesh_out_tready;
    logic [7:0]            mesh_out_tlast;
    logic [DATA_W-1:0]     mesh_out_tdata  [8];
    logic [DEST_W-1:0]     mesh_out_tdest  [8];

    mesh_4x2 #(.DATA_W(DATA_W), .DEST_W(DEST_W)) u_mesh (
        .clk(clk), .rst_n(rst_n),
        .s_axis_local_tvalid(mesh_in_tvalid),
        .s_axis_local_tready(mesh_in_tready),
        .s_axis_local_tlast (mesh_in_tlast),
        .s_axis_local_tdata (mesh_in_tdata),
        .s_axis_local_tdest (mesh_in_tdest),
        .m_axis_local_tvalid(mesh_out_tvalid),
        .m_axis_local_tready(mesh_out_tready),
        .m_axis_local_tlast (mesh_out_tlast),
        .m_axis_local_tdata (mesh_out_tdata),
        .m_axis_local_tdest (mesh_out_tdest)
    );

    // ----- 4 Mem Core (idx 0..3) -----
    logic [N_CORES-1:0]  mem_cmd_start;
    logic [3:0]          mem_cmd_opcode      [N_CORES];
    logic [ADDR_W-1:0]   mem_cmd_ddr_addr    [N_CORES];
    logic [15:0]         mem_cmd_burst_len   [N_CORES];
    logic [ADDR_W-1:0]   mem_cmd_sram_offset [N_CORES];
    logic [DEST_W-1:0]   mem_cmd_tdest       [N_CORES];
    logic [N_CORES-1:0]  mem_cmd_busy;
    logic [N_CORES-1:0]  mem_cmd_done;
    logic [N_CORES-1:0]  mem_rx_pkt_done;

    genvar gm;
    generate
        for (gm = 0; gm < N_CORES; gm++) begin : g_mem
            mem_core_stub #(
                .DATA_W(DATA_W), .DEST_W(DEST_W), .ADDR_W(ADDR_W), .DDR_DEPTH(4096)
            ) u_mem (
                .clk(clk), .rst_n(rst_n),
                .s_axis_tvalid(mesh_out_tvalid[gm]),
                .s_axis_tready(mesh_out_tready[gm]),
                .s_axis_tdata (mesh_out_tdata [gm]),
                .s_axis_tlast (mesh_out_tlast [gm]),
                .s_axis_tdest (mesh_out_tdest [gm]),
                .m_axis_tvalid(mesh_in_tvalid[gm]),
                .m_axis_tready(mesh_in_tready[gm]),
                .m_axis_tdata (mesh_in_tdata [gm]),
                .m_axis_tlast (mesh_in_tlast [gm]),
                .m_axis_tdest (mesh_in_tdest [gm]),
                .cmd_start(mem_cmd_start[gm]),
                .cmd_opcode(mem_cmd_opcode[gm]),
                .cmd_ddr_addr(mem_cmd_ddr_addr[gm]),
                .cmd_burst_len(mem_cmd_burst_len[gm]),
                .cmd_sram_offset(mem_cmd_sram_offset[gm]),
                .cmd_tdest(mem_cmd_tdest[gm]),
                .cmd_busy(mem_cmd_busy[gm]),
                .cmd_done(mem_cmd_done[gm]),
                .last_rx_opcode(),
                .last_rx_burst_len(),
                .last_rx_addr(),
                .rx_pkt_done(mem_rx_pkt_done[gm])
            );
        end
    endgenerate

    // ----- 4 ConvCore (idx 4..7) -----
    logic [DEST_W-1:0]   conv_ofm_dst   [N_CORES];
    logic [3:0]          conv_ofm_opcode[N_CORES];
    logic [15:0]         conv_compute_delay [N_CORES];
    logic [N_CORES-1:0]  conv_compute_done;

    genvar gc;
    generate
        for (gc = 0; gc < N_CORES; gc++) begin : g_conv
            conv_core_stub #(
                .DATA_W(DATA_W), .DEST_W(DEST_W), .ADDR_W(ADDR_W)
            ) u_conv (
                .clk(clk), .rst_n(rst_n),
                .core_id(8'(gc)),
                .compute_delay(conv_compute_delay[gc]),
                .s_axis_tvalid(mesh_out_tvalid[gc + 4]),    // idx 4..7
                .s_axis_tready(mesh_out_tready[gc + 4]),
                .s_axis_tdata (mesh_out_tdata [gc + 4]),
                .s_axis_tlast (mesh_out_tlast [gc + 4]),
                .s_axis_tdest (mesh_out_tdest [gc + 4]),
                .m_axis_tvalid(mesh_in_tvalid[gc + 4]),
                .m_axis_tready(mesh_in_tready[gc + 4]),
                .m_axis_tdata (mesh_in_tdata [gc + 4]),
                .m_axis_tlast (mesh_in_tlast [gc + 4]),
                .m_axis_tdest (mesh_in_tdest [gc + 4]),
                .start_compute(1'b0),
                .ofm_dst(conv_ofm_dst[gc]),
                .ofm_opcode(conv_ofm_opcode[gc]),
                .compute_done(conv_compute_done[gc]),
                .last_rx_opcode(),
                .last_rx_burst_len(),
                .last_rx_addr()
            );
        end
    endgenerate

    // ----- Layer 配置 (跟 5x1 一致, 便于对比) -----
    localparam int N_LAYERS = 3;
    int layer_burst_len   [N_LAYERS] = '{32, 16, 64};
    int layer_compute     [N_LAYERS] = '{200, 400, 300};

    // ----- Per-Mem RX 计数 -----
    int mem_rx_pkt_count [N_CORES];
    always @(posedge clk) begin
        if (rst_n) begin
            for (int c = 0; c < N_CORES; c++)
                if (mem_rx_pkt_done[c]) mem_rx_pkt_count[c] = mem_rx_pkt_count[c] + 1;
        end
    end

    // ----- TB 主流程 -----
    longint t_start;
    longint t_layer_start [N_LAYERS];
    longint t_layer_end   [N_LAYERS];

    function automatic logic [DEST_W-1:0] make_dest(input int x, y);
        return {y[3:0], x[3:0]};
    endfunction

    initial begin
        for (int c = 0; c < N_CORES; c++) begin
            mem_cmd_start[c]       = 0;
            mem_cmd_opcode[c]      = 0;
            mem_cmd_ddr_addr[c]    = 0;
            mem_cmd_burst_len[c]   = 0;
            mem_cmd_sram_offset[c] = 0;
            mem_cmd_tdest[c]       = 0;
            // Conv[c] 的 OFM 目标 = 对应 Mem[c] (x=c, y=0)
            conv_ofm_dst[c]        = make_dest(c, 0);
            conv_ofm_opcode[c]     = 4'h5;
            conv_compute_delay[c]  = 0;
            mem_rx_pkt_count[c]    = 0;
        end

        // 初始化 4 个 Mem 的 DDR mock (用 hierarchical 直接给每个 mem)
        for (int i = 0; i < 256; i++) begin
            g_mem[0].u_mem.ddr_mem[i] = {32'h0, 32'h0, 32'd0, 32'(i)};
            g_mem[1].u_mem.ddr_mem[i] = {32'h0, 32'h0, 32'd1, 32'(i)};
            g_mem[2].u_mem.ddr_mem[i] = {32'h0, 32'h0, 32'd2, 32'(i)};
            g_mem[3].u_mem.ddr_mem[i] = {32'h0, 32'h0, 32'd3, 32'(i)};
        end

        #20 rst_n = 1;
        #10;

        $display("============================================================");
        $display("== tb_mesh_4x2_chain: %0d-layer chain (4 Mem ↔ 4 Conv, 1 hop) ==", N_LAYERS);
        $display("============================================================");

        t_start = $time;

        for (int l = 0; l < N_LAYERS; l++) begin : layer_loop
            automatic int burst   = layer_burst_len[l];
            automatic int compute = layer_compute[l];
            automatic int rx_targets [N_CORES];
            t_layer_start[l] = $time;

            $display("\n[Layer %0d] burst_len=%0d/core, compute_delay=%0d cy",
                      l, burst, compute);

            for (int c = 0; c < N_CORES; c++) begin
                conv_compute_delay[c] = 16'(compute);
                rx_targets[c] = mem_rx_pkt_count[c] + 1;
            end

            // 4 Mem 并行发 IFB packet 到对应 Conv (1 hop south)
            // Mem[c] 在 (c, 0), Conv[c] 在 (c, 1), 用 make_dest(c, 1) 路由到 Conv[c]
            @(posedge clk);
            for (int c = 0; c < N_CORES; c++) begin
                mem_cmd_start[c]       = 1'b1;
                mem_cmd_opcode[c]      = 4'h0;          // WRITE_IFB
                mem_cmd_ddr_addr[c]    = ADDR_W'(0);
                mem_cmd_burst_len[c]   = 16'(burst);
                mem_cmd_sram_offset[c] = ADDR_W'(0);
                mem_cmd_tdest[c]       = make_dest(c, 1);   // → Conv[c]
            end
            @(posedge clk);
            for (int c = 0; c < N_CORES; c++) mem_cmd_start[c] = 1'b0;

            // 等所有 4 OFM 都被各自 Mem 收到
            wait ((mem_rx_pkt_count[0] >= rx_targets[0]) &&
                  (mem_rx_pkt_count[1] >= rx_targets[1]) &&
                  (mem_rx_pkt_count[2] >= rx_targets[2]) &&
                  (mem_rx_pkt_count[3] >= rx_targets[3]));
            @(posedge clk); @(posedge clk);
            t_layer_end[l] = $time;

            $display("  Layer %0d done @ t=%0t (cycles=%0d)",
                      l, $time, ($time - t_layer_start[l]) / 10);
        end

        $display("\n============================================================");
        $display("  Total wall: %0d ns (%0d cycles @ 10 ns)",
                  $time - t_start, ($time - t_start) / 10);
        $display("============================================================");

        $display("\n[Per-layer cycles]");
        for (int l = 0; l < N_LAYERS; l++) begin
            automatic int cyc = (t_layer_end[l] - t_layer_start[l]) / 10;
            $display("  Layer %0d: %0d cycles (burst=%0d, compute=%0d)",
                      l, cyc, layer_burst_len[l], layer_compute[l]);
        end

        $display("\n[Per-router fire count]  (idx = y*4 + x)");
        $display("  idx (x,y)  in_LOCAL  in_N   in_S   in_E   in_W   out_LOCAL");
        for (int idx = 0; idx < 8; idx++) begin
            automatic int in_l, in_n, in_s, in_e, in_w, out_l;
            automatic int x = idx % 4;
            automatic int y = idx / 4;
            case (idx)
                0: begin
                    in_l  = u_mesh.g_router[0].u_r.in_fire[0];
                    in_n  = u_mesh.g_router[0].u_r.in_fire[1];
                    in_s  = u_mesh.g_router[0].u_r.in_fire[2];
                    in_e  = u_mesh.g_router[0].u_r.in_fire[3];
                    in_w  = u_mesh.g_router[0].u_r.in_fire[4];
                    out_l = u_mesh.g_router[0].u_r.out_fire[0];
                end
                1: begin
                    in_l  = u_mesh.g_router[1].u_r.in_fire[0];
                    in_n  = u_mesh.g_router[1].u_r.in_fire[1];
                    in_s  = u_mesh.g_router[1].u_r.in_fire[2];
                    in_e  = u_mesh.g_router[1].u_r.in_fire[3];
                    in_w  = u_mesh.g_router[1].u_r.in_fire[4];
                    out_l = u_mesh.g_router[1].u_r.out_fire[0];
                end
                2: begin
                    in_l  = u_mesh.g_router[2].u_r.in_fire[0];
                    in_n  = u_mesh.g_router[2].u_r.in_fire[1];
                    in_s  = u_mesh.g_router[2].u_r.in_fire[2];
                    in_e  = u_mesh.g_router[2].u_r.in_fire[3];
                    in_w  = u_mesh.g_router[2].u_r.in_fire[4];
                    out_l = u_mesh.g_router[2].u_r.out_fire[0];
                end
                3: begin
                    in_l  = u_mesh.g_router[3].u_r.in_fire[0];
                    in_n  = u_mesh.g_router[3].u_r.in_fire[1];
                    in_s  = u_mesh.g_router[3].u_r.in_fire[2];
                    in_e  = u_mesh.g_router[3].u_r.in_fire[3];
                    in_w  = u_mesh.g_router[3].u_r.in_fire[4];
                    out_l = u_mesh.g_router[3].u_r.out_fire[0];
                end
                4: begin
                    in_l  = u_mesh.g_router[4].u_r.in_fire[0];
                    in_n  = u_mesh.g_router[4].u_r.in_fire[1];
                    in_s  = u_mesh.g_router[4].u_r.in_fire[2];
                    in_e  = u_mesh.g_router[4].u_r.in_fire[3];
                    in_w  = u_mesh.g_router[4].u_r.in_fire[4];
                    out_l = u_mesh.g_router[4].u_r.out_fire[0];
                end
                5: begin
                    in_l  = u_mesh.g_router[5].u_r.in_fire[0];
                    in_n  = u_mesh.g_router[5].u_r.in_fire[1];
                    in_s  = u_mesh.g_router[5].u_r.in_fire[2];
                    in_e  = u_mesh.g_router[5].u_r.in_fire[3];
                    in_w  = u_mesh.g_router[5].u_r.in_fire[4];
                    out_l = u_mesh.g_router[5].u_r.out_fire[0];
                end
                6: begin
                    in_l  = u_mesh.g_router[6].u_r.in_fire[0];
                    in_n  = u_mesh.g_router[6].u_r.in_fire[1];
                    in_s  = u_mesh.g_router[6].u_r.in_fire[2];
                    in_e  = u_mesh.g_router[6].u_r.in_fire[3];
                    in_w  = u_mesh.g_router[6].u_r.in_fire[4];
                    out_l = u_mesh.g_router[6].u_r.out_fire[0];
                end
                7: begin
                    in_l  = u_mesh.g_router[7].u_r.in_fire[0];
                    in_n  = u_mesh.g_router[7].u_r.in_fire[1];
                    in_s  = u_mesh.g_router[7].u_r.in_fire[2];
                    in_e  = u_mesh.g_router[7].u_r.in_fire[3];
                    in_w  = u_mesh.g_router[7].u_r.in_fire[4];
                    out_l = u_mesh.g_router[7].u_r.out_fire[0];
                end
            endcase
            $display("  %0d (%0d,%0d)   %5d   %5d  %5d  %5d  %5d   %5d",
                      idx, x, y, in_l, in_n, in_s, in_e, in_w, out_l);
        end

        // Per-Mem util
        begin
            automatic int total_cyc = ($time - t_start) / 10;
            $display("\n[Per-Mem link util]");
            for (int c = 0; c < N_CORES; c++) begin
                automatic int tx_fire = (c == 0) ? u_mesh.g_router[0].u_r.in_fire[0] :
                                         (c == 1) ? u_mesh.g_router[1].u_r.in_fire[0] :
                                         (c == 2) ? u_mesh.g_router[2].u_r.in_fire[0] :
                                                    u_mesh.g_router[3].u_r.in_fire[0];
                automatic int rx_fire = (c == 0) ? u_mesh.g_router[0].u_r.out_fire[0] :
                                         (c == 1) ? u_mesh.g_router[1].u_r.out_fire[0] :
                                         (c == 2) ? u_mesh.g_router[2].u_r.out_fire[0] :
                                                    u_mesh.g_router[3].u_r.out_fire[0];
                $display("  Mem%0d: TX=%0d (%.1f%%)  RX=%0d (%.1f%%)",
                          c, tx_fire, real'(tx_fire) / real'(total_cyc) * 100.0,
                             rx_fire, real'(rx_fire) / real'(total_cyc) * 100.0);
            end
        end

        $display("\n============================================================");
        $display("  RESULT: PASS  (4x2 mesh chain profile completed)");
        $display("============================================================");
        $finish;
    end

    initial begin
        #2000000 $display("WATCHDOG TIMEOUT"); $finish;
    end
endmodule
