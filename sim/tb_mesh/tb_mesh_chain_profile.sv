`timescale 1ns/1ps

// =============================================================================
// tb_mesh_chain_profile.sv  --  4-core mesh + Mem Core 多层 chain 性能 profile
//
// 目的:
//   1. 模拟 ResNet 类工作负载: N 层 chain, 每层 Mem 推 IFB 给 4 ConvCore, ConvCore
//      自动 compute (compute_delay 拍) 后发 OFM 回 Mem.
//   2. 测系统瓶颈: 总 cycles, per-router fire%, Mem 热点 saturation.
//
// 拓扑: 5x1 mesh, x=0=Mem, x=1..4=Conv0..3
//
// Layer 配置 (模拟 ResNet 早期 layer, 切片 4 核):
//   L0: 32 word/core,  compute=200 cy   (大 IFB, 短 compute = BW bound)
//   L1: 16 word/core,  compute=400 cy   (compute bound)
//   L2: 64 word/core,  compute=300 cy   (中等)
//
// 流程 (每层):
//   1. Mem 顺序发 4 个 IFB packet 到 4 ConvCore
//   2. 4 ConvCore 自动 compute_delay 拍后, 各自发 OFM 回 Mem
//   3. Mem 收完 4 个 OFM packet → 进下一层
//
// 输出: 总 cycles, per-layer cycles, per-router fire counts, Mem 端 cycles utilization.
// =============================================================================

module tb_mesh_chain_profile;
    timeunit 1ns; timeprecision 1ps;

    localparam int DATA_W = 128;
    localparam int DEST_W = 8;
    localparam int ADDR_W = 20;
    localparam int N_CORES = 4;
    localparam int MEM_IDX = 0;

    logic clk = 0; always #5 clk = ~clk;
    logic rst_n = 0;

    // ----- mesh 5x1 -----
    logic [4:0]            mesh_in_tvalid;
    logic [4:0]            mesh_in_tready;
    logic [4:0]            mesh_in_tlast;
    logic [DATA_W-1:0]     mesh_in_tdata  [5];
    logic [DEST_W-1:0]     mesh_in_tdest  [5];
    logic [4:0]            mesh_out_tvalid;
    logic [4:0]            mesh_out_tready;
    logic [4:0]            mesh_out_tlast;
    logic [DATA_W-1:0]     mesh_out_tdata  [5];
    logic [DEST_W-1:0]     mesh_out_tdest  [5];

    mesh_5x1 #(.DATA_W(DATA_W), .DEST_W(DEST_W)) u_mesh (
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

    // ----- Mem Core @ idx 0 -----
    logic                mem_cmd_start;
    logic [3:0]          mem_cmd_opcode;
    logic [ADDR_W-1:0]   mem_cmd_ddr_addr;
    logic [15:0]         mem_cmd_burst_len;
    logic [ADDR_W-1:0]   mem_cmd_sram_offset;
    logic [DEST_W-1:0]   mem_cmd_tdest;
    logic                mem_cmd_busy;
    logic                mem_cmd_done;
    logic [3:0]          mem_last_rx_opcode;
    logic [15:0]         mem_last_rx_burst_len;
    logic [ADDR_W-1:0]   mem_last_rx_addr;
    logic                mem_rx_pkt_done;

    mem_core_stub #(
        .DATA_W(DATA_W), .DEST_W(DEST_W), .ADDR_W(ADDR_W), .DDR_DEPTH(8192)
    ) u_mem (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tvalid(mesh_out_tvalid[MEM_IDX]),
        .s_axis_tready(mesh_out_tready[MEM_IDX]),
        .s_axis_tdata (mesh_out_tdata [MEM_IDX]),
        .s_axis_tlast (mesh_out_tlast [MEM_IDX]),
        .s_axis_tdest (mesh_out_tdest [MEM_IDX]),
        .m_axis_tvalid(mesh_in_tvalid[MEM_IDX]),
        .m_axis_tready(mesh_in_tready[MEM_IDX]),
        .m_axis_tdata (mesh_in_tdata [MEM_IDX]),
        .m_axis_tlast (mesh_in_tlast [MEM_IDX]),
        .m_axis_tdest (mesh_in_tdest [MEM_IDX]),
        .cmd_start(mem_cmd_start), .cmd_opcode(mem_cmd_opcode),
        .cmd_ddr_addr(mem_cmd_ddr_addr), .cmd_burst_len(mem_cmd_burst_len),
        .cmd_sram_offset(mem_cmd_sram_offset), .cmd_tdest(mem_cmd_tdest),
        .cmd_busy(mem_cmd_busy), .cmd_done(mem_cmd_done),
        .last_rx_opcode(mem_last_rx_opcode), .last_rx_burst_len(mem_last_rx_burst_len),
        .last_rx_addr(mem_last_rx_addr), .rx_pkt_done(mem_rx_pkt_done)
    );

    // ----- 4 ConvCore @ idx 1..4 -----
    logic [DEST_W-1:0]   conv_ofm_dst   [N_CORES];
    logic [3:0]          conv_ofm_opcode[N_CORES];
    logic [15:0]         conv_compute_delay [N_CORES];
    logic [N_CORES-1:0]  conv_compute_done;
    logic [3:0]          conv_last_rx_opcode  [N_CORES];
    logic [15:0]         conv_last_rx_burst_len[N_CORES];
    logic [ADDR_W-1:0]   conv_last_rx_addr    [N_CORES];

    genvar gc;
    generate
        for (gc = 0; gc < N_CORES; gc++) begin : g_conv
            conv_core_stub #(
                .DATA_W(DATA_W), .DEST_W(DEST_W), .ADDR_W(ADDR_W)
            ) u_conv (
                .clk(clk), .rst_n(rst_n),
                .core_id(8'(gc)),
                .compute_delay(conv_compute_delay[gc]),
                .s_axis_tvalid(mesh_out_tvalid[gc+1]),
                .s_axis_tready(mesh_out_tready[gc+1]),
                .s_axis_tdata (mesh_out_tdata [gc+1]),
                .s_axis_tlast (mesh_out_tlast [gc+1]),
                .s_axis_tdest (mesh_out_tdest [gc+1]),
                .m_axis_tvalid(mesh_in_tvalid[gc+1]),
                .m_axis_tready(mesh_in_tready[gc+1]),
                .m_axis_tdata (mesh_in_tdata [gc+1]),
                .m_axis_tlast (mesh_in_tlast [gc+1]),
                .m_axis_tdest (mesh_in_tdest [gc+1]),
                .start_compute(1'b0),                  // 内部自动触发, 不用 TB
                .ofm_dst(conv_ofm_dst[gc]),
                .ofm_opcode(conv_ofm_opcode[gc]),
                .compute_done(conv_compute_done[gc]),
                .last_rx_opcode(conv_last_rx_opcode[gc]),
                .last_rx_burst_len(conv_last_rx_burst_len[gc]),
                .last_rx_addr(conv_last_rx_addr[gc])
            );
        end
    endgenerate

    // ----- Layer 配置 -----
    localparam int N_LAYERS = 3;
    int layer_burst_len   [N_LAYERS] = '{32, 16, 64};   // word/core
    int layer_compute     [N_LAYERS] = '{200, 400, 300}; // 等待拍数

    // ----- TB tasks -----
    task automatic mem_send_cmd(input logic [3:0] op, input int ddr_addr,
                                  input int burst, input int sram_off,
                                  input logic [DEST_W-1:0] dst);
        @(posedge clk);
        mem_cmd_start       = 1'b1;
        mem_cmd_opcode      = op;
        mem_cmd_ddr_addr    = ADDR_W'(ddr_addr);
        mem_cmd_burst_len   = 16'(burst);
        mem_cmd_sram_offset = ADDR_W'(sram_off);
        mem_cmd_tdest       = dst;
        @(posedge clk);
        mem_cmd_start       = 1'b0;
        do @(posedge clk); while (!mem_cmd_done);
    endtask

    function automatic logic [DEST_W-1:0] make_dest(input int x, y);
        return {y[3:0], x[3:0]};
    endfunction

    // ----- 计数 Mem RX 完成的 packet 数 (用来判定一层的 OFM 全收到) -----
    int mem_rx_pkt_count;
    always @(posedge clk) begin
        if (rst_n && mem_rx_pkt_done) mem_rx_pkt_count = mem_rx_pkt_count + 1;
    end

    // ----- 主流程 -----
    longint t_start;
    longint t_layer_start [N_LAYERS];
    longint t_layer_end   [N_LAYERS];

    initial begin
        mem_cmd_start       = 0;
        mem_cmd_opcode      = 0;
        mem_cmd_ddr_addr    = 0;
        mem_cmd_burst_len   = 0;
        mem_cmd_sram_offset = 0;
        mem_cmd_tdest       = 0;
        mem_rx_pkt_count    = 0;
        for (int c = 0; c < N_CORES; c++) begin
            conv_ofm_dst[c]       = make_dest(0, 0);   // 全部 OFM 写回 Mem (0,0)
            conv_ofm_opcode[c]    = 4'h5;              // WRITE_DDR_OFB
            conv_compute_delay[c] = 0;
        end

        // 初始化 Mem DDR mock (各核 region 给随机数据)
        // 每核 region: ddr[c*0x100 .. c*0x100 + max_burst]
        // OFM 写回区: ddr[c*0x100 + 0x800 ..]  (跟 IFB 区不冲突)
        // 简化: IFB 跟 OFM 共用同一区, OFM 覆盖 IFB (反正不检查 bit-exact)
        for (int c = 0; c < N_CORES; c++)
            for (int i = 0; i < 256; i++)
                u_mem.ddr_mem[c * 'h100 + i] = {32'h0, 32'h0, 32'(c), 32'(i)};

        #20 rst_n = 1;
        #10;

        $display("============================================================");
        $display("== tb_mesh_chain_profile: %0d-layer chain (Mem→4Conv→Mem) ==", N_LAYERS);
        $display("============================================================");

        t_start = $time;

        for (int l = 0; l < N_LAYERS; l++) begin : layer_loop
            automatic int burst   = layer_burst_len[l];
            automatic int compute = layer_compute[l];
            automatic int rx_target = mem_rx_pkt_count + 4;
            t_layer_start[l] = $time;

            $display("\n[Layer %0d] burst_len=%0d/core, compute_delay=%0d cy",
                      l, burst, compute);

            // 设置 ConvCore compute_delay
            for (int c = 0; c < N_CORES; c++)
                conv_compute_delay[c] = 16'(compute);

            // Mem 顺序发 4 个 IFB packet 到 4 ConvCore
            for (int c = 0; c < N_CORES; c++) begin : mem_send
                automatic int ddr_addr = c * 'h100;
                mem_send_cmd(4'h0, ddr_addr, burst, ddr_addr, make_dest(c + 1, 0));
            end

            // 等所有 4 OFM 都被 Mem 收到 (mem_rx_pkt_count 增加 4)
            wait (mem_rx_pkt_count >= rx_target);
            @(posedge clk); @(posedge clk);
            t_layer_end[l] = $time;

            $display("  Layer %0d done @ t=%0t (cycles=%0d)",
                      l, $time, ($time - t_layer_start[l]) / 10);
        end

        $display("\n============================================================");
        $display("  Total wall: %0d ns (%0d cycles @ 10 ns)",
                  $time - t_start, ($time - t_start) / 10);
        $display("============================================================");

        // ===== Profiling 报告 =====
        $display("\n[Per-layer cycles]");
        for (int l = 0; l < N_LAYERS; l++) begin
            automatic int cyc = (t_layer_end[l] - t_layer_start[l]) / 10;
            $display("  Layer %0d: %0d cycles (burst=%0d, compute_delay=%0d)",
                      l, cyc, layer_burst_len[l], layer_compute[l]);
        end

        $display("\n[Per-router fire count]");
        $display("  router  in_LOCAL  in_W   in_E   out_LOCAL  out_W  out_E");
        // 5 个 router (x=0..4)
        for (int x = 0; x < 5; x++) begin
            automatic int in_l, in_w, in_e, out_l, out_w, out_e;
            case (x)
                0: begin
                    in_l  = u_mesh.g_router[0].u_r.in_fire[0];
                    in_w  = u_mesh.g_router[0].u_r.in_fire[4];
                    in_e  = u_mesh.g_router[0].u_r.in_fire[3];
                    out_l = u_mesh.g_router[0].u_r.out_fire[0];
                    out_w = u_mesh.g_router[0].u_r.out_fire[4];
                    out_e = u_mesh.g_router[0].u_r.out_fire[3];
                end
                1: begin
                    in_l  = u_mesh.g_router[1].u_r.in_fire[0];
                    in_w  = u_mesh.g_router[1].u_r.in_fire[4];
                    in_e  = u_mesh.g_router[1].u_r.in_fire[3];
                    out_l = u_mesh.g_router[1].u_r.out_fire[0];
                    out_w = u_mesh.g_router[1].u_r.out_fire[4];
                    out_e = u_mesh.g_router[1].u_r.out_fire[3];
                end
                2: begin
                    in_l  = u_mesh.g_router[2].u_r.in_fire[0];
                    in_w  = u_mesh.g_router[2].u_r.in_fire[4];
                    in_e  = u_mesh.g_router[2].u_r.in_fire[3];
                    out_l = u_mesh.g_router[2].u_r.out_fire[0];
                    out_w = u_mesh.g_router[2].u_r.out_fire[4];
                    out_e = u_mesh.g_router[2].u_r.out_fire[3];
                end
                3: begin
                    in_l  = u_mesh.g_router[3].u_r.in_fire[0];
                    in_w  = u_mesh.g_router[3].u_r.in_fire[4];
                    in_e  = u_mesh.g_router[3].u_r.in_fire[3];
                    out_l = u_mesh.g_router[3].u_r.out_fire[0];
                    out_w = u_mesh.g_router[3].u_r.out_fire[4];
                    out_e = u_mesh.g_router[3].u_r.out_fire[3];
                end
                4: begin
                    in_l  = u_mesh.g_router[4].u_r.in_fire[0];
                    in_w  = u_mesh.g_router[4].u_r.in_fire[4];
                    in_e  = u_mesh.g_router[4].u_r.in_fire[3];
                    out_l = u_mesh.g_router[4].u_r.out_fire[0];
                    out_w = u_mesh.g_router[4].u_r.out_fire[4];
                    out_e = u_mesh.g_router[4].u_r.out_fire[3];
                end
            endcase
            $display("  x=%0d     %5d   %5d  %5d   %5d      %5d  %5d",
                      x, in_l, in_w, in_e, out_l, out_w, out_e);
        end

        // Mem 单点利用率
        begin
            automatic int total_cyc = ($time - t_start) / 10;
            automatic int mem_in_l  = u_mesh.g_router[0].u_r.in_fire[0];
            automatic int mem_out_l = u_mesh.g_router[0].u_r.out_fire[0];
            $display("\n[Mem Core (router 0) link util]");
            $display("  Mem TX out (Mem→mesh):  %0d fire / %0d cycles = %.1f%%",
                      mem_in_l, total_cyc, real'(mem_in_l) / real'(total_cyc) * 100.0);
            $display("  Mem RX in  (mesh→Mem):  %0d fire / %0d cycles = %.1f%%",
                      mem_out_l, total_cyc, real'(mem_out_l) / real'(total_cyc) * 100.0);
        end

        $display("\n============================================================");
        $display("  RESULT: PASS  (mesh chain profile completed)");
        $display("============================================================");
        $finish;
    end

    initial begin
        #2000000 $display("WATCHDOG TIMEOUT"); $finish;
    end
endmodule
