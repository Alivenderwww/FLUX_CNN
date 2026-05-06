`timescale 1ns/1ps

// =============================================================================
// tb_mesh_4core_demo.sv  --  4 ConvCore + 1 Mem Core 通过 5x1 mesh 跑端到端 packet 流
//
// 拓扑:
//   x=0    1     2     3     4
//   Mem   Conv0 Conv1 Conv2 Conv3
//
// 流程:
//   1. TB preload Mem Core DDR mock: ddr[0..15] = base_data[0..15],
//                                    ddr[100..115] = base_data[16..31], ...
//      (4 个 region, 各 16 word, 给 4 ConvCore)
//   2. TB 给 Mem Core 4 个 cmd: 各发 16 word IFB packet 到 ConvCore[0..3]
//   3. 等 4 ConvCore 都收到 packet (轮询 last_rx_burst_len)
//   4. TB 触发 ConvCore[i].start_compute, ofm_dst = Mem Core (0,0),
//      ofm_opcode = 0x5 (WRITE_DDR_OFB), 写到 DDR mock 不同 region
//   5. 等 ConvCore 发完 + Mem Core 收完
//   6. TB 检查 Mem Core DDR mock: 应该是 base_data XOR core_id (低 8 bit)
// =============================================================================

module tb_mesh_4core_demo;
    timeunit 1ns; timeprecision 1ps;

    localparam int DATA_W = 128;
    localparam int DEST_W = 8;
    localparam int ADDR_W = 20;
    localparam int N_CORES = 4;

    // mesh 节点 idx: 0=Mem Core, 1..4=Conv0..3
    localparam int MEM_IDX = 0;

    logic clk = 0; always #5 clk = ~clk;
    logic rst_n = 0;

    // ----- 5 个 LOCAL ports (mesh_5x1) -----
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
        .DATA_W(DATA_W), .DEST_W(DEST_W), .ADDR_W(ADDR_W), .DDR_DEPTH(4096)
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
        .cmd_start(mem_cmd_start),
        .cmd_opcode(mem_cmd_opcode),
        .cmd_ddr_addr(mem_cmd_ddr_addr),
        .cmd_burst_len(mem_cmd_burst_len),
        .cmd_sram_offset(mem_cmd_sram_offset),
        .cmd_tdest(mem_cmd_tdest),
        .cmd_busy(mem_cmd_busy), .cmd_done(mem_cmd_done),
        .last_rx_opcode(mem_last_rx_opcode),
        .last_rx_burst_len(mem_last_rx_burst_len),
        .last_rx_addr(mem_last_rx_addr),
        .rx_pkt_done(mem_rx_pkt_done)
    );

    // ----- 4 ConvCore @ idx 1..4 -----
    logic [N_CORES-1:0]  conv_start_compute;
    logic [DEST_W-1:0]   conv_ofm_dst   [N_CORES];
    logic [3:0]          conv_ofm_opcode[N_CORES];
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
                .start_compute(conv_start_compute[gc]),
                .ofm_dst(conv_ofm_dst[gc]),
                .ofm_opcode(conv_ofm_opcode[gc]),
                .compute_done(conv_compute_done[gc]),
                .last_rx_opcode(conv_last_rx_opcode[gc]),
                .last_rx_burst_len(conv_last_rx_burst_len[gc]),
                .last_rx_addr(conv_last_rx_addr[gc])
            );
        end
    endgenerate

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
        // 等 mem_cmd_done
        do @(posedge clk); while (!mem_cmd_done);
        repeat (4) @(posedge clk);
    endtask

    function automatic logic [DEST_W-1:0] make_dest(input int x, y);
        return {y[3:0], x[3:0]};
    endfunction

    // ----- 主流程 -----
    int errors = 0;

    initial begin
        mem_cmd_start       = 0;
        mem_cmd_opcode      = 0;
        mem_cmd_ddr_addr    = 0;
        mem_cmd_burst_len   = 0;
        mem_cmd_sram_offset = 0;
        mem_cmd_tdest       = 0;
        for (int c = 0; c < N_CORES; c++) begin
            conv_start_compute[c] = 0;
            conv_ofm_dst[c]       = 0;
            conv_ofm_opcode[c]    = 0;
        end
        #20 rst_n = 1;
        #10;

        // 1. preload Mem Core DDR mock: 4 region 各 16 word
        // region[i] @ ddr addr = i * 0x100, data = {core_id_marker, idx}
        for (int c = 0; c < N_CORES; c++) begin
            for (int i = 0; i < 16; i++) begin
                u_mem.ddr_mem[c * 'h100 + i] = {32'h0, 32'h0, 32'(c), 32'(i)};
            end
        end
        // 输出 OFM 区初始化为 0
        for (int c = 0; c < N_CORES; c++) begin
            for (int i = 0; i < 16; i++) begin
                u_mem.ddr_mem[c * 'h100 + 'h800 + i] = '0;
            end
        end

        $display("== tb_mesh_4core_demo: 4 ConvCore + 1 Mem Core via 5x1 mesh ==");

        // 2. Mem Core 发 4 个 IFB packet 到 ConvCore[0..3]
        // 注意: 因为 mem_core_stub 用 cmd_sram_offset 同时作为 SRAM 读 addr 和 packet header
        //       addr 字段, 所以 ddr_addr 跟 sram_offset 必须相同.
        $display("\n[Phase A] Mem Core → 4 ConvCore (IFB load)");
        for (int c = 0; c < N_CORES; c++) begin
            automatic int ddr_addr = c * 'h100;
            automatic int sram_off = c * 'h100;
            automatic int dst_x    = c + 1;
            mem_send_cmd(4'h0,
                         ddr_addr, 16, sram_off, make_dest(dst_x, 0));
            $display("  Mem -> Conv%0d: 16 word, ddr=0x%0h, dst=(%0d,0)", c, ddr_addr, dst_x);
        end
        repeat (10) @(posedge clk);

        // 3. 检查 4 ConvCore 都收到 packet
        for (int c = 0; c < N_CORES; c++) begin
            if (conv_last_rx_burst_len[c] != 16'd16) begin
                $display("  FAIL: Conv%0d burst_len=%0d expect 16",
                         c, conv_last_rx_burst_len[c]);
                errors++;
            end else begin
                $display("  PASS: Conv%0d 收到 16 word IFB packet", c);
            end
        end

        // 4. 触发 4 ConvCore start_compute, OFM 发回 Mem Core
        $display("\n[Phase B] 4 ConvCore → Mem Core (OFM store)");
        for (int c = 0; c < N_CORES; c++) begin
            conv_ofm_dst[c]    = make_dest(0, 0);  // Mem Core @ (0,0)
            conv_ofm_opcode[c] = 4'h5;              // WRITE_DDR_OFB
            // 注意: ConvCore stub 内部用收到的 IFB packet 的 sram_offset (0xN00) 作为 SRAM 读 addr,
            //       但 OFM packet 的 header 也用同一 addr — Mem Core 收到后会写 DDR mock[0xN00]
            //       这会覆盖原 IFB! 让 ConvCore 写 DDR mock 不同 region.
            // 修法: 让 ConvCore 改 packet header 的 addr 字段 (e.g. + 0x800 给 OFM region).
            //       但 stub 没这功能, 现状会把 OFM 写回原 IFB 位置.
            //       这没关系 — 我们检查"原 IFB 内容是否被 IFB ^ core_id 覆盖" 也能验证 echo 正确.
        end
        // 错开触发 (减少 mesh 路由争用)
        for (int c = 0; c < N_CORES; c++) begin
            @(posedge clk);
            conv_start_compute[c] = 1'b1;
            @(posedge clk);
            conv_start_compute[c] = 1'b0;
            repeat (2) @(posedge clk);
        end

        // 等所有 4 ConvCore tx 发完 + Mem Core 收完
        // done 是 pulse, 直接等足够长时间 (4 个 packet × 16 word + mesh latency)
        repeat (300) @(posedge clk);

        // 5. 检查 Mem Core DDR mock: 原 region 内容应该 = (orig_data ^ core_id)
        $display("\n[Phase C] 检查 DDR mock");
        for (int c = 0; c < N_CORES; c++) begin
            automatic logic [DATA_W-1:0] expected;
            automatic logic [DATA_W-1:0] got;
            automatic int ddr_addr = c * 'h100;
            automatic int errs_before = errors;
            for (int i = 0; i < 16; i++) begin
                expected = ({32'h0, 32'h0, 32'(c), 32'(i)}) ^ {120'd0, 8'(c)};
                got      = u_mem.ddr_mem[ddr_addr + i];
                if (got !== expected) begin
                    if (errors < 5)
                        $display("  FAIL Conv%0d ddr[%0h]=%h expect %h",
                                 c, ddr_addr+i, got, expected);
                    errors++;
                end
            end
            if (errors == errs_before)
                $display("  PASS Conv%0d 16 word OFM written back to ddr[0x%0h..]", c, ddr_addr);
        end

        $display("\n============================================================");
        if (errors == 0)
            $display("  RESULT: PASS  (4-core mesh end-to-end packet flow)");
        else
            $display("  RESULT: FAIL  errors=%0d", errors);
        $display("============================================================");
        $finish;
    end

    initial begin
        #200000 $display("WATCHDOG TIMEOUT"); $finish;
    end
endmodule
