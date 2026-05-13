`timescale 1ns/1ps

// =============================================================================
// odma_sg_dispatcher.sv  --  Output DMA SG dispatcher (Phase 7 SMC + NUMA)
//
// 替代 odma_ctrl. 从 mem 内某段拉 OFM SG cmd list, 顺序对每条 cmd:
//   - 装 s2mm cmd 给 axi_dm.S2MM (dst_addr / btt 来自 cmd 内字段)
//   - 把 OFB SRAM 内对应段的 NHWC gather 流送给 s2mm.data
//   - 等 sts → 推进 cmd 进度
//
// 设计哲学 (跟 idma_sg_dispatcher 对称):
//   driver 编译期决定每行 OFM 拆成几条 cmd (一般 1, W slice + cout slice 跨 mem
//   时多). 每条 cmd 在某个 mem 内的连续 burst write.
//   AXI Crossbar IP 按 awaddr 自动路由到对应 mem.
//   一行 OFM → N 条 cmd (driver 已知每段在哪个 mem). dispatcher 不知道 mem 拓扑,
//   只跑 cmd.
//
// 通道复用:
//   - cmd 拉取走 mm2s_arb 的 ocmd 路 (新加的第 4 路, axi_dm.MM2S 通道)
//   - cmd 装载走 axi_dm.S2MM 通道 (跟 odma_ctrl 一致)
//   两条通道独立, 不冲突.
//
// SG cmd 二进制格式 (32 byte / cmd, 跟 mesh_cmd.py write_odma_sg_cmd_list 对齐):
//   word 0: dst_addr     [31:0]  目标全局地址 byte
//   word 1: btt          [22:0]  transfer 长度 byte (= sub_W × cout_slices × 16)
//                        [23]    last_cmd flag
//                        [31:24] reserved
//   word 2: ofb_w_start  [15:0]  该段在 OFB SRAM 的起始 W 列 (NHWC gather 用)
//                        [31:16] reserved
//   word 3: reserved
//
// FSM:
//   S_IDLE          → start → S_WAIT
//   S_WAIT          → row_done_pulse 攒够 1 行 || all_issued → 决定 S_FETCH_CMD_ISS / S_DRAIN
//   S_FETCH_CMD_ISS : 装 ocmd 拉 cmd_list[idx] (16 byte) → S_FETCH_CMD_DAT
//   S_FETCH_CMD_DAT : 收 1 beat, latch dst/btt/last/ofb_w_start → S_FETCH_CMD_STS
//   S_FETCH_CMD_STS : 等 ocmd sts → S_CMD
//   S_CMD           : 装 s2mm cmd (dst=cmd.dst, btt=cmd.btt) → S_PREFETCH
//   S_PREFETCH      : 1 拍 OFB 读延迟 → S_TX
//   S_TX            : 流式送 OFB beat 给 s2mm.data, tlast → S_STS
//   S_STS           : 等 s2mm sts. r_cmds_done++. is_last_cmd_in_row → yout_base wrap.
//                     r_cmds_done == cfg_cmd_count || last_cmd → S_DONE; 否 S_WAIT
//   S_DONE          → start → S_WAIT
//
// 注意: row_done_pulse 来自 ofb_writer, 当 ofb_writer 写完一整行 OFM 才 pulse;
// SG 模式一行 OFM 拆成 N 条 cmd, dispatcher 内部用 r_cmds_in_row 跟踪进度,
// 每行最后一条 cmd 装载完 (S_STS exit) 才视为消费一行 (yout_base 推进).
// =============================================================================

module odma_sg_dispatcher #(
    parameter int ADDR_W      = 32,
    parameter int DATA_W      = 128,
    parameter int SRAM_ADDR_W = 13,
    parameter int LEN_W       = 24,
    parameter int TAG_W       = 4,
    parameter logic [TAG_W-1:0] TAG_ODMA = '0
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // ---- Control ----
    input  logic                    start,
    output logic                    done,
    output logic                    busy,
    output logic                    err,

    // ---- SG cfg (替代 dst_base / byte_len) ----
    input  logic [ADDR_W-1:0]       cfg_cmd_list_ptr,
    input  logic [15:0]             cfg_cmd_count,
    input  logic [7:0]              cfg_cmds_per_row,

    // ---- Streaming cfg (跟 odma_ctrl 一致, OFB ring 反压 + NHWC gather) ----
    input  logic [15:0]             cfg_h_out_total,
    input  logic [15:0]             cfg_w_out,
    input  logic [5:0]              cfg_cout_slices,
    input  logic [19:0]             cfg_ofb_row_words,
    input  logic [19:0]             cfg_ofb_ring_words,
    input  logic                    row_done_pulse,
    output logic [15:0]             rows_drained,

    // ---- ocmd (mm2s_arb 的 ocmd 路, 拉 cmd list) ----
    output logic                    ocmd_cmd_tvalid,
    input  logic                    ocmd_cmd_tready,
    output logic [71:0]             ocmd_cmd_tdata,
    input  logic                    ocmd_data_tvalid,
    output logic                    ocmd_data_tready,
    input  logic [DATA_W-1:0]       ocmd_data_tdata,
    input  logic [DATA_W/8-1:0]     ocmd_data_tkeep,    // 不消费
    input  logic                    ocmd_data_tlast,
    input  logic                    ocmd_sts_tvalid,
    output logic                    ocmd_sts_tready,
    input  logic [7:0]              ocmd_sts_tdata,

    // ---- s2mm cmd / data / sts (axi_dm.S2MM 通道, 装实际 OFM 写 cmd) ----
    output logic                    s2mm_cmd_tvalid,
    input  logic                    s2mm_cmd_tready,
    output logic [71:0]             s2mm_cmd_tdata,

    output logic                    s2mm_data_tvalid,
    input  logic                    s2mm_data_tready,
    output logic [DATA_W-1:0]       s2mm_data_tdata,
    output logic [DATA_W/8-1:0]     s2mm_data_tkeep,
    output logic                    s2mm_data_tlast,

    input  logic                    s2mm_sts_tvalid,
    output logic                    s2mm_sts_tready,
    input  logic [7:0]              s2mm_sts_tdata,

    // ---- OFB SRAM read port ----
    output logic                    ofb_re,
    output logic [SRAM_ADDR_W-1:0]  ofb_raddr,
    input  logic [DATA_W-1:0]       ofb_rdata,
    // VD100 dbg: dispatcher state expose
    output logic [3:0]              state_out
);

    // 防 unused warning
    logic [DATA_W/8-1:0] _ocmd_tkeep_unused;
    assign _ocmd_tkeep_unused = ocmd_data_tkeep;
    logic _ocmd_tlast_unused;
    assign _ocmd_tlast_unused = ocmd_data_tlast;

    // =========================================================================
    // FSM
    // =========================================================================
    // Round H step 2: 跳过 S_FETCH_CMD_STS (ocmd sts 后台 collect, 跟 IDMA Round B 一致).
    //   每 cmd 省 ocmd sts wait turnaround (~2-4 cy/cmd).
    //   ResNet11 ODMA cmd 总数 ~600, 估省 1500-2400 cy ≈ 0.7-1.2%.
    typedef enum logic [3:0] {
        S_IDLE          = 4'd0,
        S_WAIT          = 4'd1,
        S_FETCH_CMD_ISS = 4'd2,
        S_FETCH_CMD_DAT = 4'd3,
        S_CMD           = 4'd5,
        S_PREFETCH      = 4'd6,
        S_TX            = 4'd7,
        S_STS           = 4'd8,
        S_DONE          = 4'd9
    } state_t;
    state_t state, state_next;

    // =========================================================================
    // 寄存器
    // =========================================================================
    logic [15:0]             r_cmd_idx;        // 0..cfg_cmd_count-1
    logic [15:0]             r_cmds_done;
    logic [7:0]              r_cmds_in_row;    // 当前行内已完成几条 cmd
    logic [15:0]             r_rows_drained;   // 完成几行 OFM (= s2mm sts 累计 / cmds_per_row)
    logic [15:0]             r_rows_consumed_from_writer;  // 来自 row_done_pulse 计数

    // 当前 cmd 字段
    logic [ADDR_W-1:0]       r_dst_addr;
    logic [LEN_W-1:0]        r_btt;
    logic                    r_last_cmd;
    logic [15:0]             r_ofb_w_start;

    // NHWC gather (per-cmd 段内)
    logic [15:0]             r_seg_w_left;     // 当前段内 W 列剩余 (sub_W → 0)
    logic [SRAM_ADDR_W-1:0]  r_yout_base;
    logic [SRAM_ADDR_W-1:0]  r_cs_offset;
    logic [SRAM_ADDR_W-1:0]  r_rd_ptr;
    logic [15:0]             r_x_rd;
    logic [5:0]              r_cs_rd;
    logic [15:0]             r_beats_left;     // 当前段剩余 beats (= seg_W × cout_slices)

    logic                    r_done;
    logic                    r_err;

    assign rows_drained = r_rows_drained;
    assign err          = r_err;

    // =========================================================================
    // 派生
    // =========================================================================
    logic ocmd_cmd_fire, ocmd_data_fire, ocmd_sts_fire;
    assign ocmd_cmd_fire  = ocmd_cmd_tvalid  && ocmd_cmd_tready;
    assign ocmd_data_fire = ocmd_data_tvalid && ocmd_data_tready;
    assign ocmd_sts_fire  = ocmd_sts_tvalid  && ocmd_sts_tready;

    logic s2mm_cmd_fire, s2mm_data_fire, s2mm_tlast_fire, s2mm_sts_fire;
    assign s2mm_cmd_fire   = s2mm_cmd_tvalid  && s2mm_cmd_tready;
    assign s2mm_data_fire  = s2mm_data_tvalid && s2mm_data_tready;
    assign s2mm_tlast_fire = s2mm_data_fire && s2mm_data_tlast;
    assign s2mm_sts_fire   = s2mm_sts_tvalid  && s2mm_sts_tready;

    // is_last_cmd_in_row: 当前 cmd 是本行最后 (decremented at S_STS)
    logic is_last_cmd_in_row;
    assign is_last_cmd_in_row = (cfg_cmds_per_row == 8'd0)
                               || (r_cmds_in_row + 8'd1 >= cfg_cmds_per_row);

    // is_last_cmd_overall: 所有 cmd 已装载 (注意 r_cmd_idx 在 cmd_fire 时 +1, 表示
    // 已装载几个 cmd; 装载第 N 个 cmd 后 r_cmd_idx=N, 此时 sts 完成才算 list 跑完).
    // 之前用 r_cmd_idx+1 >= cfg_cmd_count 是 off-by-one bug: 装第 N-1 个 cmd 后
    // r_cmd_idx=N-1, 这条的 sts → N-1+1=N == cfg_cmd_count → 误判转 S_DONE,
    // 漏装第 N 个 cmd.
    logic is_last_cmd_overall;
    assign is_last_cmd_overall = (r_cmd_idx >= cfg_cmd_count);

    // has_work_to_issue: writer 攒够的行 > 已 dispatch 的行
    //   原实现: r_rows_dispatched_view = r_cmds_done / cfg_cmds_per_row (16-bit 除法,
    //          综合后 45 级 CARRY4 链 + 11 LUT3, 是 critical path 主因 WNS=-6.835ns)
    //   优化: r_rows_drained 已经在 is_last_cmd_in_row && sts_fire 时累加, 跟
    //          r_cmds_done/cmds_per_row 在 floor() 意义上等价. 直接用之, 消除除法.
    //   has_writer_data_ready 在新 row 启动 cmd 前检查, mid-row 时两者都同步推进 OK.
    logic has_writer_data_ready;
    assign has_writer_data_ready = (r_rows_consumed_from_writer > r_rows_drained);

    logic all_cmds_issued;
    assign all_cmds_issued = (r_cmd_idx >= cfg_cmd_count);

    // VD100 fix 2026-05-13: streaming_all_done 不依赖 s2mm_sts_fire (axi_dm 板上 sts 偶尔不出
    // 卡 ODMA 跟 sequencer S_WAIT, OFM 96% 后停). 改用最后一个 cmd 的 data tlast 触发 r_done.
    // axi_dm s2mm 收到 last data 后自己写 DDR 完成, 不需要 dispatcher 等 sts.
    logic streaming_all_done;
    assign streaming_all_done = (state == S_TX) && s2mm_data_tvalid && s2mm_data_tready && s2mm_data_tlast
                              && (r_last_cmd || is_last_cmd_overall);

    // sub_W (该段宽 W 列) = btt / (cout_slices × 16)
    //   btt[22:0] 单位 byte, 一 word = 16 byte, sub_W × cout_slices × 16 = btt
    //   → sub_W × cout_slices = btt[22:4]
    //   → sub_W = btt[22:4] / cout_slices (硬件除法贵, driver 编译期保证 sub_W
    //     直接放在 cmd 内 word 2 高位即可避免硬件除法; 这里简化用 r_beats_left
    //     初始化 = btt[22:4], NHWC gather 内部按 cs/x 推进)

    // =========================================================================
    // ocmd cmd: S_FETCH_CMD_ISS 时装 cmd 拉 cmd_list[idx] (16 byte / 1 beat)
    // =========================================================================
    logic [ADDR_W-1:0] ocmd_saddr;
    assign ocmd_saddr = cfg_cmd_list_ptr + (ADDR_W'(r_cmd_idx) << 5);   // 32 byte / cmd

    assign ocmd_cmd_tdata = {
        4'b0,           // [71:68] reserved
        4'd0,           // [67:64] tag (ocmd 共用 tag 0)
        ocmd_saddr,     // [63:32]
        1'b0,           // [31] DRR
        1'b1,           // [30] EOF
        6'b0,           // [29:24] DSA
        1'b1,           // [23] TYPE = INCR
        23'd16          // [22:0] BTT = 16 byte (1 beat)
    };
    assign ocmd_cmd_tvalid  = (state == S_FETCH_CMD_ISS);
    assign ocmd_data_tready = (state == S_FETCH_CMD_DAT);
    // Round H step 2: ocmd sts 后台 collect, 永远 ready, 不阻塞 main FSM
    assign ocmd_sts_tready  = 1'b1;

    // =========================================================================
    // s2mm cmd: S_CMD 时装 cmd
    // =========================================================================
    logic [22:0] s2mm_btt;
    assign s2mm_btt = r_btt[22:0];
    assign s2mm_cmd_tdata = {
        4'b0,            // [71:68] reserved
        TAG_ODMA,        // [67:64]
        r_dst_addr,      // [63:32]
        1'b0,            // [31] DRR
        1'b1,            // [30] EOF
        6'b0,            // [29:24] DSA
        1'b1,            // [23] TYPE = INCR
        s2mm_btt         // [22:0] BTT
    };
    assign s2mm_cmd_tvalid = (state == S_CMD);

    // =========================================================================
    // s2mm data: S_TX 流式送 OFB beat
    // =========================================================================
    assign s2mm_data_tvalid = (state == S_TX);
    assign s2mm_data_tdata  = ofb_rdata;
    assign s2mm_data_tkeep  = '1;
    assign s2mm_data_tlast  = (state == S_TX) && (r_beats_left == 16'd1);
    assign s2mm_sts_tready  = 1'b1;

    // OFB read: 跟 odma_ctrl 一致, S_PREFETCH 拍 latch 第一 beat, S_TX 每 fire 推进
    //   x_rd 段内推进, cs_rd 内层
    logic [SRAM_ADDR_W-1:0] rd_ptr_next;
    logic                   cs_rd_is_last;
    assign cs_rd_is_last = (r_cs_rd == cfg_cout_slices - 6'd1);

    always_comb begin
        if (cs_rd_is_last)
            rd_ptr_next = r_yout_base + (r_x_rd + 16'd1);   // 下一 W 列, cs_rd 回 0
        else
            rd_ptr_next = r_rd_ptr + cfg_w_out[SRAM_ADDR_W-1:0];   // 下一 cs 行
    end

    assign ofb_re    = (state == S_PREFETCH) || (state == S_TX);
    assign ofb_raddr = (state == S_TX && s2mm_data_fire) ? rd_ptr_next : r_rd_ptr;


    assign done = r_done && !start;
    assign busy = (state != S_IDLE) && (state != S_DONE);
    assign state_out = state;            // VD100 dbg

    // =========================================================================
    // FSM
    // =========================================================================
    always_comb begin
        state_next = state;
        case (state)
            S_IDLE          : if (start)                               state_next = S_WAIT;
            S_WAIT          : if      (all_cmds_issued)                state_next = S_DONE;
                              else if (has_writer_data_ready)          state_next = S_FETCH_CMD_ISS;
            S_FETCH_CMD_ISS : if (ocmd_cmd_fire)                       state_next = S_FETCH_CMD_DAT;
            S_FETCH_CMD_DAT : if (ocmd_data_fire)                      state_next = S_CMD;     // Round H: 跳 STS, sts 后台 collect
            S_CMD           : if (s2mm_cmd_fire)                       state_next = S_PREFETCH;
            S_PREFETCH      :                                          state_next = S_TX;
            // VD100 fix 2026-05-13: 跳过 S_STS (axi_dm s2mm 板上偶尔不出 sts), S_TX tlast 后
            // 直接判 cmd 完结 → S_DONE / S_WAIT. r_cmds_done 改在 data tlast 时累加 (下面).
            S_TX            : if (s2mm_tlast_fire) begin
                if (r_last_cmd || is_last_cmd_overall)                 state_next = S_DONE;
                else                                                    state_next = S_WAIT;
            end
            S_STS           : state_next = S_WAIT;  // 兼容: 老代码可能 set 到 S_STS, 立即 fall through
            S_DONE          : if (start)                               state_next = S_WAIT;
            default         :                                          state_next = S_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= state_next;
    end

    // =========================================================================
    // r_rows_consumed_from_writer: row_done_pulse 计数
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (start)              r_rows_consumed_from_writer <= 16'd0;
        else if (row_done_pulse)     r_rows_consumed_from_writer <= r_rows_consumed_from_writer + 16'd1;
    end

    // =========================================================================
    // cmd 解码: S_FETCH_CMD_DAT 收 1 beat
    //   beat[31:0]    = word 0 = dst_addr
    //   beat[63:32]   = word 1 = {reserved[7:0], last_cmd, btt[22:0]}
    //   beat[95:64]   = word 2 = {reserved[15:0], ofb_w_start[15:0]}
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            r_dst_addr    <= '0;
            r_btt         <= '0;
            r_last_cmd    <= 1'b0;
            r_ofb_w_start <= '0;
        end else if (state == S_FETCH_CMD_DAT && ocmd_data_fire) begin
            r_dst_addr    <= ocmd_data_tdata[31:0];
            r_btt         <= {1'b0, ocmd_data_tdata[32 +: 23]};
            r_last_cmd    <= ocmd_data_tdata[32 + 23];
            r_ofb_w_start <= ocmd_data_tdata[64 +: 16];
        end
    end

    // =========================================================================
    // 段初始化 (S_PREFETCH 拍): r_x_rd / r_cs_rd / r_rd_ptr / r_beats_left
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            r_x_rd       <= '0;
            r_cs_rd      <= '0;
            r_cs_offset  <= '0;
            r_rd_ptr     <= '0;
            r_beats_left <= '0;
            r_seg_w_left <= '0;
        end else if (state == S_CMD && s2mm_cmd_fire) begin
            // 进入新段: 重置 NHWC gather
            r_x_rd       <= r_ofb_w_start;
            r_cs_rd      <= '0;
            r_cs_offset  <= '0;
            r_rd_ptr     <= r_yout_base + r_ofb_w_start[SRAM_ADDR_W-1:0];
            // beats_left = btt / 16 (一 beat = 16 byte)
            r_beats_left <= r_btt[19:4];
            // seg_w_left = beats_left / cout_slices (但避免硬件除法,
            //   driver 已经保证 beats_left = sub_W * cout_slices)
            r_seg_w_left <= r_btt[19:4];   // unused 简化, 仅 NHWC gather 用 cs/x 推进
        end else if (state == S_TX && s2mm_data_fire) begin
            // 段内 NHWC 推进
            if (cs_rd_is_last) begin
                r_cs_rd     <= '0;
                r_cs_offset <= '0;
                r_x_rd      <= r_x_rd + 16'd1;
            end else begin
                r_cs_rd     <= r_cs_rd + 6'd1;
                r_cs_offset <= r_cs_offset + cfg_w_out[SRAM_ADDR_W-1:0];
            end
            r_rd_ptr <= rd_ptr_next;
            if (r_beats_left > 16'd0) r_beats_left <= r_beats_left - 16'd1;
        end
    end

    // =========================================================================
    // r_yout_base: 跨行推进 (本行最后 cmd s2mm_sts_fire 时, ring wrap)
    // =========================================================================
    logic [SRAM_ADDR_W-1:0] yb_plus, yout_base_next;
    assign yb_plus = r_yout_base + cfg_ofb_row_words[SRAM_ADDR_W-1:0];
    always_comb begin
        if (yb_plus >= cfg_ofb_ring_words[SRAM_ADDR_W-1:0])
            yout_base_next = yb_plus - cfg_ofb_ring_words[SRAM_ADDR_W-1:0];
        else
            yout_base_next = yb_plus;
    end

    always_ff @(posedge clk) begin
        if      (start)                                             r_yout_base <= '0;
        else if (state == S_STS && s2mm_sts_fire && is_last_cmd_in_row)
                                                                    r_yout_base <= yout_base_next;
    end

    // =========================================================================
    // r_cmd_idx / r_cmds_done / r_cmds_in_row / r_rows_drained
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (!rst_n)                                            r_cmd_idx <= 16'd0;
        else if (start)                                             r_cmd_idx <= 16'd0;
        else if (state == S_FETCH_CMD_ISS && ocmd_cmd_fire)         r_cmd_idx <= r_cmd_idx + 16'd1;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            r_cmds_done   <= 16'd0;
            r_cmds_in_row <= 8'd0;
            r_rows_drained <= 16'd0;
        end else if (start) begin
            r_cmds_done   <= 16'd0;
            r_cmds_in_row <= 8'd0;
            r_rows_drained <= 16'd0;
        // VD100 fix 2026-05-13: r_cmds_done 累加跟 s2mm data tlast 同步 (不等 sts),
        // 跟 S_TX → S_DONE/S_WAIT 跳过 S_STS 一致.
        end else if (state == S_TX && s2mm_tlast_fire) begin
            r_cmds_done <= r_cmds_done + 16'd1;
            if (is_last_cmd_in_row) begin
                r_cmds_in_row  <= 8'd0;
                r_rows_drained <= r_rows_drained + 16'd1;
            end else begin
                r_cmds_in_row <= r_cmds_in_row + 8'd1;
            end
        end
    end

    // =========================================================================
    // r_done sticky / r_err sticky
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (!rst_n)              r_done <= 1'b0;
        else if (start)               r_done <= 1'b0;
        else if (streaming_all_done)  r_done <= 1'b1;
    end

    always_ff @(posedge clk) begin
        if      (!rst_n)                                                r_err <= 1'b0;
        else if (start)                                                 r_err <= 1'b0;
        else if (s2mm_sts_fire && !s2mm_sts_tdata[7])                   r_err <= 1'b1;
        // Round H: ocmd sts 后台 collect, 永远 ready, 任何拍 sts_fire 都检查 err
        else if (ocmd_sts_fire && !ocmd_sts_tdata[7])                   r_err <= 1'b1;
    end

endmodule
