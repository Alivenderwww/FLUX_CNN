`timescale 1ns/1ps

// =============================================================================
// idma_sg_dispatcher.sv  --  IDMA SG (Scatter-Gather) cmd list dispatcher
//
// [Phase 7 SMC + NUMA] 用来替代 idma_ctrl. 接口签名跟 idma_ctrl 几乎一致 (mm2s
// cmd/data/sts + IFB 写口 + ring 反压), 这样 core_top.sv 用 generate-if 在
// SG_MODE 下用 dispatcher / 默认 mode 下用 idma_ctrl, 一行接线不同就够了.
//
// 设计哲学:
//   driver 编译期把"该核该层要拉哪些段"算好, 写成 cmd list 放在 mem 某段.
//   dispatcher 顺序拉每条 cmd, 一条 cmd = 一段连续 burst (在某一个 mem 内).
//   AXI Crossbar / SmartConnect IP 按 awaddr 自动路由到对应 mem slave.
//   Halo 列只一份: driver 把"邻居 mem 内的左/右 halo 列"作为独立 cmd 加到 list,
//   dispatcher 不需要知道这是 halo, 它只跑 cmd.
//
// 通道复用:
//   cmd 拉取 + 实际 data 拉取都走 mm2s_arb 的 idma 端口 (axi_dm.MM2S 通道).
//   dispatcher 内部串行: 先发 cmd 拉 32 byte cmd list, 收完后再发 cmd 拉用户 data.
//   这样不需要新增 axi master read 通道, axi_m_mux 不动.
//
// SG cmd 二进制格式 (32 byte / cmd, DATA_W=128 = 2 beats):
//   word 0: src_addr     [31:0]   transfer 起点 byte addr (全局地址)
//   word 1: btt          [22:0]   transfer 长度 byte (Bytes To Transfer)
//                        [23]     last_cmd flag (=1 时这条是 list 末尾, 优先级高于 cmd_count)
//                        [31:24]  reserved
//   word 2: sram_offset  [12:0]   IFB SRAM 写入起点 (word offset)
//                        [31:13]  reserved
//   word 3: reserved
//
//   beat 0 (DATA_W=128) = {word3[31:0], word2[31:0], word1[31:0], word0[31:0]}
//   注意: AXI 的 little-endian byte order 让 word 0 在 tdata[31:0]
//   这里是简化 PoC 格式, 不跟 Xilinx axi_dma SG 64-byte descriptor 对齐;
//   driver (mesh_cmd.py write_sg_cmd_list) 跟 RTL 同步即可.
//
// FSM (J-7 Round B 流水化, 2026-05-07):
//   S_IDLE       → start → S_FETCH_CMD_ISSUE
//   S_FETCH_CMD_ISSUE: 装 cmd 拉 cmd_list[r_cmd_idx] (16 byte) → S_FETCH_CMD_DATA
//   S_FETCH_CMD_DATA : 收 1 beat 16 byte, latch cmd info → S_RING_WAIT
//                       (跳过原 S_FETCH_CMD_STS, sts 后台 collect)
//   S_RING_WAIT      : 检查 ring 有空间 → S_ISSUE
//   S_ISSUE          : 装实际 cmd 拉 user data → S_DATA
//   S_DATA           : 收 data, 写 IFB SRAM. tlast → next cmd (跳过原 S_STS)
//                       后台 sts collector 单独累计 r_cmds_done.
//   S_DONE           : 等 r_cmds_done == r_cmd_idx (所有 sts 收完) → r_done sticky.
//
// Round B 省去 S_FETCH_CMD_STS + S_STS 共 2 拍/cmd 的 sts wait turnaround.
// cross-mem layer per_row=2 时 C1/C2 cmd_count 翻倍, 省 ~2*H 拍/layer ≈ 5~8% 整网.
// sts 后台 collector 永远 ready, 错误 sticky 不阻塞 main FSM.
// =============================================================================

module idma_sg_dispatcher #(
    parameter int ADDR_W      = 32,
    parameter int DATA_W      = 128,
    parameter int SRAM_ADDR_W = 13,
    parameter int LEN_W       = 24,
    parameter int TAG_W       = 4,
    parameter logic [TAG_W-1:0] TAG = '0
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // ---- Control ----
    input  logic                    start,
    output logic                    done,
    output logic                    busy,
    output logic                    err,

    // ---- SG 配置 (替代 idma_ctrl 的 src_base/byte_len) ----
    input  logic [ADDR_W-1:0]       cfg_cmd_list_ptr,   // cmd list 起点 (全局地址 byte)
    input  logic [15:0]             cfg_cmd_count,      // cmd 总数 (sanity check;
                                                         // last_cmd flag 优先终止)
    input  logic [7:0]              cfg_cmds_per_row,   // 每行 IFM 由几条 cmd 组成
                                                         // (W slice 跨 mem 时 > 1)

    // ---- Streaming 配置 (跟 idma_ctrl 一致, line_buffer ring 反压) ----
    input  logic [15:0]             cfg_h_in_total,
    input  logic [7:0]              cfg_ifb_strip_rows,
    input  logic [19:0]             cfg_ifb_ring_words,
    input  logic [15:0]             rows_consumed,
    output logic [15:0]             rows_available,

    // ---- DataMover MM2S CMD stream (跟 mm2s_arb idma input 对接) ----
    output logic                    mm2s_cmd_tvalid,
    input  logic                    mm2s_cmd_tready,
    output logic [71:0]             mm2s_cmd_tdata,

    // ---- DataMover MM2S DATA stream (in) ----
    input  logic                    mm2s_data_tvalid,
    output logic                    mm2s_data_tready,
    input  logic [DATA_W-1:0]       mm2s_data_tdata,
    input  logic [DATA_W/8-1:0]     mm2s_data_tkeep,    // 不消费
    input  logic                    mm2s_data_tlast,

    // ---- DataMover MM2S STS stream (in, 8-bit per cmd) ----
    input  logic                    mm2s_sts_tvalid,
    output logic                    mm2s_sts_tready,
    input  logic [7:0]              mm2s_sts_tdata,

    // ---- IFB SRAM write port ----
    output logic                    ifb_we,
    output logic [SRAM_ADDR_W-1:0]  ifb_waddr,
    output logic [DATA_W-1:0]       ifb_wdata,

    // VD100 dbg: dispatcher.st expose 给 cfg_regs ADDR_SEQ_DBG
    output logic [3:0]              state_out
);

    // unused tkeep 防 lint
    logic [DATA_W/8-1:0] _tkeep_unused;
    assign _tkeep_unused = mm2s_data_tkeep;

    // SG cmd 一条占 16 byte 实际有效 (我们设计 32 byte 但只用 word 0..3 共 16 byte)
    localparam int CMD_BTT       = 16;        // 拉 cmd 的 transfer 长度 (byte)

    // J-7 Round B: 跳过 S_FETCH_CMD_STS / S_STS (sts 后台 collect), main FSM 6 状态.
    typedef enum logic [3:0] {
        S_IDLE          = 4'd0,
        S_FETCH_CMD_ISS = 4'd1,
        S_FETCH_CMD_DAT = 4'd2,
        S_RING_WAIT     = 4'd4,
        S_ISSUE         = 4'd5,
        S_DATA          = 4'd6,
        S_DONE          = 4'd8
    } state_t;
    state_t st, st_next;

    // ---- 当前正在处理的 cmd 字段 ----
    logic [ADDR_W-1:0]      r_src_addr;
    logic [LEN_W-1:0]       r_btt;
    logic                   r_last_cmd;
    logic                   sts_drained;   // VD100 fix forward decl
    logic [SRAM_ADDR_W-1:0] r_sram_offset;
    logic [SRAM_ADDR_W-1:0] r_sram_wptr;

    // ---- cmd list 进度 ----
    logic [15:0]       r_cmd_idx;       // 0..cfg_cmd_count-1
    logic [15:0]       r_cmds_done;     // 完成几条 cmd (跟 r_cmd_idx 区别在 STS 后才递增)
    logic [15:0]       r_rows_pushed;   // 推进的行数 = r_cmds_done / cfg_cmds_per_row

    logic              r_done;
    logic              r_err;

    assign busy = (st != S_IDLE) && (st != S_DONE);
    assign done = r_done && !start;
    assign err  = r_err;
    assign state_out = st;          // VD100 dbg
    assign rows_available = r_rows_pushed;

    // =========================================================================
    // mm2s_cmd_tdata: 两阶段共享 (FETCH_CMD_ISS 时拉 cmd_list[idx], ISSUE 时拉 user data)
    // =========================================================================
    // VD100 fix 2026-05-12: 4KB-boundary split — axi_dm IP (board axi_noc & sim axi_smc)
    // 在跨 4KB 边界 burst 时不出 m_axis_mm2s_tlast 卡死. dispatcher 内部按 4KB 拆 cmd.
    // cap_btt = min(r_btt, 4KB - src[11:0]). S_ISSUE 发 cap_btt 大小 cmd;
    // S_DATA tlast 后若 remaining > 0 再回 S_ISSUE 发 sub-cmd; remaining=0 才 cmd_idx++.
    logic [22:0] cap_btt;
    logic [22:0] room_to_4k;
    assign room_to_4k = 23'h800000 - {11'd0, r_src_addr[11:0]};  // ↓ 实际只用低位, 加宽免溢出
    wire   [22:0] room4k = 23'h001000 - {11'd0, r_src_addr[11:0]};
    assign cap_btt = (r_btt[22:0] > room4k) ? room4k : r_btt[22:0];


    logic [ADDR_W-1:0]      cmd_saddr;
    logic [22:0]            cmd_btt23;
    always_comb begin
        if (st == S_FETCH_CMD_ISS) begin
            // 拉 cmd_list[idx]: 起点 = cfg_cmd_list_ptr + idx * 32 (32 byte / cmd)
            cmd_saddr = cfg_cmd_list_ptr + (ADDR_W'(r_cmd_idx) << 5);
            cmd_btt23 = 23'(CMD_BTT);
        end else begin   // S_ISSUE: 用 cap_btt (4KB-clipped)
            cmd_saddr = r_src_addr;
            cmd_btt23 = cap_btt;
        end
    end

    assign mm2s_cmd_tdata = {
        4'b0,           // [71:68] reserved
        TAG,            // [67:64]
        cmd_saddr,      // [63:32]
        1'b0,           // [31] DRR
        1'b1,           // [30] EOF
        6'b0,           // [29:24] DSA
        1'b1,           // [23] TYPE = INCR
        cmd_btt23       // [22:0] BTT
    };
    assign mm2s_cmd_tvalid = (st == S_FETCH_CMD_ISS) || (st == S_ISSUE);

    // data tready: 拉 cmd 时 OR 拉用户 data 时
    assign mm2s_data_tready = (st == S_FETCH_CMD_DAT) || (st == S_DATA);
    // J-7 Round B: sts 后台 collector 永远 ready, 不让 main FSM 等 sts
    assign mm2s_sts_tready  = 1'b1;

    // =========================================================================
    // IFB SRAM 写: 仅 S_DATA 阶段 (拉 cmd 阶段不写 IFB)
    // =========================================================================
    assign ifb_we    = (st == S_DATA) && mm2s_data_tvalid && mm2s_data_tready;
    assign ifb_waddr = r_sram_wptr;
    assign ifb_wdata = mm2s_data_tdata;

    // =========================================================================
    // ring 反压: r_rows_pushed - rows_consumed >= cfg_ifb_strip_rows 时 ring 满
    //   strip_rows = 0 表示 batch mode (整图装下), 反压 disable
    //
    // 注意 line_buffer.rows_consumed_raw += stride 在 stride>1 layer 时, 一行 OFM
    // 完成消费多行 IFM, 让 rows_consumed > rows_pushed (uint underflow). 这种情况
    // ring 内应被认为"有充裕空间" (反压无需触发). 用 signed 比较或加 fallthrough.
    // =========================================================================
    logic ring_has_space;
    logic ring_consumer_ahead;
    logic [15:0] rows_diff;
    assign rows_diff = r_rows_pushed - rows_consumed;
    assign ring_consumer_ahead = (rows_consumed >= r_rows_pushed);    // consumer 跑超 producer
    assign ring_has_space = (cfg_ifb_strip_rows == '0)
                          || ring_consumer_ahead
                          || (rows_diff < {8'd0, cfg_ifb_strip_rows});

    // =========================================================================
    // FSM next-state
    // =========================================================================
    // J-7 Round B: main FSM 跳过 STS state, sts 后台 collect.
    //   S_DATA tlast 后直接判断是否最后 cmd (r_last_cmd / r_cmd_idx>=cmd_count):
    //     是 → S_DONE (但 S_DONE 内还要等所有 sts 收完才置 r_done)
    //     否 → S_FETCH_CMD_ISS 下一条
    always_comb begin
        st_next = st;
        case (st)
            S_IDLE          : if (start && cfg_cmd_count > 0) st_next = S_FETCH_CMD_ISS;
            S_FETCH_CMD_ISS : if (mm2s_cmd_tvalid && mm2s_cmd_tready) st_next = S_FETCH_CMD_DAT;
            S_FETCH_CMD_DAT : if (mm2s_data_tvalid && mm2s_data_tready && mm2s_data_tlast)
                                                                         st_next = S_RING_WAIT;
            S_RING_WAIT     : if (ring_has_space)                         st_next = S_ISSUE;
            S_ISSUE         : if (mm2s_cmd_tvalid && mm2s_cmd_tready)    st_next = S_DATA;
            S_DATA          : if (mm2s_data_tvalid && mm2s_data_tready && mm2s_data_tlast) begin
                // VD100 4KB split: 如果还有 remaining (r_btt > cap_btt), 回 S_ISSUE 发 sub-cmd.
                // remaining=0 表示 logical cmd 完成, cmd_idx++ 在另一处 always_ff.
                if (r_btt[22:0] != cap_btt)                                st_next = S_ISSUE;
                else if (r_last_cmd || (r_cmd_idx >= cfg_cmd_count))      st_next = S_DONE;
                else                                                       st_next = S_FETCH_CMD_ISS;
            end
            S_DONE          : if (sts_drained)                            st_next = S_IDLE;   // VD100 fix: 等所有 sts 收完再退出, 否则 r_done 不 set
            default         : st_next = S_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) st <= S_IDLE;
        else        st <= st_next;
    end


    // =========================================================================
    // cmd 解码: S_FETCH_CMD_DAT 收 1 beat (DATA_W=128 = 16 byte) latch word 0..3
    //   beat[31:0]   = word 0 = src_addr
    //   beat[63:32]  = word 1 = {reserved[7:0], last_cmd[1], btt[22:0]}
    //   beat[95:64]  = word 2 = sram_offset (低 13 bit 有效)
    //   beat[127:96] = word 3 = reserved
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            r_src_addr    <= '0;
            r_btt         <= '0;
            r_last_cmd    <= 1'b0;
            r_sram_offset <= '0;
        end else if (st == S_FETCH_CMD_DAT && mm2s_data_tvalid && mm2s_data_tready) begin
            r_src_addr    <= mm2s_data_tdata[31:0];
            r_btt         <= {1'b0, mm2s_data_tdata[32 +: 23]};
            r_last_cmd    <= mm2s_data_tdata[32 + 23];
            r_sram_offset <= mm2s_data_tdata[64 +: SRAM_ADDR_W];
        end else if (st == S_DATA && mm2s_data_tvalid && mm2s_data_tready && mm2s_data_tlast) begin
            // VD100 4KB split: tlast 后 src += cap_btt, btt -= cap_btt (准备下 sub-cmd or done)
            r_src_addr <= r_src_addr + {9'd0, cap_btt};
            r_btt      <= r_btt      - {1'b0, cap_btt};
        end
    end

    // =========================================================================
    // IFB SRAM write pointer:
    //   每条 cmd 起点 = r_sram_offset, 内部递增, 跨 cfg_ifb_ring_words 时 wrap
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n)                              r_sram_wptr <= '0;
        // VD100 4KB split: wptr reset 移到 S_RING_WAIT (logical cmd 第 1 sub-cmd 之前),
        // sub-cmd 走 S_DATA→S_ISSUE 不重置 wptr, 让 sub-cmd 在 logical cmd 内部连续累加.
        else if (st == S_RING_WAIT)              r_sram_wptr <= r_sram_offset;
        else if (st == S_DATA && ifb_we) begin
            // wrap (cfg_ifb_ring_words = 0 → 不 wrap, 跑 batch mode)
            if ((cfg_ifb_ring_words != '0)
              && ({{(20-SRAM_ADDR_W){1'b0}}, r_sram_wptr} + 20'd1
                                             >= cfg_ifb_ring_words))
                r_sram_wptr <= '0;
            else
                r_sram_wptr <= r_sram_wptr + 1'b1;
        end
    end

    // =========================================================================
    // r_cmd_idx ++ 在 S_DATA tlast (= dispatch 完一条 cmd 的 user data, J-7 改)
    //   每条 cmd 实际有 2 次 cmd_fire: fetch_cmd_list (16 byte) + issue_user_data (变长).
    //   r_cmd_idx 用于 fetch ptr (cmd_list[r_cmd_idx]) 跟 cmds_per_row 累加.
    //   S_DATA tlast = 第 N 条 cmd 完整 dispatch (data 已写 IFB, sts 还在路上).
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (!rst_n) r_cmd_idx <= '0;
        else if (start)  r_cmd_idx <= '0;
        // VD100 4KB split: cmd_idx ++ 只在 sub-cmd 全发完 (r_btt == cap_btt 即 last sub) 时
        else if (st == S_DATA && mm2s_data_tvalid && mm2s_data_tready && mm2s_data_tlast
                 && r_btt[22:0] == cap_btt)
                         r_cmd_idx <= r_cmd_idx + 16'd1;
    end

    // =========================================================================
    // r_rows_pushed: 跟 r_cmd_idx 同步推进 (issue-driven, 不等 sts)
    //   ring 反压用此 → in-flight 但未 sts 的 cmd 也占 ring 空间, 安全保守.
    //   line_buffer 看 rows_available = r_rows_pushed, data 已写入 IFB (S_DATA tlast 后).
    // =========================================================================
    logic [7:0] r_cmds_in_row;     // 当前行内已 dispatch 几条 cmd (0..cfg_cmds_per_row-1)
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            r_cmds_in_row <= '0;
            r_rows_pushed <= '0;
        end else if (start) begin
            r_cmds_in_row <= '0;
            r_rows_pushed <= '0;
        // VD100 4KB split: 仅 sub-cmd 全完成 (r_btt == cap_btt) 时增 row counter
        end else if (st == S_DATA && mm2s_data_tvalid && mm2s_data_tready && mm2s_data_tlast
                     && r_btt[22:0] == cap_btt) begin
            if (cfg_cmds_per_row == 8'd0 || r_cmds_in_row + 8'd1 >= cfg_cmds_per_row) begin
                r_cmds_in_row <= '0;
                r_rows_pushed <= r_rows_pushed + 16'd1;
            end else begin
                r_cmds_in_row <= r_cmds_in_row + 8'd1;
            end
        end
    end

    // =========================================================================
    // J-7 后台 sts collector (跟 main FSM 解耦, 永远 ready)
    //   r_cmds_done 累计 sts 收到的条数 (跨 fetch_cmd 跟 user_data 的所有 sts).
    //   每条逻辑 cmd 实际有 2 次 sts (fetch + user_data), 所以 sts 总数 = 2 × cmd_count.
    //   只用作 S_DONE drain 检查 (等 r_cmds_done == r_cmd_idx*2 ?).
    //
    //   实际更简单: r_cmds_done 单独 count, S_DONE 时检查 sts_pending = 0.
    //   sts_pending = (cmd_fire 计数 - sts_fire 计数), 只在 0 时才真 done.
    // =========================================================================
    logic [15:0] r_cmd_fires_total;     // 累计 mm2s_cmd_tvalid&ready 次数 (= 总 cmd 数)
    logic [15:0] r_sts_collected;       // 累计 sts_fire 次数
    always_ff @(posedge clk) begin
        if (!rst_n || start) begin
            r_cmd_fires_total <= '0;
            r_sts_collected   <= '0;
            r_cmds_done       <= '0;
        end else begin
            if (mm2s_cmd_tvalid && mm2s_cmd_tready)
                r_cmd_fires_total <= r_cmd_fires_total + 16'd1;
            if (mm2s_sts_tvalid && mm2s_sts_tready) begin
                r_sts_collected   <= r_sts_collected   + 16'd1;
                r_cmds_done       <= r_cmds_done       + 16'd1;
            end
        end
    end

    // =========================================================================
    // r_done sticky: 进 S_DONE 后等所有 sts 收完才置 r_done
    //   sts_pending = r_cmd_fires_total - r_sts_collected (in-flight cmd 数)
    //   pending = 0 才允许 r_done = 1
    // =========================================================================
    // VD100 fix 2026-05-13: r_done 不依赖 sts_collected (axi_dm 板上 sts 偶尔不出卡死).
    // 用 cmd_idx 达 cmd_count 判, sts FIFO 自然吸收, dispatcher 不等.
    assign sts_drained = (r_cmd_idx >= cfg_cmd_count);
    always_ff @(posedge clk) begin
        if      (!rst_n)                              r_done <= 1'b0;
        else if (start)                               r_done <= 1'b0;
        else if (st == S_DONE)                        r_done <= 1'b1;
    end

    // =========================================================================
    // r_err: sts[7] = 0 表示错误 (sticky). 后台 collect 时检查.
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (!rst_n)                                                  r_err <= 1'b0;
        else if (mm2s_sts_tvalid && mm2s_sts_tready && !mm2s_sts_tdata[7]) r_err <= 1'b1;
    end

    // =========================================================================
    // 性能 counter (论文素材, 不影响功能)
    //   hs_fetch_cy:    在 S_FETCH_CMD_ISS / S_FETCH_CMD_DAT 状态总 cy (拉 SG cmd 时间)
    //   hs_ring_wait_cy: 在 S_RING_WAIT 状态总 cy (line_buffer ring full 反压)
    //   hs_issue_cy:    在 S_ISSUE 状态总 cy (发 user data cmd)
    //   hs_data_cy:     在 S_DATA 状态总 cy (传 user data, mac_array 跑数据来源)
    //   hs_done_cy:     在 S_DONE 状态总 cy (等 sts drain)
    // =========================================================================
    logic [31:0] hs_fetch_cy, hs_ring_wait_cy, hs_issue_cy, hs_data_cy, hs_done_cy;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            hs_fetch_cy     <= '0;
            hs_ring_wait_cy <= '0;
            hs_issue_cy     <= '0;
            hs_data_cy      <= '0;
            hs_done_cy      <= '0;
        end else begin
            case (st)
                S_FETCH_CMD_ISS,
                S_FETCH_CMD_DAT: hs_fetch_cy     <= hs_fetch_cy     + 32'd1;
                S_RING_WAIT    : hs_ring_wait_cy <= hs_ring_wait_cy + 32'd1;
                S_ISSUE        : hs_issue_cy     <= hs_issue_cy     + 32'd1;
                S_DATA         : hs_data_cy      <= hs_data_cy      + 32'd1;
                S_DONE         : hs_done_cy      <= hs_done_cy      + 32'd1;
            endcase
        end
    end

endmodule
