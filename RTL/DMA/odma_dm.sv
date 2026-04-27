`timescale 1ns/1ps

// =============================================================================
// odma_dm.sv  --  Output DMA via Xilinx AXI DataMover S2MM
//
// 替换原 odma.sv 的 AXI4 write master 实现, 改用 DataMover S2MM. 对外接口
// (cfg / row_done_pulse / OFB SRAM 读端 / rows_drained) 与原 odma 兼容.
//
// 设计:
//   - 每输出行 1 条 S2MM cmd, BTT = w_out × cout_slices × 16 字节
//   - 收到 row_done_pulse → r_rows_produced++; r_rows_produced > r_rows_drained
//     时下发 cmd 并开始送 stream
//   - OFB SRAM NHWC gather: yout_base + cs_offset + x_rd, 每 stream beat 推进
//     (与原 odma 完全一致)
//   - stream tlast 在该行最后一拍拉起; tlast_fire → r_rows_drained++ → 释放 OFB
//   - DataMover S2MM 内部处理 AW/W/B + AXI burst 切分, 我们只发 stream
//
// 命令字 (与 idma_dm/wdma_dm 一致, EOF=1):
//   {4'b0, TAG_ODMA, dst_addr[31:0], DRR=0, EOF=1, DSA=0, TYPE=INCR, BTT}
//
// 状态机 (cmd 流水化, 利用 DataMover 内部 cmd FIFO):
//   S_IDLE → start → S_WAIT
//   S_WAIT → has_work_to_issue → S_CMD; all_issued → S_DRAIN
//   S_CMD → cmd_fire → S_PREFETCH (tready 自动反压: cmd FIFO 满会卡住)
//   S_PREFETCH → S_TX
//   S_TX → tlast_fire → S_WAIT (立刻准备下一条 cmd, 不等 sts)
//   S_DRAIN → all_drained → S_DONE
//
// 两个独立 counter:
//   rows_issued  : 在 cmd_fire 推进; 表示已下发到 DataMover cmd FIFO 的行数,
//                   决定下一条 cmd 用什么地址 + 还要不要发更多 cmd.
//   r_rows_drained : 在 sts_fire 推进; 表示 DataMover 已拿到 DDR BRESP 的行数,
//                   决定 OFB ring 反压 (rows_consumed/rows_drained 给 ofb_writer)
//                   + layer_done 何时拉起.
//
// 两者之差 = in-flight cmd 数, 但**我们不数它** —— DataMover 的
// s_axis_s2mm_cmd_tready 自动反压 (cmd FIFO 默认深度 4), 我们只管发, FIFO 满
// 自动卡 cmd_fire 即可.
//
// 性能要点: tlast_fire 后 1 cycle 就能再发新 cmd, 不像旧版要等 ~10 cy 拿 sts.
// 对每行计算时间短的 K=1 ds 层有显著加速.
// =============================================================================

module odma_dm #(
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

    // ---- Descriptor ----
    input  logic [ADDR_W-1:0]       dst_base,
    input  logic [LEN_W-1:0]        byte_len,        // 保留字段

    // ---- Streaming 配置 (与原 odma 一致) ----
    input  logic [15:0]             cfg_h_out_total,
    input  logic [15:0]             cfg_w_out,
    input  logic [5:0]              cfg_cout_slices,
    input  logic [19:0]             cfg_ofb_row_words,        // W_OUT × cout_slices
    input  logic [ADDR_W-1:0]       cfg_ddr_ofm_row_stride,
    input  logic [19:0]             cfg_ofb_ring_words,
    input  logic                    row_done_pulse,
    output logic [15:0]             rows_drained,

    // ---- DataMover S2MM CMD stream (out) ----
    output logic                    s2mm_cmd_tvalid,
    input  logic                    s2mm_cmd_tready,
    output logic [71:0]             s2mm_cmd_tdata,

    // ---- DataMover S2MM DATA stream (out, 我们送数据给 DataMover) ----
    output logic                    s2mm_data_tvalid,
    input  logic                    s2mm_data_tready,
    output logic [DATA_W-1:0]       s2mm_data_tdata,
    output logic [DATA_W/8-1:0]     s2mm_data_tkeep,
    output logic                    s2mm_data_tlast,

    // ---- DataMover S2MM STS stream (in) ----
    input  logic                    s2mm_sts_tvalid,
    output logic                    s2mm_sts_tready,
    input  logic [7:0]              s2mm_sts_tdata,

    // ---- OFB SRAM read port ----
    output logic                    ofb_re,
    output logic [SRAM_ADDR_W-1:0]  ofb_raddr,
    input  logic [DATA_W-1:0]       ofb_rdata
);

    // 兼容字段: byte_len 不用
    logic [LEN_W-1:0] _byte_len_unused;
    assign _byte_len_unused = byte_len;

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [2:0] {
        S_IDLE     = 3'd0,
        S_WAIT     = 3'd1,    // 等 row_done_pulse 攒够 1 行 / 流水间隙
        S_CMD      = 3'd2,    // 发 S2MM cmd (tready 自动反压 cmd FIFO 满)
        S_PREFETCH = 3'd3,    // 1 拍 OFB 读延迟
        S_TX       = 3'd4,    // 送 stream beats
        S_DRAIN    = 3'd5,    // 所有 cmd 已发, 等剩余 sts 全部回来
        S_DONE     = 3'd6
    } state_t;
    state_t state, state_next;

    // =========================================================================
    // 寄存器
    // =========================================================================
    logic [ADDR_W-1:0]       cur_addr;             // 当前要 issue 的下一条 cmd 的 DDR 起点
    logic [15:0]             row_beats_left;       // 当前 streaming row 剩余 beats
    logic [15:0]             r_rows_produced;      // 来自 ofb_writer 的 row_done_pulse 计数
    logic [15:0]             r_rows_issued;        // cmd_fire 计数 (cmd 流水线最前端)
    logic [15:0]             r_rows_drained;       // sts_fire 计数 (cmd 流水线最末端 = DDR commit)
    logic [SRAM_ADDR_W-1:0]  rd_ptr;
    logic [15:0]             x_rd_cnt;
    logic [5:0]              cs_rd_cnt;
    logic [SRAM_ADDR_W-1:0]  yout_base;
    logic [SRAM_ADDR_W-1:0]  cs_offset;
    logic                    r_done;
    logic                    r_err;

    assign rows_drained = r_rows_drained;
    assign err          = r_err;

    // =========================================================================
    // 派生量
    // =========================================================================
    logic cmd_fire, data_fire, tlast_fire, sts_fire;
    assign cmd_fire   = s2mm_cmd_tvalid  && s2mm_cmd_tready;
    assign data_fire  = s2mm_data_tvalid && s2mm_data_tready;
    assign tlast_fire = data_fire && s2mm_data_tlast;
    assign sts_fire   = s2mm_sts_tvalid  && s2mm_sts_tready;

    // has_work_to_issue: ofb_writer 已攒够的行数 > 我们已下发的 cmd 数, 且
    //                    还没把整图所有行都下发完.
    logic has_work_to_issue;
    assign has_work_to_issue = (r_rows_produced != r_rows_issued) &&
                               (r_rows_issued < cfg_h_out_total);

    logic all_issued;
    assign all_issued = (r_rows_issued >= cfg_h_out_total);

    // 全图 commit 完: 在 sts_fire 拍判定 (此刻 r_rows_drained 是旧值, 这条 sts
    // 当前正要算上)
    logic streaming_all_done;
    assign streaming_all_done = sts_fire &&
                                (r_rows_drained == cfg_h_out_total - 16'd1);

    // NHWC gather: 下一拍 OFB 地址
    logic cs_rd_is_last, x_rd_is_last, yout_rd_done;
    assign cs_rd_is_last = (cs_rd_cnt == cfg_cout_slices - 6'd1);
    assign x_rd_is_last  = (x_rd_cnt  == cfg_w_out        - 16'd1);
    assign yout_rd_done  = cs_rd_is_last && x_rd_is_last;

    logic [SRAM_ADDR_W-1:0] yb_plus, yout_base_next;
    assign yb_plus = yout_base + cfg_ofb_row_words[SRAM_ADDR_W-1:0];
    always_comb begin
        if (yb_plus >= cfg_ofb_ring_words[SRAM_ADDR_W-1:0])
            yout_base_next = yb_plus - cfg_ofb_ring_words[SRAM_ADDR_W-1:0];
        else
            yout_base_next = yb_plus;
    end

    logic [SRAM_ADDR_W-1:0] rd_ptr_next;
    always_comb begin
        if      (yout_rd_done)  rd_ptr_next = yout_base_next;
        else if (cs_rd_is_last) rd_ptr_next = yout_base + (x_rd_cnt + 16'd1);
        else                    rd_ptr_next = rd_ptr + cfg_w_out[SRAM_ADDR_W-1:0];
    end

    // BTT (字节) = w_out × cout_slices × 16
    logic [22:0] cmd_btt;
    assign cmd_btt = {cfg_ofb_row_words[18:0], 4'b0};

    // =========================================================================
    // 命令字
    // =========================================================================
    assign s2mm_cmd_tdata = {
        4'b0,            // [71:68] RSVD
        TAG_ODMA,        // [67:64] TAG
        cur_addr,        // [63:32] DADDR
        1'b0,            // [31]    DRR
        1'b1,            // [30]    EOF
        6'b0,            // [29:24] DSA
        1'b1,            // [23]    TYPE = INCR
        cmd_btt          // [22:0]  BTT
    };
    assign s2mm_cmd_tvalid = (state == S_CMD);

    // =========================================================================
    // 输出 stream (从 OFB 读 → DataMover S2MM)
    //   tvalid/tlast 在 S_TX, 数据来自 ofb_rdata (上一拍 ofb_re 已发起)
    // =========================================================================
    assign s2mm_data_tvalid = (state == S_TX);
    assign s2mm_data_tdata  = ofb_rdata;
    assign s2mm_data_tkeep  = '1;
    assign s2mm_data_tlast  = (state == S_TX) && (row_beats_left == 16'd1);
    assign s2mm_sts_tready  = 1'b1;

    // OFB read: PREFETCH 拍 latch 第一 beat; TX 阶段每 fire 推进 rd_ptr_next
    assign ofb_re    = (state == S_PREFETCH) || (state == S_TX);
    assign ofb_raddr = (state == S_TX && data_fire) ? rd_ptr_next : rd_ptr;

    assign done = r_done && !start;
    assign busy = (state != S_IDLE) && (state != S_DONE);

    // =========================================================================
    // FSM
    // =========================================================================
    always_comb begin
        state_next = state;
        case (state)
            S_IDLE     : if (start)                     state_next = S_WAIT;
            S_WAIT     : if      (has_work_to_issue)    state_next = S_CMD;
                         else if (all_issued)           state_next = S_DRAIN;
            S_CMD      : if (cmd_fire)                  state_next = S_PREFETCH;
            S_PREFETCH :                                state_next = S_TX;
            S_TX       : if (tlast_fire)                state_next = S_WAIT;
            S_DRAIN    : if (streaming_all_done)        state_next = S_DONE;
            S_DONE     : if (start)                     state_next = S_WAIT;
            default    :                                state_next = S_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= state_next;
    end

    // =========================================================================
    // 数据路径 (start 初始化)
    // =========================================================================
    // r_rows_produced: 来自 ofb_writer 的 row_done_pulse
    always_ff @(posedge clk) begin
        if      (start)              r_rows_produced <= 16'd0;
        else if (row_done_pulse)     r_rows_produced <= r_rows_produced + 16'd1;
        else                         r_rows_produced <= r_rows_produced;
    end

    // r_rows_issued: cmd_fire 推进 (cmd 已下发到 DataMover cmd FIFO)
    always_ff @(posedge clk) begin
        if      (start)        r_rows_issued <= 16'd0;
        else if (cmd_fire)     r_rows_issued <= r_rows_issued + 16'd1;
        else                   r_rows_issued <= r_rows_issued;
    end

    // r_rows_drained: sts_fire 推进 (= DDR 写已 commit)
    always_ff @(posedge clk) begin
        if      (start)        r_rows_drained <= 16'd0;
        else if (sts_fire)     r_rows_drained <= r_rows_drained + 16'd1;
        else                   r_rows_drained <= r_rows_drained;
    end

    // cur_addr: 给 cmd 用 → 在 cmd_fire 推进 (跟 r_rows_issued 同步)
    always_ff @(posedge clk) begin
        if      (start)        cur_addr <= dst_base;
        else if (cmd_fire)     cur_addr <= cur_addr + cfg_ddr_ofm_row_stride;
        else                   cur_addr <= cur_addr;
    end

    // row_beats_left: 进入新行重置, 每 data_fire -1
    always_ff @(posedge clk) begin
        if      (start)                            row_beats_left <= cfg_ofb_row_words[15:0];
        else if (tlast_fire)                       row_beats_left <= cfg_ofb_row_words[15:0];
        else if (data_fire)                        row_beats_left <= row_beats_left - 16'd1;
        else                                       row_beats_left <= row_beats_left;
    end

    // NHWC gather counters (与原 odma 一致)
    always_ff @(posedge clk) begin
        if      (start)     rd_ptr <= '0;
        else if (data_fire) rd_ptr <= rd_ptr_next;
        else                rd_ptr <= rd_ptr;
    end

    always_ff @(posedge clk) begin
        if      (start)     cs_rd_cnt <= '0;
        else if (data_fire) begin
            if (cs_rd_is_last) cs_rd_cnt <= '0;
            else               cs_rd_cnt <= cs_rd_cnt + 6'd1;
        end
    end

    always_ff @(posedge clk) begin
        if      (start)                      x_rd_cnt <= '0;
        else if (data_fire && cs_rd_is_last) begin
            if (x_rd_is_last)    x_rd_cnt <= '0;
            else                 x_rd_cnt <= x_rd_cnt + 16'd1;
        end
    end

    always_ff @(posedge clk) begin
        if      (start)     cs_offset <= '0;
        else if (data_fire) begin
            if (cs_rd_is_last) cs_offset <= '0;
            else               cs_offset <= cs_offset + cfg_w_out[SRAM_ADDR_W-1:0];
        end
    end

    always_ff @(posedge clk) begin
        if      (start)                    yout_base <= '0;
        else if (data_fire && yout_rd_done) yout_base <= yout_base_next;
    end

    // =========================================================================
    // 控制
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (!rst_n)              r_done <= 1'b0;
        else if (start)               r_done <= 1'b0;
        else if (streaming_all_done)  r_done <= 1'b1;
        else                          r_done <= r_done;
    end

    always_ff @(posedge clk) begin
        if      (!rst_n)                            r_err <= 1'b0;
        else if (start)                             r_err <= 1'b0;
        else if (sts_fire && !s2mm_sts_tdata[7])    r_err <= 1'b1;
        else                                        r_err <= r_err;
    end

endmodule
