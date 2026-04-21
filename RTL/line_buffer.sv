`timescale 1ns/1ps

// =============================================================================
// line_buffer.sv  --  Activation Stream Feeder (IFB → ring act_buf → MAC)
//
// v3 优化：跨 round pipeline。消除 v2 中 per-round 的 2 拍 startup bubble。
// Phase C-2 扩展：padding 支持 (pad_top / pad_left 影响 issue 坐标；越界位置
// 不发 IFB 读、直接给 act_buf 喂 0)。
//
// 核心思想：
//   - consume 侧不需要 counter（只顺序读 act_buf）；act_buf 作纯 ring buffer，
//     读/写指针模 ARF_DEPTH，永不随 round 边界重置。
//   - issue 侧是唯一的 counter 源。当 iss_pos 走完当前 round 最后一个位置，
//     同拍 advance 外层 counter (kx/ky/cins/tile/yout/cs) 和 ptr_kx_base。
//     下一拍立即 issue 新 round 的第 0 个 read，无需 bubble。
//   - fifo_count 追踪 act_buf 占用；issue 以 (fifo_count+in_flight < ARF_DEPTH)
//     背压，mac 吃得慢时 issue 自然停。
//
// Padding 实现（Phase C-2）：
//   - 用 signed 累加器 y_row_base / x_tile_base 跟踪窗口起点，避免 signed 乘法
//     (yout*stride, tile*tile_w*stride) 在 line_buffer 内部展开
//       y_row_base:  strip 开始 = -pad_top；每 yout wrap += stride
//       x_tile_base: strip 开始 = -pad_left；每 tile wrap += tile_in_step
//       iss_pos_s:   = iss_pos * stride （复用原 ifb_rd_offset 的 shift 逻辑）
//   - current_src_y = y_row_base + ky_cnt (signed)
//     current_src_x = x_tile_base + iss_pos_s + kx_cnt (signed)
//   - is_pad = (src_y<0) | (src_y>=cfg_h_in) | (src_x<0) | (src_x>=cfg_w_in)
//   - issue_advance 每拍 +1 (含 pad)；ifb_re 只在非 pad 拍 1
//   - arrival 用 is_pad_d1 选择 0 或 ifb_rdata 入 act_buf
//   - cfg_ifb_base 由 Sequencer 预扣 pad offset: cfg_ifb_base - pad_top*w_in
//     - pad_left。ptr_*_base 推进逻辑不变（=虚拟坐标累加），物理 IFB 地址
//     在 ky=pad_top && kx=pad_left (首个非 pad 位置) 时自然指向全局 cfg_ifb_base。
//
// 6 层外层循环 (issue 侧自跑)：for cs / yout / tile / cins / ky / kx
// =============================================================================

module line_buffer #(
    parameter int NUM_PE     = 16,
    parameter int DATA_WIDTH = 8,
    parameter int ARF_DEPTH  = 32,
    parameter int SRAM_DEPTH = 8192,
    parameter int ADDR_W     = 20
)(
    input  logic                                 clk,
    input  logic                                 rst_n,
    input  logic                                 start,
    output logic                                 done,

    // ---- cfg ----
    input  logic [15:0]                          cfg_h_out,
    input  logic [15:0]                          cfg_w_in,
    input  logic [3:0]                           cfg_k,
    input  logic [2:0]                           cfg_stride,
    input  logic [5:0]                           cfg_cin_slices,
    input  logic [5:0]                           cfg_cout_slices,
    input  logic [5:0]                           cfg_tile_w,
    input  logic [7:0]                           cfg_num_tiles,
    input  logic [5:0]                           cfg_last_valid_w,
    input  logic [ADDR_W-1:0]                    cfg_ifb_base,
    input  logic [ADDR_W-1:0]                    cfg_ifb_ring_words,
    input  logic [ADDR_W-1:0]                    cfg_ifb_row_step,
    input  logic [ADDR_W-1:0]                    cfg_tile_in_step,
    input  logic [ADDR_W-1:0]                    cfg_iss_step,    // stride × cin_slices (iss_pos 步长 word)
    input  logic [ADDR_W-1:0]                    cfg_ifb_ky_step, // W_IN × cin_slices (ky 步长 word)
    input  logic [15:0]                          cfg_tile_pix_step, // TILE_W × stride (像素步长，用于 pad 判定)
    input  logic                                 cfg_arf_reuse_en,  // 1: kx sliding-window reuse（仅 stride=1 && K>1）

    // ---- Padding (Phase C-2) ----
    input  logic [3:0]                           cfg_pad_top,
    input  logic [3:0]                           cfg_pad_left,
    input  logic [15:0]                          cfg_h_in,     // 虚拟图像高度 (pad 前的有效输入行数)

    // ---- Streaming 配置（v2） ----
    // cfg_idma_streaming=1 时启用 ring wrap：每个 cin_slice 在 IFB 占
    // cfg_ifb_ring_words = ifb_strip_rows × W_IN × cin_slices；pointer 推进时 mod 这个值
    // 实现回卷。batch 模式下 (=0) 不 wrap，行为与 v1 完全一致。
    input  logic                                 cfg_idma_streaming,

    // ---- IFB 读端口 ----
    output logic                                 ifb_re,
    output logic [$clog2(SRAM_DEPTH)-1:0]        ifb_raddr,
    input  logic [NUM_PE*DATA_WIDTH-1:0]         ifb_rdata,

    // ---- act stream → mac_array ----
    output logic                                 act_valid,
    output logic [NUM_PE*DATA_WIDTH-1:0]         act_vec,
    input  logic                                 act_ready,

    // ---- Row credit 输出（给 IDMA streaming mode） ----
    // rows_consumed = "line_buffer 已彻底用完的输入行数"，yout 每推进 1 就 +stride
    output logic [15:0]                          rows_consumed,

    // ---- Row forward-pressure 输入（streaming 下 IDMA 已写完的行数） ----
    input  logic [15:0]                          rows_available
);

    localparam int AW      = $clog2(SRAM_DEPTH);
    localparam int BUF_AW  = $clog2(ARF_DEPTH);    // 5 bits for ARF_DEPTH=32
    localparam int CNT_W   = $clog2(ARF_DEPTH+1);  // 6 bits to hold 0..32

    // =========================================================================
    // 状态机
    // =========================================================================
    typedef enum logic [1:0] {
        S_IDLE = 2'd0,
        S_RUN  = 2'd1,
        S_DONE = 2'd2
    } state_t;
    state_t state, state_next;

    // =========================================================================
    // 寄存器声明
    // =========================================================================
    // 外层 6 级 counter
    logic [3:0]  kx_cnt;
    logic [3:0]  ky_cnt;
    logic [5:0]  cins_cnt;
    logic [7:0]  tile_cnt;
    logic [15:0] yout_cnt;
    logic [5:0]  cs_cnt;

    // 5 层 running base（cs 不影响 IFB）
    logic [ADDR_W-1:0] ptr_yout_base;
    logic [ADDR_W-1:0] ptr_tile_base;
    logic [ADDR_W-1:0] ptr_cins_base;
    logic [ADDR_W-1:0] ptr_ky_base;
    logic [ADDR_W-1:0] ptr_kx_base;

    // Issue 内部 state
    logic [5:0]  iss_pos;            // 0..cur_valid_w-1
    logic        issues_all_done;
    logic        issue_advance_d1;   // 原 ifb_re_d1 语义重命名：每个 issue 动作延迟 1 拍 (含 pad)

    // ARF reuse (Phase E-2) —— FILL 和 CONSUME 完全并行（同原 v3 pipeline 模型）
    //   FILL 是"生产者": fill_pos 推进，发 ifb_re，每 arrival 写 ARF 线性地址 wr_idx_fill
    //   CONSUME 是"消费者": kx × iss_pos 推进（act_fire 驱动），组合读 ARF[kx+iss_pos]
    //   背压: act_valid = (rd_idx_cons < wr_idx_fill)  —— 读的位置已经被 arrival 填完
    //   每 ky 切换: evt_iss_kx_wrap 拍 CONSUME 完成最后一读，下一拍 FILL 归 0 重新开始
    logic [5:0]        fill_pos;       // 0..cur_fill_len-1 (+ terminal = cur_fill_len)
    logic [ADDR_W-1:0] fill_offset;    // = fill_pos × cin_slices (word 累加器)
    logic [BUF_AW:0]   wr_idx_fill;    // reuse_en=1 FILL 线性写指针（每 ky 归 0，上限 cur_fill_len ≤ ARF_DEPTH）

    // Padding 累加器 (Phase C-2)
    logic signed [16:0] y_row_base;   // yout=0 时 = -pad_top；每 yout wrap += stride
    logic signed [16:0] x_tile_base;  // tile=0 时 = -pad_left；每 tile wrap += tile_in_step
    logic               is_pad_d1;    // arrival 对齐的 pad 标志

    // Ring buffer
    logic [BUF_AW-1:0] wr_idx;
    logic [BUF_AW-1:0] rd_idx;
    logic [CNT_W-1:0]  fifo_count;
    logic [NUM_PE*DATA_WIDTH-1:0] act_buf [0:ARF_DEPTH-1];

    // =========================================================================
    // 派生量 + 边界
    // =========================================================================
    logic [5:0] cur_valid_w;
    assign cur_valid_w = (tile_cnt == cfg_num_tiles - 8'd1) ? cfg_last_valid_w : cfg_tile_w;

    // FILL 长度：仅 reuse_en=1 下有意义；= cur_valid_w + K - 1（滑动窗口覆盖 kx 全范围）
    // 编译器必须保证 cur_fill_len ≤ ARF_DEPTH（tile_w ≤ 33-K）
    logic [5:0] cur_fill_len;
    assign cur_fill_len = cur_valid_w + {2'd0, cfg_k} - 6'd1;

    logic fill_pos_is_terminal;   // fill_pos 已跑满（= cur_fill_len），不再发 fill_issue
    assign fill_pos_is_terminal = (fill_pos == cur_fill_len);

    logic kx_is_last, ky_is_last, cins_is_last, tile_is_last, yout_is_last, cs_is_last;
    assign kx_is_last   = (kx_cnt   == cfg_k             - 4'd1);
    assign ky_is_last   = (ky_cnt   == cfg_k             - 4'd1);
    assign cins_is_last = (cins_cnt == cfg_cin_slices    - 6'd1);
    assign tile_is_last = (tile_cnt == cfg_num_tiles     - 8'd1);
    assign yout_is_last = (yout_cnt == cfg_h_out         - 16'd1);
    assign cs_is_last   = (cs_cnt   == cfg_cout_slices   - 6'd1);

    logic iss_pos_is_last;
    assign iss_pos_is_last = (iss_pos == cur_valid_w - 6'd1);

    logic all_is_last;
    assign all_is_last = iss_pos_is_last && kx_is_last && ky_is_last &&
                         cins_is_last && tile_is_last && yout_is_last && cs_is_last;

    // =========================================================================
    // Padding 判定（组合）—— 用累加器 + ky/kx 得 signed 坐标，和 cfg_h_in/w_in 比较
    //   状态机三态（PAD_TOP → DATA → PAD_BOT 以 ky 为自然维度；PAD_LEFT →
    //   DATA → PAD_RIGHT 以 kx 为自然维度）由 is_pad_{top/bot/left/right} 组合
    //   信号隐式表达，无需独立 state 寄存器。
    // =========================================================================
    // iss_pos_s = iss_pos * stride（继承原 ifb_rd_offset 的 shift 实现，支持 stride 1/2）
    logic signed [16:0] iss_pos_s;
    always_comb begin
        case (cfg_stride)
            3'd1:    iss_pos_s = $signed({11'd0, iss_pos});
            3'd2:    iss_pos_s = $signed({10'd0, iss_pos, 1'b0});
            default: iss_pos_s = $signed({11'd0, iss_pos});
        endcase
    end

    logic signed [16:0] current_src_y, current_src_x;
    assign current_src_y = y_row_base  + $signed({13'd0, ky_cnt});
    assign current_src_x = x_tile_base + iss_pos_s + $signed({13'd0, kx_cnt});

    logic is_pad_top_y, is_pad_bot_y, is_pad_left_x, is_pad_right_x, is_pad;
    assign is_pad_top_y   = (current_src_y < 0);
    assign is_pad_bot_y   = ($signed({1'b0, cfg_h_in}) <= current_src_y);
    assign is_pad_left_x  = (current_src_x < 0);
    assign is_pad_right_x = ($signed({1'b0, cfg_w_in}) <= current_src_x);
    assign is_pad         = is_pad_top_y | is_pad_bot_y | is_pad_left_x | is_pad_right_x;

    // =========================================================================
    // 握手 + IFB 读地址
    // =========================================================================
    // Streaming forward-pressure: yout=Y 需要的最大行号 < rows_available 才可 issue。
    //   公式（考虑 pad_top / pad_bot）：
    //     rows_needed = clamp(y_row_base + K, 0, cfg_h_in)
    //   - 下界 0：本 yout 完全在 pad_top 区，无需 IDMA 提供
    //   - 上界 cfg_h_in：kernel 底边超出图像（落在 pad_bot 区），pad_bot 行是虚拟的，
    //     不会由 IDMA 提供；rows_needed 不能超过 cfg_h_in，否则永远 stall
    logic signed [16:0] rows_needed_signed;
    logic [15:0] rows_needed;
    assign rows_needed_signed = y_row_base + $signed({13'd0, cfg_k});
    assign rows_needed        = (rows_needed_signed < 0)                        ? 16'd0
                              : (rows_needed_signed > $signed({1'b0, cfg_h_in})) ? cfg_h_in
                              : rows_needed_signed[15:0];

    logic streaming_rows_ready;
    assign streaming_rows_ready = !cfg_idma_streaming || (rows_available >= rows_needed);

    // =========================================================================
    // Issue 信号（两种模式）
    //   reuse_en=0 (原 v3 行为)：iss_pos × kx × ky × ... 直接扫 IFB，每 issue_ok 发一次 ifb_re
    //   reuse_en=1 (E-2 sliding window)：每 (tile,cins,ky) 开头 FILL cur_fill_len 个像素到
    //     ARF[0..cur_fill_len-1]；CONSUME 阶段 kx+iss_pos 只读 ARF，不发 ifb_re。
    // =========================================================================
    logic issue_ok_std;  // reuse_en=0 路径
    assign issue_ok_std = (state == S_RUN) && !issues_all_done &&
                          ({1'b0, fifo_count} + {6'b0, issue_advance_d1} < (CNT_W+1)'(ARF_DEPTH)) &&
                          streaming_rows_ready;

    logic fill_issue;    // reuse_en=1 每拍发 IFB 读直到 fill_pos 达 cur_fill_len；与 CONSUME 并行
    assign fill_issue = cfg_arf_reuse_en && (state == S_RUN) && !issues_all_done &&
                        !fill_pos_is_terminal && streaming_rows_ready;

    // "任何 issue" —— 用于 arrival / is_pad_d1 latch / issue_advance_d1
    logic issue_any;
    assign issue_any = cfg_arf_reuse_en ? fill_issue : issue_ok_std;

    // 物理 IFB 读地址：
    //   reuse_en=0: iss_offset 累加器 = iss_pos × cfg_iss_step（原）
    //   reuse_en=1: fill_offset 累加器 = fill_pos × cin_slices（FILL 阶段线性填一整行）
    //     base 用 ptr_ky_base（当前 ky 行起点，稳定整个 FILL+CONSUME 周期）
    //     不能用 ptr_kx_base —— CONSUME 并行运行会让它随 kx wrap 漂移
    logic [ADDR_W-1:0] iss_offset;

    // FILL 阶段专用 pad 判定：x = x_tile_base + fill_pos（stride=1 only）
    logic signed [16:0] current_src_x_fill;
    assign current_src_x_fill = x_tile_base + $signed({11'd0, fill_pos});
    logic is_pad_fill;
    assign is_pad_fill = (current_src_y < 0) || ($signed({1'b0, cfg_h_in}) <= current_src_y) ||
                         (current_src_x_fill < 0) || ($signed({1'b0, cfg_w_in}) <= current_src_x_fill);

    logic is_pad_now;    // 给 ifb_re gate 和 is_pad_d1 latch
    assign is_pad_now = cfg_arf_reuse_en ? is_pad_fill : is_pad;

    assign ifb_re    = issue_any && !is_pad_now;
    // ptr_*_base 已经是 ring-wrapped，但 +offset 可能再次跨 ring 边界，需再 wrap 一次
    logic [ADDR_W-1:0] ifb_raddr_raw;
    assign ifb_raddr_raw = cfg_arf_reuse_en ? (ptr_ky_base + fill_offset)
                                            : (ptr_kx_base + iss_offset);
    assign ifb_raddr     = wrap_addr(ifb_raddr_raw);

    // act_valid / act_vec
    //   reuse_en=0: fifo_count > 0 （原）
    //   reuse_en=1: sub_phase=CONS 时 ARF 已填好，直接组合读 kx+iss_pos
    logic [BUF_AW:0]    rd_idx_cons_wide;
    logic [BUF_AW-1:0]  rd_idx_cons;
    assign rd_idx_cons_wide = {1'b0, kx_cnt} + {{(BUF_AW+1-6){1'b0}}, iss_pos};
    assign rd_idx_cons      = rd_idx_cons_wide[BUF_AW-1:0];

    // reuse_en=1 act_valid：rd_idx_cons 位置已 filled（wr_idx_fill 严格大于 rd_idx_cons）
    logic reuse_act_valid;
    assign reuse_act_valid = (state == S_RUN) && !issues_all_done &&
                             ({1'b0, rd_idx_cons} < wr_idx_fill);
    assign act_valid = cfg_arf_reuse_en ? reuse_act_valid : (fifo_count > 0);
    assign act_vec   = act_buf[cfg_arf_reuse_en ? rd_idx_cons : rd_idx];

    logic act_fire, arrival;
    assign act_fire = act_valid & act_ready;
    assign arrival  = issue_advance_d1;          // 每个 issue 动作（含 pad）1 拍后入队

    // =========================================================================
    // 命名事件信号（comb）—— 方式 1：cs 下移到 yout 内
    //   iss_pos → kx → ky → cins → tile → cs → yout（从内到外）
    //   evt_iss_pos_wrap : iss_pos 走完 cur_valid_w（一个 (ky,kx) round）
    //   evt_iss_kx_wrap  : kx 进位 (ky 推进)
    //   evt_iss_ky_wrap  : ky 进位 (cins 推进)
    //   evt_iss_cins_wrap: cins 进位 (tile 推进)
    //   evt_iss_tile_wrap: tile 进位 (cs 推进 —— 新: 一个 (yout,cs) 完成)
    //   evt_iss_cs_wrap  : cs 进位 (yout 推进 —— 新: 一个 yout 的所有 cs 完成)
    //   evt_iss_yout_wrap: yout 进位 (整 strip 结束)
    // =========================================================================
    logic evt_start;
    logic evt_iss_pos_wrap;
    logic evt_iss_kx_wrap;
    logic evt_iss_ky_wrap;
    logic evt_iss_cins_wrap;
    logic evt_iss_tile_wrap;
    logic evt_iss_cs_wrap;
    logic evt_iss_yout_wrap;

    // 统一推进信号：reuse_en=0 由 issue_ok_std 驱动（原行为）；reuse_en=1 由 CONSUME 的 act_fire 驱动
    logic evt_advance_any;
    assign evt_advance_any = cfg_arf_reuse_en ? act_fire : issue_ok_std;

    always_comb begin
        evt_start         = ((state == S_IDLE) || (state == S_DONE)) && start;
        evt_iss_pos_wrap  = evt_advance_any    && iss_pos_is_last;
        evt_iss_kx_wrap   = evt_iss_pos_wrap   && kx_is_last;
        evt_iss_ky_wrap   = evt_iss_kx_wrap    && ky_is_last;
        evt_iss_cins_wrap = evt_iss_ky_wrap    && cins_is_last;
        evt_iss_tile_wrap = evt_iss_cins_wrap  && tile_is_last;
        evt_iss_cs_wrap   = evt_iss_tile_wrap  && cs_is_last;
        evt_iss_yout_wrap = evt_iss_cs_wrap    && yout_is_last;
    end


    // =========================================================================
    // 三段式 FSM
    // =========================================================================
    // reuse_en=0: 等 fifo 被 mac 排空 + 最后一 arrival 落地
    // reuse_en=1: issues_all_done 由 CONSUME 的 act_fire 设置，设置时 FILL 已完成 + CONSUME 全部产出
    logic run_drained;
    assign run_drained = cfg_arf_reuse_en ? (issues_all_done && !issue_advance_d1)
                                          : (issues_all_done && fifo_count == 0 && !issue_advance_d1);

    always_comb begin
        state_next = state;
        case (state)
            S_IDLE : if (start)          state_next = S_RUN;
            S_RUN  : if (run_drained)    state_next = S_DONE;
            S_DONE : if (start)          state_next = S_RUN;   // 多 strip 重启
            default:                     state_next = S_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= state_next;
    end

    assign done = (state == S_DONE);

    // =========================================================================
    // 外层 6 级 counter
    //   state=S_IDLE 下 issue_ok=0，counter 值不影响任何输出；evt_start 初始化。
    //   按 §6 不加复位。
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (evt_start)          iss_pos <= '0;
        else if (evt_iss_pos_wrap)   iss_pos <= '0;
        else if (evt_advance_any)    iss_pos <= iss_pos + 6'd1;
        else                         iss_pos <= iss_pos;
    end

    always_ff @(posedge clk) begin
        if      (evt_start)          kx_cnt <= '0;
        else if (evt_iss_kx_wrap)    kx_cnt <= '0;
        else if (evt_iss_pos_wrap)   kx_cnt <= kx_cnt + 4'd1;
        else                         kx_cnt <= kx_cnt;
    end

    always_ff @(posedge clk) begin
        if      (evt_start)          ky_cnt <= '0;
        else if (evt_iss_ky_wrap)    ky_cnt <= '0;
        else if (evt_iss_kx_wrap)    ky_cnt <= ky_cnt + 4'd1;
        else                         ky_cnt <= ky_cnt;
    end

    always_ff @(posedge clk) begin
        if      (evt_start)          cins_cnt <= '0;
        else if (evt_iss_cins_wrap)  cins_cnt <= '0;
        else if (evt_iss_ky_wrap)    cins_cnt <= cins_cnt + 6'd1;
        else                         cins_cnt <= cins_cnt;
    end

    always_ff @(posedge clk) begin
        if      (evt_start)          tile_cnt <= '0;
        else if (evt_iss_tile_wrap)  tile_cnt <= '0;
        else if (evt_iss_cins_wrap)  tile_cnt <= tile_cnt + 8'd1;
        else                         tile_cnt <= tile_cnt;
    end

    // 方式 1：cs 在 yout 内 → cs 由 evt_iss_tile_wrap 推进，yout 由 evt_iss_cs_wrap 推进
    always_ff @(posedge clk) begin
        if      (evt_start)          cs_cnt <= '0;
        else if (evt_iss_cs_wrap)    cs_cnt <= '0;
        else if (evt_iss_tile_wrap)  cs_cnt <= cs_cnt + 6'd1;
        else                         cs_cnt <= cs_cnt;
    end

    always_ff @(posedge clk) begin
        if      (evt_start)          yout_cnt <= '0;
        else if (evt_iss_yout_wrap)  yout_cnt <= '0;
        else if (evt_iss_cs_wrap)    yout_cnt <= yout_cnt + 16'd1;
        else                         yout_cnt <= yout_cnt;
    end

    // issues_all_done 是控制标志，参与 state_next 判定 → 复位（§6.1）。
    always_ff @(posedge clk) begin
        if      (!rst_n)                           issues_all_done <= 1'b0;
        else if (evt_start)                        issues_all_done <= 1'b0;
        else if (evt_iss_pos_wrap && all_is_last)  issues_all_done <= 1'b1;
        else                                       issues_all_done <= issues_all_done;
    end

    // iss_offset 累加器（reuse_en=0 专用；reuse_en=1 时保持 0 不用）
    always_ff @(posedge clk) begin
        if      (evt_start)          iss_offset <= '0;
        else if (evt_iss_pos_wrap)   iss_offset <= '0;
        else if (issue_ok_std)       iss_offset <= iss_offset + cfg_iss_step;
        else                         iss_offset <= iss_offset;
    end

    // fill_pos / fill_offset / wr_idx_fill (reuse_en=1 FILL; 每 ky 归 0，与 CONSUME 并行)
    //   归零时机: evt_start（首次）或 evt_iss_kx_wrap（CONSUME 走完当前 ky，下一拍开始新 ky 的 FILL）
    //   fill_pos: 每 fill_issue +1，达 cur_fill_len 后停
    //   wr_idx_fill: 每 arrival +1（cum 与 fill_pos 差 1 拍延迟）
    logic fill_reset;
    assign fill_reset = evt_start || evt_iss_kx_wrap;

    always_ff @(posedge clk) begin
        if      (!rst_n)        fill_pos <= '0;
        else if (fill_reset)    fill_pos <= '0;
        else if (fill_issue)    fill_pos <= fill_pos + 6'd1;
        else                    fill_pos <= fill_pos;
    end

    always_ff @(posedge clk) begin
        if      (fill_reset)    fill_offset <= '0;
        else if (fill_issue)    fill_offset <= fill_offset + {{(ADDR_W-6){1'b0}}, cfg_cin_slices};
        else                    fill_offset <= fill_offset;
    end

    // =========================================================================
    // 5 层 running base（数据路径，无复位；evt_start 初始化）
    //   层级继承：外层进位时，内层 base 跟随外层新值重置；否则按自身步长推进。
    //
    //   Streaming ring wrap（cfg_idma_streaming=1）：
    //     NHWC 布局：每行含所有 cin_slice 的 word 相邻，跨 cin_slice 步长 = 1 word。
    //     ring wrap 模数 = cfg_ifb_ring_words = ifb_strip_rows × W_IN × cin_slices。
    //   Batch 模式 (cfg_idma_streaming=0): wrap 函数 no-op，行为与 v1 完全一致。
    // =========================================================================

    // 组合 wrap 函数：若启用 streaming 且 val 跨过 cin_step 边界，减去一份 cin_step
    function automatic logic [ADDR_W-1:0] wrap_addr(input logic [ADDR_W-1:0] val);
        if (cfg_idma_streaming && val >= cfg_ifb_ring_words)
            wrap_addr = val - cfg_ifb_ring_words;
        else
            wrap_addr = val;
    endfunction

    logic [ADDR_W-1:0] w_yout_plus_rowstep;    // 外层到 ptr_*_base 的级联候选值
    logic [ADDR_W-1:0] w_tile_plus_tilestep;
    logic [ADDR_W-1:0] w_ky_plus_win;
    logic [ADDR_W-1:0] w_kx_plus_one;
    assign w_yout_plus_rowstep  = wrap_addr(ptr_yout_base + cfg_ifb_row_step);
    assign w_tile_plus_tilestep = wrap_addr(ptr_tile_base + cfg_tile_in_step);
    assign w_ky_plus_win        = wrap_addr(ptr_ky_base   + cfg_ifb_ky_step);
    assign w_kx_plus_one        = wrap_addr(ptr_kx_base   + {{(ADDR_W-6){1'b0}}, cfg_cin_slices});

    always_ff @(posedge clk) begin
        if      (evt_start)                           ptr_yout_base <= cfg_ifb_base;
        else if (evt_iss_yout_wrap)    ptr_yout_base <= cfg_ifb_base;
        else if (evt_iss_cs_wrap && !yout_is_last)  ptr_yout_base <= w_yout_plus_rowstep;
        else                                          ptr_yout_base <= ptr_yout_base;
    end

    always_ff @(posedge clk) begin
        if      (evt_start)                           ptr_tile_base <= cfg_ifb_base;
        else if (evt_iss_yout_wrap)                   ptr_tile_base <= cfg_ifb_base;
        else if (evt_iss_cs_wrap && !yout_is_last)    ptr_tile_base <= w_yout_plus_rowstep;
        else if (evt_iss_tile_wrap && !cs_is_last)    ptr_tile_base <= ptr_yout_base;  // cs 切换回 yout 起点
        else if (evt_iss_cins_wrap && !tile_is_last)  ptr_tile_base <= w_tile_plus_tilestep;
        else                                          ptr_tile_base <= ptr_tile_base;
    end

    // NHWC 下跨 cin_slice 步长 = 1 word（相邻），所有累加仍可能 wrap 所以统一 wrap_addr
    always_ff @(posedge clk) begin
        if      (evt_start)                           ptr_cins_base <= cfg_ifb_base;
        else if (evt_iss_yout_wrap)                   ptr_cins_base <= cfg_ifb_base;
        else if (evt_iss_cs_wrap && !yout_is_last)    ptr_cins_base <= w_yout_plus_rowstep;
        else if (evt_iss_tile_wrap && !cs_is_last)    ptr_cins_base <= ptr_yout_base;  // cs 切换
        else if (evt_iss_cins_wrap && !tile_is_last)  ptr_cins_base <= w_tile_plus_tilestep;
        else if (evt_iss_ky_wrap   && !cins_is_last)  ptr_cins_base <= wrap_addr(ptr_cins_base + {{(ADDR_W-1){1'b0}}, 1'b1});
        else                                          ptr_cins_base <= ptr_cins_base;
    end

    always_ff @(posedge clk) begin
        if      (evt_start)                           ptr_ky_base <= cfg_ifb_base;
        else if (evt_iss_yout_wrap)                   ptr_ky_base <= cfg_ifb_base;
        else if (evt_iss_cs_wrap && !yout_is_last)    ptr_ky_base <= w_yout_plus_rowstep;
        else if (evt_iss_tile_wrap && !cs_is_last)    ptr_ky_base <= ptr_yout_base;  // cs 切换
        else if (evt_iss_cins_wrap && !tile_is_last)  ptr_ky_base <= w_tile_plus_tilestep;
        else if (evt_iss_ky_wrap   && !cins_is_last)  ptr_ky_base <= wrap_addr(ptr_cins_base + {{(ADDR_W-1){1'b0}}, 1'b1});
        else if (evt_iss_kx_wrap   && !ky_is_last)    ptr_ky_base <= w_ky_plus_win;
        else                                          ptr_ky_base <= ptr_ky_base;
    end

    always_ff @(posedge clk) begin
        if      (evt_start)                           ptr_kx_base <= cfg_ifb_base;
        else if (evt_iss_yout_wrap)                   ptr_kx_base <= cfg_ifb_base;
        else if (evt_iss_cs_wrap && !yout_is_last)    ptr_kx_base <= w_yout_plus_rowstep;
        else if (evt_iss_tile_wrap && !cs_is_last)    ptr_kx_base <= ptr_yout_base;  // cs 切换
        else if (evt_iss_cins_wrap && !tile_is_last)  ptr_kx_base <= w_tile_plus_tilestep;
        else if (evt_iss_ky_wrap   && !cins_is_last)  ptr_kx_base <= wrap_addr(ptr_cins_base + {{(ADDR_W-1){1'b0}}, 1'b1});
        else if (evt_iss_kx_wrap   && !ky_is_last)    ptr_kx_base <= w_ky_plus_win;
        else if (evt_iss_pos_wrap  && !kx_is_last)    ptr_kx_base <= w_kx_plus_one;
        else                                          ptr_kx_base <= ptr_kx_base;
    end

    // =========================================================================
    // rows_consumed: 已彻底用完的输入行数（对 IDMA streaming ring_full 判定用）
    //   raw 累加器 = yout 已完成次数 × stride；输出 = max(raw - pad_top, 0)
    //   pad_top 行是虚拟的（未进入 ring），前 pad_top/stride+1 个 yout 重用 row 0，
    //   不能过早释放。rows_consumed 不减掉 pad_top 会导致 IDMA 覆盖尚在使用的行。
    // =========================================================================
    logic [15:0] rows_consumed_raw;
    always_ff @(posedge clk) begin
        if      (evt_start)                           rows_consumed_raw <= 16'd0;
        else if (evt_iss_yout_wrap)                   rows_consumed_raw <= 16'd0;
        else if (evt_iss_cs_wrap && !yout_is_last)    rows_consumed_raw <= rows_consumed_raw + {13'd0, cfg_stride};
        else                                          rows_consumed_raw <= rows_consumed_raw;
    end

    logic [15:0] pad_top_ext;
    assign pad_top_ext   = {12'd0, cfg_pad_top};
    assign rows_consumed = (rows_consumed_raw > pad_top_ext) ?
                           (rows_consumed_raw - pad_top_ext) : 16'd0;

    // =========================================================================
    // Padding 累加器 (Phase C-2)
    //   y_row_base:  strip start = -pad_top; 每 yout 推进 += stride;
    //                 cs 切换时回到 -pad_top。
    //   x_tile_base: strip start = -pad_left; 每 tile 推进 += tile_in_step;
    //                 yout 推进或 cs 切换时回到 -pad_left。
    //   数据路径，evt_start 初始化（§6）。Signed 17-bit 足以覆盖负的 pad 起点
    //   和正的 h_out * stride / w_in 终点。
    // =========================================================================
    logic signed [16:0] neg_pad_top_ext, neg_pad_left_ext;
    assign neg_pad_top_ext  = -$signed({13'd0, cfg_pad_top});
    assign neg_pad_left_ext = -$signed({13'd0, cfg_pad_left});

    always_ff @(posedge clk) begin
        if      (evt_start)                            y_row_base <= neg_pad_top_ext;
        else if (evt_iss_yout_wrap)     y_row_base <= neg_pad_top_ext;
        else if (evt_iss_cs_wrap && !yout_is_last)   y_row_base <= y_row_base + $signed({14'd0, cfg_stride});
        else                                           y_row_base <= y_row_base;
    end

    always_ff @(posedge clk) begin
        if      (evt_start)                            x_tile_base <= neg_pad_left_ext;
        else if (evt_iss_yout_wrap)                    x_tile_base <= neg_pad_left_ext;
        else if (evt_iss_cs_wrap && !yout_is_last)     x_tile_base <= neg_pad_left_ext;  // 新 yout 的 tile 0
        else if (evt_iss_tile_wrap && !cs_is_last)     x_tile_base <= neg_pad_left_ext;  // cs 切换回 tile 0
        else if (evt_iss_cins_wrap && !tile_is_last)   x_tile_base <= x_tile_base + $signed({1'b0, cfg_tile_pix_step});
        else                                           x_tile_base <= x_tile_base;
    end

    // =========================================================================
    // issue_advance 延迟 1 拍：控制路径（gate act_buf 写入），复位必须。
    //   reuse_en=0: issue_ok_std 延 1 拍；reuse_en=1: fill_issue 延 1 拍
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) issue_advance_d1 <= 1'b0;
        else        issue_advance_d1 <= issue_any;
    end

    // is_pad 延迟 1 拍，供 arrival 时选 0 / ifb_rdata
    always_ff @(posedge clk) begin
        if (issue_any) is_pad_d1 <= is_pad_now;
    end

    // =========================================================================
    // Ring buffer 指针与占用计数（reuse_en=0 原 ring；reuse_en=1 FILL 线性写）
    //   reuse_en=0: wr_idx/rd_idx/fifo_count 维持原 ring 行为
    //   reuse_en=1: wr_idx_fill 每 FILL 归 0，每 arrival +1；rd_idx_cons 组合算；fifo_count 不用
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (evt_start)                        wr_idx <= '0;
        else if (arrival && !cfg_arf_reuse_en)     wr_idx <= wr_idx + 1'b1;
        else                                       wr_idx <= wr_idx;
    end

    always_ff @(posedge clk) begin
        if      (evt_start)                        rd_idx <= '0;
        else if (act_fire && !cfg_arf_reuse_en)    rd_idx <= rd_idx + 1'b1;
        else                                       rd_idx <= rd_idx;
    end

    always_ff @(posedge clk) begin
        if      (!rst_n)                           fifo_count <= '0;
        else if (evt_start)                        fifo_count <= '0;
        else if (cfg_arf_reuse_en)                 fifo_count <= '0;  // reuse 模式不用
        else if ( arrival && !act_fire)            fifo_count <= fifo_count + 1'b1;
        else if (!arrival &&  act_fire)            fifo_count <= fifo_count - 1'b1;
        else                                       fifo_count <= fifo_count;
    end

    // reuse_en=1 FILL 线性写指针（每 ky 归 0）
    always_ff @(posedge clk) begin
        if      (fill_reset)  wr_idx_fill <= '0;
        else if (arrival)     wr_idx_fill <= wr_idx_fill + 1'b1;
        else                  wr_idx_fill <= wr_idx_fill;
    end

    // =========================================================================
    // act_buf 存储阵列（数据路径，无复位）
    //   reuse_en=0: 写 wr_idx（ring）
    //   reuse_en=1: 写 wr_idx_fill（linear 0..cur_fill_len-1）
    // =========================================================================
    logic [BUF_AW-1:0] wr_idx_sel;
    assign wr_idx_sel = cfg_arf_reuse_en ? wr_idx_fill[BUF_AW-1:0] : wr_idx;

    always_ff @(posedge clk) begin
        if (arrival) act_buf[wr_idx_sel] <= is_pad_d1 ? '0 : ifb_rdata;
        else         act_buf[wr_idx_sel] <= act_buf[wr_idx_sel];
    end

    // =========================================================================
    // 仿真性能 counters (E-3)
    //   arrival = ARF 写；act_fire = ARF 读；ifb_re = IFB SRAM 读；
    //   pad_skip = 因 pad 被 line_buffer 吞掉（未发 IFB 读）的 issue 拍数。
    // =========================================================================
    // synthesis translate_off
    int arf_write_cnt = 0;
    int arf_read_cnt  = 0;
    int ifb_read_cnt  = 0;
    int pad_skip_cnt  = 0;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            arf_write_cnt <= 0;
            arf_read_cnt  <= 0;
            ifb_read_cnt  <= 0;
            pad_skip_cnt  <= 0;
        end else begin
            if (arrival)           arf_write_cnt <= arf_write_cnt + 1;
            if (act_fire)          arf_read_cnt  <= arf_read_cnt  + 1;
            if (ifb_re)            ifb_read_cnt  <= ifb_read_cnt  + 1;
            if (issue_any && is_pad_now) pad_skip_cnt <= pad_skip_cnt + 1;
        end
    end
    // synthesis translate_on

endmodule
