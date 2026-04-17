`timescale 1ns/1ps

// =============================================================================
// line_buffer.sv  --  Activation Stream Feeder (IFB → ring act_buf → MAC)
//
// v3 优化：跨 round pipeline。消除 v2 中 per-round 的 2 拍 startup bubble。
//
// 核心思想：
//   - consume 侧不需要 counter（只顺序读 act_buf）；act_buf 作纯 ring buffer，
//     读/写指针模 ARF_DEPTH，永不随 round 边界重置。
//   - issue 侧是唯一的 counter 源。当 iss_pos 走完当前 round 最后一个位置，
//     同拍 advance 外层 counter (kx/ky/cins/tile/yout/cs) 和 ptr_kx_base。
//     下一拍立即 issue 新 round 的第 0 个 read，无需 bubble。
//   - fifo_count 追踪 act_buf 占用；issue 以 (fifo_count+in_flight < ARF_DEPTH)
//     背压，mac 吃得慢时 issue 自然停。
//   - 首 round 仍有 2 拍 startup (IFB 读 1 拍延迟 + 首个数据到达前 1 拍)；
//     这是 1-cycle IFB 延迟的物理下限，整层只发生 1 次。
//
// 6 层外层循环 (issue 侧自跑)：for cs / yout / tile / cins / ky / kx
//
// 时序 (cur_valid_w=32)：
//   T=0..T=31: issue round 0 的 32 个位置；T=31 拍 advance counter 到 round 1
//   T=32:      issue round 1 的 pos 0 (new ptr_kx_base); arrival/fire old round
//   T=33 起:   round 1 的 issues 继续；fire round 1 pos 0 @ T=34
//   per round (非首): 32 拍 = 32 fires, 0 bubble
//   整层: 2 (首 startup) + sum(cur_valid_w) = 2 + total_MAC_fires
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
    input  logic [ADDR_W-1:0]                    cfg_ifb_cin_step,
    input  logic [ADDR_W-1:0]                    cfg_ifb_row_step,
    input  logic [ADDR_W-1:0]                    cfg_tile_in_step,

    // ---- Streaming 配置（v2） ----
    // cfg_idma_streaming=1 时启用 ring wrap：每个 cin_slice 在 IFB 占
    // cfg_ifb_cin_step = strip_rows * W_IN 字；pointer 推进时 mod 这个值
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
    output logic [15:0]                          rows_consumed
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
    logic        ifb_re_d1;

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
    // 握手 + IFB 读地址
    // =========================================================================
    logic issue_ok;
    assign issue_ok = (state == S_RUN) && !issues_all_done &&
                      ({1'b0, fifo_count} + {6'b0, ifb_re_d1} < (CNT_W+1)'(ARF_DEPTH));

    logic [ADDR_W-1:0] ifb_rd_offset;
    always_comb begin
        case (cfg_stride)
            3'd1:    ifb_rd_offset = {{(ADDR_W-6){1'b0}}, iss_pos};
            3'd2:    ifb_rd_offset = {{(ADDR_W-7){1'b0}}, iss_pos, 1'b0};
            default: ifb_rd_offset = {{(ADDR_W-6){1'b0}}, iss_pos};
        endcase
    end

    assign ifb_re    = issue_ok;
    assign ifb_raddr = ptr_kx_base + ifb_rd_offset;

    assign act_valid = (fifo_count > 0);
    assign act_vec   = act_buf[rd_idx];

    logic act_fire, arrival;
    assign act_fire = act_valid & act_ready;
    assign arrival  = ifb_re_d1;

    // =========================================================================
    // 命名事件信号（comb）—— 把外层 6 级 wrap 链打平
    //   evt_start        : IDLE 下捕获 start，下拍进 S_RUN 并初始化 counter/ptr
    //   evt_iss_pos_wrap : issue 走完一 round (kx 进位触发)
    //   evt_iss_kx_wrap  : kx 进位到末尾 (ky 进位触发)
    //   evt_iss_ky_wrap  : ky 进位到末尾 (cins 进位触发)
    //   evt_iss_cins_wrap: cins 进位到末尾 (tile 进位触发)
    //   evt_iss_tile_wrap: tile 进位到末尾 (yout 进位触发)
    //   evt_iss_yout_wrap: yout 进位到末尾 (cs 进位触发)
    // =========================================================================
    logic evt_start;
    logic evt_iss_pos_wrap;
    logic evt_iss_kx_wrap;
    logic evt_iss_ky_wrap;
    logic evt_iss_cins_wrap;
    logic evt_iss_tile_wrap;
    logic evt_iss_yout_wrap;

    always_comb begin
        evt_start         = (state == S_IDLE) && start;
        evt_iss_pos_wrap  = issue_ok           && iss_pos_is_last;
        evt_iss_kx_wrap   = evt_iss_pos_wrap   && kx_is_last;
        evt_iss_ky_wrap   = evt_iss_kx_wrap    && ky_is_last;
        evt_iss_cins_wrap = evt_iss_ky_wrap    && cins_is_last;
        evt_iss_tile_wrap = evt_iss_cins_wrap  && tile_is_last;
        evt_iss_yout_wrap = evt_iss_tile_wrap  && yout_is_last;
    end

    // =========================================================================
    // 三段式 FSM
    // =========================================================================
    always_comb begin
        state_next = state;
        case (state)
            S_IDLE : if (start)                                              state_next = S_RUN;
            S_RUN  : if (issues_all_done && fifo_count == 0 && !ifb_re_d1)   state_next = S_DONE;
            S_DONE : ;
            default:                                                          state_next = S_IDLE;
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
        else if (issue_ok)           iss_pos <= iss_pos + 6'd1;
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

    always_ff @(posedge clk) begin
        if      (evt_start)          yout_cnt <= '0;
        else if (evt_iss_yout_wrap)  yout_cnt <= '0;
        else if (evt_iss_tile_wrap)  yout_cnt <= yout_cnt + 16'd1;
        else                         yout_cnt <= yout_cnt;
    end

    always_ff @(posedge clk) begin
        if      (evt_start)                         cs_cnt <= '0;
        else if (evt_iss_yout_wrap && !cs_is_last)  cs_cnt <= cs_cnt + 6'd1;
        else                                        cs_cnt <= cs_cnt;
    end

    // issues_all_done 是控制标志，参与 state_next 判定 → 复位（§6.1）。
    always_ff @(posedge clk) begin
        if      (!rst_n)                           issues_all_done <= 1'b0;
        else if (evt_start)                        issues_all_done <= 1'b0;
        else if (evt_iss_pos_wrap && all_is_last)  issues_all_done <= 1'b1;
        else                                       issues_all_done <= issues_all_done;
    end

    // =========================================================================
    // 5 层 running base（数据路径，无复位；evt_start 初始化）
    //   层级继承：外层进位时，内层 base 跟随外层新值重置；否则按自身步长推进。
    //
    //   Streaming ring wrap（cfg_idma_streaming=1）：
    //     cfg_ifb_cin_step 在 streaming 时 = strip_rows * W_IN = 每 slice 环大小。
    //     "intra-slice" 方向的增量（row_step / tile_in_step / w_in / 1）wrap；
    //     cin_slice 方向的增量（+= cfg_ifb_cin_step）不 wrap（跨 slice 加）。
    //   Batch 模式 (cfg_idma_streaming=0): wrap 函数 no-op，行为与 v1 完全一致。
    // =========================================================================

    // 组合 wrap 函数：若启用 streaming 且 val 跨过 cin_step 边界，减去一份 cin_step
    function automatic logic [ADDR_W-1:0] wrap_addr(input logic [ADDR_W-1:0] val);
        if (cfg_idma_streaming && val >= cfg_ifb_cin_step)
            wrap_addr = val - cfg_ifb_cin_step;
        else
            wrap_addr = val;
    endfunction

    logic [ADDR_W-1:0] w_yout_plus_rowstep;    // 外层到 ptr_*_base 的级联候选值
    logic [ADDR_W-1:0] w_tile_plus_tilestep;
    logic [ADDR_W-1:0] w_ky_plus_win;
    logic [ADDR_W-1:0] w_kx_plus_one;
    assign w_yout_plus_rowstep  = wrap_addr(ptr_yout_base + cfg_ifb_row_step);
    assign w_tile_plus_tilestep = wrap_addr(ptr_tile_base + cfg_tile_in_step);
    assign w_ky_plus_win        = wrap_addr(ptr_ky_base   + {{(ADDR_W-16){1'b0}}, cfg_w_in});
    assign w_kx_plus_one        = wrap_addr(ptr_kx_base   + {{(ADDR_W-1){1'b0}}, 1'b1});

    always_ff @(posedge clk) begin
        if      (evt_start)                           ptr_yout_base <= cfg_ifb_base;
        else if (evt_iss_yout_wrap && !cs_is_last)    ptr_yout_base <= cfg_ifb_base;
        else if (evt_iss_tile_wrap && !yout_is_last)  ptr_yout_base <= w_yout_plus_rowstep;
        else                                          ptr_yout_base <= ptr_yout_base;
    end

    always_ff @(posedge clk) begin
        if      (evt_start)                           ptr_tile_base <= cfg_ifb_base;
        else if (evt_iss_yout_wrap && !cs_is_last)    ptr_tile_base <= cfg_ifb_base;
        else if (evt_iss_tile_wrap && !yout_is_last)  ptr_tile_base <= w_yout_plus_rowstep;
        else if (evt_iss_cins_wrap && !tile_is_last)  ptr_tile_base <= w_tile_plus_tilestep;
        else                                          ptr_tile_base <= ptr_tile_base;
    end

    // cins 方向 += cfg_ifb_cin_step 是跨 slice 加，**不 wrap**
    always_ff @(posedge clk) begin
        if      (evt_start)                           ptr_cins_base <= cfg_ifb_base;
        else if (evt_iss_yout_wrap && !cs_is_last)    ptr_cins_base <= cfg_ifb_base;
        else if (evt_iss_tile_wrap && !yout_is_last)  ptr_cins_base <= w_yout_plus_rowstep;
        else if (evt_iss_cins_wrap && !tile_is_last)  ptr_cins_base <= w_tile_plus_tilestep;
        else if (evt_iss_ky_wrap   && !cins_is_last)  ptr_cins_base <= ptr_cins_base + cfg_ifb_cin_step;
        else                                          ptr_cins_base <= ptr_cins_base;
    end

    always_ff @(posedge clk) begin
        if      (evt_start)                           ptr_ky_base <= cfg_ifb_base;
        else if (evt_iss_yout_wrap && !cs_is_last)    ptr_ky_base <= cfg_ifb_base;
        else if (evt_iss_tile_wrap && !yout_is_last)  ptr_ky_base <= w_yout_plus_rowstep;
        else if (evt_iss_cins_wrap && !tile_is_last)  ptr_ky_base <= w_tile_plus_tilestep;
        else if (evt_iss_ky_wrap   && !cins_is_last)  ptr_ky_base <= ptr_cins_base + cfg_ifb_cin_step;
        else if (evt_iss_kx_wrap   && !ky_is_last)    ptr_ky_base <= w_ky_plus_win;
        else                                          ptr_ky_base <= ptr_ky_base;
    end

    always_ff @(posedge clk) begin
        if      (evt_start)                           ptr_kx_base <= cfg_ifb_base;
        else if (evt_iss_yout_wrap && !cs_is_last)    ptr_kx_base <= cfg_ifb_base;
        else if (evt_iss_tile_wrap && !yout_is_last)  ptr_kx_base <= w_yout_plus_rowstep;
        else if (evt_iss_cins_wrap && !tile_is_last)  ptr_kx_base <= w_tile_plus_tilestep;
        else if (evt_iss_ky_wrap   && !cins_is_last)  ptr_kx_base <= ptr_cins_base + cfg_ifb_cin_step;
        else if (evt_iss_kx_wrap   && !ky_is_last)    ptr_kx_base <= w_ky_plus_win;
        else if (evt_iss_pos_wrap  && !kx_is_last)    ptr_kx_base <= w_kx_plus_one;
        else                                          ptr_kx_base <= ptr_kx_base;
    end

    // =========================================================================
    // rows_consumed: 已彻底用完的输入行数，yout 每推进 1 就 +cfg_stride
    //   evt_iss_tile_wrap && !yout_is_last 表示当前拍之后 yout 要进位。
    //   cs 切换时归零（下一 cs 从头开始用 IFB 行）。
    //   数据路径，evt_start 初始化即可（§6）。
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (evt_start)                              rows_consumed <= 16'd0;
        else if (evt_iss_yout_wrap && !cs_is_last)       rows_consumed <= 16'd0;
        else if (evt_iss_tile_wrap && !yout_is_last)     rows_consumed <= rows_consumed + {13'd0, cfg_stride};
        else                                             rows_consumed <= rows_consumed;
    end

    // =========================================================================
    // IFB 读回延迟 1 拍：控制路径（gate act_buf 写入），复位必须。
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) ifb_re_d1 <= 1'b0;
        else        ifb_re_d1 <= issue_ok;
    end

    // =========================================================================
    // Ring buffer 指针与占用计数
    //   wr_idx / rd_idx：由 arrival / act_fire 驱动（gated by ifb_re_d1 和
    //   fifo_count，两者都有复位），按 §6 不加复位；evt_start 初始化。
    //   fifo_count：驱动 act_valid → 必须复位。
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (evt_start)  wr_idx <= '0;
        else if (arrival)    wr_idx <= wr_idx + 1'b1;
        else                 wr_idx <= wr_idx;
    end

    always_ff @(posedge clk) begin
        if      (evt_start)  rd_idx <= '0;
        else if (act_fire)   rd_idx <= rd_idx + 1'b1;
        else                 rd_idx <= rd_idx;
    end

    always_ff @(posedge clk) begin
        if      (!rst_n)                 fifo_count <= '0;
        else if (evt_start)              fifo_count <= '0;
        else if ( arrival && !act_fire)  fifo_count <= fifo_count + 1'b1;
        else if (!arrival &&  act_fire)  fifo_count <= fifo_count - 1'b1;
        else                             fifo_count <= fifo_count;
    end

    // =========================================================================
    // act_buf 存储阵列（数据路径，无复位）
    //   arrival=ifb_re_d1 门控写；下游 act_valid=fifo_count>0 遮蔽未初始化值。
    // =========================================================================
    always_ff @(posedge clk) begin
        if (arrival) act_buf[wr_idx] <= ifb_rdata;
        else         act_buf[wr_idx] <= act_buf[wr_idx];
    end

endmodule
