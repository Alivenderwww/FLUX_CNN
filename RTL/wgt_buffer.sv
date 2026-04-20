`timescale 1ns/1ps

// =============================================================================
// wgt_buffer.sv  --  Weight Stream Feeder (WB → WRF → MAC)
//
// 两条路径由 cfg_wrf_packed 选择：
//
// --- Packed (K²·cin_slices ≤ 32) ---
//     S_IDLE → S_LOAD (一次 LOAD total_wrf 个) → S_COMPUTE → S_DONE
//     COMPUTE 6 层循环 (yout, tile, cins, ky, kx, x)，wrf_raddr 走 running base
//     wrf_base_kx = (cins·kk + ky·K + kx) 累加。
//
// --- Chunked (K²·cin_slices > 32) ---  pipelined LOAD/COMPUTE overlap
//     两套独立指针并行：
//       * compute ptr (x_cnt, pos_cnt, round_cnt, cins/tile/yout/cs_cnt)
//         drive wrf_raddr = pos_cnt 和 wgt_valid；一个 pos 持续 cur_valid_w 拍。
//       * load ptr    (l_pos, l_slots_done, l_round, l_cins, l_tile, l_yout, l_cs)
//         drive wb_raddr / wrf_we / wrf_waddr；target = compute 的下一 round。
//         l_pos 在 [0, l_cur_round_len) 内循环走；l_slots_done 跟踪写了多少 slot。
//         起点 l_start_pos_next: 若下一 target round 比刚装完的长，从
//         c_cur_round_len 开始 wrap（先写 compute 本 round 不读的高 slot，避免
//         冲突）；否则从 0 顺序写。
//
//     状态机：S_IDLE → S_LOAD (冷启动填 round 0) → S_COMPUTE (compute+load 并行) → S_DONE
//
//     Hazard 规则（三选一）：
//       (a) l_pos >= c_cur_round_len : compute 本 round 不读 slot l_pos，自由写。
//       (b) pos_cnt > l_pos          : compute 已离开 slot l_pos。
//       (c) pos_cnt == l_pos && x_is_last && wgt_fire：本拍 compute 读完 slot
//           l_pos 的最后一 fire，下一 posedge cpos 推进，WRF[l_pos] 在 posedge
//           t+2 写入时 compute 已在 l_pos+1。
//
//     同步状态 load_caught_up ∈ {0,1}。rounds_ahead = load_round - compute_round ∈ {1,2}。
//       load_caught_up=0 ⇔ rounds_ahead=1 (load 正在装 compute 下一 round)
//       load_caught_up=1 ⇔ rounds_ahead=2 (load 领先 2，等 compute 消费)
// =============================================================================

module wgt_buffer #(
    parameter int NUM_COL         = 16,
    parameter int NUM_PE          = 16,
    parameter int DATA_WIDTH      = 8,
    parameter int PSUM_WIDTH      = 32,
    parameter int WRF_DEPTH       = 32,
    parameter int MAX_COUT_SLICES = 16,    // BRF 深度，支持到 cout=256
    parameter int SRAM_DEPTH      = 8192,
    parameter int ADDR_W          = 20
)(
    input  logic                                 clk,
    input  logic                                 rst_n,
    input  logic                                 start,
    output logic                                 done,

    // ---- cfg ----
    input  logic [15:0]                          cfg_h_out,
    input  logic [5:0]                           cfg_cin_slices,
    input  logic [5:0]                           cfg_cout_slices,
    input  logic [5:0]                           cfg_tile_w,
    input  logic [7:0]                           cfg_num_tiles,
    input  logic [5:0]                           cfg_last_valid_w,
    input  logic [3:0]                           cfg_k,
    input  logic [9:0]                           cfg_kk,
    input  logic [9:0]                           cfg_total_wrf,
    input  logic                                 cfg_wrf_packed,
    input  logic [2:0]                           cfg_rounds_per_cins,
    input  logic [5:0]                           cfg_round_len_last,
    input  logic [ADDR_W-1:0]                    cfg_wb_base,
    input  logic [ADDR_W-1:0]                    cfg_wb_cout_step,

    // ---- WB 读 ----
    output logic                                 wb_re,
    output logic [$clog2(SRAM_DEPTH)-1:0]        wb_raddr,
    input  logic [NUM_COL*NUM_PE*DATA_WIDTH-1:0] wb_rdata,

    // ---- mac_array WRF 写 ----
    output logic [NUM_COL*NUM_PE-1:0]            wrf_we,
    output logic [$clog2(WRF_DEPTH)-1:0]         wrf_waddr,
    output logic [NUM_COL*NUM_PE*DATA_WIDTH-1:0] wrf_wdata,

    // ---- mac_array COMPUTE ----
    output logic                                 wgt_valid,
    output logic [$clog2(WRF_DEPTH)-1:0]         wrf_raddr,
    input  logic                                 wgt_ready,

    // ---- F-1b: bias 输出给 mac_array (当前 cs 的 16 × int32) ----
    output logic signed [NUM_COL*PSUM_WIDTH-1:0] bias_vec
);

    localparam int AW  = $clog2(SRAM_DEPTH);
    localparam int WAW = $clog2(WRF_DEPTH);

    // =========================================================================
    // 状态机
    // =========================================================================
    typedef enum logic [2:0] {
        S_IDLE      = 3'd0,
        S_BIAS_LOAD = 3'd1,   // F-1b: 从 WB[0..cout_slices-1] 拉 bias 到 BRF
        S_LOAD      = 3'd2,   // packed: 常规 LOAD；chunked: cold_load (仅整层首次)
        S_COMPUTE   = 3'd3,   // packed: 常规 COMPUTE；chunked: compute+load 并行
        S_DONE      = 3'd4
    } state_t;
    state_t state, state_next;

    // =========================================================================
    // 寄存器声明（按功能分组）
    // =========================================================================
    // --- Compute 侧共享 counter（packed/chunked 都用）---
    logic [5:0]  x_cnt;
    logic [5:0]  cins_cnt;
    logic [7:0]  tile_cnt;
    logic [15:0] yout_cnt;
    logic [5:0]  cs_cnt;

    // --- Packed 专用 ---
    logic [3:0]  kx_cnt;
    logic [3:0]  ky_cnt;
    logic [5:0]  wrf_base_cins;
    logic [5:0]  wrf_base_ky;
    logic [5:0]  wrf_base_kx;

    // --- Chunked compute 专用 ---
    logic [5:0]  pos_cnt;
    logic [2:0]  round_cnt;

    // --- Chunked load 侧 ptr ---
    logic [5:0]  l_pos;
    logic [5:0]  l_slots_done;
    logic [2:0]  l_round;
    logic [5:0]  l_cins;
    logic [7:0]  l_tile;
    logic [15:0] l_yout;
    logic [5:0]  l_cs;
    logic        l_exhausted;
    logic        load_caught_up;

    // --- Packed/cold LOAD 阶段计数 + 1 拍延迟副本 ---
    logic [9:0]  wb_rd_cnt;
    logic [9:0]  wrf_wr_cnt;
    logic        wb_re_d1;       // for S_LOAD (packed & cold)
    logic        l_wb_re_d1;     // for chunked run 路径
    logic [WAW-1:0] l_wrf_waddr_d1;

    // --- Running base ---
    logic [ADDR_W-1:0] cur_wb_base_cs;   // compute 侧 cs base（两路径都用）
    logic [ADDR_W-1:0] cur_wb_rd_base;   // packed cold-load 的 WB 起点
    logic [ADDR_W-1:0] l_wb_base;        // chunked load 的当前 round 起点
    logic [ADDR_W-1:0] l_wb_base_cs;     // chunked load 侧 cs base

    // --- F-1b: BRF + bias load 阶段 ---
    //   WB SRAM 前缀布局：[0..cout_slices-1] 放 bias (每 word 低 NUM_COL×32bit 放 16 个 int32)
    //   S_BIAS_LOAD 扫 cout_slices 次 wb_re，1 拍延后把 wb_rdata 低 512bit 写入 BRF[bias_wr_idx_d1]
    //   COMPUTE 阶段 bias_vec 输出 = BRF[cs_cnt] 组合读（NUM_COL 个 int32 打包）
    localparam int BIAS_IDX_W = $clog2(MAX_COUT_SLICES);
    logic [BIAS_IDX_W-1:0]           bias_load_cnt;
    logic [BIAS_IDX_W-1:0]           bias_wr_idx_d1;
    logic                            bias_re_gate;
    logic                            bias_re_d1;
    logic                            bias_load_done;
    logic signed [PSUM_WIDTH-1:0]    bias_rf [0:MAX_COUT_SLICES-1][0:NUM_COL-1];

    // =========================================================================
    // 派生量
    // =========================================================================
    logic [5:0] cur_valid_w;
    assign cur_valid_w = (tile_cnt == cfg_num_tiles - 8'd1) ? cfg_last_valid_w : cfg_tile_w;

    logic [5:0] c_cur_round_len;    // compute 侧当前 round 长度
    logic [5:0] l_cur_round_len;    // load 侧当前 round 长度
    assign c_cur_round_len = (round_cnt == cfg_rounds_per_cins - 3'd1) ? cfg_round_len_last : 6'd32;
    assign l_cur_round_len = (l_round   == cfg_rounds_per_cins - 3'd1) ? cfg_round_len_last : 6'd32;

    logic [9:0] cur_load_len;       // packed 一次 LOAD total_wrf；cold_load 灌 round 0
    assign cur_load_len = cfg_wrf_packed ? cfg_total_wrf : {4'd0, c_cur_round_len};

    // =========================================================================
    // 边界 flag
    // =========================================================================
    // 共享
    logic x_is_last, cins_is_last, tile_is_last, yout_is_last, cs_is_last;
    assign x_is_last    = (x_cnt    == cur_valid_w       - 6'd1);
    assign cins_is_last = (cins_cnt == cfg_cin_slices    - 6'd1);
    assign tile_is_last = (tile_cnt == cfg_num_tiles     - 8'd1);
    assign yout_is_last = (yout_cnt == cfg_h_out         - 16'd1);
    assign cs_is_last   = (cs_cnt   == cfg_cout_slices   - 6'd1);

    // Packed-only
    logic kx_is_last, ky_is_last;
    assign kx_is_last = (kx_cnt == cfg_k - 4'd1);
    assign ky_is_last = (ky_cnt == cfg_k - 4'd1);

    // Chunked compute-only
    logic pos_is_last, round_is_last;
    assign pos_is_last   = (pos_cnt   == c_cur_round_len     - 6'd1);
    assign round_is_last = (round_cnt == cfg_rounds_per_cins - 3'd1);

    // Load 侧
    logic l_round_done, l_round_is_last, l_cins_is_last, l_tile_is_last, l_yout_is_last, l_cs_is_last;
    assign l_round_done    = (l_slots_done == l_cur_round_len     - 6'd1);
    assign l_round_is_last = (l_round      == cfg_rounds_per_cins - 3'd1);
    assign l_cins_is_last  = (l_cins       == cfg_cin_slices      - 6'd1);
    assign l_tile_is_last  = (l_tile       == cfg_num_tiles       - 8'd1);
    assign l_yout_is_last  = (l_yout       == cfg_h_out           - 16'd1);
    assign l_cs_is_last    = (l_cs         == cfg_cout_slices     - 6'd1);

    logic [5:0] l_pos_next;
    assign l_pos_next = (l_pos == l_cur_round_len - 6'd1) ? 6'd0 : (l_pos + 6'd1);

    // 下一 load round 的 round_len（基于 l_round 跨 round 后新值）
    logic [2:0] l_round_next_id;
    logic [5:0] l_cur_round_len_next;
    assign l_round_next_id      = l_round_is_last ? 3'd0 : (l_round + 3'd1);
    assign l_cur_round_len_next = (l_round_next_id == cfg_rounds_per_cins - 3'd1)
                                ? cfg_round_len_last : 6'd32;

    // 下一 target round 起点：若下一 round_len > 刚装完的，从 l_cur_round_len 起步
    // （先写 compute 本 round 不读的高 slot），否则从 0 顺序写。
    logic [5:0] l_start_pos_next;
    assign l_start_pos_next = (l_cur_round_len_next > l_cur_round_len)
                            ? l_cur_round_len : 6'd0;

    // 当前 (yout, cs) 的 COMPUTE 打完最后一拍（方式 1：cs 在 yout 内）
    // 去掉 yout_is_last，每个 (yout, cs) 扫完就触发
    logic cs_compute_last;
    assign cs_compute_last = cfg_wrf_packed
        ? (x_is_last && kx_is_last  && ky_is_last    && cins_is_last && tile_is_last)
        : (x_is_last && pos_is_last && round_is_last && cins_is_last && tile_is_last);

    // =========================================================================
    // 握手 / hazard / gate
    // =========================================================================
    logic wgt_fire;
    assign wgt_fire = (state == S_COMPUTE) && wgt_valid && wgt_ready;

    // Packed/cold LOAD 阶段的 WB 读 gate
    logic sload_wb_re_gate;
    assign sload_wb_re_gate = (state == S_LOAD) && (wb_rd_cnt < cur_load_len);

    // S_BIAS_LOAD 阶段的 WB 读 gate：bias_load_cnt 扫 0..cfg_cout_slices-1
    assign bias_re_gate   = (state == S_BIAS_LOAD) && ({1'b0, bias_load_cnt} < cfg_cout_slices);
    assign bias_load_done = (state == S_BIAS_LOAD) && ({1'b0, bias_load_cnt} == cfg_cout_slices);

    // Chunked run 阶段 hazard 判定
    logic chunked_hazard_ok;
    assign chunked_hazard_ok = (l_pos >= c_cur_round_len)
                            || (pos_cnt >  l_pos)
                            || (pos_cnt == l_pos && x_is_last && wgt_fire);

    logic chunked_load_active;
    assign chunked_load_active = !cfg_wrf_packed && (state == S_COMPUTE)
                                 && !l_exhausted && !load_caught_up
                                 && chunked_hazard_ok;

    logic sload_done;
    assign sload_done = (wrf_wr_cnt == cur_load_len);

    // =========================================================================
    // 命名事件（comb）
    //   evt_start             : IDLE 捕获 start
    //   evt_cold_load_done    : S_LOAD → S_COMPUTE 切换当拍
    //   evt_pk_*              : packed compute 侧各级 wrap 链
    //   evt_ck_*              : chunked compute 侧各级 wrap 链
    //   evt_ld / evt_ld_*     : chunked load 侧 wb_re 和跨 round 链
    // =========================================================================
    logic evt_start;
    logic evt_cold_load_done;

    logic evt_pk_fire, evt_pk_x_wrap, evt_pk_kx_wrap, evt_pk_ky_wrap;
    logic evt_pk_cins_wrap, evt_pk_tile_wrap, evt_pk_cs_wrap, evt_pk_yout_wrap;

    logic evt_ck_fire, evt_ck_x_wrap, evt_ck_pos_wrap, evt_ck_round_wrap;
    logic evt_ck_cins_wrap, evt_ck_tile_wrap, evt_ck_cs_wrap, evt_ck_yout_wrap;

    logic evt_ld, evt_ld_round_end;
    logic evt_ld_round_wrap, evt_ld_cins_wrap, evt_ld_tile_wrap, evt_ld_yout_wrap;

    always_comb begin
        evt_start          = (state == S_IDLE) && start;
        evt_cold_load_done = (state == S_LOAD) && sload_done;

        evt_pk_fire       =  cfg_wrf_packed && (state == S_COMPUTE) && wgt_fire;
        evt_pk_x_wrap     = evt_pk_fire    && x_is_last;
        evt_pk_kx_wrap    = evt_pk_x_wrap  && kx_is_last;
        evt_pk_ky_wrap    = evt_pk_kx_wrap && ky_is_last;
        evt_pk_cins_wrap  = evt_pk_ky_wrap && cins_is_last;
        evt_pk_tile_wrap  = evt_pk_cins_wrap && tile_is_last;
        evt_pk_cs_wrap    = evt_pk_tile_wrap && cs_is_last;
        evt_pk_yout_wrap  = evt_pk_cs_wrap   && yout_is_last;

        evt_ck_fire       = !cfg_wrf_packed && (state == S_COMPUTE) && wgt_fire;
        evt_ck_x_wrap     = evt_ck_fire    && x_is_last;
        evt_ck_pos_wrap   = evt_ck_x_wrap  && pos_is_last;
        evt_ck_round_wrap = evt_ck_pos_wrap && round_is_last;
        evt_ck_cins_wrap  = evt_ck_round_wrap && cins_is_last;
        evt_ck_tile_wrap  = evt_ck_cins_wrap && tile_is_last;
        evt_ck_cs_wrap    = evt_ck_tile_wrap && cs_is_last;
        evt_ck_yout_wrap  = evt_ck_cs_wrap   && yout_is_last;

        evt_ld             = chunked_load_active;
        evt_ld_round_end   = evt_ld              && l_round_done;
        evt_ld_round_wrap  = evt_ld_round_end    && l_round_is_last;
        evt_ld_cins_wrap   = evt_ld_round_wrap   && l_cins_is_last;
        evt_ld_tile_wrap   = evt_ld_cins_wrap    && l_tile_is_last;
        evt_ld_yout_wrap   = evt_ld_tile_wrap    && l_yout_is_last;
    end

    // =========================================================================
    // 端口输出 (comb)
    // =========================================================================
    logic [ADDR_W-1:0] wb_raddr_sload, wb_raddr_srun;
    assign wb_raddr_sload = cur_wb_rd_base + {{(ADDR_W-10){1'b0}}, wb_rd_cnt};
    assign wb_raddr_srun  = l_wb_base      + {{(ADDR_W-6) {1'b0}}, l_pos};

    assign wb_re    = sload_wb_re_gate | chunked_load_active | bias_re_gate;
    assign wb_raddr = bias_re_gate        ? {{(AW-BIAS_IDX_W){1'b0}}, bias_load_cnt}
                    : chunked_load_active ? wb_raddr_srun[AW-1:0]
                                          : wb_raddr_sload[AW-1:0];

    assign wrf_we    = {(NUM_COL*NUM_PE){wb_re_d1 | l_wb_re_d1}};
    assign wrf_waddr = l_wb_re_d1 ? l_wrf_waddr_d1 : wrf_wr_cnt[WAW-1:0];
    assign wrf_wdata = wb_rdata;

    assign wgt_valid = (state == S_COMPUTE);
    assign wrf_raddr = cfg_wrf_packed ? wrf_base_kx[WAW-1:0] : pos_cnt[WAW-1:0];

    assign done = (state == S_DONE);

    // =========================================================================
    // 三段式 FSM
    // =========================================================================
    always_comb begin
        state_next = state;
        case (state)
            S_IDLE      : if (start)          state_next = S_BIAS_LOAD;
            S_BIAS_LOAD : if (bias_load_done) state_next = S_LOAD;
            S_LOAD      : if (sload_done)     state_next = S_COMPUTE;
            S_COMPUTE   : if (wgt_fire && cs_compute_last) begin
                // 方式 1：每 (yout, cs) 扫完；packed 都要重 LOAD 下一 (yout, cs)
                // chunked 由内部 load ptr 并行覆盖 WRF
                state_next = (yout_is_last && cs_is_last) ? S_DONE
                           : (cfg_wrf_packed ? S_LOAD : S_COMPUTE);
            end
            S_DONE      : ;
            default     :                     state_next = S_IDLE;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= state_next;
    end

    // =========================================================================
    // F-1b: BIAS_LOAD counter + BRF 写
    //   reset on evt_start，每 bias_re_gate +1；读完 cout_slices 个 → bias_load_done=1
    //   wb_rdata 低 NUM_COL×PSUM_WIDTH 位是 16 个 int32 bias（上位 pad 0），1 拍延后写 BRF
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (!rst_n)        bias_load_cnt <= '0;
        else if (evt_start)     bias_load_cnt <= '0;
        else if (bias_re_gate)  bias_load_cnt <= bias_load_cnt + 1'd1;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            bias_re_d1     <= 1'b0;
            bias_wr_idx_d1 <= '0;
        end else begin
            bias_re_d1     <= bias_re_gate;
            bias_wr_idx_d1 <= bias_load_cnt;
        end
    end

    // BRF 数据：数据路径，reset 清 0（避免 bias_en=0 场景下残留 X，给 mac_array acc_seed 注入 0）
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int s = 0; s < MAX_COUT_SLICES; s++)
                for (int c = 0; c < NUM_COL; c++)
                    bias_rf[s][c] <= '0;
        end else if (bias_re_d1) begin
            for (int c = 0; c < NUM_COL; c++) begin
                bias_rf[bias_wr_idx_d1][c] <=
                    $signed(wb_rdata[c*PSUM_WIDTH +: PSUM_WIDTH]);
            end
        end
    end

    // bias_vec 组合读：当前 cs_cnt 对应的 16 个 int32
    always_comb begin
        for (int c = 0; c < NUM_COL; c++) begin
            bias_vec[c*PSUM_WIDTH +: PSUM_WIDTH] = bias_rf[cs_cnt[BIAS_IDX_W-1:0]][c];
        end
    end

    // =========================================================================
    // Packed/cold LOAD 计数
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)              wb_rd_cnt <= '0;
        else if (evt_start)           wb_rd_cnt <= '0;
        else if (evt_cold_load_done)  wb_rd_cnt <= '0;
        else if (sload_wb_re_gate)    wb_rd_cnt <= wb_rd_cnt + 10'd1;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)              wrf_wr_cnt <= '0;
        else if (evt_start)           wrf_wr_cnt <= '0;
        else if (evt_cold_load_done)  wrf_wr_cnt <= '0;
        else if (wb_re_d1)            wrf_wr_cnt <= wrf_wr_cnt + 10'd1;
    end

    // wb_re_d1 / l_wb_re_d1 / l_wrf_waddr_d1：wb_re 到 WRF 写之间的 1 拍延迟
    // 副本。三者都只是"上拍 gate" 的影子，强相关，合并维护。
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wb_re_d1       <= 1'b0;
            l_wb_re_d1     <= 1'b0;
            l_wrf_waddr_d1 <= '0;
        end else begin
            wb_re_d1       <= sload_wb_re_gate;
            l_wb_re_d1     <= chunked_load_active;
            l_wrf_waddr_d1 <= l_pos[WAW-1:0];
        end
    end

    // =========================================================================
    // Compute 侧共享 counter（packed/chunked 合并：一次只有一种事件活跃）
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                               x_cnt <= '0;
        else if (evt_start || evt_cold_load_done)      x_cnt <= '0;
        else if (evt_pk_x_wrap || evt_ck_x_wrap)       x_cnt <= '0;
        else if (evt_pk_fire   || evt_ck_fire)         x_cnt <= x_cnt + 6'd1;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                                 cins_cnt <= '0;
        else if (evt_start || evt_cold_load_done)        cins_cnt <= '0;
        else if (evt_pk_cins_wrap || evt_ck_cins_wrap)   cins_cnt <= '0;
        else if (evt_pk_ky_wrap)                         cins_cnt <= cins_cnt + 6'd1;
        else if (evt_ck_round_wrap)                      cins_cnt <= cins_cnt + 6'd1;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                                 tile_cnt <= '0;
        else if (evt_start || evt_cold_load_done)        tile_cnt <= '0;
        else if (evt_pk_tile_wrap || evt_ck_tile_wrap)   tile_cnt <= '0;
        else if (evt_pk_cins_wrap || evt_ck_cins_wrap)   tile_cnt <= tile_cnt + 8'd1;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                                 yout_cnt <= '0;
        else if (evt_start || evt_cold_load_done)        yout_cnt <= '0;
        else if (evt_pk_yout_wrap || evt_ck_yout_wrap)   yout_cnt <= '0;
        else if (evt_pk_cs_wrap   || evt_ck_cs_wrap)     yout_cnt <= yout_cnt + 16'd1;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                                                  cs_cnt <= '0;
        else if (evt_start)                                               cs_cnt <= '0;
        else if (evt_pk_cs_wrap   || evt_ck_cs_wrap)                      cs_cnt <= '0;
        else if (evt_pk_tile_wrap || evt_ck_tile_wrap)                    cs_cnt <= cs_cnt + 6'd1;
    end

    // =========================================================================
    // Packed-only counter
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)              kx_cnt <= '0;
        else if (evt_cold_load_done)  kx_cnt <= '0;
        else if (evt_pk_kx_wrap)      kx_cnt <= '0;
        else if (evt_pk_x_wrap)       kx_cnt <= kx_cnt + 4'd1;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)              ky_cnt <= '0;
        else if (evt_cold_load_done)  ky_cnt <= '0;
        else if (evt_pk_ky_wrap)      ky_cnt <= '0;
        else if (evt_pk_kx_wrap)      ky_cnt <= ky_cnt + 4'd1;
    end

    // Packed wrf_base_* running base（cins/ky/kx 三级）
    logic [5:0] kk_lo6, k_lo6;
    assign kk_lo6 = cfg_kk[5:0];
    assign k_lo6  = {2'd0, cfg_k};

    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                                    wrf_base_cins <= '0;
        else if (evt_cold_load_done)                        wrf_base_cins <= '0;
        else if (evt_pk_cins_wrap || evt_pk_tile_wrap)                          wrf_base_cins <= '0;
        else if (evt_pk_ky_wrap)                            wrf_base_cins <= wrf_base_cins + kk_lo6;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                                    wrf_base_ky <= '0;
        else if (evt_cold_load_done)                        wrf_base_ky <= '0;
        else if (evt_pk_cins_wrap || evt_pk_tile_wrap)                          wrf_base_ky <= '0;
        else if (evt_pk_ky_wrap)                            wrf_base_ky <= wrf_base_cins + kk_lo6;
        else if (evt_pk_kx_wrap)                            wrf_base_ky <= wrf_base_ky + k_lo6;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                                    wrf_base_kx <= '0;
        else if (evt_cold_load_done)                        wrf_base_kx <= '0;
        else if (evt_pk_cins_wrap || evt_pk_tile_wrap)                          wrf_base_kx <= '0;
        else if (evt_pk_ky_wrap)                            wrf_base_kx <= wrf_base_cins + kk_lo6;
        else if (evt_pk_kx_wrap)                            wrf_base_kx <= wrf_base_ky + k_lo6;
        else if (evt_pk_x_wrap)                             wrf_base_kx <= wrf_base_kx + 6'd1;
    end

    // =========================================================================
    // Chunked compute-only counter
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)              pos_cnt <= '0;
        else if (evt_cold_load_done)  pos_cnt <= '0;
        else if (evt_ck_pos_wrap)     pos_cnt <= '0;
        else if (evt_ck_x_wrap)       pos_cnt <= pos_cnt + 6'd1;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)              round_cnt <= '0;
        else if (evt_cold_load_done)  round_cnt <= '0;
        else if (evt_ck_round_wrap)   round_cnt <= '0;
        else if (evt_ck_pos_wrap)     round_cnt <= round_cnt + 3'd1;
    end

    // =========================================================================
    // Compute 侧 cs-base（两路径共享）
    // =========================================================================
    // 方式 1：cs 在 yout 内 → cs wrap 时 cur_wb_base_cs 归零，tile wrap 时跨到下一 cs
    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                                  cur_wb_base_cs <= '0;
        else if (evt_start)                               cur_wb_base_cs <= cfg_wb_base;
        else if (evt_pk_cs_wrap   || evt_ck_cs_wrap)      cur_wb_base_cs <= cfg_wb_base;
        else if (evt_pk_tile_wrap || evt_ck_tile_wrap)    cur_wb_base_cs <= cur_wb_base_cs + cfg_wb_cout_step;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                                  cur_wb_rd_base <= '0;
        else if (evt_start)                               cur_wb_rd_base <= cfg_wb_base;
        else if (evt_pk_cs_wrap   || evt_ck_cs_wrap)      cur_wb_rd_base <= cfg_wb_base;
        else if (evt_pk_tile_wrap || evt_ck_tile_wrap)    cur_wb_rd_base <= cur_wb_base_cs + cfg_wb_cout_step;
    end

    // =========================================================================
    // Chunked load 侧：cold_load 完成时把 load ptr 推到"下一 round"；
    //                 之后每拍跨 round/cins/tile/yout/cs 时同步更新。
    //
    // 以下常用组合信号仅用于 cold_load 触发一次性推进：
    // =========================================================================
    logic cld_multi_round;      // 多 round (rounds_per_cins > 1)
    logic cld_multi_cins;       // 单 round、多 cins
    logic cld_multi_tile;       // 单 round、单 cins、多 tile
    logic cld_multi_yout;       // 单 round、单 cins、单 tile、多 yout
    logic cld_multi_cs;         // 单 round、单 cins、单 tile、单 yout、多 cs

    always_comb begin
        cld_multi_round = (cfg_rounds_per_cins != 3'd1);
        cld_multi_cins  = !cld_multi_round  && (cfg_cin_slices != 6'd1);
        cld_multi_tile  = !cld_multi_round  && !cld_multi_cins && (cfg_num_tiles != 8'd1);
        cld_multi_yout  = !cld_multi_round  && !cld_multi_cins && !cld_multi_tile && (cfg_h_out != 16'd1);
        cld_multi_cs    = !cld_multi_round  && !cld_multi_cins && !cld_multi_tile && !cld_multi_yout && (cfg_cout_slices != 6'd1);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                                  l_pos <= '0;
        else if (evt_start)                               l_pos <= '0;
        else if (evt_cold_load_done)                      l_pos <= '0;
        else if (evt_ld_round_end)                        l_pos <= l_start_pos_next;
        else if (evt_ld)                                  l_pos <= l_pos_next;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                                  l_slots_done <= '0;
        else if (evt_start || evt_cold_load_done)         l_slots_done <= '0;
        else if (evt_ld_round_end)                        l_slots_done <= '0;
        else if (evt_ld)                                  l_slots_done <= l_slots_done + 6'd1;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                                  l_round <= '0;
        else if (evt_start)                               l_round <= '0;
        else if (evt_cold_load_done && cld_multi_round)   l_round <= 3'd1;
        else if (evt_ld_round_wrap)                       l_round <= '0;
        else if (evt_ld_round_end)                        l_round <= l_round + 3'd1;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                                  l_cins <= '0;
        else if (evt_start)                               l_cins <= '0;
        else if (evt_cold_load_done && cld_multi_cins)    l_cins <= 6'd1;
        else if (evt_ld_cins_wrap)                        l_cins <= '0;
        else if (evt_ld_round_wrap)                       l_cins <= l_cins + 6'd1;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                                  l_tile <= '0;
        else if (evt_start)                               l_tile <= '0;
        else if (evt_cold_load_done && cld_multi_tile)    l_tile <= 8'd1;
        else if (evt_ld_tile_wrap)                        l_tile <= '0;
        else if (evt_ld_cins_wrap)                        l_tile <= l_tile + 8'd1;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                                  l_yout <= '0;
        else if (evt_start)                               l_yout <= '0;
        else if (evt_cold_load_done && cld_multi_yout)    l_yout <= 16'd1;
        else if (evt_ld_yout_wrap)                        l_yout <= '0;
        else if (evt_ld_tile_wrap)                        l_yout <= l_yout + 16'd1;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                                  l_cs <= '0;
        else if (evt_start)                               l_cs <= '0;
        else if (evt_cold_load_done && cld_multi_cs)      l_cs <= 6'd1;
        else if (evt_ld_yout_wrap && !l_cs_is_last)       l_cs <= l_cs + 6'd1;
    end

    // l_exhausted: 所有 load round 都完成（最后 cs 的 yout_wrap 触发）
    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                                  l_exhausted <= 1'b0;
        else if (evt_start)                               l_exhausted <= 1'b0;
        else if (evt_cold_load_done && !cld_multi_round && !cld_multi_cins
                 && !cld_multi_tile && !cld_multi_yout && !cld_multi_cs)
                                                          l_exhausted <= 1'b1;
        else if (evt_ld_yout_wrap && l_cs_is_last)        l_exhausted <= 1'b1;
    end

    // load_caught_up: rounds_ahead ∈ {1,2}；同拍 compute/load 事件抵消时保持
    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                                                    load_caught_up <= 1'b0;
        else if (evt_start || evt_cold_load_done)                           load_caught_up <= 1'b0;
        else if (evt_ck_pos_wrap && evt_ld_round_end)                       load_caught_up <= load_caught_up;
        else if (evt_ck_pos_wrap)                                           load_caught_up <= 1'b0;
        else if (evt_ld_round_end)                                          load_caught_up <= 1'b1;
    end

    // =========================================================================
    // Chunked load 侧的 WB base（cs 内推进 + (yout,tile) 边界回到 cs_base + cs 边界 +wb_cout_step）
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                                                 l_wb_base_cs <= '0;
        else if (evt_start)                                              l_wb_base_cs <= cfg_wb_base;
        else if (evt_cold_load_done && cld_multi_cs)                     l_wb_base_cs <= cfg_wb_base + cfg_wb_cout_step;
        else if (evt_ld_yout_wrap && !l_cs_is_last)                      l_wb_base_cs <= l_wb_base_cs + cfg_wb_cout_step;
    end

    // 事件优先级（else-if 自上而下）：
    //   yout_wrap && !cs_last : 进入新 cs   → l_wb_base_cs + wb_cout_step
    //   yout_wrap             : cs 也是末尾（l_exhausted 同拍）→ 任意值即可
    //   tile_wrap             : 新 yout，同 cs → 回 l_wb_base_cs
    //   cins_wrap             : 新 tile，同 yout → 回 l_wb_base_cs
    //   round_end             : 新 round 或新 cins（同 tile） → += l_cur_round_len
    always_ff @(posedge clk or negedge rst_n) begin
        if      (!rst_n)                                                 l_wb_base <= '0;
        else if (evt_start)                                              l_wb_base <= cfg_wb_base;
        else if (evt_cold_load_done && cld_multi_round)                  l_wb_base <= cfg_wb_base + {{(ADDR_W-6){1'b0}}, c_cur_round_len};
        else if (evt_cold_load_done && cld_multi_cins)                   l_wb_base <= cfg_wb_base + {{(ADDR_W-6){1'b0}}, c_cur_round_len};
        else if (evt_cold_load_done && cld_multi_tile)                   l_wb_base <= cfg_wb_base;
        else if (evt_cold_load_done && cld_multi_yout)                   l_wb_base <= cfg_wb_base;
        else if (evt_cold_load_done && cld_multi_cs)                     l_wb_base <= cfg_wb_base + cfg_wb_cout_step;
        else if (evt_ld_yout_wrap && !l_cs_is_last)                      l_wb_base <= l_wb_base_cs + cfg_wb_cout_step;
        else if (evt_ld_tile_wrap || evt_ld_cins_wrap || evt_ld_yout_wrap) l_wb_base <= l_wb_base_cs;
        else if (evt_ld_round_end)                                        l_wb_base <= l_wb_base + {{(ADDR_W-6){1'b0}}, l_cur_round_len};
    end

endmodule
