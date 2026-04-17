`timescale 1ns/1ps

// =============================================================================
// wgt_buffer.sv  --  Weight Stream Feeder (WB → WRF → MAC)
//
// v3: packed 路径保持不变；chunked 路径重写为 pipelined LOAD/COMPUTE overlap。
//
// --- Packed (K²·cin_slices ≤ 32) ---
//     S_IDLE → S_LOAD → S_COMPUTE → S_DONE (原 v2 逻辑保持不变)
//     每 cs 入口 LOAD 一次 total_wrf 个权重；之后 COMPUTE 整个 cs。
//     COMPUTE 6 层循环 (yout, tile, cins, ky, kx, x)，wrf_raddr 用 running base
//     (cins*kk + ky*K + kx) 累加。
//
// --- Chunked (K²·cin_slices > 32) ---  pipelined, v3
//     WRF 有独立读写口。两套独立 ptr：
//       * compute ptr (x_cnt, pos_cnt, round_cnt, cins_cnt, tile_cnt, yout_cnt, cs_cnt)
//         drive wrf_raddr = pos_cnt 和 wgt_valid；按 wgt_fire 推进，一个 pos
//         持续 cur_valid_w 拍。
//       * load ptr    (l_pos, l_slots_done, l_round, l_cins, l_tile, l_yout, l_cs)
//         drive wb_raddr / wrf_we / wrf_waddr；target 始终为"compute 下一 round"。
//         l_pos 在 [0, l_cur_round_len) 内循环走；l_slots_done 跟踪写了多少 slot。
//         起点 l_start_pos_next: 若下一 target round 比刚装完的 round 长，从
//         c_cur_round_len 开始 wrap（先写 compute 本 round 不读的高 slot，避免冲突）；
//         否则从 0 顺序写。
//
//     状态机：S_IDLE → S_LOAD (冷启动填 round 0) → S_COMPUTE (compute+load 并行) → S_DONE
//
//     Hazard：WRF[p] 被 load 写的瞬间 (posedge t+2)，compute 不能正在读 p。
//       wb_re 在 cycle t 发 → wb_rdata cycle t+1 到位 → posedge t+2 写 WRF[p]。
//       安全条件（三选一）：
//         (a) l_pos >= c_cur_round_len: compute 本 round 根本不读 slot l_pos，自由写。
//         (b) pos_cnt > l_pos: compute 已离开 slot l_pos。
//         (c) pos_cnt == l_pos && x_is_last && wgt_fire: 本拍 compute 最后一次读
//             slot l_pos，下一 posedge cpos 推进，WRF[p] 在 posedge t+2 写入时
//             compute 已在 slot l_pos+1（或下一 round 的 slot 0）。
//       由于 compute 一个 pos 持续 cur_valid_w 拍、load 一拍一 slot，load 极易跑赢。
//
//     同步状态：load_caught_up ∈ {0,1}。语义 rounds_ahead=(load_target_round -
//     compute_round)，合法范围 [1,2]。
//       load_caught_up=0 <=> rounds_ahead=1 (load 正在装 compute 的下一 round)
//       load_caught_up=1 <=> rounds_ahead=2 (load 已领先 2 round，等 compute 消费)
//       cold_load 完成后 load ptr 指向 round 1, load_caught_up=0（需要装 round 1）。
//       Load 装完一 round → load_caught_up <= 1。
//       Compute 跨 round → load_caught_up <= 0。
//       同拍两事件都发生 → 保持当前值（净零）。
// =============================================================================

module wgt_buffer #(
    parameter int NUM_COL    = 16,
    parameter int NUM_PE     = 16,
    parameter int DATA_WIDTH = 8,
    parameter int WRF_DEPTH  = 32,
    parameter int SRAM_DEPTH = 8192,
    parameter int ADDR_W     = 20
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
    input  logic                                 wgt_ready       // = mac.compute_en
);

    localparam int AW  = $clog2(SRAM_DEPTH);
    localparam int WAW = $clog2(WRF_DEPTH);

    // =========================================================================
    // State
    // =========================================================================
    typedef enum logic [1:0] {
        S_IDLE    = 2'd0,
        S_LOAD    = 2'd1,    // packed: 常规 LOAD；chunked: cold_load
        S_COMPUTE = 2'd2,    // packed: 常规 COMPUTE；chunked: RUN (compute+load 并行)
        S_DONE    = 2'd3
    } state_t;
    state_t state, state_next;

    // =========================================================================
    // Compute ptr
    // =========================================================================
    logic [5:0]  x_cnt;
    logic [5:0]  cins_cnt;
    logic [7:0]  tile_cnt;
    logic [15:0] yout_cnt;
    logic [5:0]  cs_cnt;

    // Packed 专用
    logic [3:0]  kx_cnt;
    logic [3:0]  ky_cnt;
    logic [5:0]  wrf_base_cins;
    logic [5:0]  wrf_base_ky;
    logic [5:0]  wrf_base_kx;

    // Chunked compute 专用
    logic [5:0]  pos_cnt;        // 0..c_cur_round_len-1
    logic [2:0]  round_cnt;      // 0..rounds_per_cins-1

    // =========================================================================
    // Load ptr (仅 chunked)
    // =========================================================================
    logic [5:0]  l_pos;          // 当前要写的 WRF slot / WB 偏移
    logic [5:0]  l_slots_done;   // 当前 load round 已发 wb_re 的 slot 数
    logic [2:0]  l_round;
    logic [5:0]  l_cins;
    logic [7:0]  l_tile;
    logic [15:0] l_yout;
    logic [5:0]  l_cs;
    logic        l_exhausted;     // 最后一 round 已装完 → wb_re 停
    logic        load_caught_up;  // 1 = load 领先 compute 一 round，等 compute

    // =========================================================================
    // Packed/Cold LOAD 计数器 + 1-拍延迟
    // =========================================================================
    logic [9:0]  wb_rd_cnt;
    logic [9:0]  wrf_wr_cnt;
    logic        wb_re_d1;           // for packed & cold load

    // Chunked run 侧延迟寄存器（wb_rdata 回来时用）
    logic                   l_wb_re_d1;
    logic [WAW-1:0]         l_wrf_waddr_d1;

    // =========================================================================
    // Running bases
    // =========================================================================
    logic [ADDR_W-1:0] cur_wb_base_cs;   // compute 侧 cs base（packed/chunked 都用）
    logic [ADDR_W-1:0] cur_wb_rd_base;   // packed cold-load 的 WB 起点
    logic [ADDR_W-1:0] l_wb_base;        // chunked: 当前 load round 的 WB 起点
    logic [ADDR_W-1:0] l_wb_base_cs;     // chunked: load 侧 cs base

    // =========================================================================
    // 派生量
    // =========================================================================
    logic [5:0] cur_valid_w;
    assign cur_valid_w = (tile_cnt == cfg_num_tiles - 8'd1) ? cfg_last_valid_w : cfg_tile_w;

    // compute 侧当前 round 长度
    logic [5:0] c_cur_round_len;
    assign c_cur_round_len = (round_cnt == cfg_rounds_per_cins - 3'd1) ? cfg_round_len_last : 6'd32;

    // load 侧当前 round 长度
    logic [5:0] l_cur_round_len;
    assign l_cur_round_len = (l_round == cfg_rounds_per_cins - 3'd1) ? cfg_round_len_last : 6'd32;

    // Packed LOAD 一次灌 total_wrf；chunked cold_load 灌第一 round (round_cnt=0)
    logic [9:0] cur_load_len;
    assign cur_load_len = cfg_wrf_packed ? cfg_total_wrf : {4'd0, c_cur_round_len};

    // =========================================================================
    // Compute 边界 flags
    // =========================================================================
    logic x_is_last, cins_is_last, tile_is_last, yout_is_last, cs_is_last;
    assign x_is_last    = (x_cnt    == cur_valid_w       - 6'd1);
    assign cins_is_last = (cins_cnt == cfg_cin_slices    - 6'd1);
    assign tile_is_last = (tile_cnt == cfg_num_tiles     - 8'd1);
    assign yout_is_last = (yout_cnt == cfg_h_out         - 16'd1);
    assign cs_is_last   = (cs_cnt   == cfg_cout_slices   - 6'd1);

    logic kx_is_last, ky_is_last;
    assign kx_is_last = (kx_cnt == cfg_k - 4'd1);
    assign ky_is_last = (ky_cnt == cfg_k - 4'd1);

    logic pos_is_last, round_is_last;
    assign pos_is_last   = (pos_cnt   == c_cur_round_len       - 6'd1);
    assign round_is_last = (round_cnt == cfg_rounds_per_cins   - 3'd1);

    // Load 边界 flags
    logic l_round_done, l_round_is_last, l_cins_is_last;
    logic l_tile_is_last, l_yout_is_last, l_cs_is_last;
    // l_round_done: 本 round 最后一 slot 即将发 wb_re (l_slots_done == l_cur_round_len-1)
    assign l_round_done    = (l_slots_done == l_cur_round_len     - 6'd1);
    assign l_round_is_last = (l_round      == cfg_rounds_per_cins - 3'd1);
    assign l_cins_is_last  = (l_cins       == cfg_cin_slices      - 6'd1);
    assign l_tile_is_last  = (l_tile       == cfg_num_tiles       - 8'd1);
    assign l_yout_is_last  = (l_yout       == cfg_h_out           - 16'd1);
    assign l_cs_is_last    = (l_cs         == cfg_cout_slices     - 6'd1);

    // Next l_pos (wrap at l_cur_round_len)
    logic [5:0] l_pos_next;
    assign l_pos_next = (l_pos == l_cur_round_len - 6'd1) ? 6'd0 : (l_pos + 6'd1);

    // 下一 load round 的 round_len（基于 l_round_next）
    // l_round advances wraps with other counters. 对于 start_pos 计算，我们关心"下一
    // target round 的 len" vs "刚装完的 target round 的 len (= l_cur_round_len)"。
    // l_round_next = l_round_is_last ? 0 : l_round+1。
    logic [2:0] l_round_next;
    assign l_round_next = l_round_is_last ? 3'd0 : (l_round + 3'd1);
    logic [5:0] l_cur_round_len_next;
    assign l_cur_round_len_next = (l_round_next == cfg_rounds_per_cins - 3'd1)
                                ? cfg_round_len_last : 6'd32;
    // l_start_pos_next: 下一 target round 开始的 l_pos。
    //   load 下一次开始装时，compute 刚刚进到"load 刚装完的 round" (= current l_round 的目标)。
    //   那时 c_cur_round_len = l_cur_round_len (current)。
    //   若 new target's round_len > 该值，前 (new - cur) slot 可自由写（compute 不读），
    //   从 l_cur_round_len 开始写，后续 wrap。否则从 0 开始顺序写。
    logic [5:0] l_start_pos_next;
    assign l_start_pos_next = (l_cur_round_len_next > l_cur_round_len)
                            ? l_cur_round_len : 6'd0;

    // 当前 cs 的 COMPUTE 是否打完最后一拍
    logic cs_compute_last;
    assign cs_compute_last = cfg_wrf_packed
        ? (x_is_last && kx_is_last  && ky_is_last    && cins_is_last && tile_is_last && yout_is_last)
        : (x_is_last && pos_is_last && round_is_last && cins_is_last && tile_is_last && yout_is_last);

    // =========================================================================
    // LOAD 驱动信号
    // =========================================================================
    // Packed/cold_load 路径（state==S_LOAD）
    logic load_wb_re_gate_pc;
    assign load_wb_re_gate_pc = (state == S_LOAD) && (wb_rd_cnt < cur_load_len);

    // wgt_fire 提前声明（hazard 条件要用）
    logic wgt_fire;
    // wgt_valid 在下面才 assign；但 wgt_valid = (state == S_COMPUTE)，用 state 直接写。
    // wgt_ready 是 input port。
    assign wgt_fire = (state == S_COMPUTE) && wgt_ready;

    // Chunked run 路径（state==S_COMPUTE, chunked）
    // hazard: compute 在 wb_re 后的 posedge t+2 及以后不再读 WRF[l_pos]。
    //   - l_pos >= c_cur_round_len: compute 本 round 不用该 slot，自由写。
    //   - pos_cnt > l_pos: compute 已离开 slot l_pos。
    //   - pos_cnt == l_pos && x_is_last && wgt_fire: 本拍 compute 读完 slot l_pos 的
    //     最后一 fire，下一 posedge cpos++，WRF[l_pos] 在 posedge t+2 写入时
    //     compute 已在 l_pos+1，无冲突。这一 case 让 load 能装最后一 slot
    //     （即使 round_len 相等时也能穿越边界）。
    logic chunked_hazard_ok;
    assign chunked_hazard_ok = (l_pos >= c_cur_round_len)
                            || (pos_cnt >  l_pos)
                            || (pos_cnt == l_pos && x_is_last && wgt_fire);

    logic chunked_load_active;
    assign chunked_load_active = !cfg_wrf_packed && (state == S_COMPUTE)
                                 && !l_exhausted && !load_caught_up
                                 && chunked_hazard_ok;

    // 综合 wb_re
    assign wb_re = load_wb_re_gate_pc | chunked_load_active;

    // wb_raddr
    logic [ADDR_W-1:0] wb_raddr_pc;
    logic [ADDR_W-1:0] wb_raddr_run;
    assign wb_raddr_pc  = cur_wb_rd_base + {{(ADDR_W-10){1'b0}}, wb_rd_cnt};
    assign wb_raddr_run = l_wb_base      + {{(ADDR_W-6) {1'b0}}, l_pos};
    assign wb_raddr     = chunked_load_active ? wb_raddr_run[AW-1:0]
                                              : wb_raddr_pc [AW-1:0];

    // =========================================================================
    // WRF 写
    // =========================================================================
    logic wrf_we_gate_pc;
    logic wrf_we_gate_run;
    assign wrf_we_gate_pc  = wb_re_d1;            // packed & cold load
    assign wrf_we_gate_run = l_wb_re_d1;          // chunked run

    assign wrf_we    = {(NUM_COL*NUM_PE){wrf_we_gate_pc | wrf_we_gate_run}};
    assign wrf_waddr = wrf_we_gate_run ? l_wrf_waddr_d1 : wrf_wr_cnt[WAW-1:0];
    assign wrf_wdata = wb_rdata;

    // =========================================================================
    // COMPUTE 输出
    // =========================================================================
    assign wgt_valid = (state == S_COMPUTE);
    assign wrf_raddr = cfg_wrf_packed ? wrf_base_kx[WAW-1:0] : pos_cnt[WAW-1:0];
    // wgt_fire 已在前面声明并 assign

    // =========================================================================
    // 状态转移
    // =========================================================================
    logic load_done_pc;
    assign load_done_pc = (wrf_wr_cnt == cur_load_len);

    always_comb begin
        state_next = state;
        case (state)
            S_IDLE:    if (start)        state_next = S_LOAD;
            S_LOAD:    if (load_done_pc) state_next = S_COMPUTE;
            S_COMPUTE: if (wgt_fire) begin
                if (cs_compute_last)
                    // packed: 下个 cs 要重 LOAD；chunked: 继续 S_COMPUTE（load 内部并行）
                    state_next = cs_is_last ? S_DONE
                               : (cfg_wrf_packed ? S_LOAD : S_COMPUTE);
            end
            S_DONE:    ;
            default:   state_next = S_IDLE;
        endcase
    end

    assign done = (state == S_DONE);

    // =========================================================================
    // 同步事件（chunked）
    // =========================================================================
    logic compute_round_done;
    assign compute_round_done = !cfg_wrf_packed && (state == S_COMPUTE) && wgt_fire
                                && x_is_last && pos_is_last;

    // load 刚装完一 round 的最后一 wb_re
    logic load_round_done_fire;
    assign load_round_done_fire = chunked_load_active && l_round_done;

    // =========================================================================
    // 主时序
    // =========================================================================
    logic [5:0] kk_lo6;
    logic [5:0] k_lo6;
    assign kk_lo6 = cfg_kk[5:0];
    assign k_lo6  = {2'd0, cfg_k};

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            x_cnt          <= '0;
            kx_cnt         <= '0;
            ky_cnt         <= '0;
            cins_cnt       <= '0;
            tile_cnt       <= '0;
            yout_cnt       <= '0;
            cs_cnt         <= '0;
            pos_cnt        <= '0;
            round_cnt      <= '0;
            wb_rd_cnt      <= '0;
            wrf_wr_cnt     <= '0;
            wb_re_d1       <= 1'b0;
            cur_wb_base_cs <= '0;
            cur_wb_rd_base <= '0;
            wrf_base_cins  <= '0;
            wrf_base_ky    <= '0;
            wrf_base_kx    <= '0;
            l_pos          <= '0;
            l_slots_done   <= '0;
            l_round        <= '0;
            l_cins         <= '0;
            l_tile         <= '0;
            l_yout         <= '0;
            l_cs           <= '0;
            l_exhausted    <= 1'b0;
            load_caught_up <= 1'b0;
            l_wb_re_d1     <= 1'b0;
            l_wrf_waddr_d1 <= '0;
            l_wb_base      <= '0;
            l_wb_base_cs   <= '0;
        end else begin
            state <= state_next;

            // 上一拍 wb_re 的 1-拍延迟 → 本拍写 WRF
            wb_re_d1       <= load_wb_re_gate_pc;
            l_wb_re_d1     <= chunked_load_active;
            l_wrf_waddr_d1 <= l_pos[WAW-1:0];

            // ---- IDLE → LOAD ----
            if (state == S_IDLE && start) begin
                x_cnt          <= '0;
                kx_cnt         <= '0;
                ky_cnt         <= '0;
                cins_cnt       <= '0;
                tile_cnt       <= '0;
                yout_cnt       <= '0;
                cs_cnt         <= '0;
                pos_cnt        <= '0;
                round_cnt      <= '0;
                wb_rd_cnt      <= '0;
                wrf_wr_cnt     <= '0;
                cur_wb_base_cs <= cfg_wb_base;
                cur_wb_rd_base <= cfg_wb_base;
                wrf_base_cins  <= '0;
                wrf_base_ky    <= '0;
                wrf_base_kx    <= '0;
                l_pos          <= '0;
                l_slots_done   <= '0;
                l_round        <= '0;
                l_cins         <= '0;
                l_tile         <= '0;
                l_yout         <= '0;
                l_cs           <= '0;
                l_exhausted    <= 1'b0;
                load_caught_up <= 1'b0;
                l_wb_base      <= cfg_wb_base;
                l_wb_base_cs   <= cfg_wb_base;
            end

            // ---- S_LOAD: packed/cold load ----
            if (state == S_LOAD) begin
                if (load_wb_re_gate_pc) wb_rd_cnt  <= wb_rd_cnt  + 10'd1;
                if (wb_re_d1)           wrf_wr_cnt <= wrf_wr_cnt + 10'd1;
            end

            // ---- LOAD → COMPUTE 切换 ----
            if (state == S_LOAD && state_next == S_COMPUTE) begin
                wb_rd_cnt  <= '0;
                wrf_wr_cnt <= '0;
                x_cnt      <= '0;
                pos_cnt    <= '0;
                if (cfg_wrf_packed) begin
                    kx_cnt        <= '0;
                    ky_cnt        <= '0;
                    cins_cnt      <= '0;
                    tile_cnt      <= '0;
                    yout_cnt      <= '0;
                    wrf_base_cins <= '0;
                    wrf_base_ky   <= '0;
                    wrf_base_kx   <= '0;
                end
                // chunked cold_load 完成：
                //   - 把 load ptr 推进一个 round（从 (cs=0,...,round=0) 推到下一 round）
                //   - load_caught_up <= 1（下一 round 还没装，等 compute 跨过 round 0）
                //   等等，这里的语义其实是：下一 round 还需要装。cold_load 完成后：
                //     compute 从 round 0 slot 0 读。
                //     load 目标是 round 1 slot 0（或若 only 1 round，就是下一 cins/tile 的 round 0）。
                //   但 compute 现在 pos_cnt=0，load 想发 wb_re(l_pos=0)：hazard 要 pos_cnt > 0。
                //   所以 load 必须等 compute 推进到 pos_cnt=1 以后。
                //   这个等待由 chunked_hazard_ok 自然实现。
                //   load_caught_up 设 0 才能让 load 启动；设 1 会卡死。
                //
                //   然而 load 仅当 hazard_ok && !load_caught_up 时启动。load_caught_up 的语义
                //   是"load 已领先 compute 一 round，等 compute 消费"。cold_load 完成后，
                //   load 把 round 0 装进 WRF，compute 将读 round 0，load 下一步是装 round 1。
                //   此时 rounds_ahead=1（load 目标 round 1 vs compute round 0），load_caught_up=1？
                //   但 load 还没装 round 1，它需要装 round 1。
                //
                //   我重新设计语义：load_caught_up = 1 表示 "下一 round 已装完"。
                //   cold_load 装的是 round 0 = 当前 compute round，不算"下一 round"。
                //   所以 cold_load 完成后 load_caught_up = 0（需要去装 round 1）。
                //   load 发 wb_re 直到 l_pos 走完 l_round 1 的所有 slot，然后 load_caught_up=1。
                if (!cfg_wrf_packed) begin
                    // load ptr 要推进到"下一 round"。cold load 装的是 round 0（最长 round，
                    // 或唯一 round）。next target round 的 len <= 当前 cold_load round len，
                    // 所以 start_pos = 0。
                    l_pos        <= '0;
                    l_slots_done <= '0;
                    if (cfg_rounds_per_cins != 3'd1) begin
                        // 多 round：l_round <= 1, l_wb_base += c_cur_round_len
                        l_round   <= 3'd1;
                        l_wb_base <= cfg_wb_base + {{(ADDR_W-6){1'b0}}, c_cur_round_len};
                    end else if (cfg_cin_slices != 6'd1) begin
                        // 单 round、多 cins：l_cins <= 1, l_wb_base += cur_round_len (= kk)
                        l_cins    <= 6'd1;
                        l_wb_base <= cfg_wb_base + {{(ADDR_W-6){1'b0}}, c_cur_round_len};
                    end else if (cfg_num_tiles != 8'd1) begin
                        // 单 cins、多 tile：l_tile <= 1, l_wb_base 不变（tile 共享 WB 内容）
                        l_tile    <= 8'd1;
                        l_wb_base <= cfg_wb_base;
                    end else if (cfg_h_out != 16'd1) begin
                        l_yout    <= 16'd1;
                        l_wb_base <= cfg_wb_base;
                    end else if (!cs_is_last) begin
                        // 只有一个 (yout,tile,cins,round) —— 下一 cs 就是下一 "round"
                        l_cs         <= 6'd1;
                        l_wb_base_cs <= cfg_wb_base + cfg_wb_cout_step;
                        l_wb_base    <= cfg_wb_base + cfg_wb_cout_step;
                    end else begin
                        // 总共只有一个 round：load 已经做完全部工作（cold load 填的就是 round 0
                        // 且再没有下一 round）。但这种情况不会出现在 chunked（chunked 说明
                        // total_wrf>32，必然 rounds_per_cins>1 或 cin_slices>1）。
                        l_exhausted <= 1'b1;
                    end
                    load_caught_up <= 1'b0;  // 需要装下一 round
                end
            end

            // ---- S_COMPUTE packed 路径：counter 推进 ----
            if (cfg_wrf_packed && state == S_COMPUTE && wgt_fire) begin
                if (x_is_last) begin
                    x_cnt <= '0;
                    if (kx_is_last) begin
                        kx_cnt <= '0;
                        if (ky_is_last) begin
                            ky_cnt <= '0;
                            if (cins_is_last) begin
                                cins_cnt <= '0;
                                if (tile_is_last) begin
                                    tile_cnt <= '0;
                                    if (yout_is_last) begin
                                        yout_cnt <= '0;
                                        if (!cs_is_last) begin
                                            cs_cnt         <= cs_cnt + 6'd1;
                                            cur_wb_base_cs <= cur_wb_base_cs + cfg_wb_cout_step;
                                            cur_wb_rd_base <= cur_wb_base_cs + cfg_wb_cout_step;
                                        end
                                    end else begin
                                        yout_cnt      <= yout_cnt + 16'd1;
                                        wrf_base_cins <= '0;
                                        wrf_base_ky   <= '0;
                                        wrf_base_kx   <= '0;
                                    end
                                end else begin
                                    tile_cnt      <= tile_cnt + 8'd1;
                                    wrf_base_cins <= '0;
                                    wrf_base_ky   <= '0;
                                    wrf_base_kx   <= '0;
                                end
                            end else begin
                                cins_cnt      <= cins_cnt + 6'd1;
                                wrf_base_cins <= wrf_base_cins + kk_lo6;
                                wrf_base_ky   <= wrf_base_cins + kk_lo6;
                                wrf_base_kx   <= wrf_base_cins + kk_lo6;
                            end
                        end else begin
                            ky_cnt      <= ky_cnt + 4'd1;
                            wrf_base_ky <= wrf_base_ky + k_lo6;
                            wrf_base_kx <= wrf_base_ky + k_lo6;
                        end
                    end else begin
                        kx_cnt      <= kx_cnt + 4'd1;
                        wrf_base_kx <= wrf_base_kx + 6'd1;
                    end
                end else begin
                    x_cnt <= x_cnt + 6'd1;
                end
            end

            // ---- S_COMPUTE chunked 路径：compute ptr 推进 ----
            if (!cfg_wrf_packed && state == S_COMPUTE && wgt_fire) begin
                if (x_is_last) begin
                    x_cnt <= '0;
                    if (pos_is_last) begin
                        pos_cnt <= '0;
                        if (round_is_last) begin
                            round_cnt <= '0;
                            if (cins_is_last) begin
                                cins_cnt <= '0;
                                if (tile_is_last) begin
                                    tile_cnt <= '0;
                                    if (yout_is_last) begin
                                        yout_cnt <= '0;
                                        if (!cs_is_last) begin
                                            cs_cnt         <= cs_cnt + 6'd1;
                                            cur_wb_base_cs <= cur_wb_base_cs + cfg_wb_cout_step;
                                        end
                                    end else begin
                                        yout_cnt <= yout_cnt + 16'd1;
                                    end
                                end else begin
                                    tile_cnt <= tile_cnt + 8'd1;
                                end
                            end else begin
                                cins_cnt <= cins_cnt + 6'd1;
                            end
                        end else begin
                            round_cnt <= round_cnt + 3'd1;
                        end
                    end else begin
                        pos_cnt <= pos_cnt + 6'd1;
                    end
                end else begin
                    x_cnt <= x_cnt + 6'd1;
                end
            end

            // ---- S_COMPUTE chunked 路径：load ptr 推进 ----
            // 每当本拍 chunked_load_active=1（即发了 wb_re），l_slots_done++、l_pos wrap。
            // 若 l_round_done=1：清零 l_slots_done，推进外层 counter，设 l_pos 到 start_pos_next。
            if (chunked_load_active) begin
                if (l_round_done) begin
                    // 跨 round：l_slots_done 归 0，l_pos 跳到下一 target round 的 start
                    l_slots_done <= '0;
                    l_pos        <= l_start_pos_next;
                    if (l_round_is_last) begin
                        l_round <= '0;
                        if (l_cins_is_last) begin
                            l_cins    <= '0;
                            if (l_tile_is_last) begin
                                l_tile    <= '0;
                                if (l_yout_is_last) begin
                                    l_yout <= '0;
                                    if (!l_cs_is_last) begin
                                        l_cs         <= l_cs + 6'd1;
                                        l_wb_base_cs <= l_wb_base_cs + cfg_wb_cout_step;
                                        l_wb_base    <= l_wb_base_cs + cfg_wb_cout_step;
                                    end else begin
                                        l_exhausted <= 1'b1;
                                    end
                                end else begin
                                    l_yout    <= l_yout + 16'd1;
                                    l_wb_base <= l_wb_base_cs;
                                end
                            end else begin
                                l_tile    <= l_tile + 8'd1;
                                l_wb_base <= l_wb_base_cs;
                            end
                        end else begin
                            l_cins    <= l_cins + 6'd1;
                            l_wb_base <= l_wb_base + {{(ADDR_W-6){1'b0}}, l_cur_round_len};
                        end
                    end else begin
                        l_round   <= l_round + 3'd1;
                        l_wb_base <= l_wb_base + {{(ADDR_W-6){1'b0}}, l_cur_round_len};
                    end
                end else begin
                    l_slots_done <= l_slots_done + 6'd1;
                    l_pos        <= l_pos_next;
                end
            end

            // ---- load_caught_up 同步 ----
            // 语义: load_caught_up=1 <=> rounds_ahead==2 (load 超前 compute 2 round，停)
            //       load_caught_up=0 <=> rounds_ahead==1 (load 正在装下一 round，活跃)
            //   Compute 跨 round (compute_round_id++): rounds_ahead--
            //   Load 完成一 round (load_round_id++): rounds_ahead++
            //   同拍两事件: rounds_ahead 不变 → load_caught_up 保持
            if (!cfg_wrf_packed) begin
                if (compute_round_done && load_round_done_fire) begin
                    load_caught_up <= load_caught_up;   // no-op, keep
                end else if (compute_round_done) begin
                    load_caught_up <= 1'b0;
                end else if (load_round_done_fire) begin
                    load_caught_up <= 1'b1;
                end
            end
        end
    end

endmodule
