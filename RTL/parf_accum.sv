`timescale 1ns/1ps

// =============================================================================
// parf_accum.sv  --  Partial-Sum Accumulator (FILL/DRAIN overlap + per-col 子存储)
//
// 架构重构 (K 阶段): parf_data 拆分为每列独立的 parf_col 子模块, 外层管理
// FILL/DRAIN 计数、握手、地址生成. 为 Kx-fold 的 "每列独立 wr_addr 偏移" 做准备.
//
// v2 优化 (保留): DRAIN 隐藏在下一 tile FILL 的 first_round 里 (cross-tile overlap).
//   - FILL : wr_addr / kk_cnt / cins_cnt / fill_tile_cnt 推进, 持续累加
//   - DRAIN: rd_addr / drain_tile_cnt 推进; drain_active 在 fill_tile_done 时拉起
//
// 冲突避免 (保留):
//   - Drain 读 parf_col.rdata (旧值 = tile N 最终累加)
//   - Fill first_round 写 parf_col[wr_addr] (新值 = tile N+1 第 0 次)
//   - 同拍同址: 组合读旧值 → drain 拿 tile N; 同步写覆盖 → 下拍 register = tile N+1
//   - fill 不能在 first_round 内领先 drain (否则后续 drain 会读到 tile N+1)
//
// 同步规则 1 (overlap first_round):
//   overlap = drain_active && fill_first_round
//   psum_in_ready = overlap ? acc_out_ready : 1'b1
//
// 同步规则 2 (fill_tile_done vs drain_tile_done, Phase C-2 bug fix):
//   cur_valid_w_fill 很小时 (特别是 last_valid_w=1), fill tile 总时间可能
//   < drain 前一 tile 的时间. Fill 提前完成会覆盖 drain_tile_cnt / rd_addr.
//   Fix: fill 的最后一 fire 需要等 drain 完成前一 tile.
//
// 当前版本: 所有列共享 wr_addr / rd_addr / we (等价原单体版本).
// Kx-fold 未启用时功能零变化. 后续通过 per-col wr_addr 偏移引入折叠支持.
// =============================================================================
module parf_accum #(
    parameter int NUM_COL    = 16,
    parameter int PSUM_WIDTH = 32,
    parameter int PARF_DEPTH = 32
)(
    input  logic                                 clk,
    input  logic                                 rst_n,

    // ---- cfg ----
    input  logic [5:0]                           cfg_tile_w,
    input  logic [5:0]                           cfg_last_valid_w,
    input  logic [7:0]                           cfg_num_tiles,
    input  logic [5:0]                           cfg_cin_slices,
    input  logic [9:0]                           cfg_kk,

    // ---- Kx-fold cfg ----
    // 无 fold 时 cout_orig=16, kx_per_group=0 → 所有列 wr_addr 一致, 行为不变
    input  logic [5:0]                           cfg_fold_cout_orig,
    input  logic [4:0]                           cfg_fold_cout_groups,
    input  logic [3:0]                           cfg_fold_col_shift,  // kxper/stride (整数), 无 fold 时 0

    // ---- upstream: mac_array psum ----
    input  logic                                 psum_in_valid,
    input  logic signed [NUM_COL*PSUM_WIDTH-1:0] psum_in_vec,
    output logic                                 psum_in_ready,

    // ---- downstream: sdp / ofb_writer ----
    output logic                                 acc_out_valid,
    output logic signed [NUM_COL*PSUM_WIDTH-1:0] acc_out_vec,
    input  logic                                 acc_out_ready,

    // ---- seed feedback to mac_array (F-1b bias) ----
    output logic                                 is_first_round_fill_out,
    output logic signed [NUM_COL*PSUM_WIDTH-1:0] old_psum_at_wr
);

    localparam int PAW = $clog2(PARF_DEPTH);

    // =========================================================================
    // FILL / DRAIN 共享计数器
    // =========================================================================
    logic [PAW-1:0] wr_addr;
    logic [9:0]     kk_cnt;
    logic [5:0]     cins_cnt;
    logic [7:0]     fill_tile_cnt;

    logic [PAW-1:0] rd_addr;
    logic [7:0]     drain_tile_cnt;
    logic           drain_active;

    // =========================================================================
    // 派生量 + 边界
    // =========================================================================
    logic [5:0] cur_valid_w_fill, cur_valid_w_drain;
    assign cur_valid_w_fill  = (fill_tile_cnt  == cfg_num_tiles - 8'd1) ? cfg_last_valid_w : cfg_tile_w;
    assign cur_valid_w_drain = (drain_tile_cnt == cfg_num_tiles - 8'd1) ? cfg_last_valid_w : cfg_tile_w;

    logic wr_is_last_col, kk_is_last, cins_is_last, rd_is_last, fill_tile_is_last, drain_tile_is_last;
    // Kx-fold: fill 的 base 地址要扫到 cur_valid_w + (groups-1)*col_shift - 1,
    // 给最后一个列组留足 systolic 尾部 (以 iss_pos 单位计量, 已除以 stride).
    // 无 fold 时 col_shift=0 → 等价于原公式.
    logic [4:0] cur_cout_groups_m1;
    logic [8:0] cur_valid_w_fill_ext_9;
    logic [5:0] cur_valid_w_fill_ext;
    assign cur_cout_groups_m1    = cfg_fold_cout_groups - 5'd1;
    assign cur_valid_w_fill_ext_9 = {3'd0, cur_valid_w_fill} +
                                    cur_cout_groups_m1 * cfg_fold_col_shift;
    assign cur_valid_w_fill_ext   = cur_valid_w_fill_ext_9[5:0];

    assign wr_is_last_col     = (wr_addr        == cur_valid_w_fill_ext - 6'd1);
    assign kk_is_last         = (kk_cnt         == cfg_kk            - 10'd1);
    assign cins_is_last       = (cins_cnt       == cfg_cin_slices    - 6'd1);
    assign rd_is_last         = (rd_addr        == cur_valid_w_drain - 6'd1);
    assign fill_tile_is_last  = (fill_tile_cnt  == cfg_num_tiles     - 8'd1);
    assign drain_tile_is_last = (drain_tile_cnt == cfg_num_tiles     - 8'd1);

    logic is_first_round_fill;
    assign is_first_round_fill = (cins_cnt == 6'd0) && (kk_cnt == 10'd0);

    logic overlap;
    assign overlap = drain_active && is_first_round_fill;

    // =========================================================================
    // 握手
    // =========================================================================
    logic drain_fire;
    assign drain_fire = drain_active && acc_out_ready;

    logic fill_would_tile_done;
    logic drain_stall_fill;
    assign fill_would_tile_done = wr_is_last_col && kk_is_last && cins_is_last;
    assign drain_stall_fill     = fill_would_tile_done && drain_active &&
                                  !(drain_fire && rd_is_last);

    assign psum_in_ready = (!overlap || acc_out_ready) && !drain_stall_fill;
    assign acc_out_valid = drain_active;

    logic fill_fire;
    assign fill_fire = psum_in_valid && psum_in_ready;

    // =========================================================================
    // 推进事件 + tile-done
    // =========================================================================
    logic ev_fill_wr_wrap;
    logic ev_fill_kk_wrap;
    logic fill_tile_done;
    logic drain_tile_done;

    always_comb begin
        ev_fill_wr_wrap = fill_fire && wr_is_last_col;
        ev_fill_kk_wrap = ev_fill_wr_wrap && kk_is_last;
        fill_tile_done  = ev_fill_kk_wrap && cins_is_last;
        drain_tile_done = drain_fire && rd_is_last;
    end

    assign is_first_round_fill_out = is_first_round_fill;

    // =========================================================================
    // FILL 侧寄存器 (控制路径, 复位)
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (!rst_n)           wr_addr <= '0;
        else if (ev_fill_wr_wrap)  wr_addr <= '0;
        else if (fill_fire)        wr_addr <= wr_addr + {{(PAW-1){1'b0}}, 1'b1};
    end

    always_ff @(posedge clk) begin
        if      (!rst_n)           kk_cnt <= '0;
        else if (ev_fill_kk_wrap)  kk_cnt <= '0;
        else if (ev_fill_wr_wrap)  kk_cnt <= kk_cnt + 10'd1;
    end

    always_ff @(posedge clk) begin
        if      (!rst_n)           cins_cnt <= '0;
        else if (fill_tile_done)   cins_cnt <= '0;
        else if (ev_fill_kk_wrap)  cins_cnt <= cins_cnt + 6'd1;
    end

    always_ff @(posedge clk) begin
        if      (!rst_n)                                fill_tile_cnt <= '0;
        else if (fill_tile_done &&  fill_tile_is_last)  fill_tile_cnt <= '0;
        else if (fill_tile_done && !fill_tile_is_last)  fill_tile_cnt <= fill_tile_cnt + 8'd1;
    end

    // =========================================================================
    // DRAIN 侧寄存器
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (!rst_n)           drain_active <= 1'b0;
        else if (fill_tile_done)   drain_active <= 1'b1;
        else if (drain_tile_done)  drain_active <= 1'b0;
    end

    always_ff @(posedge clk) begin
        if      (fill_tile_done)   rd_addr <= '0;
        else if (drain_tile_done)  rd_addr <= '0;
        else if (drain_fire)       rd_addr <= rd_addr + {{(PAW-1){1'b0}}, 1'b1};
    end

    always_ff @(posedge clk) begin
        if      (fill_tile_done)                           drain_tile_cnt <= fill_tile_cnt;
        else if (drain_tile_done &&  drain_tile_is_last)   drain_tile_cnt <= '0;
        else if (drain_tile_done && !drain_tile_is_last)   drain_tile_cnt <= drain_tile_cnt + 8'd1;
    end

    // =========================================================================
    // 每列 PSUM 存储实例化
    //   Kx-fold: 每列 wr_addr = base - col_group * kx_per_group
    //            每列 we 仅在 wr_addr_col ∈ [0, cur_valid_w_fill) 范围内才使能
    //   无 fold (kx_per_group=0 或 cout_orig=16): offset=0, 所有列同步, 行为不变
    // =========================================================================
    logic                  we_col      [NUM_COL];
    logic [PAW-1:0]        wr_addr_col  [NUM_COL];

    // col_group[c] = c / cfg_fold_cout_orig. cout_orig ∈ {1,2,4,8,16}, 用 case 避免除法.
    always_comb begin
        for (int c = 0; c < NUM_COL; c++) begin
            logic [3:0] cg;
            logic signed [PAW+3:0] wa_ext;
            unique case (cfg_fold_cout_orig)
                6'd16:   cg = 4'd0;
                6'd8:    cg = {3'd0, c[3]};
                6'd4:    cg = {2'd0, c[3:2]};
                6'd2:    cg = {1'd0, c[3:1]};
                6'd1:    cg = c[3:0];
                default: cg = 4'd0;   // 非 pow-of-2: 退化为无 fold
            endcase
            wa_ext = $signed({4'd0, wr_addr}) -
                     $signed({{(PAW+4-4){1'b0}}, cg}) *
                     $signed({{(PAW+4-4){1'b0}}, cfg_fold_col_shift});
            // 有效区间: 0 ≤ wa_ext < cur_valid_w_fill
            we_col[c]      = fill_fire &&
                             !wa_ext[PAW+3] &&      // 非负
                             (wa_ext < $signed({4'd0, cur_valid_w_fill}));
            wr_addr_col[c] = wa_ext[PAW-1:0];
        end
    end

    // old_psum_at_wr 给 mac_array 做 acc_seed 融合: 用各列自己的 wr_addr_col 读
    generate
        for (genvar gc = 0; gc < NUM_COL; gc++) begin : g_col
            parf_col #(
                .PSUM_WIDTH(PSUM_WIDTH),
                .PARF_DEPTH(PARF_DEPTH)
            ) u_col (
                .clk      (clk),
                .we       (we_col[gc]),
                .wr_addr  (wr_addr_col[gc]),
                .wdata    (psum_in_vec  [gc*PSUM_WIDTH +: PSUM_WIDTH]),
                .rd_addr  (rd_addr),
                .rdata    (acc_out_vec  [gc*PSUM_WIDTH +: PSUM_WIDTH]),
                .old_at_wr(old_psum_at_wr[gc*PSUM_WIDTH +: PSUM_WIDTH])
            );
        end
    endgenerate

    // =========================================================================
    // 仿真 perf counters
    // =========================================================================
    // synthesis translate_off
    int fill_fire_cnt  = 0;
    int drain_fire_cnt = 0;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            fill_fire_cnt  <= 0;
            drain_fire_cnt <= 0;
        end else begin
            if (fill_fire)  fill_fire_cnt  <= fill_fire_cnt  + 1;
            if (drain_fire) drain_fire_cnt <= drain_fire_cnt + 1;
        end
    end
    // synthesis translate_on

endmodule
