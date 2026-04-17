`timescale 1ns/1ps

// =============================================================================
// parf_accum.sv  --  Partial-Sum Accumulator with FILL/DRAIN overlap
//
// v2 优化：DRAIN 隐藏在下一 tile FILL 的 first_round 里（cross-tile overlap）。
// FILL 和 DRAIN 是两条并行推进的流水：
//   - FILL : wr_addr / kk_cnt / cins_cnt / fill_tile_cnt 推进，持续累加
//   - DRAIN: rd_addr / drain_tile_cnt 推进；drain_active 在 fill_tile_done 时拉起
//
// 冲突避免：
//   - Drain 读 parf_data[rd_addr] (旧值 = tile N 最终累加)
//   - Fill 的 first_round 写 parf_data[wr_addr] (新值 = tile N+1 第 0 次)
//   - 同拍同址：组合读旧值 → drain 拿 tile N；同步写覆盖 → 下拍 register = tile N+1
//   - 但 fill 不能在 first_round 内领先 drain (否则后续 drain 会读到 tile N+1)
//
// 同步规则：**overlap 期间 fill 只能在 drain 也 fire 时 fire**。
//   overlap = drain_active && fill_first_round
//   psum_in_ready = overlap ? acc_out_ready : 1'b1
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

    // ---- upstream: mac_array psum ----
    input  logic                                 psum_in_valid,
    input  logic signed [NUM_COL*PSUM_WIDTH-1:0] psum_in_vec,
    output logic                                 psum_in_ready,

    // ---- downstream: sdp / ofb_writer ----
    output logic                                 acc_out_valid,
    output logic signed [NUM_COL*PSUM_WIDTH-1:0] acc_out_vec,
    input  logic                                 acc_out_ready
);

    localparam int PAW = $clog2(PARF_DEPTH);

    // =========================================================================
    // 存储 + FILL / DRAIN 计数器声明
    // =========================================================================
    logic signed [NUM_COL*PSUM_WIDTH-1:0] parf_data [0:PARF_DEPTH-1];

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
    assign wr_is_last_col     = (wr_addr        == cur_valid_w_fill  - 6'd1);
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
    assign psum_in_ready = !overlap || acc_out_ready;
    assign acc_out_valid = drain_active;
    assign acc_out_vec   = parf_data[rd_addr];

    logic fill_fire, drain_fire;
    assign fill_fire  = psum_in_valid && psum_in_ready;
    assign drain_fire = drain_active  && acc_out_ready;

    // =========================================================================
    // 命名事件（comb）—— 把 FILL 推进链打平成一组 wrap 标志
    // =========================================================================
    logic ev_fill_wr_wrap;      // wr_addr 回卷 → kk_cnt 推进
    logic ev_fill_kk_wrap;      // kk_cnt  回卷 → cins_cnt 推进
    logic fill_tile_done;       // 当前 fill tile 的全部 cins × kk × col 完成
    logic drain_tile_done;      // 当前 drain tile 的全部 col 完成

    always_comb begin
        ev_fill_wr_wrap = fill_fire && wr_is_last_col;
        ev_fill_kk_wrap = ev_fill_wr_wrap && kk_is_last;
        fill_tile_done  = ev_fill_kk_wrap && cins_is_last;
        drain_tile_done = drain_fire && rd_is_last;
    end

    // =========================================================================
    // 累加新值 (comb)
    // =========================================================================
    logic signed [NUM_COL*PSUM_WIDTH-1:0] old_val, new_val;
    assign old_val = parf_data[wr_addr];

    always_comb begin
        for (int c = 0; c < NUM_COL; c++) begin
            logic signed [PSUM_WIDTH-1:0] lhs, rhs;
            lhs = is_first_round_fill ? '0 : old_val[c*PSUM_WIDTH +: PSUM_WIDTH];
            rhs = psum_in_vec[c*PSUM_WIDTH +: PSUM_WIDTH];
            new_val[c*PSUM_WIDTH +: PSUM_WIDTH] = lhs + rhs;
        end
    end

    // =========================================================================
    // FILL 侧寄存器（控制路径：决定写地址和 psum_in_ready，复位必须）
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (!rst_n)           wr_addr <= '0;
        else if (ev_fill_wr_wrap)  wr_addr <= '0;
        else if (fill_fire)        wr_addr <= wr_addr + {{(PAW-1){1'b0}}, 1'b1};
        else                       wr_addr <= wr_addr;
    end

    always_ff @(posedge clk) begin
        if      (!rst_n)           kk_cnt <= '0;
        else if (ev_fill_kk_wrap)  kk_cnt <= '0;
        else if (ev_fill_wr_wrap)  kk_cnt <= kk_cnt + 10'd1;
        else                       kk_cnt <= kk_cnt;
    end

    always_ff @(posedge clk) begin
        if      (!rst_n)           cins_cnt <= '0;
        else if (fill_tile_done)   cins_cnt <= '0;
        else if (ev_fill_kk_wrap)  cins_cnt <= cins_cnt + 6'd1;
        else                       cins_cnt <= cins_cnt;
    end

    always_ff @(posedge clk) begin
        if      (!rst_n)                                fill_tile_cnt <= '0;
        else if (fill_tile_done &&  fill_tile_is_last)  fill_tile_cnt <= '0;
        else if (fill_tile_done && !fill_tile_is_last)  fill_tile_cnt <= fill_tile_cnt + 8'd1;
        else                                            fill_tile_cnt <= fill_tile_cnt;
    end

    // =========================================================================
    // DRAIN 侧寄存器
    //   drain_active 控制路径（驱动 acc_out_valid）→ 必须复位。
    //   rd_addr / drain_tile_cnt：由 fill_tile_done 初始化，drain_active=0 时
    //   下游不观察，按 §6 不加复位。
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (!rst_n)           drain_active <= 1'b0;
        else if (fill_tile_done)   drain_active <= 1'b1;
        else if (drain_tile_done)  drain_active <= 1'b0;
        else                       drain_active <= drain_active;
    end

    always_ff @(posedge clk) begin
        if      (fill_tile_done)   rd_addr <= '0;
        else if (drain_tile_done)  rd_addr <= '0;
        else if (drain_fire)       rd_addr <= rd_addr + {{(PAW-1){1'b0}}, 1'b1};
        else                       rd_addr <= rd_addr;
    end

    // fill_tile_done 时 drain_tile_cnt 锁存当时的 fill_tile_cnt（同步拍）
    always_ff @(posedge clk) begin
        if      (fill_tile_done)                           drain_tile_cnt <= fill_tile_cnt;
        else if (drain_tile_done &&  drain_tile_is_last)   drain_tile_cnt <= '0;
        else if (drain_tile_done && !drain_tile_is_last)   drain_tile_cnt <= drain_tile_cnt + 8'd1;
        else                                               drain_tile_cnt <= drain_tile_cnt;
    end

    // =========================================================================
    // PARF 存储阵列（数据路径，无复位）
    //   每 tile 开头 is_first_round_fill=1 写 seed 覆盖，无需初始化。
    //   下游 acc_out_vec 被 drain_active 遮蔽，drain_active 的复位保证 X 不外泄。
    // =========================================================================
    always_ff @(posedge clk) begin
        if (fill_fire) parf_data[wr_addr] <= new_val;
        else           parf_data[wr_addr] <= parf_data[wr_addr];
    end

endmodule
