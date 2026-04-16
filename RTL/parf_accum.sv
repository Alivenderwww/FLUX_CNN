`timescale 1ns/1ps

// =============================================================================
// parf_accum.sv  --  Partial-Sum Accumulator with FILL/DRAIN overlap
//
// v2 优化：DRAIN 隐藏在下一 tile FILL 的 first_round 里（cross-tile overlap）。
// FILL 和 DRAIN 成为两个独立并行运行的 FSM：
//   - FILL: wr_addr / kk_cnt / cins_cnt / fill_tile_cnt 推进，持续累加
//   - DRAIN: rd_addr / drain_tile_cnt 推进；drain_active 在 fill_tile_done 时拉起
//
// 冲突避免：
//   - Drain 读 parf_data[rd_addr] (旧值 = tile N 最终累加)
//   - Fill 的 first_round 写 parf_data[wr_addr] (新值 = tile N+1 第 0 次)
//   - 同拍同址：组合读旧值 → drain 拿 tile N；同步写覆盖 → 下拍 register = tile N+1.
//   - 但 fill 不能在 first_round 内领先 drain (否则后续 drain 会读到 tile N+1)。
//
// 同步规则：**overlap 期间 fill 只能在 drain 也 fire 时 fire**。
//   overlap = drain_active && fill_first_round
//   psum_in_ready = overlap ? acc_out_ready : 1'b1
//
// v1 的纯计数 (cin_slices × kk) 累加轮次不变；PARF_DEPTH 不变；对外接口不变。
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
    // 存储
    // =========================================================================
    logic signed [NUM_COL*PSUM_WIDTH-1:0] parf_data [0:PARF_DEPTH-1];

    // =========================================================================
    // FILL 侧寄存器
    // =========================================================================
    logic [PAW-1:0] wr_addr;
    logic [9:0]     kk_cnt;
    logic [5:0]     cins_cnt;
    logic [7:0]     fill_tile_cnt;

    // =========================================================================
    // DRAIN 侧寄存器
    // =========================================================================
    logic [PAW-1:0] rd_addr;
    logic [7:0]     drain_tile_cnt;
    logic           drain_active;

    // =========================================================================
    // cur_valid_w (fill 和 drain 用不同 tile_cnt)
    // =========================================================================
    logic [5:0] cur_valid_w_fill, cur_valid_w_drain;
    assign cur_valid_w_fill  = (fill_tile_cnt  == cfg_num_tiles - 8'd1) ? cfg_last_valid_w : cfg_tile_w;
    assign cur_valid_w_drain = (drain_tile_cnt == cfg_num_tiles - 8'd1) ? cfg_last_valid_w : cfg_tile_w;

    // =========================================================================
    // 边界
    // =========================================================================
    logic wr_is_last_col, kk_is_last, cins_is_last, rd_is_last;
    assign wr_is_last_col = (wr_addr  == cur_valid_w_fill  - 6'd1);
    assign kk_is_last     = (kk_cnt   == cfg_kk            - 10'd1);
    assign cins_is_last   = (cins_cnt == cfg_cin_slices    - 6'd1);
    assign rd_is_last     = (rd_addr  == cur_valid_w_drain - 6'd1);

    // =========================================================================
    // first_round + overlap 同步
    // =========================================================================
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

    logic fill_tile_done, drain_tile_done;
    assign fill_tile_done  = fill_fire  && wr_is_last_col && kk_is_last && cins_is_last;
    assign drain_tile_done = drain_fire && rd_is_last;

    // =========================================================================
    // 累加新值
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
    // 时序
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_addr        <= '0;
            kk_cnt         <= '0;
            cins_cnt       <= '0;
            fill_tile_cnt  <= '0;
            rd_addr        <= '0;
            drain_tile_cnt <= '0;
            drain_active   <= 1'b0;
            for (int i = 0; i < PARF_DEPTH; i++) parf_data[i] <= '0;
        end else begin
            // ---- FILL: 累加 + 计数推进 ----
            if (fill_fire) begin
                parf_data[wr_addr] <= new_val;
                if (wr_is_last_col) begin
                    wr_addr <= '0;
                    if (kk_is_last) begin
                        kk_cnt <= '0;
                        if (cins_is_last) begin
                            cins_cnt      <= '0;
                            fill_tile_cnt <= (fill_tile_cnt == cfg_num_tiles - 8'd1) ? 8'd0
                                                                                     : fill_tile_cnt + 8'd1;
                        end else begin
                            cins_cnt <= cins_cnt + 6'd1;
                        end
                    end else begin
                        kk_cnt <= kk_cnt + 10'd1;
                    end
                end else begin
                    wr_addr <= wr_addr + {{(PAW-1){1'b0}}, 1'b1};
                end
            end

            // ---- DRAIN: 推进 rd_addr + 更新 drain_tile_cnt ----
            if (drain_fire) begin
                if (rd_is_last) begin
                    rd_addr        <= '0;
                    drain_active   <= 1'b0;
                    drain_tile_cnt <= (drain_tile_cnt == cfg_num_tiles - 8'd1) ? 8'd0
                                                                               : drain_tile_cnt + 8'd1;
                end else begin
                    rd_addr <= rd_addr + {{(PAW-1){1'b0}}, 1'b1};
                end
            end

            // ---- fill_tile_done 触发 drain 启动 ----
            // 注意：如果同拍 drain_tile_done 也发生 (极罕见)，fill_tile_done 重置 drain_active=1 优先生效
            if (fill_tile_done) begin
                drain_active   <= 1'b1;
                rd_addr        <= '0;
                drain_tile_cnt <= fill_tile_cnt;  // 此刻 fill_tile_cnt 还是即将完成的 tile 值
            end
        end
    end

endmodule
