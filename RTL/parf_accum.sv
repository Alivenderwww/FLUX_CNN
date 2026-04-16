`timescale 1ns/1ps

// =============================================================================
// parf_accum.sv  --  Partial-Sum Accumulator (Special FIFO)
//
// 替代原 mac_col 内的 PARF，解耦到阵列外部，以 valid-ready 方式串入流水。
//
// 累加次序（由 line_buffer / wgt_buffer 共同保证 psum 抵达顺序）：
//     for cins_cnt in 0..cin_slices-1
//       for kk_cnt   in 0..kk-1          // 全局 kernel 位置；chunked 下跨 round
//         for pos    in 0..valid_w-1     // tile 内 32 个输出列 (写入 wr_addr)
//           一拍 psum_in_vec → parf_data[wr_addr] 累加一次
//
// 首次写入 (cins_cnt==0 && kk_cnt==0) 直接覆盖；其余累加。
// 全部累加完 (cins_cnt, kk_cnt, wr_addr) 都到末尾后，整个 PARF 进入 DRAIN。
// DRAIN 阶段按 rd_addr=0..valid_w-1 对下游 valid；握手成功推进。
// DRAIN 完回到 FILL，等下一 tile。
//
// v1 简化：FILL 与 DRAIN 严格串行，不做 cross-tile overlap（v2 再优化）。
//
// 时序：
//   - psum_in_valid 的那拍即完成组合读 old_val + 加法，posedge 写回 parf_data。
//   - DRAIN 时 acc_out_vec = parf_data[rd_addr]（组合读）。
// =============================================================================

module parf_accum #(
    parameter int NUM_COL    = 16,
    parameter int PSUM_WIDTH = 32,
    parameter int PARF_DEPTH = 32
)(
    input  logic                                 clk,
    input  logic                                 rst_n,

    // ---- cfg (来自 cfg_regs，静态) ----
    input  logic [5:0]                           cfg_tile_w,       // 常规 tile 列数 (通常 32)
    input  logic [5:0]                           cfg_last_valid_w, // 最后一 tile 列数 (可能 < tile_w)
    input  logic [7:0]                           cfg_num_tiles,    // 一行 yout 内的 tile 数
    input  logic [5:0]                           cfg_cin_slices,   // 1..63
    input  logic [9:0]                           cfg_kk,           // K*K，1..256

    // ---- upstream: mac_array psum 流 ----
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
    // 存储：PARF_DEPTH 行，每行 NUM_COL × PSUM_WIDTH（16×32 = 512 bit）
    // DFF 数组 + 组合读写，综合时推断为 LUT-RAM。
    // =========================================================================
    logic signed [NUM_COL*PSUM_WIDTH-1:0] parf_data [0:PARF_DEPTH-1];

    // =========================================================================
    // 状态机
    // =========================================================================
    typedef enum logic [0:0] {
        S_FILL  = 1'b0,
        S_DRAIN = 1'b1
    } state_t;

    state_t state, state_next;

    // 写指针 + 嵌套累加计数（去中心化：自己在本模块算累加是否完成）
    logic [PAW-1:0] wr_addr;
    logic [9:0]     kk_cnt;        // 0..cfg_kk-1
    logic [5:0]     cins_cnt;      // 0..cfg_cin_slices-1

    // tile 索引（仅跟踪行内 tile 位置，用于选 valid_w；yout/cs 不关心）
    logic [7:0]     tile_cnt;      // 0..cfg_num_tiles-1

    // 读指针（DRAIN 时使用）
    logic [PAW-1:0] rd_addr;

    // 当前 tile 的有效列数
    logic [5:0]     cur_valid_w;
    assign cur_valid_w = (tile_cnt == cfg_num_tiles - 8'd1) ? cfg_last_valid_w : cfg_tile_w;

    // =========================================================================
    // 握手
    // =========================================================================
    assign psum_in_ready = (state == S_FILL);
    assign acc_out_valid = (state == S_DRAIN);
    assign acc_out_vec   = parf_data[rd_addr];

    logic fill_fire, drain_fire;
    assign fill_fire  = (state == S_FILL)  && psum_in_valid && psum_in_ready;
    assign drain_fire = (state == S_DRAIN) && acc_out_valid && acc_out_ready;

    // =========================================================================
    // 首轮判定：决定是覆盖写还是累加
    // =========================================================================
    logic is_first_round;
    assign is_first_round = (cins_cnt == 6'd0) && (kk_cnt == 10'd0);

    logic signed [NUM_COL*PSUM_WIDTH-1:0] old_val, new_val;
    assign old_val = parf_data[wr_addr];

    always_comb begin
        for (int c = 0; c < NUM_COL; c++) begin
            logic signed [PSUM_WIDTH-1:0] lhs, rhs;
            lhs = is_first_round ? '0 : old_val[c*PSUM_WIDTH +: PSUM_WIDTH];
            rhs = psum_in_vec[c*PSUM_WIDTH +: PSUM_WIDTH];
            new_val[c*PSUM_WIDTH +: PSUM_WIDTH] = lhs + rhs;
        end
    end

    // =========================================================================
    // 边界信号
    // =========================================================================
    logic wr_is_last_col;      // wr_addr 将走到最后一列
    logic kk_is_last;          // kk_cnt 将走到最后一个 kernel 位
    logic cins_is_last;        // cins_cnt 将走到最后一片
    logic rd_is_last;

    assign wr_is_last_col = (wr_addr  == cur_valid_w     - 6'd1);
    assign kk_is_last     = (kk_cnt   == cfg_kk          - 10'd1);
    assign cins_is_last   = (cins_cnt == cfg_cin_slices  - 6'd1);
    assign rd_is_last     = (rd_addr  == cur_valid_w     - 6'd1);

    logic tile_fill_done;
    assign tile_fill_done = wr_is_last_col && kk_is_last && cins_is_last;

    // =========================================================================
    // 状态转移
    // =========================================================================
    always_comb begin
        state_next = state;
        case (state)
            S_FILL:  if (fill_fire  && tile_fill_done) state_next = S_DRAIN;
            S_DRAIN: if (drain_fire && rd_is_last)     state_next = S_FILL;
            default: state_next = S_FILL;
        endcase
    end

    // =========================================================================
    // 时序更新
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_FILL;
            wr_addr  <= '0;
            kk_cnt   <= '0;
            cins_cnt <= '0;
            tile_cnt <= '0;
            rd_addr  <= '0;
            for (int i = 0; i < PARF_DEPTH; i++) parf_data[i] <= '0;
        end else begin
            state <= state_next;

            // ---- FILL: 累加写回 + 推进嵌套计数 ----
            if (fill_fire) begin
                parf_data[wr_addr] <= new_val;

                if (wr_is_last_col) begin
                    wr_addr <= '0;
                    if (kk_is_last) begin
                        kk_cnt <= '0;
                        if (cins_is_last) cins_cnt <= '0;            // 整 tile 完成，转 DRAIN
                        else              cins_cnt <= cins_cnt + 6'd1;
                    end else begin
                        kk_cnt <= kk_cnt + 10'd1;
                    end
                end else begin
                    wr_addr <= wr_addr + {{(PAW-1){1'b0}}, 1'b1};
                end
            end

            // ---- DRAIN: 推进 rd_addr；tile 完整 drain 后 tile_cnt 环推进 ----
            if (drain_fire) begin
                if (rd_is_last) begin
                    rd_addr  <= '0;
                    tile_cnt <= (tile_cnt == cfg_num_tiles - 8'd1) ? 8'd0 : tile_cnt + 8'd1;
                end else begin
                    rd_addr <= rd_addr + {{(PAW-1){1'b0}}, 1'b1};
                end
            end
        end
    end

endmodule
