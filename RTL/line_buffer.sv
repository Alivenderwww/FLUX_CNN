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

    // ---- IFB 读端口 ----
    output logic                                 ifb_re,
    output logic [$clog2(SRAM_DEPTH)-1:0]        ifb_raddr,
    input  logic [NUM_PE*DATA_WIDTH-1:0]         ifb_rdata,

    // ---- act stream → mac_array ----
    output logic                                 act_valid,
    output logic [NUM_PE*DATA_WIDTH-1:0]         act_vec,
    input  logic                                 act_ready
);

    localparam int AW      = $clog2(SRAM_DEPTH);
    localparam int BUF_AW  = $clog2(ARF_DEPTH);    // 5 bits for ARF_DEPTH=32
    localparam int CNT_W   = $clog2(ARF_DEPTH+1);  // 6 bits to hold 0..32

    // =========================================================================
    // 状态
    // =========================================================================
    typedef enum logic [1:0] {
        S_IDLE = 2'd0,
        S_RUN  = 2'd1,
        S_DONE = 2'd2
    } state_t;
    state_t state, state_next;

    // =========================================================================
    // 外层 6 级 counter (仅 issue 侧)
    // =========================================================================
    logic [3:0]  kx_cnt;
    logic [3:0]  ky_cnt;
    logic [5:0]  cins_cnt;
    logic [7:0]  tile_cnt;
    logic [15:0] yout_cnt;
    logic [5:0]  cs_cnt;

    // =========================================================================
    // 5 层 running base（cs 不影响 IFB）
    // =========================================================================
    logic [ADDR_W-1:0] ptr_yout_base;
    logic [ADDR_W-1:0] ptr_tile_base;
    logic [ADDR_W-1:0] ptr_cins_base;
    logic [ADDR_W-1:0] ptr_ky_base;
    logic [ADDR_W-1:0] ptr_kx_base;

    // =========================================================================
    // Issue 内部 state
    // =========================================================================
    logic [5:0]        iss_pos;       // 0..cur_valid_w-1 (当前 round 内位置)
    logic              issues_all_done;   // 所有 issue 已发出
    logic              ifb_re_d1;     // 上拍 issue，这拍 rdata 到

    // =========================================================================
    // Ring buffer
    // =========================================================================
    logic [BUF_AW-1:0] wr_idx;        // modular 索引 act_buf
    logic [BUF_AW-1:0] rd_idx;
    logic [CNT_W-1:0]  fifo_count;    // 0..ARF_DEPTH

    logic [NUM_PE*DATA_WIDTH-1:0] act_buf [0:ARF_DEPTH-1];

    // =========================================================================
    // cur_valid_w + 边界 (issue 侧)
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

    logic all_is_last;   // 当前 round 是整层最后一个 round
    assign all_is_last = iss_pos_is_last && kx_is_last && ky_is_last &&
                         cins_is_last && tile_is_last && yout_is_last && cs_is_last;

    // =========================================================================
    // 握手
    // =========================================================================
    logic issue_ok;
    // issue 需要：未全发完 AND buffer 有空间（考虑 in-flight 占位）
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

    // Consume 侧
    assign act_valid = (fifo_count > 0);
    assign act_vec   = act_buf[rd_idx];

    logic fire;
    assign fire = act_valid & act_ready;

    logic arrival;
    assign arrival = ifb_re_d1;

    // =========================================================================
    // 状态转移
    // =========================================================================
    always_comb begin
        state_next = state;
        case (state)
            S_IDLE:  if (start)                                          state_next = S_RUN;
            // 所有 issues 已发、fifo 已空、无 in-flight：DONE
            S_RUN:   if (issues_all_done && fifo_count == 0 && !ifb_re_d1) state_next = S_DONE;
            S_DONE:  ;
            default: state_next = S_IDLE;
        endcase
    end

    assign done = (state == S_DONE);

    // =========================================================================
    // counter / ptr 推进 task（iss_pos wrap 时调用）
    // =========================================================================
    task automatic advance_counters_and_ptrs();
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
                                cs_cnt <= cs_cnt + 6'd1;
                                ptr_yout_base <= cfg_ifb_base;
                                ptr_tile_base <= cfg_ifb_base;
                                ptr_cins_base <= cfg_ifb_base;
                                ptr_ky_base   <= cfg_ifb_base;
                                ptr_kx_base   <= cfg_ifb_base;
                            end
                        end else begin
                            yout_cnt <= yout_cnt + 16'd1;
                            ptr_yout_base <= ptr_yout_base + cfg_ifb_row_step;
                            ptr_tile_base <= ptr_yout_base + cfg_ifb_row_step;
                            ptr_cins_base <= ptr_yout_base + cfg_ifb_row_step;
                            ptr_ky_base   <= ptr_yout_base + cfg_ifb_row_step;
                            ptr_kx_base   <= ptr_yout_base + cfg_ifb_row_step;
                        end
                    end else begin
                        tile_cnt <= tile_cnt + 8'd1;
                        ptr_tile_base <= ptr_tile_base + cfg_tile_in_step;
                        ptr_cins_base <= ptr_tile_base + cfg_tile_in_step;
                        ptr_ky_base   <= ptr_tile_base + cfg_tile_in_step;
                        ptr_kx_base   <= ptr_tile_base + cfg_tile_in_step;
                    end
                end else begin
                    cins_cnt <= cins_cnt + 6'd1;
                    ptr_cins_base <= ptr_cins_base + cfg_ifb_cin_step;
                    ptr_ky_base   <= ptr_cins_base + cfg_ifb_cin_step;
                    ptr_kx_base   <= ptr_cins_base + cfg_ifb_cin_step;
                end
            end else begin
                ky_cnt <= ky_cnt + 4'd1;
                ptr_ky_base <= ptr_ky_base + {{(ADDR_W-16){1'b0}}, cfg_w_in};
                ptr_kx_base <= ptr_ky_base + {{(ADDR_W-16){1'b0}}, cfg_w_in};
            end
        end else begin
            kx_cnt <= kx_cnt + 4'd1;
            ptr_kx_base <= ptr_kx_base + {{(ADDR_W-1){1'b0}}, 1'b1};
        end
    endtask

    // =========================================================================
    // 时序
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            kx_cnt          <= '0;
            ky_cnt          <= '0;
            cins_cnt        <= '0;
            tile_cnt        <= '0;
            yout_cnt        <= '0;
            cs_cnt          <= '0;
            iss_pos         <= '0;
            issues_all_done <= 1'b0;
            ifb_re_d1       <= 1'b0;
            wr_idx          <= '0;
            rd_idx          <= '0;
            fifo_count      <= '0;
            ptr_yout_base   <= '0;
            ptr_tile_base   <= '0;
            ptr_cins_base   <= '0;
            ptr_ky_base     <= '0;
            ptr_kx_base     <= '0;
            for (int i = 0; i < ARF_DEPTH; i++) act_buf[i] <= '0;
        end else begin
            state     <= state_next;
            ifb_re_d1 <= issue_ok;

            // ---- IDLE → RUN: 初始化 ----
            if (state == S_IDLE && start) begin
                kx_cnt          <= '0;
                ky_cnt          <= '0;
                cins_cnt        <= '0;
                tile_cnt        <= '0;
                yout_cnt        <= '0;
                cs_cnt          <= '0;
                iss_pos         <= '0;
                issues_all_done <= 1'b0;
                wr_idx          <= '0;
                rd_idx          <= '0;
                fifo_count      <= '0;
                ptr_yout_base   <= cfg_ifb_base;
                ptr_tile_base   <= cfg_ifb_base;
                ptr_cins_base   <= cfg_ifb_base;
                ptr_ky_base     <= cfg_ifb_base;
                ptr_kx_base     <= cfg_ifb_base;
            end

            // ---- Issue 侧: iss_pos 推进 + 外层 counter wrap ----
            if (issue_ok) begin
                if (iss_pos_is_last) begin
                    iss_pos <= '0;
                    if (all_is_last) begin
                        issues_all_done <= 1'b1;
                    end else begin
                        advance_counters_and_ptrs();
                    end
                end else begin
                    iss_pos <= iss_pos + 6'd1;
                end
            end

            // ---- Arrival: 写 act_buf + 推进 wr_idx ----
            if (arrival) begin
                act_buf[wr_idx] <= ifb_rdata;
                wr_idx          <= wr_idx + 1'b1;    // 5-bit 自然 mod 32
            end

            // ---- Fire: 推进 rd_idx ----
            if (fire) begin
                rd_idx <= rd_idx + 1'b1;             // 5-bit 自然 mod 32
            end

            // ---- fifo_count 更新 ----
            // arrival +1, fire -1, both/neither: no change
            case ({arrival, fire})
                2'b10:   fifo_count <= fifo_count + 1'b1;
                2'b01:   fifo_count <= fifo_count - 1'b1;
                default: ;
            endcase
        end
    end

endmodule
