`timescale 1ns/1ps

// =============================================================================
// line_buffer.sv  --  Activation Stream Feeder (IFB → act_buf → MAC)
//
// v2 优化：同一 (ky,kx,cins,tile,yout,cs) round 内 **LOAD/STREAM overlap**。
// act_buf 保留为 32-entry ring buffer，写指针 wr_ptr 和读指针 rd_ptr 分离：
//   - IFB 读每拍都可以发（wr_ptr + in_flight < cur_valid_w）
//   - mac 读每拍都可以发（rd_ptr < wr_ptr）
// 同一 round 内 LOAD 和 STREAM 天然 overlap；每 round 从 v1 的 2N+1 拍降到 N+2。
//
// 6 层嵌套循环（和 v1 相同）：for cs / yout / tile / cins / ky / kx
// round 边界：rd_ptr 达到 cur_valid_w → 外层 counter 推进；wr/rd 指针归零。
//
// 握手：
//   - ifb_re 由 issue_ok 闸门，不依赖 mac fire（解耦读/流）
//   - act_valid = (rd_ptr < wr_ptr)：有数据就流出
//   - mac.act_ready 控制 fire 和 rd_ptr 推进（与 ifb 读节奏解耦）
//
// 地址运算：5 层 running base（yout/tile/cins/ky/kx），推进用纯加法。
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

    localparam int AW = $clog2(SRAM_DEPTH);

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
    // 6 层外层 counter
    // =========================================================================
    logic [3:0]  kx_cnt;
    logic [3:0]  ky_cnt;
    logic [5:0]  cins_cnt;
    logic [7:0]  tile_cnt;
    logic [15:0] yout_cnt;
    logic [5:0]  cs_cnt;

    // =========================================================================
    // 5 层 running base (cs 不影响 IFB)
    // =========================================================================
    logic [ADDR_W-1:0] ptr_yout_base;
    logic [ADDR_W-1:0] ptr_tile_base;
    logic [ADDR_W-1:0] ptr_cins_base;
    logic [ADDR_W-1:0] ptr_ky_base;
    logic [ADDR_W-1:0] ptr_kx_base;

    // =========================================================================
    // Ring-buffer 内部 ptr
    // =========================================================================
    logic [5:0] wr_ptr;          // 下一个要写入 act_buf 的位置 (0..cur_valid_w)
    logic [5:0] rd_ptr;          // 下一个要读给 mac 的位置    (0..cur_valid_w)
    logic       ifb_re_d1;       // 上拍发了 ifb_re，这拍数据到

    logic [NUM_PE*DATA_WIDTH-1:0] act_buf [0:ARF_DEPTH-1];

    // =========================================================================
    // cur_valid_w + 边界
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

    logic all_last;
    assign all_last = kx_is_last && ky_is_last && cins_is_last && tile_is_last && yout_is_last && cs_is_last;

    // =========================================================================
    // Stream 握手
    // =========================================================================
    assign act_valid = (state == S_RUN) && (rd_ptr < wr_ptr);
    assign act_vec   = act_buf[rd_ptr];

    logic fire;
    assign fire = act_valid & act_ready;

    // round_done: 本拍 fire 刚好把最后一个位置消费完
    logic round_done;
    assign round_done = fire && (rd_ptr == cur_valid_w - 6'd1);

    // =========================================================================
    // IFB 读 (issue)
    // =========================================================================
    // 下一次要发读的 round 内偏移 = wr_ptr + ifb_re_d1
    logic [5:0] issue_offset;
    assign issue_offset = wr_ptr + {5'd0, ifb_re_d1};

    logic issue_ok;
    assign issue_ok = (state == S_RUN) && !round_done && (issue_offset < cur_valid_w);

    logic [ADDR_W-1:0] ifb_rd_offset;
    always_comb begin
        case (cfg_stride)
            3'd1:    ifb_rd_offset = {{(ADDR_W-6){1'b0}}, issue_offset};
            3'd2:    ifb_rd_offset = {{(ADDR_W-7){1'b0}}, issue_offset, 1'b0};
            default: ifb_rd_offset = {{(ADDR_W-6){1'b0}}, issue_offset};
        endcase
    end

    assign ifb_re    = issue_ok;
    assign ifb_raddr = ptr_kx_base + ifb_rd_offset;

    // =========================================================================
    // 状态转移
    // =========================================================================
    always_comb begin
        state_next = state;
        case (state)
            S_IDLE:  if (start)                         state_next = S_RUN;
            S_RUN:   if (round_done && all_last)        state_next = S_DONE;
            S_DONE:  ;
            default: state_next = S_IDLE;
        endcase
    end

    assign done = (state == S_DONE);

    // =========================================================================
    // counter / ptr 推进 task (round_done 时调用)
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
    // 主时序
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            kx_cnt        <= '0;
            ky_cnt        <= '0;
            cins_cnt      <= '0;
            tile_cnt      <= '0;
            yout_cnt      <= '0;
            cs_cnt        <= '0;
            wr_ptr        <= '0;
            rd_ptr        <= '0;
            ifb_re_d1     <= 1'b0;
            ptr_yout_base <= '0;
            ptr_tile_base <= '0;
            ptr_cins_base <= '0;
            ptr_ky_base   <= '0;
            ptr_kx_base   <= '0;
            for (int i = 0; i < ARF_DEPTH; i++) act_buf[i] <= '0;
        end else begin
            state     <= state_next;
            ifb_re_d1 <= issue_ok;

            // ---- IDLE: 等 start ----
            if (state == S_IDLE && start) begin
                kx_cnt        <= '0;
                ky_cnt        <= '0;
                cins_cnt      <= '0;
                tile_cnt      <= '0;
                yout_cnt      <= '0;
                cs_cnt        <= '0;
                wr_ptr        <= '0;
                rd_ptr        <= '0;
                ptr_yout_base <= cfg_ifb_base;
                ptr_tile_base <= cfg_ifb_base;
                ptr_cins_base <= cfg_ifb_base;
                ptr_ky_base   <= cfg_ifb_base;
                ptr_kx_base   <= cfg_ifb_base;
            end

            // ---- RUN: 处理 arrival / fire / round_done ----
            if (state == S_RUN) begin
                // Arrival: 上拍发过 re，这拍数据到 → 写 act_buf[wr_ptr]，wr_ptr++
                if (ifb_re_d1) begin
                    act_buf[wr_ptr] <= ifb_rdata;
                    wr_ptr          <= wr_ptr + 6'd1;
                end

                // Fire: mac 消费 act_buf[rd_ptr]，rd_ptr++
                if (fire) begin
                    rd_ptr <= rd_ptr + 6'd1;
                end

                // round_done 覆盖：重置 ring 指针 + 推进外层 counter
                if (round_done) begin
                    wr_ptr    <= '0;
                    rd_ptr    <= '0;
                    ifb_re_d1 <= 1'b0;   // 覆盖前面的 ifb_re_d1 <= issue_ok
                    advance_counters_and_ptrs();
                end
            end
        end
    end

endmodule
