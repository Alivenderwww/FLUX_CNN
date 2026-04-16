`timescale 1ns/1ps

// =============================================================================
// line_buffer.sv  --  Activation Stream Feeder (IFB → ARF → MAC)
//
// 接替原架构中 core_ctrl 产生的 IFB 读地址 + ARF 写/读 + 激活驱动的所有职责，
// 自带 6 层循环推进。输出 act_vec 以 valid-ready 握手驱动 mac_array。
//
// 6 层嵌套循环（最外到最内）：
//     for cs   in 0..cout_slices-1       (cs 对 IFB 读无影响，但整个 IFB 要重跑一遍)
//       for yout in 0..h_out-1
//         for tile in 0..num_tiles-1
//           for cins in 0..cin_slices-1
//             for ky in 0..K-1
//               for kx in 0..K-1
//                 [LOAD]   cur_valid_w 拍：发 IFB 读 → 1 拍后回填 act_buf
//                 [STREAM] cur_valid_w 拍：act_buf[idx] → mac_array (valid-ready)
//
// v1 简化：LOAD/STREAM 严格串行；不做 stride=1 滑窗复用（kx=0..K-1 都完整读）。
// 即一组 (ky,kx) 的 LOAD 耗 cur_valid_w+1 拍、STREAM 耗 cur_valid_w 拍。
// v2 再上 overlap 和滑窗复用。
//
// 地址运算：维护 5 层 running base（yout / tile / cins / ky / kx），
// 层推进时只做加法，零乘法器。IFB 内 x 偏移 rd_ifb_cnt*stride 是小乘法
// （stride∈{1,2} 综合时退化为 shift/wire），可接受。
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
    // 状态机
    // =========================================================================
    typedef enum logic [1:0] {
        S_IDLE   = 2'd0,
        S_LOAD   = 2'd1,   // IFB 读 + 写 act_buf
        S_STREAM = 2'd2,   // act_buf → mac
        S_DONE   = 2'd3
    } state_t;
    state_t state, state_next;

    // =========================================================================
    // 6 层计数
    // =========================================================================
    logic [3:0]  kx_cnt;
    logic [3:0]  ky_cnt;
    logic [5:0]  cins_cnt;
    logic [7:0]  tile_cnt;
    logic [15:0] yout_cnt;
    logic [5:0]  cs_cnt;

    // =========================================================================
    // 5 层运行基址（cs 层 = cfg_ifb_base 常量，省略）
    // =========================================================================
    logic [ADDR_W-1:0] ptr_yout_base;
    logic [ADDR_W-1:0] ptr_tile_base;
    logic [ADDR_W-1:0] ptr_cins_base;
    logic [ADDR_W-1:0] ptr_ky_base;
    logic [ADDR_W-1:0] ptr_kx_base;    // 当前 (cs,yout,tile,cins,ky,kx) 的 IFB 起点

    // =========================================================================
    // LOAD / STREAM 内部计数 + buffer
    // =========================================================================
    logic [5:0] rd_ifb_cnt;     // 已发出的 IFB 读数
    logic [5:0] wr_buf_cnt;     // 已写入 act_buf 的数（= rd_ifb_cnt 延 1 拍）
    logic       ifb_re_d1;      // ifb_re 打 1 拍，用于判定 ifb_rdata 有效
    logic [5:0] stream_idx;

    logic [NUM_PE*DATA_WIDTH-1:0] act_buf [0:ARF_DEPTH-1];

    // =========================================================================
    // 当前 tile 的有效列数
    // =========================================================================
    logic [5:0] cur_valid_w;
    assign cur_valid_w = (tile_cnt == cfg_num_tiles - 8'd1) ? cfg_last_valid_w : cfg_tile_w;

    // =========================================================================
    // 边界
    // =========================================================================
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
    // IFB 地址
    // =========================================================================
    logic [ADDR_W-1:0] ifb_rd_offset;
    always_comb begin
        // rd_ifb_cnt × stride，stride 实际只会是 1 或 2
        case (cfg_stride)
            3'd1:    ifb_rd_offset = {{(ADDR_W-6){1'b0}}, rd_ifb_cnt};
            3'd2:    ifb_rd_offset = {{(ADDR_W-7){1'b0}}, rd_ifb_cnt, 1'b0};
            default: ifb_rd_offset = {{(ADDR_W-6){1'b0}}, rd_ifb_cnt}; // 兜底
        endcase
    end

    logic ifb_re_gate;
    assign ifb_re_gate = (state == S_LOAD) && (rd_ifb_cnt < cur_valid_w);
    assign ifb_re      = ifb_re_gate;
    assign ifb_raddr   = (ptr_kx_base + ifb_rd_offset);

    // =========================================================================
    // act stream
    // =========================================================================
    assign act_valid = (state == S_STREAM);
    assign act_vec   = act_buf[stream_idx];

    logic stream_fire;
    assign stream_fire = (state == S_STREAM) && act_valid && act_ready;

    logic load_complete;
    assign load_complete = (wr_buf_cnt == cur_valid_w);

    logic stream_last;
    assign stream_last = (stream_idx == cur_valid_w - 6'd1);

    // =========================================================================
    // 状态转移
    // =========================================================================
    always_comb begin
        state_next = state;
        case (state)
            S_IDLE:   if (start)                             state_next = S_LOAD;
            S_LOAD:   if (load_complete)                     state_next = S_STREAM;
            S_STREAM: if (stream_fire && stream_last)
                         state_next = all_last ? S_DONE : S_LOAD;
            S_DONE:  ;
            default: state_next = S_IDLE;
        endcase
    end

    assign done = (state == S_DONE);

    // =========================================================================
    // counter / ptr 推进（在 stream_last 那拍做）
    // 嵌套顺序：kx → ky → cins → tile → yout → cs
    // =========================================================================
    task automatic advance_counters_and_ptrs();
        // 此任务在 always_ff 里调用，产生 non-blocking 赋值
        // 注意：外层边界全部进位时，inner counters 归零、更外层推进、对应 ptr base 归位/递增
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
                                // cs 推进：IFB 从头再跑一遍
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
            state         <= S_IDLE;
            kx_cnt        <= '0;
            ky_cnt        <= '0;
            cins_cnt      <= '0;
            tile_cnt      <= '0;
            yout_cnt      <= '0;
            cs_cnt        <= '0;
            rd_ifb_cnt    <= '0;
            wr_buf_cnt    <= '0;
            stream_idx    <= '0;
            ifb_re_d1     <= 1'b0;
            ptr_yout_base <= '0;
            ptr_tile_base <= '0;
            ptr_cins_base <= '0;
            ptr_ky_base   <= '0;
            ptr_kx_base   <= '0;
            for (int i = 0; i < ARF_DEPTH; i++) act_buf[i] <= '0;
        end else begin
            state     <= state_next;
            ifb_re_d1 <= ifb_re;

            // ---- IDLE: 等 start，初始化 ----
            if (state == S_IDLE && start) begin
                kx_cnt        <= '0;
                ky_cnt        <= '0;
                cins_cnt      <= '0;
                tile_cnt      <= '0;
                yout_cnt      <= '0;
                cs_cnt        <= '0;
                rd_ifb_cnt    <= '0;
                wr_buf_cnt    <= '0;
                stream_idx    <= '0;
                ptr_yout_base <= cfg_ifb_base;
                ptr_tile_base <= cfg_ifb_base;
                ptr_cins_base <= cfg_ifb_base;
                ptr_ky_base   <= cfg_ifb_base;
                ptr_kx_base   <= cfg_ifb_base;
            end

            // ---- LOAD: 发读 + 回填 ----
            if (state == S_LOAD) begin
                if (ifb_re_gate) begin
                    rd_ifb_cnt <= rd_ifb_cnt + 6'd1;
                end
                if (ifb_re_d1) begin
                    act_buf[wr_buf_cnt] <= ifb_rdata;
                    wr_buf_cnt <= wr_buf_cnt + 6'd1;
                end
            end

            // ---- STREAM: act_buf 流出 ----
            if (state == S_STREAM && stream_fire) begin
                if (stream_last) begin
                    // 本轮 (ky,kx) 结束，推进外层嵌套并重置内部计数
                    stream_idx <= '0;
                    rd_ifb_cnt <= '0;
                    wr_buf_cnt <= '0;
                    advance_counters_and_ptrs();
                end else begin
                    stream_idx <= stream_idx + 6'd1;
                end
            end
        end
    end

endmodule
