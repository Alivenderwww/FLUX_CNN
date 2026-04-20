`timescale 1ns/1ps

// =============================================================================
// odma.sv  --  Output DMA Engine (OFB SRAM → DDR, S2MM)
//
// 支持两种模式（由 cfg_odma_streaming 切换）：
//
// Batch 模式 (cfg_odma_streaming=0)：v1 行为
//   按 descriptor (dst_base, byte_len) 一次性把内部 OFB SRAM 的连续内容搬到外部
//   DDR。rd_ptr 从 0 线性递增，AW 按 256 beat 上限切 burst。
//
// Streaming 模式 (cfg_odma_streaming=1)：v2 row-ring
//   OFB 作 row-level ring buffer。ODMA 每行 pacing：
//     * ofb_writer 每写完 1 行 OFM → row_done_pulse → rows_produced++
//     * 当 rows_produced > rows_drained 才进 S_AW；否则停 S_AW_WAIT
//     * 每行发 ceil(cfg_w_out / 256) 条 burst；AW 地址按 cfg_ddr_ofm_row_stride
//       跨行，burst 内按 beats×16 推进
//     * rd_ptr 在 cfg_ofb_ring_words (= strip_rows * W_OUT) 处 wrap
//     * 每行最后一个 burst 的 b_fire 触发 rows_drained++（用于反压 ofb_writer）
//     * 整体停止条件：rows_drained == cfg_h_out_total
//   Streaming 前提 cout_slices = 1 (Cout ≤ 16)；cs > 1 留 v3
//
// 状态机（共享 PREFETCH/W/B 路径）：
//   S_IDLE → start → streaming ? S_AW_WAIT : S_AW
//   S_AW_WAIT → has_work → S_AW
//   S_AW → aw_fire → S_PREFETCH → (1 cycle) → S_W
//   S_W → w_last_fire → S_B
//   S_B → b_fire：
//       streaming_all_done          → S_DONE
//       batch_last_beat             → S_DONE
//       streaming_row_complete_b    → S_AW_WAIT  (等下一行 or 检查 has_work)
//       else                        → S_AW       (同行更多 burst / 下一 batch burst)
//
// SRAM 读时序（sram_model 1 拍延迟）：
//   S_PREFETCH 拉 re=1、raddr=rd_ptr；下拍 rdata=mem[rd_ptr] 到位进入 S_W。
//   S_W 每拍 re=1、raddr=rd_ptr + (w_fire ? 1 : 0)；streaming 下 raddr 需按
//   cfg_ofb_ring_words wrap。
//
// 复位：state / r_done / r_rows_produced / r_rows_drained 同步复位；数据路径
// 靠 start 初始化（§6）。
// =============================================================================

module odma #(
    parameter int ADDR_W       = 32,
    parameter int DATA_W       = 128,
    parameter int M_ID         = 2,
    parameter int SRAM_ADDR_W  = 13,
    parameter int LEN_W        = 24
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // ---- Control ----
    input  logic                    start,
    output logic                    done,
    output logic                    busy,

    // ---- Descriptor (batch 用) ----
    input  logic [ADDR_W-1:0]       dst_base,
    input  logic [LEN_W-1:0]        byte_len,

    // ---- Streaming 配置 ----
    input  logic                    cfg_odma_streaming,
    input  logic [15:0]             cfg_h_out_total,         // 整图 OFM 行数
    input  logic [15:0]             cfg_w_out,               // 每行 OFM 像素数
    input  logic [5:0]              cfg_cout_slices,         // NHWC: 每像素的 cs 段数
    input  logic [19:0]             cfg_ofb_row_words,       // W_OUT × cout_slices (每 yout beat 数)
    input  logic [ADDR_W-1:0]       cfg_ddr_ofm_row_stride,  // DDR 跨行字节步长
    input  logic [19:0]             cfg_ofb_ring_words,       // OFB ring wrap 模数（words）
    input  logic                    row_done_pulse,          // 来自 ofb_writer
    output logic [15:0]             rows_drained,            // 给 ofb_writer 做反压

    // ---- AXI4 M (AR/R 内部 tie 0) ----
    output logic [M_ID-1:0]         M_AWID,
    output logic [ADDR_W-1:0]       M_AWADDR,
    output logic [7:0]              M_AWLEN,
    output logic [1:0]              M_AWBURST,
    output logic                    M_AWVALID,
    input  logic                    M_AWREADY,

    output logic [DATA_W-1:0]       M_WDATA,
    output logic [DATA_W/8-1:0]     M_WSTRB,
    output logic                    M_WLAST,
    output logic                    M_WVALID,
    input  logic                    M_WREADY,

    input  logic [M_ID-1:0]         M_BID,
    input  logic [1:0]              M_BRESP,
    input  logic                    M_BVALID,
    output logic                    M_BREADY,

    output logic [M_ID-1:0]         M_ARID,
    output logic [ADDR_W-1:0]       M_ARADDR,
    output logic [7:0]              M_ARLEN,
    output logic [1:0]              M_ARBURST,
    output logic                    M_ARVALID,
    input  logic                    M_ARREADY,

    input  logic [M_ID-1:0]         M_RID,
    input  logic [DATA_W-1:0]       M_RDATA,
    input  logic [1:0]              M_RRESP,
    input  logic                    M_RLAST,
    input  logic                    M_RVALID,
    output logic                    M_RREADY,

    // ---- OFB SRAM read port ----
    output logic                    ofb_re,
    output logic [SRAM_ADDR_W-1:0]  ofb_raddr,
    input  logic [DATA_W-1:0]       ofb_rdata
);

    localparam int BYTES_PER_BEAT = DATA_W / 8;
    localparam int BEAT_SHIFT     = $clog2(BYTES_PER_BEAT);
    localparam int MAX_BEATS      = 256;

    // =========================================================================
    // 读通道 tie 0（ODMA 只写）
    // =========================================================================
    assign M_ARID    = '0;
    assign M_ARADDR  = '0;
    assign M_ARLEN   = '0;
    assign M_ARBURST = 2'b01;
    assign M_ARVALID = 1'b0;
    assign M_RREADY  = 1'b1;

    assign M_AWID    = '0;
    assign M_AWBURST = 2'b01;     // INCR

    // =========================================================================
    // 状态机（加 S_AW_WAIT for streaming）
    // =========================================================================
    typedef enum logic [2:0] {
        S_IDLE     = 3'd0,
        S_AW_WAIT  = 3'd1,   // streaming: 等 row_done_pulse 攒够 1 行再发 AW
        S_AW       = 3'd2,
        S_PREFETCH = 3'd3,
        S_W        = 3'd4,
        S_B        = 3'd5,
        S_DONE     = 3'd6
    } state_t;
    state_t state, state_next;

    // =========================================================================
    // 寄存器
    // =========================================================================
    logic [ADDR_W-1:0]       cur_addr;            // AXI 起始字节地址（随 burst 推进）
    logic [ADDR_W-1:0]       row_base_addr;       // streaming: 当前行 DDR base
    logic [LEN_W-1:0]        beats_remaining;     // batch 剩余 beats
    logic [15:0]             row_beats_left;      // streaming: 当前行剩余 beats
    logic [15:0]             r_rows_produced;     // streaming: 已攒够可 drain 的行
    logic [15:0]             r_rows_drained;      // streaming: 已 drain 完成的行
    logic [SRAM_ADDR_W-1:0]  rd_ptr;
    // NHWC gather 侧：2 个 counter + yout_base 拼出 rd_ptr
    logic [15:0]             x_rd_cnt;            // 0..W_OUT-1
    logic [5:0]              cs_rd_cnt;           // 0..cout_slices-1
    logic [SRAM_ADDR_W-1:0]  yout_base;           // 本 yout 在 OFB 的基址 (streaming 下 wrap)
    logic [8:0]              burst_beats;
    logic [8:0]              burst_sent;
    logic                    r_done;

    assign rows_drained = r_rows_drained;

    // =========================================================================
    // 派生量 / 命名事件
    // =========================================================================
    logic aw_fire, w_fire, w_last_fire, b_fire;
    assign aw_fire     = M_AWVALID && M_AWREADY;
    assign w_fire      = M_WVALID  && M_WREADY;
    assign w_last_fire = w_fire    && M_WLAST;
    assign b_fire      = M_BVALID  && M_BREADY;

    // beats_to_issue：batch 用 beats_remaining，streaming 用 row_beats_left
    logic [8:0] beats_to_issue;
    always_comb begin
        if (cfg_odma_streaming) begin
            if (row_beats_left >= 16'd256)  beats_to_issue = 9'd256;
            else                             beats_to_issue = {1'b0, row_beats_left[7:0]};
        end else begin
            if (beats_remaining >= LEN_W'(MAX_BEATS)) beats_to_issue = 9'd256;
            else                                       beats_to_issue = {1'b0, beats_remaining[7:0]};
        end
    end

    // Streaming: 当前 b_fire 是一整行最后一个 burst（row_beats_left 已经 0）
    logic streaming_row_complete_b;
    assign streaming_row_complete_b = cfg_odma_streaming && b_fire && (row_beats_left == 16'd0);

    // Streaming: 全图 drain 完
    logic streaming_all_done;
    assign streaming_all_done = streaming_row_complete_b &&
                                (r_rows_drained == cfg_h_out_total - 16'd1);

    // Streaming: 有活干（至少攒了一整行没 drain）
    logic streaming_has_work;
    assign streaming_has_work = cfg_odma_streaming && (r_rows_produced != r_rows_drained);

    // Batch: 最后 beat 的 b_fire
    logic batch_last_beat;
    assign batch_last_beat = !cfg_odma_streaming && b_fire && (beats_remaining == LEN_W'(0));

    // rd_ptr NHWC gather: 下一拍要读的 OFB 地址 = yout_base + cs_rd × W_OUT + x_rd
    //   按 "for x: for cs: fire" 的顺序扫描
    // cs_offset 累加器避免组合乘法
    logic [SRAM_ADDR_W-1:0] cs_offset;   // cs_rd_cnt × cfg_w_out (累加器推进)

    logic cs_rd_is_last;
    logic x_rd_is_last;
    assign cs_rd_is_last = (cs_rd_cnt == cfg_cout_slices - 6'd1);
    assign x_rd_is_last  = (x_rd_cnt  == cfg_w_out        - 16'd1);

    // yout 切换条件：一个 yout 扫完 (x=W_OUT-1 && cs=cout_slices-1)
    logic yout_rd_done;
    assign yout_rd_done = cs_rd_is_last && x_rd_is_last;

    // 下一拍 yout_base (streaming: wrap at ring_words; batch: 线性)
    logic [SRAM_ADDR_W-1:0] yb_plus;
    logic [SRAM_ADDR_W-1:0] yout_base_next;
    assign yb_plus = yout_base + cfg_ofb_row_words[SRAM_ADDR_W-1:0];
    always_comb begin
        if (cfg_odma_streaming && (yb_plus >= cfg_ofb_ring_words[SRAM_ADDR_W-1:0]))
            yout_base_next = yb_plus - cfg_ofb_ring_words[SRAM_ADDR_W-1:0];
        else
            yout_base_next = yb_plus;
    end

    // rd_ptr 当前值 (= yout_base + cs_offset + x_rd_cnt)
    // rd_ptr 保留作寄存器供 ofb_raddr 用；每 w_fire 跳到下一 beat 的地址
    logic [SRAM_ADDR_W-1:0] rd_ptr_next;
    always_comb begin
        if (yout_rd_done)       rd_ptr_next = yout_base_next;              // 新 yout 起点
        else if (cs_rd_is_last) rd_ptr_next = yout_base + (x_rd_cnt + 16'd1); // cs wrap, x++
        else                    rd_ptr_next = rd_ptr + cfg_w_out[SRAM_ADDR_W-1:0]; // cs++ (same x)
    end

    // =========================================================================
    // 端口输出
    // =========================================================================
    assign M_AWADDR = cur_addr;
    assign M_AWLEN  = beats_to_issue[7:0] - 8'd1;
    assign M_AWVALID= (state == S_AW);

    assign M_WDATA  = ofb_rdata;
    assign M_WSTRB  = '1;
    assign M_WVALID = (state == S_W);
    assign M_WLAST  = (state == S_W) && (burst_sent == (burst_beats - 9'd1));

    assign M_BREADY = (state == S_B);

    assign ofb_re    = (state == S_PREFETCH) || (state == S_W);
    assign ofb_raddr = (state == S_W && w_fire) ? rd_ptr_next : rd_ptr;

    assign done = r_done;
    assign busy = (state != S_IDLE) && (state != S_DONE);

    // =========================================================================
    // FSM 三段式
    // =========================================================================
    always_comb begin
        state_next = state;
        case (state)
            S_IDLE    : if (start) state_next = cfg_odma_streaming ? S_AW_WAIT : S_AW;
            S_AW_WAIT : if (streaming_has_work)    state_next = S_AW;
            S_AW      : if (aw_fire)               state_next = S_PREFETCH;
            S_PREFETCH:                             state_next = S_W;
            S_W       : if (w_last_fire)           state_next = S_B;
            S_B       : if (b_fire) begin
                          if      (streaming_all_done)        state_next = S_DONE;
                          else if (batch_last_beat)           state_next = S_DONE;
                          else if (streaming_row_complete_b)  state_next = S_AW_WAIT;
                          else                                state_next = S_AW;
                        end
            S_DONE    : if (start) state_next = cfg_odma_streaming ? S_AW_WAIT : S_AW;
            default   :                            state_next = S_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= state_next;
    end

    // =========================================================================
    // 数据路径寄存器
    // =========================================================================
    // cur_addr: streaming 跨行时跳到 row_base_next；batch/同行用 aw_fire+burst_bytes
    always_ff @(posedge clk) begin
        if      (start)                     cur_addr <= dst_base;
        else if (streaming_row_complete_b)  cur_addr <= row_base_addr + cfg_ddr_ofm_row_stride;
        else if (aw_fire)                   cur_addr <= cur_addr + (ADDR_W'(beats_to_issue) << BEAT_SHIFT);
        else                                cur_addr <= cur_addr;
    end

    // row_base_addr: streaming only，每 drain 完 1 行 += row_stride
    always_ff @(posedge clk) begin
        if      (start)                     row_base_addr <= dst_base;
        else if (streaming_row_complete_b)  row_base_addr <= row_base_addr + cfg_ddr_ofm_row_stride;
        else                                row_base_addr <= row_base_addr;
    end

    // beats_remaining: batch 用；streaming 下不参与 done 判定，但保留递减逻辑
    always_ff @(posedge clk) begin
        if      (start)  beats_remaining <= byte_len >> BEAT_SHIFT;
        else if (w_fire) beats_remaining <= beats_remaining - 1;
        else             beats_remaining <= beats_remaining;
    end

    // row_beats_left: 方式 1 下每 yout = cfg_w_out × cout_slices = cfg_ofb_row_words
    always_ff @(posedge clk) begin
        if      (start)                     row_beats_left <= cfg_ofb_row_words[15:0];
        else if (streaming_row_complete_b)  row_beats_left <= cfg_ofb_row_words[15:0];
        else if (w_fire)                    row_beats_left <= row_beats_left - 16'd1;
        else                                row_beats_left <= row_beats_left;
    end

    // rd_ptr NHWC gather 推进：按 (yout, x, cs) 顺序扫描
    always_ff @(posedge clk) begin
        if      (start)  rd_ptr <= '0;
        else if (w_fire) rd_ptr <= rd_ptr_next;
        else             rd_ptr <= rd_ptr;
    end

    // x_rd_cnt / cs_rd_cnt / cs_offset / yout_base 推进（gather 逻辑）
    always_ff @(posedge clk) begin
        if      (start)          cs_rd_cnt <= '0;
        else if (w_fire) begin
            if (cs_rd_is_last)   cs_rd_cnt <= '0;
            else                 cs_rd_cnt <= cs_rd_cnt + 6'd1;
        end
    end

    always_ff @(posedge clk) begin
        if      (start)          x_rd_cnt <= '0;
        else if (w_fire && cs_rd_is_last) begin
            if (x_rd_is_last)    x_rd_cnt <= '0;
            else                 x_rd_cnt <= x_rd_cnt + 16'd1;
        end
    end

    always_ff @(posedge clk) begin
        if      (start)          cs_offset <= '0;
        else if (w_fire) begin
            if (cs_rd_is_last)   cs_offset <= '0;
            else                 cs_offset <= cs_offset + cfg_w_out[SRAM_ADDR_W-1:0];
        end
    end

    always_ff @(posedge clk) begin
        if      (start)                    yout_base <= '0;
        else if (w_fire && yout_rd_done)   yout_base <= yout_base_next;
    end

    always_ff @(posedge clk) begin
        if      (aw_fire) burst_beats <= beats_to_issue;
        else              burst_beats <= burst_beats;
    end

    always_ff @(posedge clk) begin
        if      (aw_fire) burst_sent <= '0;
        else if (w_fire)  burst_sent <= burst_sent + 1;
        else              burst_sent <= burst_sent;
    end

    // =========================================================================
    // 控制路径寄存器（同步复位）
    // =========================================================================
    // r_rows_produced: 每收到 1 拍 row_done_pulse +1
    always_ff @(posedge clk) begin
        if      (!rst_n)          r_rows_produced <= 16'd0;
        else if (start)           r_rows_produced <= 16'd0;
        else if (row_done_pulse)  r_rows_produced <= r_rows_produced + 16'd1;
        else                      r_rows_produced <= r_rows_produced;
    end

    // r_rows_drained: 每 streaming_row_complete_b +1
    always_ff @(posedge clk) begin
        if      (!rst_n)                     r_rows_drained <= 16'd0;
        else if (start)                      r_rows_drained <= 16'd0;
        else if (streaming_row_complete_b)   r_rows_drained <= r_rows_drained + 16'd1;
        else                                 r_rows_drained <= r_rows_drained;
    end

    // r_done 锁存
    always_ff @(posedge clk) begin
        if      (!rst_n)              r_done <= 1'b0;
        else if (start)               r_done <= 1'b0;
        else if (batch_last_beat)     r_done <= 1'b1;
        else if (streaming_all_done)  r_done <= 1'b1;
        else                          r_done <= r_done;
    end

endmodule
