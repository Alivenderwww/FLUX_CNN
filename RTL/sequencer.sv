`timescale 1ns/1ps

// =============================================================================
// sequencer.sv  --  Descriptor Sequencer
//
// 从 desc_fifo pop 一条 descriptor，解析字段，发 start pulse 给核流水 +
// IDMA + ODMA（WDMA 只在 is_first 时发），输出 strip-level cfg 覆盖。
//
// Descriptor 字段布局（little-endian word 0 在低位，见 docs §2.1）：
//   word 0 [3:0]    type           0=NOP 1=CONV 2=BARRIER F=END
//   word 0 [7:4]    flags          {rsvd, streaming_en, is_last, is_first}
//   word 0 [11:8]   pad_top
//   word 0 [15:12]  pad_bot
//   word 0 [19:16]  pad_left
//   word 0 [23:20]  pad_right
//   word 1 [15:0]   strip_y_start
//   word 1 [31:16]  n_yout_strip
//   word 2 [19:0]   ifb_ddr_offset (bytes, 相对 cfg_idma_src_base)
//   word 3 [23:0]   ifb_byte_len
//   word 4 [19:0]   ofb_ddr_offset (bytes, 相对 cfg_odma_dst_base)
//   word 5 [23:0]   ofb_byte_len
//   word 6..7       rsvd
//
// 方案 E：wgt_buffer 只在 flags.is_first 时启动（layer 粒度），后续 strip
// 不再重启，核流水 valid-ready 握手自然对齐。WDMA 类似，只在 is_first 时发
// start_wdma（权重整层一次搬完）。
//
// 复位：FSM + layer_busy + layer_done 控制路径同步复位；descriptor 字段
// latch 数据路径无复位 (§6)。
// =============================================================================

module sequencer (
    input  logic                clk,
    input  logic                rst_n,

    // ---- CTRL ----
    input  logic                start_layer_pulse,
    output logic                layer_busy,
    output logic                layer_done,

    // ---- FIFO read port ----
    input  logic [255:0]        fifo_rd_data,
    input  logic                fifo_empty,
    output logic                fifo_rd_en,

    // ---- Global cfg (layer-level fallback) ----
    input  logic [15:0]         cfg_h_out_total,

    // ---- Strip-level cfg output (给 line_buffer / ofb_writer) ----
    output logic [15:0]         strip_n_yout,
    output logic [3:0]          strip_pad_top,
    output logic [3:0]          strip_pad_bot,
    output logic [3:0]          strip_pad_left,
    output logic [3:0]          strip_pad_right,
    output logic [15:0]         strip_y_start,

    // ---- Per-strip DMA 参数输出（给 core_top 组合成 IDMA/ODMA src_base/len） ----
    output logic [19:0]         strip_ifb_ddr_offset,
    output logic [23:0]         strip_ifb_byte_len,
    output logic [19:0]         strip_ofb_ddr_offset,
    output logic [23:0]         strip_ofb_byte_len,

    // ---- Start pulses ----
    output logic                start_core_pulse,     // line_buffer/ofb_writer (每 strip)
    output logic                start_wgt_pulse,      // wgt_buffer (仅 is_first)
    output logic                start_idma_pulse,     // IDMA (每 strip)
    output logic                start_odma_pulse,     // ODMA (每 strip)
    output logic                start_wdma_pulse,     // WDMA (仅 is_first)

    // ---- Done aggregation ----
    input  logic                core_strip_done,
    input  logic                idma_strip_done,
    input  logic                odma_strip_done,
    input  logic                wdma_done
);

    // =========================================================================
    // Descriptor type 编码
    // =========================================================================
    localparam logic [3:0] TYPE_NOP     = 4'h0;
    localparam logic [3:0] TYPE_CONV    = 4'h1;
    localparam logic [3:0] TYPE_BARRIER = 4'h2;
    localparam logic [3:0] TYPE_END     = 4'hF;

    // =========================================================================
    // FSM (J-2 起单一 streaming 数据路径)
    // =========================================================================
    // 两阶段: PRELOAD (wdma if is_first) → DISPATCH (core+wgt(is_first)+idma+odma 并发)
    //   IDMA ↔ line_buffer 用 rows_available / rows_consumed 行级反压
    //   ODMA ↔ ofb_writer  用 row_done_pulse / rows_drained 行级反压
    typedef enum logic [3:0] {
        S_IDLE         = 4'd0,
        S_FETCH        = 4'd1,
        S_PRELOAD      = 4'd2,    // 发 wdma(is_first), 等完成
        S_DISPATCH     = 4'd3,    // 并发启 core + wgt(is_first) + idma + odma
        S_WAIT         = 4'd4,    // 等 core + idma + odma 全 done
        S_BARRIER      = 4'd5,
        S_END          = 4'd6
    } state_t;
    state_t state, state_next;

    // =========================================================================
    // 解析当前 FIFO 头部 descriptor 的字段（组合）
    // =========================================================================
    logic [3:0]   hd_type;
    logic         hd_is_first;
    logic         hd_is_last;
    // flags[6] FLAG_STREAMING_EN 历史字段; J-2 起恒 1, 硬件不再解析
    logic [3:0]   hd_pad_top, hd_pad_bot, hd_pad_left, hd_pad_right;
    logic [15:0]  hd_strip_y_start;
    logic [15:0]  hd_n_yout_strip;
    logic [19:0]  hd_ifb_ddr_offset;
    logic [23:0]  hd_ifb_byte_len;
    logic [19:0]  hd_ofb_ddr_offset;
    logic [23:0]  hd_ofb_byte_len;

    assign hd_type           = fifo_rd_data[3:0];
    assign hd_is_first       = fifo_rd_data[4];
    assign hd_is_last        = fifo_rd_data[5];
    // bit 6 = FLAG_STREAMING_EN, J-2 后废弃不读
    assign hd_pad_top        = fifo_rd_data[11:8];
    assign hd_pad_bot        = fifo_rd_data[15:12];
    assign hd_pad_left       = fifo_rd_data[19:16];
    assign hd_pad_right      = fifo_rd_data[23:20];
    assign hd_strip_y_start  = fifo_rd_data[47:32];
    assign hd_n_yout_strip   = fifo_rd_data[63:48];
    assign hd_ifb_ddr_offset = fifo_rd_data[83:64];   // word 2 [19:0]
    assign hd_ifb_byte_len   = fifo_rd_data[119:96];  // word 3 [23:0]
    assign hd_ofb_ddr_offset = fifo_rd_data[147:128]; // word 4 [19:0]
    assign hd_ofb_byte_len   = fifo_rd_data[183:160]; // word 5 [23:0]

    // =========================================================================
    // Latched descriptor 字段（FETCH 成功后保存）
    // =========================================================================
    logic [15:0]  r_strip_n_yout;
    logic [3:0]   r_pad_top, r_pad_bot, r_pad_left, r_pad_right;
    logic         r_is_first;
    logic [15:0]  r_strip_y_start;
    logic [19:0]  r_ifb_ddr_offset;
    logic [23:0]  r_ifb_byte_len;
    logic [19:0]  r_ofb_ddr_offset;
    logic [23:0]  r_ofb_byte_len;

    // =========================================================================
    // FIFO pop: FETCH 状态 + !empty
    // =========================================================================
    logic fetch_ok;
    assign fetch_ok   = (state == S_FETCH) && !fifo_empty;
    assign fifo_rd_en = fetch_ok;

    // =========================================================================
    // FSM next-state
    // =========================================================================
    always_comb begin
        state_next = state;
        case (state)
            S_IDLE : if (start_layer_pulse)                 state_next = S_FETCH;
            S_FETCH: if (fetch_ok) begin
                         unique case (hd_type)
                             TYPE_NOP    : state_next = S_FETCH;
                             TYPE_CONV   : state_next = hd_is_first ? S_PRELOAD : S_DISPATCH;
                             TYPE_BARRIER: state_next = S_BARRIER;
                             TYPE_END    : state_next = S_END;
                             default     : state_next = S_FETCH;
                         endcase
                     end
            // PRELOAD 等 wdma (is_first only) 完成
            S_PRELOAD : if (!r_is_first || wdma_done)       state_next = S_DISPATCH;
            S_DISPATCH: state_next = S_WAIT;
            S_WAIT    : if (core_strip_done && idma_strip_done && odma_strip_done)
                            state_next = S_FETCH;
            S_BARRIER : if (core_strip_done && idma_strip_done && odma_strip_done && wdma_done)
                            state_next = S_FETCH;
            S_END     : if (wdma_done && idma_strip_done && odma_strip_done)
                            state_next = S_IDLE;
            default   :     state_next = S_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= state_next;
    end

    // =========================================================================
    // Descriptor latch（FETCH 成功且 type==CONV 时）
    // =========================================================================
    logic latch_conv;
    assign latch_conv = fetch_ok && (hd_type == TYPE_CONV);

    always_ff @(posedge clk) begin
        if (latch_conv) begin
            r_strip_n_yout    <= hd_n_yout_strip;
            r_pad_top         <= hd_pad_top;
            r_pad_bot         <= hd_pad_bot;
            r_pad_left        <= hd_pad_left;
            r_pad_right       <= hd_pad_right;
            r_is_first        <= hd_is_first;
            r_strip_y_start   <= hd_strip_y_start;
            r_ifb_ddr_offset  <= hd_ifb_ddr_offset;
            r_ifb_byte_len    <= hd_ifb_byte_len;
            r_ofb_ddr_offset  <= hd_ofb_ddr_offset;
            r_ofb_byte_len    <= hd_ofb_byte_len;
        end
    end

    // =========================================================================
    // 输出：DISPATCH 发 1 拍 start pulses
    //   核流水 + IDMA + ODMA 每 strip 都发；WDMA + wgt_buffer 仅 is_first
    // =========================================================================
    // PRELOAD 进入的第一拍脉冲（此时 r_* 已经 latch）
    logic preload_entry;
    logic r_prev_state_preload;
    always_ff @(posedge clk) begin
        if (!rst_n) r_prev_state_preload <= 1'b0;
        else        r_prev_state_preload <= (state == S_PRELOAD);
    end
    assign preload_entry = (state == S_PRELOAD) && !r_prev_state_preload;

    assign start_core_pulse = (state == S_DISPATCH);
    assign start_wgt_pulse  = (state == S_DISPATCH) && r_is_first;
    // J-2: IDMA / ODMA 同 DISPATCH 并发启动
    assign start_idma_pulse = (state == S_DISPATCH);
    assign start_odma_pulse = (state == S_DISPATCH);
    // WDMA: is_first 时 PRELOAD 进入启 (同拍 r_is_first 已 latch)
    assign start_wdma_pulse = preload_entry && r_is_first;

    // =========================================================================
    // Strip-level cfg 输出：layer_busy 时用 latch 值；IDLE 时给安全默认
    // =========================================================================
    assign layer_busy = (state != S_IDLE);

    assign strip_n_yout         = layer_busy ? r_strip_n_yout    : cfg_h_out_total;
    assign strip_pad_top        = layer_busy ? r_pad_top         : 4'd0;
    assign strip_pad_bot        = layer_busy ? r_pad_bot         : 4'd0;
    assign strip_pad_left       = layer_busy ? r_pad_left        : 4'd0;
    assign strip_pad_right      = layer_busy ? r_pad_right       : 4'd0;
    assign strip_y_start        = layer_busy ? r_strip_y_start   : 16'd0;
    assign strip_ifb_ddr_offset = r_ifb_ddr_offset;
    assign strip_ifb_byte_len   = r_ifb_byte_len;
    assign strip_ofb_ddr_offset = r_ofb_ddr_offset;
    assign strip_ofb_byte_len   = r_ofb_byte_len;

    // =========================================================================
    // layer_done sticky：S_END → S_IDLE 时 latch 1；start_layer_pulse 清零
    // =========================================================================
    logic r_layer_done;
    always_ff @(posedge clk) begin
        if      (!rst_n)                                      r_layer_done <= 1'b0;
        else if (start_layer_pulse)                           r_layer_done <= 1'b0;
        else if (state == S_END && state_next == S_IDLE)      r_layer_done <= 1'b1;
    end
    assign layer_done = r_layer_done;

endmodule
