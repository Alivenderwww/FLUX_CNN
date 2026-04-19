`timescale 1ns/1ps

// =============================================================================
// sequencer.sv  --  Descriptor Sequencer (Phase C-1 骨架)
//
// 从 desc_fifo pop 一条 descriptor，解析 type / flags / pad / n_yout_strip
// 等字段，产出 strip-level cfg override + 发 start_core_pulse 给核流水。
//
// Phase C-1 范围：
//   * 只驱动核流水（line_buffer / wgt_buffer / ofb_writer）的 start
//   * IDMA / WDMA / ODMA 的 start 仍由老 CTRL[1..3] 路径驱动（Phase C-5 再切）
//   * 方案 E：wgt_buffer 的 start 只在 flags.is_first 发（layer 粒度）
//   * BARRIER descriptor 在 C-1 不验证；END 触发 layer_done
//
// Descriptor 字段布局（bit offset，little-endian word 0 在低位）：
//   word 0 [3:0]    type         0=NOP 1=CONV 2=BARRIER F=END
//   word 0 [7:4]    flags        {rsvd, streaming_en, is_last, is_first}
//   word 0 [11:8]   pad_top
//   word 0 [15:12]  pad_bot
//   word 0 [19:16]  pad_left
//   word 0 [23:20]  pad_right
//   word 1 [31:16]  strip_y_start
//   word 1 [47:32]  n_yout_strip     (这里 bit offset 用的是整 256-bit 位置)
//   word 2..7       IDMA/ODMA DDR offset & byte_len（C-1 未使用，保留）
//
// Fallback：layer_busy=0 时 strip_n_yout = cfg_h_out_total，pad_*=0，保证老
// CTRL 路径（不经过 Sequencer）的核流水行为完全等价 v3。
//
// 复位：FSM + layer_busy + layer_done sticky 控制路径同步复位；descriptor
// 字段 latch 数据路径无复位。
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

    // ---- Global cfg fallback ----
    input  logic [15:0]         cfg_h_out_total,

    // ---- Strip-level cfg output (给 line_buffer / ofb_writer) ----
    output logic [15:0]         strip_n_yout,
    output logic [3:0]          strip_pad_top,
    output logic [3:0]          strip_pad_bot,
    output logic [3:0]          strip_pad_left,
    output logic [3:0]          strip_pad_right,
    output logic                strip_streaming_en,
    output logic [15:0]         strip_y_start,        // 调试观测

    // ---- Start pulse ----
    output logic                start_core_pulse,     // 给 line_buffer/ofb_writer
    output logic                start_wgt_pulse,      // 给 wgt_buffer (仅 is_first)

    // ---- Done aggregation ----
    input  logic                core_strip_done,
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
    // FSM
    // =========================================================================
    typedef enum logic [2:0] {
        S_IDLE     = 3'd0,
        S_FETCH    = 3'd1,
        S_DISPATCH = 3'd2,
        S_WAIT     = 3'd3,
        S_BARRIER  = 3'd4,
        S_END      = 3'd5
    } state_t;
    state_t state, state_next;

    // =========================================================================
    // 解析当前 FIFO 头部 descriptor 的字段（组合）
    // =========================================================================
    logic [3:0]  hd_type;
    logic        hd_is_first;
    logic        hd_is_last;
    logic        hd_streaming_en;
    logic [3:0]  hd_pad_top, hd_pad_bot, hd_pad_left, hd_pad_right;
    logic [15:0] hd_strip_y_start;
    logic [15:0] hd_n_yout_strip;

    assign hd_type          = fifo_rd_data[3:0];
    assign hd_is_first      = fifo_rd_data[4];
    assign hd_is_last       = fifo_rd_data[5];
    assign hd_streaming_en  = fifo_rd_data[6];
    assign hd_pad_top       = fifo_rd_data[11:8];
    assign hd_pad_bot       = fifo_rd_data[15:12];
    assign hd_pad_left      = fifo_rd_data[19:16];
    assign hd_pad_right     = fifo_rd_data[23:20];
    assign hd_strip_y_start = fifo_rd_data[47:32];
    assign hd_n_yout_strip  = fifo_rd_data[63:48];

    // =========================================================================
    // Latched descriptor 字段（FETCH 成功后保存，DISPATCH/WAIT 期间稳定驱动核流水）
    // =========================================================================
    logic [15:0] r_strip_n_yout;
    logic [3:0]  r_pad_top, r_pad_bot, r_pad_left, r_pad_right;
    logic        r_streaming_en;
    logic        r_is_first;
    logic [15:0] r_strip_y_start;

    // =========================================================================
    // FIFO pop：FETCH 状态 + !empty 时 pop（组合）
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
                             TYPE_NOP    : state_next = S_FETCH;       // 跳过
                             TYPE_CONV   : state_next = S_DISPATCH;
                             TYPE_BARRIER: state_next = S_BARRIER;
                             TYPE_END    : state_next = S_END;
                             default     : state_next = S_FETCH;       // 未知 type 当 NOP
                         endcase
                     end
            S_DISPATCH: state_next = S_WAIT;
            S_WAIT  : if (core_strip_done)                  state_next = S_FETCH;
            S_BARRIER: if (core_strip_done && wdma_done)    state_next = S_FETCH;
            S_END   : if (wdma_done)                        state_next = S_IDLE;
            default :                                       state_next = S_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= state_next;
    end

    // =========================================================================
    // Descriptor latch（FETCH 成功且 type==CONV 时）
    //   NOP / BARRIER / END 不需要 latch strip 字段
    //   数据路径无复位（§6）
    // =========================================================================
    logic latch_conv;
    assign latch_conv = fetch_ok && (hd_type == TYPE_CONV);

    always_ff @(posedge clk) begin
        if (latch_conv) begin
            r_strip_n_yout  <= hd_n_yout_strip;
            r_pad_top       <= hd_pad_top;
            r_pad_bot       <= hd_pad_bot;
            r_pad_left      <= hd_pad_left;
            r_pad_right     <= hd_pad_right;
            r_streaming_en  <= hd_streaming_en;
            r_is_first      <= hd_is_first;
            r_strip_y_start <= hd_strip_y_start;
        end
    end

    // =========================================================================
    // 输出：DISPATCH 发 1 拍 start_core_pulse；is_first 时同拍 start_wgt_pulse
    // =========================================================================
    assign start_core_pulse = (state == S_DISPATCH);
    assign start_wgt_pulse  = (state == S_DISPATCH) && r_is_first;

    // =========================================================================
    // Strip-level cfg 输出：layer_busy 时用 latch 值，否则 fallback 到全局
    //   fallback 保证老 CTRL 路径（不走 Sequencer）的核流水看到 cfg_h_out 原
    //   语义，pad_* = 0
    // =========================================================================
    assign layer_busy  = (state != S_IDLE);

    assign strip_n_yout       = layer_busy ? r_strip_n_yout  : cfg_h_out_total;
    assign strip_pad_top      = layer_busy ? r_pad_top       : 4'd0;
    assign strip_pad_bot      = layer_busy ? r_pad_bot       : 4'd0;
    assign strip_pad_left     = layer_busy ? r_pad_left      : 4'd0;
    assign strip_pad_right    = layer_busy ? r_pad_right     : 4'd0;
    assign strip_streaming_en = layer_busy ? r_streaming_en  : 1'b0;
    assign strip_y_start      = layer_busy ? r_strip_y_start : 16'd0;

    // =========================================================================
    // layer_done sticky：S_END 完成后 latch 1，start_layer_pulse 清零
    // =========================================================================
    logic r_layer_done;
    always_ff @(posedge clk) begin
        if      (!rst_n)                                      r_layer_done <= 1'b0;
        else if (start_layer_pulse)                           r_layer_done <= 1'b0;
        else if (state == S_END && state_next == S_IDLE)      r_layer_done <= 1'b1;
    end
    assign layer_done = r_layer_done;

endmodule
