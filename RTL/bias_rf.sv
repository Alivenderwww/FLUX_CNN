`timescale 1ns/1ps

// =============================================================================
// bias_rf.sv  --  Bias Register File for SDP
//
// 替换原 wgt_buffer 内 bias RF (32 cs × 16 col × INT32 编译期分配, 浪费 + 限
// MAX_COUT_SLICES=32). 新设计:
//   - 本模块只持有"当前 cs"的 16 个 INT32 bias 值 (= 16 × 32 = 512 bit FF)
//   - bias 的"完整存储"放在 Shortcut Bank SRAM (128b 字宽, 1 个 cs 占 4 个字)
//   - 跨 cs 时 (cs_cnt 变化) 触发 4 拍 SRAM 读 prefetch, 把新 cs 的 16 个 INT32
//     拉进本地 RF, 期间 bias_ready=0 让上游 stall
//   - 稳态下 bias_vec[16 × INT32] 是 RF 组合输出, 1 拍可用
//
// SRAM 字布局 (128b/word):
//   word[0]: 4 × INT32 (col 0..3 of bias for this cs)
//   word[1]: 4 × INT32 (col 4..7)
//   word[2]: 4 × INT32 (col 8..11)
//   word[3]: 4 × INT32 (col 12..15)
//
// 触发: 监控 cs_cnt; 检测到变化 → start prefetch (state S_LOAD, 4 拍 SRAM 读).
// 完成 → 回 S_READY. 上层 SDP/ofb_writer 看 bias_ready 决定是否进 SDP fire.
//
// 复位: state + r_loaded_cs 同步复位; RF 是数据路径无复位.
// =============================================================================

module bias_rf #(
    parameter int NUM_COL     = 16,
    parameter int PSUM_WIDTH  = 32,
    parameter int SRAM_DATA_W = 128,                     // Shortcut Bank 字宽
    parameter int SRAM_ADDR_W = 13                       // Shortcut Bank 地址宽
)(
    input  logic                              clk,
    input  logic                              rst_n,

    // ---- 控制 ----
    // start: 一次新 layer / 新 case 开始时拉 1 拍, 强制重新 prefetch (无视当前 RF)
    input  logic                              start,

    // ---- cs 跟踪 (来自上游, 决定要 prefetch 哪一段 bias) ----
    input  logic [5:0]                        cs_cnt,           // 0..cout_slices-1
    input  logic [SRAM_ADDR_W-1:0]            cfg_bias_base,    // bias 在 SRAM 中的起始地址 (word index)

    // ---- Shortcut Bank 读端口 (本模块发起) ----
    output logic                              br_re,
    output logic [SRAM_ADDR_W-1:0]            br_raddr,
    input  logic [SRAM_DATA_W-1:0]            br_rdata,

    // ---- 输出给 SDP ----
    output logic [NUM_COL*PSUM_WIDTH-1:0]     bias_vec,         // 16 × INT32 packed
    output logic                              bias_ready        // 1 = bias_vec 有效, 上游可消费
);

    localparam int WORDS_PER_CS = (NUM_COL * PSUM_WIDTH) / SRAM_DATA_W;  // 16×32 / 128 = 4

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [1:0] {
        S_IDLE   = 2'd0,    // 复位后或 start 前
        S_LOAD   = 2'd1,    // 正在 prefetch (4 拍 SRAM 读)
        S_SETTLE = 2'd2,    // 等最后一拍 sb_rdata 写到 rf (1 拍)
        S_READY  = 2'd3     // RF 持有当前 r_loaded_cs 的 bias, 输出有效
    } state_t;
    state_t state, state_next;

    // =========================================================================
    // 寄存器
    // =========================================================================
    logic [PSUM_WIDTH-1:0] rf [0:NUM_COL-1];               // 主 RF, 16 × INT32 = 512 bit FF
    logic [5:0]            r_loaded_cs;                    // RF 当前持有哪个 cs 的 bias
    logic [1:0]            load_word_idx;                  // 0..3, 当前要读的 SRAM word
    logic [1:0]            load_word_idx_d1;               // 上一拍发起的 word_idx, 用于把 br_rdata 写到对应 4 个 RF entry
    logic                  load_re_d1;                     // 上一拍 br_re, 用来 latch rdata 的 valid 边沿

    // =========================================================================
    // 派生
    // =========================================================================
    logic cs_changed;
    assign cs_changed = (cs_cnt != r_loaded_cs);

    logic load_done;
    assign load_done = (state == S_LOAD) && (load_word_idx == 2'd3) && br_re;

    // SRAM 读地址 = cfg_bias_base + cs_cnt × 4 + load_word_idx
    logic [SRAM_ADDR_W-1:0] cs_word_offset;
    assign cs_word_offset = SRAM_ADDR_W'({cs_cnt, 2'b0});  // cs × 4
    assign br_raddr = cfg_bias_base[SRAM_ADDR_W-1:0]
                    + cs_word_offset
                    + {{(SRAM_ADDR_W-2){1'b0}}, load_word_idx};

    assign br_re = (state == S_LOAD);

    // =========================================================================
    // FSM
    // =========================================================================
    always_comb begin
        state_next = state;
        case (state)
            S_IDLE   : if (start)        state_next = S_LOAD;
            S_LOAD   : if (load_done)    state_next = S_SETTLE;
            S_SETTLE :                   state_next = S_READY;     // 1 拍等 rf[12..15] 写完
            S_READY  : if (cs_changed)   state_next = S_LOAD;
            default  :                   state_next = S_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if      (!rst_n)  state <= S_IDLE;
        else if (start)   state <= S_LOAD;     // start 强制重新 load (新 case)
        else              state <= state_next;
    end

    // load_word_idx: S_LOAD 期间 0→1→2→3, 完成后 wrap
    always_ff @(posedge clk) begin
        if      (!rst_n)               load_word_idx <= '0;
        else if (state != S_LOAD)      load_word_idx <= '0;       // 离开 LOAD 时归零, 下次进 LOAD 重新从 0
        else if (br_re)                load_word_idx <= load_word_idx + 2'd1;
    end

    // 1 拍延迟 (br_re → br_rdata 有效是下一拍, sram 同步读)
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            load_re_d1       <= 1'b0;
            load_word_idx_d1 <= '0;
        end else begin
            load_re_d1       <= br_re;
            load_word_idx_d1 <= load_word_idx;
        end
    end

    // r_loaded_cs: 完成 load 时记录当前是哪个 cs 的 bias 在 RF 里
    always_ff @(posedge clk) begin
        if      (!rst_n)     r_loaded_cs <= 6'd63;        // 非法值, 强制首次 cs_changed
        else if (start)      r_loaded_cs <= 6'd63;
        else if (load_done)  r_loaded_cs <= cs_cnt;
    end

    // =========================================================================
    // RF 写: br_rdata 上一拍发的 4 word, 这一拍到达 → 写到 RF 的 4 entry
    //   word_idx_d1=0 → rf[0..3]
    //   word_idx_d1=1 → rf[4..7]
    //   ...
    // =========================================================================
    always_ff @(posedge clk) begin
        if (load_re_d1) begin
            for (int w = 0; w < 4; w++) begin
                rf[load_word_idx_d1 * 4 + w] <= br_rdata[w*PSUM_WIDTH +: PSUM_WIDTH];
            end
        end
    end

    // =========================================================================
    // 输出
    // =========================================================================
    always_comb begin
        for (int c = 0; c < NUM_COL; c++) begin
            bias_vec[c*PSUM_WIDTH +: PSUM_WIDTH] = rf[c];
        end
    end

    assign bias_ready = (state == S_READY);

endmodule
