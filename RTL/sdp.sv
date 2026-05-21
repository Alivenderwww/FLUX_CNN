// -----------------------------------------------------------------------------
// File        : sdp.sv
// Description : Single Data Processor (SDP) — bias add + per-tensor quant + residual
//
// 流水化 (J-4): 3-stage pipeline (valid 跟数据 lockstep 流动, 无 sideband delay)
//
//   stage 0 (输入 comb): psum_in + bias_in = biased (33-bit signed)
//   stage 1 latch     : prod_d     = biased * mult       // DSP48 input + M reg
//                        shortcut_d = shortcut_in
//                        valid_d
//   stage 2a (comb)   : 主路径 round + shift + zp
//                        shortcut path: sc_prod = shortcut_d * shortcut_mult,
//                                        sc_shifted = sc_prod >>> shortcut_shift,
//                                        sc_ext = residual_en ? ext : 0
//   stage 2a latch    : q_zp_d, sc_ext_d, valid_dd
//   stage 2b (comb out): summed = q_zp_d + sc_ext_d, relu + clip + trunc → ofm_data
//                        valid_out = valid_dd
//
// cfg signals (mult/shift/clip/zp/relu/round/residual/shortcut_mult/shortcut_shift)
// 在 layer 内 stable, sequencer 跨 layer 切换前会 wait pipeline drained, 所以
// SDP pipeline 内部用 cfg 直连 (不 latch), 不会出现 mid-pipeline cfg 变化.
//
// total latency: 3 cycles (input fire → output valid 3 拍后; 2026-05-18 SDP 加 stage 0a)
//   ofb_writer 用 valid_out 当 ofb_we, in_flight 计数器扩到 [2:0] 容纳 max 3.
//
// J-3 → J-4 timing 演进:
//   J-3 (1-stage SDP): WNS +0.781 ns @ 100 MHz, critical path stage 2 全链
//                       (round + shift + zp + sc + relu + clip), 45 levels CARRY4=33
//   J-4 (2-stage SDP): 拆点在 q_zp_d / sc_ext_d, stage 2a 含变量移位是大开销,
//                      stage 2b 只剩 add + relu + clip + trunc, 应能 met 125 MHz
// -----------------------------------------------------------------------------
`timescale 1ns/1ps

module sdp #(
    parameter int NUM_COL    = 16,
    parameter int PSUM_WIDTH = 32
)(
    input  logic                              clk,
    input  logic                              rst_n,

    // --- 量化 cfg (cfg_regs 直接驱动, layer-stable) ---
    input  logic [5:0]                        shift_amt,
    input  logic signed [31:0]                mult,
    input  logic signed [8:0]                 zp_out,
    input  logic signed [8:0]                 clip_min,
    input  logic signed [8:0]                 clip_max,
    input  logic                              round_en,
    input  logic                              relu_en,

    // --- residual / shortcut cfg ---
    input  logic                              residual_en,
    input  logic signed [15:0]                shortcut_mult,
    input  logic [4:0]                        shortcut_shift,

    // --- 数据通路 (stage 0 输入) ---
    input  logic [NUM_COL*PSUM_WIDTH-1:0]     psum_in,
    input  logic [NUM_COL*PSUM_WIDTH-1:0]     bias_in,
    input  logic [NUM_COL*8-1:0]              shortcut_in,
    input  logic                              valid_in,

    // --- 输出 (stage 2b, 3 cycle latency; 2026-05-18 stage 0a 拆 add+mult) ---
    output logic [NUM_COL*8-1:0]              ofm_data,
    output logic                              valid_out
);

    localparam int EXT_W = PSUM_WIDTH + 10;   // 42: 容纳 q+zp 和 + shortcut 不溢

    // mult / shift 由 ofb_writer 端 (* keep = "true" *) latch 一拍后送入 SDP, 这里
    // 直接用 input port 值 (= 已经 latched 后的 cfg). 不在 SDP 内重复 latch.
    // =========================================================================
    // Stage 0 → 0a (latch): psum + bias 拼成 biased
    //   2026-05-18: 拆 stage. 原版 (biased + biased*mult) 同一拍, Vivado 把 32×33-bit
    //   signed mult 拆 LUTCY 链 (logic levels 26-30), 跑不过 150 MHz. 拆成 add 单独 1
    //   拍 + mult 单独 1 拍后, Vivado 能推 DSP58 (M reg = 1-cycle), 满足 150 MHz.
    //   代价: SDP latency 2→3 cycle, streaming 吞吐不变, 推理首 OFM 多 1 cycle 无感.
    // =========================================================================
    logic signed [PSUM_WIDTH:0] biased_d [0:NUM_COL-1];
    logic                       valid_b;

    always_ff @(posedge clk) begin
        if (!rst_n) valid_b <= 1'b0;
        else        valid_b <= valid_in;
    end

    always_ff @(posedge clk) begin
        for (int c = 0; c < NUM_COL; c++) begin
            biased_d[c] <= (PSUM_WIDTH+1)'($signed(psum_in[c*PSUM_WIDTH +: PSUM_WIDTH]))
                         + (PSUM_WIDTH+1)'($signed(bias_in[c*PSUM_WIDTH +: PSUM_WIDTH]));
        end
    end

    // shortcut_in 跟 biased_d 同步, 多 1 拍 latch 跟 prod_d 对齐
    logic signed [7:0] shortcut_b [0:NUM_COL-1];
    always_ff @(posedge clk) begin
        for (int c = 0; c < NUM_COL; c++) begin
            shortcut_b[c] <= $signed(shortcut_in[c*8 +: 8]);
        end
    end

    // =========================================================================
    // Stage 0a → 1 (latch): prod_d = biased_d * mult, shortcut 透传, valid
    //   biased_d 已 reg, mult 由 ofb_writer 端 KEEP register latch (避免 Vivado retime
    //   合并掉), 32×33 → 64 signed mult Vivado 推 DSP58 (1-cycle).
    // =========================================================================
    logic signed [63:0]    prod_d      [0:NUM_COL-1];
    logic signed [7:0]     shortcut_d  [0:NUM_COL-1];
    logic                  valid_d;

    always_ff @(posedge clk) begin
        if (!rst_n) valid_d <= 1'b0;
        else        valid_d <= valid_b;
    end

    always_ff @(posedge clk) begin
        for (int c = 0; c < NUM_COL; c++) begin
            prod_d[c]     <= $signed(biased_d[c]) * $signed(mult);
            shortcut_d[c] <= shortcut_b[c];
        end
    end

    // =========================================================================
    // Stage 2a (comb after stage 1):
    //   主路径 prod_d → +round_bias → >>> shift → +zp_out  → q_zp_ch
    //   shortcut path shortcut_d → × mult → >>> sc_shift → ext (residual gate) → sc_ext_ch
    // =========================================================================
    logic signed [63:0]      round_bias;
    assign round_bias = (round_en && shift_amt != 6'd0) ?
                        (64'sd1 <<< (shift_amt - 6'd1)) : 64'sd0;

    logic signed [EXT_W-1:0] zp_out_ext, clip_min_ext, clip_max_ext;
    assign zp_out_ext   = EXT_W'(zp_out);
    assign clip_min_ext = EXT_W'(clip_min);
    assign clip_max_ext = EXT_W'(clip_max);

    logic signed [63:0]       round_ch      [0:NUM_COL-1];
    logic signed [63:0]       shifted_ch    [0:NUM_COL-1];
    logic signed [EXT_W-1:0]  q_zp_ch       [0:NUM_COL-1];
    logic signed [23:0]       sc_prod_ch    [0:NUM_COL-1];
    logic signed [23:0]       sc_shifted_ch [0:NUM_COL-1];
    logic signed [EXT_W-1:0]  sc_ext_ch     [0:NUM_COL-1];

    always_comb begin
        for (int c = 0; c < NUM_COL; c++) begin
            round_ch[c]      = prod_d[c] + round_bias;
            shifted_ch[c]    = round_ch[c] >>> shift_amt;
            q_zp_ch[c]       = shifted_ch[c][EXT_W-1:0] + zp_out_ext;

            sc_prod_ch[c]    = shortcut_d[c] * shortcut_mult;
            sc_shifted_ch[c] = sc_prod_ch[c] >>> shortcut_shift;
            sc_ext_ch[c]     = residual_en ? EXT_W'(sc_shifted_ch[c]) : '0;
        end
    end

    // =========================================================================
    // Stage 2a latch: q_zp_d, sc_ext_d, valid_dd
    // =========================================================================
    logic signed [EXT_W-1:0]  q_zp_d   [0:NUM_COL-1];
    logic signed [EXT_W-1:0]  sc_ext_d [0:NUM_COL-1];
    logic                     valid_dd;

    always_ff @(posedge clk) begin
        if (!rst_n) valid_dd <= 1'b0;
        else        valid_dd <= valid_d;
    end

    always_ff @(posedge clk) begin
        for (int c = 0; c < NUM_COL; c++) begin
            q_zp_d[c]   <= q_zp_ch[c];
            sc_ext_d[c] <= sc_ext_ch[c];
        end
    end

    // =========================================================================
    // Stage 2b (comb output): summed + relu + clip + trunc
    // =========================================================================
    logic signed [EXT_W-1:0]  summed_ch [0:NUM_COL-1];
    logic signed [EXT_W-1:0]  act_ch    [0:NUM_COL-1];
    logic signed [EXT_W-1:0]  clip_ch   [0:NUM_COL-1];

    always_comb begin
        valid_out = valid_dd;
        for (int c = 0; c < NUM_COL; c++) begin
            summed_ch[c] = q_zp_d[c] + sc_ext_d[c];
            act_ch[c]    = (relu_en && summed_ch[c] < 0) ? '0 : summed_ch[c];
            if      (act_ch[c] < clip_min_ext) clip_ch[c] = clip_min_ext;
            else if (act_ch[c] > clip_max_ext) clip_ch[c] = clip_max_ext;
            else                               clip_ch[c] = act_ch[c];
            ofm_data[c*8 +: 8] = clip_ch[c][7:0];
        end
    end

endmodule
