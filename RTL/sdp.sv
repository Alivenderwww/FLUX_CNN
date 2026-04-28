// -----------------------------------------------------------------------------
// File        : sdp.sv
// Description : Single Data Processor (SDP) — bias add + per-tensor quant + residual
//
// 流水（纯组合，0 延迟，本期一拍完成；时序紧时后续可切寄存器）：
//   ① psum_signed [c]   = $signed(psum_in[c])                          // INT32
//   ② biased    [c]     = psum_signed[c] + bias_in[c]                  // INT33
//   ③ prod      [c]     = biased[c] * mult                             // INT64
//   ④ rounded   [c]     = round_en ? prod[c] + 2^(shift-1) : prod[c]
//   ⑤ shifted   [c]     = rounded[c] >>> shift_amt                     // arithmetic shift
//   ⑥ q_zp      [c]     = shifted[c] + zp_out                          // EXT_W signed
//   ⑦ shortcut_scaled[c]= residual_en ? (shortcut_in[c] * shortcut_mult) >>> shortcut_shift : 0
//   ⑧ summed    [c]     = q_zp[c] + shortcut_scaled[c]                 // EXT_W signed
//   ⑨ act       [c]     = relu_en && summed<0 ? 0 : summed
//   ⑩ clipped   [c]     = clip(act, clip_min, clip_max)
//   ⑪ ofm_data  [c]     = clipped[c][7:0]                              // INT8 truncate
//
// bias 已经从 mac_array 的 first_round seed 路径迁移到这里 (post-PARF). bias_in
// 由 bias_rf 模块从 Shortcut Bank 拉的当前 cs 的 16 个 INT32 值组合提供.
//
// shortcut (residual) add: 由编译器 emit cfg_residual_en. shortcut_in 来自
// ofb_writer 从 Shortcut Bank 当前 (yout, x, cs) 位置组合读出的 16 × INT8.
// shortcut_mult / shortcut_shift 做量化 scale 对齐 (上一层 scale → 当前层 scale).
// residual_en=0 时 shortcut path 加 0, 行为退化为纯 conv+bias.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps

module sdp #(
    parameter int NUM_COL    = 16,
    parameter int PSUM_WIDTH = 32
)(
    // --- 现有量化 cfg (由 cfg_regs 直接驱动) ---
    input  logic [5:0]                        shift_amt,
    input  logic signed [31:0]                mult,
    input  logic signed [8:0]                 zp_out,
    input  logic signed [8:0]                 clip_min,
    input  logic signed [8:0]                 clip_max,
    input  logic                              round_en,
    input  logic                              relu_en,

    // --- residual / shortcut cfg ---
    input  logic                              residual_en,        // 0=跳过 shortcut path
    input  logic signed [15:0]                shortcut_mult,      // INT16 scale align 乘子
    input  logic [4:0]                        shortcut_shift,     // 0..31 算术右移

    // --- 数据通路 ---
    input  logic [NUM_COL*PSUM_WIDTH-1:0]     psum_in,            // 来自 parf_accum
    input  logic [NUM_COL*PSUM_WIDTH-1:0]     bias_in,            // 来自 bias_rf
    input  logic [NUM_COL*8-1:0]              shortcut_in,        // 来自 Shortcut Bank (ofb_writer 取地址)
    input  logic                              valid_in,
    output logic [NUM_COL*8-1:0]              ofm_data,
    output logic                              valid_out
);

    localparam int EXT_W = PSUM_WIDTH + 10;   // 42: 容纳 q+zp 和 + shortcut 不溢

    logic signed [PSUM_WIDTH-1:0]   psum_ch       [0:NUM_COL-1];
    logic signed [PSUM_WIDTH:0]     biased_ch     [0:NUM_COL-1];   // INT33 (+1 bit headroom)
    logic signed [63:0]             prod_ch       [0:NUM_COL-1];
    logic signed [63:0]             round_ch      [0:NUM_COL-1];
    logic signed [63:0]             shifted_ch    [0:NUM_COL-1];
    logic signed [EXT_W-1:0]        q_zp_ch       [0:NUM_COL-1];
    logic signed [7:0]              shortcut_ch   [0:NUM_COL-1];
    logic signed [23:0]             sc_prod_ch    [0:NUM_COL-1];   // INT8 × INT16 = INT24
    logic signed [23:0]             sc_shifted_ch [0:NUM_COL-1];
    logic signed [EXT_W-1:0]        sc_ext_ch     [0:NUM_COL-1];
    logic signed [EXT_W-1:0]        summed_ch     [0:NUM_COL-1];
    logic signed [EXT_W-1:0]        act_ch        [0:NUM_COL-1];
    logic signed [EXT_W-1:0]        clip_ch       [0:NUM_COL-1];

    // 9-bit signed cfg sign-extend 到 EXT_W
    logic signed [EXT_W-1:0] zp_out_ext, clip_min_ext, clip_max_ext;
    assign zp_out_ext   = EXT_W'(zp_out);
    assign clip_min_ext = EXT_W'(clip_min);
    assign clip_max_ext = EXT_W'(clip_max);

    logic signed [63:0] round_bias;
    assign round_bias = (round_en && shift_amt != 6'd0) ?
                        (64'sd1 <<< (shift_amt - 6'd1)) : 64'sd0;

    always_comb begin
        valid_out = valid_in;
        for (int c = 0; c < NUM_COL; c++) begin
            // -- 主路径 (psum + bias → quantize) --
            psum_ch[c]    = $signed(psum_in[c*PSUM_WIDTH +: PSUM_WIDTH]);
            biased_ch[c]  = (PSUM_WIDTH+1)'(psum_ch[c]) + (PSUM_WIDTH+1)'($signed(bias_in[c*PSUM_WIDTH +: PSUM_WIDTH]));
            prod_ch[c]    = $signed(biased_ch[c]) * $signed(mult);
            round_ch[c]   = prod_ch[c] + round_bias;
            shifted_ch[c] = round_ch[c] >>> shift_amt;
            q_zp_ch[c]    = shifted_ch[c][EXT_W-1:0] + zp_out_ext;

            // -- shortcut 路径 (rescale + sign extend), residual_en=0 时归零 --
            shortcut_ch[c]    = $signed(shortcut_in[c*8 +: 8]);
            sc_prod_ch[c]     = shortcut_ch[c] * shortcut_mult;
            sc_shifted_ch[c]  = sc_prod_ch[c] >>> shortcut_shift;
            sc_ext_ch[c]      = residual_en ? EXT_W'(sc_shifted_ch[c]) : '0;

            // -- 合并 --
            summed_ch[c] = q_zp_ch[c] + sc_ext_ch[c];

            // -- ReLU + Clip + Trunc --
            act_ch[c] = (relu_en && summed_ch[c] < 0) ? '0 : summed_ch[c];
            if      (act_ch[c] < clip_min_ext) clip_ch[c] = clip_min_ext;
            else if (act_ch[c] > clip_max_ext) clip_ch[c] = clip_max_ext;
            else                               clip_ch[c] = act_ch[c];

            ofm_data[c*8 +: 8] = clip_ch[c][7:0];
        end
    end

endmodule
