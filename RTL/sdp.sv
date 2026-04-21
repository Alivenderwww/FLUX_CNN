// -----------------------------------------------------------------------------
// File        : sdp.sv
// Description : Single Data Processor (SDP) — per-tensor symmetric int8 quant
//
// 流水（纯组合，0 延迟）：
//   1. prod[c]    = acc[c] * mult                 (int32 × int32 → int64)
//   2. rounded[c] = round_en ? prod[c] + 2^(shift-1) : prod[c]   (asymmetric rounding)
//   3. q[c]       = rounded[c] >>> shift                         (arithmetic shift)
//   4. q_zp[c]    = q[c] + zp_out
//   5. act[c]     = (relu_en && q_zp[c] < 0) ? 0 : q_zp[c]
//   6. clipped[c] = clip(act[c], clip_min, clip_max)
//   7. ofm_data[c*8+:8] = clipped[c][7:0]         (signed int8 truncate)
//
// 向后兼容（uint8 等价）：
//   mult=1, zp_out=0, relu_en=1, clip=[0,255], round_en=0 → 等同原 shift+ReLU+clip[0,255]
//
// 典型 per-tensor symmetric int8 ReLU 层：
//   mult=M (≈ s_x*s_w/s_y * 2^shift, int32), shift=24~31, zp_out=0, relu_en=1,
//   clip=[0,127] (ReLU) or [-128,127] (no ReLU)
// -----------------------------------------------------------------------------
`timescale 1ns/1ps

module sdp #(
    parameter int NUM_COL    = 16,
    parameter int PSUM_WIDTH = 32
)(
    // --- cfg (组合输入，由 cfg_regs 直接驱动) ---
    input  logic [5:0]                        shift_amt,
    input  logic signed [31:0]                mult,
    input  logic signed [8:0]                 zp_out,
    input  logic signed [8:0]                 clip_min,
    input  logic signed [8:0]                 clip_max,
    input  logic                              round_en,
    input  logic                              relu_en,

    // --- 数据通路 ---
    input  logic [NUM_COL*PSUM_WIDTH-1:0]     psum_in,
    input  logic                              valid_in,
    output logic [NUM_COL*8-1:0]              ofm_data,
    output logic                              valid_out
);

    localparam int EXT_W = PSUM_WIDTH + 10;   // 42: 足够放 q+zp_out，不溢

    logic signed [PSUM_WIDTH-1:0] psum_ch  [0:NUM_COL-1];
    logic signed [63:0]           prod_ch  [0:NUM_COL-1];
    logic signed [63:0]           round_ch [0:NUM_COL-1];
    logic signed [63:0]           shifted_ch [0:NUM_COL-1];
    logic signed [EXT_W-1:0]      q_zp_ch  [0:NUM_COL-1];
    logic signed [EXT_W-1:0]      act_ch   [0:NUM_COL-1];
    logic signed [EXT_W-1:0]      clip_ch  [0:NUM_COL-1];

    // Sign-extend 9-bit signed cfg 到 EXT_W signed —— 必须通过 signed 变量保证比较走 signed 路径。
    // 直接写 `{{(EXT_W-9){x[8]}}, x}` 会产生 unsigned concat，混进 signed 比较时 SV 会把两边都
    // 当 unsigned 比，负值被当作巨大正值 → clip 误饱和（Phase H L4 relu_en=0 bug 根因）。
    logic signed [EXT_W-1:0] zp_out_ext, clip_min_ext, clip_max_ext;
    assign zp_out_ext   = EXT_W'(zp_out);     // SV: signed → 更宽 signed 自动 sign-extend
    assign clip_min_ext = EXT_W'(clip_min);
    assign clip_max_ext = EXT_W'(clip_max);

    logic signed [63:0] round_bias;
    assign round_bias = (round_en && shift_amt != 6'd0) ?
                        (64'sd1 <<< (shift_amt - 6'd1)) : 64'sd0;

    always_comb begin
        valid_out = valid_in;
        for (int c = 0; c < NUM_COL; c++) begin
            // 1. Extract signed int32 psum
            psum_ch[c] = $signed(psum_in[c*PSUM_WIDTH +: PSUM_WIDTH]);

            // 2. Multiply
            prod_ch[c] = $signed(psum_ch[c]) * $signed(mult);

            // 3. Round bias then arithmetic right shift
            round_ch[c]   = prod_ch[c] + round_bias;
            shifted_ch[c] = round_ch[c] >>> shift_amt;

            // 4. Add output zero-point (signed add)
            q_zp_ch[c] = shifted_ch[c][EXT_W-1:0] + zp_out_ext;

            // 5. ReLU (apply before clip)
            act_ch[c] = (relu_en && q_zp_ch[c] < 0) ? '0 : q_zp_ch[c];

            // 6. Clip to [clip_min, clip_max] (signed 比较)
            if      (act_ch[c] < clip_min_ext) clip_ch[c] = clip_min_ext;
            else if (act_ch[c] > clip_max_ext) clip_ch[c] = clip_max_ext;
            else                               clip_ch[c] = act_ch[c];

            // 7. Truncate to 8-bit (signed int8 / or uint8 in legacy config)
            ofm_data[c*8 +: 8] = clip_ch[c][7:0];
        end
    end

endmodule
