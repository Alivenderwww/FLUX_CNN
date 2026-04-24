`timescale 1ns/1ps

// =============================================================================
// psum_reshape.sv  --  parf → sdp 之间的组合式 PSUM 归约级
//
// Kx-fold 场景下, 16 个 PE 列被切成 cfg_cout_groups 个列组, 每组有 cfg_cout_orig
// 列, 承担原 cout 的同一组输出在不同 kx 偏移下的贡献. drain 时按 cout 维度
// 把这些组对齐累加, 给 SDP 提供 cfg_cout_orig 路 psum.
//
// cout_fake = cout_orig * cout_groups ≤ NUM_COL (16).
//
// 映射关系:
//   out[co] = Σ_{g=0..cout_groups-1} in[g * cout_orig + co]  (co ∈ [0, cout_orig))
//   out[co] = 0 (co ∈ [cout_orig, NUM_COL))
//
// 无折叠 (cout_groups=1) 时: out[co] = in[co], 直通.
//
// 纯组合 — 不占 cycle, 不影响流水线延迟. Adder tree 深度 log2(max_groups)=4.
// =============================================================================
module psum_reshape #(
    parameter int NUM_COL    = 16,
    parameter int PSUM_WIDTH = 32
)(
    // cfg
    input  logic [5:0]                           cfg_cout_orig,      // 1..NUM_COL
    input  logic [4:0]                           cfg_cout_groups,    // 1..NUM_COL

    // upstream (parf drain, 每列一条 int32 psum)
    input  logic                                 in_valid,
    input  logic signed [NUM_COL*PSUM_WIDTH-1:0] in_vec,

    // downstream (sdp, 只看 out[0..cout_orig-1], 其余清零)
    output logic                                 out_valid,
    output logic signed [NUM_COL*PSUM_WIDTH-1:0] out_vec
);

    // valid 直通
    assign out_valid = in_valid;

    // 归约: 对每个 co, 累加对应 group 的源列
    always_comb begin
        for (int co = 0; co < NUM_COL; co++) begin
            logic signed [PSUM_WIDTH-1:0] acc;
            acc = '0;
            if ({1'b0, co[4:0]} < cfg_cout_orig) begin
                for (int g = 0; g < NUM_COL; g++) begin
                    logic [5:0] src_col;
                    src_col = g[5:0] * cfg_cout_orig + co[5:0];
                    if ({3'b0, g[4:0]} < cfg_cout_groups && src_col < NUM_COL) begin
                        acc = acc + $signed(in_vec[src_col[3:0]*PSUM_WIDTH +: PSUM_WIDTH]);
                    end
                end
            end
            out_vec[co*PSUM_WIDTH +: PSUM_WIDTH] = acc;
        end
    end

endmodule
