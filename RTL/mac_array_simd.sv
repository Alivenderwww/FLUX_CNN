`timescale 1ns/1ps

// =============================================================================
// mac_array_simd.sv  --  16×16 INT8 MAC Array, INT8 SIMD on DSP48E1 版本
//
// 跟原 mac_array.sv 接口完全一致, 内部用 mac_simd_pair (1 DSP 装 2 PE).
// 把 16 col 配对成 8 col-pair (col 0/1, col 2/3, ..., col 14/15), 每 col-pair 用
// 16 个 mac_simd_pair (16 row, 每 row 1 simd_pair).
//
// 总 DSP: 8 col-pair × 16 row = 128 DSP / 核 (vs 原 256 LUT-MAC)
// 4 核 = 512 mac DSP + 320 sdp DSP = 832 DSP, K325T 840 上限内 ✓
//
// PE 共享模式不变:
//   原 mac_array: 16 col 共享同一 act_in_vec (16 cin), col c 内 PE i 算 input[i] × w[c][i]
//   改后:         8 col-pair 共享 act_in_vec, pair p 内 row r 算
//                 input[r] × w[col_a=2p][r] (prod_0) + input[r] × w[col_b=2p+1][r] (prod_1)
//
// 加法树跟原 mac_col 一致 (16 row 加和 → col_psum), 但每 col-pair 输出 2 个 col_psum.
//
// 时序: compute_en T → dsp_p reg T+1 → mac_simd_pair sign_corr reg T+2
//       → adder_tree_reg T+3 → psum_out T+3
//       比原 mac_array 多 1 拍 (3-stage pipe vs 2-stage), 因 sign correction 必须独立 reg
//       阻断 DSP cascade (否则 Vivado 把减法器也推 DSP, simd_pair 占 2 DSP).
// =============================================================================

module mac_array_simd #(
    parameter int NUM_COL     = 16,
    parameter int NUM_PE      = 16,
    parameter int DATA_WIDTH  = 8,
    parameter int PSUM_WIDTH  = 32,
    parameter int WRF_DEPTH   = 32,
    parameter int PROD_WIDTH  = 17    // mac_simd_pair 输出宽度 (含 sign corr 1 bit headroom)
)(
    input  logic                                clk,
    input  logic                                rst_n,

    // ---- 权重写端口 (wgt_buffer 的 LOAD), 跟原 mac_array 相同布局 ----
    input  logic [NUM_COL*NUM_PE-1:0]           wrf_we,
    input  logic [$clog2(WRF_DEPTH)-1:0]        wrf_waddr,
    input  logic [NUM_COL*NUM_PE*DATA_WIDTH-1:0]wrf_wdata,

    // ---- 激活 / 读权重地址（COMPUTE） ----
    input  logic [NUM_PE*DATA_WIDTH-1:0]        act_in_vec,
    input  logic                                act_valid,
    output logic                                act_ready,

    input  logic [$clog2(WRF_DEPTH)-1:0]        wrf_raddr,
    input  logic                                wgt_valid,
    output logic                                wgt_ready,

    // ---- psum 输出到 parf_accum ----
    output logic                                 psum_out_valid,
    output logic signed [NUM_COL*PSUM_WIDTH-1:0] psum_out_vec,
    input  logic                                 psum_in_ready,

    input  logic                                 is_first_round_fill,
    input  logic signed [NUM_COL*PSUM_WIDTH-1:0] old_psum_vec
);

    localparam int NUM_PAIR = NUM_COL / 2;   // 8 col-pair

    // =========================================================================
    // Handshake (Elastic Join) — 3-stage pipe (s1=dsp_p, s2=sign_corr, s3=adder_tree)
    // =========================================================================
    logic pipe_s1_valid, pipe_s2_valid, pipe_s3_valid;
    logic can_advance;
    assign can_advance = (~pipe_s3_valid) | psum_in_ready;

    assign act_ready = can_advance & wgt_valid;
    assign wgt_ready = can_advance & act_valid;

    logic compute_en;
    assign compute_en = can_advance;

    logic stage0_has_data;
    assign stage0_has_data = act_valid & wgt_valid;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            pipe_s1_valid <= 1'b0;
            pipe_s2_valid <= 1'b0;
            pipe_s3_valid <= 1'b0;
        end else if (can_advance) begin
            pipe_s1_valid <= stage0_has_data;
            pipe_s2_valid <= pipe_s1_valid;
            pipe_s3_valid <= pipe_s2_valid;
        end
    end

    assign psum_out_valid = pipe_s3_valid;

    // =========================================================================
    // mac_simd_pair 阵列: NUM_PAIR × NUM_PE (= 8 × 16 = 128)
    // 每 simd_pair 算 1 个 act × {w_col_a, w_col_b}, 输出 2 个独立 prod
    //   prod_a_arr[p][r] = act_in_vec[r] × w[col_a=2p][r]
    //   prod_b_arr[p][r] = act_in_vec[r] × w[col_b=2p+1][r]
    // =========================================================================
    logic signed [PROD_WIDTH-1:0] prod_a_arr [NUM_PAIR][NUM_PE];
    logic signed [PROD_WIDTH-1:0] prod_b_arr [NUM_PAIR][NUM_PE];

    genvar p, r;
    generate
        for (p = 0; p < NUM_PAIR; p++) begin : gen_pair
            // pair p 对应 col_a = 2p, col_b = 2p+1
            localparam int COL_A = 2 * p;
            localparam int COL_B = 2 * p + 1;

            // 每 row 一个 mac_simd_pair
            for (r = 0; r < NUM_PE; r++) begin : gen_row
                // PE 索引 (旧布局): col_a 内的第 r 个 PE = COL_A * NUM_PE + r
                //                   col_b 内的第 r 个 PE = COL_B * NUM_PE + r
                localparam int PE_A_IDX = COL_A * NUM_PE + r;
                localparam int PE_B_IDX = COL_B * NUM_PE + r;

                mac_simd_pair #(
                    .DATA_WIDTH(DATA_WIDTH),
                    .WRF_DEPTH (WRF_DEPTH),
                    .PROD_WIDTH(PROD_WIDTH)
                ) u_simd (
                    .clk        (clk),
                    .rst_n      (rst_n),
                    .wrf_we     ({wrf_we[PE_B_IDX], wrf_we[PE_A_IDX]}),
                    .wrf_waddr  (wrf_waddr),
                    .wrf_wdata_0(wrf_wdata[PE_A_IDX*DATA_WIDTH +: DATA_WIDTH]),
                    .wrf_wdata_1(wrf_wdata[PE_B_IDX*DATA_WIDTH +: DATA_WIDTH]),
                    .wrf_raddr  (wrf_raddr),
                    .act_in     (act_in_vec[r*DATA_WIDTH +: DATA_WIDTH]),
                    .compute_en (compute_en),
                    .prod_0     (prod_a_arr[p][r]),
                    .prod_1     (prod_b_arr[p][r])
                );
            end
        end
    endgenerate

    // =========================================================================
    // 加法树: 每 col-pair 拆出 2 个 col 各自的 16-input adder tree, 跟原 mac_col 等价
    // 输出 16 个 col_psum, 每个对应原 mac_array 的一列
    // =========================================================================
    logic signed [PSUM_WIDTH-1:0] col_psum_comb [NUM_COL];
    logic signed [PSUM_WIDTH-1:0] col_psum_reg  [NUM_COL];

    generate
        for (p = 0; p < NUM_PAIR; p++) begin : gen_tree_pair
            localparam int COL_A = 2 * p;
            localparam int COL_B = 2 * p + 1;

            always_comb begin
                col_psum_comb[COL_A] = '0;
                col_psum_comb[COL_B] = '0;
                for (int rr = 0; rr < NUM_PE; rr++) begin
                    col_psum_comb[COL_A] = col_psum_comb[COL_A] + $signed(prod_a_arr[p][rr]);
                    col_psum_comb[COL_B] = col_psum_comb[COL_B] + $signed(prod_b_arr[p][rr]);
                end
            end
        end
    endgenerate

    // 加法树 reg (stage 2)
    genvar c;
    generate
        for (c = 0; c < NUM_COL; c++) begin : gen_col_reg
            always_ff @(posedge clk) begin
                if (compute_en) col_psum_reg[c] <= col_psum_comb[c];
            end

            // acc seed: is_first_round_fill 时 0, 否则 old_psum
            logic signed [PSUM_WIDTH-1:0] acc_seed;
            assign acc_seed = is_first_round_fill
                            ? '0
                            : old_psum_vec[c*PSUM_WIDTH +: PSUM_WIDTH];

            assign psum_out_vec[c*PSUM_WIDTH +: PSUM_WIDTH] = col_psum_reg[c] + acc_seed;
        end
    endgenerate

    // =========================================================================
    // Simulation-only: 握手 & MAC 计数器 (跟 mac_array.sv 保持一致, TB 直接读)
    // =========================================================================
    // synthesis translate_off
    int hs_act_fire   = 0;
    int hs_act_stall  = 0;
    int hs_act_idle   = 0;
    int hs_wgt_fire   = 0;
    int hs_wgt_stall  = 0;
    int hs_wgt_idle   = 0;
    int hs_psum_fire  = 0;
    int hs_psum_stall = 0;
    int hs_psum_idle  = 0;
    int mac_fire_cnt  = 0;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            hs_act_fire <= 0; hs_act_stall <= 0; hs_act_idle <= 0;
        end else if ( act_valid &&  act_ready)  hs_act_fire  <= hs_act_fire  + 1;
        else if ( act_valid && !act_ready)      hs_act_stall <= hs_act_stall + 1;
        else if (!act_valid &&  act_ready)      hs_act_idle  <= hs_act_idle  + 1;
    end
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            hs_wgt_fire <= 0; hs_wgt_stall <= 0; hs_wgt_idle <= 0;
        end else if ( wgt_valid &&  wgt_ready)  hs_wgt_fire  <= hs_wgt_fire  + 1;
        else if ( wgt_valid && !wgt_ready)      hs_wgt_stall <= hs_wgt_stall + 1;
        else if (!wgt_valid &&  wgt_ready)      hs_wgt_idle  <= hs_wgt_idle  + 1;
    end
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            hs_psum_fire <= 0; hs_psum_stall <= 0; hs_psum_idle <= 0;
        end else if ( psum_out_valid &&  psum_in_ready) hs_psum_fire  <= hs_psum_fire  + 1;
        else if ( psum_out_valid && !psum_in_ready)     hs_psum_stall <= hs_psum_stall + 1;
        else if (!psum_out_valid &&  psum_in_ready)     hs_psum_idle  <= hs_psum_idle  + 1;
    end
    always_ff @(posedge clk) begin
        if (!rst_n)                                       mac_fire_cnt <= 0;
        else if (act_valid && wgt_valid && can_advance)   mac_fire_cnt <= mac_fire_cnt + 1;
    end
    // synthesis translate_on

endmodule
