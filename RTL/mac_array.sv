`timescale 1ns/1ps

// =============================================================================
// mac_array.sv  --  16×16 int8 MAC Array with valid-ready Handshakes
//
// 用法：
//   上游
//     line_buffer  ─►{act_vec, act_valid} ─►┐
//                                            ├── join ── compute ── 2 拍 pipe
//     wgt_buffer   ─►{wrf_raddr, wgt_valid}─►┘            │
//                                                         ▼
//                                         parf_accum ◄─ {psum_out, psum_out_valid}
//                                                         │
//                                                         └──psum_in_ready (背压)
//
// 握手策略：**global stall**
//     advance    = psum_in_ready         (下游 ready → 整个 pipe 推进一步)
//     compute_en = advance               (传给 mac_col 和 mac_pe；stall 时寄存器保持)
//     act_ready  = advance
//     wgt_ready  = advance
//
//     两级 pipeline valid 追踪：
//         pipe_s1_valid ← (advance ? act_valid & wgt_valid : hold)
//         pipe_s2_valid ← (advance ? pipe_s1_valid         : hold)
//     psum_out_valid = pipe_s2_valid
//
// 时序（act_valid=1 at T, psum_in_ready=1 连续）：
//   T   : advance=1, act/wgt join, mac_pe captures act*wgt into prod_out@T+1
//   T+1 : adder_tree_reg@T+2 = Σ prod_out@T+1 = Σ act*wgt@T
//   T+2 : psum_out_valid=1, psum_out=Σ act*wgt@T
//
// pipe 内残留数据在 upstream 停止后、parf 仍 ready 时可由 bubble（valid=0）
// 连续 2 拍推挤流出，由 parf 正常消费。
// =============================================================================

module mac_array #(
    parameter int NUM_COL     = 16,
    parameter int NUM_PE      = 16,
    parameter int DATA_WIDTH  = 8,
    parameter int PSUM_WIDTH  = 32,
    parameter int WRF_DEPTH   = 32
)(
    input  logic                                clk,
    input  logic                                rst_n,

    // ---- 权重写端口 (wgt_buffer 的 LOAD) ----
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
    input  logic                                 psum_in_ready
);

    // =========================================================================
    // Handshake (Elastic Join)
    // =========================================================================
    logic pipe_s1_valid, pipe_s2_valid;

    // can_advance: pipe 可以推进一步 (stage 2 empty，或本拍正被下游消费)
    logic can_advance;
    assign can_advance = (~pipe_s2_valid) | psum_in_ready;

    // act/wgt ready: 只有当 pipe 可推进、且对方也 valid 时才消费
    assign act_ready = can_advance & wgt_valid;
    assign wgt_ready = can_advance & act_valid;

    // compute_en 传给 mac_col / mac_pe 作为 pipe advance enable
    logic compute_en;
    assign compute_en = can_advance;

    // =========================================================================
    // Pipeline valid 追踪（2 级：prod_out reg、adder_tree reg）
    // =========================================================================
    logic stage0_has_data;
    assign stage0_has_data = act_valid & wgt_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pipe_s1_valid <= 1'b0;
            pipe_s2_valid <= 1'b0;
        end else if (can_advance) begin
            pipe_s1_valid <= stage0_has_data;
            pipe_s2_valid <= pipe_s1_valid;
        end
    end

    assign psum_out_valid = pipe_s2_valid;

    // =========================================================================
    // Simulation-only: 握手 & MAC 计数器
    //   - hs_*_fire  : V=1 & R=1, 真握手成功
    //   - hs_*_stall : V=1 & R=0, 上游被下游堵 (下游慢)
    //   - hs_*_idle  : V=0 & R=1, 下游空等上游 (上游慢)
    //   - true_fire_cnt : stage 0 join 成功的周期数 (= 真正送入 pipe 的 MAC 拍数)
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
    int true_fire_cnt = 0;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            hs_act_fire   <= 0;
            hs_act_stall  <= 0;
            hs_act_idle   <= 0;
            hs_wgt_fire   <= 0;
            hs_wgt_stall  <= 0;
            hs_wgt_idle   <= 0;
            hs_psum_fire  <= 0;
            hs_psum_stall <= 0;
            hs_psum_idle  <= 0;
            true_fire_cnt <= 0;
        end else begin
            if      ( act_valid &&  act_ready) hs_act_fire  <= hs_act_fire  + 1;
            else if ( act_valid && !act_ready) hs_act_stall <= hs_act_stall + 1;
            else if (!act_valid &&  act_ready) hs_act_idle  <= hs_act_idle  + 1;

            if      ( wgt_valid &&  wgt_ready) hs_wgt_fire  <= hs_wgt_fire  + 1;
            else if ( wgt_valid && !wgt_ready) hs_wgt_stall <= hs_wgt_stall + 1;
            else if (!wgt_valid &&  wgt_ready) hs_wgt_idle  <= hs_wgt_idle  + 1;

            if      ( psum_out_valid &&  psum_in_ready) hs_psum_fire  <= hs_psum_fire  + 1;
            else if ( psum_out_valid && !psum_in_ready) hs_psum_stall <= hs_psum_stall + 1;
            else if (!psum_out_valid &&  psum_in_ready) hs_psum_idle  <= hs_psum_idle  + 1;
        end
    end

    // true_fire_cnt 单独一个 always_ff (与上面的 if-else 链隔离)
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            true_fire_cnt <= 0;
        end else if (act_valid && wgt_valid && can_advance) begin
            true_fire_cnt <= true_fire_cnt + 1;
        end
    end
    // synthesis translate_on

    // =========================================================================
    // NUM_COL 列 PE 计算
    // =========================================================================
    genvar c;
    generate
        for (c = 0; c < NUM_COL; c++) begin : gen_col
            logic [NUM_PE-1:0]             col_wrf_we;
            logic [NUM_PE*DATA_WIDTH-1:0]  col_wrf_wdata;
            logic signed [PSUM_WIDTH-1:0]  col_psum_out;

            always_comb begin
                col_wrf_we    = wrf_we   [c*NUM_PE            +: NUM_PE];
                col_wrf_wdata = wrf_wdata[c*NUM_PE*DATA_WIDTH +: NUM_PE*DATA_WIDTH];
            end

            assign psum_out_vec[c*PSUM_WIDTH +: PSUM_WIDTH] = col_psum_out;

            mac_col #(
                .NUM_PE    (NUM_PE),
                .DATA_WIDTH(DATA_WIDTH),
                .PSUM_WIDTH(PSUM_WIDTH),
                .WRF_DEPTH (WRF_DEPTH)
            ) u_col (
                .clk        (clk),
                .rst_n      (rst_n),
                .wrf_we     (col_wrf_we),
                .wrf_waddr  (wrf_waddr),
                .wrf_wdata  (col_wrf_wdata),
                .wrf_raddr  (wrf_raddr),
                .act_in_vec (act_in_vec),
                .compute_en (compute_en),
                .psum_out   (col_psum_out)
            );
        end
    endgenerate

endmodule
