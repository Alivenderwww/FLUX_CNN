`timescale 1ns/1ps

// 核心计算列 (Column of PEs) - 纯计算版本
// 包含 NUM_PE 个 PE、加法树、加法树流水寄存器。
//
// 相比原版本：PARF 已被外置到 parf_accum 模块 (特殊 FIFO)。
// 本模块只输出已累加乘积 (adder_tree_reg)，由上层 parf_accum 负责时间累加。
//
// 时序：compute_en (T) → prod_out (T+1) → adder_tree_reg (T+2)
// 所以 psum_out 相对 compute_en 有 2 拍固定延迟。
module mac_col #(
    parameter int NUM_PE      = 16,
    parameter int DATA_WIDTH  = 8,
    parameter int PROD_WIDTH  = 16,
    parameter int PSUM_WIDTH  = 32,
    parameter int WRF_DEPTH   = 32
)(
    input  logic                                clk,
    input  logic                                rst_n,

    // 权重配置接口 (广播给所有 PE)
    input  logic [NUM_PE-1:0]                   wrf_we,
    input  logic [$clog2(WRF_DEPTH)-1:0]        wrf_waddr,
    input  logic [NUM_PE*DATA_WIDTH-1:0]        wrf_wdata,

    // 计算控制接口
    input  logic [$clog2(WRF_DEPTH)-1:0]        wrf_raddr,
    input  logic signed [NUM_PE*DATA_WIDTH-1:0] act_in_vec,
    input  logic                                compute_en,

    // 输出结果 (纯 NUM_PE 加和，未经时间累加)
    output logic signed [PSUM_WIDTH-1:0]        psum_out
);

    //=========================================================================
    // 1. 乘法器阵列
    //=========================================================================
    logic signed [PROD_WIDTH-1:0] prod_array [0:NUM_PE-1];

    genvar i;
    generate
        for (i = 0; i < NUM_PE; i++) begin : gen_pe
            mac_pe #(
                .DATA_WIDTH(DATA_WIDTH),
                .WRF_DEPTH (WRF_DEPTH)
            ) u_pe (
                .clk       (clk),
                .rst_n     (rst_n),
                .wrf_we    (wrf_we[i]),
                .wrf_waddr (wrf_waddr),
                .wrf_wdata (wrf_wdata[i*DATA_WIDTH +: DATA_WIDTH]),
                .wrf_raddr (wrf_raddr),
                .act_in    (act_in_vec[i*DATA_WIDTH +: DATA_WIDTH]),
                .compute_en(compute_en),
                .prod_out  (prod_array[i])
            );
        end
    endgenerate

    //=========================================================================
    // 2. 空间加法树 - NUM_PE 个乘积求和
    //=========================================================================
    logic signed [PSUM_WIDTH-1:0] adder_tree_out;

    always_comb begin
        adder_tree_out = '0;
        for (int p = 0; p < NUM_PE; p++) begin
            adder_tree_out = adder_tree_out + $signed(prod_array[p]);
        end
    end

    // 打一拍 (对齐 mac_pe 的 1 拍乘法器延迟)
    // compute_en 作为 pipeline advance 闸门：stall 时保持 adder_tree_reg。
    logic signed [PSUM_WIDTH-1:0] adder_tree_reg;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)           adder_tree_reg <= '0;
        else if (compute_en)  adder_tree_reg <= adder_tree_out;
    end

    assign psum_out = adder_tree_reg;

endmodule
