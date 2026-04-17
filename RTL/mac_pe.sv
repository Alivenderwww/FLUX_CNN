`timescale 1ns/1ps

// 核心乘加单元 (Multiply-Accumulate Processing Element)
// 包含一个乘法器和一个局部权重寄存器堆 (WRF)
//
// prod_out 是数据路径：compute_en 未拉高时保持；上电 X 不影响下游（上游 valid
// 会在 mac_col 的加法树寄存器起始阶段遮蔽无效值）。按 §6 不加复位。
module mac_pe #(
    parameter int DATA_WIDTH = 8,
    parameter int WRF_DEPTH  = 32
)(
    input  logic                               clk,
    input  logic                               rst_n,

    input  logic                               wrf_we,
    input  logic [$clog2(WRF_DEPTH)-1:0]       wrf_waddr,
    input  logic [DATA_WIDTH-1:0]              wrf_wdata,

    input  logic [$clog2(WRF_DEPTH)-1:0]       wrf_raddr,
    input  logic signed [DATA_WIDTH-1:0]       act_in,
    input  logic                               compute_en,

    output logic signed [2*DATA_WIDTH-1:0]     prod_out
);

    logic [DATA_WIDTH-1:0] active_weight_us;

    std_rf #(
        .DATA_WIDTH(DATA_WIDTH),
        .DEPTH(WRF_DEPTH)
    ) u_wrf (
        .clk   (clk),
        .rst_n (rst_n),
        .we    (wrf_we),
        .waddr (wrf_waddr),
        .wdata (wrf_wdata),
        .raddr (wrf_raddr),
        .rdata (active_weight_us)
    );

    logic signed [DATA_WIDTH-1:0] active_weight;
    always_comb begin
        active_weight = signed'(active_weight_us);
    end

    // 乘法打一拍，减轻组合逻辑关键路径。
    // compute_en 相当于 pipeline advance 闸门：stall 时保持 prod_out，让已计算
    // 的乘积顺着 pipe 继续流。
    always_ff @(posedge clk) begin
        if (compute_en) prod_out <= act_in * active_weight;
        else            prod_out <= prod_out;
    end

endmodule
