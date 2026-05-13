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

    // VD100 isolate sim 2026-05-14: 乘法换成 act_in passthrough (省 DSP + 加速综合)
    // 目的: bypass mac_array DSP 让 P&R 快得多, 隔离测试 axi 流水线 stuck. 实际 layer 0
    // 卡在 ODMA S_TX 等 axi_dm s2mm, 跟 mac_array 计算结果无关. OFM 数据是 garbage
    // 但 cmd 序列 + AW address pattern 跟原 design 一致.
    // 之前: if (compute_en) prod_out <= act_in * active_weight;
`ifdef FLUX_MAC_BYPASS
    always_ff @(posedge clk) begin
        if (compute_en) prod_out <= {{DATA_WIDTH{1'b0}}, act_in};   // sign-extended passthrough
        else            prod_out <= prod_out;
    end
    // 避免 unused warning
    wire _unused_weight = |active_weight;
`else
    always_ff @(posedge clk) begin
        if (compute_en) prod_out <= act_in * active_weight;
        else            prod_out <= prod_out;
    end
`endif

endmodule
