`timescale 1ns/1ps

// 核心乘加单元 (Multiply-Accumulate Processing Element)
// 包含一个乘法器和一个局部权重寄存器堆 (WRF)
module mac_pe #(
    parameter int DATA_WIDTH = 8,  // 输入数据和权重的位宽
    parameter int WRF_DEPTH  = 32  // 权重寄存器堆的深度
)(
    input  logic                               clk,
    input  logic                               rst_n,
    
    // 权重配置接口
    input  logic                               wrf_we,      // WRF写使能
    input  logic [$clog2(WRF_DEPTH)-1:0]       wrf_waddr,   // WRF写地址
    input  logic [DATA_WIDTH-1:0]              wrf_wdata,   // WRF写数据
    
    // 计算接口
    input  logic [$clog2(WRF_DEPTH)-1:0]       wrf_raddr,   // WRF读地址 (当前使用的权重索引)
    input  logic signed [DATA_WIDTH-1:0]       act_in,      // 激活值输入 (8-bit)
    input  logic                               compute_en,  // 计算使能信号
    
    // 输出接口
    output logic signed [2*DATA_WIDTH-1:0]     prod_out     // 乘积输出 (16-bit)
);

    // 权重寄存器堆 (Weight Register File)
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
    
    // 当前激活的权重 (用于计算)
    logic signed [DATA_WIDTH-1:0] active_weight;
    always_comb begin
        active_weight = signed'(active_weight_us);
    end
    
    // 乘法器逻辑 (时序逻辑)
    // 乘法打一拍，减轻组合逻辑关键路径判断
    // synthesis translate_off
    int mac_ops_cnt = 0;
    always_ff @(posedge clk) begin
        if (rst_n && compute_en) begin
            mac_ops_cnt++;
        end
    end
    // synthesis translate_on
    
    // compute_en 在新架构里相当于 global-stall pipeline 的 advance 信号。
    // 不再把未使能拍清零 — stall 时保持 prod_out，让已经计算的乘积顺着 pipe 继续流。
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)           prod_out <= '0;
        else if (compute_en)  prod_out <= act_in * active_weight;
    end

endmodule
