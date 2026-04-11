`timescale 1ns/1ps

// 基础乘加单元 (Multiply-Accumulate Processing Element)
// 包含一个乘法器和一个局部的权重寄存器堆 (WRF)
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
    logic signed [DATA_WIDTH-1:0] wrf [0:WRF_DEPTH-1];
    
    // 权重写入逻辑 (时序逻辑)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < WRF_DEPTH; i++) begin
                wrf[i] <= '0;
            end
        end else if (wrf_we) begin
            wrf[wrf_waddr] <= signed'(wrf_wdata);
            $display("Time=%0t, PE written! waddr=%0d, wdata=%h", $time, wrf_waddr, wrf_wdata);
        end
    end
    
    // 权重读取逻辑 (组合逻辑)
    logic signed [DATA_WIDTH-1:0] active_weight;
    always_comb begin
        active_weight = wrf[wrf_raddr];
    end
    
    // 乘法器逻辑 (时序逻辑)
    // 乘法结果打一拍，提高频率，将关键路径切断
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prod_out <= '0;
        end else if (compute_en) begin
            prod_out <= act_in * active_weight;
        end else begin
            prod_out <= '0;
        end
    end

endmodule
