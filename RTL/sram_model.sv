`timescale 1ns/1ps

// 行为级SRAM模型 (Behavioral SRAM Model)
// 用于模拟 IFB 和 WB，仅做功能验证使用。
module sram_model #(
    parameter int DEPTH      = 1024,     // 存储深度
    parameter int DATA_WIDTH = 64        // 位宽 (IFB: 64-bit, WB: 512-bit)
)(
    input  logic                            clk,
    input  logic                            we,        // 写使能
    input  logic [$clog2(DEPTH)-1:0]        waddr,     // 写地址
    input  logic [DATA_WIDTH-1:0]           wdata,     // 写数据
    
    input  logic                            re,        // 读使能
    input  logic [$clog2(DEPTH)-1:0]        raddr,     // 读地址
    output logic [DATA_WIDTH-1:0]           rdata      // 读数据
);

    logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    
    // 同步写
    always @(posedge clk) begin
        if (we) begin
            mem[waddr] <= wdata;
        end
    end
    
    // 同步读 (修改为标准的1拍读延迟模型)
    always_ff @(posedge clk) begin
        if (re) begin
            rdata <= mem[raddr];
        end else begin
            rdata <= '0;
        end
    end

endmodule
