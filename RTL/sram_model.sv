`timescale 1ns/1ps

// 行为级 SRAM 模型 (Behavioral SRAM Model)
// 用于模拟 IFB 和 WB，仅做功能验证使用。
//
// 存储阵列 mem 是数据路径，无复位：上游 we 未拉高前任何读出值都被下游 gate。
// 同步读（1 拍读延迟）：rdata 寄存器也按数据路径处理，无复位。
module sram_model #(
    parameter int DEPTH      = 1024,
    parameter int DATA_WIDTH = 64,
    // ADDR_W 默认跟 DEPTH 强绑定; caller 可显式传更宽 (e.g. = AW shared bus)
    // 让 SHB / IFB 用同一个 sram_model 但物理 depth 不同时端口宽度可独立配置.
    parameter int ADDR_W     = $clog2(DEPTH)
)(
    input  logic                            clk,
    input  logic                            we,
    input  logic [ADDR_W-1:0]               waddr,
    input  logic [DATA_WIDTH-1:0]           wdata,

    input  logic                            re,
    input  logic [ADDR_W-1:0]               raddr,
    output logic [DATA_WIDTH-1:0]           rdata
);
    // 内部用 $clog2(DEPTH) 索引; 多余高位 (ADDR_W > $clog2(DEPTH)) 自动截掉
    localparam int INT_AW = $clog2(DEPTH);
    logic [INT_AW-1:0] waddr_int;
    logic [INT_AW-1:0] raddr_int;
    assign waddr_int = waddr[INT_AW-1:0];
    assign raddr_int = raddr[INT_AW-1:0];

    logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // synthesis translate_off
    int sram_read_cnt;
    int sram_write_cnt;

    always_ff @(posedge clk) begin
        if (we) sram_write_cnt <= sram_write_cnt + 1;
        else    sram_write_cnt <= sram_write_cnt;
    end

    always_ff @(posedge clk) begin
        if (re) sram_read_cnt <= sram_read_cnt + 1;
        else    sram_read_cnt <= sram_read_cnt;
    end
    // synthesis translate_on

    // 同步写
    always_ff @(posedge clk) begin
        if (we) mem[waddr_int] <= wdata;
        else    mem[waddr_int] <= mem[waddr_int];
    end

    // 同步读 (1 拍延迟)
    always_ff @(posedge clk) begin
        if (re) rdata <= mem[raddr_int];
        else    rdata <= '0;
    end

endmodule
