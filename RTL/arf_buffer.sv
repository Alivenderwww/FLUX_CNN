`timescale 1ns/1ps

// 激活值寄存器堆 (Activation Register File - ARF)
// 一个移位寄存器堆，用于行内复用。
module arf_buffer #(
    parameter int ARF_DEPTH   = 32,      // ARF深度 (即一次连续计算的像素数)
    parameter int DATA_WIDTH  = 64       // 每次移动的数据位宽 (8个通道 * 8 bit)
)(
    input  logic                               clk,
    input  logic                               rst_n,
    
    // IFB接口 (单像素补给)
    input  logic                               arf_shift_en,  // 移位使能
    input  logic [DATA_WIDTH-1:0]              new_pixel_in,  // 从IFB读取的新像素 (8通道)
    
    // 全局刷新接口 (换行时)
    input  logic                               arf_flush_en,  // 全局刷新使能
    input  logic [$clog2(ARF_DEPTH)-1:0]       arf_flush_addr,// 刷新写入地址
    input  logic [DATA_WIDTH-1:0]              arf_flush_data,// 刷新写入数据
    
    // 阵列计算接口 (吐出滑动窗口末端的数据)
    input  logic [$clog2(ARF_DEPTH)-1:0]       arf_read_addr, // 读地址 (计算阶段读取当前周期的像素)
    output logic [DATA_WIDTH-1:0]              act_out_vec    // 送给MAC阵列的8通道像素
);

    // 寄存器堆内存
    logic [DATA_WIDTH-1:0] arf_ram [0:ARF_DEPTH-1];
    
    // ARF写入逻辑 (支持移位与绝对地址刷新)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < ARF_DEPTH; i++) begin
                arf_ram[i] <= '0;
            end
        end else if (arf_flush_en) begin
            // 换行刷新，直接覆盖对应地址
            arf_ram[arf_flush_addr] <= arf_flush_data;
        end else if (arf_shift_en) begin
            // 行内移位，所有数据前移，最新数据填入最末端 (索引ARF_DEPTH-1)
            for (int i = 0; i < ARF_DEPTH-1; i++) begin
                arf_ram[i] <= arf_ram[i+1];
            end
            arf_ram[ARF_DEPTH-1] <= new_pixel_in;
        end
    end
    
    // ARF读取逻辑 (组合逻辑送出，支持写入当拍的前馈 Bypass)
    always_comb begin
        if (arf_flush_en && (arf_flush_addr == arf_read_addr)) begin
            // 当流水线中“读SRAM、写ARF、并同时计算”在同一个周期发生时，直接把数据前馈给 MAC
            act_out_vec = arf_flush_data;
        end else begin
            act_out_vec = arf_ram[arf_read_addr];
        end
    end

endmodule
