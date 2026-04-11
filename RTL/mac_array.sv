`timescale 1ns/1ps

// 核心阵列 (8x8 MAC Array)
// 包含8列PE，每列负责一个输出通道。单周期计算8个输入通道，输出8个输出通道的部分和。
module mac_array #(
    parameter int NUM_COL     = 8,       // 列数 (对应输出通道数)
    parameter int NUM_PE      = 8,       // 每列PE数 (对应输入通道数)
    parameter int DATA_WIDTH  = 8,       // 数据位宽
    parameter int PSUM_WIDTH  = 32,      // 累加器输出位宽
    parameter int WRF_DEPTH   = 32,      // 权重RF深度
    parameter int PARF_DEPTH  = 32       // PARF深度
)(
    input  logic                                clk,
    input  logic                                rst_n,
    
    // 权重配置接口 (每个PE独立写入)
    input  logic [NUM_COL*NUM_PE-1:0]           wrf_we,       // WRF写使能 (一维平铺, 64-bit)
    input  logic [$clog2(WRF_DEPTH)-1:0]        wrf_waddr,    // WRF写地址
    input  logic [NUM_COL*NUM_PE*DATA_WIDTH-1:0]wrf_wdata,    // WRF写数据 (平铺的64个权重, 64x8=512-bit)
    
    // 激活值输入 (广播给所有列)
    input  logic [$clog2(WRF_DEPTH)-1:0]        wrf_raddr,    // 读地址 (当前使用的权重索引)
    input  logic [NUM_PE*DATA_WIDTH-1:0]        act_in_vec,   // 8通道激活输入 (64-bit)
    input  logic                                compute_en,   // 计算使能信号
    
    // PARF接口 (对所有列共享控制)
    input  logic [$clog2(PARF_DEPTH)-1:0]       parf_addr,    // 累加地址 (对应32个空间像素)
    input  logic                                parf_clear,   // 是否清零PARF累加
    input  logic                                parf_we,      // 写回使能
    
    // 输出接口
    output logic signed [NUM_COL*PSUM_WIDTH-1:0] psum_out_vec // 8通道的32-bit累加结果平铺 (256-bit)
);

    genvar c;
    generate
        for (c = 0; c < NUM_COL; c++) begin : gen_col
            // 提取对应列的写使能和数据
            logic [NUM_PE-1:0]                   col_wrf_we;
            logic [NUM_PE*DATA_WIDTH-1:0]        col_wrf_wdata;
            logic signed [PSUM_WIDTH-1:0]        col_psum_out;
            
            always_comb begin
                col_wrf_we    = wrf_we[c*NUM_PE +: NUM_PE];
                col_wrf_wdata = wrf_wdata[c*NUM_PE*DATA_WIDTH +: NUM_PE*DATA_WIDTH];
            end
            
            // 将对应列的输出拼接到总输出向量中
            assign psum_out_vec[c*PSUM_WIDTH +: PSUM_WIDTH] = col_psum_out;
            
            mac_col #(
                .NUM_PE     (NUM_PE),
                .DATA_WIDTH (DATA_WIDTH),
                .PSUM_WIDTH (PSUM_WIDTH),
                .WRF_DEPTH  (WRF_DEPTH),
                .PARF_DEPTH (PARF_DEPTH)
            ) u_col (
                .clk        (clk),
                .rst_n      (rst_n),
                .wrf_we     (col_wrf_we),
                .wrf_waddr  (wrf_waddr),
                .wrf_wdata  (col_wrf_wdata),
                
                // 计算接口 (激活输入广播)
                .wrf_raddr  (wrf_raddr),
                .act_in_vec (act_in_vec),
                .compute_en (compute_en),
                
                // PARF控制 (全阵列同步)
                .parf_addr  (parf_addr),
                .parf_clear (parf_clear),
                .parf_we    (parf_we),
                
                .psum_out   (col_psum_out)
            );
        end
    endgenerate

endmodule
