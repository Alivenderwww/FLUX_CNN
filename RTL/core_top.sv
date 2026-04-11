`timescale 1ns/1ps

// 核心顶层模块 (Core Top)
// 将 8x8 MAC阵列、ARF、WB(SRAM)、IFB(SRAM) 和 控制器连线
module core_top #(
    parameter int NUM_COL     = 8,       // 列数
    parameter int NUM_PE      = 8,       // 每列PE数
    parameter int DATA_WIDTH  = 8,       // 数据位宽
    parameter int PSUM_WIDTH  = 32,      // Psum位宽
    parameter int WRF_DEPTH   = 32,      // 权重RF深度
    parameter int ARF_DEPTH   = 32,      // 激活值RF深度
    parameter int PARF_DEPTH  = 32,      // PARF深度
    parameter int SRAM_DEPTH  = 1024     // SRAM深度
)(
    input  logic                                clk,
    input  logic                                rst_n,
    
    // 外部配置接口
    input  logic                                start,        // 启动一次计算
    input  logic [3:0]                          kernel_size,  // 卷积核大小 (如 9)
    input  logic [3:0]                          kernel_dim,   // 卷积核边长 (如 3)
    input  logic [15:0]                         image_width,  // 图像宽度
    input  logic [7:0]                          num_pixels,   // 一批次计算像素数 (如 32)
    
    // 外部SRAM写入接口 (用于测试加载)
    input  logic                                ifb_we_ext,
    input  logic [$clog2(SRAM_DEPTH)-1:0]       ifb_waddr_ext,
    input  logic [63:0]                         ifb_wdata_ext,
    
    input  logic                                wb_we_ext,
    input  logic [$clog2(SRAM_DEPTH)-1:0]       wb_waddr_ext,
    input  logic [511:0]                        wb_wdata_ext,
    
    // 输出
    output logic signed [NUM_COL*PSUM_WIDTH-1:0] psum_out_vec,
    output logic                                 done
);

    //=============================================================================
    // 1. 控制器信号线
    //=============================================================================
    logic                                wb_re;
    logic [$clog2(SRAM_DEPTH)-1:0]       wb_raddr;
    logic                                ifb_re;
    logic [$clog2(SRAM_DEPTH)-1:0]       ifb_raddr;
    logic [63:0]                         wrf_we;
    logic [$clog2(WRF_DEPTH)-1:0]        wrf_waddr;
    logic                                arf_shift_en;
    logic                                arf_flush_en;
    logic [$clog2(ARF_DEPTH)-1:0]        arf_flush_addr;
    logic [$clog2(ARF_DEPTH)-1:0]        arf_read_addr;
    logic                                compute_en;
    logic [$clog2(WRF_DEPTH)-1:0]        wrf_raddr;
    logic [$clog2(PARF_DEPTH)-1:0]       parf_addr;
    logic                                parf_clear;
    logic                                parf_we;

    //=============================================================================
    // 2. SRAM 例化 (IFB & WB)
    //=============================================================================
    logic [63:0]  ifb_rdata;
    logic [511:0] wb_rdata;
    
    sram_model #(
        .DEPTH(SRAM_DEPTH),
        .DATA_WIDTH(64)
    ) u_ifb (
        .clk   (clk),
        .we    (ifb_we_ext),
        .waddr (ifb_waddr_ext),
        .wdata (ifb_wdata_ext),
        .re    (ifb_re),
        .raddr (ifb_raddr),
        .rdata (ifb_rdata)
    );
    
    sram_model #(
        .DEPTH(SRAM_DEPTH),
        .DATA_WIDTH(512)
    ) u_wb (
        .clk   (clk),
        .we    (wb_we_ext),
        .waddr (wb_waddr_ext),
        .wdata (wb_wdata_ext),
        .re    (wb_re),
        .raddr (wb_raddr),
        .rdata (wb_rdata)
    );

    //=============================================================================
    // 3. ARF 例化 (Activation Register File)
    //=============================================================================
    logic [63:0] act_out_vec;
    
    arf_buffer #(
        .ARF_DEPTH(ARF_DEPTH),
        .DATA_WIDTH(64)
    ) u_arf (
        .clk           (clk),
        .rst_n         (rst_n),
        .arf_shift_en  (arf_shift_en),
        .new_pixel_in  (ifb_rdata),       // 从IFB读取的新像素补入末端
        .arf_flush_en  (arf_flush_en),
        .arf_flush_addr(arf_flush_addr),
        .arf_flush_data(ifb_rdata),       // 刷新写入的初始像素数据
        .arf_read_addr (arf_read_addr),
        .act_out_vec   (act_out_vec)
    );

    //=============================================================================
    // 4. MAC阵列 例化
    //=============================================================================
    mac_array #(
        .NUM_COL(NUM_COL),
        .NUM_PE(NUM_PE),
        .DATA_WIDTH(DATA_WIDTH),
        .PSUM_WIDTH(PSUM_WIDTH),
        .WRF_DEPTH(WRF_DEPTH),
        .PARF_DEPTH(PARF_DEPTH)
    ) u_mac_array (
        .clk        (clk),
        .rst_n      (rst_n),
        
        .wrf_we     (wrf_we),
        .wrf_waddr  (wrf_waddr),
        .wrf_wdata  (wb_rdata),       // 从Weight Buffer平铺写入
        
        .wrf_raddr  (wrf_raddr),
        .act_in_vec (act_out_vec),    // 从ARF读取的8通道输入
        .compute_en (compute_en),
        
        .parf_addr  (parf_addr),
        .parf_clear (parf_clear),
        .parf_we    (parf_we),
        
        .psum_out_vec(psum_out_vec)
    );

    //=============================================================================
    // 5. 控制器 例化
    //=============================================================================
    core_ctrl #(
        .WRF_DEPTH(WRF_DEPTH),
        .ARF_DEPTH(ARF_DEPTH),
        .PARF_DEPTH(PARF_DEPTH),
        .SRAM_DEPTH(SRAM_DEPTH)
    ) u_ctrl (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start),
        .kernel_size    (kernel_size),
        .kernel_dim     (kernel_dim),
        .image_width    (image_width),
        .num_pixels     (num_pixels),
        
        .wb_re          (wb_re),
        .wb_raddr       (wb_raddr),
        .ifb_re         (ifb_re),
        .ifb_raddr      (ifb_raddr),
        
        .wrf_we         (wrf_we),
        .wrf_waddr      (wrf_waddr),
        
        .arf_shift_en   (arf_shift_en),
        .arf_flush_en   (arf_flush_en),
        .arf_flush_addr (arf_flush_addr),
        .arf_read_addr  (arf_read_addr),
        
        .compute_en     (compute_en),
        .wrf_raddr      (wrf_raddr),
        .parf_addr      (parf_addr),
        .parf_clear     (parf_clear),
        .parf_we        (parf_we),
        
        .done           (done)
    );

endmodule
