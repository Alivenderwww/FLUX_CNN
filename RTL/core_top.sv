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
    parameter int SRAM_DEPTH  = 1024,    // SRAM深度
    parameter int INST_DEPTH  = 1024     // 指令SRAM深度
)(
    input  logic                                clk,
    input  logic                                rst_n,
    
    // 外部配置接口
    input  logic                                start,        // 启动一次计算
    
    // 指令SRAM写入接口 (用于加载程序)
    input  logic                                inst_we_ext,
    input  logic [$clog2(INST_DEPTH)-1:0]       inst_waddr_ext,
    input  logic [63:0]                         inst_wdata_ext,
    
    // 外部SRAM写入接口 (用于测试加载)
    input  logic                                ifb_we_ext,
    input  logic [$clog2(SRAM_DEPTH)-1:0]       ifb_waddr_ext,
    input  logic [63:0]                         ifb_wdata_ext,
    
    input  logic                                wb_we_ext,
    input  logic [$clog2(SRAM_DEPTH)-1:0]       wb_waddr_ext,
    input  logic [511:0]                        wb_wdata_ext,
    
    // OFB读取接口 (用于测试提取结果)
    input  logic                                ofb_re_ext,
    input  logic [$clog2(SRAM_DEPTH)-1:0]       ofb_raddr_ext,
    output logic [63:0]                         ofb_rdata_ext,
    
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
    logic                                ofb_we;
    logic [$clog2(SRAM_DEPTH)-1:0]       ofb_waddr;
    logic                                sdp_en;
    logic [63:0]                         wrf_we;
    logic [$clog2(WRF_DEPTH)-1:0]        wrf_waddr;
    logic                                arf_we;
    logic [$clog2(ARF_DEPTH)-1:0]        arf_waddr;
    logic [$clog2(ARF_DEPTH)-1:0]        arf_read_addr;
    logic                                compute_en;
    logic [$clog2(WRF_DEPTH)-1:0]        wrf_raddr;
    logic [$clog2(PARF_DEPTH)-1:0]       parf_addr;
    logic                                parf_clear;
    logic                                parf_we;

    //=============================================================================
    // 2. SRAM 例化 (INST_SRAM, IFB & WB)
    //=============================================================================
    logic [63:0]  inst_rdata;
    logic [63:0]  ifb_rdata;
    logic [511:0] wb_rdata;
    logic [63:0]  ofb_wdata;
    
    logic                                inst_re;
    logic [$clog2(INST_DEPTH)-1:0]       inst_raddr;

    sram_model #(
        .DEPTH(INST_DEPTH),
        .DATA_WIDTH(64)
    ) u_inst_sram (
        .clk   (clk),
        .we    (inst_we_ext),
        .waddr (inst_waddr_ext),
        .wdata (inst_wdata_ext),
        .re    (inst_re),
        .raddr (inst_raddr),
        .rdata (inst_rdata)
    );
    
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

    sram_model #(
        .DEPTH(SRAM_DEPTH),
        .DATA_WIDTH(64)
    ) u_ofb (
        .clk   (clk),
        .we    (ofb_we),
        .waddr (ofb_waddr),
        .wdata (ofb_wdata),
        .re    (ofb_re_ext),
        .raddr (ofb_raddr_ext),
        .rdata (ofb_rdata_ext)
    );

    //=============================================================================
    // 3. ARF 例化 (Activation Register File)
    //=============================================================================
    logic [63:0] act_out_vec;
    logic [63:0] act_to_mac;   // ARF 输出 + LD32MAC bypass 前向路径

    std_rf #(
        .DEPTH(ARF_DEPTH),
        .DATA_WIDTH(NUM_PE*DATA_WIDTH)
    ) u_arf (
        .clk        (clk),
        .rst_n      (rst_n),
        .we         (arf_we),
        .waddr      (arf_waddr),
        .wdata      (ifb_rdata),
        .raddr      (arf_read_addr),
        .rdata      (act_out_vec)
    );

    // LD32MAC bypass：当 ARF 正在写入的地址与 MAC 读取地址相同时（读写同周期），
    // 直接将 ifb_rdata 前向传给 MAC，避免读到 std_rf 的旧值。
    // 其余情况（arf_we=0，或地址不同）正常走 ARF 输出。
    always_comb begin
        if (arf_we && (arf_waddr == arf_read_addr))
            act_to_mac = ifb_rdata;
        else
            act_to_mac = act_out_vec;
    end

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
        .act_in_vec (act_to_mac),    // ARF 输出（含 LD32MAC bypass）
        .compute_en (compute_en),
        
        .parf_addr  (parf_addr),
        .parf_clear (parf_clear),
        .parf_we    (parf_we),
        
        .psum_out_vec(psum_out_vec)
    );

    //=============================================================================
    // 5. SDP 单数据处理器例化
    //    输入: psum_out_vec (来自 MAC 阵列 PARF)
    //    输出: ofb_wdata (打包 uint8, 写入 OFB)
    //=============================================================================
    logic                  sdp_shift_we;
    logic [4:0]            sdp_shift_wdata;

    sdp #(
        .NUM_COL   (NUM_COL),
        .PSUM_WIDTH(PSUM_WIDTH)
    ) u_sdp (
        .clk          (clk),
        .rst_n        (rst_n),
        .shift_we     (sdp_shift_we),
        .shift_wdata  (sdp_shift_wdata),
        .psum_in      (psum_out_vec),
        .valid_in     (ofb_we),         // ofb_we 已含 d2 延迟，与 psum_out_vec 对齐
        .relu_en      (sdp_en),
        .ofm_data     (ofb_wdata),
        .valid_out    ()                // 当前等同于 ofb_we，暂不接回
    );

    //=============================================================================
    // 6. 控制器 例化
    //=============================================================================
    core_ctrl #(
        .WRF_DEPTH(WRF_DEPTH),
        .ARF_DEPTH(ARF_DEPTH),
        .PARF_DEPTH(PARF_DEPTH),
        .SRAM_DEPTH(SRAM_DEPTH),
        .INST_DEPTH(INST_DEPTH)
    ) u_ctrl (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start),
        
        .inst_re        (inst_re),
        .inst_addr      (inst_raddr),
        .inst_data      (core_isa_pkg::inst_t'(inst_rdata)),
        
        .wb_re          (wb_re),
        .wb_raddr       (wb_raddr),
        .ifb_re         (ifb_re),
        .ifb_raddr      (ifb_raddr),
        
        .ofb_we         (ofb_we),
        .ofb_waddr      (ofb_waddr),
        .sdp_en_out     (sdp_en),
        
        .wrf_we         (wrf_we),
        .wrf_waddr      (wrf_waddr),
        
        .arf_we         (arf_we),
        .arf_waddr      (arf_waddr),
        .arf_read_addr  (arf_read_addr),
        
        .compute_en     (compute_en),
        .wrf_raddr      (wrf_raddr),
        .parf_addr      (parf_addr),
        .parf_clear     (parf_clear),
        .parf_we        (parf_we),

        .sdp_shift_we   (sdp_shift_we),
        .sdp_shift_wdata(sdp_shift_wdata),

        .done           (done)
    );

endmodule
