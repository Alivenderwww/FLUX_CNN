`timescale 1ns/1ps

// 核心计算列 (Column of PEs)
// 包含8个PE，一个3级加法树，和一个用于跨周期累加的深度为32的寄存器堆 (PARF)
module mac_col #(
    parameter int NUM_PE      = 8,       // 一列的PE数量 (输入通道数)
    parameter int DATA_WIDTH  = 8,       // 输入数据位宽
    parameter int PROD_WIDTH  = 16,      // 乘积位宽
    parameter int PSUM_WIDTH  = 32,      // 累加器位宽
    parameter int WRF_DEPTH   = 32,      // 每个PE内WRF深度
    parameter int PARF_DEPTH  = 32       // PARF寄存器堆深度
)(
    input  logic                                clk,
    input  logic                                rst_n,
    
    // 权重配置接口 (广播给所有PE)
    input  logic [NUM_PE-1:0]                   wrf_we,       // 对应每个PE的写使能
    input  logic [$clog2(WRF_DEPTH)-1:0]        wrf_waddr,    // 写入地址 (所有PE共享)
    input  logic [NUM_PE*DATA_WIDTH-1:0]        wrf_wdata,    // 写入数据向量 (针对每个PE不同数据)
    
    // 计算控制接口
    input  logic [$clog2(WRF_DEPTH)-1:0]        wrf_raddr,    // 读地址 (读取同一组权重)
    input  logic signed [NUM_PE*DATA_WIDTH-1:0] act_in_vec,   // 8个通道激活值向量
    input  logic                                compute_en,   // 计算使能信号 (作用于乘法和加法)
    
    // PARF接口
    input  logic [$clog2(PARF_DEPTH)-1:0]       parf_addr,    // 累加地址 (PARF当前累加位置)
    input  logic                                parf_clear,   // 是否清空当前PARF地址的历史值
    input  logic                                parf_we,      // PARF写回使能
    
    // 输出结果 (可送入SDP)
    output logic signed [PSUM_WIDTH-1:0]        psum_out      // 列的输出结果 (32-bit)
);

    //=============================================================================
    // 1. 乘法器阵列实例化 (PE Array)
    //=============================================================================
    logic signed [PROD_WIDTH-1:0] prod_array [0:NUM_PE-1];
    
    genvar i;
    generate
        for (i = 0; i < NUM_PE; i++) begin : gen_pe
            mac_pe #(
                .DATA_WIDTH(DATA_WIDTH),
                .WRF_DEPTH (WRF_DEPTH )
            ) u_pe (
                .clk       (clk),
                .rst_n     (rst_n),
                
                // 权重写接口拆解
                .wrf_we    (wrf_we[i]),
                .wrf_waddr (wrf_waddr),
                .wrf_wdata (wrf_wdata[i*DATA_WIDTH +: DATA_WIDTH]),
                
                // 读接口和计算
                .wrf_raddr (wrf_raddr),
                .act_in    (act_in_vec[i*DATA_WIDTH +: DATA_WIDTH]), // 截取对应通道输入
                .compute_en(compute_en),
                
                // 乘积输出
                .prod_out  (prod_array[i])
            );
        end
    endgenerate

    //=============================================================================
    // 2. 空间加法树 (Spatial Adder Tree) - 将NUM_PE个通道的部分和相加
    // 参数化循环实现，支持任意 NUM_PE
    //=============================================================================
    logic signed [PSUM_WIDTH-1:0] adder_tree_out;

    always_comb begin
        adder_tree_out = '0;
        for (int p = 0; p < NUM_PE; p++) begin
            adder_tree_out = adder_tree_out + $signed(prod_array[p]);
        end
    end

    // 打一拍以对齐乘法器延时 (Pipeline)
    logic signed [PSUM_WIDTH-1:0] adder_tree_reg;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            adder_tree_reg <= '0;
        end else begin
            adder_tree_reg <= adder_tree_out;
        end
    end
    
    //=============================================================================
    // 3. Psum累加与 PARF (Psum Adder RF)
    //=============================================================================
    logic [PSUM_WIDTH-1:0] parf_rdata;
    
    // 读地址和写地址保持一致 (就地累加)
    logic [$clog2(PARF_DEPTH)-1:0] parf_addr_d1, parf_addr_reg; // 延迟信号以对齐计算流水线
    logic parf_we_d1, parf_we_reg;
    logic parf_clear_d1, parf_clear_reg;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            parf_addr_d1   <= '0;
            parf_we_d1     <= 1'b0;
            parf_clear_d1  <= 1'b0;
            
            parf_addr_reg  <= '0;
            parf_we_reg    <= 1'b0;
            parf_clear_reg <= 1'b0;
        end else begin
            // 延迟1拍
            parf_addr_d1   <= parf_addr;
            parf_we_d1     <= parf_we;
            parf_clear_d1  <= parf_clear;
            
            // 延迟2拍 (对齐adder_tree_reg)
            parf_addr_reg  <= parf_addr_d1;
            parf_we_reg    <= parf_we_d1;
            parf_clear_reg <= parf_clear_d1;
        end
    end

    // 时间累加器声明
    logic signed [PSUM_WIDTH-1:0] accum_out;

    // 实例化标准RF作为PARF存储
    std_rf #(
        .DATA_WIDTH(PSUM_WIDTH),
        .DEPTH(PARF_DEPTH)
    ) u_parf (
        .clk   (clk),
        .rst_n (rst_n),
        .we    (parf_we_reg),
        .waddr (parf_addr_reg),
        .wdata (accum_out),
        .raddr (parf_addr_reg),
        .rdata (parf_rdata)
    );

    // 读取PARF旧值组合逻辑
    logic signed [PSUM_WIDTH-1:0] parf_old_val;
    always_comb begin
        if (parf_clear_reg) begin
            parf_old_val = '0; // 换层或通道块刷新时，历史值为0
        end else begin
            parf_old_val = signed'(parf_rdata);
        end
    end
    
    // 时间累加 (Temporal Accumulation)
    always_comb begin
        accum_out = parf_old_val + adder_tree_reg;
    end
    
    // 列输出 (直接连接最后计算结果或PARF特定地址)
    always_comb begin
        psum_out = signed'(parf_rdata); 
    end

endmodule
