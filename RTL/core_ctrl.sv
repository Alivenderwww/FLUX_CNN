`timescale 1ns/1ps

// 核心状态机/控制器 (Core Controller)
// 负责数据流的总体调度 (Order 2: 空间交织流水线)
module core_ctrl #(
    parameter int WRF_DEPTH   = 32,      // 权重RF深度
    parameter int ARF_DEPTH   = 32,      // 激活值RF深度
    parameter int PARF_DEPTH  = 32,      // PARF深度
    parameter int SRAM_DEPTH  = 1024     // IFB和WB深度
)(
    input  logic                                clk,
    input  logic                                rst_n,
    
    // 配置信号 (从顶层控制)
    input  logic                                start,        // 启动计算信号
    input  logic [3:0]                          kernel_size,  // 卷积核大小 (如 3x3=9)
    input  logic [7:0]                          num_pixels,   // 连续计算像素数 (通常等于 PARF_DEPTH)
    
    // SRAM 控制接口
    output logic                                wb_re,        // Weight Buffer 读使能
    output logic [$clog2(SRAM_DEPTH)-1:0]       wb_raddr,     // 权重读取地址
    
    output logic                                ifb_re,       // IFB 读使能
    output logic [$clog2(SRAM_DEPTH)-1:0]       ifb_raddr,    // IFB 读地址
    
    // WRF 控制接口
    output logic [63:0]                         wrf_we,       // 所有PE的写使能
    output logic [$clog2(WRF_DEPTH)-1:0]        wrf_waddr,    // 写地址
    
    // ARF 控制接口
    output logic                                arf_shift_en, // 移位使能
    output logic                                arf_flush_en, // 刷新使能
    output logic [$clog2(ARF_DEPTH)-1:0]        arf_flush_addr,// 刷新地址
    output logic [$clog2(ARF_DEPTH)-1:0]        arf_read_addr, // 读出到MAC的地址
    
    // MAC & PARF 控制接口
    output logic                                compute_en,   // MAC计算使能
    output logic [$clog2(WRF_DEPTH)-1:0]        wrf_raddr,    // MAC读取WRF地址
    output logic [$clog2(PARF_DEPTH)-1:0]       parf_addr,    // PARF当前累加位置
    output logic                                parf_clear,   // 是否清零PARF
    output logic                                parf_we,      // 写回使能
    
    // 状态指示
    output logic                                done          // 一次 Base Tile 完成
);

    //=============================================================================
    // 1. 状态机定义
    //=============================================================================
    typedef enum logic [2:0] {
        IDLE,           // 等待开始
        LOAD_WEIGHTS,   // 将所有 3x3 权重从 WB 搬运到 WRF (9 个周期)
        FLUSH_ARF,      // 换行刷新，从 IFB 读取首批 32 个像素 (32 个周期)
        COMPUTE_TILE    // 计算基础块，固定权重算 32 像素，切换权重算 9 轮
    } state_t;
    
    state_t current_state, next_state;
    
    // 状态寄存器
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
        end
    end

    //=============================================================================
    // 2. 内部计数器
    //=============================================================================
    logic [3:0]  weight_cnt;      // 记录已加载/计算的权重位置 (0~8)
    logic [7:0]  pixel_cnt;       // 记录已读取/计算的像素数 (0~31)
    
    // 计数器逻辑
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            weight_cnt <= '0;
            pixel_cnt  <= '0;
        end else begin
            case (current_state)
                IDLE: begin
                    weight_cnt <= '0;
                    pixel_cnt  <= '0;
                end
                
                LOAD_WEIGHTS: begin
                    // 每个周期加载一个位置的权重
                    if (weight_cnt == kernel_size - 1) begin
                        weight_cnt <= '0;
                    end else begin
                        weight_cnt <= weight_cnt + 1'b1;
                    end
                end
                
                FLUSH_ARF: begin
                    // 从IFB读取32个像素
                    if (pixel_cnt == num_pixels - 1) begin
                        pixel_cnt <= '0;
                    end else begin
                        pixel_cnt <= pixel_cnt + 1'b1;
                    end
                end
                
                COMPUTE_TILE: begin
                    // 嵌套循环：外层权重(0~8)，内层像素(0~31)
                    if (pixel_cnt == num_pixels - 1) begin
                        pixel_cnt <= '0;
                        if (weight_cnt == kernel_size - 1) begin
                            weight_cnt <= '0;
                        end else begin
                            weight_cnt <= weight_cnt + 1'b1;
                        end
                    end else begin
                        pixel_cnt <= pixel_cnt + 1'b1;
                    end
                end
                
                default: begin
                    weight_cnt <= '0;
                    pixel_cnt  <= '0;
                end
            endcase
        end
    end

    //=============================================================================
    // 3. 状态转移组合逻辑
    //=============================================================================
    always_comb begin
        next_state = current_state; // 默认保持当前状态
        
        case (current_state)
            IDLE: begin
                if (start) begin
                    next_state = LOAD_WEIGHTS;
                end
            end
            
            LOAD_WEIGHTS: begin
                // 如果权重加载完成，进入刷新ARF阶段
                if (weight_cnt == kernel_size - 1) begin
                    next_state = FLUSH_ARF;
                end
            end
            
            FLUSH_ARF: begin
                // 如果初次32个像素读取完成，进入计算
                if (pixel_cnt == num_pixels - 1) begin
                    next_state = COMPUTE_TILE;
                end
            end
            
            COMPUTE_TILE: begin
                // 32 像素 x 9 个位置计算完成，回到IDLE
                if ((pixel_cnt == num_pixels - 1) && (weight_cnt == kernel_size - 1)) begin
                    next_state = IDLE;
                end
            end
            
            default: next_state = IDLE;
        endcase
    end

    //=============================================================================
    // 4. 控制信号输出逻辑
    //=============================================================================
    always_comb begin
        // 默认初始化所有输出信号
        wb_re          = 1'b0;
        wb_raddr       = '0;
        
        ifb_re         = 1'b0;
        ifb_raddr      = '0;
        
        wrf_we         = '0;
        wrf_waddr      = '0;
        
        arf_shift_en   = 1'b0;
        arf_flush_en   = 1'b0;
        arf_flush_addr = '0;
        arf_read_addr  = '0;
        
        compute_en     = 1'b0;
        wrf_raddr      = '0;
        parf_addr      = '0;
        parf_clear     = 1'b0;
        parf_we        = 1'b0;
        
        done           = 1'b0;

        case (current_state)
            LOAD_WEIGHTS: begin
                // 从 WB 读取，写入 WRF。全亮WE表示广播给所有PE
                wb_re     = 1'b1;
                wb_raddr  = weight_cnt;
                wrf_we    = '1;
                wrf_waddr = weight_cnt;
            end
            
            FLUSH_ARF: begin
                // 从 IFB 读取初始 32 像素
                ifb_re         = 1'b1;
                // 假设输入特征从地址0开始连续存放
                ifb_raddr      = pixel_cnt;
                arf_flush_en   = 1'b1;
                arf_flush_addr = pixel_cnt;
            end
            
            COMPUTE_TILE: begin
                // 激活 MAC 计算
                compute_en    = 1'b1;
                
                // 1. 读取对应位置权重
                wrf_raddr     = weight_cnt;
                
                // 2. ARF提供输入数据，按像素索引滑动
                arf_read_addr = pixel_cnt;
                
                // 3. PARF累加地址 (对应当前计算的像素)
                parf_addr     = pixel_cnt;
                parf_we       = 1'b1;
                
                // 4. 当计算第0个权重(第一轮)时，PARF需要清零历史无用数据
                if (weight_cnt == 0) begin
                    parf_clear = 1'b1;
                end
                
                // 5. 数据复用：换行移位逻辑
                // 如果当前像素达到末尾，且需要读取新的像素用于下一轮核位置滑动，
                // 则触发ARF移位并从IFB读取1个新像素 (这里简化：仅供参考，不涉及复杂跨行)
                if (pixel_cnt == num_pixels - 1) begin
                    // TODO: 对于复杂的行间滑动，在此处启动 ifb_re 和 arf_shift_en
                    // 此处保留给完整系统集成
                end
            end
            
            default: ; // IDLE state is covered by default assigns
        endcase
        
        if ((current_state == COMPUTE_TILE) && (pixel_cnt == num_pixels - 1) && (weight_cnt == kernel_size - 1)) begin
            done = 1'b1;
        end
    end

endmodule
