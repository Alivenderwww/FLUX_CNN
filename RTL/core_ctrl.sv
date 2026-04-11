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
    input  logic [3:0]                          kernel_size,  // 卷积核大小总和 (如 3x3=9)
    input  logic [3:0]                          kernel_dim,   // 卷积核边长 (如 3)
    input  logic [15:0]                         image_width,  // 图像宽度 (用于计算换行偏移)
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
        IDLE,                // 等待开始
        LOAD_WEIGHTS,        // 搬运所有权重到WRF
        PRE_READ_ROW,        // 发起首个SRAM像素读取 (1周期读延迟预取)
        COMPUTE_ROW_NEW,     // 换行首次读取(32周期流水线：发读->收写ARF+算)
        PRE_SHIFT,           // 权轴滑动，发起移位补充读取 (1周期预取)
        SHIFT_WAIT,          // 接收SRAM数据，触发ARF整体移位 (1周期等待)
        COMPUTE_ROW_SHIFTED  // 已补充完1像素，纯计算当前ARF中的32个像素
    } state_t;
    
    state_t current_state, next_state;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) current_state <= IDLE;
        else current_state <= next_state;
    end

    //=============================================================================
    // 2. 内部计数器
    //=============================================================================
    logic [3:0]  weight_cnt; // 全局权重索引 (0~kernel_size-1)
    logic [3:0]  kx, ky;     // 二维权重坐标 (0~kernel_dim-1)
    logic [7:0]  pixel_cnt;  // 当前计算/读取的像素索引 (0~num_pixels-1)
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            weight_cnt <= '0;
            kx <= '0;
            ky <= '0;
            pixel_cnt  <= '0;
        end else begin
            case (current_state)
                IDLE: begin
                    weight_cnt <= '0;
                    kx <= '0;
                    ky <= '0;
                    pixel_cnt  <= '0;
                end
                
                LOAD_WEIGHTS: begin
                    if (weight_cnt == kernel_size - 1) weight_cnt <= '0; // 算前重置
                    else weight_cnt <= weight_cnt + 1'b1;
                end
                
                COMPUTE_ROW_NEW, COMPUTE_ROW_SHIFTED: begin
                    // 在计算周期的每一个节拍累加pixel_cnt
                    if (pixel_cnt == num_pixels - 1) begin
                        pixel_cnt <= '0;
                        
                        // 32 个像素算完，切权重
                        if (kx == kernel_dim - 1) begin
                            kx <= '0;
                            if (ky == kernel_dim - 1) begin
                                ky <= '0;
                                weight_cnt <= '0; // 所有权重循环结束
                            end else begin
                                ky <= ky + 1'b1;  // 换权重行
                                weight_cnt <= weight_cnt + 1'b1;
                            end
                        end else begin
                            kx <= kx + 1'b1;      // 同行切换权重
                            weight_cnt <= weight_cnt + 1'b1;
                        end
                    end else begin
                        pixel_cnt <= pixel_cnt + 1'b1;
                    end
                end
                
                // 其他状态保持计数器不变
                default: ; 
            endcase
        end
    end

    //=============================================================================
    // 2.5 写权重延迟寄存器 (匹配SRAM的1拍读延迟)
    //=============================================================================
    logic [63:0]                         wrf_we_d1;
    logic [$clog2(WRF_DEPTH)-1:0]        wrf_waddr_d1;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wrf_we_d1    <= '0;
            wrf_waddr_d1 <= '0;
        end else begin
            if (current_state == LOAD_WEIGHTS) begin
                wrf_we_d1    <= '1;
                wrf_waddr_d1 <= weight_cnt;
            end else begin
                wrf_we_d1    <= '0;
                wrf_waddr_d1 <= '0;
            end
        end
    end
    //=============================================================================
    always_comb begin
        next_state = current_state; 
        
        case (current_state)
            IDLE: begin
                if (start) next_state = LOAD_WEIGHTS;
            end
            
            LOAD_WEIGHTS: begin
                if (weight_cnt == kernel_size - 1) next_state = PRE_READ_ROW;
            end
            
            PRE_READ_ROW: begin
                next_state = COMPUTE_ROW_NEW; // 预取完1周期直接进入计算流水
            end
            
            COMPUTE_ROW_NEW: begin
                if (pixel_cnt == num_pixels - 1) begin
                    if (kx == kernel_dim - 1 && ky == kernel_dim - 1) next_state = IDLE;
                    else if (kx == kernel_dim - 1) next_state = PRE_READ_ROW; // ky变了，换行重读
                    else next_state = PRE_SHIFT;                              // ky没变，同行移位
                end
            end
            
            PRE_SHIFT: begin
                next_state = SHIFT_WAIT; // 发完读地址等一拍数据
            end
            
            SHIFT_WAIT: begin
                next_state = COMPUTE_ROW_SHIFTED; // 收到数据并移位后，进行纯算
            end
            
            COMPUTE_ROW_SHIFTED: begin
                if (pixel_cnt == num_pixels - 1) begin
                    if (kx == kernel_dim - 1 && ky == kernel_dim - 1) next_state = IDLE;
                    else if (kx == kernel_dim - 1) next_state = PRE_READ_ROW; // 换行
                    else next_state = PRE_SHIFT;                              // 移位
                end
            end
            
            default: next_state = IDLE;
        endcase
    end

    //=============================================================================
    // 4. 控制信号输出逻辑
    //=============================================================================
    always_comb begin
        // 默认初始化
        wb_re          = 1'b0;
        wb_raddr       = '0;
        ifb_re         = 1'b0;
        ifb_raddr      = '0;
        wrf_we         = wrf_we_d1;    // 延迟1拍写入
        wrf_waddr      = wrf_waddr_d1; // 延迟1拍地址
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
                wb_re     = 1'b1;
                wb_raddr  = weight_cnt;
                // wrf_we 和 wrf_waddr 在组合逻辑中被赋值为对应的 _d1 寄存器输出，以匹配 SRAM 延迟
            end
            
            PRE_READ_ROW: begin
                // 发出 ky 行第 0 个像素的读请求 (预取)
                ifb_re    = 1'b1;
                ifb_raddr = (ky * image_width);
            end
            
            COMPUTE_ROW_NEW: begin
                // 流水线设计: 发起对pixel_cnt+1的读请求，同时接收pixel_cnt的数据并计算
                
                // 1. 发起下一次读请求 (除非已经是最后一次读了)
                if (pixel_cnt < num_pixels - 1) begin
                    ifb_re    = 1'b1;
                    ifb_raddr = (ky * image_width) + pixel_cnt + 1;
                end
                
                // 2. 接收来自 SRAM 的数据并写入 ARF
                arf_flush_en   = 1'b1;
                arf_flush_addr = pixel_cnt;
                
                // 3. 计算使能 (由于有 Bypass 逻辑，act_out_vec 当前拍就是 SRAM 数据)
                compute_en    = 1'b1;
                wrf_raddr     = weight_cnt;
                arf_read_addr = pixel_cnt; // 读刚写入的同地址，触发 Bypass
                parf_addr     = pixel_cnt;
                parf_we       = 1'b1;
                
                // 仅在第一个权重的计算轮次清空对应像素的历史 PARF
                if (kx == 0 && ky == 0) begin
                    parf_clear = 1'b1;
                end
            end
            
            PRE_SHIFT: begin
                // 同行滑动，只发读最右边新窗口的 1 个像素的请求
                ifb_re    = 1'b1;
                ifb_raddr = (ky * image_width) + (num_pixels - 1) + kx;
            end
            
            SHIFT_WAIT: begin
                // 收到新像素，打入 ARF 末尾，全体左移 1 位
                arf_shift_en = 1'b1;
            end
            
            COMPUTE_ROW_SHIFTED: begin
                // 纯计算状态，ARF已就绪
                compute_en    = 1'b1;
                wrf_raddr     = weight_cnt;
                arf_read_addr = pixel_cnt;
                parf_addr     = pixel_cnt;
                parf_we       = 1'b1;
                
                // 后续权重均是在同行/同像素上做累加，不清空 PARF
                parf_clear    = 1'b0;
            end
            
            default: ; 
        endcase
        
        // 生成完成信号
        if ((current_state == COMPUTE_ROW_NEW || current_state == COMPUTE_ROW_SHIFTED) && 
            (pixel_cnt == num_pixels - 1) && (kx == kernel_dim - 1) && (ky == kernel_dim - 1)) begin
            done = 1'b1;
        end
    end

endmodule
