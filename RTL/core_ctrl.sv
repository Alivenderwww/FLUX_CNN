`timescale 1ns/1ps
import core_isa_pkg::*;

// 指令译码器与微状态机 (Instruction Decoder & Micro-Sequencer)
// 负责读取宏指令，并将其展开为底层的微小控制信号
module core_ctrl #(
    parameter int WRF_DEPTH   = 32,      // 权重RF深度
    parameter int ARF_DEPTH   = 32,      // 激活值RF深度
    parameter int PARF_DEPTH  = 32,      // PARF深度
    parameter int SRAM_DEPTH  = 1024,    // IFB/WB 深度
    parameter int INST_DEPTH  = 1024     // 指令SRAM深度
)(
    input  logic                                clk,
    input  logic                                rst_n,
    
    // 整体控制信号
    input  logic                                start,        // 启动执行
    output logic                                done,         // 执行完成所有指令
    
    // 指令提取接口 (Instruction Fetch)
    output logic                                inst_re,
    output logic [$clog2(INST_DEPTH)-1:0]       inst_addr,
    input  inst_t                               inst_data,
    
    // SRAM 控制接口
    output logic                                wb_re,        // Weight Buffer 读使能
    output logic [$clog2(SRAM_DEPTH)-1:0]       wb_raddr,     // 权重读取地址
    
    output logic                                ifb_re,       // IFB 读使能
    output logic [$clog2(SRAM_DEPTH)-1:0]       ifb_raddr,    // IFB 读地址
    
    // WRF 控制接口 (Weight Register File)
    output logic [63:0]                         wrf_we,       // 所有PE的写使能
    output logic [$clog2(WRF_DEPTH)-1:0]        wrf_waddr,    // 写地址
    
    // ARF 控制接口 (Activation Register File)
    output logic                                arf_shift_en, // 移位使能
    output logic                                arf_flush_en, // 刷新使能 (加载新数据)
    output logic [$clog2(ARF_DEPTH)-1:0]        arf_flush_addr,// 刷新写入地址
    output logic [$clog2(ARF_DEPTH)-1:0]        arf_read_addr, // 读出到MAC的地址
    
    // MAC & PARF 控制接口
    output logic                                compute_en,   // MAC计算使能
    output logic [$clog2(WRF_DEPTH)-1:0]        wrf_raddr,    // MAC读取WRF地址
    output logic [$clog2(PARF_DEPTH)-1:0]       parf_addr,    // PARF当前累加/写入位置
    output logic                                parf_clear,   // 是否清零PARF
    output logic                                parf_we       // PARF写回使能
);

    //=============================================================================
    // 1. 微状态机定义
    //=============================================================================
    typedef enum logic [3:0] {
        IDLE,
        FETCH,
        DECODE,
        EXEC_LD_WGT,
        EXEC_LD_ARF,
        PRE_MAC_SHIFT,
        WAIT_MAC_SHIFT,
        EXEC_MAC,
        EXEC_ST_OFM,
        FINISH
    } state_t;
    
    state_t current_state, next_state;
    
    //=============================================================================
    // 2. 内部寄存器 (指令寄存、程序计数器、循环计数器)
    //=============================================================================
    logic [$clog2(INST_DEPTH)-1:0] pc;          // 程序计数器
    inst_t                         inst_reg;    // 当前执行的指令
    logic [15:0]                   cnt;         // 微状态循环计数器 (对应 inst.length)
    
    // 延迟匹配寄存器 (为了匹配 SRAM 的 1 周期读延迟)
    logic [63:0]                   wrf_we_d1;
    logic [$clog2(WRF_DEPTH)-1:0]  wrf_waddr_d1;
    logic                          arf_flush_en_d1;
    logic [$clog2(ARF_DEPTH)-1:0]  arf_flush_addr_d1;
    logic                          arf_shift_en_d1;
    
    //=============================================================================
    // 3. 时序逻辑更新
    //=============================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
            pc            <= '0;
            cnt           <= '0;
            inst_reg      <= '0;
            
            wrf_we_d1         <= '0;
            wrf_waddr_d1      <= '0;
            arf_flush_en_d1   <= '0;
            arf_flush_addr_d1 <= '0;
            arf_shift_en_d1   <= '0;
        end else begin
            current_state <= next_state;
            
            // 写入 WRF 时的 1 拍延迟同步 (从 WB 读出需要 1 拍)
            if (current_state == EXEC_LD_WGT && cnt < inst_reg.length) begin
                wrf_we_d1    <= '1;
                wrf_waddr_d1 <= inst_reg.wgt_rf_addr + cnt[4:0];
            end else begin
                wrf_we_d1    <= '0;
                wrf_waddr_d1 <= '0;
            end
            
            // 写入 ARF 时的 1 拍延迟同步 (从 IFB 读出需要 1 拍)
            if (current_state == EXEC_LD_ARF && cnt < inst_reg.length) begin
                arf_flush_en_d1   <= 1'b1;
                arf_flush_addr_d1 <= cnt[4:0];
            end else begin
                arf_flush_en_d1   <= 1'b0;
                arf_flush_addr_d1 <= '0;
            end
            
            if (current_state == PRE_MAC_SHIFT) begin
                arf_shift_en_d1 <= 1'b1;
            end else begin
                arf_shift_en_d1 <= 1'b0;
            end

            case (current_state)
                IDLE: begin
                    pc  <= '0;
                    cnt <= '0;
                end
                
                FETCH: begin
                    // FETCH 状态下只发出读请求
                end
                
                DECODE: begin
                    inst_reg <= inst_data; // 锁存指令
                    cnt      <= '0;
                    // 如果是 OP_NOP，我们在 DECODE 里就增加 PC，状态机自己会跳回 FETCH
                    if (inst_data.opcode == OP_NOP) begin
                        pc <= pc + 1'b1;
                    end
                end
                
                EXEC_LD_WGT, EXEC_LD_ARF, EXEC_MAC, EXEC_ST_OFM: begin
                    if (cnt == inst_reg.length - 1) begin
                        cnt <= '0;
                        pc  <= pc + 1'b1; // 执行完指令，PC 自增
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end
                
                PRE_MAC_SHIFT: begin
                    // 只有 1 个周期的预取
                end
                
                FINISH: begin
                    // 保持 FINISH
                end
                
                default: ;
            endcase
        end
    end

    //=============================================================================
    // 4. 状态机跳转逻辑
    //=============================================================================
    always_comb begin
        next_state = current_state;
        
        case (current_state)
            IDLE: begin
                if (start) next_state = FETCH;
            end
            
            FETCH: begin
                next_state = DECODE; // 等待1周期指令SRAM读出
            end
            
            DECODE: begin
                // 根据 opcode 跳转到对应的执行状态
                case (inst_data.opcode)
                    OP_NOP:       next_state = FETCH; // 忽略 NOP，继续取指
                    OP_LD_WGT:    next_state = EXEC_LD_WGT;
                    OP_LD_ARF:    next_state = EXEC_LD_ARF;
                    OP_MAC_RUN:   begin
                        // 如果带移位，需先发一次读取请求并等待
                        if (inst_data.shift_arf) next_state = PRE_MAC_SHIFT;
                        else                     next_state = EXEC_MAC;
                    end
                    OP_ST_OFM:    next_state = EXEC_ST_OFM;
                    // 如果遇到未定义的 opcode 或者结束指令标志，视为结束
                    default:      next_state = FINISH; 
                endcase
            end
            
            EXEC_LD_WGT, EXEC_LD_ARF, EXEC_MAC, EXEC_ST_OFM: begin
                if (cnt == inst_reg.length - 1) next_state = FETCH;
            end
            
            PRE_MAC_SHIFT: begin
                next_state = WAIT_MAC_SHIFT;
            end
            
            WAIT_MAC_SHIFT: begin
                next_state = EXEC_MAC;
            end
            
            FINISH: begin
                next_state = FINISH;
            end
            
            default: next_state = IDLE;
        endcase
    end

    //=============================================================================
    // 5. 输出控制信号逻辑 (纯组合逻辑)
    //=============================================================================
    always_comb begin
        // 默认初始化
        inst_re        = 1'b0;
        inst_addr      = '0;
        
        wb_re          = 1'b0;
        wb_raddr       = '0;
        ifb_re         = 1'b0;
        ifb_raddr      = '0;
        
        // 延迟写的信号直接赋给端口
        wrf_we         = wrf_we_d1;
        wrf_waddr      = wrf_waddr_d1;
        arf_flush_en   = arf_flush_en_d1;
        arf_flush_addr = arf_flush_addr_d1;
        arf_shift_en   = arf_shift_en_d1;
        
        arf_read_addr  = '0;
        
        compute_en     = 1'b0;
        wrf_raddr      = '0;
        parf_addr      = '0;
        parf_clear     = 1'b0;
        parf_we        = 1'b0;
        done           = 1'b0;

        case (current_state)
            IDLE: begin
                // wait
            end
            
            FETCH: begin
                inst_re   = 1'b1;
                inst_addr = pc;
            end
            
            EXEC_LD_WGT: begin
                // 发起 SRAM 读请求，目标是 Weight Buffer
                if (cnt < inst_reg.length) begin
                    wb_re    = 1'b1;
                    wb_raddr = inst_reg.sram_addr + cnt[14:0];
                end
            end
            
            EXEC_LD_ARF: begin
                // 发起 IFB 读请求
                if (cnt < inst_reg.length) begin
                    ifb_re    = 1'b1;
                    ifb_raddr = inst_reg.sram_addr + cnt[14:0];
                end
            end
            
            PRE_MAC_SHIFT: begin
                // 取需移入的一个新像素 (约定的 sram_addr)
                ifb_re    = 1'b1;
                ifb_raddr = inst_reg.sram_addr; 
            end
            
            WAIT_MAC_SHIFT: begin
                // 空闲一拍，等待 IFB 数据准备好，同时也给 ARF 足够的时间移位
            end
            
            EXEC_MAC: begin
                // 到这里时，ARF 已经完成了移位（如果有），可以安全地进行计算
                
                // 开始纯计算
                compute_en    = 1'b1;
                wrf_raddr     = inst_reg.wgt_rf_addr;          // 从指令获取使用哪个权重
                arf_read_addr = cnt[4:0];                      // MAC 一次算 length 拍，依次读 ARF
                parf_addr     = inst_reg.parf_addr + cnt[4:0]; // 写入 PARF 的对应位置
                parf_we       = 1'b1;
                
                // 根据指令决定是否清空累加器，如果为1，则全流水线周期内持续清零对应的PARF历史值
                if (inst_reg.clr_parf) parf_clear = 1'b1; 
            end
            
            EXEC_ST_OFM: begin
                // TODO: Store 到外部，需要实现 Outmap Buffer 控制。暂为空。
            end
            
            FINISH: begin
                done = 1'b1;
            end
            
            default: ;
        endcase
    end

endmodule