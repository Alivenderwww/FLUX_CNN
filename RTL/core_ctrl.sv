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
    
    // OFB 控制接口
    output logic                                ofb_we,       // OFB 写使能
    output logic [$clog2(SRAM_DEPTH)-1:0]       ofb_waddr,    // OFB 写地址
    output logic                                sdp_en_out,   // 输出给后处理的 SDP 使能
    
    // WRF 控制接口 (Weight Register File)
    output logic [63:0]                         wrf_we,       // 所有PE的写使能
    output logic [$clog2(WRF_DEPTH)-1:0]        wrf_waddr,    // 写地址
    
    // ARF 控制接口 (Activation Register File)
    output logic                                arf_we,       // ARF 写使能
    output logic [$clog2(ARF_DEPTH)-1:0]        arf_waddr,    // ARF 写地址
    output logic [$clog2(ARF_DEPTH)-1:0]        arf_read_addr, // 读出到MAC的地址
    
    // MAC & PARF 控制接口
    output logic                                compute_en,   // MAC计算使能
    output logic [$clog2(WRF_DEPTH)-1:0]        wrf_raddr,    // MAC读取WRF地址
    output logic [$clog2(PARF_DEPTH)-1:0]       parf_addr,    // PARF当前累加/写入位置
    output logic                                parf_clear,   // 是否清零PARF
    output logic                                parf_we,      // PARF写回使能

    // SDP 配置接口 (OP_LD_SDP 在 DECODE 内写入，持久保存在 SDP 模块)
    output logic                                sdp_shift_we,   // SDP shift_amt 写使能
    output logic [4:0]                          sdp_shift_wdata // 新的 shift_amt 值
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
        EXEC_MAC,
        EXEC_ST_OFM,
        EXEC_LD1MAC,  // Load 1 pixel to ARF[ld_arf_addr] AND run MAC simultaneously
        EXEC_LD32MAC, // Load N pixels into ARF AND run MAC simultaneously (length+1 cycles total)
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
    logic                          arf_we_d1;
    logic [$clog2(ARF_DEPTH)-1:0]  arf_waddr_d1;
    
    // OFB 延迟寄存器 (PARF 读出需要 2 周期)
    logic                          ofb_we_d1, ofb_we_d2;
    logic [$clog2(SRAM_DEPTH)-1:0] ofb_waddr_d1, ofb_waddr_d2;
    logic                          sdp_en_d1, sdp_en_d2;
    
    // 标量寄存器文件 (8 x 16-bit)
    // r0 = IFB 基址偏移（自动叠加到所有 IFB 读地址）
    // r1 = OFB 基址偏移（自动叠加到所有 OFB 写地址）
    // r2-r7 = 通用（循环计数器、临时变量等）
    logic [15:0]                   scalar_rf [0:7];
    
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
            arf_we_d1         <= '0;
            arf_waddr_d1      <= '0;
            
            ofb_we_d1         <= '0;
            ofb_waddr_d1      <= '0;
            ofb_we_d2         <= '0;
            ofb_waddr_d2      <= '0;
            sdp_en_d1         <= '0;
            sdp_en_d2         <= '0;
            for (int i = 0; i < 8; i++) scalar_rf[i] <= '0;
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
            // EXEC_LD_ARF:  连续写入 length 个像素
            // EXEC_LD1MAC:  仅在 cnt=0 写入 1 个像素到 ld_arf_addr（与 MAC 并行）
            // EXEC_LD32MAC: 连续写入 length 个像素（cnt=0..length-1），MAC 在 cnt=1..length 同步运行
            if ((current_state == EXEC_LD_ARF  && cnt < inst_reg.length) ||
                (current_state == EXEC_LD32MAC  && cnt < inst_reg.length)) begin
                arf_we_d1    <= 1'b1;
                arf_waddr_d1 <= inst_reg.arf_addr + cnt[4:0];
            end else if (current_state == EXEC_LD1MAC && cnt == 0) begin
                arf_we_d1    <= 1'b1;
                arf_waddr_d1 <= inst_reg.ld_arf_addr;
            end else begin
                arf_we_d1    <= 1'b0;
                arf_waddr_d1 <= '0;
            end

            // 写入 OFB 时的 2 拍延迟同步 (从发送 parf_addr 到 psum_out_vec 可用需要 2 拍)
            if (current_state == EXEC_ST_OFM && cnt < inst_reg.length) begin
                ofb_we_d1    <= 1'b1;
                ofb_waddr_d1 <= scalar_rf[1][14:0] + inst_reg.sram_addr + cnt[14:0];  // r1 = OFB 基址偏移
                sdp_en_d1    <= inst_reg.sdp_en;
            end else begin
                ofb_we_d1    <= 1'b0;
                ofb_waddr_d1 <= '0;
                sdp_en_d1    <= 1'b0;
            end
            
            ofb_we_d2    <= ofb_we_d1;
            ofb_waddr_d2 <= ofb_waddr_d1;
            sdp_en_d2    <= sdp_en_d1;

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
                    // DECODE 内直接完成的指令：NOP、标量运算、跳转
                    case (inst_data.opcode)
                        OP_NOP: begin
                            pc <= pc + 1'b1;
                        end

                        OP_LI: begin
                            // scalar_rf[rd] <- imm16（rd 在 arf_addr[2:0]，imm16 在 length 字段）
                            scalar_rf[inst_data.arf_addr[2:0]] <= inst_data.length;
                            pc <= pc + 1'b1;
                        end

                        OP_ALU: begin
                            // clr_parf=0: 加法；clr_parf=1: 减法
                            // sdp_en=0: 寄存器模式；sdp_en=1: 立即数模式（imm16 在 length 字段）
                            if (!inst_data.sdp_en) begin  // 寄存器模式: rd <- rs1 op rs2
                                scalar_rf[inst_data.arf_addr[2:0]] <= !inst_data.clr_parf
                                    ? (scalar_rf[inst_data.wgt_rf_addr[2:0]] + scalar_rf[inst_data.parf_addr[2:0]])
                                    : (scalar_rf[inst_data.wgt_rf_addr[2:0]] - scalar_rf[inst_data.parf_addr[2:0]]);
                            end else begin                 // 立即数模式: rd <- rs1 op imm16
                                scalar_rf[inst_data.arf_addr[2:0]] <= !inst_data.clr_parf
                                    ? (scalar_rf[inst_data.wgt_rf_addr[2:0]] + inst_data.length)
                                    : (scalar_rf[inst_data.wgt_rf_addr[2:0]] - inst_data.length);
                            end
                            pc <= pc + 1'b1;
                        end

                        OP_JMP: begin
                            // 无条件跳转：PC <- target_pc（target 在 sram_addr 字段）
                            pc <= inst_data.sram_addr[$clog2(INST_DEPTH)-1:0];
                        end

                        OP_BNZ: begin
                            // 递减并按需跳转PC：rs 在 arf_addr[2:0]，target 在 sram_addr
                            if (scalar_rf[inst_data.arf_addr[2:0]] != '0) begin
                                scalar_rf[inst_data.arf_addr[2:0]] <= scalar_rf[inst_data.arf_addr[2:0]] - 1'b1;
                                pc <= inst_data.sram_addr[$clog2(INST_DEPTH)-1:0];
                            end else begin
                                pc <= pc + 1'b1;
                            end
                        end

                        OP_LD_SDP: begin
                            // SDP 反量化右移参数：shift_amt <- ld_arf_addr[4:0]
                            // sdp_shift_we 在组合逻辑输出侧产生，此处只推进 PC
                            pc <= pc + 1'b1;
                        end

                        default: begin
                            // 其他 opcode 进 EXEC 状态，pc 由 EXEC 退出时更新
                        end
                    endcase
                end
                
                EXEC_LD_WGT, EXEC_LD_ARF, EXEC_MAC, EXEC_ST_OFM, EXEC_LD1MAC: begin
                    if (cnt == inst_reg.length - 1) begin
                        cnt <= '0;
                        pc  <= pc + 1'b1; // 执行完指令，PC 自增
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                EXEC_LD32MAC: begin
                    // 比普通 EXEC 多跑 1 拍（cnt=0 为 IFB 预取，cnt=1..length 为 MAC）
                    if (cnt == inst_reg.length) begin
                        cnt <= '0;
                        pc  <= pc + 1'b1;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
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
                    OP_MAC_RUN:   next_state = EXEC_MAC;
                    OP_ST_OFM:    next_state = EXEC_ST_OFM;
                    OP_LD1MAC:    next_state = EXEC_LD1MAC;
                    OP_LD32MAC:   next_state = EXEC_LD32MAC;
                    // 标量指令全部在 DECODE 内完成，直接回 FETCH
                    OP_LI,
                    OP_ALU,
                    OP_JMP,
                    OP_BNZ,
                    OP_LD_SDP:    next_state = FETCH;
                    // 其他（包括 OP_FINISH）进 FINISH
                    default:      next_state = FINISH;
                endcase
            end
            
            EXEC_LD_WGT, EXEC_LD_ARF, EXEC_MAC, EXEC_ST_OFM, EXEC_LD1MAC: begin
                if (cnt == inst_reg.length - 1) next_state = FETCH;
            end

            EXEC_LD32MAC: begin
                if (cnt == inst_reg.length) next_state = FETCH; // 多跑 1 拍
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
        ofb_we         = ofb_we_d2;
        ofb_waddr      = ofb_waddr_d2;
        sdp_en_out     = sdp_en_d2;

        wrf_we         = wrf_we_d1;
        wrf_waddr      = wrf_waddr_d1;
        arf_we         = arf_we_d1;
        arf_waddr      = arf_waddr_d1;

        arf_read_addr  = '0;

        compute_en     = 1'b0;
        wrf_raddr      = '0;
        parf_addr      = '0;
        parf_clear     = 1'b0;
        parf_we        = 1'b0;
        done           = 1'b0;

        // SDP 配置默认不写
        sdp_shift_we    = 1'b0;
        sdp_shift_wdata = '0;

        case (current_state)
            IDLE: begin
                // wait
            end
            
            FETCH: begin
                inst_re   = 1'b1;
                inst_addr = pc;
            end

            DECODE: begin
                // OP_LD_SDP: 在 DECODE 状态组合驱动 SDP 写使能
                // shift_amt 写入 sdp.sv 的寄存器（由 sdp_shift_we 触发 FF 捕获）
                if (inst_data.opcode == OP_LD_SDP) begin
                    sdp_shift_we    = 1'b1;
                    sdp_shift_wdata = inst_data.ld_arf_addr; // [4:0] = shift_amt
                end
            end
            
            EXEC_LD_WGT: begin
                // 发起 SRAM 读请求，目标是 Weight Buffer
                if (cnt < inst_reg.length) begin
                    wb_re    = 1'b1;
                    wb_raddr = inst_reg.sram_addr + cnt[14:0];
                end
            end
            
            EXEC_LD_ARF: begin
                // 发起 IFB 读请求（r0 为 IFB 基址偏移）
                if (cnt < inst_reg.length) begin
                    ifb_re    = 1'b1;
                    ifb_raddr = scalar_rf[0][14:0] + inst_reg.sram_addr + cnt[14:0];
                end
            end
            
            EXEC_MAC: begin
                // 开始纯计算
                compute_en    = 1'b1;
                wrf_raddr     = inst_reg.wgt_rf_addr;          // 从指令获取使用哪个权重
                arf_read_addr = inst_reg.arf_addr + cnt[4:0];  // NEW: use arf_addr
                parf_addr     = inst_reg.parf_addr + cnt[4:0]; // 写入 PARF 的对应位置
                parf_we       = 1'b1;
                
                // 根据指令决定是否清空累加器，如果为1，则全流水线周期内持续清零对应的PARF历史值
                if (inst_reg.clr_parf) parf_clear = 1'b1; 
            end
            
            EXEC_ST_OFM: begin
                // 发起 PARF 读请求（通过直接设置 parf_addr）
                // MAC 阵列会在 2 拍后将 psum_out_vec 吐出，ofb_we 和 ofb_waddr 的延迟管线会同时到达
                parf_addr = inst_reg.parf_addr + cnt[4:0];
            end

            EXEC_LD1MAC: begin
                // 单像素 kx 切换优化：写 ARF[ld_arf_addr] 与 MAC 计算同时进行
                // ARF 写入地址（ld_arf_addr）与 MAC 读取地址（arf_addr+cnt）不重叠，无冒险

                // 仅在第 0 拍发起 IFB 单像素读（d1 流水线负责 ARF 实际写入）
                if (cnt == 0) begin
                    ifb_re    = 1'b1;
                    ifb_raddr = scalar_rf[0][14:0] + inst_reg.sram_addr;
                end

                // MAC 全程运行（与上面的 IFB 读并行）
                compute_en    = 1'b1;
                wrf_raddr     = inst_reg.wgt_rf_addr;
                arf_read_addr = inst_reg.arf_addr + cnt[4:0];
                parf_addr     = inst_reg.parf_addr + cnt[4:0];
                parf_we       = 1'b1;
                if (inst_reg.clr_parf) parf_clear = 1'b1;
            end

            EXEC_LD32MAC: begin
                // 初始加载优化：从 IFB 连续读 length 个像素写入 ARF，同时 MAC 并行运行。
                //
                // 时序：cnt=0 为 IFB 预取拍（无 MAC）；cnt=1..length 为重叠拍：
                //   - d1 流水将 pixel[cnt-1] 写入 ARF[arf_addr+cnt-1]
                //   - bypass MUX（core_top）检测写读地址相同时直接前向传给 MAC
                //   - MAC 读 arf_read_addr=arf_addr+(cnt-1)，得到 pixel[cnt-1] ✓

                // IFB 连续读（cnt=0..length-1）（r0 为 IFB 基址偏移）
                if (cnt < inst_reg.length) begin
                    ifb_re    = 1'b1;
                    ifb_raddr = scalar_rf[0][14:0] + inst_reg.sram_addr + cnt[14:0];
                end

                // MAC 从 cnt=1 开始运行（pixel[0] 在 cnt=1 的 d1 写入时才可用）
                if (cnt > 0) begin
                    compute_en    = 1'b1;
                    wrf_raddr     = inst_reg.wgt_rf_addr;
                    arf_read_addr = inst_reg.arf_addr + (cnt[4:0] - 1'b1);
                    parf_addr     = inst_reg.parf_addr + (cnt[4:0] - 1'b1);
                    parf_we       = 1'b1;
                    if (inst_reg.clr_parf) parf_clear = 1'b1;
                end
            end

            FINISH: begin
                done = 1'b1;
            end
            
            default: ;
        endcase
    end

endmodule