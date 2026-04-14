`timescale 1ns/1ps
import core_isa_pkg::*;

// 指令译码器与微状态机 (Instruction Decoder & Micro-Sequencer)
// 负责读取宏指令，并将其展开为底层的微小控制信号
//
// 【3级指令流水】IF → ID → EX
//   在 EX 尾部（cnt == end_cnt-2）提前发出 IF，
//   cnt == end_cnt-1 锁存 inst_data 到 pf_inst（完成 ID），
//   cnt == end_cnt   EX 结束，直接切换到下一条指令的 EX，零空泡。
//   分支（BNZ/JMP）和短指令（end_cnt<2）回退到普通 FETCH 路径。
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
        EXEC_LDnMAC,   // Unified Load-n-and-MAC
                       // ld_len = inst_reg.length[15:11]+1  pixels loaded from IFB into ARF
                       // mac_len= inst_reg.length[10:0]     MAC active cycles
                       // Timing: cnt=0 IFB prefetch only; cnt=1..mac_len IFB (if cnt<ld_len) + MAC
                       // Total cycles = mac_len+1;  arf_read_addr = arf_addr + (cnt-1)
        FINISH
    } state_t;

    state_t current_state, next_state;

    //=============================================================================
    // 2. 内部寄存器
    //=============================================================================
    logic [$clog2(INST_DEPTH)-1:0] pc;       // 当前执行指令的 PC
    inst_t                         inst_reg; // 当前执行的指令
    logic [15:0]                   cnt;      // 微状态循环计数器

    // 延迟匹配寄存器 (为了匹配 SRAM 的 1 周期读延迟)
    logic [63:0]                   wrf_we_d1;
    logic [$clog2(WRF_DEPTH)-1:0]  wrf_waddr_d1;
    logic                          arf_we_d1;
    logic [$clog2(ARF_DEPTH)-1:0]  arf_waddr_d1;

    // OFB 延迟寄存器 (PARF 读出需要 2 周期)
    logic                          ofb_we_d1, ofb_we_d2;
    logic [$clog2(SRAM_DEPTH)-1:0] ofb_waddr_d1, ofb_waddr_d2;
    logic                          sdp_en_d1, sdp_en_d2;

    // EXEC_LDnMAC 步进计算临时变量
    logic [2:0]  eff_stride;
    logic [14:0] strided_off;

    // LDnMAC 字段解包（仅对 OP_LDnMAC 有效）
    logic [5:0]  ldnmac_ld_len;
    logic [10:0] ldnmac_mac_len;
    logic [15:0] ldnmac_end_cnt;
    assign ldnmac_ld_len  = {1'b0, inst_reg.length[15:11]} + 6'd1;
    assign ldnmac_mac_len = inst_reg.length[10:0];
    assign ldnmac_end_cnt = {5'b0, ldnmac_mac_len};

    // 标量寄存器文件 (8 x 16-bit)
    logic [15:0] scalar_rf [0:7];

    //=============================================================================
    // 3. 【流水线预取】寄存器与控制
    //=============================================================================
    // pf_inst      : 预取并完成 ID 的下一条指令字
    // pf_pc        : 预取指令对应的 PC（即将执行的 pc+1）
    // pf_valid     : 预取寄存器已就绪，EX 结束时可以直通
    // pf_if_pending: 已发出 IF 请求，等待下一拍 inst_data 有效
    inst_t                         pf_inst;
    logic [$clog2(INST_DEPTH)-1:0] pf_pc;
    logic                          pf_valid;
    logic                          pf_if_pending;

    // 当前 EX 指令的 end_cnt（组合逻辑，供预取触发判断）
    logic [15:0] cur_end_cnt;
    always_comb begin
        case (current_state)
            EXEC_LDnMAC:                      cur_end_cnt = ldnmac_end_cnt;
            EXEC_LD_WGT, EXEC_LD_ARF,
            EXEC_MAC,    EXEC_ST_OFM:         cur_end_cnt = inst_reg.length - 16'd1;
            default:                          cur_end_cnt = 16'hFFFF; // 不会触发
        endcase
    end

    // 预取触发：cnt 到达 end_cnt-2 时发出 IF
    // 条件：EX 状态、end_cnt>=2、预取槽空闲
    logic pf_trigger;
    assign pf_trigger = (cnt == cur_end_cnt - 16'd2)
                      && (cur_end_cnt >= 16'd2)
                      && !pf_valid && !pf_if_pending
                      && (current_state inside {EXEC_LDnMAC, EXEC_LD_WGT,
                                                EXEC_LD_ARF, EXEC_MAC, EXEC_ST_OFM});

    //=============================================================================
    // 3b. 流水线直通判断（组合逻辑，供 always_ff 和 next_state 使用）
    //=============================================================================
    // opcode→EXEC 状态映射函数
    function automatic state_t opcode_to_exec_state(input opcode_e op);
        case (op)
            OP_LD_WGT:              return EXEC_LD_WGT;
            OP_LD_ARF:              return EXEC_LD_ARF;
            OP_MAC_RUN:             return EXEC_MAC;
            OP_ST_OFM:              return EXEC_ST_OFM;
            OP_LD1MAC, OP_LDnMAC:   return EXEC_LDnMAC;
            default:                return FETCH;
        endcase
    endfunction

    // pf_inst 是 EXEC 类指令（非标量/FINISH）
    logic pf_is_exec;
    assign pf_is_exec = pf_valid && !(pf_inst.opcode inside {
                            OP_NOP, OP_LI, OP_ALU, OP_JMP, OP_BNZ, OP_LD_SDP,
                            opcode_e'(4'd15) /* OP_FINISH */ });

    // EXEC 末尾判断
    logic at_exec_end;
    assign at_exec_end = (
        (current_state == EXEC_LDnMAC  && cnt == ldnmac_end_cnt) ||
        ((current_state inside {EXEC_LD_WGT, EXEC_LD_ARF, EXEC_MAC, EXEC_ST_OFM})
         && cnt == inst_reg.length - 1)
    );

    // EX→EX 直通条件
    logic pf_direct;
    assign pf_direct = at_exec_end && pf_is_exec;

    //=============================================================================
    // 4. 时序逻辑更新
    //=============================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
            pc            <= '0;
            cnt           <= '0;
            inst_reg      <= '0;

            wrf_we_d1     <= '0;  wrf_waddr_d1 <= '0;
            arf_we_d1     <= '0;  arf_waddr_d1 <= '0;
            ofb_we_d1     <= '0;  ofb_waddr_d1 <= '0;
            ofb_we_d2     <= '0;  ofb_waddr_d2 <= '0;
            sdp_en_d1     <= '0;  sdp_en_d2    <= '0;

            pf_inst       <= '0;
            pf_pc         <= '0;
            pf_valid      <= 1'b0;
            pf_if_pending <= 1'b0;

            for (int i = 0; i < 8; i++) scalar_rf[i] <= '0;
        end else begin
            current_state <= next_state;

            // ----------------------------------------------------------------
            // 预取流水线控制（独立于主 FSM）
            // ----------------------------------------------------------------

            // 步骤1：触发 IF（组合信号 pf_trigger 驱动，此处记录 pending 和 pc）
            if (pf_trigger) begin
                pf_if_pending <= 1'b1;
                pf_pc         <= pc + 1'b1;   // 预取顺序 PC+1（分支另处理）
            end

            // 步骤2：IF 完成，inst_data 有效，锁存到 pf_inst（完成 ID）
            if (pf_if_pending) begin
                pf_inst       <= inst_data;
                pf_valid      <= 1'b1;
                pf_if_pending <= 1'b0;
            end

            // 步骤3：EX→EX 直通——在 EXEC 最后一拍（cnt==end_cnt）时处理
            // 若 pf_valid：加载 pf_inst，cnt←0，跳过 FETCH/DECODE
            // 清除 pf_valid 无论走哪条路
            if (pf_valid && cnt == cur_end_cnt
                && (current_state inside {EXEC_LDnMAC, EXEC_LD_WGT,
                                          EXEC_LD_ARF, EXEC_MAC, EXEC_ST_OFM})) begin
                pf_valid <= 1'b0;
                // inst_reg 和 pc 的实际加载在下方主 case 里处理（直通分支）
            end

            // ----------------------------------------------------------------
            // WRF 写延迟（WB 读出需要 1 拍）
            // ----------------------------------------------------------------
            if (current_state == EXEC_LD_WGT && cnt < inst_reg.length) begin
                wrf_we_d1    <= '1;
                wrf_waddr_d1 <= inst_reg.wgt_rf_addr + cnt[4:0];
            end else begin
                wrf_we_d1    <= '0;
                wrf_waddr_d1 <= '0;
            end

            // ----------------------------------------------------------------
            // ARF 写延迟（IFB 读出需要 1 拍）
            // ----------------------------------------------------------------
            if (current_state == EXEC_LD_ARF && cnt < inst_reg.length) begin
                arf_we_d1    <= 1'b1;
                arf_waddr_d1 <= inst_reg.arf_addr + cnt[4:0];
            end else if (current_state == EXEC_LDnMAC && cnt < {11'b0, ldnmac_ld_len}) begin
                arf_we_d1    <= 1'b1;
                arf_waddr_d1 <= inst_reg.ld_arf_addr + cnt[4:0];
            end else begin
                arf_we_d1    <= 1'b0;
                arf_waddr_d1 <= '0;
            end

            // ----------------------------------------------------------------
            // OFB 写延迟（PARF 读出需要 2 拍）
            // ----------------------------------------------------------------
            if (current_state == EXEC_ST_OFM && cnt < inst_reg.length) begin
                ofb_we_d1    <= 1'b1;
                ofb_waddr_d1 <= scalar_rf[1][14:0] + inst_reg.sram_addr + cnt[14:0];
                sdp_en_d1    <= inst_reg.sdp_en;
            end else begin
                ofb_we_d1    <= 1'b0;
                ofb_waddr_d1 <= '0;
                sdp_en_d1    <= 1'b0;
            end
            ofb_we_d2    <= ofb_we_d1;
            ofb_waddr_d2 <= ofb_waddr_d1;
            sdp_en_d2    <= sdp_en_d1;

            // ----------------------------------------------------------------
            // 主 FSM 状态动作
            // ----------------------------------------------------------------
            case (current_state)
                IDLE: begin
                    pc  <= '0;
                    cnt <= '0;
                end

                FETCH: begin
                    // 只发出读请求（组合逻辑），此处无额外动作
                end

                DECODE: begin
                    inst_reg <= inst_data;
                    cnt      <= '0;
                    case (inst_data.opcode)
                        OP_NOP: begin
                            pc <= pc + 1'b1;
                        end
                        OP_LI: begin
                            scalar_rf[inst_data.arf_addr[2:0]] <= inst_data.length;
                            pc <= pc + 1'b1;
                        end
                        OP_ALU: begin
                            if (!inst_data.sdp_en) begin
                                scalar_rf[inst_data.arf_addr[2:0]] <= !inst_data.clr_parf
                                    ? (scalar_rf[inst_data.wgt_rf_addr[2:0]] + scalar_rf[inst_data.parf_addr[2:0]])
                                    : (scalar_rf[inst_data.wgt_rf_addr[2:0]] - scalar_rf[inst_data.parf_addr[2:0]]);
                            end else begin
                                scalar_rf[inst_data.arf_addr[2:0]] <= !inst_data.clr_parf
                                    ? (scalar_rf[inst_data.wgt_rf_addr[2:0]] + inst_data.length)
                                    : (scalar_rf[inst_data.wgt_rf_addr[2:0]] - inst_data.length);
                            end
                            pc <= pc + 1'b1;
                        end
                        OP_JMP: begin
                            pc <= inst_data.sram_addr[$clog2(INST_DEPTH)-1:0];
                        end
                        OP_BNZ: begin
                            if (scalar_rf[inst_data.arf_addr[2:0]] != '0) begin
                                scalar_rf[inst_data.arf_addr[2:0]] <= scalar_rf[inst_data.arf_addr[2:0]] - 1'b1;
                                pc <= inst_data.sram_addr[$clog2(INST_DEPTH)-1:0];
                            end else begin
                                pc <= pc + 1'b1;
                            end
                        end
                        OP_LD_SDP: begin
                            pc <= pc + 1'b1;
                        end
                        default: begin
                            // EXEC 类指令：pc 在 EXEC 退出时更新
                        end
                    endcase
                end

                EXEC_LD_WGT, EXEC_LD_ARF, EXEC_MAC, EXEC_ST_OFM: begin
                    if (cnt == inst_reg.length - 1) begin
                        // EX 结束
                        if (pf_direct) begin
                            // ---- 流水线直通：pf_inst 是 EXEC 类，零空泡切换 ----
                            inst_reg <= pf_inst;
                            pc       <= pf_pc;       // pf_pc = 预取指令本身的 PC
                            cnt      <= '0;
                        end else begin
                            cnt <= '0;
                            pc  <= pc + 1'b1;
                            pf_valid <= 1'b0;
                        end
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                EXEC_LDnMAC: begin
                    if (cnt == ldnmac_end_cnt) begin
                        // EX 结束
                        if (pf_direct) begin
                            // ---- 流水线直通：pf_inst 是 EXEC 类，零空泡切换 ----
                            inst_reg <= pf_inst;
                            pc       <= pf_pc;
                            cnt      <= '0;
                        end else begin
                            cnt <= '0;
                            pc  <= pc + 1'b1;
                            pf_valid <= 1'b0;
                        end
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
    // 5. 状态机跳转逻辑（含流水线直通分支）
    //=============================================================================

    always_comb begin
        next_state = current_state;

        case (current_state)
            IDLE: begin
                if (start) next_state = FETCH;
            end

            FETCH: begin
                next_state = DECODE;
            end

            DECODE: begin
                case (inst_data.opcode)
                    OP_NOP:       next_state = FETCH;
                    OP_LD_WGT:    next_state = EXEC_LD_WGT;
                    OP_LD_ARF:    next_state = EXEC_LD_ARF;
                    OP_MAC_RUN:   next_state = EXEC_MAC;
                    OP_ST_OFM:    next_state = EXEC_ST_OFM;
                    OP_LD1MAC,
                    OP_LDnMAC:    next_state = EXEC_LDnMAC;
                    OP_LI, OP_ALU,
                    OP_JMP, OP_BNZ,
                    OP_LD_SDP:    next_state = FETCH;
                    default:      next_state = FINISH;
                endcase
            end

            EXEC_LD_WGT, EXEC_LD_ARF, EXEC_MAC, EXEC_ST_OFM: begin
                if (cnt == inst_reg.length - 1) begin
                    if (pf_direct)
                        next_state = opcode_to_exec_state(pf_inst.opcode);
                    else
                        next_state = FETCH;  // 标量预取或无预取：正常取指
                end
            end

            EXEC_LDnMAC: begin
                if (cnt == ldnmac_end_cnt) begin
                    if (pf_direct)
                        next_state = opcode_to_exec_state(pf_inst.opcode);
                    else
                        next_state = FETCH;
                end
            end

            FINISH: begin
                next_state = FINISH;
            end

            default: next_state = IDLE;
        endcase
    end

    //=============================================================================
    // 6. 输出控制信号逻辑 (纯组合逻辑)
    //=============================================================================
    always_comb begin
        // 默认
        inst_re         = 1'b0;
        inst_addr       = '0;
        wb_re           = 1'b0;
        wb_raddr        = '0;
        ifb_re          = 1'b0;
        ifb_raddr       = '0;

        ofb_we          = ofb_we_d2;
        ofb_waddr       = ofb_waddr_d2;
        sdp_en_out      = sdp_en_d2;

        wrf_we          = wrf_we_d1;
        wrf_waddr       = wrf_waddr_d1;
        arf_we          = arf_we_d1;
        arf_waddr       = arf_waddr_d1;

        arf_read_addr   = '0;
        compute_en      = 1'b0;
        wrf_raddr       = '0;
        parf_addr       = '0;
        parf_clear      = 1'b0;
        parf_we         = 1'b0;
        done            = 1'b0;
        sdp_shift_we    = 1'b0;
        sdp_shift_wdata = '0;

        // 预取 IF 请求（优先级高于 FETCH 状态）
        // pf_trigger 拉高时，组合逻辑立即发出 inst_re 和 inst_addr
        if (pf_trigger) begin
            inst_re   = 1'b1;
            inst_addr = pc + 1'b1;
        end

        case (current_state)
            IDLE: begin
                // 等待
            end

            FETCH: begin
                // 仅当预取未占用总线时发出 FETCH 的 IF
                if (!pf_trigger) begin
                    inst_re   = 1'b1;
                    inst_addr = pc;
                end
            end

            DECODE: begin
                // 若预取了标量/FINISH，在此处直接用 pf_inst（已锁存在 pf_inst）
                // 判断使用 pf_valid 路径还是正常 inst_data 路径
                // 注意：进入 DECODE 状态有两种方式：
                //   (a) 正常 FETCH→DECODE：inst_data 是从 INST_SRAM 来的当前指令
                //   (b) 直通后预取了标量指令：pf_valid && !pf_is_exec
                // 两种情况下 inst_data（来自 SRAM）的内容都已正确锁存，此处仅驱动 SDP
                if (inst_data.opcode == OP_LD_SDP) begin
                    sdp_shift_we    = 1'b1;
                    sdp_shift_wdata = inst_data.ld_arf_addr;
                end
            end

            EXEC_LD_WGT: begin
                if (cnt < inst_reg.length) begin
                    wb_re    = 1'b1;
                    wb_raddr = inst_reg.sram_addr + cnt[14:0];
                end
            end

            EXEC_LD_ARF: begin
                if (cnt < inst_reg.length) begin
                    ifb_re    = 1'b1;
                    ifb_raddr = scalar_rf[0][14:0] + inst_reg.sram_addr + cnt[14:0];
                end
            end

            EXEC_MAC: begin
                compute_en    = 1'b1;
                wrf_raddr     = inst_reg.wgt_rf_addr;
                arf_read_addr = inst_reg.arf_addr + cnt[4:0];
                parf_addr     = inst_reg.parf_addr + cnt[4:0];
                parf_we       = 1'b1;
                if (inst_reg.clr_parf) parf_clear = 1'b1;
            end

            EXEC_ST_OFM: begin
                parf_addr = inst_reg.parf_addr + cnt[4:0];
            end

            EXEC_LDnMAC: begin
                eff_stride  = (inst_reg.stride == 3'd0) ? 3'd1 : inst_reg.stride;
                strided_off = cnt[14:0] * {12'b0, eff_stride};

                // IFB 读：cnt=0..ld_len-1
                if (cnt < {11'b0, ldnmac_ld_len}) begin
                    ifb_re    = 1'b1;
                    ifb_raddr = scalar_rf[0][14:0] + inst_reg.sram_addr + strided_off;
                end

                // MAC：cnt=1..mac_len
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
