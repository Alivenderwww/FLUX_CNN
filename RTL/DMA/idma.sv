`timescale 1ns/1ps

// =============================================================================
// idma.sv  --  Input DMA Engine (DDR → IFB SRAM, MM2S)
//
// 功能：从外部 DDR 按 descriptor `(src_base, byte_len)` 读入 IFM 数据，写到
// 内部 IFB SRAM（从 SRAM addr 0 开始线性写）。AXI4 M，只用 AR + R 通道。
//
// 描述符：
//   src_base : 外部 DDR 起始字节地址（假设 4KB 对齐；未对齐场景 v2 支持）
//   byte_len : 要搬运的字节数（必须是 DATA_W/8 = 16 字节的整数倍）
//
// Burst 切分：
//   每 burst 最多 256 beat（AXI4 上限），即 4KB；ceil(byte_len/4096) 条 burst
//
// 状态机：
//   S_IDLE → start → S_AR (发 ARLEN burst) → S_R (收 R beat，每拍写 IFB)
//          → RLAST: 还有剩余 → S_AR；否则 → S_DONE
//   S_DONE → start → S_AR (支持下一层重用)
//
// 接口契约：
//   - AW/W/B 通道内部 tie 0（IDMA 只读）；接 axi_m_mux 时 mux 看 ARVALID 决定
//   - ifb_we / waddr / wdata：组合 passthrough R handshake + wr_ptr，直接驱动
//     外部 SRAM 的同步写端口
//
// 复位：控制路径（FSM + done 锁存）同步复位；数据路径（cur_addr / beats /
// wr_ptr）不复位，start 触发时初始化（§6）。
// =============================================================================

module idma #(
    parameter int ADDR_W       = 32,       // AXI 外部地址位宽
    parameter int DATA_W       = 128,      // AXI data 宽度（= IFB 字宽）
    parameter int M_ID         = 2,        // AXI ID 位宽
    parameter int SRAM_ADDR_W  = 13,       // IFB SRAM 深度位宽（8192 words）
    parameter int LEN_W        = 24        // byte_len 位宽
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // ---- Control ----
    input  logic                    start,            // 1-拍脉冲
    output logic                    done,             // 锁存，start 时清零
    output logic                    busy,             // 高电平 = 运行中

    // ---- Descriptor (static during run) ----
    input  logic [ADDR_W-1:0]       src_base,
    input  logic [LEN_W-1:0]        byte_len,

    // ---- AXI4 M (to DDR via axi_m_mux) ----
    output logic [M_ID-1:0]         M_AWID,
    output logic [ADDR_W-1:0]       M_AWADDR,
    output logic [7:0]              M_AWLEN,
    output logic [1:0]              M_AWBURST,
    output logic                    M_AWVALID,
    input  logic                    M_AWREADY,

    output logic [DATA_W-1:0]       M_WDATA,
    output logic [DATA_W/8-1:0]     M_WSTRB,
    output logic                    M_WLAST,
    output logic                    M_WVALID,
    input  logic                    M_WREADY,

    input  logic [M_ID-1:0]         M_BID,
    input  logic [1:0]              M_BRESP,
    input  logic                    M_BVALID,
    output logic                    M_BREADY,

    output logic [M_ID-1:0]         M_ARID,
    output logic [ADDR_W-1:0]       M_ARADDR,
    output logic [7:0]              M_ARLEN,
    output logic [1:0]              M_ARBURST,
    output logic                    M_ARVALID,
    input  logic                    M_ARREADY,

    input  logic [M_ID-1:0]         M_RID,
    input  logic [DATA_W-1:0]       M_RDATA,
    input  logic [1:0]              M_RRESP,
    input  logic                    M_RLAST,
    input  logic                    M_RVALID,
    output logic                    M_RREADY,

    // ---- IFB SRAM write port ----
    output logic                    ifb_we,
    output logic [SRAM_ADDR_W-1:0]  ifb_waddr,
    output logic [DATA_W-1:0]       ifb_wdata
);

    localparam int BYTES_PER_BEAT = DATA_W / 8;           // 16 for 128-bit
    localparam int BEAT_SHIFT     = $clog2(BYTES_PER_BEAT);
    localparam int MAX_BEATS      = 256;                   // AXI4 burst 上限

    // =========================================================================
    // 写通道 tie 0（IDMA 只读）
    // =========================================================================
    assign M_AWID    = '0;
    assign M_AWADDR  = '0;
    assign M_AWLEN   = '0;
    assign M_AWBURST = 2'b01;
    assign M_AWVALID = 1'b0;
    assign M_WDATA   = '0;
    assign M_WSTRB   = '0;
    assign M_WLAST   = 1'b0;
    assign M_WVALID  = 1'b0;
    assign M_BREADY  = 1'b1;   // 始终吞掉任何误回来的 B resp

    assign M_ARID    = '0;
    assign M_ARBURST = 2'b01;  // INCR

    // =========================================================================
    // 状态机
    // =========================================================================
    typedef enum logic [1:0] {
        S_IDLE = 2'd0,
        S_AR   = 2'd1,
        S_R    = 2'd2,
        S_DONE = 2'd3
    } state_t;
    state_t state, state_next;

    // =========================================================================
    // 寄存器
    // =========================================================================
    logic [ADDR_W-1:0]       cur_addr;         // 下一 burst 的 AXI 起始字节地址
    logic [LEN_W-1:0]        beats_remaining;  // 还没收完的总 beat 数
    logic [SRAM_ADDR_W-1:0]  wr_ptr;           // IFB SRAM 写指针（word）
    logic                    r_done;           // 锁存 done

    // =========================================================================
    // 派生量
    // =========================================================================
    logic [8:0] beats_to_issue;   // 1..256（9-bit 容纳 256）
    always_comb begin
        if (beats_remaining >= LEN_W'(MAX_BEATS))
            beats_to_issue = 9'd256;
        else
            beats_to_issue = {1'b0, beats_remaining[7:0]};
    end

    logic ar_fire, r_fire, r_last_fire;
    assign ar_fire     = M_ARVALID && M_ARREADY;
    assign r_fire      = M_RVALID  && M_RREADY;
    assign r_last_fire = r_fire   && M_RLAST;

    // =========================================================================
    // 端口输出
    // =========================================================================
    assign M_ARADDR  = cur_addr;
    assign M_ARLEN   = beats_to_issue[7:0] - 8'd1;
    assign M_ARVALID = (state == S_AR);
    assign M_RREADY  = (state == S_R);

    assign ifb_we    = r_fire;
    assign ifb_waddr = wr_ptr;
    assign ifb_wdata = M_RDATA;

    assign done = r_done;
    assign busy = (state != S_IDLE) && (state != S_DONE);

    // =========================================================================
    // FSM 三段式
    // =========================================================================
    always_comb begin
        state_next = state;
        case (state)
            S_IDLE : if (start)       state_next = S_AR;
            S_AR   : if (ar_fire)     state_next = S_R;
            S_R    : if (r_last_fire) state_next = (beats_remaining == LEN_W'(1)) ? S_DONE : S_AR;
            S_DONE : if (start)       state_next = S_AR;
            default:                   state_next = S_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= state_next;
    end

    // =========================================================================
    // 数据路径寄存器（start 初始化，无复位）
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (start)   cur_addr <= src_base;
        else if (ar_fire) cur_addr <= cur_addr + (ADDR_W'(beats_to_issue) << BEAT_SHIFT);
        else              cur_addr <= cur_addr;
    end

    always_ff @(posedge clk) begin
        if      (start)  beats_remaining <= byte_len >> BEAT_SHIFT;
        else if (r_fire) beats_remaining <= beats_remaining - 1;
        else             beats_remaining <= beats_remaining;
    end

    always_ff @(posedge clk) begin
        if      (start)  wr_ptr <= '0;
        else if (r_fire) wr_ptr <= wr_ptr + 1;
        else             wr_ptr <= wr_ptr;
    end

    // =========================================================================
    // done 锁存：控制路径，复位必须
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (!rst_n)                                                r_done <= 1'b0;
        else if (start)                                                 r_done <= 1'b0;
        else if (r_last_fire && beats_remaining == LEN_W'(1))           r_done <= 1'b1;
        else                                                            r_done <= r_done;
    end

endmodule
