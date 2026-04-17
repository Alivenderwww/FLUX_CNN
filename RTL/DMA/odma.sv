`timescale 1ns/1ps

// =============================================================================
// odma.sv  --  Output DMA Engine (OFB SRAM → DDR, S2MM)
//
// 功能：按 descriptor `(dst_base, byte_len)` 把内部 OFB SRAM 的连续内容搬到
// 外部 DDR。AXI4 M，只用 AW + W + B 通道。
//
// 描述符：
//   dst_base : DDR 起始字节地址（假设 4KB 对齐）
//   byte_len : 搬运字节数（必须是 DATA_W/8 = 16 字节的整数倍）
//
// Burst 切分：最大 256 beat；ceil(byte_len/4096) 条 burst。
//
// 状态机：
//   S_IDLE → start → S_AW → aw_fire → S_PREFETCH → (1 cycle) → S_W
//   S_W 每 beat 发 WDATA=ofb_rdata。WLAST 在 burst_sent == burst_beats-1 时
//   断言。w_last_fire → S_B。S_B 等 BVALID。
//   b_fire 时：still more beats → S_AW；beats_remaining==0 → S_DONE。
//
// SRAM 读时序（sram_model 1 拍延迟）：
//   S_PREFETCH 拉 re=1、raddr=rd_ptr；下拍 rdata=mem[rd_ptr] 到位进入 S_W。
//   S_W 每拍 re=1、raddr=rd_ptr + (w_fire ? 1 : 0)——w_fire 时预读下一 beat；
//   stall 时保持 raddr 不动，rdata 不变（需 sram_model 在 re=1 持续读）。
//
// 复位：state / r_done 同步复位；数据路径 ptr / burst_* 靠 start 初始化（§6）。
// =============================================================================

module odma #(
    parameter int ADDR_W       = 32,
    parameter int DATA_W       = 128,
    parameter int M_ID         = 2,
    parameter int SRAM_ADDR_W  = 13,
    parameter int LEN_W        = 24
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // ---- Control ----
    input  logic                    start,
    output logic                    done,
    output logic                    busy,

    // ---- Descriptor ----
    input  logic [ADDR_W-1:0]       dst_base,
    input  logic [LEN_W-1:0]        byte_len,

    // ---- AXI4 M (AR/R 内部 tie 0) ----
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

    // ---- OFB SRAM read port ----
    output logic                    ofb_re,
    output logic [SRAM_ADDR_W-1:0]  ofb_raddr,
    input  logic [DATA_W-1:0]       ofb_rdata
);

    localparam int BYTES_PER_BEAT = DATA_W / 8;
    localparam int BEAT_SHIFT     = $clog2(BYTES_PER_BEAT);
    localparam int MAX_BEATS      = 256;

    // =========================================================================
    // 读通道 tie 0（ODMA 只写）
    // =========================================================================
    assign M_ARID    = '0;
    assign M_ARADDR  = '0;
    assign M_ARLEN   = '0;
    assign M_ARBURST = 2'b01;
    assign M_ARVALID = 1'b0;
    assign M_RREADY  = 1'b1;

    assign M_AWID    = '0;
    assign M_AWBURST = 2'b01;     // INCR

    // =========================================================================
    // 状态机
    // =========================================================================
    typedef enum logic [2:0] {
        S_IDLE     = 3'd0,
        S_AW       = 3'd1,
        S_PREFETCH = 3'd2,
        S_W        = 3'd3,
        S_B        = 3'd4,
        S_DONE     = 3'd5
    } state_t;
    state_t state, state_next;

    // =========================================================================
    // 寄存器
    // =========================================================================
    logic [ADDR_W-1:0]       cur_addr;
    logic [LEN_W-1:0]        beats_remaining;
    logic [SRAM_ADDR_W-1:0]  rd_ptr;
    logic [8:0]              burst_beats;
    logic [8:0]              burst_sent;
    logic                    r_done;

    // =========================================================================
    // 派生量 / 命名事件
    // =========================================================================
    logic [8:0] beats_to_issue;
    always_comb begin
        if (beats_remaining >= LEN_W'(MAX_BEATS))
            beats_to_issue = 9'd256;
        else
            beats_to_issue = {1'b0, beats_remaining[7:0]};
    end

    logic aw_fire, w_fire, w_last_fire, b_fire;
    assign aw_fire     = M_AWVALID && M_AWREADY;
    assign w_fire      = M_WVALID  && M_WREADY;
    assign w_last_fire = w_fire    && M_WLAST;
    assign b_fire      = M_BVALID  && M_BREADY;

    // =========================================================================
    // 端口输出
    // =========================================================================
    assign M_AWADDR = cur_addr;
    assign M_AWLEN  = beats_to_issue[7:0] - 8'd1;
    assign M_AWVALID= (state == S_AW);

    assign M_WDATA  = ofb_rdata;
    assign M_WSTRB  = '1;
    assign M_WVALID = (state == S_W);
    assign M_WLAST  = (state == S_W) && (burst_sent == (burst_beats - 9'd1));

    assign M_BREADY = (state == S_B);

    assign ofb_re    = (state == S_PREFETCH) || (state == S_W);
    assign ofb_raddr = rd_ptr + ((state == S_W && w_fire) ? SRAM_ADDR_W'(1) : SRAM_ADDR_W'(0));

    assign done = r_done;
    assign busy = (state != S_IDLE) && (state != S_DONE);

    // =========================================================================
    // FSM 三段式
    // =========================================================================
    always_comb begin
        state_next = state;
        case (state)
            S_IDLE     : if (start)        state_next = S_AW;
            S_AW       : if (aw_fire)      state_next = S_PREFETCH;
            S_PREFETCH :                    state_next = S_W;
            S_W        : if (w_last_fire)  state_next = S_B;
            S_B        : if (b_fire)       state_next = (beats_remaining == LEN_W'(0)) ? S_DONE : S_AW;
            S_DONE     : if (start)        state_next = S_AW;
            default    :                    state_next = S_IDLE;
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
        if      (start)   cur_addr <= dst_base;
        else if (aw_fire) cur_addr <= cur_addr + (ADDR_W'(beats_to_issue) << BEAT_SHIFT);
        else              cur_addr <= cur_addr;
    end

    always_ff @(posedge clk) begin
        if      (start)  beats_remaining <= byte_len >> BEAT_SHIFT;
        else if (w_fire) beats_remaining <= beats_remaining - 1;
        else             beats_remaining <= beats_remaining;
    end

    always_ff @(posedge clk) begin
        if      (start)  rd_ptr <= '0;
        else if (w_fire) rd_ptr <= rd_ptr + 1;
        else             rd_ptr <= rd_ptr;
    end

    always_ff @(posedge clk) begin
        if      (aw_fire) burst_beats <= beats_to_issue;
        else              burst_beats <= burst_beats;
    end

    always_ff @(posedge clk) begin
        if      (aw_fire) burst_sent <= '0;
        else if (w_fire)  burst_sent <= burst_sent + 1;
        else              burst_sent <= burst_sent;
    end

    // =========================================================================
    // done 锁存（控制路径，复位必须）
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (!rst_n)                                r_done <= 1'b0;
        else if (start)                                 r_done <= 1'b0;
        else if (b_fire && beats_remaining == LEN_W'(0))r_done <= 1'b1;
        else                                            r_done <= r_done;
    end

endmodule
