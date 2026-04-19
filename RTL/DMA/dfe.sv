`timescale 1ns/1ps

// =============================================================================
// dfe.sv  --  Descriptor Fetch Engine (DDR → desc_fifo, AXI4 read master)
//
// Phase C-1: 从 DDR 拉 N 条 256-bit descriptor 到片内 desc_fifo。
// 每条 descriptor = 32 byte = 2 个 128-bit AXI beat；little-endian word
// ordering（第 1 个 beat 填 rd_data[127:0] = word[3..0]，第 2 个 beat 填
// rd_data[255:128] = word[7..4]）。
//
// 限制（Phase C-1）：desc_count ≤ 128（一次 AR burst 最多 256 beat）。更大
// 的 list 未来加分 burst 调度。
//
// 反压：FIFO 满时 R 通道 r_ready=0 反压 AXI。AR 一次性发完，R 通道自然节流。
//
// 复位：控制路径 FSM 同步复位；数据路径（addr / counters）由 start 初始化。
// =============================================================================

module dfe #(
    parameter int ADDR_W = 32,
    parameter int DATA_W = 128,
    parameter int M_ID   = 2,
    parameter int FIFO_W = 256
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // ---- Control ----
    input  logic                    start,
    output logic                    done,
    output logic                    busy,

    // ---- Descriptor list ----
    input  logic [ADDR_W-1:0]       desc_list_base,
    input  logic [15:0]             desc_count,       // 条数（≤ 128）

    // ---- AXI4 M (read only; AW/W/B 通道 tie 0 in core_top) ----
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

    // ---- FIFO write port ----
    output logic [FIFO_W-1:0]       fifo_wdata,
    output logic                    fifo_we,
    input  logic                    fifo_full
);

    localparam [1:0] AXI_BURST_INCR = 2'b01;

    // =========================================================================
    // FSM
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
    logic [ADDR_W-1:0]  r_base;
    logic [15:0]        r_count;          // descriptor 条数
    logic [15:0]        r_beats_total;    // = r_count * 2
    logic [15:0]        r_beats_rcvd;     // 已接收 beat 数
    logic               r_phase;          // 0 = 期待 beat 0 (low), 1 = 期待 beat 1 (high)
    logic [DATA_W-1:0]  r_low_latch;      // 缓存下半 descriptor
    logic               r_done;

    // =========================================================================
    // AXI AR 通道（一次性发完整个 burst）
    // =========================================================================
    assign M_ARID    = '0;
    assign M_ARADDR  = r_base;
    assign M_ARLEN   = 8'(r_beats_total - 16'd1);   // AXI beats = ARLEN + 1
    assign M_ARBURST = AXI_BURST_INCR;
    assign M_ARVALID = (state == S_AR);

    // =========================================================================
    // AXI R 通道
    //   phase=0: 无条件 ready，latch low half
    //   phase=1: ready = !fifo_full，fire 时写 FIFO 并 toggle phase
    // =========================================================================
    logic r_fire;
    assign M_RREADY = (state == S_R) && (r_phase == 1'b0 || !fifo_full);
    assign r_fire   = M_RVALID && M_RREADY;

    assign fifo_wdata = {M_RDATA, r_low_latch};
    assign fifo_we    = r_fire && (r_phase == 1'b1);

    // =========================================================================
    // FSM (三段式)
    // =========================================================================
    always_comb begin
        state_next = state;
        case (state)
            S_IDLE : if (start)                                     state_next = S_AR;
            S_AR   : if (M_ARREADY)                                 state_next = S_R;
            S_R    : if (r_fire && M_RLAST)                         state_next = S_DONE;
            S_DONE : ;
            default:                                                state_next = S_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= state_next;
    end

    assign busy = (state == S_AR) || (state == S_R);
    assign done = r_done;

    // =========================================================================
    // 数据路径寄存器（start 时初始化，§6 无复位）
    // =========================================================================
    always_ff @(posedge clk) begin
        if (start) begin
            r_base        <= desc_list_base;
            r_count       <= desc_count;
            r_beats_total <= {desc_count, 1'b0};       // × 2
            r_beats_rcvd  <= '0;
        end else if (r_fire) begin
            r_beats_rcvd  <= r_beats_rcvd + 16'd1;
        end
    end

    always_ff @(posedge clk) begin
        if (start)       r_phase <= 1'b0;
        else if (r_fire) r_phase <= ~r_phase;
    end

    always_ff @(posedge clk) begin
        if (r_fire && r_phase == 1'b0) r_low_latch <= M_RDATA;
    end

    // r_done 是外部观测信号 → 控制路径（§6.1），同步复位
    always_ff @(posedge clk) begin
        if (!rst_n)                              r_done <= 1'b0;
        else if (start)                          r_done <= 1'b0;
        else if (r_fire && M_RLAST)              r_done <= 1'b1;
    end

endmodule
