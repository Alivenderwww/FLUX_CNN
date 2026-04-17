`timescale 1ns/1ps

// =============================================================================
// idma.sv  --  Input DMA Engine (DDR → IFB SRAM, MM2S)
//
// 支持两种模式（由 cfg_idma_streaming 切换）：
//
// Batch 模式 (cfg_idma_streaming=0)：v1 行为
//   按 descriptor (src_base, byte_len) 一次性搬运整块 IFM；wr_ptr 从 0 线性
//   递增；byte_len 通常 = 整张图 IFB 字节数。用于 h_in 能整图装下的场景。
//
// Streaming 模式 (cfg_idma_streaming=1)：v2 row-ring
//   IFB 作 row-level ring buffer。IDMA 每拍 pacing：
//     * 每次 AR 最多发一整行（W_IN beats）或 256 beats (AXI 上限) — 取小
//     * 每行完成后检查 rows_written - rows_consumed；若达到 strip_rows 进
//       S_WAIT 等 line_buffer 消费
//     * wr_ptr 在 cfg_ifb_cin_step (= strip_rows * W_IN) 处 wrap
//     * 整体停止条件：rows_written == cfg_h_in_total
//   Streaming 前提 cin_slices = 1（Phase B 限制）；cin_slices > 1 留 Phase D
//
// 复位：控制路径（FSM + r_done）同步复位；数据路径（addr / counters）由
// start 初始化（§6）。
// =============================================================================

module idma #(
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

    // ---- Descriptor (batch 用) ----
    input  logic [ADDR_W-1:0]       src_base,
    input  logic [LEN_W-1:0]        byte_len,

    // ---- Streaming 配置 ----
    input  logic                    cfg_idma_streaming,
    input  logic [15:0]             cfg_h_in_total,    // 整图行数
    input  logic [7:0]              cfg_ifb_strip_rows,// IFB ring 行数
    input  logic [15:0]             cfg_w_in,          // 每行 beats 数
    input  logic [19:0]             cfg_ifb_cin_step,  // ring wrap 模数 (= strip_rows*W_IN)
    input  logic [15:0]             rows_consumed,     // 来自 line_buffer

    // ---- AXI4 M ----
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

    localparam int BYTES_PER_BEAT = DATA_W / 8;
    localparam int BEAT_SHIFT     = $clog2(BYTES_PER_BEAT);
    localparam int MAX_BEATS      = 256;

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
    assign M_BREADY  = 1'b1;

    assign M_ARID    = '0;
    assign M_ARBURST = 2'b01;

    // =========================================================================
    // 状态机（加 S_WAIT for streaming）
    // =========================================================================
    typedef enum logic [2:0] {
        S_IDLE = 3'd0,
        S_AR   = 3'd1,
        S_R    = 3'd2,
        S_WAIT = 3'd3,   // streaming：ring 满，等 line_buffer 消费
        S_DONE = 3'd4
    } state_t;
    state_t state, state_next;

    // =========================================================================
    // 寄存器
    // =========================================================================
    logic [ADDR_W-1:0]       cur_addr;         // AXI 起始字节地址
    logic [LEN_W-1:0]        beats_remaining;  // batch 剩余 beats
    logic [15:0]             row_beats_remaining; // streaming 当前行剩余 beats
    logic [15:0]             rows_written;     // streaming 已写完的行数
    logic [SRAM_ADDR_W-1:0]  wr_ptr;
    logic                    r_done;

    // =========================================================================
    // 派生量
    // =========================================================================
    logic ar_fire, r_fire, r_last_fire;
    assign ar_fire     = M_ARVALID && M_ARREADY;
    assign r_fire      = M_RVALID  && M_RREADY;
    assign r_last_fire = r_fire   && M_RLAST;

    logic streaming_row_end;  // 当前 r_fire 拍完成一整行
    assign streaming_row_end = cfg_idma_streaming && r_fire && (row_beats_remaining == 16'd1);

    logic streaming_all_done; // 整图流完
    assign streaming_all_done = streaming_row_end && (rows_written == cfg_h_in_total - 16'd1);

    // Ring 满判定（streaming only）
    //   已灌入未消费的行 >= strip_rows 时不能再发 AR
    logic ring_full;
    assign ring_full = cfg_idma_streaming &&
                       ((rows_written - rows_consumed) >= {8'd0, cfg_ifb_strip_rows});

    // beats_to_issue：batch 用 beats_remaining，streaming 用 row_beats_remaining
    logic [8:0] beats_to_issue;
    always_comb begin
        if (cfg_idma_streaming) begin
            if (row_beats_remaining >= 16'd256)  beats_to_issue = 9'd256;
            else                                  beats_to_issue = {1'b0, row_beats_remaining[7:0]};
        end else begin
            if (beats_remaining >= LEN_W'(MAX_BEATS)) beats_to_issue = 9'd256;
            else                                       beats_to_issue = {1'b0, beats_remaining[7:0]};
        end
    end

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
    logic batch_last_beat;
    assign batch_last_beat = !cfg_idma_streaming && r_last_fire && (beats_remaining == LEN_W'(1));

    always_comb begin
        state_next = state;
        case (state)
            S_IDLE : if (start) state_next = S_AR;
            S_AR   : begin
                if (cfg_idma_streaming && ring_full) state_next = S_WAIT;
                else if (ar_fire)                     state_next = S_R;
            end
            S_R    : if (r_last_fire) begin
                if (batch_last_beat)          state_next = S_DONE;
                else if (streaming_all_done)  state_next = S_DONE;
                else                          state_next = S_AR;
            end
            S_WAIT : if (!ring_full)   state_next = S_AR;
            S_DONE : if (start)        state_next = S_AR;
            default:                    state_next = S_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= state_next;
    end

    // =========================================================================
    // 数据路径寄存器
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (start)   cur_addr <= src_base;
        else if (ar_fire) cur_addr <= cur_addr + (ADDR_W'(beats_to_issue) << BEAT_SHIFT);
        else              cur_addr <= cur_addr;
    end

    // beats_remaining：batch 用；streaming 下不参与 done 判定，但保留递减逻辑
    always_ff @(posedge clk) begin
        if      (start)  beats_remaining <= byte_len >> BEAT_SHIFT;
        else if (r_fire) beats_remaining <= beats_remaining - 1;
        else             beats_remaining <= beats_remaining;
    end

    // streaming row_beats_remaining：每行开头 = W_IN，r_fire 减 1，行末回 W_IN
    always_ff @(posedge clk) begin
        if      (start)              row_beats_remaining <= cfg_w_in;
        else if (streaming_row_end)  row_beats_remaining <= cfg_w_in;   // 下一行
        else if (r_fire)             row_beats_remaining <= row_beats_remaining - 16'd1;
        else                         row_beats_remaining <= row_beats_remaining;
    end

    // rows_written: streaming only，每完成一行 +1
    always_ff @(posedge clk) begin
        if      (start)              rows_written <= 16'd0;
        else if (streaming_row_end)  rows_written <= rows_written + 16'd1;
        else                         rows_written <= rows_written;
    end

    // wr_ptr: batch 线性；streaming 在 cfg_ifb_cin_step 处 wrap
    logic [SRAM_ADDR_W-1:0] wr_ptr_wrap_limit_m1;
    assign wr_ptr_wrap_limit_m1 = cfg_ifb_cin_step[SRAM_ADDR_W-1:0] - SRAM_ADDR_W'(1);

    always_ff @(posedge clk) begin
        if      (start)  wr_ptr <= '0;
        else if (r_fire) begin
            if (cfg_idma_streaming && (wr_ptr == wr_ptr_wrap_limit_m1))
                wr_ptr <= '0;
            else
                wr_ptr <= wr_ptr + 1;
        end
        else             wr_ptr <= wr_ptr;
    end

    // =========================================================================
    // done 锁存
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (!rst_n)              r_done <= 1'b0;
        else if (start)               r_done <= 1'b0;
        else if (batch_last_beat)     r_done <= 1'b1;
        else if (streaming_all_done)  r_done <= 1'b1;
        else                          r_done <= r_done;
    end

endmodule
