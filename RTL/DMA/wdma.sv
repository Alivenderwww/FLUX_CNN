`timescale 1ns/1ps

// =============================================================================
// wdma.sv  --  Weight DMA Engine (DDR → WB SRAM, MM2S with 128→2048 packing)
//
// 功能：按 descriptor `(src_base, byte_len)` 从 DDR 读权重，每 16 个 128-bit
// AXI beat 打包成 1 个 2048-bit WB SRAM 字并写入。
//
// WB 字的 128-bit 子字布局（匹配 gen_isa_test.py 里 wb.txt 的每行 hex 字节序）：
//   bits [127:0]      = beat 0   (col 0, PE 0..15)
//   bits [255:128]    = beat 1   (col 1)
//   ...
//   bits [2047:1920]  = beat 15  (col 15)
// 即：beat 0 在 LSB，beat 15 在 MSB。WB_DATA_W = 2048 = 16 × DATA_W (128)。
//
// 打包实现：2048-bit 移位寄存器，每 beat 右移 128-bit、新 beat 进 MSB。16 次
// 移位后 LSB 装的是 beat 0。第 16 beat（beat_in_word==15）同拍用组合
// `{M_RDATA, word_buffer[2047:128]}` 直接凑齐，省一拍。
//
// 描述符：
//   src_base : DDR 字节地址（假设 4KB 对齐）
//   byte_len : 必须是 WB_DATA_W/8 = 256 字节的整数倍
//
// Burst 切分：max 256 beat（= 16 个 WB 字）。AR 跨 burst 不破坏 beat_in_word
// 连续性（burst 间 r_fire=0，counter 自然保持）。
//
// 状态机：S_IDLE → S_AR → S_R → (more: S_AR；else: S_DONE)
// 复位：控制路径（state、r_done）同步复位；数据路径（ptr/buffer）start 初始化。
// =============================================================================

module wdma #(
    parameter int ADDR_W       = 32,
    parameter int DATA_W       = 128,
    parameter int WB_DATA_W    = 2048,
    parameter int M_ID         = 2,
    parameter int SRAM_ADDR_W  = 13,
    parameter int LEN_W        = 24
)(
    input  logic                      clk,
    input  logic                      rst_n,

    // ---- Control ----
    input  logic                      start,
    output logic                      done,
    output logic                      busy,

    // ---- Descriptor ----
    input  logic [ADDR_W-1:0]         src_base,
    input  logic [LEN_W-1:0]          byte_len,

    // ---- AXI4 M (AW/W/B tied off, 只读) ----
    output logic [M_ID-1:0]           M_AWID,
    output logic [ADDR_W-1:0]         M_AWADDR,
    output logic [7:0]                M_AWLEN,
    output logic [1:0]                M_AWBURST,
    output logic                      M_AWVALID,
    input  logic                      M_AWREADY,

    output logic [DATA_W-1:0]         M_WDATA,
    output logic [DATA_W/8-1:0]       M_WSTRB,
    output logic                      M_WLAST,
    output logic                      M_WVALID,
    input  logic                      M_WREADY,

    input  logic [M_ID-1:0]           M_BID,
    input  logic [1:0]                M_BRESP,
    input  logic                      M_BVALID,
    output logic                      M_BREADY,

    output logic [M_ID-1:0]           M_ARID,
    output logic [ADDR_W-1:0]         M_ARADDR,
    output logic [7:0]                M_ARLEN,
    output logic [1:0]                M_ARBURST,
    output logic                      M_ARVALID,
    input  logic                      M_ARREADY,

    input  logic [M_ID-1:0]           M_RID,
    input  logic [DATA_W-1:0]         M_RDATA,
    input  logic [1:0]                M_RRESP,
    input  logic                      M_RLAST,
    input  logic                      M_RVALID,
    output logic                      M_RREADY,

    // ---- WB SRAM write port (2048-bit 宽) ----
    output logic                      wb_we,
    output logic [SRAM_ADDR_W-1:0]    wb_waddr,
    output logic [WB_DATA_W-1:0]      wb_wdata
);

    localparam int BYTES_PER_BEAT  = DATA_W / 8;           // 16
    localparam int BEAT_SHIFT      = $clog2(BYTES_PER_BEAT);
    localparam int BEATS_PER_WORD  = WB_DATA_W / DATA_W;   // 16
    localparam int BIW_W           = $clog2(BEATS_PER_WORD);
    localparam int MAX_BEATS       = 256;

    // =========================================================================
    // 写通道 / 读 response 通道 tie 0（WDMA 只读）
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
    logic [ADDR_W-1:0]       cur_addr;
    logic [LEN_W-1:0]        beats_remaining;
    logic [SRAM_ADDR_W-1:0]  wb_wr_ptr;
    logic [BIW_W-1:0]        beat_in_word;       // 0..15
    logic [WB_DATA_W-1:0]    word_buffer;        // 16 beat 移位缓冲
    logic                    r_done;

    // =========================================================================
    // 派生量 / 事件
    // =========================================================================
    logic [8:0] beats_to_issue;
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

    // 组合：凑齐的完整 WB 字（当前 M_RDATA 顶进 MSB + 旧 buffer 下移）
    logic [WB_DATA_W-1:0] wdata_assembled;
    assign wdata_assembled = {M_RDATA, word_buffer[WB_DATA_W-1:DATA_W]};

    // 当前拍是完成一个 WB 字的最后一拍（beat_in_word==15 且 r_fire）
    logic complete_word;
    assign complete_word = r_fire && (beat_in_word == BIW_W'(BEATS_PER_WORD - 1));

    // =========================================================================
    // 端口输出
    // =========================================================================
    assign M_ARADDR  = cur_addr;
    assign M_ARLEN   = beats_to_issue[7:0] - 8'd1;
    assign M_ARVALID = (state == S_AR);
    assign M_RREADY  = (state == S_R);

    assign wb_we    = complete_word;
    assign wb_waddr = wb_wr_ptr;
    assign wb_wdata = wdata_assembled;

    // F-2 多 case：start 同拍 done 立即掉 0，避免 Sequencer 看到上一 case 残留 done
    assign done = r_done && !start;
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
    // 数据路径（start 初始化，无复位）
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

    // word_buffer：每 r_fire 右移 128-bit，新 beat 进 MSB
    always_ff @(posedge clk) begin
        if (r_fire) word_buffer <= wdata_assembled;
        else        word_buffer <= word_buffer;
    end

    // beat_in_word：0..15 循环
    always_ff @(posedge clk) begin
        if      (start)         beat_in_word <= '0;
        else if (complete_word) beat_in_word <= '0;
        else if (r_fire)        beat_in_word <= beat_in_word + 1;
        else                    beat_in_word <= beat_in_word;
    end

    // WB 写指针：每完成一个字递增
    always_ff @(posedge clk) begin
        if      (start)         wb_wr_ptr <= '0;
        else if (complete_word) wb_wr_ptr <= wb_wr_ptr + 1;
        else                    wb_wr_ptr <= wb_wr_ptr;
    end

    // =========================================================================
    // done 锁存（控制路径，复位必须）
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (!rst_n)                                         r_done <= 1'b0;
        else if (start)                                          r_done <= 1'b0;
        else if (r_last_fire && beats_remaining == LEN_W'(1))    r_done <= 1'b1;
        else                                                     r_done <= r_done;
    end

endmodule
