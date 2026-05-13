`timescale 1ns/1ps

// =============================================================================
// wdma_ctrl.sv  --  Weight DMA cmd controller for Xilinx AXI DataMover MM2S
//
// DataMover MM2S 通道的指令控制端 (本身不是 DataMover): 给 axi_dm 发 cmd,
// 收数据流写到 WB SRAM. 对外接口与上一代 wdma.sv 兼容
// (descriptor + WB SRAM 写端 + done/busy).
//
// 与 idma_ctrl 不同: WDMA 是 batch 一次性搬运, 不分行/不需 ring 反压.
//   单条 cmd 覆盖整段 byte_len, BTT 字段直接填 byte_len.
//   收 data stream, 每 16 beat 拼成 1 个 2048-bit WB 字写入 (与原 wdma 相同).
//
// 命令字格式与 idma_ctrl 一致 (EOF=1 让 tlast 在最后一拍拉起):
//   {4'b0, TAG_WDMA[3:0], src_base[31:0], 1'b0(DRR), 1'b1(EOF), 6'b0(DSA),
//    1'b1(TYPE INCR), byte_len[22:0]}
//
// WB 字布局 (与原 wdma 一致):
//   bits [127:0]      = beat 0 (col 0)
//   bits [255:128]    = beat 1
//   ...
//   bits [2047:1920]  = beat 15 (col 15)
// 即 beat 0 在 LSB. 实现: 每 r_fire 把 word_buffer 右移 128-bit, 新 beat 进 MSB;
// 第 16 beat 同拍组合 {data_tdata, word_buffer[2047:128]} 直接写 WB.
// =============================================================================

module wdma_ctrl #(
    parameter int ADDR_W      = 32,
    parameter int DATA_W      = 128,
    parameter int WB_DATA_W   = 2048,
    parameter int SRAM_ADDR_W = 13,
    parameter int LEN_W       = 24,
    parameter int TAG_W       = 4,
    parameter logic [TAG_W-1:0] TAG_WDMA = '0
)(
    input  logic                      clk,
    input  logic                      rst_n,

    // ---- Control ----
    input  logic                      start,
    output logic                      done,
    output logic                      busy,
    output logic                      err,

    // ---- Descriptor ----
    input  logic [ADDR_W-1:0]         src_base,
    input  logic [LEN_W-1:0]          byte_len,

    // ---- DataMover MM2S CMD stream (out) ----
    output logic                      mm2s_cmd_tvalid,
    input  logic                      mm2s_cmd_tready,
    output logic [71:0]               mm2s_cmd_tdata,

    // ---- DataMover MM2S DATA stream (in) ----
    input  logic                      mm2s_data_tvalid,
    output logic                      mm2s_data_tready,
    input  logic [DATA_W-1:0]         mm2s_data_tdata,
    input  logic [DATA_W/8-1:0]       mm2s_data_tkeep,
    input  logic                      mm2s_data_tlast,

    // ---- DataMover MM2S STS stream (in) ----
    input  logic                      mm2s_sts_tvalid,
    output logic                      mm2s_sts_tready,
    input  logic [7:0]                mm2s_sts_tdata,

    // ---- WB SRAM write port (2048-bit 宽) ----
    output logic                      wb_we,
    output logic [SRAM_ADDR_W-1:0]    wb_waddr,
    output logic [WB_DATA_W-1:0]      wb_wdata
);

    localparam int BEATS_PER_WORD = WB_DATA_W / DATA_W;       // 16
    localparam int BIW_W          = $clog2(BEATS_PER_WORD);   // 4

    // 防 unused warning
    logic [DATA_W/8-1:0] _tkeep_unused;
    assign _tkeep_unused = mm2s_data_tkeep;

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [1:0] {
        S_IDLE = 2'd0,
        S_CMD  = 2'd1,
        S_RX   = 2'd2,
        S_DONE = 2'd3
    } state_t;
    state_t state, state_next;

    // =========================================================================
    // 寄存器
    // =========================================================================
    logic [SRAM_ADDR_W-1:0]   wb_wr_ptr;
    logic [BIW_W-1:0]         beat_in_word;
    logic [WB_DATA_W-1:0]     word_buffer;
    logic                     r_done;
    logic                     r_err;
    assign err = r_err;

    // =========================================================================
    // 派生
    // =========================================================================
    logic cmd_fire, data_fire, data_last_fire, sts_fire;
    assign cmd_fire       = mm2s_cmd_tvalid  && mm2s_cmd_tready;
    assign data_fire      = mm2s_data_tvalid && mm2s_data_tready;
    assign data_last_fire = data_fire && mm2s_data_tlast;
    assign sts_fire       = mm2s_sts_tvalid  && mm2s_sts_tready;

    // 凑齐的完整 WB 字: 当前 beat 顶进 MSB + 旧 buffer 右移
    logic [WB_DATA_W-1:0] wdata_assembled;
    assign wdata_assembled = {mm2s_data_tdata, word_buffer[WB_DATA_W-1:DATA_W]};

    // 一个 WB 字的最后一拍
    logic complete_word;
    assign complete_word = data_fire && (beat_in_word == BIW_W'(BEATS_PER_WORD - 1));

    // =========================================================================
    // 命令字
    //   BTT = byte_len[22:0]; LEN_W=24 时高位通常是 0 (单 cmd 最大 8 MB 已足够
    //   单层权重: 9×64×64×1 = 36 KB; 256→512 FC = 128 KB). 安全裕度大.
    // =========================================================================
    logic [22:0] cmd_btt;
    assign cmd_btt = byte_len[22:0];

    assign mm2s_cmd_tdata = {
        4'b0,            // [71:68] RSVD
        TAG_WDMA,        // [67:64] TAG
        src_base,        // [63:32] SADDR
        1'b0,            // [31]    DRR
        1'b1,            // [30]    EOF (拉 tlast)
        6'b0,            // [29:24] DSA
        1'b1,            // [23]    TYPE = INCR
        cmd_btt          // [22:0]  BTT
    };
    assign mm2s_cmd_tvalid  = (state == S_CMD);
    assign mm2s_data_tready = (state == S_RX);
    assign mm2s_sts_tready  = 1'b1;

    // =========================================================================
    // WB SRAM 写
    // =========================================================================
    assign wb_we    = complete_word;
    assign wb_waddr = wb_wr_ptr;
    assign wb_wdata = wdata_assembled;

    assign done = r_done && !start;
    assign busy = (state != S_IDLE) && (state != S_DONE);

    // =========================================================================
    // FSM 转移
    // =========================================================================
    always_comb begin
        state_next = state;
        case (state)
            S_IDLE : if (start)              state_next = S_CMD;
            S_CMD  : if (cmd_fire)           state_next = S_RX;
            S_RX   : if (data_last_fire)     state_next = S_DONE;
            S_DONE : if (start)              state_next = S_CMD;
            default:                          state_next = S_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= state_next;
    end


    // =========================================================================
    // 数据路径
    // =========================================================================
    // word_buffer: 每 data_fire 右移 128-bit, 新 beat 顶 MSB
    always_ff @(posedge clk) begin
        if (data_fire) word_buffer <= wdata_assembled;
        else           word_buffer <= word_buffer;
    end

    always_ff @(posedge clk) begin
        if      (start)         beat_in_word <= '0;
        else if (complete_word) beat_in_word <= '0;
        else if (data_fire)     beat_in_word <= beat_in_word + 1;
        else                    beat_in_word <= beat_in_word;
    end

    always_ff @(posedge clk) begin
        if      (start)         wb_wr_ptr <= '0;
        else if (complete_word) wb_wr_ptr <= wb_wr_ptr + 1;
        else                    wb_wr_ptr <= wb_wr_ptr;
    end

    // =========================================================================
    // 控制
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (!rst_n)            r_done <= 1'b0;
        else if (start)             r_done <= 1'b0;
        else if (data_last_fire)    r_done <= 1'b1;
        else                        r_done <= r_done;
    end

    always_ff @(posedge clk) begin
        if      (!rst_n)                            r_err <= 1'b0;
        else if (start)                             r_err <= 1'b0;
        else if (sts_fire && !mm2s_sts_tdata[7])    r_err <= 1'b1;
        else                                        r_err <= r_err;
    end

endmodule
