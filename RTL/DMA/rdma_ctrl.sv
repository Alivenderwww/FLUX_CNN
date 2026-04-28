`timescale 1ns/1ps

// =============================================================================
// rdma_ctrl.sv  --  Residual / Bias DMA cmd controller for AXI DataMover MM2S
//
// DataMover MM2S 通道的第 3 个用户 (跟 idma_ctrl + wdma_ctrl 经 mm2s_arb 共享).
// 单条 cmd batch 模式: 一次性把 [bias 段 + shortcut 段] 整体从 DDR 拉进
// Shortcut Bank SRAM (NUM_PE × DATA_WIDTH = 128 bit 宽, 跟 OFB 同字宽).
//
// 跟 wdma_ctrl 的区别:
//   - 输出 SRAM 字宽 = 128 bit (1 拍 1 字), wdma 是 2048 bit (16 拍打包成 1 字)
//   - 因此 rdma 不需要 word_buffer, 也不需要 beat_in_word counter
//   - 数据流接收一拍写一拍, 简单直接
//
// 命令字格式 (与 idma/wdma 一致, EOF=1 让 tlast 在最后一拍拉起):
//   {4'b0, TAG_RDMA[3:0], src_base[31:0], DRR=0, EOF=1, DSA=0, TYPE=INCR, byte_len[22:0]}
// =============================================================================

module rdma_ctrl #(
    parameter int ADDR_W      = 32,
    parameter int DATA_W      = 128,
    parameter int SRAM_ADDR_W = 13,
    parameter int LEN_W       = 24,
    parameter int TAG_W       = 4,
    parameter logic [TAG_W-1:0] TAG_RDMA = 4'd2
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

    // ---- Shortcut Bank SRAM 写端口 (128 bit 宽) ----
    output logic                      sb_we,
    output logic [SRAM_ADDR_W-1:0]    sb_waddr,
    output logic [DATA_W-1:0]         sb_wdata
);

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
    logic [SRAM_ADDR_W-1:0]   wr_ptr;
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

    // BTT (字节)
    logic [22:0] cmd_btt;
    assign cmd_btt = byte_len[22:0];

    // =========================================================================
    // 命令字
    // =========================================================================
    assign mm2s_cmd_tdata = {
        4'b0,            // [71:68] RSVD
        TAG_RDMA,        // [67:64] TAG
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
    // SRAM 写
    // =========================================================================
    assign sb_we    = data_fire;
    assign sb_waddr = wr_ptr;
    assign sb_wdata = mm2s_data_tdata;

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
    // wr_ptr: 每拍 data_fire +1, start 归零
    always_ff @(posedge clk) begin
        if      (start)     wr_ptr <= '0;
        else if (data_fire) wr_ptr <= wr_ptr + 1'b1;
        else                wr_ptr <= wr_ptr;
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
