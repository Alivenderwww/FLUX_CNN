`timescale 1ns/1ps

// =============================================================================
// idma_ctrl.sv  --  Input DMA cmd controller for Xilinx AXI DataMover MM2S
//
// 这个模块本身**不是** DataMover, 是 DataMover MM2S 通道的指令控制端:
// 给 axi_dm 发 (addr, btt, ...) 命令, 收数据流写到 IFB SRAM. AXI4 master
// 由 axi_dm 实例驱动, 不在这里. cfg / rows_consumed / IFB 写端口接口与上一代
// idma.sv 兼容.
//
//     idma_ctrl.cmd ─→ axi_dm.s_axis_mm2s_cmd       (72-bit cmd word)
//     idma_ctrl.data ←─ axi_dm.m_axis_mm2s          (data stream)
//     idma_ctrl.sts ←─ axi_dm.m_axis_mm2s_sts       (status, 8-bit)
//
// AXI4 总线 (m_axi_mm2s_*) 由外部 axi_dm 实例直接驱动到 DDR.
//
// 设计与原 idma 一致: 1 cmd / 1 输入行, BTT = ky_step × 16 字节. 每行收完
// (data_tlast=1) 后判 ring 是否满, 满则 S_WAIT 等 line_buffer 消费.
//
// 命令字 (72-bit, addr_width=32, btt_used=23, tag_width=4)
// 字段位置参考 Xilinx 官方 example axis_cmd_gen_mm2s.vhd:
//   cmd <= "0000" & "0000" & address & '0' & '1' & "000000" & '1' & btt
//   [71:68] RSVD = 0
//   [67:64] TAG = TAG_IDMA
//   [63:32] SADDR (DDR 起始地址)
//   [31]    DRR = 0  (don't drain partial bursts)
//   [30]    EOF = 1  *** 关键 *** EOF=1 才会让 DataMover 在 BTT 字节传完后
//                                  在 m_axis_mm2s_tlast 拉 1 拍, 标记 stream
//                                  最后一 beat. EOF=0 → tlast 永不 assert.
//   [29:24] DSA = 0
//   [23]    TYPE = 1 (INCR)
//   [22:0]  BTT (字节数)
//
// 复位: 控制路径 (FSM + r_done + err_latch) 同步复位; 数据路径
// (cur_addr / rows_written / wr_ptr) 由 start 初始化.
// =============================================================================

module idma_ctrl #(
    parameter int ADDR_W      = 32,
    parameter int DATA_W      = 128,
    parameter int SRAM_ADDR_W = 13,
    parameter int LEN_W       = 24,
    parameter int TAG_W       = 4,
    parameter logic [TAG_W-1:0] TAG_IDMA = '0
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // ---- Control ----
    input  logic                    start,
    output logic                    done,
    output logic                    busy,
    output logic                    err,            // sticky, status 非 OK 即拉起

    // ---- Descriptor ----
    input  logic [ADDR_W-1:0]       src_base,
    input  logic [LEN_W-1:0]        byte_len,       // 保留字段 (streaming 不用)

    // ---- Streaming 配置 (与原 idma 二进制兼容) ----
    input  logic [15:0]             cfg_h_in_total,
    input  logic [7:0]              cfg_ifb_strip_rows,
    input  logic [19:0]             cfg_ifb_ky_step,    // 每行 beats 数 (NHWC: W_IN × cin_slices)
    input  logic [19:0]             cfg_ifb_ring_words, // ring wrap 模数
    input  logic [15:0]             rows_consumed,
    output logic [15:0]             rows_available,

    // ---- DataMover MM2S CMD stream (out) ----
    output logic                    mm2s_cmd_tvalid,
    input  logic                    mm2s_cmd_tready,
    output logic [71:0]             mm2s_cmd_tdata,

    // ---- DataMover MM2S DATA stream (in) ----
    input  logic                    mm2s_data_tvalid,
    output logic                    mm2s_data_tready,
    input  logic [DATA_W-1:0]       mm2s_data_tdata,
    input  logic [DATA_W/8-1:0]     mm2s_data_tkeep,    // 我们整 beat, 全 1; 不参与判定
    input  logic                    mm2s_data_tlast,

    // ---- DataMover MM2S STS stream (in, 8-bit per cmd) ----
    input  logic                    mm2s_sts_tvalid,
    output logic                    mm2s_sts_tready,
    input  logic [7:0]              mm2s_sts_tdata,

    // ---- IFB SRAM write port ----
    output logic                    ifb_we,
    output logic [SRAM_ADDR_W-1:0]  ifb_waddr,
    output logic [DATA_W-1:0]       ifb_wdata
);

    localparam int BYTES_PER_BEAT = DATA_W / 8;        // 16
    localparam int BEAT_SHIFT     = $clog2(BYTES_PER_BEAT);  // 4

    // 防 unused warning (descriptor 兼容字段) + tkeep 不消费
    logic [LEN_W-1:0] _byte_len_unused;
    assign _byte_len_unused = byte_len;
    logic [DATA_W/8-1:0] _tkeep_unused;
    assign _tkeep_unused = mm2s_data_tkeep;

    // =========================================================================
    // FSM
    //   原 idma 的 S_AR/S_R 在 DataMover 下合并为 S_CMD/S_RX. ring_full 在
    //   S_CMD 入口判定, 满则 S_WAIT.
    // =========================================================================
    typedef enum logic [2:0] {
        S_IDLE = 3'd0,
        S_CMD  = 3'd1,
        S_RX   = 3'd2,
        S_WAIT = 3'd3,
        S_DONE = 3'd4
    } state_t;
    state_t state, state_next;

    // =========================================================================
    // 寄存器
    // =========================================================================
    logic [ADDR_W-1:0]       cur_addr;        // 下一条 cmd 的 DDR 起始地址
    logic [15:0]             rows_written;    // 已写完的行数
    assign rows_available = rows_written;
    logic [SRAM_ADDR_W-1:0]  wr_ptr;          // IFB ring 写指针
    logic                    r_done;
    logic                    r_err;
    assign err = r_err;

    // =========================================================================
    // 派生
    // =========================================================================
    logic cmd_fire, data_fire, data_last_fire, sts_fire;
    assign cmd_fire       = mm2s_cmd_tvalid  && mm2s_cmd_tready;
    assign data_fire      = mm2s_data_tvalid && mm2s_data_tready;
    assign data_last_fire = data_fire && mm2s_data_tlast;
    assign sts_fire       = mm2s_sts_tvalid  && mm2s_sts_tready;

    logic streaming_all_done;
    assign streaming_all_done = data_last_fire && (rows_written == cfg_h_in_total - 16'd1);

    logic ring_full;
    assign ring_full = ((rows_written - rows_consumed) >= {8'd0, cfg_ifb_strip_rows});

    // BTT (字节) = ky_step beats × 16 B/beat
    logic [22:0] cmd_btt;
    assign cmd_btt = {cfg_ifb_ky_step[18:0], 4'b0};

    // =========================================================================
    // 命令字组装
    // =========================================================================
    assign mm2s_cmd_tdata = {
        4'b0,            // [71:68] RSVD (no cache_user used in our config)
        TAG_IDMA,        // [67:64] TAG
        cur_addr,        // [63:32] SADDR
        1'b0,            // [31]    DRR = 0
        1'b1,            // [30]    EOF = 1  (拉 tlast on last beat, 关键!)
        6'b0,            // [29:24] DSA = 0
        1'b1,            // [23]    TYPE = INCR
        cmd_btt          // [22:0]  BTT
    };
    assign mm2s_cmd_tvalid  = (state == S_CMD) && !ring_full;
    assign mm2s_data_tready = (state == S_RX);
    assign mm2s_sts_tready  = 1'b1;          // 永远 ready, 状态字一来就吃掉

    // =========================================================================
    // IFB SRAM 写
    // =========================================================================
    assign ifb_we    = data_fire;
    assign ifb_waddr = wr_ptr;
    assign ifb_wdata = mm2s_data_tdata;

    // F-2 多 case: start 同拍 done 立即掉 0
    assign done = r_done && !start;
    assign busy = (state != S_IDLE) && (state != S_DONE);

    // =========================================================================
    // FSM 转移
    // =========================================================================
    always_comb begin
        state_next = state;
        case (state)
            S_IDLE : if (start)              state_next = S_CMD;
            S_CMD  : if (ring_full)          state_next = S_WAIT;
                     else if (cmd_fire)      state_next = S_RX;
            S_RX   : if (data_last_fire) begin
                if (streaming_all_done)      state_next = S_DONE;
                else                         state_next = S_CMD;
            end
            S_WAIT : if (!ring_full)         state_next = S_CMD;
            S_DONE : if (start)              state_next = S_CMD;
            default:                         state_next = S_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= state_next;
    end

    // =========================================================================
    // 数据路径寄存器 (无复位, start 初始化)
    // =========================================================================
    // cur_addr: cmd_fire 后跳到下一行
    always_ff @(posedge clk) begin
        if      (start)    cur_addr <= src_base;
        else if (cmd_fire) cur_addr <= cur_addr + (ADDR_W'(cmd_btt));
        else               cur_addr <= cur_addr;
    end

    // rows_written: 每收完一整行 +1
    always_ff @(posedge clk) begin
        if      (start)              rows_written <= 16'd0;
        else if (data_last_fire)     rows_written <= rows_written + 16'd1;
        else                         rows_written <= rows_written;
    end

    // wr_ptr: 在 cfg_ifb_ring_words 处 wrap
    logic [SRAM_ADDR_W-1:0] wr_ptr_wrap_limit_m1;
    assign wr_ptr_wrap_limit_m1 = cfg_ifb_ring_words[SRAM_ADDR_W-1:0] - SRAM_ADDR_W'(1);

    always_ff @(posedge clk) begin
        if      (start)      wr_ptr <= '0;
        else if (data_fire) begin
            if (wr_ptr == wr_ptr_wrap_limit_m1) wr_ptr <= '0;
            else                                wr_ptr <= wr_ptr + 1;
        end
        else                 wr_ptr <= wr_ptr;
    end

    // =========================================================================
    // 控制寄存器 (有复位)
    // =========================================================================
    // r_done
    always_ff @(posedge clk) begin
        if      (!rst_n)              r_done <= 1'b0;
        else if (start)               r_done <= 1'b0;
        else if (streaming_all_done)  r_done <= 1'b1;
        else                          r_done <= r_done;
    end

    // err sticky: 状态字 [7]=OK 位 (1=OK). 任何 sts_fire 拿到 [7]=0 拉起 r_err.
    // PG022 v5.1 status word: [7]=OK [6]=SLVERR [5]=DECERR [4]=INTERR [3:0]=TAG
    always_ff @(posedge clk) begin
        if      (!rst_n)                            r_err <= 1'b0;
        else if (start)                             r_err <= 1'b0;
        else if (sts_fire && !mm2s_sts_tdata[7])    r_err <= 1'b1;
        else                                        r_err <= r_err;
    end

endmodule
