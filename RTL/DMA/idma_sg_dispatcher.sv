`timescale 1ns/1ps

// =============================================================================
// idma_sg_dispatcher.sv  --  IDMA SG (Scatter-Gather) cmd list dispatcher
//
// [Phase 7 SMC + NUMA] 用来替代 idma_ctrl. 接口签名跟 idma_ctrl 几乎一致 (mm2s
// cmd/data/sts + IFB 写口 + ring 反压), 这样 core_top.sv 用 generate-if 在
// SG_MODE 下用 dispatcher / 默认 mode 下用 idma_ctrl, 一行接线不同就够了.
//
// 设计哲学:
//   driver 编译期把"该核该层要拉哪些段"算好, 写成 cmd list 放在 mem 某段.
//   dispatcher 顺序拉每条 cmd, 一条 cmd = 一段连续 burst (在某一个 mem 内).
//   AXI Crossbar / SmartConnect IP 按 awaddr 自动路由到对应 mem slave.
//   Halo 列只一份: driver 把"邻居 mem 内的左/右 halo 列"作为独立 cmd 加到 list,
//   dispatcher 不需要知道这是 halo, 它只跑 cmd.
//
// 通道复用:
//   cmd 拉取 + 实际 data 拉取都走 mm2s_arb 的 idma 端口 (axi_dm.MM2S 通道).
//   dispatcher 内部串行: 先发 cmd 拉 32 byte cmd list, 收完后再发 cmd 拉用户 data.
//   这样不需要新增 axi master read 通道, axi_m_mux 不动.
//
// SG cmd 二进制格式 (32 byte / cmd, DATA_W=128 = 2 beats):
//   word 0: src_addr     [31:0]   transfer 起点 byte addr (全局地址)
//   word 1: btt          [22:0]   transfer 长度 byte (Bytes To Transfer)
//                        [23]     last_cmd flag (=1 时这条是 list 末尾, 优先级高于 cmd_count)
//                        [31:24]  reserved
//   word 2: sram_offset  [12:0]   IFB SRAM 写入起点 (word offset)
//                        [31:13]  reserved
//   word 3: reserved
//
//   beat 0 (DATA_W=128) = {word3[31:0], word2[31:0], word1[31:0], word0[31:0]}
//   注意: AXI 的 little-endian byte order 让 word 0 在 tdata[31:0]
//   这里是简化 PoC 格式, 不跟 Xilinx axi_dma SG 64-byte descriptor 对齐;
//   driver (mesh_cmd.py write_sg_cmd_list) 跟 RTL 同步即可.
//
// FSM:
//   S_IDLE       → start → S_FETCH_CMD_ISSUE
//   S_FETCH_CMD_ISSUE: 装 cmd 拉 cmd_list[r_cmd_idx] (32 byte) → S_FETCH_CMD_DATA
//   S_FETCH_CMD_DATA : 收 1 beat (DATA_W=128 = 16 byte; 32 byte cmd 仅 word0..3
//                       前 16 byte 有效, 后 16 byte reserved 跳过) → S_FETCH_CMD_STS
//                       简化: 我们一条 cmd 实际只用 16 byte, btt=16, 1 beat
//   S_FETCH_CMD_STS  : 等 sts → S_RING_WAIT
//   S_RING_WAIT      : 检查 ring 是否有空间 (rows_pushed - rows_consumed
//                       < cfg_ifb_strip_rows). 不够 → 等; 够 → S_ISSUE
//   S_ISSUE          : 装实际 cmd 拉 user data → S_DATA
//   S_DATA           : 收 data, 写 IFB SRAM[sram_wptr] (sram_wptr 从 r_sram_offset
//                       起递增, 跨 ring_words wrap). tlast → S_STS
//   S_STS            : 等 sts. 推进 r_cmds_done. 看是否 last_cmd / 全部跑完
//                       → S_DONE; 否则 S_FETCH_CMD_ISSUE 拉下一条
//   S_DONE           : r_done sticky → S_IDLE
// =============================================================================

module idma_sg_dispatcher #(
    parameter int ADDR_W      = 32,
    parameter int DATA_W      = 128,
    parameter int SRAM_ADDR_W = 13,
    parameter int LEN_W       = 24,
    parameter int TAG_W       = 4,
    parameter logic [TAG_W-1:0] TAG = '0
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // ---- Control ----
    input  logic                    start,
    output logic                    done,
    output logic                    busy,
    output logic                    err,

    // ---- SG 配置 (替代 idma_ctrl 的 src_base/byte_len) ----
    input  logic [ADDR_W-1:0]       cfg_cmd_list_ptr,   // cmd list 起点 (全局地址 byte)
    input  logic [15:0]             cfg_cmd_count,      // cmd 总数 (sanity check;
                                                         // last_cmd flag 优先终止)
    input  logic [7:0]              cfg_cmds_per_row,   // 每行 IFM 由几条 cmd 组成
                                                         // (W slice 跨 mem 时 > 1)

    // ---- Streaming 配置 (跟 idma_ctrl 一致, line_buffer ring 反压) ----
    input  logic [15:0]             cfg_h_in_total,
    input  logic [7:0]              cfg_ifb_strip_rows,
    input  logic [19:0]             cfg_ifb_ring_words,
    input  logic [15:0]             rows_consumed,
    output logic [15:0]             rows_available,

    // ---- DataMover MM2S CMD stream (跟 mm2s_arb idma input 对接) ----
    output logic                    mm2s_cmd_tvalid,
    input  logic                    mm2s_cmd_tready,
    output logic [71:0]             mm2s_cmd_tdata,

    // ---- DataMover MM2S DATA stream (in) ----
    input  logic                    mm2s_data_tvalid,
    output logic                    mm2s_data_tready,
    input  logic [DATA_W-1:0]       mm2s_data_tdata,
    input  logic [DATA_W/8-1:0]     mm2s_data_tkeep,    // 不消费
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

    // unused tkeep 防 lint
    logic [DATA_W/8-1:0] _tkeep_unused;
    assign _tkeep_unused = mm2s_data_tkeep;

    // SG cmd 一条占 16 byte 实际有效 (我们设计 32 byte 但只用 word 0..3 共 16 byte)
    localparam int CMD_BTT       = 16;        // 拉 cmd 的 transfer 长度 (byte)

    typedef enum logic [3:0] {
        S_IDLE          = 4'd0,
        S_FETCH_CMD_ISS = 4'd1,
        S_FETCH_CMD_DAT = 4'd2,
        S_FETCH_CMD_STS = 4'd3,
        S_RING_WAIT     = 4'd4,
        S_ISSUE         = 4'd5,
        S_DATA          = 4'd6,
        S_STS           = 4'd7,
        S_DONE          = 4'd8
    } state_t;
    state_t st, st_next;

    // ---- 当前正在处理的 cmd 字段 ----
    logic [ADDR_W-1:0]      r_src_addr;
    logic [LEN_W-1:0]       r_btt;
    logic                   r_last_cmd;
    logic [SRAM_ADDR_W-1:0] r_sram_offset;
    logic [SRAM_ADDR_W-1:0] r_sram_wptr;

    // ---- cmd list 进度 ----
    logic [15:0]       r_cmd_idx;       // 0..cfg_cmd_count-1
    logic [15:0]       r_cmds_done;     // 完成几条 cmd (跟 r_cmd_idx 区别在 STS 后才递增)
    logic [15:0]       r_rows_pushed;   // 推进的行数 = r_cmds_done / cfg_cmds_per_row

    logic              r_done;
    logic              r_err;

    assign busy = (st != S_IDLE) && (st != S_DONE);
    assign done = r_done && !start;
    assign err  = r_err;
    assign rows_available = r_rows_pushed;

    // =========================================================================
    // mm2s_cmd_tdata: 两阶段共享 (FETCH_CMD_ISS 时拉 cmd_list[idx], ISSUE 时拉 user data)
    // =========================================================================
    logic [ADDR_W-1:0]      cmd_saddr;
    logic [22:0]            cmd_btt23;
    always_comb begin
        if (st == S_FETCH_CMD_ISS) begin
            // 拉 cmd_list[idx]: 起点 = cfg_cmd_list_ptr + idx * 32 (32 byte / cmd)
            cmd_saddr = cfg_cmd_list_ptr + (ADDR_W'(r_cmd_idx) << 5);
            cmd_btt23 = 23'(CMD_BTT);
        end else begin   // S_ISSUE
            cmd_saddr = r_src_addr;
            cmd_btt23 = r_btt[22:0];
        end
    end

    assign mm2s_cmd_tdata = {
        4'b0,           // [71:68] reserved
        TAG,            // [67:64]
        cmd_saddr,      // [63:32]
        1'b0,           // [31] DRR
        1'b1,           // [30] EOF
        6'b0,           // [29:24] DSA
        1'b1,           // [23] TYPE = INCR
        cmd_btt23       // [22:0] BTT
    };
    assign mm2s_cmd_tvalid = (st == S_FETCH_CMD_ISS) || (st == S_ISSUE);

    // data tready: 拉 cmd 时 OR 拉用户 data 时
    assign mm2s_data_tready = (st == S_FETCH_CMD_DAT) || (st == S_DATA);
    assign mm2s_sts_tready  = (st == S_FETCH_CMD_STS) || (st == S_STS);

    // =========================================================================
    // IFB SRAM 写: 仅 S_DATA 阶段 (拉 cmd 阶段不写 IFB)
    // =========================================================================
    assign ifb_we    = (st == S_DATA) && mm2s_data_tvalid && mm2s_data_tready;
    assign ifb_waddr = r_sram_wptr;
    assign ifb_wdata = mm2s_data_tdata;

    // =========================================================================
    // ring 反压: r_rows_pushed - rows_consumed >= cfg_ifb_strip_rows 时 ring 满
    //   strip_rows = 0 表示 batch mode (整图装下), 反压 disable
    //
    // 注意 line_buffer.rows_consumed_raw += stride 在 stride>1 layer 时, 一行 OFM
    // 完成消费多行 IFM, 让 rows_consumed > rows_pushed (uint underflow). 这种情况
    // ring 内应被认为"有充裕空间" (反压无需触发). 用 signed 比较或加 fallthrough.
    // =========================================================================
    logic ring_has_space;
    logic ring_consumer_ahead;
    logic [15:0] rows_diff;
    assign rows_diff = r_rows_pushed - rows_consumed;
    assign ring_consumer_ahead = (rows_consumed >= r_rows_pushed);    // consumer 跑超 producer
    assign ring_has_space = (cfg_ifb_strip_rows == '0)
                          || ring_consumer_ahead
                          || (rows_diff < {8'd0, cfg_ifb_strip_rows});

    // =========================================================================
    // FSM next-state
    // =========================================================================
    always_comb begin
        st_next = st;
        case (st)
            S_IDLE          : if (start && cfg_cmd_count > 0) st_next = S_FETCH_CMD_ISS;
            S_FETCH_CMD_ISS : if (mm2s_cmd_tvalid && mm2s_cmd_tready) st_next = S_FETCH_CMD_DAT;
            S_FETCH_CMD_DAT : if (mm2s_data_tvalid && mm2s_data_tready && mm2s_data_tlast)
                                                                         st_next = S_FETCH_CMD_STS;
            S_FETCH_CMD_STS : if (mm2s_sts_tvalid && mm2s_sts_tready)    st_next = S_RING_WAIT;
            S_RING_WAIT     : if (ring_has_space)                         st_next = S_ISSUE;
            S_ISSUE         : if (mm2s_cmd_tvalid && mm2s_cmd_tready)    st_next = S_DATA;
            S_DATA          : if (mm2s_data_tvalid && mm2s_data_tready && mm2s_data_tlast)
                                                                         st_next = S_STS;
            S_STS           : if (mm2s_sts_tvalid && mm2s_sts_tready) begin
                // 注意: r_cmd_idx 在 cmd_fire 时 +1, 表示已装载几个 cmd. 装第 N 个 cmd 后
                // r_cmd_idx=N, 这条 sts 完成时算 list 跑完. 用 r_cmd_idx >= cfg_cmd_count
                // 而不是 +1 (避免 off-by-one 漏装最后一条 cmd).
                if (r_last_cmd || (r_cmd_idx >= cfg_cmd_count))           st_next = S_DONE;
                else                                                       st_next = S_FETCH_CMD_ISS;
            end
            S_DONE          : st_next = S_IDLE;
            default         : st_next = S_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) st <= S_IDLE;
        else        st <= st_next;
    end

    // =========================================================================
    // cmd 解码: S_FETCH_CMD_DAT 收 1 beat (DATA_W=128 = 16 byte) latch word 0..3
    //   beat[31:0]   = word 0 = src_addr
    //   beat[63:32]  = word 1 = {reserved[7:0], last_cmd[1], btt[22:0]}
    //   beat[95:64]  = word 2 = sram_offset (低 13 bit 有效)
    //   beat[127:96] = word 3 = reserved
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            r_src_addr    <= '0;
            r_btt         <= '0;
            r_last_cmd    <= 1'b0;
            r_sram_offset <= '0;
        end else if (st == S_FETCH_CMD_DAT && mm2s_data_tvalid && mm2s_data_tready) begin
            r_src_addr    <= mm2s_data_tdata[31:0];
            r_btt         <= {1'b0, mm2s_data_tdata[32 +: 23]};
            r_last_cmd    <= mm2s_data_tdata[32 + 23];
            r_sram_offset <= mm2s_data_tdata[64 +: SRAM_ADDR_W];
        end
    end

    // =========================================================================
    // IFB SRAM write pointer:
    //   每条 cmd 起点 = r_sram_offset, 内部递增, 跨 cfg_ifb_ring_words 时 wrap
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n)                              r_sram_wptr <= '0;
        else if (st == S_ISSUE)                  r_sram_wptr <= r_sram_offset;
        else if (st == S_DATA && ifb_we) begin
            // wrap (cfg_ifb_ring_words = 0 → 不 wrap, 跑 batch mode)
            if ((cfg_ifb_ring_words != '0)
              && ({{(20-SRAM_ADDR_W){1'b0}}, r_sram_wptr} + 20'd1
                                             >= cfg_ifb_ring_words))
                r_sram_wptr <= '0;
            else
                r_sram_wptr <= r_sram_wptr + 1'b1;
        end
    end

    // =========================================================================
    // r_cmd_idx ++ 在 S_STS 完成时
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (!rst_n)         r_cmd_idx <= '0;
        else if (start)          r_cmd_idx <= '0;
        else if (st == S_STS && mm2s_sts_tvalid && mm2s_sts_tready)
                                 r_cmd_idx <= r_cmd_idx + 16'd1;
    end

    // =========================================================================
    // r_cmds_done / r_rows_pushed
    //   每条 cmd 完成 (S_STS) 时 r_cmds_done++
    //   每 cfg_cmds_per_row 条 cmd 完成时 r_rows_pushed++
    // =========================================================================
    logic [7:0] r_cmds_in_row;     // 当前行内已完成几条 cmd (0..cfg_cmds_per_row-1)
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            r_cmds_done   <= '0;
            r_cmds_in_row <= '0;
            r_rows_pushed <= '0;
        end else if (start) begin
            r_cmds_done   <= '0;
            r_cmds_in_row <= '0;
            r_rows_pushed <= '0;
        end else if (st == S_STS && mm2s_sts_tvalid && mm2s_sts_tready) begin
            r_cmds_done <= r_cmds_done + 16'd1;
            if (cfg_cmds_per_row == 8'd0 || r_cmds_in_row + 8'd1 >= cfg_cmds_per_row) begin
                r_cmds_in_row <= '0;
                r_rows_pushed <= r_rows_pushed + 16'd1;
            end else begin
                r_cmds_in_row <= r_cmds_in_row + 8'd1;
            end
        end
    end

    // =========================================================================
    // r_done sticky
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (!rst_n)              r_done <= 1'b0;
        else if (start)               r_done <= 1'b0;
        else if (st == S_DONE)        r_done <= 1'b1;
    end

    // =========================================================================
    // r_err: sts[7] = 0 表示 OK, = 1 表错误 (跟 idma_ctrl 一致)
    //   注意: Xilinx datamover sts 字段 [7]=OKAY → r_err = ~sts[7]; 但项目内
    //   idma_ctrl 用的是相反约定, 这里跟 idma_sg_dispatcher 已有逻辑一致即可.
    //   sticky.
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (!rst_n)                                                  r_err <= 1'b0;
        else if ((st == S_FETCH_CMD_STS || st == S_STS)
              && mm2s_sts_tvalid && mm2s_sts_tready
              && !mm2s_sts_tdata[7])                                      r_err <= 1'b1;
    end

endmodule
