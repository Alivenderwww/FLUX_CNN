`timescale 1ns/1ps

// =============================================================================
// ifb_axi_slave.sv  --  AXI4 Slave 接收跨核 IFB push (M2)
//
// 用途: 多核 wrapper 中, 一个核的 ODMA 把 OFM 写出, 经 axi crossbar 路由到下游
//   核的 IFB SRAM 写口. 本模块就是 "IFB 写口 AXI4 适配".
//
// 行为模型 (跟 axi_slave_mem 同风格, 但带 ring 反压):
//   AW: fire 后锁存 awid, 进 WS_DATA. ring 满时 awready=0 反压 producer.
//   W : 每 fire 写 ifb_we/waddr/wdata 给外部 SRAM, wptr += 1 (mod ring_words).
//        WLAST 触发 rows_pushed += 1 + 进 WS_RESP.
//   B : 1 拍 BRESP=OK + BID=锁存值, 等 BREADY 后回 WS_IDLE.
//   AR/R: tie off (push only, 无读路径). crossbar 期望 5 通道完整.
//
// Ring 反压 (跟 idma_ctrl.sv 同思路):
//   ring_full = (rows_pushed - rows_consumed) >= cfg_ifb_strip_rows
//   line_buffer 消费数据 → rows_consumed↑ → ring 不满 → producer 可继续 push.
//
// 起点同步: layer 启动时 producer/consumer 都 reset, rows_pushed/rows_consumed
//   两侧用 evt_start (来自 sequencer start_layer_pulse) 同步清零, 保证差值有效.
//
// 假设:
//   - producer 每个 burst = 1 行 (cmd_btt = row_words × DATA_W/8), wlast 1 个/burst
//   - awaddr 仅作 crossbar 路由用, slave 内部用 wptr 自驱 (push only stream)
//     (producer 编译器算的 ODMA_DST_BASE 跟 slave 实际写位置可能差一个 wrap, 但
//      只要 producer 数据顺序跟 line_buffer 期望顺序一致就 OK)
// =============================================================================

module ifb_axi_slave #(
    parameter int ADDR_W   = 32,
    parameter int DATA_W   = 128,
    parameter int ID_W     = 5,
    parameter int SRAM_AW  = 13,        // log2(SRAM_DEPTH)=13
    parameter int IFB_W    = DATA_W     // = NUM_PE × DATA_WIDTH = 128
)(
    input  logic                clk,
    input  logic                rstn,

    // ---- AXI4 SI: AW / W / B (push only) ----
    input  logic [ID_W-1:0]     AWID,
    input  logic [ADDR_W-1:0]   AWADDR,
    input  logic [7:0]          AWLEN,
    input  logic [2:0]          AWSIZE,
    input  logic [1:0]          AWBURST,
    input  logic                AWLOCK,
    input  logic [3:0]          AWCACHE,
    input  logic [2:0]          AWPROT,
    input  logic [3:0]          AWQOS,
    input  logic                AWVALID,
    output logic                AWREADY,

    input  logic [DATA_W-1:0]   WDATA,
    input  logic [DATA_W/8-1:0] WSTRB,
    input  logic                WLAST,
    input  logic                WVALID,
    output logic                WREADY,

    output logic [ID_W-1:0]     BID,
    output logic [1:0]          BRESP,
    output logic                BVALID,
    input  logic                BREADY,

    // ---- AXI4 SI: AR / R (tie-off, crossbar 期望 5 通道) ----
    input  logic [ID_W-1:0]     ARID,
    input  logic [ADDR_W-1:0]   ARADDR,
    input  logic [7:0]          ARLEN,
    input  logic [2:0]          ARSIZE,
    input  logic [1:0]          ARBURST,
    input  logic                ARLOCK,
    input  logic [3:0]          ARCACHE,
    input  logic [2:0]          ARPROT,
    input  logic [3:0]          ARQOS,
    input  logic                ARVALID,
    output logic                ARREADY,

    output logic [ID_W-1:0]     RID,
    output logic [DATA_W-1:0]   RDATA,
    output logic [1:0]          RRESP,
    output logic                RLAST,
    output logic                RVALID,
    input  logic                RREADY,

    // ---- 反压 + 同步 ----
    input  logic                evt_start_layer,    // sequencer 启 layer 脉冲, 清 rows_pushed + wptr
    input  logic                cfg_skip_idma,       // 1=consumer 模式, 接收 push; 0=producer/普通核, 拒绝 push
    input  logic [15:0]         rows_consumed,       // 来自 line_buffer
    input  logic [7:0]          cfg_ifb_strip_rows,  // ring 容量 (行)
    input  logic [SRAM_AW-1:0]  cfg_ifb_ring_words,  // ring 容量 (word)

    // ---- 输出到 IFB SRAM 写口 ----
    output logic                ifb_we,
    output logic [SRAM_AW-1:0]  ifb_waddr,
    output logic [IFB_W-1:0]    ifb_wdata,

    // ---- 给 line_buffer 的 rows_available (替代 idma_ctrl.rows_written, M2 SKIP_IDMA 路径) ----
    output logic [15:0]         rows_pushed_out
);

    // 未用字段 sink (压制 lint warning)
    wire _unused_ok = &{1'b0, AWADDR, AWLEN, AWSIZE, AWBURST, AWLOCK,
                        AWCACHE, AWPROT, AWQOS, WSTRB, ARID, ARADDR, ARLEN,
                        ARSIZE, ARBURST, ARLOCK, ARCACHE, ARPROT, ARQOS, 1'b0};

    // -------------------------------------------------------------------------
    // AR/R tie-off: 永不接受读, 永不返回数据
    // -------------------------------------------------------------------------
    assign ARREADY = 1'b0;
    assign RID     = '0;
    assign RDATA   = '0;
    assign RRESP   = 2'b00;
    assign RLAST   = 1'b0;
    assign RVALID  = 1'b0;

    // -------------------------------------------------------------------------
    // FSM: WS_IDLE → WS_DATA (W beats) → WS_RESP (B handshake) → WS_IDLE
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] { WS_IDLE, WS_DATA, WS_RESP } wst_t;
    wst_t                  wst;
    logic [ID_W-1:0]       wid_latch;
    logic [SRAM_AW-1:0]    wptr;            // SRAM 写指针 (ring wrap)
    logic [15:0]           rows_pushed;     // 累计 push 行数 (跟 rows_consumed 比)
    assign rows_pushed_out = rows_pushed;

    // 反压: ring 满 → awready=0
    logic ring_full;
    assign ring_full = (rows_pushed - rows_consumed) >= {8'd0, cfg_ifb_strip_rows};

    // cfg_skip_idma=0 (producer/独立核): 永远拒绝 push, ifb_we 也保持 0, 不污染 IFB
    assign AWREADY = (wst == WS_IDLE) && !ring_full && cfg_skip_idma;
    assign WREADY  = (wst == WS_DATA);
    assign BVALID  = (wst == WS_RESP);
    assign BID     = wid_latch;
    assign BRESP   = 2'b00;

    // SRAM 写口直接来自 W 通道
    assign ifb_we    = (wst == WS_DATA) && WVALID && WREADY;
    assign ifb_waddr = wptr;
    assign ifb_wdata = WDATA;

    // wptr: 每 W fire +1, ring wrap; layer start 时清 0
    always_ff @(posedge clk) begin
        if (!rstn) begin
            wptr <= '0;
        end else if (evt_start_layer) begin
            wptr <= '0;
        end else if (ifb_we) begin
            if (wptr + 1 == cfg_ifb_ring_words[SRAM_AW-1:0])
                wptr <= '0;
            else
                wptr <= wptr + 1;
        end
    end

    // rows_pushed: WLAST fire 时 +1; layer start 时清 0
    always_ff @(posedge clk) begin
        if (!rstn) begin
            rows_pushed <= 16'd0;
        end else if (evt_start_layer) begin
            rows_pushed <= 16'd0;
        end else if (ifb_we && WLAST) begin
            rows_pushed <= rows_pushed + 16'd1;
        end
    end

    // 状态机
    always_ff @(posedge clk) begin
        if (!rstn) begin
            wst       <= WS_IDLE;
            wid_latch <= '0;
        end else begin
            case (wst)
                WS_IDLE: if (AWVALID && AWREADY) begin
                    wid_latch <= AWID;
                    wst       <= WS_DATA;
                end
                WS_DATA: if (WVALID && WREADY && WLAST) begin
                    wst <= WS_RESP;
                end
                WS_RESP: if (BVALID && BREADY) begin
                    wst <= WS_IDLE;
                end
                default: wst <= WS_IDLE;
            endcase
        end
    end

endmodule
