`timescale 1ns/1ps

// =============================================================================
// axi_slave_mem.sv  --  极简 AXI4 full slave，背后是 memory 数组
// 仿真用；无 out-of-order、无交织、单 outstanding 即可。
// 协议要点：
//   AW: fire 后锁存 {awid, awaddr, awlen}，进 write data phase
//   W : 每 fire 写 mem[wptr]，wptr 每拍 +1；LAST 触发 B phase
//   B : 发 1 拍 BRESP=OK + BID=锁存的 awid
//   AR: fire 后锁存 {arid, araddr, arlen}，进 R phase
//   R : 每 fire 发 mem[rptr]，rptr 每拍 +1，最后一拍拉 RLAST
// =============================================================================
module axi_slave_mem #(
    parameter int ADDR_W = 32,
    parameter int DATA_W = 128,
    parameter int ID_W   = 4,
    parameter int DEPTH  = 1024,     // 单位 = DATA_W-bit word
    // 模拟真实 NoC + DDR latency. AR/AW fire 后随机 wait [MIN..MAX] cycle.
    // MAX=0 = 原 0-cycle 立刻响应 (跟 sim 历史行为一致).
    parameter int AR_LATENCY_MIN = 0,
    parameter int AR_LATENCY_MAX = 0,
    parameter int AW_LATENCY_MIN = 0,
    parameter int AW_LATENCY_MAX = 0,
    parameter int R_GAP_MIN      = 0,   // R burst 中间每拍随机插 gap (NoC 突发抖动)
    parameter int R_GAP_MAX      = 0,
    parameter int RANDOM_SEED    = 32'h12345
)(
    input  logic                clk,
    input  logic                rstn,

    input  logic [ID_W-1:0]     AWID,
    input  logic [ADDR_W-1:0]   AWADDR,
    input  logic [7:0]          AWLEN,
    input  logic [1:0]          AWBURST,
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

    input  logic [ID_W-1:0]     ARID,
    input  logic [ADDR_W-1:0]   ARADDR,
    input  logic [7:0]          ARLEN,
    input  logic [1:0]          ARBURST,
    input  logic                ARVALID,
    output logic                ARREADY,

    output logic [ID_W-1:0]     RID,
    output logic [DATA_W-1:0]   RDATA,
    output logic [1:0]          RRESP,
    output logic                RLAST,
    output logic                RVALID,
    input  logic                RREADY
);

    localparam int AIDX_W   = $clog2(DEPTH);
    localparam int BYTE_OFS = $clog2(DATA_W/8);     // bits of intra-word byte offset

    logic [DATA_W-1:0] mem [0:DEPTH-1];

    // ---- 写通道 ----
    typedef enum logic [1:0] { WS_IDLE, WS_DATA, WS_RESP } wst_t;
    wst_t wst;
    logic [ID_W-1:0]   wid_latch;
    logic [AIDX_W-1:0] wptr;
    logic [7:0]        wremain;

    assign AWREADY = (wst == WS_IDLE);
    assign WREADY  = (wst == WS_DATA);
    assign BVALID  = (wst == WS_RESP);
    assign BID     = wid_latch;
    assign BRESP   = 2'b00;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            wst       <= WS_IDLE;
            wid_latch <= '0;
            wptr      <= '0;
            wremain   <= '0;
        end else begin
            case (wst)
                WS_IDLE: if (AWVALID && AWREADY) begin
                    wid_latch <= AWID;
                    wptr      <= AWADDR[BYTE_OFS+AIDX_W-1 : BYTE_OFS];   // byte→word
                    wremain   <= AWLEN;
                    wst       <= WS_DATA;
                end
                WS_DATA: if (WVALID && WREADY) begin
                    wptr    <= wptr + 1;
                    wremain <= wremain - 1;
                    if (WLAST) wst <= WS_RESP;
                end
                WS_RESP: if (BVALID && BREADY) wst <= WS_IDLE;
                default: wst <= WS_IDLE;
            endcase
        end
    end

    always_ff @(posedge clk) begin
        if (wst == WS_DATA && WVALID && WREADY) mem[wptr] <= WDATA;
    end

    // ---- 读通道 ----
    typedef enum logic [1:0] { RS_IDLE, RS_WAIT, RS_DATA } rst_t;
    rst_t rst_s;
    logic [ID_W-1:0]   rid_latch;
    logic [AIDX_W-1:0] rptr;
    logic [7:0]        rremain;
    logic              rlast_pending;
    logic [15:0]       r_wait_cnt;  // latency counter

    // R burst 中间随机插入 gap (RVALID gating)
    logic [15:0] r_gap_cnt;
    logic        r_gap_active;
    int          r_seed = RANDOM_SEED;

    function automatic int rand_range(input int lo, input int hi);
        if (hi <= lo) return lo;
        return lo + ($random(r_seed) & 32'h7FFFFFFF) % (hi - lo + 1);
    endfunction

    assign ARREADY = (rst_s == RS_IDLE);
    assign RVALID  = (rst_s == RS_DATA) && !r_gap_active;
    assign RID     = rid_latch;
    assign RDATA   = mem[rptr];
    assign RRESP   = 2'b00;
    assign RLAST   = rlast_pending;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            rst_s         <= RS_IDLE;
            rid_latch     <= '0;
            rptr          <= '0;
            rremain       <= '0;
            rlast_pending <= 1'b0;
            r_wait_cnt    <= '0;
            r_gap_cnt     <= '0;
            r_gap_active  <= 1'b0;
        end else begin
            // R gap: 每拍 R 握手后概率插 gap
            if (r_gap_active) begin
                if (r_gap_cnt == 16'd0) r_gap_active <= 1'b0;
                else                     r_gap_cnt   <= r_gap_cnt - 16'd1;
            end
            case (rst_s)
                RS_IDLE: if (ARVALID && ARREADY) begin
                    rid_latch     <= ARID;
                    rptr          <= ARADDR[BYTE_OFS+AIDX_W-1 : BYTE_OFS];
                    rremain       <= ARLEN;
                    rlast_pending <= (ARLEN == 8'd0);
                    if (AR_LATENCY_MAX == 0) begin
                        rst_s     <= RS_DATA;
                    end else begin
                        r_wait_cnt <= rand_range(AR_LATENCY_MIN, AR_LATENCY_MAX)[15:0];
                        rst_s     <= RS_WAIT;
                    end
                end
                RS_WAIT: begin
                    if (r_wait_cnt == 16'd0) rst_s <= RS_DATA;
                    else                     r_wait_cnt <= r_wait_cnt - 16'd1;
                end
                RS_DATA: if (RVALID && RREADY) begin
                    if (rlast_pending) begin
                        rst_s         <= RS_IDLE;
                        rlast_pending <= 1'b0;
                    end else begin
                        rptr          <= rptr + 1;
                        rremain       <= rremain - 1;
                        rlast_pending <= (rremain == 8'd1);
                        // burst 中间随机插 gap 模拟 NoC 突发抖动
                        if (R_GAP_MAX > 0) begin
                            r_gap_cnt    <= rand_range(R_GAP_MIN, R_GAP_MAX)[15:0];
                            r_gap_active <= 1'b1;
                        end
                    end
                end
                default: rst_s <= RS_IDLE;
            endcase
        end
    end

    // ---- 利用率计数器 (TB profiling 用; 不参与功能) ----
    //   aw_fire  : 写突发数 (AWVALID & AWREADY)
    //   w_beats  : 写 beat 数 (WVALID & WREADY)
    //   ar_fire  : 读突发数 (ARVALID & ARREADY)
    //   r_beats  : 读 beat 数 (RVALID & RREADY)
    //   busy_cyc : 任意通道占用 (W/R 通道非 IDLE) 的周期数 — DDR 利用率指标
    int aw_fire  = 0;
    int w_beats  = 0;
    int ar_fire  = 0;
    int r_beats  = 0;
    int busy_cyc = 0;
    always_ff @(posedge clk) begin
        if (!rstn) begin
            aw_fire <= 0; w_beats <= 0; ar_fire <= 0; r_beats <= 0; busy_cyc <= 0;
        end else begin
            if (AWVALID && AWREADY)         aw_fire <= aw_fire + 1;
            if (WVALID  && WREADY)          w_beats <= w_beats + 1;
            if (ARVALID && ARREADY)         ar_fire <= ar_fire + 1;
            if (RVALID  && RREADY)          r_beats <= r_beats + 1;
            if ((wst != WS_IDLE) || (rst_s != RS_IDLE))
                                            busy_cyc <= busy_cyc + 1;
        end
    end

endmodule
