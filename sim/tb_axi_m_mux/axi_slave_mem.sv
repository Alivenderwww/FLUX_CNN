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
    parameter int DEPTH  = 1024      // 单位 = DATA_W-bit word
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
    typedef enum logic [1:0] { RS_IDLE, RS_DATA } rst_t;
    rst_t rst_s;
    logic [ID_W-1:0]   rid_latch;
    logic [AIDX_W-1:0] rptr;
    logic [7:0]        rremain;
    logic              rlast_pending;

    assign ARREADY = (rst_s == RS_IDLE);
    assign RVALID  = (rst_s == RS_DATA);
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
        end else begin
            case (rst_s)
                RS_IDLE: if (ARVALID && ARREADY) begin
                    rid_latch     <= ARID;
                    rptr          <= ARADDR[BYTE_OFS+AIDX_W-1 : BYTE_OFS];
                    rremain       <= ARLEN;
                    rlast_pending <= (ARLEN == 8'd0);
                    rst_s         <= RS_DATA;
                end
                RS_DATA: if (RVALID && RREADY) begin
                    if (rlast_pending) begin
                        rst_s         <= RS_IDLE;
                        rlast_pending <= 1'b0;
                    end else begin
                        rptr          <= rptr + 1;
                        rremain       <= rremain - 1;
                        rlast_pending <= (rremain == 8'd1);
                    end
                end
                default: rst_s <= RS_IDLE;
            endcase
        end
    end

endmodule
