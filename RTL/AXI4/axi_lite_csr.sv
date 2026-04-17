`timescale 1ns/1ps

// =============================================================================
// axi_lite_csr.sv  --  AXI4-Lite Slave → 通用寄存器读/写端口 Bridge
//
// 用途：外部 host 通过 AXI-Lite 写入核内配置寄存器（cfg_regs）+ DMA 描述符；
//       host 也能读回当前寄存器值做状态轮询。
//
// 接口：
//   - AXI-Lite S 侧 5 通道（AW/W/B + AR/R），LEN=0 默认单拍，无 burst
//   - 后端寄存器 bank 侧：
//     reg_w_en  : 单拍脉冲，本拍 reg_w_addr / reg_w_data / reg_w_strb 有效
//     reg_r_addr: 组合地址（读 AR 握手后直通出去）
//     reg_r_data: 组合读（由 bank 侧组合 mux 提供，同拍返回 AXI R）
//
// 时序：
//   W 通道：AW 与 W 可先后到达，均被 latch；两者都就位 → WS_ISSUE 一拍
//            输出 reg_w_en → WS_RESP 等 BREADY → 归 IDLE
//   R 通道：AR 来 → RS_DATA（下一拍 RVALID，带 reg_r_data）→ RREADY 归 IDLE
//
// 复位风格：同步复位（§6）；控制路径（FSM、握手 got flags）保留复位，
//           数据路径（addr/data latch）不复位（上游 valid 遮蔽）。
// =============================================================================

module axi_lite_csr #(
    parameter int ADDR_W = 12,
    parameter int DATA_W = 32
)(
    input  logic                 clk,
    input  logic                 rstn,

    // ---- AXI-Lite Slave ----
    input  logic [ADDR_W-1:0]    AWADDR,
    input  logic                 AWVALID,
    output logic                 AWREADY,

    input  logic [DATA_W-1:0]    WDATA,
    input  logic [DATA_W/8-1:0]  WSTRB,
    input  logic                 WVALID,
    output logic                 WREADY,

    output logic [1:0]           BRESP,
    output logic                 BVALID,
    input  logic                 BREADY,

    input  logic [ADDR_W-1:0]    ARADDR,
    input  logic                 ARVALID,
    output logic                 ARREADY,

    output logic [DATA_W-1:0]    RDATA,
    output logic [1:0]           RRESP,
    output logic                 RVALID,
    input  logic                 RREADY,

    // ---- 后端寄存器 bank 接口 ----
    output logic                 reg_w_en,
    output logic [ADDR_W-1:0]    reg_w_addr,
    output logic [DATA_W-1:0]    reg_w_data,
    output logic [DATA_W/8-1:0]  reg_w_strb,
    output logic [ADDR_W-1:0]    reg_r_addr,
    input  logic [DATA_W-1:0]    reg_r_data
);

    // =========================================================================
    // 写通道状态机
    // =========================================================================
    typedef enum logic [1:0] {
        WS_IDLE   = 2'd0,
        WS_ISSUE  = 2'd1,
        WS_RESP   = 2'd2
    } ws_t;
    ws_t ws, ws_next;

    logic                aw_got, w_got;
    logic [ADDR_W-1:0]   aw_addr;
    logic [DATA_W-1:0]   w_data;
    logic [DATA_W/8-1:0] w_strb;

    // 命名事件
    logic evt_aw_hs, evt_w_hs, evt_issue, evt_both_ready, evt_b_hs;
    always_comb begin
        evt_aw_hs      = (ws == WS_IDLE) && AWVALID && AWREADY;
        evt_w_hs       = (ws == WS_IDLE) && WVALID  && WREADY;
        evt_issue      = (ws == WS_ISSUE);
        evt_both_ready = (ws == WS_IDLE) &&
                         (aw_got || evt_aw_hs) &&
                         (w_got  || evt_w_hs);
        evt_b_hs       = (ws == WS_RESP)  && BVALID   && BREADY;
    end

    // 端口输出（写通道）
    assign AWREADY    = (ws == WS_IDLE) && !aw_got;
    assign WREADY     = (ws == WS_IDLE) && !w_got;
    assign BVALID     = (ws == WS_RESP);
    assign BRESP      = 2'b00;
    assign reg_w_en   = evt_issue;
    assign reg_w_addr = aw_addr;
    assign reg_w_data = w_data;
    assign reg_w_strb = w_strb;

    // 三段式 FSM
    always_comb begin
        ws_next = ws;
        case (ws)
            WS_IDLE  : if (evt_both_ready) ws_next = WS_ISSUE;
            WS_ISSUE : ws_next = WS_RESP;
            WS_RESP  : if (evt_b_hs)       ws_next = WS_IDLE;
            default  :                     ws_next = WS_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rstn) ws <= WS_IDLE;
        else       ws <= ws_next;
    end

    // aw_got / w_got：控制路径（决定 handshake 是否可 accept），复位必须
    always_ff @(posedge clk) begin
        if      (!rstn)     aw_got <= 1'b0;
        else if (evt_issue) aw_got <= 1'b0;
        else if (evt_aw_hs) aw_got <= 1'b1;
        else                aw_got <= aw_got;
    end

    always_ff @(posedge clk) begin
        if      (!rstn)     w_got <= 1'b0;
        else if (evt_issue) w_got <= 1'b0;
        else if (evt_w_hs)  w_got <= 1'b1;
        else                w_got <= w_got;
    end

    // aw_addr / w_data / w_strb：数据路径，无复位
    always_ff @(posedge clk) begin
        if (evt_aw_hs) aw_addr <= AWADDR;
        else           aw_addr <= aw_addr;
    end

    always_ff @(posedge clk) begin
        if (evt_w_hs) w_data <= WDATA;
        else          w_data <= w_data;
    end

    always_ff @(posedge clk) begin
        if (evt_w_hs) w_strb <= WSTRB;
        else          w_strb <= w_strb;
    end

    // =========================================================================
    // 读通道状态机
    // =========================================================================
    typedef enum logic {
        RS_IDLE = 1'b0,
        RS_DATA = 1'b1
    } rs_t;
    rs_t rs, rs_next;
    logic [ADDR_W-1:0] ar_addr;

    logic evt_ar_hs, evt_r_hs;
    always_comb begin
        evt_ar_hs = (rs == RS_IDLE) && ARVALID && ARREADY;
        evt_r_hs  = (rs == RS_DATA) && RVALID  && RREADY;
    end

    assign ARREADY    = (rs == RS_IDLE);
    assign RVALID     = (rs == RS_DATA);
    assign RRESP      = 2'b00;
    assign RDATA      = reg_r_data;
    // RS_IDLE 下把 ARADDR 直通后端，这拍 reg_r_data 已准备好；AR 握手时 latch，
    // RS_DATA 下继续送 latch 的 ar_addr，使 reg_r_data 保持
    assign reg_r_addr = (rs == RS_DATA) ? ar_addr : ARADDR;

    always_comb begin
        rs_next = rs;
        case (rs)
            RS_IDLE : if (evt_ar_hs) rs_next = RS_DATA;
            RS_DATA : if (evt_r_hs)  rs_next = RS_IDLE;
            default :                rs_next = RS_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rstn) rs <= RS_IDLE;
        else       rs <= rs_next;
    end

    always_ff @(posedge clk) begin
        if (evt_ar_hs) ar_addr <= ARADDR;
        else           ar_addr <= ar_addr;
    end

endmodule
