`timescale 1ns/1ps

// =============================================================================
// axi_m_mux.sv  --  N-to-1 AXI4 Master Aggregator (single clock domain)
//
// 用途：把多个 AXI4 Master（此项目：IDMA / WDMA / ODMA + 预留）聚合成一条
// 对外 AXI4 M 接口。纯同步域，不做 CDC；多时钟场景请用 axi_bus 全家桶。
//
// 结构：
//   - 控制面：实例化 axi_master_arbiter（零改动），按仲裁规则给出 per-channel
//     master_sel 选择线 + channel_lock。优先级：低 index 优先（用户代码原规则）。
//   - 数据面：5 条 AXI 通道（AW / W / AR 下行，B / R 上行）按 master_sel 做
//     组合 mux；上行通道靠 BUS_*_ID 的高 M_WIDTH 位译回目标 master。
//
// ID 编解码：
//   下行 AW / AR：B_xID = {master_idx, M_xID}        (宽度 M_ID + M_WIDTH)
//   上行 B  / R ：M_xID = B_xID[M_ID-1:0]（剥掉高位的 master 索引）
//                 master_idx = B_xID[M_ID +: M_WIDTH] 由 arbiter 译码
//
// 复位：本模块无状态寄存器（全组合 mux + 内部 arbiter 自带小状态机）。
// =============================================================================

module axi_m_mux #(
    parameter int M_WIDTH = 2,              // log2(masters)，masters = 2^M_WIDTH
    parameter int M_ID    = 2,              // per-master ID 宽度
    parameter int ADDR_W  = 32,
    parameter int DATA_W  = 128
)(
    input  logic                               clk,
    input  logic                               rstn,

    // ---- N masters 侧（packed 2D 打包） ----
    input  logic [(2**M_WIDTH)-1:0] [M_ID-1:0]     M_AWID,
    input  logic [(2**M_WIDTH)-1:0] [ADDR_W-1:0]   M_AWADDR,
    input  logic [(2**M_WIDTH)-1:0] [7:0]          M_AWLEN,
    input  logic [(2**M_WIDTH)-1:0] [1:0]          M_AWBURST,
    input  logic [(2**M_WIDTH)-1:0]                M_AWVALID,
    output logic [(2**M_WIDTH)-1:0]                M_AWREADY,

    input  logic [(2**M_WIDTH)-1:0] [DATA_W-1:0]   M_WDATA,
    input  logic [(2**M_WIDTH)-1:0] [DATA_W/8-1:0] M_WSTRB,
    input  logic [(2**M_WIDTH)-1:0]                M_WLAST,
    input  logic [(2**M_WIDTH)-1:0]                M_WVALID,
    output logic [(2**M_WIDTH)-1:0]                M_WREADY,

    output logic [(2**M_WIDTH)-1:0] [M_ID-1:0]     M_BID,
    output logic [(2**M_WIDTH)-1:0] [1:0]          M_BRESP,
    output logic [(2**M_WIDTH)-1:0]                M_BVALID,
    input  logic [(2**M_WIDTH)-1:0]                M_BREADY,

    input  logic [(2**M_WIDTH)-1:0] [M_ID-1:0]     M_ARID,
    input  logic [(2**M_WIDTH)-1:0] [ADDR_W-1:0]   M_ARADDR,
    input  logic [(2**M_WIDTH)-1:0] [7:0]          M_ARLEN,
    input  logic [(2**M_WIDTH)-1:0] [1:0]          M_ARBURST,
    input  logic [(2**M_WIDTH)-1:0]                M_ARVALID,
    output logic [(2**M_WIDTH)-1:0]                M_ARREADY,

    output logic [(2**M_WIDTH)-1:0] [M_ID-1:0]     M_RID,
    output logic [(2**M_WIDTH)-1:0] [DATA_W-1:0]   M_RDATA,
    output logic [(2**M_WIDTH)-1:0] [1:0]          M_RRESP,
    output logic [(2**M_WIDTH)-1:0]                M_RLAST,
    output logic [(2**M_WIDTH)-1:0]                M_RVALID,
    input  logic [(2**M_WIDTH)-1:0]                M_RREADY,

    // ---- 1 bus 侧（对外 AXI4 M） ----
    output logic [M_ID+M_WIDTH-1:0]   B_AWID,
    output logic [ADDR_W-1:0]         B_AWADDR,
    output logic [7:0]                B_AWLEN,
    output logic [1:0]                B_AWBURST,
    output logic                      B_AWVALID,
    input  logic                      B_AWREADY,

    output logic [DATA_W-1:0]         B_WDATA,
    output logic [DATA_W/8-1:0]       B_WSTRB,
    output logic                      B_WLAST,
    output logic                      B_WVALID,
    input  logic                      B_WREADY,

    input  logic [M_ID+M_WIDTH-1:0]   B_BID,
    input  logic [1:0]                B_BRESP,
    input  logic                      B_BVALID,
    output logic                      B_BREADY,

    output logic [M_ID+M_WIDTH-1:0]   B_ARID,
    output logic [ADDR_W-1:0]         B_ARADDR,
    output logic [7:0]                B_ARLEN,
    output logic [1:0]                B_ARBURST,
    output logic                      B_ARVALID,
    input  logic                      B_ARREADY,

    input  logic [M_ID+M_WIDTH-1:0]   B_RID,
    input  logic [DATA_W-1:0]         B_RDATA,
    input  logic [1:0]                B_RRESP,
    input  logic                      B_RLAST,
    input  logic                      B_RVALID,
    output logic                      B_RREADY
);

    // =========================================================================
    // 仲裁器：产生 per-channel master_sel（控制面）
    // =========================================================================
    logic [M_WIDTH-1:0] wr_addr_master_sel;
    logic               wr_addr_master_lock;
    logic [M_WIDTH-1:0] wr_data_master_sel;
    logic               wr_data_master_lock;
    logic [M_WIDTH-1:0] wr_resp_master_sel;
    logic [M_WIDTH-1:0] rd_addr_master_sel;
    logic               rd_addr_master_lock;
    logic [M_WIDTH-1:0] rd_data_master_sel;

    axi_master_arbiter #(
        .M_ID   (M_ID),
        .M_WIDTH(M_WIDTH)
    ) u_arbiter (
        .clk                   (clk),
        .rstn                  (rstn),
        .MASTER_WR_ADDR_VALID  (M_AWVALID),
        .MASTER_RD_ADDR_VALID  (M_ARVALID),
        .BUS_WR_ADDR_VALID     (B_AWVALID),
        .BUS_WR_ADDR_READY     (B_AWREADY),
        .BUS_WR_DATA_VALID     (B_WVALID),
        .BUS_WR_DATA_READY     (B_WREADY),
        .BUS_WR_DATA_LAST      (B_WLAST),
        .BUS_WR_BACK_ID        (B_BID),
        .BUS_RD_ADDR_VALID     (B_ARVALID),
        .BUS_RD_ADDR_READY     (B_ARREADY),
        .BUS_RD_BACK_ID        (B_RID),
        .wr_addr_master_sel    (wr_addr_master_sel),
        .wr_addr_master_lock   (wr_addr_master_lock),
        .wr_data_master_sel    (wr_data_master_sel),
        .wr_data_master_lock   (wr_data_master_lock),
        .wr_resp_master_sel    (wr_resp_master_sel),
        .rd_addr_master_sel    (rd_addr_master_sel),
        .rd_addr_master_lock   (rd_addr_master_lock),
        .rd_data_master_sel    (rd_data_master_sel)
    );

    // =========================================================================
    // AW 通道：M[sel] → B，ID 左拼 master_idx
    // =========================================================================
    assign B_AWID    = {wr_addr_master_sel, M_AWID   [wr_addr_master_sel]};
    assign B_AWADDR  =                       M_AWADDR [wr_addr_master_sel];
    assign B_AWLEN   =                       M_AWLEN  [wr_addr_master_sel];
    assign B_AWBURST =                       M_AWBURST[wr_addr_master_sel];
    assign B_AWVALID =                       M_AWVALID[wr_addr_master_sel];

    always_comb begin
        M_AWREADY                     = '0;
        M_AWREADY[wr_addr_master_sel] = B_AWREADY;
    end

    // =========================================================================
    // W 通道：M[sel] → B（burst 期间 lock 保证 sel 稳定）
    // =========================================================================
    assign B_WDATA  = M_WDATA [wr_data_master_sel];
    assign B_WSTRB  = M_WSTRB [wr_data_master_sel];
    assign B_WLAST  = M_WLAST [wr_data_master_sel];
    assign B_WVALID = M_WVALID[wr_data_master_sel];

    always_comb begin
        M_WREADY                     = '0;
        M_WREADY[wr_data_master_sel] = B_WREADY;
    end

    // =========================================================================
    // B 通道：B → M[wr_resp_master_sel]，ID 剥掉高位的 master_idx
    // =========================================================================
    always_comb begin
        M_BID    = '0;
        M_BRESP  = '0;
        M_BVALID = '0;
        M_BID   [wr_resp_master_sel] = B_BID[M_ID-1:0];
        M_BRESP [wr_resp_master_sel] = B_BRESP;
        M_BVALID[wr_resp_master_sel] = B_BVALID;
    end

    assign B_BREADY = M_BREADY[wr_resp_master_sel];

    // =========================================================================
    // AR 通道：M[sel] → B
    // =========================================================================
    assign B_ARID    = {rd_addr_master_sel, M_ARID   [rd_addr_master_sel]};
    assign B_ARADDR  =                       M_ARADDR [rd_addr_master_sel];
    assign B_ARLEN   =                       M_ARLEN  [rd_addr_master_sel];
    assign B_ARBURST =                       M_ARBURST[rd_addr_master_sel];
    assign B_ARVALID =                       M_ARVALID[rd_addr_master_sel];

    always_comb begin
        M_ARREADY                     = '0;
        M_ARREADY[rd_addr_master_sel] = B_ARREADY;
    end

    // =========================================================================
    // R 通道：B → M[rd_data_master_sel]
    // =========================================================================
    always_comb begin
        M_RID    = '0;
        M_RDATA  = '0;
        M_RRESP  = '0;
        M_RLAST  = '0;
        M_RVALID = '0;
        M_RID   [rd_data_master_sel] = B_RID[M_ID-1:0];
        M_RDATA [rd_data_master_sel] = B_RDATA;
        M_RRESP [rd_data_master_sel] = B_RRESP;
        M_RLAST [rd_data_master_sel] = B_RLAST;
        M_RVALID[rd_data_master_sel] = B_RVALID;
    end

    assign B_RREADY = M_RREADY[rd_data_master_sel];

endmodule
