`timescale 1ns/1ps

// =============================================================================
// multicore_top_4ddr.sv  --  N=4 多 DDR 端口 PoC wrapper (sim 实验性)
//
// 跟 multicore_top.sv 区别:
//   * NUM_CORES 固定 4 (PoC 不参数化)
//   * crossbar 从 axi_4to5 升级 axi_4to8 (4 SI / 8 MI = 4 DDR + 4 IFB push)
//   * 输出 4 组独立 AXI4 master (bus_ddr_*[0..3]), TB 接 4 个独立 axi_slave_mem
//     测多 DDR 真带宽收益 (Patch s2d 单核已逼近 1.6 GB/s 单端口峰值, 4 个独立
//     端口理想能 4× → ResNet11 N=4 wall ↓50%+).
//   * 跨核 IFB push 仍走 crossbar 内部 (MI[4..7] = Core IFB SI), 不出顶层
//
// 地址布局:
//   DDR slot[i]: 0x0i00_0000 - 0x0iFF_FFFF (256 MB / slot)
//                MI[0..3], 出顶层 bus_ddr*_*
//   Core[i] IFB: 0x80000000 + i × 0x10000000 (256 MB / core)
//                MI[4..7], 内部接 gen_core[i].rmt_ifb_*
//
// 当前 PoC 只用作 sim TB, 不替代 multicore_top.sv (多 DDR 在 XC7K325T 单 controller
// 上没有真实意义, 跑出来收益再决定是否迁 VCU118/U280 board).
//
// 实施: 复用 multicore_top.sv 大部分结构 (axi_lite_1to4, gen_core 4 instance,
// SI ID padding), 只换中间 crossbar IP + MI 端口数 + DDR 出口. axi_dm 等核内 IP
// 跟 multicore_top.sv 一致.
// =============================================================================
`include "flux_cnn_params.svh"

module multicore_top_4ddr #(
    parameter int NUM_COL     = `FLUX_NUM_COL,
    parameter int NUM_PE      = `FLUX_NUM_PE,
    parameter int DATA_WIDTH  = `FLUX_DATA_WIDTH,
    parameter int PSUM_WIDTH  = `FLUX_PSUM_WIDTH,
    parameter int WRF_DEPTH   = `FLUX_WRF_DEPTH,
    parameter int ARF_DEPTH   = `FLUX_ARF_DEPTH,
    parameter int PARF_DEPTH  = `FLUX_PARF_DEPTH,
    parameter int SRAM_DEPTH  = `FLUX_IFB_DEPTH,
    parameter int WB_DEPTH    = `FLUX_WB_DEPTH,
    parameter int OFB_DEPTH   = `FLUX_OFB_DEPTH,
    parameter int CSR_DATA_W  = `FLUX_CSR_DATA_W,
    parameter int BUS_ADDR_W  = `FLUX_BUS_ADDR_W,
    parameter int BUS_DATA_W  = `FLUX_BUS_DATA_W,
    parameter int AXI_M_ID    = `FLUX_AXI_M_ID,
    parameter int AXI_M_WIDTH = `FLUX_AXI_M_WIDTH,
    parameter int DMA_LEN_W   = `FLUX_DMA_LEN_W,

    // PoC 固定 N=4 不参数化
    localparam int NUM_CORES   = 4,
    localparam int NUM_DDR     = 4,                 // 多 DDR 端口数 (= NUM_CORES)
    localparam int CORE_ID_W   = 2,
    localparam int HOST_CSR_AW = 12 + CORE_ID_W,    // 14 bit
    localparam int CORE_BUS_ID = AXI_M_ID + AXI_M_WIDTH,   // 4 bit
    localparam int EXT_BUS_ID  = CORE_BUS_ID + CORE_ID_W   // 6 bit
)(
    input  logic                                 clk,
    input  logic                                 rst_n,

    // ---- AXI-Lite Slave (host CSR 配置, 14 bit addr) ----
    input  logic [HOST_CSR_AW-1:0]               csr_awaddr,
    input  logic                                 csr_awvalid,
    output logic                                 csr_awready,
    input  logic [CSR_DATA_W-1:0]                csr_wdata,
    input  logic [CSR_DATA_W/8-1:0]              csr_wstrb,
    input  logic                                 csr_wvalid,
    output logic                                 csr_wready,
    output logic [1:0]                           csr_bresp,
    output logic                                 csr_bvalid,
    input  logic                                 csr_bready,
    input  logic [HOST_CSR_AW-1:0]               csr_araddr,
    input  logic                                 csr_arvalid,
    output logic                                 csr_arready,
    output logic [CSR_DATA_W-1:0]                csr_rdata,
    output logic [1:0]                           csr_rresp,
    output logic                                 csr_rvalid,
    input  logic                                 csr_rready,

    // ---- 4 个独立 AXI4 Master 出口 (各接独立 axi_slave_mem) ----
    // packed array 形式: bus_ddr_awid[i*EXT_BUS_ID +: EXT_BUS_ID] = DDR[i] AWID
    output logic [NUM_DDR*EXT_BUS_ID-1:0]        bus_ddr_awid,
    output logic [NUM_DDR*BUS_ADDR_W-1:0]        bus_ddr_awaddr,
    output logic [NUM_DDR*8-1:0]                 bus_ddr_awlen,
    output logic [NUM_DDR*2-1:0]                 bus_ddr_awburst,
    output logic [NUM_DDR-1:0]                   bus_ddr_awvalid,
    input  logic [NUM_DDR-1:0]                   bus_ddr_awready,
    output logic [NUM_DDR*BUS_DATA_W-1:0]        bus_ddr_wdata,
    output logic [NUM_DDR*(BUS_DATA_W/8)-1:0]    bus_ddr_wstrb,
    output logic [NUM_DDR-1:0]                   bus_ddr_wlast,
    output logic [NUM_DDR-1:0]                   bus_ddr_wvalid,
    input  logic [NUM_DDR-1:0]                   bus_ddr_wready,
    input  logic [NUM_DDR*EXT_BUS_ID-1:0]        bus_ddr_bid,
    input  logic [NUM_DDR*2-1:0]                 bus_ddr_bresp,
    input  logic [NUM_DDR-1:0]                   bus_ddr_bvalid,
    output logic [NUM_DDR-1:0]                   bus_ddr_bready,
    output logic [NUM_DDR*EXT_BUS_ID-1:0]        bus_ddr_arid,
    output logic [NUM_DDR*BUS_ADDR_W-1:0]        bus_ddr_araddr,
    output logic [NUM_DDR*8-1:0]                 bus_ddr_arlen,
    output logic [NUM_DDR*2-1:0]                 bus_ddr_arburst,
    output logic [NUM_DDR-1:0]                   bus_ddr_arvalid,
    input  logic [NUM_DDR-1:0]                   bus_ddr_arready,
    input  logic [NUM_DDR*EXT_BUS_ID-1:0]        bus_ddr_rid,
    input  logic [NUM_DDR*BUS_DATA_W-1:0]        bus_ddr_rdata,
    input  logic [NUM_DDR*2-1:0]                 bus_ddr_rresp,
    input  logic [NUM_DDR-1:0]                   bus_ddr_rlast,
    input  logic [NUM_DDR-1:0]                   bus_ddr_rvalid,
    output logic [NUM_DDR-1:0]                   bus_ddr_rready,

    output logic [NUM_CORES-1:0]                 done_per_core
);

    logic aresetn;
    assign aresetn = rst_n;

    localparam int NUM_MI_BUS = NUM_DDR + NUM_CORES;   // 8

    // =========================================================================
    // 1. AXI-Lite 1:4 crossbar (复用 axi_lite_1to4 IP)
    // =========================================================================
    logic [NUM_CORES*HOST_CSR_AW-1:0]   csr_m_awaddr;
    logic [NUM_CORES*3-1:0]             csr_m_awprot;
    logic [NUM_CORES-1:0]               csr_m_awvalid, csr_m_awready;
    logic [NUM_CORES*CSR_DATA_W-1:0]    csr_m_wdata;
    logic [NUM_CORES*(CSR_DATA_W/8)-1:0]csr_m_wstrb;
    logic [NUM_CORES-1:0]               csr_m_wvalid, csr_m_wready;
    logic [NUM_CORES*2-1:0]             csr_m_bresp;
    logic [NUM_CORES-1:0]               csr_m_bvalid, csr_m_bready;
    logic [NUM_CORES*HOST_CSR_AW-1:0]   csr_m_araddr;
    logic [NUM_CORES*3-1:0]             csr_m_arprot;
    logic [NUM_CORES-1:0]               csr_m_arvalid, csr_m_arready;
    logic [NUM_CORES*CSR_DATA_W-1:0]    csr_m_rdata;
    logic [NUM_CORES*2-1:0]             csr_m_rresp;
    logic [NUM_CORES-1:0]               csr_m_rvalid, csr_m_rready;

    axi_lite_1to4 u_csr_xbar (
        .aclk          (clk), .aresetn       (aresetn),
        .s_axi_awaddr  (csr_awaddr),    .s_axi_awprot  (3'b000),
        .s_axi_awvalid (csr_awvalid),   .s_axi_awready (csr_awready),
        .s_axi_wdata   (csr_wdata),     .s_axi_wstrb   (csr_wstrb),
        .s_axi_wvalid  (csr_wvalid),    .s_axi_wready  (csr_wready),
        .s_axi_bresp   (csr_bresp),
        .s_axi_bvalid  (csr_bvalid),    .s_axi_bready  (csr_bready),
        .s_axi_araddr  (csr_araddr),    .s_axi_arprot  (3'b000),
        .s_axi_arvalid (csr_arvalid),   .s_axi_arready (csr_arready),
        .s_axi_rdata   (csr_rdata),     .s_axi_rresp   (csr_rresp),
        .s_axi_rvalid  (csr_rvalid),    .s_axi_rready  (csr_rready),
        .m_axi_awaddr  (csr_m_awaddr),  .m_axi_awprot  (csr_m_awprot),
        .m_axi_awvalid (csr_m_awvalid), .m_axi_awready (csr_m_awready),
        .m_axi_wdata   (csr_m_wdata),   .m_axi_wstrb   (csr_m_wstrb),
        .m_axi_wvalid  (csr_m_wvalid),  .m_axi_wready  (csr_m_wready),
        .m_axi_bresp   (csr_m_bresp),
        .m_axi_bvalid  (csr_m_bvalid),  .m_axi_bready  (csr_m_bready),
        .m_axi_araddr  (csr_m_araddr),  .m_axi_arprot  (csr_m_arprot),
        .m_axi_arvalid (csr_m_arvalid), .m_axi_arready (csr_m_arready),
        .m_axi_rdata   (csr_m_rdata),   .m_axi_rresp   (csr_m_rresp),
        .m_axi_rvalid  (csr_m_rvalid),  .m_axi_rready  (csr_m_rready)
    );

    // =========================================================================
    // 2. SI 端 packed array (来自 4 个核的 axi_m_mux 出口)
    // =========================================================================
    logic [NUM_CORES*CORE_BUS_ID-1:0]   bus_s_awid;
    logic [NUM_CORES*BUS_ADDR_W-1:0]    bus_s_awaddr;
    logic [NUM_CORES*8-1:0]             bus_s_awlen;
    logic [NUM_CORES*3-1:0]             bus_s_awsize;
    logic [NUM_CORES*2-1:0]             bus_s_awburst;
    logic [NUM_CORES*1-1:0]             bus_s_awlock;
    logic [NUM_CORES*4-1:0]             bus_s_awcache;
    logic [NUM_CORES*3-1:0]             bus_s_awprot;
    logic [NUM_CORES*4-1:0]             bus_s_awqos;
    logic [NUM_CORES-1:0]               bus_s_awvalid, bus_s_awready;
    logic [NUM_CORES*BUS_DATA_W-1:0]    bus_s_wdata;
    logic [NUM_CORES*(BUS_DATA_W/8)-1:0]bus_s_wstrb;
    logic [NUM_CORES-1:0]               bus_s_wlast, bus_s_wvalid, bus_s_wready;
    logic [NUM_CORES*CORE_BUS_ID-1:0]   bus_s_bid;
    logic [NUM_CORES*2-1:0]             bus_s_bresp;
    logic [NUM_CORES-1:0]               bus_s_bvalid, bus_s_bready;
    logic [NUM_CORES*CORE_BUS_ID-1:0]   bus_s_arid;
    logic [NUM_CORES*BUS_ADDR_W-1:0]    bus_s_araddr;
    logic [NUM_CORES*8-1:0]             bus_s_arlen;
    logic [NUM_CORES*3-1:0]             bus_s_arsize;
    logic [NUM_CORES*2-1:0]             bus_s_arburst;
    logic [NUM_CORES*1-1:0]             bus_s_arlock;
    logic [NUM_CORES*4-1:0]             bus_s_arcache;
    logic [NUM_CORES*3-1:0]             bus_s_arprot;
    logic [NUM_CORES*4-1:0]             bus_s_arqos;
    logic [NUM_CORES-1:0]               bus_s_arvalid, bus_s_arready;
    logic [NUM_CORES*CORE_BUS_ID-1:0]   bus_s_rid;
    logic [NUM_CORES*BUS_DATA_W-1:0]    bus_s_rdata;
    logic [NUM_CORES*2-1:0]             bus_s_rresp;
    logic [NUM_CORES-1:0]               bus_s_rlast, bus_s_rvalid, bus_s_rready;

    // SI ID padding (CORE_BUS_ID=4 → EXT_BUS_ID=6, zero-extend)
    logic [NUM_CORES*EXT_BUS_ID-1:0]    bus_s_awid_padded, bus_s_arid_padded;
    logic [NUM_CORES*EXT_BUS_ID-1:0]    bus_s_bid_padded,  bus_s_rid_padded;
    genvar gp;
    generate
        for (gp = 0; gp < NUM_CORES; gp++) begin : g_si_id_pad
            assign bus_s_awid_padded[gp*EXT_BUS_ID +: EXT_BUS_ID] =
                {{(EXT_BUS_ID-CORE_BUS_ID){1'b0}}, bus_s_awid[gp*CORE_BUS_ID +: CORE_BUS_ID]};
            assign bus_s_arid_padded[gp*EXT_BUS_ID +: EXT_BUS_ID] =
                {{(EXT_BUS_ID-CORE_BUS_ID){1'b0}}, bus_s_arid[gp*CORE_BUS_ID +: CORE_BUS_ID]};
            assign bus_s_bid[gp*CORE_BUS_ID +: CORE_BUS_ID] =
                bus_s_bid_padded[gp*EXT_BUS_ID +: CORE_BUS_ID];
            assign bus_s_rid[gp*CORE_BUS_ID +: CORE_BUS_ID] =
                bus_s_rid_padded[gp*EXT_BUS_ID +: CORE_BUS_ID];
        end
    endgenerate

    // =========================================================================
    // 3. MI 端 packed array (8 = 4 DDR + 4 IFB)
    // =========================================================================
    logic [NUM_MI_BUS*EXT_BUS_ID-1:0]   bus_m_awid;
    logic [NUM_MI_BUS*BUS_ADDR_W-1:0]   bus_m_awaddr;
    logic [NUM_MI_BUS*8-1:0]            bus_m_awlen;
    logic [NUM_MI_BUS*3-1:0]            bus_m_awsize;
    logic [NUM_MI_BUS*2-1:0]            bus_m_awburst;
    logic [NUM_MI_BUS*1-1:0]            bus_m_awlock;
    logic [NUM_MI_BUS*4-1:0]            bus_m_awcache;
    logic [NUM_MI_BUS*3-1:0]            bus_m_awprot;
    logic [NUM_MI_BUS*4-1:0]            bus_m_awqos;
    logic [NUM_MI_BUS-1:0]              bus_m_awvalid, bus_m_awready;
    logic [NUM_MI_BUS*BUS_DATA_W-1:0]   bus_m_wdata;
    logic [NUM_MI_BUS*(BUS_DATA_W/8)-1:0] bus_m_wstrb;
    logic [NUM_MI_BUS-1:0]              bus_m_wlast, bus_m_wvalid, bus_m_wready;
    logic [NUM_MI_BUS*EXT_BUS_ID-1:0]   bus_m_bid;
    logic [NUM_MI_BUS*2-1:0]            bus_m_bresp;
    logic [NUM_MI_BUS-1:0]              bus_m_bvalid, bus_m_bready;
    logic [NUM_MI_BUS*EXT_BUS_ID-1:0]   bus_m_arid;
    logic [NUM_MI_BUS*BUS_ADDR_W-1:0]   bus_m_araddr;
    logic [NUM_MI_BUS*8-1:0]            bus_m_arlen;
    logic [NUM_MI_BUS*3-1:0]            bus_m_arsize;
    logic [NUM_MI_BUS*2-1:0]            bus_m_arburst;
    logic [NUM_MI_BUS*1-1:0]            bus_m_arlock;
    logic [NUM_MI_BUS*4-1:0]            bus_m_arcache;
    logic [NUM_MI_BUS*3-1:0]            bus_m_arprot;
    logic [NUM_MI_BUS*4-1:0]            bus_m_arqos;
    logic [NUM_MI_BUS-1:0]              bus_m_arvalid, bus_m_arready;
    logic [NUM_MI_BUS*EXT_BUS_ID-1:0]   bus_m_rid;
    logic [NUM_MI_BUS*BUS_DATA_W-1:0]   bus_m_rdata;
    logic [NUM_MI_BUS*2-1:0]            bus_m_rresp;
    logic [NUM_MI_BUS-1:0]              bus_m_rlast, bus_m_rvalid, bus_m_rready;

    // MI[0..3] = 4 个 DDR slot, 直连顶层 bus_ddr_*
    genvar gd;
    generate
        for (gd = 0; gd < NUM_DDR; gd++) begin : g_ddr_mi
            assign bus_ddr_awid   [gd*EXT_BUS_ID  +: EXT_BUS_ID]  = bus_m_awid   [gd*EXT_BUS_ID  +: EXT_BUS_ID];
            assign bus_ddr_awaddr [gd*BUS_ADDR_W  +: BUS_ADDR_W]  = bus_m_awaddr [gd*BUS_ADDR_W  +: BUS_ADDR_W];
            assign bus_ddr_awlen  [gd*8           +: 8]           = bus_m_awlen  [gd*8           +: 8];
            assign bus_ddr_awburst[gd*2           +: 2]           = bus_m_awburst[gd*2           +: 2];
            assign bus_ddr_awvalid[gd] = bus_m_awvalid[gd];
            assign bus_m_awready  [gd] = bus_ddr_awready[gd];
            assign bus_ddr_wdata  [gd*BUS_DATA_W  +: BUS_DATA_W]  = bus_m_wdata  [gd*BUS_DATA_W  +: BUS_DATA_W];
            assign bus_ddr_wstrb  [gd*(BUS_DATA_W/8) +: (BUS_DATA_W/8)] = bus_m_wstrb[gd*(BUS_DATA_W/8) +: (BUS_DATA_W/8)];
            assign bus_ddr_wlast  [gd] = bus_m_wlast [gd];
            assign bus_ddr_wvalid [gd] = bus_m_wvalid[gd];
            assign bus_m_wready   [gd] = bus_ddr_wready[gd];
            assign bus_m_bid      [gd*EXT_BUS_ID  +: EXT_BUS_ID]  = bus_ddr_bid  [gd*EXT_BUS_ID  +: EXT_BUS_ID];
            assign bus_m_bresp    [gd*2           +: 2]           = bus_ddr_bresp[gd*2           +: 2];
            assign bus_m_bvalid   [gd] = bus_ddr_bvalid[gd];
            assign bus_ddr_bready [gd] = bus_m_bready  [gd];
            assign bus_ddr_arid   [gd*EXT_BUS_ID  +: EXT_BUS_ID]  = bus_m_arid   [gd*EXT_BUS_ID  +: EXT_BUS_ID];
            assign bus_ddr_araddr [gd*BUS_ADDR_W  +: BUS_ADDR_W]  = bus_m_araddr [gd*BUS_ADDR_W  +: BUS_ADDR_W];
            assign bus_ddr_arlen  [gd*8           +: 8]           = bus_m_arlen  [gd*8           +: 8];
            assign bus_ddr_arburst[gd*2           +: 2]           = bus_m_arburst[gd*2           +: 2];
            assign bus_ddr_arvalid[gd] = bus_m_arvalid[gd];
            assign bus_m_arready  [gd] = bus_ddr_arready[gd];
            assign bus_m_rid      [gd*EXT_BUS_ID  +: EXT_BUS_ID]  = bus_ddr_rid  [gd*EXT_BUS_ID  +: EXT_BUS_ID];
            assign bus_m_rdata    [gd*BUS_DATA_W  +: BUS_DATA_W]  = bus_ddr_rdata[gd*BUS_DATA_W  +: BUS_DATA_W];
            assign bus_m_rresp    [gd*2           +: 2]           = bus_ddr_rresp[gd*2           +: 2];
            assign bus_m_rlast    [gd] = bus_ddr_rlast [gd];
            assign bus_m_rvalid   [gd] = bus_ddr_rvalid[gd];
            assign bus_ddr_rready [gd] = bus_m_rready  [gd];
        end
    endgenerate

    // =========================================================================
    // 4. axi_4to8 crossbar 实例 (4 SI / 8 MI)
    // =========================================================================
    axi_4to8 u_bus_xbar (
        .aclk          (clk),    .aresetn       (aresetn),
        .s_axi_awid    (bus_s_awid_padded),  .s_axi_awaddr  (bus_s_awaddr),
        .s_axi_awlen   (bus_s_awlen),        .s_axi_awsize  (bus_s_awsize),
        .s_axi_awburst (bus_s_awburst),      .s_axi_awlock  (bus_s_awlock),
        .s_axi_awcache (bus_s_awcache),      .s_axi_awprot  (bus_s_awprot),
        .s_axi_awqos   (bus_s_awqos),        .s_axi_awvalid (bus_s_awvalid),
        .s_axi_awready (bus_s_awready),      .s_axi_wdata   (bus_s_wdata),
        .s_axi_wstrb   (bus_s_wstrb),        .s_axi_wlast   (bus_s_wlast),
        .s_axi_wvalid  (bus_s_wvalid),       .s_axi_wready  (bus_s_wready),
        .s_axi_bid     (bus_s_bid_padded),   .s_axi_bresp   (bus_s_bresp),
        .s_axi_bvalid  (bus_s_bvalid),       .s_axi_bready  (bus_s_bready),
        .s_axi_arid    (bus_s_arid_padded),  .s_axi_araddr  (bus_s_araddr),
        .s_axi_arlen   (bus_s_arlen),        .s_axi_arsize  (bus_s_arsize),
        .s_axi_arburst (bus_s_arburst),      .s_axi_arlock  (bus_s_arlock),
        .s_axi_arcache (bus_s_arcache),      .s_axi_arprot  (bus_s_arprot),
        .s_axi_arqos   (bus_s_arqos),        .s_axi_arvalid (bus_s_arvalid),
        .s_axi_arready (bus_s_arready),      .s_axi_rid     (bus_s_rid_padded),
        .s_axi_rdata   (bus_s_rdata),        .s_axi_rresp   (bus_s_rresp),
        .s_axi_rlast   (bus_s_rlast),        .s_axi_rvalid  (bus_s_rvalid),
        .s_axi_rready  (bus_s_rready),
        .m_axi_awid    (bus_m_awid),         .m_axi_awaddr  (bus_m_awaddr),
        .m_axi_awlen   (bus_m_awlen),        .m_axi_awsize  (bus_m_awsize),
        .m_axi_awburst (bus_m_awburst),      .m_axi_awlock  (bus_m_awlock),
        .m_axi_awcache (bus_m_awcache),      .m_axi_awprot  (bus_m_awprot),
        .m_axi_awregion(),                   .m_axi_awqos   (bus_m_awqos),
        .m_axi_awvalid (bus_m_awvalid),      .m_axi_awready (bus_m_awready),
        .m_axi_wdata   (bus_m_wdata),        .m_axi_wstrb   (bus_m_wstrb),
        .m_axi_wlast   (bus_m_wlast),        .m_axi_wvalid  (bus_m_wvalid),
        .m_axi_wready  (bus_m_wready),       .m_axi_bid     (bus_m_bid),
        .m_axi_bresp   (bus_m_bresp),        .m_axi_bvalid  (bus_m_bvalid),
        .m_axi_bready  (bus_m_bready),       .m_axi_arid    (bus_m_arid),
        .m_axi_araddr  (bus_m_araddr),       .m_axi_arlen   (bus_m_arlen),
        .m_axi_arsize  (bus_m_arsize),       .m_axi_arburst (bus_m_arburst),
        .m_axi_arlock  (bus_m_arlock),       .m_axi_arcache (bus_m_arcache),
        .m_axi_arprot  (bus_m_arprot),       .m_axi_arregion(),
        .m_axi_arqos   (bus_m_arqos),        .m_axi_arvalid (bus_m_arvalid),
        .m_axi_arready (bus_m_arready),      .m_axi_rid     (bus_m_rid),
        .m_axi_rdata   (bus_m_rdata),        .m_axi_rresp   (bus_m_rresp),
        .m_axi_rlast   (bus_m_rlast),        .m_axi_rvalid  (bus_m_rvalid),
        .m_axi_rready  (bus_m_rready)
    );

    // =========================================================================
    // 5. 例化 4 个 core_top, 接 crossbar 各端口
    //    每核 SI[i] = 自己 axi_m_mux 出口
    //    每核 IFB SI = crossbar MI[4+i] (跨核 push 入口)
    // =========================================================================
    genvar i;
    generate
        for (i = 0; i < NUM_CORES; i++) begin : gen_core
            core_top #(
                .NUM_COL(NUM_COL), .NUM_PE(NUM_PE),
                .DATA_WIDTH(DATA_WIDTH), .PSUM_WIDTH(PSUM_WIDTH),
                .WRF_DEPTH(WRF_DEPTH), .ARF_DEPTH(ARF_DEPTH), .PARF_DEPTH(PARF_DEPTH),
                .SRAM_DEPTH(SRAM_DEPTH), .WB_DEPTH(WB_DEPTH), .OFB_DEPTH(OFB_DEPTH),
                .CSR_ADDR_W(12), .CSR_DATA_W(CSR_DATA_W),
                .BUS_ADDR_W(BUS_ADDR_W), .BUS_DATA_W(BUS_DATA_W),
                .AXI_M_ID(AXI_M_ID), .AXI_M_WIDTH(AXI_M_WIDTH),
                .DMA_LEN_W(DMA_LEN_W),
                .RMT_ID_W(EXT_BUS_ID)
            ) u_core (
                .clk(clk), .rst_n(rst_n),
                // CSR
                .csr_awaddr (csr_m_awaddr [i*HOST_CSR_AW +: 12]),
                .csr_awvalid(csr_m_awvalid[i]), .csr_awready(csr_m_awready[i]),
                .csr_wdata  (csr_m_wdata  [i*CSR_DATA_W +: CSR_DATA_W]),
                .csr_wstrb  (csr_m_wstrb  [i*(CSR_DATA_W/8) +: (CSR_DATA_W/8)]),
                .csr_wvalid (csr_m_wvalid [i]), .csr_wready (csr_m_wready [i]),
                .csr_bresp  (csr_m_bresp  [i*2 +: 2]),
                .csr_bvalid (csr_m_bvalid [i]), .csr_bready (csr_m_bready [i]),
                .csr_araddr (csr_m_araddr [i*HOST_CSR_AW +: 12]),
                .csr_arvalid(csr_m_arvalid[i]), .csr_arready(csr_m_arready[i]),
                .csr_rdata  (csr_m_rdata  [i*CSR_DATA_W +: CSR_DATA_W]),
                .csr_rresp  (csr_m_rresp  [i*2 +: 2]),
                .csr_rvalid (csr_m_rvalid [i]), .csr_rready (csr_m_rready [i]),
                // SI 端 (核出口 → crossbar SI[i])
                .bus_awid   (bus_s_awid   [i*CORE_BUS_ID +: CORE_BUS_ID]),
                .bus_awaddr (bus_s_awaddr [i*BUS_ADDR_W  +: BUS_ADDR_W]),
                .bus_awlen  (bus_s_awlen  [i*8           +: 8]),
                .bus_awsize (bus_s_awsize [i*3           +: 3]),
                .bus_awburst(bus_s_awburst[i*2           +: 2]),
                .bus_awlock (bus_s_awlock [i*1           +: 1]),
                .bus_awcache(bus_s_awcache[i*4           +: 4]),
                .bus_awprot (bus_s_awprot [i*3           +: 3]),
                .bus_awqos  (bus_s_awqos  [i*4           +: 4]),
                .bus_awvalid(bus_s_awvalid[i]), .bus_awready(bus_s_awready[i]),
                .bus_wdata  (bus_s_wdata  [i*BUS_DATA_W +: BUS_DATA_W]),
                .bus_wstrb  (bus_s_wstrb  [i*(BUS_DATA_W/8) +: (BUS_DATA_W/8)]),
                .bus_wlast  (bus_s_wlast  [i]),
                .bus_wvalid (bus_s_wvalid [i]), .bus_wready (bus_s_wready [i]),
                .bus_bid    (bus_s_bid    [i*CORE_BUS_ID +: CORE_BUS_ID]),
                .bus_bresp  (bus_s_bresp  [i*2 +: 2]),
                .bus_bvalid (bus_s_bvalid [i]), .bus_bready (bus_s_bready [i]),
                .bus_arid   (bus_s_arid   [i*CORE_BUS_ID +: CORE_BUS_ID]),
                .bus_araddr (bus_s_araddr [i*BUS_ADDR_W +: BUS_ADDR_W]),
                .bus_arlen  (bus_s_arlen  [i*8           +: 8]),
                .bus_arsize (bus_s_arsize [i*3           +: 3]),
                .bus_arburst(bus_s_arburst[i*2           +: 2]),
                .bus_arlock (bus_s_arlock [i*1           +: 1]),
                .bus_arcache(bus_s_arcache[i*4           +: 4]),
                .bus_arprot (bus_s_arprot [i*3           +: 3]),
                .bus_arqos  (bus_s_arqos  [i*4           +: 4]),
                .bus_arvalid(bus_s_arvalid[i]), .bus_arready(bus_s_arready[i]),
                .bus_rid    (bus_s_rid    [i*CORE_BUS_ID +: CORE_BUS_ID]),
                .bus_rdata  (bus_s_rdata  [i*BUS_DATA_W +: BUS_DATA_W]),
                .bus_rresp  (bus_s_rresp  [i*2 +: 2]),
                .bus_rlast  (bus_s_rlast  [i]),
                .bus_rvalid (bus_s_rvalid [i]), .bus_rready (bus_s_rready [i]),
                // Remote IFB SI (crossbar MI[4+i] → 本核 IFB)
                .rmt_ifb_awid    (bus_m_awid   [(NUM_DDR+i)*EXT_BUS_ID +: EXT_BUS_ID]),
                .rmt_ifb_awaddr  (bus_m_awaddr [(NUM_DDR+i)*BUS_ADDR_W +: BUS_ADDR_W]),
                .rmt_ifb_awlen   (bus_m_awlen  [(NUM_DDR+i)*8           +: 8]),
                .rmt_ifb_awsize  (bus_m_awsize [(NUM_DDR+i)*3           +: 3]),
                .rmt_ifb_awburst (bus_m_awburst[(NUM_DDR+i)*2           +: 2]),
                .rmt_ifb_awlock  (bus_m_awlock [(NUM_DDR+i)*1           +: 1]),
                .rmt_ifb_awcache (bus_m_awcache[(NUM_DDR+i)*4           +: 4]),
                .rmt_ifb_awprot  (bus_m_awprot [(NUM_DDR+i)*3           +: 3]),
                .rmt_ifb_awqos   (bus_m_awqos  [(NUM_DDR+i)*4           +: 4]),
                .rmt_ifb_awvalid (bus_m_awvalid[NUM_DDR+i]),
                .rmt_ifb_awready (bus_m_awready[NUM_DDR+i]),
                .rmt_ifb_wdata   (bus_m_wdata  [(NUM_DDR+i)*BUS_DATA_W +: BUS_DATA_W]),
                .rmt_ifb_wstrb   (bus_m_wstrb  [(NUM_DDR+i)*(BUS_DATA_W/8) +: (BUS_DATA_W/8)]),
                .rmt_ifb_wlast   (bus_m_wlast  [NUM_DDR+i]),
                .rmt_ifb_wvalid  (bus_m_wvalid [NUM_DDR+i]),
                .rmt_ifb_wready  (bus_m_wready [NUM_DDR+i]),
                .rmt_ifb_bid     (bus_m_bid    [(NUM_DDR+i)*EXT_BUS_ID +: EXT_BUS_ID]),
                .rmt_ifb_bresp   (bus_m_bresp  [(NUM_DDR+i)*2           +: 2]),
                .rmt_ifb_bvalid  (bus_m_bvalid [NUM_DDR+i]),
                .rmt_ifb_bready  (bus_m_bready [NUM_DDR+i]),
                .rmt_ifb_arid    (bus_m_arid   [(NUM_DDR+i)*EXT_BUS_ID +: EXT_BUS_ID]),
                .rmt_ifb_araddr  (bus_m_araddr [(NUM_DDR+i)*BUS_ADDR_W +: BUS_ADDR_W]),
                .rmt_ifb_arlen   (bus_m_arlen  [(NUM_DDR+i)*8           +: 8]),
                .rmt_ifb_arsize  (bus_m_arsize [(NUM_DDR+i)*3           +: 3]),
                .rmt_ifb_arburst (bus_m_arburst[(NUM_DDR+i)*2           +: 2]),
                .rmt_ifb_arlock  (bus_m_arlock [(NUM_DDR+i)*1           +: 1]),
                .rmt_ifb_arcache (bus_m_arcache[(NUM_DDR+i)*4           +: 4]),
                .rmt_ifb_arprot  (bus_m_arprot [(NUM_DDR+i)*3           +: 3]),
                .rmt_ifb_arqos   (bus_m_arqos  [(NUM_DDR+i)*4           +: 4]),
                .rmt_ifb_arvalid (bus_m_arvalid[NUM_DDR+i]),
                .rmt_ifb_arready (bus_m_arready[NUM_DDR+i]),
                .rmt_ifb_rid     (bus_m_rid    [(NUM_DDR+i)*EXT_BUS_ID +: EXT_BUS_ID]),
                .rmt_ifb_rdata   (bus_m_rdata  [(NUM_DDR+i)*BUS_DATA_W +: BUS_DATA_W]),
                .rmt_ifb_rresp   (bus_m_rresp  [(NUM_DDR+i)*2           +: 2]),
                .rmt_ifb_rlast   (bus_m_rlast  [NUM_DDR+i]),
                .rmt_ifb_rvalid  (bus_m_rvalid [NUM_DDR+i]),
                .rmt_ifb_rready  (bus_m_rready [NUM_DDR+i]),
                // TB 后门
                .ifb_we_ext   (1'b0), .ifb_waddr_ext('0), .ifb_wdata_ext('0),
                .wb_we_ext    (1'b0), .wb_waddr_ext ('0), .wb_wdata_ext ('0),
                .ofb_re_ext   (1'b0), .ofb_raddr_ext('0), .ofb_rdata_ext(),
                .psum_out_vec (),
                .done         (done_per_core[i])
            );
        end
    endgenerate

endmodule
