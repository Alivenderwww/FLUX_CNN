`timescale 1ns/1ps

// =============================================================================
// multicore_top.sv  --  N-core CNN accelerator wrapper (M1: shared DDR mode)
//
// 拓扑 (M1, 无 mesh, 共享 DDR):
//                                                    ┌─→ axi_2to1 IP ──→ ext AXI4 Master
//   host AXI-Lite ─→ axi_lite_1toN IP ──┬→ Core 0 ──┤   (NUM_CORES SI : 1 MI)
//                                       └→ Core 1 ──┘
//                   (1 SI : N MI, [13:12] 选核)
//
// 地址布局:
//   AXI-Lite (host 端): 14-bit, [13:12]=core_id, [11:0]=cfg_regs offset (12 bit)
//   AXI4 ext: 32-bit, 全核共享同一 DDR 视图 (host 通过 cfg 给各核 IDMA/ODMA base 区分)
//
// M2 (将来) 扩展点:
//   - axi_2to1 → axi_NtoM, M = N+1 (DDR + N 个核 IFB slave)
//   - 核内 IFB SRAM 改成 dual-write 端口 (本核 IDMA + 远端 AXI4 slave)
//   - 编译器在 ODMA_DST_BASE 写跨核 IFB 地址而非 DDR
//
// 复位: rst_n 同步分发, 所有核 + IP 同步复位.
// =============================================================================
`include "flux_cnn_params.svh"

module multicore_top #(
    parameter int NUM_CORES   = 2,           // 核数 (跟 IP gen 一致)
    // 默认值来自 flux_cnn_params.svh (params.py codegen)
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

    // 派生
    localparam int CORE_ID_W   = (NUM_CORES <= 1) ? 1 : $clog2(NUM_CORES),
    localparam int HOST_CSR_AW = 12 + CORE_ID_W,   // host 端 AXI-Lite 地址位宽
    localparam int CORE_BUS_ID = AXI_M_ID + AXI_M_WIDTH,   // 核内 bus_*id 位宽 (=4)
    localparam int EXT_BUS_ID  = CORE_BUS_ID + CORE_ID_W   // crossbar 出口 ID = 核内 ID + 核 tag
)(
    input  logic                                 clk,
    input  logic                                 rst_n,

    // ---- AXI-Lite Slave (host 配置, 14-bit 地址 = core_id[1:0] + reg[11:0]) ----
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

    // ---- 外部 AXI4 Master (聚合后单口去 DDR) ----
    output logic [EXT_BUS_ID-1:0]                bus_awid,
    output logic [BUS_ADDR_W-1:0]                bus_awaddr,
    output logic [7:0]                           bus_awlen,
    output logic [1:0]                           bus_awburst,
    output logic                                 bus_awvalid,
    input  logic                                 bus_awready,

    output logic [BUS_DATA_W-1:0]                bus_wdata,
    output logic [BUS_DATA_W/8-1:0]              bus_wstrb,
    output logic                                 bus_wlast,
    output logic                                 bus_wvalid,
    input  logic                                 bus_wready,

    input  logic [EXT_BUS_ID-1:0]                bus_bid,
    input  logic [1:0]                           bus_bresp,
    input  logic                                 bus_bvalid,
    output logic                                 bus_bready,

    output logic [EXT_BUS_ID-1:0]                bus_arid,
    output logic [BUS_ADDR_W-1:0]                bus_araddr,
    output logic [7:0]                           bus_arlen,
    output logic [1:0]                           bus_arburst,
    output logic                                 bus_arvalid,
    input  logic                                 bus_arready,

    input  logic [EXT_BUS_ID-1:0]                bus_rid,
    input  logic [BUS_DATA_W-1:0]                bus_rdata,
    input  logic [1:0]                           bus_rresp,
    input  logic                                 bus_rlast,
    input  logic                                 bus_rvalid,
    output logic                                 bus_rready,

    // ---- 各核 done (host poll / IRQ aggregation 由顶层处理) ----
    output logic [NUM_CORES-1:0]                 done_per_core
);

    logic aresetn;
    assign aresetn = rst_n;

    // =========================================================================
    // 1. AXI-Lite 1:N crossbar (Xilinx IP) -- host CSR 分发
    //    M00 base=0x0000, M01 base=0x1000, ..., 每个 12-bit 区
    // =========================================================================
    // IP MI 端 awaddr/araddr 总宽 = HOST_CSR_AW × N (per-MI 13-bit, 低 12 bit = reg offset)
    logic [NUM_CORES*HOST_CSR_AW-1:0]   csr_m_awaddr;
    logic [NUM_CORES*3-1:0]             csr_m_awprot;
    logic [NUM_CORES-1:0]               csr_m_awvalid;
    logic [NUM_CORES-1:0]               csr_m_awready;
    logic [NUM_CORES*CSR_DATA_W-1:0]    csr_m_wdata;
    logic [NUM_CORES*(CSR_DATA_W/8)-1:0]csr_m_wstrb;
    logic [NUM_CORES-1:0]               csr_m_wvalid;
    logic [NUM_CORES-1:0]               csr_m_wready;
    logic [NUM_CORES*2-1:0]             csr_m_bresp;
    logic [NUM_CORES-1:0]               csr_m_bvalid;
    logic [NUM_CORES-1:0]               csr_m_bready;
    logic [NUM_CORES*HOST_CSR_AW-1:0]   csr_m_araddr;
    logic [NUM_CORES*3-1:0]             csr_m_arprot;
    logic [NUM_CORES-1:0]               csr_m_arvalid;
    logic [NUM_CORES-1:0]               csr_m_arready;
    logic [NUM_CORES*CSR_DATA_W-1:0]    csr_m_rdata;
    logic [NUM_CORES*2-1:0]             csr_m_rresp;
    logic [NUM_CORES-1:0]               csr_m_rvalid;
    logic [NUM_CORES-1:0]               csr_m_rready;

    // axi_lite_1toN IP: NUM_CORES=2 → axi_lite_1to2, NUM_CORES=4 → axi_lite_1to4
    // (generate if 切换 IP 模块名 — IP 模块端口签名一致, 只 NUM_MI 不同)
    generate
        if (NUM_CORES == 2) begin : g_csr_xbar_2
            axi_lite_1to2 u_csr_xbar (
                .aclk          (clk),
                .aresetn       (aresetn),
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
        end else if (NUM_CORES == 4) begin : g_csr_xbar_4
            axi_lite_1to4 u_csr_xbar (
                .aclk          (clk),
                .aresetn       (aresetn),
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
        end
    endgenerate

    // =========================================================================
    // 2. AXI4 N:(N+1) crossbar (Xilinx IP) -- M2 多 MI 拓扑
    //    SI 端: N 个核 master, ID 宽度 = CORE_BUS_ID (4 bit)
    //    MI 端: MI[0]=DDR, MI[1..N]=各核 IFB SI (跨核 push 入口)
    //           ID 宽度 = EXT_BUS_ID = CORE_BUS_ID + CORE_ID_W (5 bit @ N=2)
    //    地址映射: DDR 0x0000_0000-0x7FFF_FFFF; Core[i] IFB @ 0x80000000+i*0x10000000
    // =========================================================================
    localparam int NUM_MI_BUS = NUM_CORES + 1;
    logic [NUM_CORES*CORE_BUS_ID-1:0]   bus_s_awid;
    logic [NUM_CORES*BUS_ADDR_W-1:0]    bus_s_awaddr;
    logic [NUM_CORES*8-1:0]             bus_s_awlen;
    logic [NUM_CORES*3-1:0]             bus_s_awsize;     // 全核 = 3'b100 (16 byte)
    logic [NUM_CORES*2-1:0]             bus_s_awburst;
    logic [NUM_CORES*1-1:0]             bus_s_awlock;
    logic [NUM_CORES*4-1:0]             bus_s_awcache;
    logic [NUM_CORES*3-1:0]             bus_s_awprot;
    logic [NUM_CORES*4-1:0]             bus_s_awqos;
    logic [NUM_CORES-1:0]               bus_s_awvalid;
    logic [NUM_CORES-1:0]               bus_s_awready;
    logic [NUM_CORES*BUS_DATA_W-1:0]    bus_s_wdata;
    logic [NUM_CORES*(BUS_DATA_W/8)-1:0]bus_s_wstrb;
    logic [NUM_CORES-1:0]               bus_s_wlast;
    logic [NUM_CORES-1:0]               bus_s_wvalid;
    logic [NUM_CORES-1:0]               bus_s_wready;
    logic [NUM_CORES*CORE_BUS_ID-1:0]   bus_s_bid;
    logic [NUM_CORES*2-1:0]             bus_s_bresp;
    logic [NUM_CORES-1:0]               bus_s_bvalid;
    logic [NUM_CORES-1:0]               bus_s_bready;
    logic [NUM_CORES*CORE_BUS_ID-1:0]   bus_s_arid;
    logic [NUM_CORES*BUS_ADDR_W-1:0]    bus_s_araddr;
    logic [NUM_CORES*8-1:0]             bus_s_arlen;
    logic [NUM_CORES*3-1:0]             bus_s_arsize;
    logic [NUM_CORES*2-1:0]             bus_s_arburst;
    logic [NUM_CORES*1-1:0]             bus_s_arlock;
    logic [NUM_CORES*4-1:0]             bus_s_arcache;
    logic [NUM_CORES*3-1:0]             bus_s_arprot;
    logic [NUM_CORES*4-1:0]             bus_s_arqos;
    logic [NUM_CORES-1:0]               bus_s_arvalid;
    logic [NUM_CORES-1:0]               bus_s_arready;
    logic [NUM_CORES*CORE_BUS_ID-1:0]   bus_s_rid;
    logic [NUM_CORES*BUS_DATA_W-1:0]    bus_s_rdata;
    logic [NUM_CORES*2-1:0]             bus_s_rresp;
    logic [NUM_CORES-1:0]               bus_s_rlast;
    logic [NUM_CORES-1:0]               bus_s_rvalid;
    logic [NUM_CORES-1:0]               bus_s_rready;

    // crossbar SI 端 ID 宽 = ID_WIDTH = EXT_BUS_ID (pad CORE_BUS_ID 比特上加 0).
    // Vivado axi_crossbar IP 把 SI 的 ID_WIDTH 直接当 SI ID 宽 (没有自动扩展),
    // 所以 multicore_top 这边显式 pad 跟 IP 端口对齐.
    logic [NUM_CORES*EXT_BUS_ID-1:0]    bus_s_awid_padded, bus_s_arid_padded;
    logic [NUM_CORES*EXT_BUS_ID-1:0]    bus_s_bid_padded,  bus_s_rid_padded;
    genvar gp;
    generate
        for (gp = 0; gp < NUM_CORES; gp++) begin : g_si_id_pad
            assign bus_s_awid_padded[gp*EXT_BUS_ID +: EXT_BUS_ID] =
                {{(EXT_BUS_ID-CORE_BUS_ID){1'b0}}, bus_s_awid[gp*CORE_BUS_ID +: CORE_BUS_ID]};
            assign bus_s_arid_padded[gp*EXT_BUS_ID +: EXT_BUS_ID] =
                {{(EXT_BUS_ID-CORE_BUS_ID){1'b0}}, bus_s_arid[gp*CORE_BUS_ID +: CORE_BUS_ID]};
            // IP 返回的 BID/RID 含 SI tag (跟原 ID 一起回来), 取低 CORE_BUS_ID 位还给核
            assign bus_s_bid[gp*CORE_BUS_ID +: CORE_BUS_ID] =
                bus_s_bid_padded[gp*EXT_BUS_ID +: CORE_BUS_ID];
            assign bus_s_rid[gp*CORE_BUS_ID +: CORE_BUS_ID] =
                bus_s_rid_padded[gp*EXT_BUS_ID +: CORE_BUS_ID];
        end
    endgenerate

    // crossbar MI 端 packed array: [NUM_MI_BUS * field_width]
    //   MI[0]   = DDR (接 multicore_top 顶层 bus_*)
    //   MI[i+1] = Core[i] IFB (接 gen_core[i].u_core.rmt_ifb_*)
    logic [NUM_MI_BUS*EXT_BUS_ID-1:0]   bus_m_awid;
    logic [NUM_MI_BUS*BUS_ADDR_W-1:0]   bus_m_awaddr;
    logic [NUM_MI_BUS*8-1:0]            bus_m_awlen;
    logic [NUM_MI_BUS*3-1:0]            bus_m_awsize;
    logic [NUM_MI_BUS*2-1:0]            bus_m_awburst;
    logic [NUM_MI_BUS*1-1:0]            bus_m_awlock;
    logic [NUM_MI_BUS*4-1:0]            bus_m_awcache;
    logic [NUM_MI_BUS*3-1:0]            bus_m_awprot;
    logic [NUM_MI_BUS*4-1:0]            bus_m_awqos;
    logic [NUM_MI_BUS-1:0]              bus_m_awvalid;
    logic [NUM_MI_BUS-1:0]              bus_m_awready;
    logic [NUM_MI_BUS*BUS_DATA_W-1:0]   bus_m_wdata;
    logic [NUM_MI_BUS*(BUS_DATA_W/8)-1:0] bus_m_wstrb;
    logic [NUM_MI_BUS-1:0]              bus_m_wlast;
    logic [NUM_MI_BUS-1:0]              bus_m_wvalid;
    logic [NUM_MI_BUS-1:0]              bus_m_wready;
    logic [NUM_MI_BUS*EXT_BUS_ID-1:0]   bus_m_bid;
    logic [NUM_MI_BUS*2-1:0]            bus_m_bresp;
    logic [NUM_MI_BUS-1:0]              bus_m_bvalid;
    logic [NUM_MI_BUS-1:0]              bus_m_bready;
    logic [NUM_MI_BUS*EXT_BUS_ID-1:0]   bus_m_arid;
    logic [NUM_MI_BUS*BUS_ADDR_W-1:0]   bus_m_araddr;
    logic [NUM_MI_BUS*8-1:0]            bus_m_arlen;
    logic [NUM_MI_BUS*3-1:0]            bus_m_arsize;
    logic [NUM_MI_BUS*2-1:0]            bus_m_arburst;
    logic [NUM_MI_BUS*1-1:0]            bus_m_arlock;
    logic [NUM_MI_BUS*4-1:0]            bus_m_arcache;
    logic [NUM_MI_BUS*3-1:0]            bus_m_arprot;
    logic [NUM_MI_BUS*4-1:0]            bus_m_arqos;
    logic [NUM_MI_BUS-1:0]              bus_m_arvalid;
    logic [NUM_MI_BUS-1:0]              bus_m_arready;
    logic [NUM_MI_BUS*EXT_BUS_ID-1:0]   bus_m_rid;
    logic [NUM_MI_BUS*BUS_DATA_W-1:0]   bus_m_rdata;
    logic [NUM_MI_BUS*2-1:0]            bus_m_rresp;
    logic [NUM_MI_BUS-1:0]              bus_m_rlast;
    logic [NUM_MI_BUS-1:0]              bus_m_rvalid;
    logic [NUM_MI_BUS-1:0]              bus_m_rready;

    // MI[0] = DDR: 直接连 multicore_top 顶层 bus_* 端口
    assign bus_awid     = bus_m_awid   [0*EXT_BUS_ID +: EXT_BUS_ID];
    assign bus_awaddr   = bus_m_awaddr [0*BUS_ADDR_W +: BUS_ADDR_W];
    assign bus_awlen    = bus_m_awlen  [0*8           +: 8];
    assign bus_awburst  = bus_m_awburst[0*2           +: 2];
    assign bus_awvalid  = bus_m_awvalid[0];
    assign bus_m_awready[0] = bus_awready;
    assign bus_wdata    = bus_m_wdata  [0*BUS_DATA_W +: BUS_DATA_W];
    assign bus_wstrb    = bus_m_wstrb  [0*(BUS_DATA_W/8) +: (BUS_DATA_W/8)];
    assign bus_wlast    = bus_m_wlast  [0];
    assign bus_wvalid   = bus_m_wvalid [0];
    assign bus_m_wready[0]  = bus_wready;
    assign bus_m_bid   [0*EXT_BUS_ID +: EXT_BUS_ID] = bus_bid;
    assign bus_m_bresp [0*2           +: 2]         = bus_bresp;
    assign bus_m_bvalid[0]  = bus_bvalid;
    assign bus_bready   = bus_m_bready[0];
    assign bus_arid     = bus_m_arid   [0*EXT_BUS_ID +: EXT_BUS_ID];
    assign bus_araddr   = bus_m_araddr [0*BUS_ADDR_W +: BUS_ADDR_W];
    assign bus_arlen    = bus_m_arlen  [0*8           +: 8];
    assign bus_arburst  = bus_m_arburst[0*2           +: 2];
    assign bus_arvalid  = bus_m_arvalid[0];
    assign bus_m_arready[0] = bus_arready;
    assign bus_m_rid   [0*EXT_BUS_ID +: EXT_BUS_ID] = bus_rid;
    assign bus_m_rdata [0*BUS_DATA_W +: BUS_DATA_W] = bus_rdata;
    assign bus_m_rresp [0*2           +: 2]         = bus_rresp;
    assign bus_m_rlast [0]  = bus_rlast;
    assign bus_m_rvalid[0]  = bus_rvalid;
    assign bus_rready   = bus_m_rready[0];

    // crossbar 实例化 (NUM_SI=NUM_CORES, NUM_MI=NUM_CORES+1)
    // NUM_CORES=2 → axi_2to3, NUM_CORES=4 → axi_4to5 (generate if 切 IP 模块名)
    generate
        if (NUM_CORES == 2) begin : g_bus_xbar_2
            axi_2to3 u_bus_xbar (
                .aclk          (clk),    .aresetn       (aresetn),
                .s_axi_awid    (bus_s_awid_padded),    .s_axi_awaddr  (bus_s_awaddr),
                .s_axi_awlen   (bus_s_awlen),   .s_axi_awsize  (bus_s_awsize),
                .s_axi_awburst (bus_s_awburst), .s_axi_awlock  (bus_s_awlock),
                .s_axi_awcache (bus_s_awcache), .s_axi_awprot  (bus_s_awprot),
                .s_axi_awqos   (bus_s_awqos),   .s_axi_awvalid (bus_s_awvalid),
                .s_axi_awready (bus_s_awready), .s_axi_wdata   (bus_s_wdata),
                .s_axi_wstrb   (bus_s_wstrb),   .s_axi_wlast   (bus_s_wlast),
                .s_axi_wvalid  (bus_s_wvalid),  .s_axi_wready  (bus_s_wready),
                .s_axi_bid     (bus_s_bid_padded),     .s_axi_bresp   (bus_s_bresp),
                .s_axi_bvalid  (bus_s_bvalid),  .s_axi_bready  (bus_s_bready),
                .s_axi_arid    (bus_s_arid_padded),    .s_axi_araddr  (bus_s_araddr),
                .s_axi_arlen   (bus_s_arlen),   .s_axi_arsize  (bus_s_arsize),
                .s_axi_arburst (bus_s_arburst), .s_axi_arlock  (bus_s_arlock),
                .s_axi_arcache (bus_s_arcache), .s_axi_arprot  (bus_s_arprot),
                .s_axi_arqos   (bus_s_arqos),   .s_axi_arvalid (bus_s_arvalid),
                .s_axi_arready (bus_s_arready), .s_axi_rid     (bus_s_rid_padded),
                .s_axi_rdata   (bus_s_rdata),   .s_axi_rresp   (bus_s_rresp),
                .s_axi_rlast   (bus_s_rlast),   .s_axi_rvalid  (bus_s_rvalid),
                .s_axi_rready  (bus_s_rready),
                .m_axi_awid    (bus_m_awid),    .m_axi_awaddr  (bus_m_awaddr),
                .m_axi_awlen   (bus_m_awlen),   .m_axi_awsize  (bus_m_awsize),
                .m_axi_awburst (bus_m_awburst), .m_axi_awlock  (bus_m_awlock),
                .m_axi_awcache (bus_m_awcache), .m_axi_awprot  (bus_m_awprot),
                .m_axi_awregion(),              .m_axi_awqos   (bus_m_awqos),
                .m_axi_awvalid (bus_m_awvalid), .m_axi_awready (bus_m_awready),
                .m_axi_wdata   (bus_m_wdata),   .m_axi_wstrb   (bus_m_wstrb),
                .m_axi_wlast   (bus_m_wlast),   .m_axi_wvalid  (bus_m_wvalid),
                .m_axi_wready  (bus_m_wready),  .m_axi_bid     (bus_m_bid),
                .m_axi_bresp   (bus_m_bresp),   .m_axi_bvalid  (bus_m_bvalid),
                .m_axi_bready  (bus_m_bready),  .m_axi_arid    (bus_m_arid),
                .m_axi_araddr  (bus_m_araddr),  .m_axi_arlen   (bus_m_arlen),
                .m_axi_arsize  (bus_m_arsize),  .m_axi_arburst (bus_m_arburst),
                .m_axi_arlock  (bus_m_arlock),  .m_axi_arcache (bus_m_arcache),
                .m_axi_arprot  (bus_m_arprot),  .m_axi_arregion(),
                .m_axi_arqos   (bus_m_arqos),   .m_axi_arvalid (bus_m_arvalid),
                .m_axi_arready (bus_m_arready), .m_axi_rid     (bus_m_rid),
                .m_axi_rdata   (bus_m_rdata),   .m_axi_rresp   (bus_m_rresp),
                .m_axi_rlast   (bus_m_rlast),   .m_axi_rvalid  (bus_m_rvalid),
                .m_axi_rready  (bus_m_rready)
            );
        end else if (NUM_CORES == 4) begin : g_bus_xbar_4
            axi_4to5 u_bus_xbar (
                .aclk          (clk),    .aresetn       (aresetn),
                .s_axi_awid    (bus_s_awid_padded),    .s_axi_awaddr  (bus_s_awaddr),
                .s_axi_awlen   (bus_s_awlen),   .s_axi_awsize  (bus_s_awsize),
                .s_axi_awburst (bus_s_awburst), .s_axi_awlock  (bus_s_awlock),
                .s_axi_awcache (bus_s_awcache), .s_axi_awprot  (bus_s_awprot),
                .s_axi_awqos   (bus_s_awqos),   .s_axi_awvalid (bus_s_awvalid),
                .s_axi_awready (bus_s_awready), .s_axi_wdata   (bus_s_wdata),
                .s_axi_wstrb   (bus_s_wstrb),   .s_axi_wlast   (bus_s_wlast),
                .s_axi_wvalid  (bus_s_wvalid),  .s_axi_wready  (bus_s_wready),
                .s_axi_bid     (bus_s_bid_padded),     .s_axi_bresp   (bus_s_bresp),
                .s_axi_bvalid  (bus_s_bvalid),  .s_axi_bready  (bus_s_bready),
                .s_axi_arid    (bus_s_arid_padded),    .s_axi_araddr  (bus_s_araddr),
                .s_axi_arlen   (bus_s_arlen),   .s_axi_arsize  (bus_s_arsize),
                .s_axi_arburst (bus_s_arburst), .s_axi_arlock  (bus_s_arlock),
                .s_axi_arcache (bus_s_arcache), .s_axi_arprot  (bus_s_arprot),
                .s_axi_arqos   (bus_s_arqos),   .s_axi_arvalid (bus_s_arvalid),
                .s_axi_arready (bus_s_arready), .s_axi_rid     (bus_s_rid_padded),
                .s_axi_rdata   (bus_s_rdata),   .s_axi_rresp   (bus_s_rresp),
                .s_axi_rlast   (bus_s_rlast),   .s_axi_rvalid  (bus_s_rvalid),
                .s_axi_rready  (bus_s_rready),
                .m_axi_awid    (bus_m_awid),    .m_axi_awaddr  (bus_m_awaddr),
                .m_axi_awlen   (bus_m_awlen),   .m_axi_awsize  (bus_m_awsize),
                .m_axi_awburst (bus_m_awburst), .m_axi_awlock  (bus_m_awlock),
                .m_axi_awcache (bus_m_awcache), .m_axi_awprot  (bus_m_awprot),
                .m_axi_awregion(),              .m_axi_awqos   (bus_m_awqos),
                .m_axi_awvalid (bus_m_awvalid), .m_axi_awready (bus_m_awready),
                .m_axi_wdata   (bus_m_wdata),   .m_axi_wstrb   (bus_m_wstrb),
                .m_axi_wlast   (bus_m_wlast),   .m_axi_wvalid  (bus_m_wvalid),
                .m_axi_wready  (bus_m_wready),  .m_axi_bid     (bus_m_bid),
                .m_axi_bresp   (bus_m_bresp),   .m_axi_bvalid  (bus_m_bvalid),
                .m_axi_bready  (bus_m_bready),  .m_axi_arid    (bus_m_arid),
                .m_axi_araddr  (bus_m_araddr),  .m_axi_arlen   (bus_m_arlen),
                .m_axi_arsize  (bus_m_arsize),  .m_axi_arburst (bus_m_arburst),
                .m_axi_arlock  (bus_m_arlock),  .m_axi_arcache (bus_m_arcache),
                .m_axi_arprot  (bus_m_arprot),  .m_axi_arregion(),
                .m_axi_arqos   (bus_m_arqos),   .m_axi_arvalid (bus_m_arvalid),
                .m_axi_arready (bus_m_arready), .m_axi_rid     (bus_m_rid),
                .m_axi_rdata   (bus_m_rdata),   .m_axi_rresp   (bus_m_rresp),
                .m_axi_rlast   (bus_m_rlast),   .m_axi_rvalid  (bus_m_rvalid),
                .m_axi_rready  (bus_m_rready)
            );
        end
    endgenerate

    // =========================================================================
    // 3. 例化 NUM_CORES 个 core_top, 接 crossbar 各端口
    // =========================================================================
    genvar i;
    generate
        for (i = 0; i < NUM_CORES; i++) begin : gen_core
            // 注: 核内 axi_dm IP + axi_m_mux 现在都把 size/lock/cache/prot/qos
            // forward 出来, 直接接 SI 口. 之前在这里 assign 常量是因为 axi_m_mux
            // 没 forward, 经 axi_2to1 IP 时悬空 'X' 让 awready 锁死.
            core_top #(
                .NUM_COL(NUM_COL), .NUM_PE(NUM_PE),
                .DATA_WIDTH(DATA_WIDTH), .PSUM_WIDTH(PSUM_WIDTH),
                .WRF_DEPTH(WRF_DEPTH), .ARF_DEPTH(ARF_DEPTH), .PARF_DEPTH(PARF_DEPTH),
                .SRAM_DEPTH(SRAM_DEPTH), .WB_DEPTH(WB_DEPTH), .OFB_DEPTH(OFB_DEPTH),
                .CSR_ADDR_W(12), .CSR_DATA_W(CSR_DATA_W),
                .BUS_ADDR_W(BUS_ADDR_W), .BUS_DATA_W(BUS_DATA_W),
                .AXI_M_ID(AXI_M_ID), .AXI_M_WIDTH(AXI_M_WIDTH),
                .DMA_LEN_W(DMA_LEN_W),
                .RMT_ID_W(EXT_BUS_ID)        // 跨核 IFB AXI4 SI ID 宽 = crossbar MI ID 宽
            ) u_core (
                .clk(clk), .rst_n(rst_n),

                // AXI-Lite slave (来自 csr_xbar 的 MI[i])
                .csr_awaddr (csr_m_awaddr [i*HOST_CSR_AW +: 12]),
                .csr_awvalid(csr_m_awvalid[i]),
                .csr_awready(csr_m_awready[i]),
                .csr_wdata  (csr_m_wdata  [i*CSR_DATA_W +: CSR_DATA_W]),
                .csr_wstrb  (csr_m_wstrb  [i*(CSR_DATA_W/8) +: (CSR_DATA_W/8)]),
                .csr_wvalid (csr_m_wvalid [i]),
                .csr_wready (csr_m_wready [i]),
                .csr_bresp  (csr_m_bresp  [i*2 +: 2]),
                .csr_bvalid (csr_m_bvalid [i]),
                .csr_bready (csr_m_bready [i]),
                .csr_araddr (csr_m_araddr [i*HOST_CSR_AW +: 12]),
                .csr_arvalid(csr_m_arvalid[i]),
                .csr_arready(csr_m_arready[i]),
                .csr_rdata  (csr_m_rdata  [i*CSR_DATA_W +: CSR_DATA_W]),
                .csr_rresp  (csr_m_rresp  [i*2 +: 2]),
                .csr_rvalid (csr_m_rvalid [i]),
                .csr_rready (csr_m_rready [i]),

                // AXI4 master (送给 bus_xbar 的 SI[i])
                .bus_awid   (bus_s_awid   [i*CORE_BUS_ID +: CORE_BUS_ID]),
                .bus_awaddr (bus_s_awaddr [i*BUS_ADDR_W  +: BUS_ADDR_W]),
                .bus_awlen  (bus_s_awlen  [i*8           +: 8]),
                .bus_awsize (bus_s_awsize [i*3           +: 3]),
                .bus_awburst(bus_s_awburst[i*2           +: 2]),
                .bus_awlock (bus_s_awlock [i*1           +: 1]),
                .bus_awcache(bus_s_awcache[i*4           +: 4]),
                .bus_awprot (bus_s_awprot [i*3           +: 3]),
                .bus_awqos  (bus_s_awqos  [i*4           +: 4]),
                .bus_awvalid(bus_s_awvalid[i]),
                .bus_awready(bus_s_awready[i]),
                .bus_wdata  (bus_s_wdata  [i*BUS_DATA_W  +: BUS_DATA_W]),
                .bus_wstrb  (bus_s_wstrb  [i*(BUS_DATA_W/8) +: (BUS_DATA_W/8)]),
                .bus_wlast  (bus_s_wlast  [i]),
                .bus_wvalid (bus_s_wvalid [i]),
                .bus_wready (bus_s_wready [i]),
                .bus_bid    (bus_s_bid    [i*CORE_BUS_ID +: CORE_BUS_ID]),
                .bus_bresp  (bus_s_bresp  [i*2           +: 2]),
                .bus_bvalid (bus_s_bvalid [i]),
                .bus_bready (bus_s_bready [i]),
                .bus_arid   (bus_s_arid   [i*CORE_BUS_ID +: CORE_BUS_ID]),
                .bus_araddr (bus_s_araddr [i*BUS_ADDR_W  +: BUS_ADDR_W]),
                .bus_arlen  (bus_s_arlen  [i*8           +: 8]),
                .bus_arsize (bus_s_arsize [i*3           +: 3]),
                .bus_arburst(bus_s_arburst[i*2           +: 2]),
                .bus_arlock (bus_s_arlock [i*1           +: 1]),
                .bus_arcache(bus_s_arcache[i*4           +: 4]),
                .bus_arprot (bus_s_arprot [i*3           +: 3]),
                .bus_arqos  (bus_s_arqos  [i*4           +: 4]),
                .bus_arvalid(bus_s_arvalid[i]),
                .bus_arready(bus_s_arready[i]),
                .bus_rid    (bus_s_rid    [i*CORE_BUS_ID +: CORE_BUS_ID]),
                .bus_rdata  (bus_s_rdata  [i*BUS_DATA_W  +: BUS_DATA_W]),
                .bus_rresp  (bus_s_rresp  [i*2           +: 2]),
                .bus_rlast  (bus_s_rlast  [i]),
                .bus_rvalid (bus_s_rvalid [i]),
                .bus_rready (bus_s_rready [i]),

                // M2: IFB AXI4 SI 接 crossbar MI[i+1]
                //     producer 远端核 ODMA 写 0x80000000+i*0x10000000 时被路由到这里
                .rmt_ifb_awid    (bus_m_awid   [(i+1)*EXT_BUS_ID +: EXT_BUS_ID]),
                .rmt_ifb_awaddr  (bus_m_awaddr [(i+1)*BUS_ADDR_W +: BUS_ADDR_W]),
                .rmt_ifb_awlen   (bus_m_awlen  [(i+1)*8           +: 8]),
                .rmt_ifb_awsize  (bus_m_awsize [(i+1)*3           +: 3]),
                .rmt_ifb_awburst (bus_m_awburst[(i+1)*2           +: 2]),
                .rmt_ifb_awlock  (bus_m_awlock [(i+1)*1           +: 1]),
                .rmt_ifb_awcache (bus_m_awcache[(i+1)*4           +: 4]),
                .rmt_ifb_awprot  (bus_m_awprot [(i+1)*3           +: 3]),
                .rmt_ifb_awqos   (bus_m_awqos  [(i+1)*4           +: 4]),
                .rmt_ifb_awvalid (bus_m_awvalid[i+1]),
                .rmt_ifb_awready (bus_m_awready[i+1]),
                .rmt_ifb_wdata   (bus_m_wdata  [(i+1)*BUS_DATA_W +: BUS_DATA_W]),
                .rmt_ifb_wstrb   (bus_m_wstrb  [(i+1)*(BUS_DATA_W/8) +: (BUS_DATA_W/8)]),
                .rmt_ifb_wlast   (bus_m_wlast  [i+1]),
                .rmt_ifb_wvalid  (bus_m_wvalid [i+1]),
                .rmt_ifb_wready  (bus_m_wready [i+1]),
                .rmt_ifb_bid     (bus_m_bid    [(i+1)*EXT_BUS_ID +: EXT_BUS_ID]),
                .rmt_ifb_bresp   (bus_m_bresp  [(i+1)*2           +: 2]),
                .rmt_ifb_bvalid  (bus_m_bvalid [i+1]),
                .rmt_ifb_bready  (bus_m_bready [i+1]),
                .rmt_ifb_arid    (bus_m_arid   [(i+1)*EXT_BUS_ID +: EXT_BUS_ID]),
                .rmt_ifb_araddr  (bus_m_araddr [(i+1)*BUS_ADDR_W +: BUS_ADDR_W]),
                .rmt_ifb_arlen   (bus_m_arlen  [(i+1)*8           +: 8]),
                .rmt_ifb_arsize  (bus_m_arsize [(i+1)*3           +: 3]),
                .rmt_ifb_arburst (bus_m_arburst[(i+1)*2           +: 2]),
                .rmt_ifb_arlock  (bus_m_arlock [(i+1)*1           +: 1]),
                .rmt_ifb_arcache (bus_m_arcache[(i+1)*4           +: 4]),
                .rmt_ifb_arprot  (bus_m_arprot [(i+1)*3           +: 3]),
                .rmt_ifb_arqos   (bus_m_arqos  [(i+1)*4           +: 4]),
                .rmt_ifb_arvalid (bus_m_arvalid[i+1]),
                .rmt_ifb_arready (bus_m_arready[i+1]),
                .rmt_ifb_rid     (bus_m_rid    [(i+1)*EXT_BUS_ID +: EXT_BUS_ID]),
                .rmt_ifb_rdata   (bus_m_rdata  [(i+1)*BUS_DATA_W +: BUS_DATA_W]),
                .rmt_ifb_rresp   (bus_m_rresp  [(i+1)*2           +: 2]),
                .rmt_ifb_rlast   (bus_m_rlast  [i+1]),
                .rmt_ifb_rvalid  (bus_m_rvalid [i+1]),
                .rmt_ifb_rready  (bus_m_rready [i+1]),

                // TB 后门 (M1 综合时全 0, sim 顶层 TB 接进来)
                .ifb_we_ext   (1'b0),
                .ifb_waddr_ext('0),
                .ifb_wdata_ext('0),
                .wb_we_ext    (1'b0),
                .wb_waddr_ext ('0),
                .wb_wdata_ext ('0),
                .ofb_re_ext   (1'b0),
                .ofb_raddr_ext('0),
                .ofb_rdata_ext(),

                .psum_out_vec (),

                .done         (done_per_core[i])
            );
        end
    endgenerate

endmodule
