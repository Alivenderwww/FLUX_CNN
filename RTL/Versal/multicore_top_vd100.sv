`timescale 1ns/1ps
`include "flux_cnn_params.svh"

// =============================================================================
// multicore_top_vd100.sv  --  VD100 demo 顶层 (Versal AI Edge VE2302)
//
// 跟 K325T 主线 multicore_top_smc.sv 的关键区别:
//   - NUM_CORES = 3 (vs 4), 适配 VE2302 PL 资源
//   - 不实例化 axi_smc_4to4 IP — Vitis BD 用 Versal NoC 替代, RTL 直接 expose
//     3 个 ConvCore 的 AXI master 到顶层端口
//   - 不实例化 axi_lite_1to4 IP — 顶层 expose 1 个 AXI-Lite slave (来自 PS GP),
//     内部用简单 1to3 demux 按 csr_addr[12:13] 选 core
//   - 顶层端口给 Vitis Block Design 接 NoC / DDRMC / CPM PCIe / GEM 跟 PS A72
//
// Vitis BD 连接方式 (在 Vitis Unified IDE 图形化配置):
//   PS A72 ─AXI HP─► NoC ─► DDRMC ←─ NoC ◄─AXI MM─ multicore_top_vd100 (3 master)
//   PS A72 ─AXI GP─► AXI Lite ─► multicore_top_vd100.csr_axil
//   PS A72 ◄─IRQ─── multicore_top_vd100.done_per_core
//
// AXI master 输出地址 (Vitis NoC 自动路由):
//   3 个 ConvCore 共享同一个 4 GB DDR4 地址空间, base 由 driver 决定
//   (跟 main 分支 SMC 4 mem 16MB×4 不同, VD100 用 PS DDR4 4GB 单 mem)
// =============================================================================

module multicore_top_vd100 #(
    parameter int NUM_CORES   = 3,
    parameter int CORE_ID_W   = $clog2(NUM_CORES),    // = 2
    parameter int BUS_ADDR_W  = `FLUX_BUS_ADDR_W,     // = 32
    parameter int BUS_DATA_W  = `FLUX_BUS_DATA_W,     // = 128
    parameter int CSR_DATA_W  = 32,
    parameter int CORE_BUS_ID = `FLUX_AXI_M_ID        // 内部 ConvCore 的 ID 宽度
)(
    input  logic                        clk,
    input  logic                        rst_n,

    // ======================================================================
    // host AXI-Lite slave (来自 PS A72 GP)
    //   addr[13:12] = core_id (3 个 core 各占 4KB CSR 空间, 总 12KB)
    //   addr[11:0]  = core 内部寄存器地址 (跟 main 分支 axi_lite_csr 一致)
    // ======================================================================
    input  logic [13:0]                 csr_axil_awaddr,
    input  logic                        csr_axil_awvalid,
    output logic                        csr_axil_awready,
    input  logic [CSR_DATA_W-1:0]       csr_axil_wdata,
    input  logic [CSR_DATA_W/8-1:0]     csr_axil_wstrb,
    input  logic                        csr_axil_wvalid,
    output logic                        csr_axil_wready,
    output logic [1:0]                  csr_axil_bresp,
    output logic                        csr_axil_bvalid,
    input  logic                        csr_axil_bready,
    input  logic [13:0]                 csr_axil_araddr,
    input  logic                        csr_axil_arvalid,
    output logic                        csr_axil_arready,
    output logic [CSR_DATA_W-1:0]       csr_axil_rdata,
    output logic [1:0]                  csr_axil_rresp,
    output logic                        csr_axil_rvalid,
    input  logic                        csr_axil_rready,

    // ======================================================================
    // 3 个 ConvCore AXI master (出 → Versal NoC → DDRMC)
    //   每个 master 是完整 AXI4-MM, 128-bit data (BUS_DATA_W)
    //   ID 宽度: CORE_BUS_ID (内部) + CORE_ID_W (core 区分) = 添加 core id 标记
    // ======================================================================
    output logic [NUM_CORES*(CORE_BUS_ID+CORE_ID_W)-1:0] m_axi_awid,
    output logic [NUM_CORES*BUS_ADDR_W-1:0]              m_axi_awaddr,
    output logic [NUM_CORES*8-1:0]                       m_axi_awlen,
    output logic [NUM_CORES*3-1:0]                       m_axi_awsize,
    output logic [NUM_CORES*2-1:0]                       m_axi_awburst,
    output logic [NUM_CORES-1:0]                         m_axi_awlock,
    output logic [NUM_CORES*4-1:0]                       m_axi_awcache,
    output logic [NUM_CORES*3-1:0]                       m_axi_awprot,
    output logic [NUM_CORES*4-1:0]                       m_axi_awqos,
    output logic [NUM_CORES-1:0]                         m_axi_awvalid,
    input  logic [NUM_CORES-1:0]                         m_axi_awready,
    output logic [NUM_CORES*BUS_DATA_W-1:0]              m_axi_wdata,
    output logic [NUM_CORES*(BUS_DATA_W/8)-1:0]          m_axi_wstrb,
    output logic [NUM_CORES-1:0]                         m_axi_wlast,
    output logic [NUM_CORES-1:0]                         m_axi_wvalid,
    input  logic [NUM_CORES-1:0]                         m_axi_wready,
    input  logic [NUM_CORES*(CORE_BUS_ID+CORE_ID_W)-1:0] m_axi_bid,
    input  logic [NUM_CORES*2-1:0]                       m_axi_bresp,
    input  logic [NUM_CORES-1:0]                         m_axi_bvalid,
    output logic [NUM_CORES-1:0]                         m_axi_bready,
    output logic [NUM_CORES*(CORE_BUS_ID+CORE_ID_W)-1:0] m_axi_arid,
    output logic [NUM_CORES*BUS_ADDR_W-1:0]              m_axi_araddr,
    output logic [NUM_CORES*8-1:0]                       m_axi_arlen,
    output logic [NUM_CORES*3-1:0]                       m_axi_arsize,
    output logic [NUM_CORES*2-1:0]                       m_axi_arburst,
    output logic [NUM_CORES-1:0]                         m_axi_arlock,
    output logic [NUM_CORES*4-1:0]                       m_axi_arcache,
    output logic [NUM_CORES*3-1:0]                       m_axi_arprot,
    output logic [NUM_CORES*4-1:0]                       m_axi_arqos,
    output logic [NUM_CORES-1:0]                         m_axi_arvalid,
    input  logic [NUM_CORES-1:0]                         m_axi_arready,
    input  logic [NUM_CORES*(CORE_BUS_ID+CORE_ID_W)-1:0] m_axi_rid,
    input  logic [NUM_CORES*BUS_DATA_W-1:0]              m_axi_rdata,
    input  logic [NUM_CORES*2-1:0]                       m_axi_rresp,
    input  logic [NUM_CORES-1:0]                         m_axi_rlast,
    input  logic [NUM_CORES-1:0]                         m_axi_rvalid,
    output logic [NUM_CORES-1:0]                         m_axi_rready,

    // ======================================================================
    // IRQ outputs (per-core done) → PS A72 GIC
    // ======================================================================
    output logic [NUM_CORES-1:0]        irq_done
);

    // =========================================================================
    // 1. 内部信号 (per-core CSR + AXI master)
    // =========================================================================
    // CSR demux: csr_axil → 3 个 core 的 csr_xx 接口
    logic [NUM_CORES-1:0]               c_csr_awvalid, c_csr_awready;
    logic [11:0]                        c_csr_awaddr  [NUM_CORES];
    logic [CSR_DATA_W-1:0]              c_csr_wdata   [NUM_CORES];
    logic [CSR_DATA_W/8-1:0]            c_csr_wstrb   [NUM_CORES];
    logic [NUM_CORES-1:0]               c_csr_wvalid, c_csr_wready;
    logic [1:0]                         c_csr_bresp   [NUM_CORES];
    logic [NUM_CORES-1:0]               c_csr_bvalid, c_csr_bready;
    logic [11:0]                        c_csr_araddr  [NUM_CORES];
    logic [NUM_CORES-1:0]               c_csr_arvalid, c_csr_arready;
    logic [CSR_DATA_W-1:0]              c_csr_rdata   [NUM_CORES];
    logic [1:0]                         c_csr_rresp   [NUM_CORES];
    logic [NUM_CORES-1:0]               c_csr_rvalid, c_csr_rready;

    // ConvCore AXI master 信号 (内部 array, 末尾 pack 给顶层端口)
    logic [CORE_BUS_ID-1:0]   c_bus_awid    [NUM_CORES];
    logic [BUS_ADDR_W-1:0]    c_bus_awaddr  [NUM_CORES];
    logic [7:0]               c_bus_awlen   [NUM_CORES];
    logic [2:0]               c_bus_awsize  [NUM_CORES];
    logic [1:0]               c_bus_awburst [NUM_CORES];
    logic                     c_bus_awlock  [NUM_CORES];
    logic [3:0]               c_bus_awcache [NUM_CORES];
    logic [2:0]               c_bus_awprot  [NUM_CORES];
    logic [3:0]               c_bus_awqos   [NUM_CORES];
    logic                     c_bus_awvalid [NUM_CORES];
    logic                     c_bus_awready [NUM_CORES];
    logic [BUS_DATA_W-1:0]    c_bus_wdata   [NUM_CORES];
    logic [BUS_DATA_W/8-1:0]  c_bus_wstrb   [NUM_CORES];
    logic                     c_bus_wlast   [NUM_CORES];
    logic                     c_bus_wvalid  [NUM_CORES];
    logic                     c_bus_wready  [NUM_CORES];
    logic [CORE_BUS_ID-1:0]   c_bus_bid     [NUM_CORES];
    logic [1:0]               c_bus_bresp   [NUM_CORES];
    logic                     c_bus_bvalid  [NUM_CORES];
    logic                     c_bus_bready  [NUM_CORES];
    logic [CORE_BUS_ID-1:0]   c_bus_arid    [NUM_CORES];
    logic [BUS_ADDR_W-1:0]    c_bus_araddr  [NUM_CORES];
    logic [7:0]               c_bus_arlen   [NUM_CORES];
    logic [2:0]               c_bus_arsize  [NUM_CORES];
    logic [1:0]               c_bus_arburst [NUM_CORES];
    logic                     c_bus_arlock  [NUM_CORES];
    logic [3:0]               c_bus_arcache [NUM_CORES];
    logic [2:0]               c_bus_arprot  [NUM_CORES];
    logic [3:0]               c_bus_arqos   [NUM_CORES];
    logic                     c_bus_arvalid [NUM_CORES];
    logic                     c_bus_arready [NUM_CORES];
    logic [CORE_BUS_ID-1:0]   c_bus_rid     [NUM_CORES];
    logic [BUS_DATA_W-1:0]    c_bus_rdata   [NUM_CORES];
    logic [1:0]               c_bus_rresp   [NUM_CORES];
    logic                     c_bus_rlast   [NUM_CORES];
    logic                     c_bus_rvalid  [NUM_CORES];
    logic                     c_bus_rready  [NUM_CORES];

    logic [NUM_CORES-1:0]     done_per_core;
    assign irq_done = done_per_core;

    // =========================================================================
    // 2. AXI-Lite 1to3 demux (替代 axi_lite_1to4 IP, 简化版)
    //
    //   csr_axil_awaddr/araddr[13:12] 选 core (00=core0, 01=core1, 10=core2)
    //   csr_axil_awaddr/araddr[11:0]  per-core 寄存器地址 (传给 ConvCore)
    //
    //   严格 round-robin / 单 outstanding 简化, demo 性能足够 (host CSR 写很少).
    // =========================================================================
    logic [1:0]  aw_target, ar_target;
    assign aw_target = csr_axil_awaddr[13:12];
    assign ar_target = csr_axil_araddr[13:12];

    // AW: 路由到 c_csr_awvalid[aw_target]
    always_comb begin
        for (int i = 0; i < NUM_CORES; i++) begin
            c_csr_awaddr[i]  = csr_axil_awaddr[11:0];
            c_csr_wdata[i]   = csr_axil_wdata;
            c_csr_wstrb[i]   = csr_axil_wstrb;
            c_csr_awvalid[i] = csr_axil_awvalid && (aw_target == i[1:0]);
            c_csr_wvalid[i]  = csr_axil_wvalid  && (aw_target == i[1:0]);
            c_csr_bready[i]  = csr_axil_bready  && (aw_target == i[1:0]);
            c_csr_araddr[i]  = csr_axil_araddr[11:0];
            c_csr_arvalid[i] = csr_axil_arvalid && (ar_target == i[1:0]);
            c_csr_rready[i]  = csr_axil_rready  && (ar_target == i[1:0]);
        end
    end

    // 反向: 从被选中 core 的 ready/valid/data 回流到顶层
    always_comb begin
        csr_axil_awready = (aw_target < NUM_CORES) ? c_csr_awready[aw_target] : 1'b0;
        csr_axil_wready  = (aw_target < NUM_CORES) ? c_csr_wready[aw_target]  : 1'b0;
        csr_axil_bvalid  = (aw_target < NUM_CORES) ? c_csr_bvalid[aw_target]  : 1'b0;
        csr_axil_bresp   = (aw_target < NUM_CORES) ? c_csr_bresp[aw_target]   : 2'b00;
        csr_axil_arready = (ar_target < NUM_CORES) ? c_csr_arready[ar_target] : 1'b0;
        csr_axil_rvalid  = (ar_target < NUM_CORES) ? c_csr_rvalid[ar_target]  : 1'b0;
        csr_axil_rresp   = (ar_target < NUM_CORES) ? c_csr_rresp[ar_target]   : 2'b00;
        csr_axil_rdata   = (ar_target < NUM_CORES) ? c_csr_rdata[ar_target]   : '0;
    end

    // =========================================================================
    // 3. 实例化 NUM_CORES (=3) 个 ConvCore (复用 main 分支 core_top.sv)
    // =========================================================================
    genvar gi;
    generate
        for (gi = 0; gi < NUM_CORES; gi++) begin : gen_core
            core_top #(
                .CSR_ADDR_W(12), .CSR_DATA_W(CSR_DATA_W),
                .BUS_ADDR_W(BUS_ADDR_W), .BUS_DATA_W(BUS_DATA_W),
                .AXI_M_ID(CORE_BUS_ID), .AXI_M_WIDTH(`FLUX_AXI_M_WIDTH),
                .RMT_ID_W(CORE_BUS_ID + 1),    // 跟 multicore_top_smc 一致 (rmt_ifb tie 0)
                .SG_MODE(1)                     // VD100 demo: IDMA 走 SG cmd list (跟 SMC 路径一致)
            ) u_core (
                .clk    (clk),
                .rst_n  (rst_n),

                // CSR (来自 1to3 demux)
                .csr_awaddr (c_csr_awaddr [gi]),
                .csr_awvalid(c_csr_awvalid[gi]),
                .csr_awready(c_csr_awready[gi]),
                .csr_wdata  (c_csr_wdata  [gi]),
                .csr_wstrb  (c_csr_wstrb  [gi]),
                .csr_wvalid (c_csr_wvalid [gi]),
                .csr_wready (c_csr_wready [gi]),
                .csr_bresp  (c_csr_bresp  [gi]),
                .csr_bvalid (c_csr_bvalid [gi]),
                .csr_bready (c_csr_bready [gi]),
                .csr_araddr (c_csr_araddr [gi]),
                .csr_arvalid(c_csr_arvalid[gi]),
                .csr_arready(c_csr_arready[gi]),
                .csr_rdata  (c_csr_rdata  [gi]),
                .csr_rresp  (c_csr_rresp  [gi]),
                .csr_rvalid (c_csr_rvalid [gi]),
                .csr_rready (c_csr_rready [gi]),

                // AXI master (出 → 顶层 → Vitis NoC → DDRMC)
                .bus_awid(c_bus_awid[gi]), .bus_awaddr(c_bus_awaddr[gi]),
                .bus_awlen(c_bus_awlen[gi]), .bus_awsize(c_bus_awsize[gi]),
                .bus_awburst(c_bus_awburst[gi]), .bus_awlock(c_bus_awlock[gi]),
                .bus_awcache(c_bus_awcache[gi]), .bus_awprot(c_bus_awprot[gi]),
                .bus_awqos(c_bus_awqos[gi]),
                .bus_awvalid(c_bus_awvalid[gi]), .bus_awready(c_bus_awready[gi]),
                .bus_wdata(c_bus_wdata[gi]), .bus_wstrb(c_bus_wstrb[gi]),
                .bus_wlast(c_bus_wlast[gi]), .bus_wvalid(c_bus_wvalid[gi]),
                .bus_wready(c_bus_wready[gi]),
                .bus_bid(c_bus_bid[gi]), .bus_bresp(c_bus_bresp[gi]),
                .bus_bvalid(c_bus_bvalid[gi]), .bus_bready(c_bus_bready[gi]),

                .bus_arid(c_bus_arid[gi]), .bus_araddr(c_bus_araddr[gi]),
                .bus_arlen(c_bus_arlen[gi]), .bus_arsize(c_bus_arsize[gi]),
                .bus_arburst(c_bus_arburst[gi]), .bus_arlock(c_bus_arlock[gi]),
                .bus_arcache(c_bus_arcache[gi]), .bus_arprot(c_bus_arprot[gi]),
                .bus_arqos(c_bus_arqos[gi]),
                .bus_arvalid(c_bus_arvalid[gi]), .bus_arready(c_bus_arready[gi]),
                .bus_rid(c_bus_rid[gi]), .bus_rdata(c_bus_rdata[gi]),
                .bus_rresp(c_bus_rresp[gi]),
                .bus_rlast(c_bus_rlast[gi]), .bus_rvalid(c_bus_rvalid[gi]),
                .bus_rready(c_bus_rready[gi]),

                // rmt_ifb tie 0 (NUMA 模式不用跨核 push, VD100 demo 也不用)
                .rmt_ifb_awid('0), .rmt_ifb_awaddr('0),
                .rmt_ifb_awlen('0), .rmt_ifb_awsize('0),
                .rmt_ifb_awburst('0), .rmt_ifb_awlock('0),
                .rmt_ifb_awcache('0), .rmt_ifb_awprot('0),
                .rmt_ifb_awqos('0),
                .rmt_ifb_awvalid(1'b0), .rmt_ifb_awready(),
                .rmt_ifb_wdata('0), .rmt_ifb_wstrb('0),
                .rmt_ifb_wlast(1'b0), .rmt_ifb_wvalid(1'b0), .rmt_ifb_wready(),
                .rmt_ifb_bid(), .rmt_ifb_bresp(),
                .rmt_ifb_bvalid(), .rmt_ifb_bready(1'b0),
                .rmt_ifb_arid('0), .rmt_ifb_araddr('0),
                .rmt_ifb_arlen('0), .rmt_ifb_arsize('0),
                .rmt_ifb_arburst('0), .rmt_ifb_arlock('0),
                .rmt_ifb_arcache('0), .rmt_ifb_arprot('0),
                .rmt_ifb_arqos('0),
                .rmt_ifb_arvalid(1'b0), .rmt_ifb_arready(),
                .rmt_ifb_rid(), .rmt_ifb_rdata(), .rmt_ifb_rresp(),
                .rmt_ifb_rlast(), .rmt_ifb_rvalid(), .rmt_ifb_rready(1'b0),

                // tb 后门 (synth 时 tie 0)
                .ifb_we_ext(1'b0), .ifb_waddr_ext('0), .ifb_wdata_ext('0),
                .wb_we_ext(1'b0),  .wb_waddr_ext('0),  .wb_wdata_ext('0),
                .ofb_re_ext(1'b0), .ofb_raddr_ext('0), .ofb_rdata_ext(),

                .done(done_per_core[gi]),
                .ofm_tdest(),  .ofm_opcode()
            );
        end
    endgenerate

    // =========================================================================
    // 4. AXI master pack: array → 顶层 vector 端口 (NoC 接)
    //
    // m_axi_<chan>id 包含 ConvCore 内部 ID (CORE_BUS_ID) + core_id (CORE_ID_W) 标记,
    // 让 Vitis NoC / DDRMC 端能区分 transaction 来源 (debug 用).
    // =========================================================================
    generate
        for (gi = 0; gi < NUM_CORES; gi++) begin : gen_pack
            assign m_axi_awid   [gi*(CORE_BUS_ID+CORE_ID_W) +: (CORE_BUS_ID+CORE_ID_W)] = {gi[CORE_ID_W-1:0], c_bus_awid[gi]};
            assign m_axi_awaddr [gi*BUS_ADDR_W +: BUS_ADDR_W]                            = c_bus_awaddr [gi];
            assign m_axi_awlen  [gi*8 +: 8]                                              = c_bus_awlen  [gi];
            assign m_axi_awsize [gi*3 +: 3]                                              = c_bus_awsize [gi];
            assign m_axi_awburst[gi*2 +: 2]                                              = c_bus_awburst[gi];
            assign m_axi_awlock [gi]                                                     = c_bus_awlock [gi];
            assign m_axi_awcache[gi*4 +: 4]                                              = c_bus_awcache[gi];
            assign m_axi_awprot [gi*3 +: 3]                                              = c_bus_awprot [gi];
            assign m_axi_awqos  [gi*4 +: 4]                                              = c_bus_awqos  [gi];
            assign m_axi_awvalid[gi]                                                     = c_bus_awvalid[gi];
            assign c_bus_awready[gi]                                                     = m_axi_awready[gi];
            assign m_axi_wdata  [gi*BUS_DATA_W +: BUS_DATA_W]                            = c_bus_wdata  [gi];
            assign m_axi_wstrb  [gi*(BUS_DATA_W/8) +: (BUS_DATA_W/8)]                    = c_bus_wstrb  [gi];
            assign m_axi_wlast  [gi]                                                     = c_bus_wlast  [gi];
            assign m_axi_wvalid [gi]                                                     = c_bus_wvalid [gi];
            assign c_bus_wready [gi]                                                     = m_axi_wready [gi];
            assign c_bus_bid    [gi]                                                     = m_axi_bid    [gi*(CORE_BUS_ID+CORE_ID_W) +: CORE_BUS_ID];
            assign c_bus_bresp  [gi]                                                     = m_axi_bresp  [gi*2 +: 2];
            assign c_bus_bvalid [gi]                                                     = m_axi_bvalid [gi];
            assign m_axi_bready [gi]                                                     = c_bus_bready [gi];

            assign m_axi_arid   [gi*(CORE_BUS_ID+CORE_ID_W) +: (CORE_BUS_ID+CORE_ID_W)] = {gi[CORE_ID_W-1:0], c_bus_arid[gi]};
            assign m_axi_araddr [gi*BUS_ADDR_W +: BUS_ADDR_W]                            = c_bus_araddr [gi];
            assign m_axi_arlen  [gi*8 +: 8]                                              = c_bus_arlen  [gi];
            assign m_axi_arsize [gi*3 +: 3]                                              = c_bus_arsize [gi];
            assign m_axi_arburst[gi*2 +: 2]                                              = c_bus_arburst[gi];
            assign m_axi_arlock [gi]                                                     = c_bus_arlock [gi];
            assign m_axi_arcache[gi*4 +: 4]                                              = c_bus_arcache[gi];
            assign m_axi_arprot [gi*3 +: 3]                                              = c_bus_arprot [gi];
            assign m_axi_arqos  [gi*4 +: 4]                                              = c_bus_arqos  [gi];
            assign m_axi_arvalid[gi]                                                     = c_bus_arvalid[gi];
            assign c_bus_arready[gi]                                                     = m_axi_arready[gi];
            assign c_bus_rid    [gi]                                                     = m_axi_rid    [gi*(CORE_BUS_ID+CORE_ID_W) +: CORE_BUS_ID];
            assign c_bus_rdata  [gi]                                                     = m_axi_rdata  [gi*BUS_DATA_W +: BUS_DATA_W];
            assign c_bus_rresp  [gi]                                                     = m_axi_rresp  [gi*2 +: 2];
            assign c_bus_rlast  [gi]                                                     = m_axi_rlast  [gi];
            assign c_bus_rvalid [gi]                                                     = m_axi_rvalid [gi];
            assign m_axi_rready [gi]                                                     = c_bus_rready [gi];
        end
    endgenerate

endmodule
