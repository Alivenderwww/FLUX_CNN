`timescale 1ns/1ps
`include "flux_cnn_params.svh"

// =============================================================================
// multicore_top_smc.sv  --  4 ConvCore + AXI Crossbar (4M↔4S) + 4 mem
//
// 真正 NUMA 架构 (统一全局地址, halo 不重复, 多 burst 路径)
//
// 哲学:
//   - 4 mem 物理独立, 全局地址空间统一
//   - ConvCore 端 axi_dm 升级 SG mode, 按 driver 编译期 cmd list 多 burst 跑
//   - axi_crossbar IP 按 awaddr/araddr 路由到对应 slave
//   - halo 列只在物理一份, ConvCore[c] 拉 halo 时 burst 跨 mem, axi_dm 拆 cmd 路由
//
// 全局地址 layout (driver 编译期决定):
//   mem[0]: [0x0000_0000, 0x00FF_FFFF]  (16 MB, base S00_AXI)
//   mem[1]: [0x0100_0000, 0x01FF_FFFF]
//   mem[2]: [0x0200_0000, 0x02FF_FFFF]
//   mem[3]: [0x0300_0000, 0x03FF_FFFF]
//
// 跟 multicore_top_mesh.sv 的差别:
//   - 删 mesh_4x2 + router_node + AXIS packet 协议 (用 axi crossbar IP 替代)
//   - 删 mesh_core_wrapper (ConvCore 直接 expose axi master 给 crossbar)
//   - 删 mem_core_stub 的 desc engine + axis_packet_tx (mem 是哑 axi slave)
//   - mem 用 axi_slave_mem.sv (sim model, 真硬件用 axi_dm + DDR 控制器)
//
// TODO:
//   - 等待 axi_dm IP SG mode 重生 (Syn/gen_axi_datamover.tcl 升级)
//   - 等待 axi_smc IP 生成 (Syn/gen_axi_smc_4to4.tcl 新建)
//   - idma_ctrl / odma_ctrl / wdma_ctrl 改 SG cmd 接口
// =============================================================================

module multicore_top_smc #(
    parameter int NUM_CORES   = 4,
    parameter int NUM_COL     = `FLUX_NUM_COL,
    parameter int NUM_PE      = `FLUX_NUM_PE,
    parameter int DATA_WIDTH  = `FLUX_DATA_WIDTH,
    parameter int PSUM_WIDTH  = `FLUX_PSUM_WIDTH,
    parameter int SRAM_DEPTH  = `FLUX_IFB_DEPTH,
    parameter int CSR_DATA_W  = `FLUX_CSR_DATA_W,
    parameter int BUS_ADDR_W  = `FLUX_BUS_ADDR_W,
    parameter int BUS_DATA_W  = `FLUX_BUS_DATA_W,
    parameter int AXI_M_ID    = `FLUX_AXI_M_ID,
    parameter int AXI_M_WIDTH = `FLUX_AXI_M_WIDTH,
    parameter int DMA_LEN_W   = `FLUX_DMA_LEN_W,
    // SLAVE_MEM_DEPTH: 每个 axi_slave_mem 容量 (words). sim 默认 1M (16 MB 装 ResNet11 全数据).
    //   综合时设小 (例 4K = 64 KB / mem × 4 = 256 KB BRAM) 让 FPGA 装得下.
    //   真硬件 mem 是外部 DDR, axi_slave_mem 仅 sim 用.
    parameter int SLAVE_MEM_DEPTH = 1048576,

    localparam int CORE_ID_W   = $clog2(NUM_CORES),
    localparam int HOST_CSR_AW = 12 + CORE_ID_W,
    localparam int CORE_BUS_ID = AXI_M_ID + AXI_M_WIDTH
)(
    input  logic                                 clk,
    input  logic                                 rst_n,

    // ---- AXI-Lite Slave: ConvCore CSR (host 配置, 14-bit 地址) ----
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

    // ---- per-core done sticky ----
    output logic [NUM_CORES-1:0]                 done_per_core,

    // ---- 4 个 mem axi master 出口 (synth 时连 board DDR 控制器, sim 时悬空)
    //     当 SLAVE_MEM_DEPTH=0 (synth mode) 不实例化 axi_slave_mem, m_axi_* 接出去
    //     当 SLAVE_MEM_DEPTH>0 (sim mode) 实例化 axi_slave_mem, m_axi_* tie 0 (TB 不用)
    output logic [NUM_CORES*(CORE_BUS_ID+CORE_ID_W)-1:0] m_axi_awid,
    output logic [NUM_CORES*BUS_ADDR_W-1:0]      m_axi_awaddr,
    output logic [NUM_CORES*8-1:0]               m_axi_awlen,
    output logic [NUM_CORES*2-1:0]               m_axi_awburst,
    output logic [NUM_CORES-1:0]                 m_axi_awvalid,
    input  logic [NUM_CORES-1:0]                 m_axi_awready,
    output logic [NUM_CORES*BUS_DATA_W-1:0]      m_axi_wdata,
    output logic [NUM_CORES*(BUS_DATA_W/8)-1:0]  m_axi_wstrb,
    output logic [NUM_CORES-1:0]                 m_axi_wlast,
    output logic [NUM_CORES-1:0]                 m_axi_wvalid,
    input  logic [NUM_CORES-1:0]                 m_axi_wready,
    input  logic [NUM_CORES*(CORE_BUS_ID+CORE_ID_W)-1:0] m_axi_bid,
    input  logic [NUM_CORES*2-1:0]               m_axi_bresp,
    input  logic [NUM_CORES-1:0]                 m_axi_bvalid,
    output logic [NUM_CORES-1:0]                 m_axi_bready,
    output logic [NUM_CORES*(CORE_BUS_ID+CORE_ID_W)-1:0] m_axi_arid,
    output logic [NUM_CORES*BUS_ADDR_W-1:0]      m_axi_araddr,
    output logic [NUM_CORES*8-1:0]               m_axi_arlen,
    output logic [NUM_CORES*2-1:0]               m_axi_arburst,
    output logic [NUM_CORES-1:0]                 m_axi_arvalid,
    input  logic [NUM_CORES-1:0]                 m_axi_arready,
    input  logic [NUM_CORES*(CORE_BUS_ID+CORE_ID_W)-1:0] m_axi_rid,
    input  logic [NUM_CORES*BUS_DATA_W-1:0]      m_axi_rdata,
    input  logic [NUM_CORES*2-1:0]               m_axi_rresp,
    input  logic [NUM_CORES-1:0]                 m_axi_rlast,
    input  logic [NUM_CORES-1:0]                 m_axi_rvalid,
    output logic [NUM_CORES-1:0]                 m_axi_rready
);

    logic aresetn;
    assign aresetn = rst_n;

    // =========================================================================
    // 1. AXI-Lite host CSR fanout (axi_lite_1to4 IP, 复用 multicore_top.sv)
    //    host 写 ConvCore[i].cfg_regs 时高 2 bit = core_id, 低 12 bit = reg offset
    // =========================================================================
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

    axi_lite_1to4 u_csr_xbar (
        .aclk          (clk), .aresetn(aresetn),
        .s_axi_awaddr  (csr_awaddr),  .s_axi_awprot(3'b000),
        .s_axi_awvalid (csr_awvalid), .s_axi_awready(csr_awready),
        .s_axi_wdata   (csr_wdata),   .s_axi_wstrb (csr_wstrb),
        .s_axi_wvalid  (csr_wvalid),  .s_axi_wready(csr_wready),
        .s_axi_bresp   (csr_bresp),
        .s_axi_bvalid  (csr_bvalid),  .s_axi_bready(csr_bready),
        .s_axi_araddr  (csr_araddr),  .s_axi_arprot(3'b000),
        .s_axi_arvalid (csr_arvalid), .s_axi_arready(csr_arready),
        .s_axi_rdata   (csr_rdata),   .s_axi_rresp (csr_rresp),
        .s_axi_rvalid  (csr_rvalid),  .s_axi_rready(csr_rready),
        .m_axi_awaddr  (csr_m_awaddr),  .m_axi_awprot(csr_m_awprot),
        .m_axi_awvalid (csr_m_awvalid), .m_axi_awready(csr_m_awready),
        .m_axi_wdata   (csr_m_wdata),   .m_axi_wstrb (csr_m_wstrb),
        .m_axi_wvalid  (csr_m_wvalid),  .m_axi_wready(csr_m_wready),
        .m_axi_bresp   (csr_m_bresp),
        .m_axi_bvalid  (csr_m_bvalid),  .m_axi_bready(csr_m_bready),
        .m_axi_araddr  (csr_m_araddr),  .m_axi_arprot(csr_m_arprot),
        .m_axi_arvalid (csr_m_arvalid), .m_axi_arready(csr_m_arready),
        .m_axi_rdata   (csr_m_rdata),   .m_axi_rresp (csr_m_rresp),
        .m_axi_rvalid  (csr_m_rvalid),  .m_axi_rready(csr_m_rready)
    );

    // =========================================================================
    // 2. 4 ConvCore - 每核一个 axi master 接口 (含 IDMA/ODMA/WDMA/RDMA + DFE)
    // =========================================================================
    // ConvCore[i] axi master 接口 (per-core)
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

    genvar gi;
    generate
        for (gi = 0; gi < NUM_CORES; gi++) begin : gen_core
            core_top #(
                .NUM_COL(NUM_COL), .NUM_PE(NUM_PE), .DATA_WIDTH(DATA_WIDTH),
                .PSUM_WIDTH(PSUM_WIDTH), .SRAM_DEPTH(SRAM_DEPTH),
                .CSR_ADDR_W(12), .CSR_DATA_W(CSR_DATA_W),
                .BUS_ADDR_W(BUS_ADDR_W), .BUS_DATA_W(BUS_DATA_W),
                .AXI_M_ID(AXI_M_ID), .AXI_M_WIDTH(AXI_M_WIDTH),
                .RMT_ID_W(CORE_BUS_ID + 1),
                .SG_MODE(1)            // SMC + NUMA: IDMA 走 SG cmd list
            ) u_core (
                .clk(clk), .rst_n(rst_n),
                // host CSR
                .csr_awaddr (csr_m_awaddr [gi*HOST_CSR_AW +: 12]),
                .csr_awvalid(csr_m_awvalid[gi]), .csr_awready(csr_m_awready[gi]),
                .csr_wdata  (csr_m_wdata  [gi*CSR_DATA_W +: CSR_DATA_W]),
                .csr_wstrb  (csr_m_wstrb  [gi*(CSR_DATA_W/8) +: (CSR_DATA_W/8)]),
                .csr_wvalid (csr_m_wvalid [gi]), .csr_wready (csr_m_wready [gi]),
                .csr_bresp  (csr_m_bresp  [gi*2 +: 2]),
                .csr_bvalid (csr_m_bvalid [gi]), .csr_bready (csr_m_bready [gi]),
                .csr_araddr (csr_m_araddr [gi*HOST_CSR_AW +: 12]),
                .csr_arvalid(csr_m_arvalid[gi]), .csr_arready(csr_m_arready[gi]),
                .csr_rdata  (csr_m_rdata  [gi*CSR_DATA_W +: CSR_DATA_W]),
                .csr_rresp  (csr_m_rresp  [gi*2 +: 2]),
                .csr_rvalid (csr_m_rvalid [gi]), .csr_rready (csr_m_rready [gi]),

                // bus_aw/w/b: ConvCore axi master write (ODMA / WDMA write 通道)
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

                // bus_ar/r: ConvCore axi master read (IDMA / WDMA / RDMA / DFE 共享)
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

                // rmt_ifb tie 0 (NUMA 模式不用跨核 push)
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

                // tb 后门 tie 0
                .ifb_we_ext(1'b0), .ifb_waddr_ext('0), .ifb_wdata_ext('0),
                .wb_we_ext(1'b0),  .wb_waddr_ext('0),  .wb_wdata_ext('0),
                .ofb_re_ext(1'b0), .ofb_raddr_ext('0), .ofb_rdata_ext(),

                .done(done_per_core[gi]),
                .ofm_tdest(),  .ofm_opcode()
            );
        end
    endgenerate

    // =========================================================================
    // 3. AXI Crossbar 4M ↔ 4S
    //    TODO: 用 Vivado axi_crossbar / axi_smc IP 实例化
    //    暂时用 placeholder, 等 Syn/gen_axi_smc_4to4.tcl 跑完后填充实际 IP 名
    //
    // 配置参数 (driver 编译期跟硬件 IP config 必须一致):
    //   M00_AXI_BASE = 0x0000_0000, RANGE = 16 MB → mem[0]
    //   M01_AXI_BASE = 0x0100_0000, RANGE = 16 MB → mem[1]
    //   M02_AXI_BASE = 0x0200_0000, RANGE = 16 MB → mem[2]
    //   M03_AXI_BASE = 0x0300_0000, RANGE = 16 MB → mem[3]
    // =========================================================================

    // 4 个 mem axi slave 接口 (per-mem)
    logic [CORE_BUS_ID+CORE_ID_W-1:0]   m_bus_awid    [NUM_CORES];
    logic [BUS_ADDR_W-1:0]              m_bus_awaddr  [NUM_CORES];
    logic [7:0]                         m_bus_awlen   [NUM_CORES];
    logic [1:0]                         m_bus_awburst [NUM_CORES];
    logic                               m_bus_awvalid [NUM_CORES];
    logic                               m_bus_awready [NUM_CORES];
    logic [BUS_DATA_W-1:0]              m_bus_wdata   [NUM_CORES];
    logic [BUS_DATA_W/8-1:0]            m_bus_wstrb   [NUM_CORES];
    logic                               m_bus_wlast   [NUM_CORES];
    logic                               m_bus_wvalid  [NUM_CORES];
    logic                               m_bus_wready  [NUM_CORES];
    logic [CORE_BUS_ID+CORE_ID_W-1:0]   m_bus_bid     [NUM_CORES];
    logic [1:0]                         m_bus_bresp   [NUM_CORES];
    logic                               m_bus_bvalid  [NUM_CORES];
    logic                               m_bus_bready  [NUM_CORES];
    logic [CORE_BUS_ID+CORE_ID_W-1:0]   m_bus_arid    [NUM_CORES];
    logic [BUS_ADDR_W-1:0]              m_bus_araddr  [NUM_CORES];
    logic [7:0]                         m_bus_arlen   [NUM_CORES];
    logic [1:0]                         m_bus_arburst [NUM_CORES];
    logic                               m_bus_arvalid [NUM_CORES];
    logic                               m_bus_arready [NUM_CORES];
    logic [CORE_BUS_ID+CORE_ID_W-1:0]   m_bus_rid     [NUM_CORES];
    logic [BUS_DATA_W-1:0]              m_bus_rdata   [NUM_CORES];
    logic [1:0]                         m_bus_rresp   [NUM_CORES];
    logic                               m_bus_rlast   [NUM_CORES];
    logic                               m_bus_rvalid  [NUM_CORES];
    logic                               m_bus_rready  [NUM_CORES];

    // ===== AXI Crossbar 4M ↔ 4S
    //   USE_AXI_SMC_IP=0: axi_crossbar_4to4_sim (sim model, 默认)
    //   USE_AXI_SMC_IP=1: Vivado axi_smc_4to4 IP (Syn/gen_axi_smc_4to4.tcl 生成)
    //
    //   ID 宽: SI 端 ConvCore 出口 = CORE_BUS_ID = 4 bit; MI 端 = 6 bit
    //   (CB_ID_W = CORE_BUS_ID + CORE_ID_W = 6, IP 把 SI tag 加到 ID 高位).
    // =========================================================================
    localparam int CB_ID_W = CORE_BUS_ID + CORE_ID_W;     // 6
    // 用 ifdef 选 IP 还是 sim model. 默认 sim model (兼容已 PASS 的 sim regression).
    // 用户跑完 Syn/gen_axi_smc_4to4.tcl 后, 在 vlog 加 +define+USE_AXI_SMC_IP 切到 IP.

    // pack 4 SI signals (ID 高位 padding 0)
    logic [NUM_CORES*CB_ID_W-1:0]   sb_awid_pack;
    logic [NUM_CORES*BUS_ADDR_W-1:0] sb_awaddr_pack;
    logic [NUM_CORES*8-1:0]         sb_awlen_pack;
    logic [NUM_CORES*2-1:0]         sb_awburst_pack;
    logic [NUM_CORES-1:0]           sb_awvalid_pack;
    logic [NUM_CORES-1:0]           sb_awready_pack;
    logic [NUM_CORES*BUS_DATA_W-1:0] sb_wdata_pack;
    logic [NUM_CORES*(BUS_DATA_W/8)-1:0] sb_wstrb_pack;
    logic [NUM_CORES-1:0]           sb_wlast_pack;
    logic [NUM_CORES-1:0]           sb_wvalid_pack;
    logic [NUM_CORES-1:0]           sb_wready_pack;
    logic [NUM_CORES*CB_ID_W-1:0]   sb_bid_pack;
    logic [NUM_CORES*2-1:0]         sb_bresp_pack;
    logic [NUM_CORES-1:0]           sb_bvalid_pack;
    logic [NUM_CORES-1:0]           sb_bready_pack;
    logic [NUM_CORES*CB_ID_W-1:0]   sb_arid_pack;
    logic [NUM_CORES*BUS_ADDR_W-1:0] sb_araddr_pack;
    logic [NUM_CORES*8-1:0]         sb_arlen_pack;
    logic [NUM_CORES*2-1:0]         sb_arburst_pack;
    logic [NUM_CORES-1:0]           sb_arvalid_pack;
    logic [NUM_CORES-1:0]           sb_arready_pack;
    logic [NUM_CORES*CB_ID_W-1:0]   sb_rid_pack;
    logic [NUM_CORES*BUS_DATA_W-1:0] sb_rdata_pack;
    logic [NUM_CORES*2-1:0]         sb_rresp_pack;
    logic [NUM_CORES-1:0]           sb_rlast_pack;
    logic [NUM_CORES-1:0]           sb_rvalid_pack;
    logic [NUM_CORES-1:0]           sb_rready_pack;

    logic [NUM_CORES*CB_ID_W-1:0]   mb_awid_pack;
    logic [NUM_CORES*BUS_ADDR_W-1:0] mb_awaddr_pack;
    logic [NUM_CORES*8-1:0]         mb_awlen_pack;
    logic [NUM_CORES*2-1:0]         mb_awburst_pack;
    logic [NUM_CORES-1:0]           mb_awvalid_pack;
    logic [NUM_CORES-1:0]           mb_awready_pack;
    logic [NUM_CORES*BUS_DATA_W-1:0] mb_wdata_pack;
    logic [NUM_CORES*(BUS_DATA_W/8)-1:0] mb_wstrb_pack;
    logic [NUM_CORES-1:0]           mb_wlast_pack;
    logic [NUM_CORES-1:0]           mb_wvalid_pack;
    logic [NUM_CORES-1:0]           mb_wready_pack;
    logic [NUM_CORES*CB_ID_W-1:0]   mb_bid_pack;
    logic [NUM_CORES*2-1:0]         mb_bresp_pack;
    logic [NUM_CORES-1:0]           mb_bvalid_pack;
    logic [NUM_CORES-1:0]           mb_bready_pack;
    logic [NUM_CORES*CB_ID_W-1:0]   mb_arid_pack;
    logic [NUM_CORES*BUS_ADDR_W-1:0] mb_araddr_pack;
    logic [NUM_CORES*8-1:0]         mb_arlen_pack;
    logic [NUM_CORES*2-1:0]         mb_arburst_pack;
    logic [NUM_CORES-1:0]           mb_arvalid_pack;
    logic [NUM_CORES-1:0]           mb_arready_pack;
    logic [NUM_CORES*CB_ID_W-1:0]   mb_rid_pack;
    logic [NUM_CORES*BUS_DATA_W-1:0] mb_rdata_pack;
    logic [NUM_CORES*2-1:0]         mb_rresp_pack;
    logic [NUM_CORES-1:0]           mb_rlast_pack;
    logic [NUM_CORES-1:0]           mb_rvalid_pack;
    logic [NUM_CORES-1:0]           mb_rready_pack;

    generate
        for (gi = 0; gi < NUM_CORES; gi++) begin : gen_smc_pack
            // SI in: ConvCore[gi].bus_* → crossbar SI port gi
            //   ID 高 CORE_ID_W bit 用 0 padding (我们用 owner 寄存器路由不用 ID)
            assign sb_awid_pack  [gi*CB_ID_W   +: CB_ID_W]   = {{CORE_ID_W{1'b0}}, c_bus_awid[gi]};
            assign sb_awaddr_pack[gi*BUS_ADDR_W +: BUS_ADDR_W] = c_bus_awaddr[gi];
            assign sb_awlen_pack [gi*8         +: 8]         = c_bus_awlen[gi];
            assign sb_awburst_pack[gi*2        +: 2]         = c_bus_awburst[gi];
            assign sb_awvalid_pack[gi]                       = c_bus_awvalid[gi];
            assign c_bus_awready[gi]                         = sb_awready_pack[gi];
            assign sb_wdata_pack [gi*BUS_DATA_W +: BUS_DATA_W] = c_bus_wdata[gi];
            assign sb_wstrb_pack [gi*(BUS_DATA_W/8) +: (BUS_DATA_W/8)] = c_bus_wstrb[gi];
            assign sb_wlast_pack [gi]                        = c_bus_wlast[gi];
            assign sb_wvalid_pack[gi]                        = c_bus_wvalid[gi];
            assign c_bus_wready[gi]                          = sb_wready_pack[gi];
            assign c_bus_bid[gi]                             = sb_bid_pack[gi*CB_ID_W +: CORE_BUS_ID];
            assign c_bus_bresp[gi]                           = sb_bresp_pack[gi*2 +: 2];
            assign c_bus_bvalid[gi]                          = sb_bvalid_pack[gi];
            assign sb_bready_pack[gi]                        = c_bus_bready[gi];

            assign sb_arid_pack  [gi*CB_ID_W   +: CB_ID_W]   = {{CORE_ID_W{1'b0}}, c_bus_arid[gi]};
            assign sb_araddr_pack[gi*BUS_ADDR_W +: BUS_ADDR_W] = c_bus_araddr[gi];
            assign sb_arlen_pack [gi*8         +: 8]         = c_bus_arlen[gi];
            assign sb_arburst_pack[gi*2        +: 2]         = c_bus_arburst[gi];
            assign sb_arvalid_pack[gi]                       = c_bus_arvalid[gi];
            assign c_bus_arready[gi]                         = sb_arready_pack[gi];
            assign c_bus_rid[gi]                             = sb_rid_pack[gi*CB_ID_W +: CORE_BUS_ID];
            assign c_bus_rdata[gi]                           = sb_rdata_pack[gi*BUS_DATA_W +: BUS_DATA_W];
            assign c_bus_rresp[gi]                           = sb_rresp_pack[gi*2 +: 2];
            assign c_bus_rlast[gi]                           = sb_rlast_pack[gi];
            assign c_bus_rvalid[gi]                          = sb_rvalid_pack[gi];
            assign sb_rready_pack[gi]                        = c_bus_rready[gi];

            // MI out: crossbar MI port gi → mem[gi]
            assign m_bus_awid[gi]    = mb_awid_pack[gi*CB_ID_W +: CB_ID_W];
            assign m_bus_awaddr[gi]  = mb_awaddr_pack[gi*BUS_ADDR_W +: BUS_ADDR_W];
            assign m_bus_awlen[gi]   = mb_awlen_pack[gi*8 +: 8];
            assign m_bus_awburst[gi] = mb_awburst_pack[gi*2 +: 2];
            assign m_bus_awvalid[gi] = mb_awvalid_pack[gi];
            assign mb_awready_pack[gi] = m_bus_awready[gi];
            assign m_bus_wdata[gi]   = mb_wdata_pack[gi*BUS_DATA_W +: BUS_DATA_W];
            assign m_bus_wstrb[gi]   = mb_wstrb_pack[gi*(BUS_DATA_W/8) +: (BUS_DATA_W/8)];
            assign m_bus_wlast[gi]   = mb_wlast_pack[gi];
            assign m_bus_wvalid[gi]  = mb_wvalid_pack[gi];
            assign mb_wready_pack[gi] = m_bus_wready[gi];
            assign mb_bid_pack[gi*CB_ID_W +: CB_ID_W] = m_bus_bid[gi];
            assign mb_bresp_pack[gi*2 +: 2] = m_bus_bresp[gi];
            assign mb_bvalid_pack[gi] = m_bus_bvalid[gi];
            assign m_bus_bready[gi]  = mb_bready_pack[gi];

            assign m_bus_arid[gi]    = mb_arid_pack[gi*CB_ID_W +: CB_ID_W];
            assign m_bus_araddr[gi]  = mb_araddr_pack[gi*BUS_ADDR_W +: BUS_ADDR_W];
            assign m_bus_arlen[gi]   = mb_arlen_pack[gi*8 +: 8];
            assign m_bus_arburst[gi] = mb_arburst_pack[gi*2 +: 2];
            assign m_bus_arvalid[gi] = mb_arvalid_pack[gi];
            assign mb_arready_pack[gi] = m_bus_arready[gi];
            assign mb_rid_pack[gi*CB_ID_W +: CB_ID_W] = m_bus_rid[gi];
            assign mb_rdata_pack[gi*BUS_DATA_W +: BUS_DATA_W] = m_bus_rdata[gi];
            assign mb_rresp_pack[gi*2 +: 2] = m_bus_rresp[gi];
            assign mb_rlast_pack[gi] = m_bus_rlast[gi];
            assign mb_rvalid_pack[gi] = m_bus_rvalid[gi];
            assign m_bus_rready[gi]  = mb_rready_pack[gi];
        end
    endgenerate

`ifdef USE_AXI_SMC_IP
    // ===== Vivado axi_smc_4to4 IP 实例化 =====
    //   先跑 Syn/gen_axi_smc_4to4.tcl 生成 IP, 再 vlog 加 +define+USE_AXI_SMC_IP.
    //   IP 端口比 sim model 多 awsize/awlock/awcache/awprot/awqos 等字段, 这里
    //   显式 pack from c_bus_*[gi] (core_top.sv 出口已经提供).
    logic [NUM_CORES*3-1:0]             sb_awsize_ip,  sb_arsize_ip;
    logic [NUM_CORES-1:0]               sb_awlock_ip,  sb_arlock_ip;
    logic [NUM_CORES*4-1:0]             sb_awcache_ip, sb_arcache_ip;
    logic [NUM_CORES*3-1:0]             sb_awprot_ip,  sb_arprot_ip;
    logic [NUM_CORES*4-1:0]             sb_awqos_ip,   sb_arqos_ip;
    logic [NUM_CORES*3-1:0]             mb_awsize_ip,  mb_arsize_ip;
    logic [NUM_CORES-1:0]               mb_awlock_ip,  mb_arlock_ip;
    logic [NUM_CORES*4-1:0]             mb_awcache_ip, mb_arcache_ip;
    logic [NUM_CORES*3-1:0]             mb_awprot_ip,  mb_arprot_ip;
    logic [NUM_CORES*4-1:0]             mb_awqos_ip,   mb_arqos_ip;
    generate
        for (gi = 0; gi < NUM_CORES; gi++) begin : gen_ip_extra_pack
            assign sb_awsize_ip [gi*3 +: 3] = c_bus_awsize [gi];
            assign sb_awlock_ip [gi]        = c_bus_awlock [gi];
            assign sb_awcache_ip[gi*4 +: 4] = c_bus_awcache[gi];
            assign sb_awprot_ip [gi*3 +: 3] = c_bus_awprot [gi];
            assign sb_awqos_ip  [gi*4 +: 4] = c_bus_awqos  [gi];
            assign sb_arsize_ip [gi*3 +: 3] = c_bus_arsize [gi];
            assign sb_arlock_ip [gi]        = c_bus_arlock [gi];
            assign sb_arcache_ip[gi*4 +: 4] = c_bus_arcache[gi];
            assign sb_arprot_ip [gi*3 +: 3] = c_bus_arprot [gi];
            assign sb_arqos_ip  [gi*4 +: 4] = c_bus_arqos  [gi];
        end
    endgenerate

    axi_smc_4to4 u_smc (
        .aclk         (clk),
        .aresetn      (aresetn),
        // SI (4 ConvCore axi master)
        .s_axi_awid   (sb_awid_pack),    .s_axi_awaddr (sb_awaddr_pack),
        .s_axi_awlen  (sb_awlen_pack),   .s_axi_awsize (sb_awsize_ip),
        .s_axi_awburst(sb_awburst_pack), .s_axi_awlock (sb_awlock_ip),
        .s_axi_awcache(sb_awcache_ip),   .s_axi_awprot (sb_awprot_ip),
        .s_axi_awqos  (sb_awqos_ip),
        .s_axi_awvalid(sb_awvalid_pack), .s_axi_awready(sb_awready_pack),
        .s_axi_wdata  (sb_wdata_pack),   .s_axi_wstrb  (sb_wstrb_pack),
        .s_axi_wlast  (sb_wlast_pack),   .s_axi_wvalid (sb_wvalid_pack),
        .s_axi_wready (sb_wready_pack),
        .s_axi_bid    (sb_bid_pack),     .s_axi_bresp  (sb_bresp_pack),
        .s_axi_bvalid (sb_bvalid_pack),  .s_axi_bready (sb_bready_pack),
        .s_axi_arid   (sb_arid_pack),    .s_axi_araddr (sb_araddr_pack),
        .s_axi_arlen  (sb_arlen_pack),   .s_axi_arsize (sb_arsize_ip),
        .s_axi_arburst(sb_arburst_pack), .s_axi_arlock (sb_arlock_ip),
        .s_axi_arcache(sb_arcache_ip),   .s_axi_arprot (sb_arprot_ip),
        .s_axi_arqos  (sb_arqos_ip),
        .s_axi_arvalid(sb_arvalid_pack), .s_axi_arready(sb_arready_pack),
        .s_axi_rid    (sb_rid_pack),     .s_axi_rdata  (sb_rdata_pack),
        .s_axi_rresp  (sb_rresp_pack),   .s_axi_rlast  (sb_rlast_pack),
        .s_axi_rvalid (sb_rvalid_pack),  .s_axi_rready (sb_rready_pack),
        // MI (4 mem axi slave)
        .m_axi_awid   (mb_awid_pack),    .m_axi_awaddr (mb_awaddr_pack),
        .m_axi_awlen  (mb_awlen_pack),   .m_axi_awsize (mb_awsize_ip),
        .m_axi_awburst(mb_awburst_pack), .m_axi_awlock (mb_awlock_ip),
        .m_axi_awcache(mb_awcache_ip),   .m_axi_awprot (mb_awprot_ip),
        .m_axi_awqos  (mb_awqos_ip),     .m_axi_awregion(),
        .m_axi_awvalid(mb_awvalid_pack), .m_axi_awready(mb_awready_pack),
        .m_axi_wdata  (mb_wdata_pack),   .m_axi_wstrb  (mb_wstrb_pack),
        .m_axi_wlast  (mb_wlast_pack),   .m_axi_wvalid (mb_wvalid_pack),
        .m_axi_wready (mb_wready_pack),
        .m_axi_bid    (mb_bid_pack),     .m_axi_bresp  (mb_bresp_pack),
        .m_axi_bvalid (mb_bvalid_pack),  .m_axi_bready (mb_bready_pack),
        .m_axi_arid   (mb_arid_pack),    .m_axi_araddr (mb_araddr_pack),
        .m_axi_arlen  (mb_arlen_pack),   .m_axi_arsize (mb_arsize_ip),
        .m_axi_arburst(mb_arburst_pack), .m_axi_arlock (mb_arlock_ip),
        .m_axi_arcache(mb_arcache_ip),   .m_axi_arprot (mb_arprot_ip),
        .m_axi_arqos  (mb_arqos_ip),     .m_axi_arregion(),
        .m_axi_arvalid(mb_arvalid_pack), .m_axi_arready(mb_arready_pack),
        .m_axi_rid    (mb_rid_pack),     .m_axi_rdata  (mb_rdata_pack),
        .m_axi_rresp  (mb_rresp_pack),   .m_axi_rlast  (mb_rlast_pack),
        .m_axi_rvalid (mb_rvalid_pack),  .m_axi_rready (mb_rready_pack)
    );
`else
    // ===== sim model: axi_crossbar_4to4_sim (默认, 不依赖 vivado IP 生成) =====
    axi_crossbar_4to4_sim #(
        .N(NUM_CORES), .ADDR_W(BUS_ADDR_W), .DATA_W(BUS_DATA_W), .ID_W(CB_ID_W)
    ) u_smc (
        .clk(clk), .rst_n(rst_n),
        .s_awid(sb_awid_pack), .s_awaddr(sb_awaddr_pack),
        .s_awlen(sb_awlen_pack), .s_awburst(sb_awburst_pack),
        .s_awvalid(sb_awvalid_pack), .s_awready(sb_awready_pack),
        .s_wdata(sb_wdata_pack), .s_wstrb(sb_wstrb_pack),
        .s_wlast(sb_wlast_pack), .s_wvalid(sb_wvalid_pack), .s_wready(sb_wready_pack),
        .s_bid(sb_bid_pack), .s_bresp(sb_bresp_pack),
        .s_bvalid(sb_bvalid_pack), .s_bready(sb_bready_pack),
        .s_arid(sb_arid_pack), .s_araddr(sb_araddr_pack),
        .s_arlen(sb_arlen_pack), .s_arburst(sb_arburst_pack),
        .s_arvalid(sb_arvalid_pack), .s_arready(sb_arready_pack),
        .s_rid(sb_rid_pack), .s_rdata(sb_rdata_pack), .s_rresp(sb_rresp_pack),
        .s_rlast(sb_rlast_pack), .s_rvalid(sb_rvalid_pack), .s_rready(sb_rready_pack),

        .m_awid(mb_awid_pack), .m_awaddr(mb_awaddr_pack),
        .m_awlen(mb_awlen_pack), .m_awburst(mb_awburst_pack),
        .m_awvalid(mb_awvalid_pack), .m_awready(mb_awready_pack),
        .m_wdata(mb_wdata_pack), .m_wstrb(mb_wstrb_pack),
        .m_wlast(mb_wlast_pack), .m_wvalid(mb_wvalid_pack), .m_wready(mb_wready_pack),
        .m_bid(mb_bid_pack), .m_bresp(mb_bresp_pack),
        .m_bvalid(mb_bvalid_pack), .m_bready(mb_bready_pack),
        .m_arid(mb_arid_pack), .m_araddr(mb_araddr_pack),
        .m_arlen(mb_arlen_pack), .m_arburst(mb_arburst_pack),
        .m_arvalid(mb_arvalid_pack), .m_arready(mb_arready_pack),
        .m_rid(mb_rid_pack), .m_rdata(mb_rdata_pack), .m_rresp(mb_rresp_pack),
        .m_rlast(mb_rlast_pack), .m_rvalid(mb_rvalid_pack), .m_rready(mb_rready_pack)
    );
`endif

    // =========================================================================
    // 4. 4 个 mem (axi slave). sim 用 axi_slave_mem.sv, 真硬件用 axi_dm + DDR.
    //    每 mem 对应全局地址某 16 MB 区间 (driver 编译期跟硬件 IP config 一致).
    // =========================================================================
    generate
        if (SLAVE_MEM_DEPTH > 0) begin : g_sim_mem
            // sim 模式: 实例化 axi_slave_mem 充当 mem stub, m_axi_* tie 0
            for (gi = 0; gi < NUM_CORES; gi++) begin : gen_mem
                axi_slave_mem #(
                    .ADDR_W(BUS_ADDR_W), .DATA_W(BUS_DATA_W),
                    .ID_W(CORE_BUS_ID + CORE_ID_W),
                    .DEPTH(SLAVE_MEM_DEPTH)
                ) u_mem (
                    .clk(clk), .rstn(rst_n),
                    .AWID(m_bus_awid[gi]), .AWADDR(m_bus_awaddr[gi]),
                    .AWLEN(m_bus_awlen[gi]), .AWBURST(m_bus_awburst[gi]),
                    .AWVALID(m_bus_awvalid[gi]), .AWREADY(m_bus_awready[gi]),
                    .WDATA(m_bus_wdata[gi]), .WSTRB(m_bus_wstrb[gi]),
                    .WLAST(m_bus_wlast[gi]), .WVALID(m_bus_wvalid[gi]),
                    .WREADY(m_bus_wready[gi]),
                    .BID(m_bus_bid[gi]), .BRESP(m_bus_bresp[gi]),
                    .BVALID(m_bus_bvalid[gi]), .BREADY(m_bus_bready[gi]),
                    .ARID(m_bus_arid[gi]), .ARADDR(m_bus_araddr[gi]),
                    .ARLEN(m_bus_arlen[gi]), .ARBURST(m_bus_arburst[gi]),
                    .ARVALID(m_bus_arvalid[gi]), .ARREADY(m_bus_arready[gi]),
                    .RID(m_bus_rid[gi]), .RDATA(m_bus_rdata[gi]),
                    .RRESP(m_bus_rresp[gi]),
                    .RLAST(m_bus_rlast[gi]), .RVALID(m_bus_rvalid[gi]),
                    .RREADY(m_bus_rready[gi])
                );
            end
            // sim 模式 m_axi_* tie 0 (TB 不连)
            assign m_axi_awid    = '0;  assign m_axi_awaddr  = '0;
            assign m_axi_awlen   = '0;  assign m_axi_awburst = '0;
            assign m_axi_awvalid = '0;  assign m_axi_wdata   = '0;
            assign m_axi_wstrb   = '0;  assign m_axi_wlast   = '0;
            assign m_axi_wvalid  = '0;  assign m_axi_bready  = '1;
            assign m_axi_arid    = '0;  assign m_axi_araddr  = '0;
            assign m_axi_arlen   = '0;  assign m_axi_arburst = '0;
            assign m_axi_arvalid = '0;  assign m_axi_rready  = '1;
        end else begin : g_synth_expose
            // synth 模式: 不实例化 mem stub, 把 m_bus_* (crossbar MI 端) 连接到 m_axi_* 出口
            // 真硬件 m_axi_* 接 board DDR 控制器 (e.g. MIG / axi_dm).
            for (gi = 0; gi < NUM_CORES; gi++) begin : g_axi_pack
                assign m_axi_awid   [gi*(CORE_BUS_ID+CORE_ID_W) +: CORE_BUS_ID+CORE_ID_W] = m_bus_awid[gi];
                assign m_axi_awaddr [gi*BUS_ADDR_W +: BUS_ADDR_W] = m_bus_awaddr[gi];
                assign m_axi_awlen  [gi*8 +: 8]                   = m_bus_awlen[gi];
                assign m_axi_awburst[gi*2 +: 2]                   = m_bus_awburst[gi];
                assign m_axi_awvalid[gi]                          = m_bus_awvalid[gi];
                assign m_bus_awready[gi]                          = m_axi_awready[gi];
                assign m_axi_wdata  [gi*BUS_DATA_W +: BUS_DATA_W] = m_bus_wdata[gi];
                assign m_axi_wstrb  [gi*(BUS_DATA_W/8) +: (BUS_DATA_W/8)] = m_bus_wstrb[gi];
                assign m_axi_wlast  [gi]                          = m_bus_wlast[gi];
                assign m_axi_wvalid [gi]                          = m_bus_wvalid[gi];
                assign m_bus_wready [gi]                          = m_axi_wready[gi];
                assign m_bus_bid    [gi] = m_axi_bid[gi*(CORE_BUS_ID+CORE_ID_W) +: CORE_BUS_ID+CORE_ID_W];
                assign m_bus_bresp  [gi] = m_axi_bresp[gi*2 +: 2];
                assign m_bus_bvalid [gi] = m_axi_bvalid[gi];
                assign m_axi_bready [gi] = m_bus_bready[gi];
                assign m_axi_arid   [gi*(CORE_BUS_ID+CORE_ID_W) +: CORE_BUS_ID+CORE_ID_W] = m_bus_arid[gi];
                assign m_axi_araddr [gi*BUS_ADDR_W +: BUS_ADDR_W] = m_bus_araddr[gi];
                assign m_axi_arlen  [gi*8 +: 8]                   = m_bus_arlen[gi];
                assign m_axi_arburst[gi*2 +: 2]                   = m_bus_arburst[gi];
                assign m_axi_arvalid[gi]                          = m_bus_arvalid[gi];
                assign m_bus_arready[gi]                          = m_axi_arready[gi];
                assign m_bus_rid    [gi] = m_axi_rid[gi*(CORE_BUS_ID+CORE_ID_W) +: CORE_BUS_ID+CORE_ID_W];
                assign m_bus_rdata  [gi] = m_axi_rdata[gi*BUS_DATA_W +: BUS_DATA_W];
                assign m_bus_rresp  [gi] = m_axi_rresp[gi*2 +: 2];
                assign m_bus_rlast  [gi] = m_axi_rlast[gi];
                assign m_bus_rvalid [gi] = m_axi_rvalid[gi];
                assign m_axi_rready [gi] = m_bus_rready[gi];
            end
        end
    endgenerate

endmodule
