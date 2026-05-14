`timescale 1ns/1ps
`include "flux_cnn_params.svh"

// =============================================================================
// multicore_top_minimal.sv  --  Minimum bring-up wrapper (单核, BRAM-only)
//
// 目的: bring-up 用最小系统. 跟 multicore_top_vd100 区别:
//   - NUM_CORES = 1 (不跨核, 不用 mesh)
//   - 跨核 IFB AXI4 SI 端口全部 tie 0 (不接受远端 push)
//   - 不暴露 SRAM 直接 read/write 测试口 (board 不用)
//   - 不暴露 mesh OFM tdest/opcode
//   - 顶层 1 个 AXI-Lite slave (csr_axil) + 1 个 AXI4 master (m_axi)
//
// 用途: Stage 1 bring-up — ConvCore m_axi → smartconnect → axi_bram_ctrl →
//       BRAM (vendor IP, 已知好使). 完全绕开 axi_noc + PL DDR4 + mesh + CIPS A72.
//
// 控制流: PC ─JTAG─► vivado HW Manager run_hw_axi
//          ├ jtag_to_axi_csr  ─► csr_axil  (写 ConvCore CSR)
//          └ jtag_to_axi_bulk ─► smartconnect ─► axi_bram_ctrl ─► BRAM
//                                       ▲
//                            ConvCore.m_axi (走 SG mode 拉 desc + 数据)
//
// SG_MODE=1 (跟 board PDI 一致, idma_sg_dispatcher + odma_sg_dispatcher).
// =============================================================================

module multicore_top_minimal #(
    parameter int CSR_DATA_W  = 32,
    parameter int CSR_ADDR_W  = 12,                            // 单核 csr 12-bit (跟 ConvCore.csr_awaddr 一致)
    parameter int BUS_ADDR_W  = `FLUX_BUS_ADDR_W,              // 32
    parameter int BUS_DATA_W  = `FLUX_BUS_DATA_W,              // 128
    parameter int CORE_BUS_ID = `FLUX_AXI_M_ID + `FLUX_AXI_M_WIDTH  // 4 (跟 core_top.bus_*id 一致)
)(
    input  logic                        clk,
    input  logic                        rst_n,

    // ---- AXI-Lite Slave (host CSR) ----
    input  logic [CSR_ADDR_W-1:0]       csr_axil_awaddr,
    input  logic                        csr_axil_awvalid,
    output logic                        csr_axil_awready,
    input  logic [CSR_DATA_W-1:0]       csr_axil_wdata,
    input  logic [CSR_DATA_W/8-1:0]     csr_axil_wstrb,
    input  logic                        csr_axil_wvalid,
    output logic                        csr_axil_wready,
    output logic [1:0]                  csr_axil_bresp,
    output logic                        csr_axil_bvalid,
    input  logic                        csr_axil_bready,
    input  logic [CSR_ADDR_W-1:0]       csr_axil_araddr,
    input  logic                        csr_axil_arvalid,
    output logic                        csr_axil_arready,
    output logic [CSR_DATA_W-1:0]       csr_axil_rdata,
    output logic [1:0]                  csr_axil_rresp,
    output logic                        csr_axil_rvalid,
    input  logic                        csr_axil_rready,

    // ---- AXI4 Master (ConvCore 出, 接 smartconnect → axi_bram_ctrl) ----
    output logic [CORE_BUS_ID-1:0]      m_axi_awid,
    output logic [BUS_ADDR_W-1:0]       m_axi_awaddr,
    output logic [7:0]                  m_axi_awlen,
    output logic [2:0]                  m_axi_awsize,
    output logic [1:0]                  m_axi_awburst,
    output logic                        m_axi_awlock,
    output logic [3:0]                  m_axi_awcache,
    output logic [2:0]                  m_axi_awprot,
    output logic [3:0]                  m_axi_awqos,
    output logic                        m_axi_awvalid,
    input  logic                        m_axi_awready,
    output logic [BUS_DATA_W-1:0]       m_axi_wdata,
    output logic [BUS_DATA_W/8-1:0]     m_axi_wstrb,
    output logic                        m_axi_wlast,
    output logic                        m_axi_wvalid,
    input  logic                        m_axi_wready,
    input  logic [CORE_BUS_ID-1:0]      m_axi_bid,
    input  logic [1:0]                  m_axi_bresp,
    input  logic                        m_axi_bvalid,
    output logic                        m_axi_bready,
    output logic [CORE_BUS_ID-1:0]      m_axi_arid,
    output logic [BUS_ADDR_W-1:0]       m_axi_araddr,
    output logic [7:0]                  m_axi_arlen,
    output logic [2:0]                  m_axi_arsize,
    output logic [1:0]                  m_axi_arburst,
    output logic                        m_axi_arlock,
    output logic [3:0]                  m_axi_arcache,
    output logic [2:0]                  m_axi_arprot,
    output logic [3:0]                  m_axi_arqos,
    output logic                        m_axi_arvalid,
    input  logic                        m_axi_arready,
    input  logic [CORE_BUS_ID-1:0]      m_axi_rid,
    input  logic [BUS_DATA_W-1:0]       m_axi_rdata,
    input  logic [1:0]                  m_axi_rresp,
    input  logic                        m_axi_rlast,
    input  logic                        m_axi_rvalid,
    output logic                        m_axi_rready,

    // ---- 调试 / IRQ (顶层 done sticky) ----
    output logic                        irq_done
);

    // -------------------------------------------------------------------------
    // 实例化 1 个 ConvCore (SG_MODE=1 用 SG dispatcher, 跟 board 一致)
    // -------------------------------------------------------------------------
    localparam int RMT_ID_W = CORE_BUS_ID + 1;   // 跨核 IFB SI ID 宽, 单核不用 但要给值

    core_top #(
        .SG_MODE  (1),
        .RMT_ID_W (RMT_ID_W)
    ) u_core (
        .clk            (clk),
        .rst_n          (rst_n),

        // CSR slave
        .csr_awaddr     (csr_axil_awaddr),
        .csr_awvalid    (csr_axil_awvalid),
        .csr_awready    (csr_axil_awready),
        .csr_wdata      (csr_axil_wdata),
        .csr_wstrb      (csr_axil_wstrb),
        .csr_wvalid     (csr_axil_wvalid),
        .csr_wready     (csr_axil_wready),
        .csr_bresp      (csr_axil_bresp),
        .csr_bvalid     (csr_axil_bvalid),
        .csr_bready     (csr_axil_bready),
        .csr_araddr     (csr_axil_araddr),
        .csr_arvalid    (csr_axil_arvalid),
        .csr_arready    (csr_axil_arready),
        .csr_rdata      (csr_axil_rdata),
        .csr_rresp      (csr_axil_rresp),
        .csr_rvalid     (csr_axil_rvalid),
        .csr_rready     (csr_axil_rready),

        // AXI master (直通到顶层)
        .bus_awid       (m_axi_awid),
        .bus_awaddr     (m_axi_awaddr),
        .bus_awlen      (m_axi_awlen),
        .bus_awsize     (m_axi_awsize),
        .bus_awburst    (m_axi_awburst),
        .bus_awlock     (m_axi_awlock),
        .bus_awcache    (m_axi_awcache),
        .bus_awprot     (m_axi_awprot),
        .bus_awqos      (m_axi_awqos),
        .bus_awvalid    (m_axi_awvalid),
        .bus_awready    (m_axi_awready),
        .bus_wdata      (m_axi_wdata),
        .bus_wstrb      (m_axi_wstrb),
        .bus_wlast      (m_axi_wlast),
        .bus_wvalid     (m_axi_wvalid),
        .bus_wready     (m_axi_wready),
        .bus_bid        (m_axi_bid),
        .bus_bresp      (m_axi_bresp),
        .bus_bvalid     (m_axi_bvalid),
        .bus_bready     (m_axi_bready),
        .bus_arid       (m_axi_arid),
        .bus_araddr     (m_axi_araddr),
        .bus_arlen      (m_axi_arlen),
        .bus_arsize     (m_axi_arsize),
        .bus_arburst    (m_axi_arburst),
        .bus_arlock     (m_axi_arlock),
        .bus_arcache    (m_axi_arcache),
        .bus_arprot     (m_axi_arprot),
        .bus_arqos      (m_axi_arqos),
        .bus_arvalid    (m_axi_arvalid),
        .bus_arready    (m_axi_arready),
        .bus_rid        (m_axi_rid),
        .bus_rdata      (m_axi_rdata),
        .bus_rresp      (m_axi_rresp),
        .bus_rlast      (m_axi_rlast),
        .bus_rvalid     (m_axi_rvalid),
        .bus_rready     (m_axi_rready),

        // 跨核 IFB AXI4 SI: 单核不需要, 全 tie 0
        .rmt_ifb_awid   ('0),
        .rmt_ifb_awaddr ('0),
        .rmt_ifb_awlen  ('0),
        .rmt_ifb_awsize ('0),
        .rmt_ifb_awburst('0),
        .rmt_ifb_awlock ('0),
        .rmt_ifb_awcache('0),
        .rmt_ifb_awprot ('0),
        .rmt_ifb_awqos  ('0),
        .rmt_ifb_awvalid(1'b0),
        .rmt_ifb_awready(),    // 输出, leave open
        .rmt_ifb_wdata  ('0),
        .rmt_ifb_wstrb  ('0),
        .rmt_ifb_wlast  (1'b0),
        .rmt_ifb_wvalid (1'b0),
        .rmt_ifb_wready (),
        .rmt_ifb_bid    (),
        .rmt_ifb_bresp  (),
        .rmt_ifb_bvalid (),
        .rmt_ifb_bready (1'b1),
        .rmt_ifb_arid   ('0),
        .rmt_ifb_araddr ('0),
        .rmt_ifb_arlen  ('0),
        .rmt_ifb_arsize ('0),
        .rmt_ifb_arburst('0),
        .rmt_ifb_arlock (1'b0),
        .rmt_ifb_arcache('0),
        .rmt_ifb_arprot ('0),
        .rmt_ifb_arqos  ('0),
        .rmt_ifb_arvalid(1'b0),
        .rmt_ifb_arready(),
        .rmt_ifb_rid    (),
        .rmt_ifb_rdata  (),
        .rmt_ifb_rresp  (),
        .rmt_ifb_rlast  (),
        .rmt_ifb_rvalid (),
        .rmt_ifb_rready (1'b1),

        // 外部 SRAM 测试口 (board 不用)
        .ifb_we_ext     (1'b0),
        .ifb_waddr_ext  ('0),
        .ifb_wdata_ext  ('0),
        .wb_we_ext      (1'b0),
        .wb_waddr_ext   ('0),
        .wb_wdata_ext   ('0),
        .ofb_re_ext     (1'b0),
        .ofb_raddr_ext  ('0),
        .ofb_rdata_ext  (),
        .psum_out_vec   (),

        // 顶层 done
        .done           (irq_done),

        // Mesh OFM tdest/opcode: 单核不用, leave open
        .ofm_tdest      (),
        .ofm_opcode     ()
    );

    // 检查 ConvCore 内部 AXI ID 宽度跟顶层一致
    // (core_top 内部 AXI_M_ID + AXI_M_WIDTH = 4, 跟 CORE_BUS_ID 一致)

endmodule
