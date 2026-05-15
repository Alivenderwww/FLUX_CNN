`timescale 1ns/1ps
`include "flux_cnn_params.svh"

// =============================================================================
// multicore_top_n4.sv  --  N=4 多核 + BD SmartConnect (无内部 SMC)
//
// 跟 multicore_top_smc.sv 区别:
//   - 不内嵌 axi_smc_4to4 IP (BD 顶层用 SmartConnect IP 代替)
//   - 不内嵌 axi_lite_1to4 IP (BD SmartConnect 自动 CSR fanout)
//   - 暴露 N=4 个独立 csr_axil + N=4 个独立 m_axi (一对一 per core)
//   - 跨核 IFB AXI (rmt_ifb) 仍 tie 0 (NUMA 模式)
//
// 用途: vd100 board demo N=4 多核 + 4 URAM(BD 内 emb_mem_gen) + SmartConnect 5×8
//   BD 内连:
//     CIPS.M_AXI_FPD ─┐
//     m_axi_0..3 ─────┤  SmartConnect ─→ csr_axil_0..3
//                     ├                ─→ axi_bram_ctrl_0..3 (URAM 1MB each)
//
// SG_MODE=1 (跟 vd100_minimal 一致, ConvCore IDMA/ODMA 走 SG cmd list).
// FLUX_MAC_SIMD: 综合时加 +define+FLUX_MAC_SIMD 开启 DSP48E1 INT8 SIMD MAC.
// =============================================================================

module multicore_top_n4 #(
    parameter int NUM_CORES   = 4,
    parameter int CSR_DATA_W  = 32,
    parameter int CSR_ADDR_W  = 12,   // per-core 12-bit (BD SmartConnect 路由)
    parameter int BUS_ADDR_W  = `FLUX_BUS_ADDR_W,              // 32
    parameter int BUS_DATA_W  = `FLUX_BUS_DATA_W,              // 128
    parameter int CORE_BUS_ID = `FLUX_AXI_M_ID + `FLUX_AXI_M_WIDTH  // 4
)(
    input  logic                        clk,
    input  logic                        rst_n,

    // ---- AXI-Lite Slave × N (host CSR per core, 12-bit addr) ----
    input  logic [NUM_CORES*CSR_ADDR_W-1:0]      csr_axil_awaddr,
    input  logic [NUM_CORES-1:0]                 csr_axil_awvalid,
    output logic [NUM_CORES-1:0]                 csr_axil_awready,
    input  logic [NUM_CORES*CSR_DATA_W-1:0]      csr_axil_wdata,
    input  logic [NUM_CORES*(CSR_DATA_W/8)-1:0]  csr_axil_wstrb,
    input  logic [NUM_CORES-1:0]                 csr_axil_wvalid,
    output logic [NUM_CORES-1:0]                 csr_axil_wready,
    output logic [NUM_CORES*2-1:0]               csr_axil_bresp,
    output logic [NUM_CORES-1:0]                 csr_axil_bvalid,
    input  logic [NUM_CORES-1:0]                 csr_axil_bready,
    input  logic [NUM_CORES*CSR_ADDR_W-1:0]      csr_axil_araddr,
    input  logic [NUM_CORES-1:0]                 csr_axil_arvalid,
    output logic [NUM_CORES-1:0]                 csr_axil_arready,
    output logic [NUM_CORES*CSR_DATA_W-1:0]      csr_axil_rdata,
    output logic [NUM_CORES*2-1:0]               csr_axil_rresp,
    output logic [NUM_CORES-1:0]                 csr_axil_rvalid,
    input  logic [NUM_CORES-1:0]                 csr_axil_rready,

    // ---- AXI4 Master × N (per-core m_axi, 接 BD SmartConnect → URAM) ----
    output logic [NUM_CORES*CORE_BUS_ID-1:0]     m_axi_awid,
    output logic [NUM_CORES*BUS_ADDR_W-1:0]      m_axi_awaddr,
    output logic [NUM_CORES*8-1:0]               m_axi_awlen,
    output logic [NUM_CORES*3-1:0]               m_axi_awsize,
    output logic [NUM_CORES*2-1:0]               m_axi_awburst,
    output logic [NUM_CORES-1:0]                 m_axi_awlock,
    output logic [NUM_CORES*4-1:0]               m_axi_awcache,
    output logic [NUM_CORES*3-1:0]               m_axi_awprot,
    output logic [NUM_CORES*4-1:0]               m_axi_awqos,
    output logic [NUM_CORES-1:0]                 m_axi_awvalid,
    input  logic [NUM_CORES-1:0]                 m_axi_awready,
    output logic [NUM_CORES*BUS_DATA_W-1:0]      m_axi_wdata,
    output logic [NUM_CORES*(BUS_DATA_W/8)-1:0]  m_axi_wstrb,
    output logic [NUM_CORES-1:0]                 m_axi_wlast,
    output logic [NUM_CORES-1:0]                 m_axi_wvalid,
    input  logic [NUM_CORES-1:0]                 m_axi_wready,
    input  logic [NUM_CORES*CORE_BUS_ID-1:0]     m_axi_bid,
    input  logic [NUM_CORES*2-1:0]               m_axi_bresp,
    input  logic [NUM_CORES-1:0]                 m_axi_bvalid,
    output logic [NUM_CORES-1:0]                 m_axi_bready,
    output logic [NUM_CORES*CORE_BUS_ID-1:0]     m_axi_arid,
    output logic [NUM_CORES*BUS_ADDR_W-1:0]      m_axi_araddr,
    output logic [NUM_CORES*8-1:0]               m_axi_arlen,
    output logic [NUM_CORES*3-1:0]               m_axi_arsize,
    output logic [NUM_CORES*2-1:0]               m_axi_arburst,
    output logic [NUM_CORES-1:0]                 m_axi_arlock,
    output logic [NUM_CORES*4-1:0]               m_axi_arcache,
    output logic [NUM_CORES*3-1:0]               m_axi_arprot,
    output logic [NUM_CORES*4-1:0]               m_axi_arqos,
    output logic [NUM_CORES-1:0]                 m_axi_arvalid,
    input  logic [NUM_CORES-1:0]                 m_axi_arready,
    input  logic [NUM_CORES*CORE_BUS_ID-1:0]     m_axi_rid,
    input  logic [NUM_CORES*BUS_DATA_W-1:0]      m_axi_rdata,
    input  logic [NUM_CORES*2-1:0]               m_axi_rresp,
    input  logic [NUM_CORES-1:0]                 m_axi_rlast,
    input  logic [NUM_CORES-1:0]                 m_axi_rvalid,
    output logic [NUM_CORES-1:0]                 m_axi_rready,

    // ---- IRQ: 任一核 done sticky 拉高 ----
    output logic                                 irq_done
);

    localparam int RMT_ID_W = CORE_BUS_ID + 1;

    logic [NUM_CORES-1:0] core_done;
    assign irq_done = |core_done;   // 任一核 done → IRQ 高

    // -------------------------------------------------------------------------
    // 实例化 NUM_CORES 个 ConvCore (generate-for)
    // -------------------------------------------------------------------------
    genvar gi;
    generate
        for (gi = 0; gi < NUM_CORES; gi++) begin : g_core
            core_top #(
                .SG_MODE  (1),
                .RMT_ID_W (RMT_ID_W)
            ) u_core (
                .clk            (clk),
                .rst_n          (rst_n),

                // CSR slave (12-bit per core)
                .csr_awaddr     (csr_axil_awaddr [gi*CSR_ADDR_W +: CSR_ADDR_W]),
                .csr_awvalid    (csr_axil_awvalid[gi]),
                .csr_awready    (csr_axil_awready[gi]),
                .csr_wdata      (csr_axil_wdata  [gi*CSR_DATA_W +: CSR_DATA_W]),
                .csr_wstrb      (csr_axil_wstrb  [gi*(CSR_DATA_W/8) +: (CSR_DATA_W/8)]),
                .csr_wvalid     (csr_axil_wvalid [gi]),
                .csr_wready     (csr_axil_wready [gi]),
                .csr_bresp      (csr_axil_bresp  [gi*2 +: 2]),
                .csr_bvalid     (csr_axil_bvalid [gi]),
                .csr_bready     (csr_axil_bready [gi]),
                .csr_araddr     (csr_axil_araddr [gi*CSR_ADDR_W +: CSR_ADDR_W]),
                .csr_arvalid    (csr_axil_arvalid[gi]),
                .csr_arready    (csr_axil_arready[gi]),
                .csr_rdata      (csr_axil_rdata  [gi*CSR_DATA_W +: CSR_DATA_W]),
                .csr_rresp      (csr_axil_rresp  [gi*2 +: 2]),
                .csr_rvalid     (csr_axil_rvalid [gi]),
                .csr_rready     (csr_axil_rready [gi]),

                // AXI4 master (per-core m_axi, BD SmartConnect 路由)
                .bus_awid       (m_axi_awid    [gi*CORE_BUS_ID +: CORE_BUS_ID]),
                .bus_awaddr     (m_axi_awaddr  [gi*BUS_ADDR_W +: BUS_ADDR_W]),
                .bus_awlen      (m_axi_awlen   [gi*8 +: 8]),
                .bus_awsize     (m_axi_awsize  [gi*3 +: 3]),
                .bus_awburst    (m_axi_awburst [gi*2 +: 2]),
                .bus_awlock     (m_axi_awlock  [gi]),
                .bus_awcache    (m_axi_awcache [gi*4 +: 4]),
                .bus_awprot     (m_axi_awprot  [gi*3 +: 3]),
                .bus_awqos      (m_axi_awqos   [gi*4 +: 4]),
                .bus_awvalid    (m_axi_awvalid [gi]),
                .bus_awready    (m_axi_awready [gi]),
                .bus_wdata      (m_axi_wdata   [gi*BUS_DATA_W +: BUS_DATA_W]),
                .bus_wstrb      (m_axi_wstrb   [gi*(BUS_DATA_W/8) +: (BUS_DATA_W/8)]),
                .bus_wlast      (m_axi_wlast   [gi]),
                .bus_wvalid     (m_axi_wvalid  [gi]),
                .bus_wready     (m_axi_wready  [gi]),
                .bus_bid        (m_axi_bid     [gi*CORE_BUS_ID +: CORE_BUS_ID]),
                .bus_bresp      (m_axi_bresp   [gi*2 +: 2]),
                .bus_bvalid     (m_axi_bvalid  [gi]),
                .bus_bready     (m_axi_bready  [gi]),
                .bus_arid       (m_axi_arid    [gi*CORE_BUS_ID +: CORE_BUS_ID]),
                .bus_araddr     (m_axi_araddr  [gi*BUS_ADDR_W +: BUS_ADDR_W]),
                .bus_arlen      (m_axi_arlen   [gi*8 +: 8]),
                .bus_arsize     (m_axi_arsize  [gi*3 +: 3]),
                .bus_arburst    (m_axi_arburst [gi*2 +: 2]),
                .bus_arlock     (m_axi_arlock  [gi]),
                .bus_arcache    (m_axi_arcache [gi*4 +: 4]),
                .bus_arprot     (m_axi_arprot  [gi*3 +: 3]),
                .bus_arqos      (m_axi_arqos   [gi*4 +: 4]),
                .bus_arvalid    (m_axi_arvalid [gi]),
                .bus_arready    (m_axi_arready [gi]),
                .bus_rid        (m_axi_rid     [gi*CORE_BUS_ID +: CORE_BUS_ID]),
                .bus_rdata      (m_axi_rdata   [gi*BUS_DATA_W +: BUS_DATA_W]),
                .bus_rresp      (m_axi_rresp   [gi*2 +: 2]),
                .bus_rlast      (m_axi_rlast   [gi]),
                .bus_rvalid     (m_axi_rvalid  [gi]),
                .bus_rready     (m_axi_rready  [gi]),

                // 跨核 IFB AXI4 SI: NUMA 模式不用, 全 tie 0
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
                .rmt_ifb_awready(),
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

                // 外部 SRAM 测试口: 不用
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

                // per-core done sticky
                .done           (core_done[gi]),

                // Mesh OFM tdest/opcode: 不用
                .ofm_tdest      (),
                .ofm_opcode     ()
            );
        end
    endgenerate

endmodule
