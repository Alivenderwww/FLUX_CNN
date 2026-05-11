`timescale 1ns/1ps
`include "flux_cnn_params.svh"

// =============================================================================
// multicore_top_vd100_bd.sv  --  BD 友好 wrapper
//
// 把 multicore_top_vd100 的 packed AXI 端口拆成 3 套独立 AXI4 master + 1 个 AXI-Lite
// slave 的命名风格端口, 加 Xilinx X_INTERFACE_INFO attribute 让 Vivado BD 自动识别
// 成 AXI interface.
//
// BD 端 connect_bd_intf_net 直接连 axi_noc.S00_AXI / S01_AXI / S02_AXI 跟
// smartconnect.M00_AXI 即可.
// =============================================================================

(* CORE_GENERATION_INFO = "multicore_top_vd100_bd,multicore_top_vd100,{NUM_CORES=3}" *)
module multicore_top_vd100_bd (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF csr_axil:m00_axi:m01_axi:m02_axi, ASSOCIATED_RESET rst_n" *)
    input  logic                clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 rst_n RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  logic                rst_n,

    // ====== CSR AXI-Lite slave (来自 PS GP via smartconnect) ======
    (* X_INTERFACE_PARAMETER = "MODE Slave, ADDR_WIDTH 14, DATA_WIDTH 32, PROTOCOL AXI4LITE, FREQ_HZ 100000000, ID_WIDTH 0, AWUSER_WIDTH 0, ARUSER_WIDTH 0, WUSER_WIDTH 0, RUSER_WIDTH 0, BUSER_WIDTH 0, READ_WRITE_MODE READ_WRITE, HAS_BURST 0, HAS_LOCK 0, HAS_PROT 0, HAS_CACHE 0, HAS_QOS 0, HAS_REGION 0, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 1, NUM_READ_OUTSTANDING 1, NUM_WRITE_OUTSTANDING 1, MAX_BURST_LENGTH 1" *)
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil AWADDR" *) input  logic [13:0] csr_axil_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil AWVALID" *) input  logic        csr_axil_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil AWREADY" *) output logic        csr_axil_awready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil WDATA" *)   input  logic [31:0] csr_axil_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil WSTRB" *)   input  logic [3:0]  csr_axil_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil WVALID" *)  input  logic        csr_axil_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil WREADY" *)  output logic        csr_axil_wready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil BRESP" *)   output logic [1:0]  csr_axil_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil BVALID" *)  output logic        csr_axil_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil BREADY" *)  input  logic        csr_axil_bready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil ARADDR" *)  input  logic [13:0] csr_axil_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil ARVALID" *) input  logic        csr_axil_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil ARREADY" *) output logic        csr_axil_arready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil RDATA" *)   output logic [31:0] csr_axil_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil RRESP" *)   output logic [1:0]  csr_axil_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil RVALID" *)  output logic        csr_axil_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil RREADY" *)  input  logic        csr_axil_rready,

    // ====== M00_AXI master (Core 0 → axi_noc.S00_AXI) ======
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi AWID"    *) output logic [4:0]   m00_axi_awid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi AWADDR"  *) output logic [31:0]  m00_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi AWLEN"   *) output logic [7:0]   m00_axi_awlen,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi AWSIZE"  *) output logic [2:0]   m00_axi_awsize,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi AWBURST" *) output logic [1:0]   m00_axi_awburst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi AWLOCK"  *) output logic         m00_axi_awlock,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi AWCACHE" *) output logic [3:0]   m00_axi_awcache,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi AWPROT"  *) output logic [2:0]   m00_axi_awprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi AWQOS"   *) output logic [3:0]   m00_axi_awqos,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi AWVALID" *) output logic         m00_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi AWREADY" *) input  logic         m00_axi_awready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi WDATA"   *) output logic [127:0] m00_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi WSTRB"   *) output logic [15:0]  m00_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi WLAST"   *) output logic         m00_axi_wlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi WVALID"  *) output logic         m00_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi WREADY"  *) input  logic         m00_axi_wready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi BID"     *) input  logic [4:0]   m00_axi_bid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi BRESP"   *) input  logic [1:0]   m00_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi BVALID"  *) input  logic         m00_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi BREADY"  *) output logic         m00_axi_bready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi ARID"    *) output logic [4:0]   m00_axi_arid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi ARADDR"  *) output logic [31:0]  m00_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi ARLEN"   *) output logic [7:0]   m00_axi_arlen,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi ARSIZE"  *) output logic [2:0]   m00_axi_arsize,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi ARBURST" *) output logic [1:0]   m00_axi_arburst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi ARLOCK"  *) output logic         m00_axi_arlock,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi ARCACHE" *) output logic [3:0]   m00_axi_arcache,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi ARPROT"  *) output logic [2:0]   m00_axi_arprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi ARQOS"   *) output logic [3:0]   m00_axi_arqos,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi ARVALID" *) output logic         m00_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi ARREADY" *) input  logic         m00_axi_arready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi RID"     *) input  logic [4:0]   m00_axi_rid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi RDATA"   *) input  logic [127:0] m00_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi RRESP"   *) input  logic [1:0]   m00_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi RLAST"   *) input  logic         m00_axi_rlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi RVALID"  *) input  logic         m00_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m00_axi RREADY"  *) output logic         m00_axi_rready,

    // ====== M01_AXI master (Core 1) ======
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi AWID"    *) output logic [4:0]   m01_axi_awid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi AWADDR"  *) output logic [31:0]  m01_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi AWLEN"   *) output logic [7:0]   m01_axi_awlen,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi AWSIZE"  *) output logic [2:0]   m01_axi_awsize,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi AWBURST" *) output logic [1:0]   m01_axi_awburst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi AWLOCK"  *) output logic         m01_axi_awlock,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi AWCACHE" *) output logic [3:0]   m01_axi_awcache,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi AWPROT"  *) output logic [2:0]   m01_axi_awprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi AWQOS"   *) output logic [3:0]   m01_axi_awqos,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi AWVALID" *) output logic         m01_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi AWREADY" *) input  logic         m01_axi_awready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi WDATA"   *) output logic [127:0] m01_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi WSTRB"   *) output logic [15:0]  m01_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi WLAST"   *) output logic         m01_axi_wlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi WVALID"  *) output logic         m01_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi WREADY"  *) input  logic         m01_axi_wready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi BID"     *) input  logic [4:0]   m01_axi_bid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi BRESP"   *) input  logic [1:0]   m01_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi BVALID"  *) input  logic         m01_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi BREADY"  *) output logic         m01_axi_bready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi ARID"    *) output logic [4:0]   m01_axi_arid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi ARADDR"  *) output logic [31:0]  m01_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi ARLEN"   *) output logic [7:0]   m01_axi_arlen,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi ARSIZE"  *) output logic [2:0]   m01_axi_arsize,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi ARBURST" *) output logic [1:0]   m01_axi_arburst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi ARLOCK"  *) output logic         m01_axi_arlock,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi ARCACHE" *) output logic [3:0]   m01_axi_arcache,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi ARPROT"  *) output logic [2:0]   m01_axi_arprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi ARQOS"   *) output logic [3:0]   m01_axi_arqos,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi ARVALID" *) output logic         m01_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi ARREADY" *) input  logic         m01_axi_arready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi RID"     *) input  logic [4:0]   m01_axi_rid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi RDATA"   *) input  logic [127:0] m01_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi RRESP"   *) input  logic [1:0]   m01_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi RLAST"   *) input  logic         m01_axi_rlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi RVALID"  *) input  logic         m01_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m01_axi RREADY"  *) output logic         m01_axi_rready,

    // ====== M02_AXI master (Core 2) ======
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi AWID"    *) output logic [4:0]   m02_axi_awid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi AWADDR"  *) output logic [31:0]  m02_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi AWLEN"   *) output logic [7:0]   m02_axi_awlen,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi AWSIZE"  *) output logic [2:0]   m02_axi_awsize,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi AWBURST" *) output logic [1:0]   m02_axi_awburst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi AWLOCK"  *) output logic         m02_axi_awlock,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi AWCACHE" *) output logic [3:0]   m02_axi_awcache,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi AWPROT"  *) output logic [2:0]   m02_axi_awprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi AWQOS"   *) output logic [3:0]   m02_axi_awqos,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi AWVALID" *) output logic         m02_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi AWREADY" *) input  logic         m02_axi_awready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi WDATA"   *) output logic [127:0] m02_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi WSTRB"   *) output logic [15:0]  m02_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi WLAST"   *) output logic         m02_axi_wlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi WVALID"  *) output logic         m02_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi WREADY"  *) input  logic         m02_axi_wready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi BID"     *) input  logic [4:0]   m02_axi_bid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi BRESP"   *) input  logic [1:0]   m02_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi BVALID"  *) input  logic         m02_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi BREADY"  *) output logic         m02_axi_bready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi ARID"    *) output logic [4:0]   m02_axi_arid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi ARADDR"  *) output logic [31:0]  m02_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi ARLEN"   *) output logic [7:0]   m02_axi_arlen,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi ARSIZE"  *) output logic [2:0]   m02_axi_arsize,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi ARBURST" *) output logic [1:0]   m02_axi_arburst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi ARLOCK"  *) output logic         m02_axi_arlock,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi ARCACHE" *) output logic [3:0]   m02_axi_arcache,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi ARPROT"  *) output logic [2:0]   m02_axi_arprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi ARQOS"   *) output logic [3:0]   m02_axi_arqos,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi ARVALID" *) output logic         m02_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi ARREADY" *) input  logic         m02_axi_arready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi RID"     *) input  logic [4:0]   m02_axi_rid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi RDATA"   *) input  logic [127:0] m02_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi RRESP"   *) input  logic [1:0]   m02_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi RLAST"   *) input  logic         m02_axi_rlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi RVALID"  *) input  logic         m02_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m02_axi RREADY"  *) output logic         m02_axi_rready,

    // ====== IRQ output (3-bit done → PS GIC) ======
    (* X_INTERFACE_INFO = "xilinx.com:signal:interrupt:1.0 irq_done INTERRUPT" *)
    output logic [2:0]          irq_done,

    // ====== Debug expose (m00_axi 内部信号到顶层, BD 端 axis_ila 接) ======
    output logic                 dbg_awvalid_0,
    output logic                 dbg_awready_0,
    output logic                 dbg_arvalid_0,
    output logic                 dbg_arready_0,
    output logic                 dbg_wvalid_0,
    output logic                 dbg_rvalid_0,
    output logic [31:0]          dbg_awaddr_0,
    output logic [31:0]          dbg_araddr_0,
    output logic [3:0]           dbg_awid_0,
    output logic [3:0]           dbg_arid_0,
    output logic [7:0]           dbg_awlen_0,
    output logic [7:0]           dbg_arlen_0,
    // === 第二组: dfe + axi_arbiter 内部信号 (v18 原配, 撤销 v19 sequencer 改动) ===
    output logic [1:0]           dbg_dfe_state_0,
    output logic                 dbg_dfe_m_arvalid_0,
    output logic                 dbg_dfe_m_arready_0,
    output logic                 dbg_dfe_r_phase_0,
    output logic                 dbg_dfe_r_done_0,
    output logic [15:0]          dbg_dfe_r_beats_rcvd_0,
    output logic                 dbg_arb_rd_lock_0,
    output logic [1:0]           dbg_arb_rd_sel_0,
    output logic [1:0]           dbg_arb_cu_rd_sel_0,
    output logic [3:0]           dbg_master_arvalid_0,
    output logic [3:0]           dbg_master_arready_0,
    output logic [3:0]           dbg_master_rvalid_0,
    output logic                 dbg_start_dfe_pulse_0,
    output logic                 dbg_dfe_busy_0,
    output logic                 dbg_dfe_done_0,
    output logic                 dbg_idma_busy_0,
    output logic                 dbg_layer_busy_0,
    output logic                 dbg_csr_aw_fire_0  // PS GP 写 CSR 检测
);

    // 内部 packed vector 信号 (跟 multicore_top_vd100 端口对齐).
    // ID 宽度 = CORE_BUS_ID + CORE_ID_W = 2 + 2 = 4 bit / core (FLUX_AXI_M_ID=2).
    // m00..m02_axi 端口对外声明 5 bit 是 axi_noc 期望. 拆 / 拼时高位补 0.
    logic [3*4-1:0]    iw_awid,    iw_arid,    iw_bid,    iw_rid;
    logic [3*32-1:0]   iw_awaddr,  iw_araddr;
    logic [3*8-1:0]    iw_awlen,   iw_arlen;
    logic [3*3-1:0]    iw_awsize,  iw_arsize;
    logic [3*2-1:0]    iw_awburst, iw_arburst, iw_bresp, iw_rresp;
    logic [3-1:0]      iw_awlock,  iw_arlock;
    logic [3*4-1:0]    iw_awcache, iw_arcache;
    logic [3*3-1:0]    iw_awprot,  iw_arprot;
    logic [3*4-1:0]    iw_awqos,   iw_arqos;
    logic [3-1:0]      iw_awvalid, iw_awready, iw_wlast, iw_wvalid, iw_wready;
    logic [3-1:0]      iw_bvalid,  iw_bready,  iw_arvalid, iw_arready;
    logic [3-1:0]      iw_rlast,   iw_rvalid,  iw_rready;
    logic [3*128-1:0]  iw_wdata,   iw_rdata;
    logic [3*16-1:0]   iw_wstrb;

    // ===== M00 → packed vector slot 0 (ID 4→5 bit zero-extend, response 5→4 bit truncate) =====
    assign m00_axi_awid    = {1'b0, iw_awid[0*4 +:4]}; assign m00_axi_arid    = {1'b0, iw_arid[0*4 +:4]};
    assign m00_axi_awaddr  = iw_awaddr  [0*32 +:32];  assign m00_axi_araddr  = iw_araddr  [0*32 +:32];
    assign m00_axi_awlen   = iw_awlen   [0*8  +:8];   assign m00_axi_arlen   = iw_arlen   [0*8  +:8];
    assign m00_axi_awsize  = iw_awsize  [0*3  +:3];   assign m00_axi_arsize  = iw_arsize  [0*3  +:3];
    assign m00_axi_awburst = iw_awburst [0*2  +:2];   assign m00_axi_arburst = iw_arburst [0*2  +:2];
    assign m00_axi_awlock  = iw_awlock  [0];          assign m00_axi_arlock  = iw_arlock  [0];
    assign m00_axi_awcache = iw_awcache [0*4  +:4];   assign m00_axi_arcache = iw_arcache [0*4  +:4];
    assign m00_axi_awprot  = iw_awprot  [0*3  +:3];   assign m00_axi_arprot  = iw_arprot  [0*3  +:3];
    assign m00_axi_awqos   = iw_awqos   [0*4  +:4];   assign m00_axi_arqos   = iw_arqos   [0*4  +:4];
    assign m00_axi_awvalid = iw_awvalid [0];          assign m00_axi_arvalid = iw_arvalid [0];
    assign iw_awready[0]   = m00_axi_awready;         assign iw_arready[0]   = m00_axi_arready;
    assign m00_axi_wdata   = iw_wdata   [0*128 +:128];assign m00_axi_wstrb   = iw_wstrb   [0*16 +:16];
    assign m00_axi_wlast   = iw_wlast   [0];          assign m00_axi_wvalid  = iw_wvalid  [0];
    assign iw_wready[0]    = m00_axi_wready;
    assign iw_bid[0*4 +:4] = m00_axi_bid[3:0];        assign iw_bresp[0*2 +:2] = m00_axi_bresp;
    assign iw_bvalid[0]    = m00_axi_bvalid;          assign m00_axi_bready  = iw_bready  [0];
    assign iw_rid[0*4 +:4] = m00_axi_rid[3:0];        assign iw_rdata[0*128 +:128] = m00_axi_rdata;
    assign iw_rresp[0*2 +:2] = m00_axi_rresp;         assign iw_rlast[0]     = m00_axi_rlast;
    assign iw_rvalid[0]    = m00_axi_rvalid;          assign m00_axi_rready  = iw_rready  [0];

    // ===== M01 → packed vector slot 1 (4→5 bit) =====
    assign m01_axi_awid    = {1'b0, iw_awid[1*4 +:4]}; assign m01_axi_arid    = {1'b0, iw_arid[1*4 +:4]};
    assign m01_axi_awaddr  = iw_awaddr  [1*32 +:32];  assign m01_axi_araddr  = iw_araddr  [1*32 +:32];
    assign m01_axi_awlen   = iw_awlen   [1*8  +:8];   assign m01_axi_arlen   = iw_arlen   [1*8  +:8];
    assign m01_axi_awsize  = iw_awsize  [1*3  +:3];   assign m01_axi_arsize  = iw_arsize  [1*3  +:3];
    assign m01_axi_awburst = iw_awburst [1*2  +:2];   assign m01_axi_arburst = iw_arburst [1*2  +:2];
    assign m01_axi_awlock  = iw_awlock  [1];          assign m01_axi_arlock  = iw_arlock  [1];
    assign m01_axi_awcache = iw_awcache [1*4  +:4];   assign m01_axi_arcache = iw_arcache [1*4  +:4];
    assign m01_axi_awprot  = iw_awprot  [1*3  +:3];   assign m01_axi_arprot  = iw_arprot  [1*3  +:3];
    assign m01_axi_awqos   = iw_awqos   [1*4  +:4];   assign m01_axi_arqos   = iw_arqos   [1*4  +:4];
    assign m01_axi_awvalid = iw_awvalid [1];          assign m01_axi_arvalid = iw_arvalid [1];
    assign iw_awready[1]   = m01_axi_awready;         assign iw_arready[1]   = m01_axi_arready;
    assign m01_axi_wdata   = iw_wdata   [1*128 +:128];assign m01_axi_wstrb   = iw_wstrb   [1*16 +:16];
    assign m01_axi_wlast   = iw_wlast   [1];          assign m01_axi_wvalid  = iw_wvalid  [1];
    assign iw_wready[1]    = m01_axi_wready;
    assign iw_bid[1*4 +:4] = m01_axi_bid[3:0];        assign iw_bresp[1*2 +:2] = m01_axi_bresp;
    assign iw_bvalid[1]    = m01_axi_bvalid;          assign m01_axi_bready  = iw_bready  [1];
    assign iw_rid[1*4 +:4] = m01_axi_rid[3:0];        assign iw_rdata[1*128 +:128] = m01_axi_rdata;
    assign iw_rresp[1*2 +:2] = m01_axi_rresp;         assign iw_rlast[1]     = m01_axi_rlast;
    assign iw_rvalid[1]    = m01_axi_rvalid;          assign m01_axi_rready  = iw_rready  [1];

    // ===== M02 → packed vector slot 2 (4→5 bit) =====
    assign m02_axi_awid    = {1'b0, iw_awid[2*4 +:4]}; assign m02_axi_arid    = {1'b0, iw_arid[2*4 +:4]};
    assign m02_axi_awaddr  = iw_awaddr  [2*32 +:32];  assign m02_axi_araddr  = iw_araddr  [2*32 +:32];
    assign m02_axi_awlen   = iw_awlen   [2*8  +:8];   assign m02_axi_arlen   = iw_arlen   [2*8  +:8];
    assign m02_axi_awsize  = iw_awsize  [2*3  +:3];   assign m02_axi_arsize  = iw_arsize  [2*3  +:3];
    assign m02_axi_awburst = iw_awburst [2*2  +:2];   assign m02_axi_arburst = iw_arburst [2*2  +:2];
    assign m02_axi_awlock  = iw_awlock  [2];          assign m02_axi_arlock  = iw_arlock  [2];
    assign m02_axi_awcache = iw_awcache [2*4  +:4];   assign m02_axi_arcache = iw_arcache [2*4  +:4];
    assign m02_axi_awprot  = iw_awprot  [2*3  +:3];   assign m02_axi_arprot  = iw_arprot  [2*3  +:3];
    assign m02_axi_awqos   = iw_awqos   [2*4  +:4];   assign m02_axi_arqos   = iw_arqos   [2*4  +:4];
    assign m02_axi_awvalid = iw_awvalid [2];          assign m02_axi_arvalid = iw_arvalid [2];
    assign iw_awready[2]   = m02_axi_awready;         assign iw_arready[2]   = m02_axi_arready;
    assign m02_axi_wdata   = iw_wdata   [2*128 +:128];assign m02_axi_wstrb   = iw_wstrb   [2*16 +:16];
    assign m02_axi_wlast   = iw_wlast   [2];          assign m02_axi_wvalid  = iw_wvalid  [2];
    assign iw_wready[2]    = m02_axi_wready;
    assign iw_bid[2*4 +:4] = m02_axi_bid[3:0];        assign iw_bresp[2*2 +:2] = m02_axi_bresp;
    assign iw_bvalid[2]    = m02_axi_bvalid;          assign m02_axi_bready  = iw_bready  [2];
    assign iw_rid[2*4 +:4] = m02_axi_rid[3:0];        assign iw_rdata[2*128 +:128] = m02_axi_rdata;
    assign iw_rresp[2*2 +:2] = m02_axi_rresp;         assign iw_rlast[2]     = m02_axi_rlast;
    assign iw_rvalid[2]    = m02_axi_rvalid;          assign m02_axi_rready  = iw_rready  [2];

    // ===== Debug expose: 把 iw_* (即 multicore_top_vd100 出口) 关键 m00_axi 信号外接 =====
    assign dbg_awvalid_0 = iw_awvalid[0];
    assign dbg_awready_0 = iw_awready[0];
    assign dbg_arvalid_0 = iw_arvalid[0];
    assign dbg_arready_0 = iw_arready[0];
    assign dbg_wvalid_0  = iw_wvalid[0];
    assign dbg_rvalid_0  = iw_rvalid[0];
    assign dbg_awaddr_0  = iw_awaddr[0*32 +: 32];
    assign dbg_araddr_0  = iw_araddr[0*32 +: 32];
    assign dbg_awid_0    = iw_awid[0*4 +: 4];
    assign dbg_arid_0    = iw_arid[0*4 +: 4];
    assign dbg_awlen_0   = iw_awlen[0*8 +: 8];
    assign dbg_arlen_0   = iw_arlen[0*8 +: 8];
    // 第二组: hier ref 拿 ConvCore[0] 内部信号 (v18 原配, 撤销 v19 sequencer 改动)
    assign dbg_dfe_state_0       = u_inner.gen_core[0].u_core.u_dfe.state;
    assign dbg_dfe_m_arvalid_0   = u_inner.gen_core[0].u_core.u_dfe.M_ARVALID;
    assign dbg_dfe_m_arready_0   = u_inner.gen_core[0].u_core.u_dfe.M_ARREADY;
    assign dbg_dfe_r_phase_0     = u_inner.gen_core[0].u_core.u_dfe.r_phase;
    assign dbg_dfe_r_done_0      = u_inner.gen_core[0].u_core.u_dfe.r_done;
    assign dbg_dfe_r_beats_rcvd_0= u_inner.gen_core[0].u_core.u_dfe.r_beats_rcvd;
    assign dbg_arb_rd_lock_0     = u_inner.gen_core[0].u_core.u_axi_mux.u_arbiter.rd_addr_channel_lock;
    assign dbg_arb_rd_sel_0      = u_inner.gen_core[0].u_core.u_axi_mux.u_arbiter.rd_addr_master_sel;
    assign dbg_arb_cu_rd_sel_0   = u_inner.gen_core[0].u_core.u_axi_mux.u_arbiter.cu_rd_addr_master_sel;
    assign dbg_master_arvalid_0  = u_inner.gen_core[0].u_core.m_arvalid;
    assign dbg_master_arready_0  = u_inner.gen_core[0].u_core.m_arready;
    assign dbg_master_rvalid_0   = u_inner.gen_core[0].u_core.m_rvalid;
    assign dbg_start_dfe_pulse_0 = u_inner.gen_core[0].u_core.cfg_start_dfe_pulse;
    assign dbg_dfe_busy_0        = u_inner.gen_core[0].u_core.dfe_busy;
    assign dbg_dfe_done_0        = u_inner.gen_core[0].u_core.dfe_done;
    assign dbg_idma_busy_0       = u_inner.gen_core[0].u_core.idma_busy;
    assign dbg_layer_busy_0      = u_inner.gen_core[0].u_core.layer_busy;
    assign dbg_csr_aw_fire_0     = csr_axil_awvalid && csr_axil_awready;

    // ===== 内部实例化 multicore_top_vd100 =====
    // NUM_CORES=3 (恢复, NUM_CORES=1 触发 csr_axil 副作用让 POKE timeout)
    multicore_top_vd100 #(
        .NUM_CORES(3),
        .MAX_CORES(3)
    ) u_inner (
        .clk(clk), .rst_n(rst_n),

        .csr_axil_awaddr (csr_axil_awaddr),  .csr_axil_awvalid(csr_axil_awvalid),
        .csr_axil_awready(csr_axil_awready), .csr_axil_wdata  (csr_axil_wdata),
        .csr_axil_wstrb  (csr_axil_wstrb),   .csr_axil_wvalid (csr_axil_wvalid),
        .csr_axil_wready (csr_axil_wready),  .csr_axil_bresp  (csr_axil_bresp),
        .csr_axil_bvalid (csr_axil_bvalid),  .csr_axil_bready (csr_axil_bready),
        .csr_axil_araddr (csr_axil_araddr),  .csr_axil_arvalid(csr_axil_arvalid),
        .csr_axil_arready(csr_axil_arready), .csr_axil_rdata  (csr_axil_rdata),
        .csr_axil_rresp  (csr_axil_rresp),   .csr_axil_rvalid (csr_axil_rvalid),
        .csr_axil_rready (csr_axil_rready),

        .m_axi_awid   (iw_awid),    .m_axi_awaddr (iw_awaddr),
        .m_axi_awlen  (iw_awlen),   .m_axi_awsize (iw_awsize),
        .m_axi_awburst(iw_awburst), .m_axi_awlock (iw_awlock),
        .m_axi_awcache(iw_awcache), .m_axi_awprot (iw_awprot),
        .m_axi_awqos  (iw_awqos),
        .m_axi_awvalid(iw_awvalid), .m_axi_awready(iw_awready),
        .m_axi_wdata  (iw_wdata),   .m_axi_wstrb  (iw_wstrb),
        .m_axi_wlast  (iw_wlast),   .m_axi_wvalid (iw_wvalid),
        .m_axi_wready (iw_wready),
        .m_axi_bid    (iw_bid),     .m_axi_bresp  (iw_bresp),
        .m_axi_bvalid (iw_bvalid),  .m_axi_bready (iw_bready),
        .m_axi_arid   (iw_arid),    .m_axi_araddr (iw_araddr),
        .m_axi_arlen  (iw_arlen),   .m_axi_arsize (iw_arsize),
        .m_axi_arburst(iw_arburst), .m_axi_arlock (iw_arlock),
        .m_axi_arcache(iw_arcache), .m_axi_arprot (iw_arprot),
        .m_axi_arqos  (iw_arqos),
        .m_axi_arvalid(iw_arvalid), .m_axi_arready(iw_arready),
        .m_axi_rid    (iw_rid),     .m_axi_rdata  (iw_rdata),
        .m_axi_rresp  (iw_rresp),
        .m_axi_rlast  (iw_rlast),   .m_axi_rvalid (iw_rvalid),
        .m_axi_rready (iw_rready),

        .irq_done(irq_done)
    );

endmodule
