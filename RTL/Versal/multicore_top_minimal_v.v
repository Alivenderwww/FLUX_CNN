`timescale 1ns/1ps

// =============================================================================
// multicore_top_minimal_v.v  --  Verilog wrapper for SV multicore_top_minimal
//
// Vivado BD `create_bd_cell -type module -reference` 不接受 SystemVerilog 顶层
// 文件 (filemgmt 56-195 error). 用 .v wrapper 暴露端口给 BD, 内部实例化 SV 模块.
//
// 端口签名 1:1 对应 multicore_top_minimal.sv. 加 X_INTERFACE_INFO/PARAMETER
// 让 BD 识别成 AXI4 interface (slave csr_axil + master m_axi).
// =============================================================================

(* CORE_GENERATION_INFO = "multicore_top_minimal_v,multicore_top_minimal,{}" *)
module multicore_top_minimal_v (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF csr_axil:m_axi, ASSOCIATED_RESET rst_n" *)
    input                clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 rst_n RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input                rst_n,

    // ---- AXI-Lite Slave (host CSR) ----
    (* X_INTERFACE_PARAMETER = "MODE Slave, ADDR_WIDTH 12, DATA_WIDTH 32, PROTOCOL AXI4LITE, FREQ_HZ 100000000, ID_WIDTH 0, HAS_BURST 0, HAS_LOCK 0, HAS_PROT 0, HAS_CACHE 0, HAS_QOS 0, HAS_REGION 0, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 1" *)
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil AWADDR" *)  input  [11:0] csr_axil_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil AWVALID" *) input         csr_axil_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil AWREADY" *) output        csr_axil_awready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil WDATA" *)   input  [31:0] csr_axil_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil WSTRB" *)   input  [3:0]  csr_axil_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil WVALID" *)  input         csr_axil_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil WREADY" *)  output        csr_axil_wready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil BRESP" *)   output [1:0]  csr_axil_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil BVALID" *)  output        csr_axil_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil BREADY" *)  input         csr_axil_bready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil ARADDR" *)  input  [11:0] csr_axil_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil ARVALID" *) input         csr_axil_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil ARREADY" *) output        csr_axil_arready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil RDATA" *)   output [31:0] csr_axil_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil RRESP" *)   output [1:0]  csr_axil_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil RVALID" *)  output        csr_axil_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil RREADY" *)  input         csr_axil_rready,

    // ---- AXI4 Master (ConvCore 出, 接 smartconnect → axi_bram_ctrl) ----
    (* X_INTERFACE_PARAMETER = "MODE Master, PROTOCOL AXI4, ADDR_WIDTH 32, DATA_WIDTH 128, ID_WIDTH 4, FREQ_HZ 100000000, NUM_WRITE_OUTSTANDING 1, NUM_WRITE_THREADS 1, NUM_READ_OUTSTANDING 1, NUM_READ_THREADS 1, READ_WRITE_MODE READ_WRITE, HAS_BURST 1, HAS_LOCK 1, HAS_PROT 1, HAS_CACHE 1, HAS_QOS 1, HAS_REGION 0, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 1, SUPPORTS_NARROW_BURST 1, MAX_BURST_LENGTH 256, AWUSER_WIDTH 0, ARUSER_WIDTH 0, WUSER_WIDTH 0, RUSER_WIDTH 0, BUSER_WIDTH 0" *)
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWID" *)    output [3:0]   m_axi_awid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWADDR" *)  output [31:0]  m_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWLEN" *)   output [7:0]   m_axi_awlen,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWSIZE" *)  output [2:0]   m_axi_awsize,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWBURST" *) output [1:0]   m_axi_awburst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWLOCK" *)  output         m_axi_awlock,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWCACHE" *) output [3:0]   m_axi_awcache,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWPROT" *)  output [2:0]   m_axi_awprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWQOS" *)   output [3:0]   m_axi_awqos,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWVALID" *) output         m_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWREADY" *) input          m_axi_awready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WDATA" *)   output [127:0] m_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WSTRB" *)   output [15:0]  m_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WLAST" *)   output         m_axi_wlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WVALID" *)  output         m_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WREADY" *)  input          m_axi_wready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi BID" *)     input  [3:0]   m_axi_bid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi BRESP" *)   input  [1:0]   m_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi BVALID" *)  input          m_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi BREADY" *)  output         m_axi_bready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARID" *)    output [3:0]   m_axi_arid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARADDR" *)  output [31:0]  m_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARLEN" *)   output [7:0]   m_axi_arlen,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARSIZE" *)  output [2:0]   m_axi_arsize,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARBURST" *) output [1:0]   m_axi_arburst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARLOCK" *)  output         m_axi_arlock,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARCACHE" *) output [3:0]   m_axi_arcache,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARPROT" *)  output [2:0]   m_axi_arprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARQOS" *)   output [3:0]   m_axi_arqos,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARVALID" *) output         m_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARREADY" *) input          m_axi_arready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RID" *)     input  [3:0]   m_axi_rid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RDATA" *)   input  [127:0] m_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RRESP" *)   input  [1:0]   m_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RLAST" *)   input          m_axi_rlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RVALID" *)  input          m_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RREADY" *)  output         m_axi_rready,

    // ---- IRQ ----
    output                              irq_done
);

    // 实例化 SV 模块 (端口直通)
    multicore_top_minimal u_mc_minimal_sv (
        .clk(clk),
        .rst_n(rst_n),
        .csr_axil_awaddr(csr_axil_awaddr),
        .csr_axil_awvalid(csr_axil_awvalid),
        .csr_axil_awready(csr_axil_awready),
        .csr_axil_wdata(csr_axil_wdata),
        .csr_axil_wstrb(csr_axil_wstrb),
        .csr_axil_wvalid(csr_axil_wvalid),
        .csr_axil_wready(csr_axil_wready),
        .csr_axil_bresp(csr_axil_bresp),
        .csr_axil_bvalid(csr_axil_bvalid),
        .csr_axil_bready(csr_axil_bready),
        .csr_axil_araddr(csr_axil_araddr),
        .csr_axil_arvalid(csr_axil_arvalid),
        .csr_axil_arready(csr_axil_arready),
        .csr_axil_rdata(csr_axil_rdata),
        .csr_axil_rresp(csr_axil_rresp),
        .csr_axil_rvalid(csr_axil_rvalid),
        .csr_axil_rready(csr_axil_rready),
        .m_axi_awid(m_axi_awid),
        .m_axi_awaddr(m_axi_awaddr),
        .m_axi_awlen(m_axi_awlen),
        .m_axi_awsize(m_axi_awsize),
        .m_axi_awburst(m_axi_awburst),
        .m_axi_awlock(m_axi_awlock),
        .m_axi_awcache(m_axi_awcache),
        .m_axi_awprot(m_axi_awprot),
        .m_axi_awqos(m_axi_awqos),
        .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awready(m_axi_awready),
        .m_axi_wdata(m_axi_wdata),
        .m_axi_wstrb(m_axi_wstrb),
        .m_axi_wlast(m_axi_wlast),
        .m_axi_wvalid(m_axi_wvalid),
        .m_axi_wready(m_axi_wready),
        .m_axi_bid(m_axi_bid),
        .m_axi_bresp(m_axi_bresp),
        .m_axi_bvalid(m_axi_bvalid),
        .m_axi_bready(m_axi_bready),
        .m_axi_arid(m_axi_arid),
        .m_axi_araddr(m_axi_araddr),
        .m_axi_arlen(m_axi_arlen),
        .m_axi_arsize(m_axi_arsize),
        .m_axi_arburst(m_axi_arburst),
        .m_axi_arlock(m_axi_arlock),
        .m_axi_arcache(m_axi_arcache),
        .m_axi_arprot(m_axi_arprot),
        .m_axi_arqos(m_axi_arqos),
        .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(m_axi_arready),
        .m_axi_rid(m_axi_rid),
        .m_axi_rdata(m_axi_rdata),
        .m_axi_rresp(m_axi_rresp),
        .m_axi_rlast(m_axi_rlast),
        .m_axi_rvalid(m_axi_rvalid),
        .m_axi_rready(m_axi_rready),
        .irq_done(irq_done)
    );

endmodule
