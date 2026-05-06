`timescale 1ns/1ps

// =============================================================================
// multicore_top_mesh.sv  --  4-core CNN accelerator with mesh AXIS NoC (Phase 6)
//
// 跟 multicore_top.sv 的差别:
//   - axi_4to5 crossbar (write 路径) 替换成 mesh_4x2 + mesh_core_wrapper
//   - IFB / OFM 走 AXIS packet 在 mesh 上路由 (跟 4-DDR PoC 等价 BW 模型, 但解耦)
//   - WB / RDMA / desc fetch 仍走 axi master read, 接 sim-internal axi_slave_mem
//   - host CSR 仍 axi_lite_1to4 fanout (跟 multicore_top 一致)
//
// 拓扑 (4x2 mesh):
//   y=1   Conv0(0,1) ─ Conv1(1,1) ─ Conv2(2,1) ─ Conv3(3,1)    LOCAL[4..7]
//           │            │            │            │
//   y=0   Mem0(0,0) ─ Mem1(1,0) ─ Mem2(2,0) ─ Mem3(3,0)        LOCAL[0..3]
//
// Conv[i] 的 OFM 走 mesh 1 hop south 到 Mem[i] (1-1 对应, 路由 0 hop 拥堵).
//
// 端口 (sim-only):
//   - clk / rst_n / csr_* (同 multicore_top, host AXI-Lite)
//   - done_per_core (4 bit)
//   - bus_* (AXI master read aggregate, 4 SI ConvCore.bus_ar/r → 1 MI DDR-mock)
//   - bus_aw/w/b 这边 tie 0 (mesh 模式下 ODMA 走 mesh 不走 DDR)
//
// TB 通过 hier ref 操作 4 mem_core.ddr_mem (preload IFB + 检查 OFM)
// TB 通过 hier ref 驱动 4 mem_core.cmd_* 推 packet (Step B 之前 stub).
// =============================================================================
`include "flux_cnn_params.svh"

module multicore_top_mesh #(
    parameter int NUM_CORES   = 4,                          // 固定 4
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
    parameter int DEST_W      = 8,                          // mesh tdest 宽 (4y + 4x)
    parameter int NUMA_MODE   = 0,                          // 0 = Phase 6 push, 1 = NUMA pull

    localparam int CORE_ID_W   = $clog2(NUM_CORES),
    localparam int HOST_CSR_AW = 12 + CORE_ID_W,
    localparam int CORE_BUS_ID = AXI_M_ID + AXI_M_WIDTH,
    localparam int RMT_ID_W    = AXI_M_ID + AXI_M_WIDTH + 1
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

    // ---- AXI-Lite Slave: Mem Core CSR (host 配置 mem cmd 寄存器) ----
    //   addr [13:12]=core_id, [11:0]=mem 内部 CSR offset
    input  logic [HOST_CSR_AW-1:0]               mem_csr_awaddr,
    input  logic                                 mem_csr_awvalid,
    output logic                                 mem_csr_awready,
    input  logic [CSR_DATA_W-1:0]                mem_csr_wdata,
    input  logic [CSR_DATA_W/8-1:0]              mem_csr_wstrb,
    input  logic                                 mem_csr_wvalid,
    output logic                                 mem_csr_wready,
    output logic [1:0]                           mem_csr_bresp,
    output logic                                 mem_csr_bvalid,
    input  logic                                 mem_csr_bready,
    input  logic [HOST_CSR_AW-1:0]               mem_csr_araddr,
    input  logic                                 mem_csr_arvalid,
    output logic                                 mem_csr_arready,
    output logic [CSR_DATA_W-1:0]                mem_csr_rdata,
    output logic [1:0]                           mem_csr_rresp,
    output logic                                 mem_csr_rvalid,
    input  logic                                 mem_csr_rready,

    // ---- 外部 AXI4 Master (read aggregate 给 DDR; write 端 tie 0) ----
    // 4 ConvCore.bus_ar/r 走 axi_4to5 (复用 IP) → 出口接 DDR
    output logic [CORE_BUS_ID+CORE_ID_W-1:0]     bus_arid,
    output logic [BUS_ADDR_W-1:0]                bus_araddr,
    output logic [7:0]                           bus_arlen,
    output logic [2:0]                           bus_arsize,
    output logic [1:0]                           bus_arburst,
    output logic                                 bus_arlock,
    output logic [3:0]                           bus_arcache,
    output logic [2:0]                           bus_arprot,
    output logic [3:0]                           bus_arqos,
    output logic                                 bus_arvalid,
    input  logic                                 bus_arready,
    input  logic [CORE_BUS_ID+CORE_ID_W-1:0]     bus_rid,
    input  logic [BUS_DATA_W-1:0]                bus_rdata,
    input  logic [1:0]                           bus_rresp,
    input  logic                                 bus_rlast,
    input  logic                                 bus_rvalid,
    output logic                                 bus_rready,

    // ---- per-core done sticky ----
    output logic [NUM_CORES-1:0]                 done_per_core
);

    localparam int EXT_BUS_ID = CORE_BUS_ID + CORE_ID_W;
    logic aresetn;
    assign aresetn = rst_n;

    // =========================================================================
    // 1. AXI-Lite host CSR fanout (axi_lite_1to4 IP, 复用 multicore_top.sv 的)
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

    // =========================================================================
    // 1b. Mem core CSR fanout (axi_lite_1to4 第二份, 给 4 个 mem stub)
    // =========================================================================
    logic [NUM_CORES*HOST_CSR_AW-1:0]   mem_m_awaddr;
    logic [NUM_CORES*3-1:0]             mem_m_awprot;
    logic [NUM_CORES-1:0]               mem_m_awvalid;
    logic [NUM_CORES-1:0]               mem_m_awready;
    logic [NUM_CORES*CSR_DATA_W-1:0]    mem_m_wdata;
    logic [NUM_CORES*(CSR_DATA_W/8)-1:0]mem_m_wstrb;
    logic [NUM_CORES-1:0]               mem_m_wvalid;
    logic [NUM_CORES-1:0]               mem_m_wready;
    logic [NUM_CORES*2-1:0]             mem_m_bresp;
    logic [NUM_CORES-1:0]               mem_m_bvalid;
    logic [NUM_CORES-1:0]               mem_m_bready;
    logic [NUM_CORES*HOST_CSR_AW-1:0]   mem_m_araddr;
    logic [NUM_CORES*3-1:0]             mem_m_arprot;
    logic [NUM_CORES-1:0]               mem_m_arvalid;
    logic [NUM_CORES-1:0]               mem_m_arready;
    logic [NUM_CORES*CSR_DATA_W-1:0]    mem_m_rdata;
    logic [NUM_CORES*2-1:0]             mem_m_rresp;
    logic [NUM_CORES-1:0]               mem_m_rvalid;
    logic [NUM_CORES-1:0]               mem_m_rready;

    axi_lite_1to4 u_mem_csr_xbar (
        .aclk          (clk),
        .aresetn       (aresetn),
        .s_axi_awaddr  (mem_csr_awaddr),  .s_axi_awprot  (3'b000),
        .s_axi_awvalid (mem_csr_awvalid), .s_axi_awready (mem_csr_awready),
        .s_axi_wdata   (mem_csr_wdata),   .s_axi_wstrb   (mem_csr_wstrb),
        .s_axi_wvalid  (mem_csr_wvalid),  .s_axi_wready  (mem_csr_wready),
        .s_axi_bresp   (mem_csr_bresp),
        .s_axi_bvalid  (mem_csr_bvalid),  .s_axi_bready  (mem_csr_bready),
        .s_axi_araddr  (mem_csr_araddr),  .s_axi_arprot  (3'b000),
        .s_axi_arvalid (mem_csr_arvalid), .s_axi_arready (mem_csr_arready),
        .s_axi_rdata   (mem_csr_rdata),   .s_axi_rresp   (mem_csr_rresp),
        .s_axi_rvalid  (mem_csr_rvalid),  .s_axi_rready  (mem_csr_rready),
        .m_axi_awaddr  (mem_m_awaddr),    .m_axi_awprot  (mem_m_awprot),
        .m_axi_awvalid (mem_m_awvalid),   .m_axi_awready (mem_m_awready),
        .m_axi_wdata   (mem_m_wdata),     .m_axi_wstrb   (mem_m_wstrb),
        .m_axi_wvalid  (mem_m_wvalid),    .m_axi_wready  (mem_m_wready),
        .m_axi_bresp   (mem_m_bresp),
        .m_axi_bvalid  (mem_m_bvalid),    .m_axi_bready  (mem_m_bready),
        .m_axi_araddr  (mem_m_araddr),    .m_axi_arprot  (mem_m_arprot),
        .m_axi_arvalid (mem_m_arvalid),   .m_axi_arready (mem_m_arready),
        .m_axi_rdata   (mem_m_rdata),     .m_axi_rresp   (mem_m_rresp),
        .m_axi_rvalid  (mem_m_rvalid),    .m_axi_rready  (mem_m_rready)
    );

    // =========================================================================
    // 2. 4 个 ConvCore.bus_ar/r aggregate → 单一 DDR slave
    //    用 axi_4to5 IP 复用 (write 通道 tie 0, read 通道仲裁 4 SI → 1 MI DDR)
    //    简化: 直接用 axi_arbiter (4 SI → 1 MI), 项目现有 RTL
    // =========================================================================
    // 4 ConvCore.bus_ar/r 信号 (per-core)
    logic [CORE_BUS_ID-1:0]   c_bus_arid    [NUM_CORES];
    logic [BUS_ADDR_W-1:0]    c_bus_araddr  [NUM_CORES];
    logic [7:0]               c_bus_arlen   [NUM_CORES];
    logic [2:0]               c_bus_arsize  [NUM_CORES];
    logic [1:0]               c_bus_arburst [NUM_CORES];
    logic                     c_bus_arlock  [NUM_CORES];
    logic [3:0]               c_bus_arcache [NUM_CORES];
    logic [2:0]               c_bus_arprot  [NUM_CORES];
    logic [3:0]               c_bus_arqos   [NUM_CORES];
    logic [NUM_CORES-1:0]     c_bus_arvalid;
    logic [NUM_CORES-1:0]     c_bus_arready;
    logic [CORE_BUS_ID-1:0]   c_bus_rid     [NUM_CORES];
    logic [BUS_DATA_W-1:0]    c_bus_rdata   [NUM_CORES];
    logic [1:0]               c_bus_rresp   [NUM_CORES];
    logic [NUM_CORES-1:0]     c_bus_rlast;
    logic [NUM_CORES-1:0]     c_bus_rvalid;
    logic [NUM_CORES-1:0]     c_bus_rready;

    // 简化仲裁: round-robin 1-deep (sim only). 实际 deployment 用 axi_4to1 IP.
    // 先给 4 SI 串行选择 1 个 active master, 其他等.
    // 用一个简单的 priority arbiter (固定优先级 0>1>2>3, sim PoC 够用).
    logic [1:0] r_active;
    logic       r_busy;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            r_active <= 2'd0;
            r_busy   <= 1'b0;
        end else begin
            if (!r_busy) begin
                // 寻找下一个 valid SI
                if      (c_bus_arvalid[0]) begin r_active <= 2'd0; r_busy <= 1'b1; end
                else if (c_bus_arvalid[1]) begin r_active <= 2'd1; r_busy <= 1'b1; end
                else if (c_bus_arvalid[2]) begin r_active <= 2'd2; r_busy <= 1'b1; end
                else if (c_bus_arvalid[3]) begin r_active <= 2'd3; r_busy <= 1'b1; end
            end else begin
                // 等当前 burst 完成 (rlast fire)
                if (bus_rvalid && bus_rready && bus_rlast) r_busy <= 1'b0;
            end
        end
    end

    // 输出聚合 (跟 r_active 选)
    assign bus_arid     = {2'(r_active), c_bus_arid[r_active]};   // ID 加 core_id 标记
    assign bus_araddr   = c_bus_araddr[r_active];
    assign bus_arlen    = c_bus_arlen[r_active];
    assign bus_arsize   = c_bus_arsize[r_active];
    assign bus_arburst  = c_bus_arburst[r_active];
    assign bus_arlock   = c_bus_arlock[r_active];
    assign bus_arcache  = c_bus_arcache[r_active];
    assign bus_arprot   = c_bus_arprot[r_active];
    assign bus_arqos    = c_bus_arqos[r_active];
    assign bus_arvalid  = r_busy ? c_bus_arvalid[r_active] : 1'b0;
    assign bus_rready   = r_busy ? c_bus_rready[r_active]  : 1'b0;

    // 反向: rdata/rid/rresp/rlast 只送给 active core
    genvar gi;
    generate
        for (gi = 0; gi < NUM_CORES; gi++) begin : g_rdy
            assign c_bus_arready[gi] = r_busy && (r_active == gi[1:0]) && bus_arready;
            assign c_bus_rid[gi]     = bus_rid[CORE_BUS_ID-1:0];
            assign c_bus_rdata[gi]   = bus_rdata;
            assign c_bus_rresp[gi]   = bus_rresp;
            assign c_bus_rlast[gi]   = bus_rlast;
            assign c_bus_rvalid[gi]  = r_busy && (r_active == gi[1:0]) && bus_rvalid;
        end
    endgenerate

    // =========================================================================
    // 3. 4x2 mesh + 4 ConvCore + 4 Mem Core
    // =========================================================================
    // mesh 8 LOCAL ports (idx 0..3 = Mem, 4..7 = Conv)
    logic [7:0]            mesh_in_tvalid;
    logic [7:0]            mesh_in_tready;
    logic [7:0]            mesh_in_tlast;
    logic [BUS_DATA_W-1:0] mesh_in_tdata  [8];
    logic [DEST_W-1:0]     mesh_in_tdest  [8];
    logic [7:0]            mesh_out_tvalid;
    logic [7:0]            mesh_out_tready;
    logic [7:0]            mesh_out_tlast;
    logic [BUS_DATA_W-1:0] mesh_out_tdata  [8];
    logic [DEST_W-1:0]     mesh_out_tdest  [8];

    mesh_4x2 #(.DATA_W(BUS_DATA_W), .DEST_W(DEST_W)) u_mesh (
        .clk(clk), .rst_n(rst_n),
        .s_axis_local_tvalid(mesh_in_tvalid),
        .s_axis_local_tready(mesh_in_tready),
        .s_axis_local_tlast (mesh_in_tlast),
        .s_axis_local_tdata (mesh_in_tdata),
        .s_axis_local_tdest (mesh_in_tdest),
        .m_axis_local_tvalid(mesh_out_tvalid),
        .m_axis_local_tready(mesh_out_tready),
        .m_axis_local_tlast (mesh_out_tlast),
        .m_axis_local_tdata (mesh_out_tdata),
        .m_axis_local_tdest (mesh_out_tdest)
    );

    // ----- 4 Mem Core stub @ idx 0..3 (mesh y=0) -----
    generate
        for (gi = 0; gi < NUM_CORES; gi++) begin : gen_mem
            mem_core_stub #(
                .DATA_W(BUS_DATA_W), .DEST_W(DEST_W),
                .ADDR_W(BUS_ADDR_W), .DDR_DEPTH(4*1048576), // 64 MB / mem (装 OFM_LAYER 16-48MB 区)
                .CSR_ADDR_W(12), .CSR_DATA_W(CSR_DATA_W)
            ) u_mem (
                .clk(clk), .rst_n(rst_n),
                .s_axis_tvalid(mesh_out_tvalid[gi]),
                .s_axis_tready(mesh_out_tready[gi]),
                .s_axis_tdata (mesh_out_tdata [gi]),
                .s_axis_tlast (mesh_out_tlast [gi]),
                .s_axis_tdest (mesh_out_tdest [gi]),
                .m_axis_tvalid(mesh_in_tvalid[gi]),
                .m_axis_tready(mesh_in_tready[gi]),
                .m_axis_tdata (mesh_in_tdata [gi]),
                .m_axis_tlast (mesh_in_tlast [gi]),
                .m_axis_tdest (mesh_in_tdest [gi]),
                // mem CSR (来自 mem axi_lite_1to4 MI[gi]; addr 取低 12 位)
                .csr_awaddr (mem_m_awaddr [gi*HOST_CSR_AW +: 12]),
                .csr_awvalid(mem_m_awvalid[gi]),
                .csr_awready(mem_m_awready[gi]),
                .csr_wdata  (mem_m_wdata  [gi*CSR_DATA_W +: CSR_DATA_W]),
                .csr_wstrb  (mem_m_wstrb  [gi*(CSR_DATA_W/8) +: (CSR_DATA_W/8)]),
                .csr_wvalid (mem_m_wvalid [gi]),
                .csr_wready (mem_m_wready [gi]),
                .csr_bresp  (mem_m_bresp  [gi*2 +: 2]),
                .csr_bvalid (mem_m_bvalid [gi]),
                .csr_bready (mem_m_bready [gi]),
                .csr_araddr (mem_m_araddr [gi*HOST_CSR_AW +: 12]),
                .csr_arvalid(mem_m_arvalid[gi]),
                .csr_arready(mem_m_arready[gi]),
                .csr_rdata  (mem_m_rdata  [gi*CSR_DATA_W +: CSR_DATA_W]),
                .csr_rresp  (mem_m_rresp  [gi*2 +: 2]),
                .csr_rvalid (mem_m_rvalid [gi]),
                .csr_rready (mem_m_rready [gi]),
                .last_rx_opcode(),
                .last_rx_burst_len(),
                .last_rx_addr(),
                .rx_pkt_done()
            );
        end
    endgenerate

    // ----- 4 ConvCore (mesh_core_wrapper) @ idx 4..7 (mesh y=1) -----
    //   OFM tdest / opcode 现在由 ConvCore 内部 cfg_regs 驱动 (host 经 AXI-Lite 写
    //   ADDR_OFM_TDEST / ADDR_OFM_OPCODE), 顶层不再硬编码.
    generate
        for (gi = 0; gi < NUM_CORES; gi++) begin : gen_core
            mesh_core_wrapper #(
                .NUM_COL(NUM_COL), .NUM_PE(NUM_PE),
                .DATA_WIDTH(DATA_WIDTH), .PSUM_WIDTH(PSUM_WIDTH),
                .SRAM_DEPTH(SRAM_DEPTH),
                .CSR_ADDR_W(12), .CSR_DATA_W(CSR_DATA_W),
                .BUS_ADDR_W(BUS_ADDR_W), .BUS_DATA_W(BUS_DATA_W),
                .AXI_M_ID(AXI_M_ID), .AXI_M_WIDTH(AXI_M_WIDTH),
                .DEST_W(DEST_W),
                .RMT_ID_W(EXT_BUS_ID),
                .NUMA_MODE(NUMA_MODE),
                .CORE_LOCAL_ID(8'h10 | gi)
            ) u_conv (
                .clk(clk), .rst_n(rst_n),

                // host CSR (来自 axi_lite_1to4 MI[gi])
                .csr_awaddr (csr_m_awaddr [gi*HOST_CSR_AW +: 12]),
                .csr_awvalid(csr_m_awvalid[gi]),
                .csr_awready(csr_m_awready[gi]),
                .csr_wdata  (csr_m_wdata  [gi*CSR_DATA_W +: CSR_DATA_W]),
                .csr_wstrb  (csr_m_wstrb  [gi*(CSR_DATA_W/8) +: (CSR_DATA_W/8)]),
                .csr_wvalid (csr_m_wvalid [gi]),
                .csr_wready (csr_m_wready [gi]),
                .csr_bresp  (csr_m_bresp  [gi*2 +: 2]),
                .csr_bvalid (csr_m_bvalid [gi]),
                .csr_bready (csr_m_bready [gi]),
                .csr_araddr (csr_m_araddr [gi*HOST_CSR_AW +: 12]),
                .csr_arvalid(csr_m_arvalid[gi]),
                .csr_arready(csr_m_arready[gi]),
                .csr_rdata  (csr_m_rdata  [gi*CSR_DATA_W +: CSR_DATA_W]),
                .csr_rresp  (csr_m_rresp  [gi*2 +: 2]),
                .csr_rvalid (csr_m_rvalid [gi]),
                .csr_rready (csr_m_rready [gi]),

                // mesh AXIS 接 mesh_4x2 LOCAL[gi+4]
                .s_axis_tvalid(mesh_out_tvalid[gi + 4]),
                .s_axis_tready(mesh_out_tready[gi + 4]),
                .s_axis_tdata (mesh_out_tdata [gi + 4]),
                .s_axis_tlast (mesh_out_tlast [gi + 4]),
                .s_axis_tdest (mesh_out_tdest [gi + 4]),
                .m_axis_tvalid(mesh_in_tvalid[gi + 4]),
                .m_axis_tready(mesh_in_tready[gi + 4]),
                .m_axis_tdata (mesh_in_tdata [gi + 4]),
                .m_axis_tlast (mesh_in_tlast [gi + 4]),
                .m_axis_tdest (mesh_in_tdest [gi + 4]),

                // AXI master read (WB / RDMA / desc fetch) → 仲裁后接 DDR
                .bus_arid   (c_bus_arid   [gi]),
                .bus_araddr (c_bus_araddr [gi]),
                .bus_arlen  (c_bus_arlen  [gi]),
                .bus_arsize (c_bus_arsize [gi]),
                .bus_arburst(c_bus_arburst[gi]),
                .bus_arlock (c_bus_arlock [gi]),
                .bus_arcache(c_bus_arcache[gi]),
                .bus_arprot (c_bus_arprot [gi]),
                .bus_arqos  (c_bus_arqos  [gi]),
                .bus_arvalid(c_bus_arvalid[gi]),
                .bus_arready(c_bus_arready[gi]),
                .bus_rid    (c_bus_rid    [gi]),
                .bus_rdata  (c_bus_rdata  [gi]),
                .bus_rresp  (c_bus_rresp  [gi]),
                .bus_rlast  (c_bus_rlast  [gi]),
                .bus_rvalid (c_bus_rvalid [gi]),
                .bus_rready (c_bus_rready [gi]),

                .done(done_per_core[gi])
            );
        end
    endgenerate

endmodule
