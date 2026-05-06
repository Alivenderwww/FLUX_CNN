`timescale 1ns/1ps

// =============================================================================
// mesh_core_wrapper.sv  --  ConvCore + mesh AXIS 桥 wrap (Phase 6 hybrid)
//
// 把 core_top 嵌进来 + 加 mesh 双桥, 让 ConvCore 通过 AXIS 跟 mesh 通信:
//
//   mesh AXIS slave  → axis_to_axi_writer → core_top.rmt_ifb_*  (IFB push 路径)
//   core_top.bus_aw/w/b → axi_writer_to_axis → mesh AXIS master (ODMA 出 mesh)
//   core_top.bus_ar/r   → 外部 (TB / Mem Core, DDR-mock 还是用)
//
// host CSR 接口 + mesh 跟 ar/r 通道暴露给上层. ConvCore 内部 idma/wdma/odma_ctrl
// 不动, 但 cfg_skip_idma=1 会让 idma 跳过 (跟 M2 一致).
//
// 注意:
//   - ar/r (read) 通道仍需要外部 DDR-mock 服务 (WB / RDMA / desc 拉取)
//   - 实际 deployment 时 Mem Core 可以兼做 axi-mm slave 服务 read, 或者另加 axis-to-axi-reader 桥
//   - 此 wrapper 只覆盖 write 路径 (IFB 进 + OFM 出), read 路径暂时直接 expose
// =============================================================================

module mesh_core_wrapper #(
    parameter int NUM_COL    = `FLUX_NUM_COL,
    parameter int NUM_PE     = `FLUX_NUM_PE,
    parameter int DATA_WIDTH = `FLUX_DATA_WIDTH,
    parameter int PSUM_WIDTH = `FLUX_PSUM_WIDTH,
    parameter int SRAM_DEPTH = `FLUX_IFB_DEPTH,
    parameter int CSR_ADDR_W = 12,
    parameter int CSR_DATA_W = `FLUX_CSR_DATA_W,
    parameter int BUS_ADDR_W = `FLUX_BUS_ADDR_W,
    parameter int BUS_DATA_W = `FLUX_BUS_DATA_W,
    parameter int AXI_M_ID   = `FLUX_AXI_M_ID,
    parameter int AXI_M_WIDTH = `FLUX_AXI_M_WIDTH,
    parameter int DEST_W     = 8,
    parameter int RMT_ID_W   = AXI_M_ID + AXI_M_WIDTH + 1,
    // NUMA_MODE=0: Phase 6 push 模式 (mem 推 IFB packet → ConvCore.rmt_ifb;
    //              ConvCore.bus_ar/r 直 expose 给上层 DDR)
    // NUMA_MODE=1: NUMA pull 模式 (ConvCore IDMA 用全局地址主动拉, bus_ar/r 走 mesh
    //              READ_REQ/READ_RESP packet; rmt_ifb 不再用)
    parameter int NUMA_MODE  = 0,
    // CORE_LOCAL_ID: 本核在 mesh 中的 LOCAL idx (e.g. ConvCore[c] 接 idx 4+c, 即 0x10|c)
    //   axi_reader_to_axis 用作 READ_REQ packet 的 return_id (mem 收到 RESP 后回这里)
    parameter int CORE_LOCAL_ID = 8'h10
)(
    input  logic                                 clk,
    input  logic                                 rst_n,

    // ---- Host CSR (AXI-Lite) — 直接 forward 给 core_top ----
    input  logic [CSR_ADDR_W-1:0]                csr_awaddr,
    input  logic                                 csr_awvalid,
    output logic                                 csr_awready,
    input  logic [CSR_DATA_W-1:0]                csr_wdata,
    input  logic [CSR_DATA_W/8-1:0]              csr_wstrb,
    input  logic                                 csr_wvalid,
    output logic                                 csr_wready,
    output logic [1:0]                           csr_bresp,
    output logic                                 csr_bvalid,
    input  logic                                 csr_bready,
    input  logic [CSR_ADDR_W-1:0]                csr_araddr,
    input  logic                                 csr_arvalid,
    output logic                                 csr_arready,
    output logic [CSR_DATA_W-1:0]                csr_rdata,
    output logic [1:0]                           csr_rresp,
    output logic                                 csr_rvalid,
    input  logic                                 csr_rready,

    // ---- Mesh AXIS slave (mesh in → IFB push) ----
    input  logic                                 s_axis_tvalid,
    output logic                                 s_axis_tready,
    input  logic [BUS_DATA_W-1:0]                s_axis_tdata,
    input  logic                                 s_axis_tlast,
    input  logic [DEST_W-1:0]                    s_axis_tdest,

    // ---- Mesh AXIS master (ODMA out → mesh) ----
    output logic                                 m_axis_tvalid,
    input  logic                                 m_axis_tready,
    output logic [BUS_DATA_W-1:0]                m_axis_tdata,
    output logic                                 m_axis_tlast,
    output logic [DEST_W-1:0]                    m_axis_tdest,

    // ---- AXI Master Read 通道 (仍走外部 DDR-mock, WB/RDMA fetch) ----
    output logic [AXI_M_ID+AXI_M_WIDTH-1:0]      bus_arid,
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
    input  logic [AXI_M_ID+AXI_M_WIDTH-1:0]      bus_rid,
    input  logic [BUS_DATA_W-1:0]                bus_rdata,
    input  logic [1:0]                           bus_rresp,
    input  logic                                 bus_rlast,
    input  logic                                 bus_rvalid,
    output logic                                 bus_rready,

    // ---- Done sticky / debug ----
    output logic                                 done
);

    // ---- ConvCore cfg_regs 输出: ODMA 出包的 mesh 目的节点 + opcode ----
    logic [DEST_W-1:0]                 cfg_ofm_tdest;
    logic [3:0]                        cfg_ofm_opcode;

    // ---- ConvCore 内部 axi master write 通道 (会被桥接到 mesh) ----
    logic [AXI_M_ID+AXI_M_WIDTH-1:0]   core_bus_awid;
    logic [BUS_ADDR_W-1:0]             core_bus_awaddr;
    logic [7:0]                        core_bus_awlen;
    logic [2:0]                        core_bus_awsize;
    logic [1:0]                        core_bus_awburst;
    logic                              core_bus_awlock;
    logic [3:0]                        core_bus_awcache;
    logic [2:0]                        core_bus_awprot;
    logic [3:0]                        core_bus_awqos;
    logic                              core_bus_awvalid;
    logic                              core_bus_awready;
    logic [BUS_DATA_W-1:0]             core_bus_wdata;
    logic [BUS_DATA_W/8-1:0]           core_bus_wstrb;
    logic                              core_bus_wlast;
    logic                              core_bus_wvalid;
    logic                              core_bus_wready;
    logic [AXI_M_ID+AXI_M_WIDTH-1:0]   core_bus_bid;
    logic [1:0]                        core_bus_bresp;
    logic                              core_bus_bvalid;
    logic                              core_bus_bready;

    // ---- ConvCore 的 rmt_ifb_axi_slave (来自 mesh 桥) ----
    logic [RMT_ID_W-1:0]               rmt_ifb_awid;
    logic [BUS_ADDR_W-1:0]             rmt_ifb_awaddr;
    logic [7:0]                        rmt_ifb_awlen;
    logic [2:0]                        rmt_ifb_awsize;
    logic [1:0]                        rmt_ifb_awburst;
    logic                              rmt_ifb_awlock;
    logic [3:0]                        rmt_ifb_awcache;
    logic [2:0]                        rmt_ifb_awprot;
    logic [3:0]                        rmt_ifb_awqos;
    logic                              rmt_ifb_awvalid;
    logic                              rmt_ifb_awready;
    logic [BUS_DATA_W-1:0]             rmt_ifb_wdata;
    logic [BUS_DATA_W/8-1:0]           rmt_ifb_wstrb;
    logic                              rmt_ifb_wlast;
    logic                              rmt_ifb_wvalid;
    logic                              rmt_ifb_wready;
    logic [RMT_ID_W-1:0]               rmt_ifb_bid;
    logic [1:0]                        rmt_ifb_bresp;
    logic                              rmt_ifb_bvalid;
    logic                              rmt_ifb_bready;

    // =========================================================================
    // s_axis demux (NUMA 模式有效):
    //   WRITE / WRITE_DDR_OFB packet (opcode 0x0/0x5) → u_rx_bridge (push IFB)
    //   READ_RESP packet         (opcode 0x2)         → u_read_resp_bridge (NUMA RESP)
    // Phase 6 模式 (NUMA_MODE=0) 直接 forward 给 u_rx_bridge, 不 demux
    // =========================================================================
    logic                rx_axis_tvalid;
    logic                rx_axis_tready;
    logic                resp_axis_tvalid;
    logic                resp_axis_tready;
    logic [3:0]          s_axis_hdr_opcode;
    assign s_axis_hdr_opcode = s_axis_tdata[127:124];

    typedef enum logic [1:0] { S_ROUTE_IDLE, S_ROUTE_WR, S_ROUTE_RESP } s_route_t;
    s_route_t s_route;

    generate if (NUMA_MODE == 0) begin : g_phase6_in
        // Phase 6: 全部 packet 进 u_rx_bridge
        assign rx_axis_tvalid    = s_axis_tvalid;
        assign s_axis_tready     = rx_axis_tready;
        assign resp_axis_tvalid  = 1'b0;
    end else begin : g_numa_in
        always_ff @(posedge clk) begin
            if (!rst_n) s_route <= S_ROUTE_IDLE;
            else if (s_axis_tvalid && s_axis_tready) begin
                if (s_route == S_ROUTE_IDLE) begin
                    if (s_axis_hdr_opcode == 4'h2) s_route <= s_axis_tlast ? S_ROUTE_IDLE : S_ROUTE_RESP;
                    else                            s_route <= s_axis_tlast ? S_ROUTE_IDLE : S_ROUTE_WR;
                end else if (s_axis_tlast) begin
                    s_route <= S_ROUTE_IDLE;
                end
            end
        end
        always_comb begin
            rx_axis_tvalid   = 1'b0;
            resp_axis_tvalid = 1'b0;
            s_axis_tready    = 1'b0;
            case (s_route)
                S_ROUTE_IDLE: begin
                    if (s_axis_hdr_opcode == 4'h2) begin
                        resp_axis_tvalid = s_axis_tvalid;
                        s_axis_tready    = resp_axis_tready;
                    end else begin
                        rx_axis_tvalid   = s_axis_tvalid;
                        s_axis_tready    = rx_axis_tready;
                    end
                end
                S_ROUTE_WR: begin
                    rx_axis_tvalid = s_axis_tvalid;
                    s_axis_tready  = rx_axis_tready;
                end
                S_ROUTE_RESP: begin
                    resp_axis_tvalid = s_axis_tvalid;
                    s_axis_tready    = resp_axis_tready;
                end
            endcase
        end
    end endgenerate

    // ---- Bridge 1: AXIS slave (WRITE packet) → AXI4 master writer (写 ConvCore.rmt_ifb_*) ----
    axis_to_axi_writer #(
        .DATA_W(BUS_DATA_W), .DEST_W(DEST_W),
        .ADDR_W(BUS_ADDR_W), .ID_W(RMT_ID_W)
    ) u_rx_bridge (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tvalid(rx_axis_tvalid), .s_axis_tready(rx_axis_tready),
        .s_axis_tdata(s_axis_tdata),   .s_axis_tlast(s_axis_tlast),
        .s_axis_tdest(s_axis_tdest),
        .cfg_base_addr(32'h0),
        .m_axi_awid(rmt_ifb_awid), .m_axi_awaddr(rmt_ifb_awaddr),
        .m_axi_awlen(rmt_ifb_awlen), .m_axi_awsize(rmt_ifb_awsize),
        .m_axi_awburst(rmt_ifb_awburst), .m_axi_awlock(rmt_ifb_awlock),
        .m_axi_awcache(rmt_ifb_awcache),
        .m_axi_awprot(rmt_ifb_awprot), .m_axi_awqos(rmt_ifb_awqos),
        .m_axi_awvalid(rmt_ifb_awvalid), .m_axi_awready(rmt_ifb_awready),
        .m_axi_wdata(rmt_ifb_wdata), .m_axi_wstrb(rmt_ifb_wstrb),
        .m_axi_wlast(rmt_ifb_wlast), .m_axi_wvalid(rmt_ifb_wvalid),
        .m_axi_wready(rmt_ifb_wready),
        .m_axi_bid(rmt_ifb_bid), .m_axi_bresp(rmt_ifb_bresp),
        .m_axi_bvalid(rmt_ifb_bvalid), .m_axi_bready(rmt_ifb_bready)
    );

    // =========================================================================
    // Bridge 2: ConvCore.bus_aw/w/b (ODMA write) → m_axis (WRITE packet)
    //   NUMA_MODE=0: tdest=cfg_ofm_tdest 单值 (legacy push)
    //   NUMA_MODE=1: tdest=awaddr[25:24] 自动解码 mem ID
    // =========================================================================
    logic              wr_axis_tvalid;
    logic              wr_axis_tready;
    logic [BUS_DATA_W-1:0] wr_axis_tdata;
    logic              wr_axis_tlast;
    logic [DEST_W-1:0] wr_axis_tdest;

    axi_writer_to_axis #(
        .DATA_W(BUS_DATA_W), .DEST_W(DEST_W),
        .ADDR_W(BUS_ADDR_W), .ID_W(AXI_M_ID + AXI_M_WIDTH)
    ) u_tx_bridge (
        .clk(clk), .rst_n(rst_n),
        .s_axi_awid(core_bus_awid), .s_axi_awaddr(core_bus_awaddr),
        .s_axi_awlen(core_bus_awlen), .s_axi_awsize(core_bus_awsize),
        .s_axi_awburst(core_bus_awburst),
        .s_axi_awvalid(core_bus_awvalid), .s_axi_awready(core_bus_awready),
        .s_axi_wdata(core_bus_wdata), .s_axi_wstrb(core_bus_wstrb),
        .s_axi_wlast(core_bus_wlast), .s_axi_wvalid(core_bus_wvalid),
        .s_axi_wready(core_bus_wready),
        .s_axi_bid(core_bus_bid), .s_axi_bresp(core_bus_bresp),
        .s_axi_bvalid(core_bus_bvalid), .s_axi_bready(core_bus_bready),
        .cfg_opcode(cfg_ofm_opcode),
        .cfg_use_addr_route(NUMA_MODE == 1),
        .cfg_tdest(cfg_ofm_tdest),
        .m_axis_tvalid(wr_axis_tvalid), .m_axis_tready(wr_axis_tready),
        .m_axis_tdata(wr_axis_tdata),   .m_axis_tlast(wr_axis_tlast),
        .m_axis_tdest(wr_axis_tdest)
    );

    // =========================================================================
    // Bridge 3 (NUMA only): ConvCore.bus_ar (IDMA read) → m_axis (READ_REQ packet)
    // =========================================================================
    logic              rrq_axis_tvalid;
    logic              rrq_axis_tready;
    logic [BUS_DATA_W-1:0] rrq_axis_tdata;
    logic              rrq_axis_tlast;
    logic [DEST_W-1:0] rrq_axis_tdest;

    // ConvCore.bus_ar 信号: NUMA 模式接 axi_reader_to_axis, Phase 6 模式直 expose
    logic [AXI_M_ID+AXI_M_WIDTH-1:0]   core_bus_arid;
    logic [BUS_ADDR_W-1:0]             core_bus_araddr;
    logic [7:0]                        core_bus_arlen;
    logic [2:0]                        core_bus_arsize;
    logic [1:0]                        core_bus_arburst;
    logic                              core_bus_arlock;
    logic [3:0]                        core_bus_arcache;
    logic [2:0]                        core_bus_arprot;
    logic [3:0]                        core_bus_arqos;
    logic                              core_bus_arvalid;
    logic                              core_bus_arready;
    logic [AXI_M_ID+AXI_M_WIDTH-1:0]   core_bus_rid;
    logic [BUS_DATA_W-1:0]             core_bus_rdata;
    logic [1:0]                        core_bus_rresp;
    logic                              core_bus_rlast;
    logic                              core_bus_rvalid;
    logic                              core_bus_rready;

    generate if (NUMA_MODE == 1) begin : g_numa_read
        // axi_reader_to_axis 输出 total_beats (= arlen+1), 给 resp 桥跟踪 rlast
        logic [15:0] rrq_total_beats;
        axi_reader_to_axis #(
            .DATA_W(BUS_DATA_W), .DEST_W(DEST_W),
            .ADDR_W(BUS_ADDR_W), .ID_W(AXI_M_ID + AXI_M_WIDTH)
        ) u_rrq_bridge (
            .clk(clk), .rst_n(rst_n),
            .s_axi_arid(core_bus_arid), .s_axi_araddr(core_bus_araddr),
            .s_axi_arlen(core_bus_arlen), .s_axi_arsize(core_bus_arsize),
            .s_axi_arburst(core_bus_arburst),
            .s_axi_arvalid(core_bus_arvalid), .s_axi_arready(core_bus_arready),
            .m_axis_tvalid(rrq_axis_tvalid), .m_axis_tready(rrq_axis_tready),
            .m_axis_tdata(rrq_axis_tdata),   .m_axis_tlast(rrq_axis_tlast),
            .m_axis_tdest(rrq_axis_tdest),
            .cfg_return_tdest(8'(CORE_LOCAL_ID)),
            .total_beats_out(rrq_total_beats)
        );

        // RESP 桥: s_axis (READ_RESP packet) → ConvCore.bus_r
        // ARID 跟踪: 简化用上次 AR 的 ARID (单 outstanding burst 假设)
        logic [AXI_M_ID+AXI_M_WIDTH-1:0] r_outstanding_arid;
        always_ff @(posedge clk) begin
            if (!rst_n) r_outstanding_arid <= '0;
            else if (core_bus_arvalid && core_bus_arready) r_outstanding_arid <= core_bus_arid;
        end

        axis_to_axi_read_resp #(
            .DATA_W(BUS_DATA_W), .DEST_W(DEST_W), .ID_W(AXI_M_ID + AXI_M_WIDTH)
        ) u_resp_bridge (
            .clk(clk), .rst_n(rst_n),
            .s_axis_tvalid(resp_axis_tvalid), .s_axis_tready(resp_axis_tready),
            .s_axis_tdata(s_axis_tdata),     .s_axis_tlast(s_axis_tlast),
            .s_axis_tdest(s_axis_tdest),
            .m_axi_rid(core_bus_rid), .m_axi_rdata(core_bus_rdata),
            .m_axi_rresp(core_bus_rresp), .m_axi_rlast(core_bus_rlast),
            .m_axi_rvalid(core_bus_rvalid), .m_axi_rready(core_bus_rready),
            .cfg_outstanding_arid(r_outstanding_arid),
            .cfg_total_beats(rrq_total_beats)
        );
    end else begin : g_phase6_read
        // Phase 6: bus_ar/r 直 forward 到 wrapper port (外部 DDR), NUMA 桥不工作
        assign rrq_axis_tvalid = 1'b0;
        assign resp_axis_tready = 1'b0;
    end endgenerate

    // bus_ar/r forward (NUMA 模式时 wrapper port 不用, tie 0/expose; Phase 6 时 1:1)
    generate if (NUMA_MODE == 0) begin : g_p6_arrf
        assign bus_arid    = core_bus_arid;
        assign bus_araddr  = core_bus_araddr;
        assign bus_arlen   = core_bus_arlen;
        assign bus_arsize  = core_bus_arsize;
        assign bus_arburst = core_bus_arburst;
        assign bus_arlock  = core_bus_arlock;
        assign bus_arcache = core_bus_arcache;
        assign bus_arprot  = core_bus_arprot;
        assign bus_arqos   = core_bus_arqos;
        assign bus_arvalid = core_bus_arvalid;
        assign core_bus_arready = bus_arready;
        assign core_bus_rid     = bus_rid;
        assign core_bus_rdata   = bus_rdata;
        assign core_bus_rresp   = bus_rresp;
        assign core_bus_rlast   = bus_rlast;
        assign core_bus_rvalid  = bus_rvalid;
        assign bus_rready  = core_bus_rready;
    end else begin : g_numa_arrf
        // NUMA: wrapper bus_ar/r port tie 0 / 不动 (read 走 mesh, 不出 wrapper)
        assign bus_arid    = '0;
        assign bus_araddr  = '0;
        assign bus_arlen   = '0;
        assign bus_arsize  = '0;
        assign bus_arburst = '0;
        assign bus_arlock  = '0;
        assign bus_arcache = '0;
        assign bus_arprot  = '0;
        assign bus_arqos   = '0;
        assign bus_arvalid = 1'b0;
        assign bus_rready  = 1'b0;
    end endgenerate

    // =========================================================================
    // m_axis 出口 mux: WRITE packet (u_tx_bridge) + READ_REQ packet (u_rrq_bridge)
    //   NUMA_MODE=0: 直接走 wr (Phase 6)
    //   NUMA_MODE=1: 两路 mux (packet-locked, 防 wormhole 切换)
    // =========================================================================
    typedef enum logic [1:0] { M_IDLE, M_WR, M_RRQ } m_route_t;
    m_route_t m_route;

    generate if (NUMA_MODE == 0) begin : g_phase6_out
        assign m_axis_tvalid  = wr_axis_tvalid;
        assign m_axis_tdata   = wr_axis_tdata;
        assign m_axis_tlast   = wr_axis_tlast;
        assign m_axis_tdest   = wr_axis_tdest;
        assign wr_axis_tready = m_axis_tready;
        assign rrq_axis_tready = 1'b0;
    end else begin : g_numa_out
        always_ff @(posedge clk) begin
            if (!rst_n) m_route <= M_IDLE;
            else begin
                case (m_route)
                    M_IDLE: begin
                        if      (wr_axis_tvalid)  m_route <= wr_axis_tlast  ? M_IDLE : M_WR;
                        else if (rrq_axis_tvalid) m_route <= rrq_axis_tlast ? M_IDLE : M_RRQ;
                    end
                    M_WR : if (wr_axis_tvalid && wr_axis_tready && wr_axis_tlast)   m_route <= M_IDLE;
                    M_RRQ: if (rrq_axis_tvalid && rrq_axis_tready && rrq_axis_tlast) m_route <= M_IDLE;
                endcase
            end
        end
        always_comb begin
            m_axis_tvalid   = 1'b0; m_axis_tdata = '0; m_axis_tlast = 1'b0; m_axis_tdest = '0;
            wr_axis_tready  = 1'b0; rrq_axis_tready = 1'b0;
            case (m_route)
                M_IDLE: begin
                    if (wr_axis_tvalid) begin
                        m_axis_tvalid = wr_axis_tvalid; m_axis_tdata = wr_axis_tdata;
                        m_axis_tlast = wr_axis_tlast; m_axis_tdest = wr_axis_tdest;
                        wr_axis_tready = m_axis_tready;
                    end else if (rrq_axis_tvalid) begin
                        m_axis_tvalid = rrq_axis_tvalid; m_axis_tdata = rrq_axis_tdata;
                        m_axis_tlast = rrq_axis_tlast; m_axis_tdest = rrq_axis_tdest;
                        rrq_axis_tready = m_axis_tready;
                    end
                end
                M_WR : begin
                    m_axis_tvalid = wr_axis_tvalid; m_axis_tdata = wr_axis_tdata;
                    m_axis_tlast = wr_axis_tlast; m_axis_tdest = wr_axis_tdest;
                    wr_axis_tready = m_axis_tready;
                end
                M_RRQ: begin
                    m_axis_tvalid = rrq_axis_tvalid; m_axis_tdata = rrq_axis_tdata;
                    m_axis_tlast = rrq_axis_tlast; m_axis_tdest = rrq_axis_tdest;
                    rrq_axis_tready = m_axis_tready;
                end
            endcase
        end
    end endgenerate

    // ---- core_top 实例 ----
    core_top #(
        .NUM_COL(NUM_COL), .NUM_PE(NUM_PE), .DATA_WIDTH(DATA_WIDTH),
        .PSUM_WIDTH(PSUM_WIDTH), .SRAM_DEPTH(SRAM_DEPTH),
        .CSR_ADDR_W(CSR_ADDR_W), .CSR_DATA_W(CSR_DATA_W),
        .BUS_ADDR_W(BUS_ADDR_W), .BUS_DATA_W(BUS_DATA_W),
        .AXI_M_ID(AXI_M_ID), .AXI_M_WIDTH(AXI_M_WIDTH),
        .RMT_ID_W(RMT_ID_W)
    ) u_core (
        .clk(clk), .rst_n(rst_n),
        .csr_awaddr(csr_awaddr), .csr_awvalid(csr_awvalid), .csr_awready(csr_awready),
        .csr_wdata(csr_wdata), .csr_wstrb(csr_wstrb),
        .csr_wvalid(csr_wvalid), .csr_wready(csr_wready),
        .csr_bresp(csr_bresp), .csr_bvalid(csr_bvalid), .csr_bready(csr_bready),
        .csr_araddr(csr_araddr), .csr_arvalid(csr_arvalid), .csr_arready(csr_arready),
        .csr_rdata(csr_rdata), .csr_rresp(csr_rresp),
        .csr_rvalid(csr_rvalid), .csr_rready(csr_rready),

        // bus_* axi master: write 通道接 axi_writer_to_axis, read 通道直 expose
        .bus_awid(core_bus_awid), .bus_awaddr(core_bus_awaddr),
        .bus_awlen(core_bus_awlen), .bus_awsize(core_bus_awsize),
        .bus_awburst(core_bus_awburst), .bus_awlock(core_bus_awlock),
        .bus_awcache(core_bus_awcache), .bus_awprot(core_bus_awprot),
        .bus_awqos(core_bus_awqos),
        .bus_awvalid(core_bus_awvalid), .bus_awready(core_bus_awready),
        .bus_wdata(core_bus_wdata), .bus_wstrb(core_bus_wstrb),
        .bus_wlast(core_bus_wlast), .bus_wvalid(core_bus_wvalid),
        .bus_wready(core_bus_wready),
        .bus_bid(core_bus_bid), .bus_bresp(core_bus_bresp),
        .bus_bvalid(core_bus_bvalid), .bus_bready(core_bus_bready),

        // bus_ar/r 接 core_bus_ar/r 内部信号 (NUMA 时进 axi_reader_to_axis, Phase 6 时直接 forward 到 wrapper port)
        .bus_arid(core_bus_arid), .bus_araddr(core_bus_araddr),
        .bus_arlen(core_bus_arlen), .bus_arsize(core_bus_arsize),
        .bus_arburst(core_bus_arburst), .bus_arlock(core_bus_arlock),
        .bus_arcache(core_bus_arcache), .bus_arprot(core_bus_arprot),
        .bus_arqos(core_bus_arqos),
        .bus_arvalid(core_bus_arvalid), .bus_arready(core_bus_arready),
        .bus_rid(core_bus_rid), .bus_rdata(core_bus_rdata), .bus_rresp(core_bus_rresp),
        .bus_rlast(core_bus_rlast), .bus_rvalid(core_bus_rvalid), .bus_rready(core_bus_rready),

        // rmt_ifb_*: 接 axis_to_axi_writer 输出
        .rmt_ifb_awid(rmt_ifb_awid), .rmt_ifb_awaddr(rmt_ifb_awaddr),
        .rmt_ifb_awlen(rmt_ifb_awlen), .rmt_ifb_awsize(rmt_ifb_awsize),
        .rmt_ifb_awburst(rmt_ifb_awburst), .rmt_ifb_awlock(rmt_ifb_awlock),
        .rmt_ifb_awcache(rmt_ifb_awcache), .rmt_ifb_awprot(rmt_ifb_awprot),
        .rmt_ifb_awqos(rmt_ifb_awqos),
        .rmt_ifb_awvalid(rmt_ifb_awvalid), .rmt_ifb_awready(rmt_ifb_awready),
        .rmt_ifb_wdata(rmt_ifb_wdata), .rmt_ifb_wstrb(rmt_ifb_wstrb),
        .rmt_ifb_wlast(rmt_ifb_wlast), .rmt_ifb_wvalid(rmt_ifb_wvalid),
        .rmt_ifb_wready(rmt_ifb_wready),
        .rmt_ifb_bid(rmt_ifb_bid), .rmt_ifb_bresp(rmt_ifb_bresp),
        .rmt_ifb_bvalid(rmt_ifb_bvalid), .rmt_ifb_bready(rmt_ifb_bready),
        // ar/r tie 0 (rmt 不读)
        .rmt_ifb_arid('0), .rmt_ifb_araddr('0),
        .rmt_ifb_arlen('0), .rmt_ifb_arsize('0),
        .rmt_ifb_arburst('0), .rmt_ifb_arlock('0),
        .rmt_ifb_arcache('0), .rmt_ifb_arprot('0),
        .rmt_ifb_arqos('0),
        .rmt_ifb_arvalid(1'b0), .rmt_ifb_arready(),
        .rmt_ifb_rid(), .rmt_ifb_rdata(), .rmt_ifb_rresp(),
        .rmt_ifb_rlast(), .rmt_ifb_rvalid(), .rmt_ifb_rready(1'b0),

        // tb 后门写口 tie 0
        .ifb_we_ext(1'b0), .ifb_waddr_ext('0), .ifb_wdata_ext('0),
        .wb_we_ext(1'b0),  .wb_waddr_ext('0),  .wb_wdata_ext('0),
        .ofb_re_ext(1'b0), .ofb_raddr_ext('0), .ofb_rdata_ext(),

        .done(done),

        // mesh: cfg_regs 直通到 wrapper 内 axi_writer_to_axis
        .ofm_tdest(cfg_ofm_tdest),
        .ofm_opcode(cfg_ofm_opcode)
    );

endmodule
