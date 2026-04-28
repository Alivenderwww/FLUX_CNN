`timescale 1ns/1ps

// =============================================================================
// core_top.sv  --  Handshake-Pipelined Core
//
// 去中心化架构：cfg_regs + 5 个自驱动模块 + 3 个 SRAM：
//
//   line_buffer ─►(act valid/ready)─► mac_array ─►(psum valid/ready)─► parf_accum
//                                      ▲                                   │
//                       wgt_buffer ─►(wgt valid/ready)                      ▼
//                                                                        ofb_writer → OFB
//
// 每个 feeder 自带 6 层循环 + start/done；模块间握手对齐节拍，天然消除气泡。
// 外部 host 通过 AXI-Lite S 写入配置 + DMA 描述符 + 启停控制；done 仍作顶层
// observability 输出。TB 用 $readmemh 填 IFB/WB mem（hier），用 AXI-Lite 下
// 发 cfg 和 start。
// =============================================================================

module core_top #(
    parameter int NUM_COL     = 16,
    parameter int NUM_PE      = 16,
    parameter int DATA_WIDTH  = 8,
    parameter int PSUM_WIDTH  = 32,
    parameter int WRF_DEPTH   = 32,
    parameter int ARF_DEPTH   = 32,
    parameter int PARF_DEPTH  = 32,
    parameter int SRAM_DEPTH  = 8192,
    parameter int CSR_ADDR_W  = 12,        // AXI-Lite 地址位宽
    parameter int CSR_DATA_W  = 32,
    parameter int BUS_ADDR_W  = 32,        // 外部 AXI M 地址位宽（DDR）
    parameter int BUS_DATA_W  = 128,       // 外部 AXI M 数据位宽
    parameter int AXI_M_ID    = 2,         // per-DMA AXI ID 位宽
    parameter int AXI_M_WIDTH = 2,         // log2(master 数) — 4 slot: IDMA/WDMA/ODMA/reserved
    parameter int DMA_LEN_W   = 24         // DMA byte_len 位宽
)(
    input  logic                                 clk,
    input  logic                                 rst_n,

    // ---- AXI-Lite Slave (host 配置通道) ----
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

    // ---- 外部 AXI4 Master (DMA 聚合出核) ----
    output logic [AXI_M_ID+AXI_M_WIDTH-1:0]      bus_awid,
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

    input  logic [AXI_M_ID+AXI_M_WIDTH-1:0]      bus_bid,
    input  logic [1:0]                           bus_bresp,
    input  logic                                 bus_bvalid,
    output logic                                 bus_bready,

    output logic [AXI_M_ID+AXI_M_WIDTH-1:0]      bus_arid,
    output logic [BUS_ADDR_W-1:0]                bus_araddr,
    output logic [7:0]                           bus_arlen,
    output logic [1:0]                           bus_arburst,
    output logic                                 bus_arvalid,
    input  logic                                 bus_arready,

    input  logic [AXI_M_ID+AXI_M_WIDTH-1:0]      bus_rid,
    input  logic [BUS_DATA_W-1:0]                bus_rdata,
    input  logic [1:0]                           bus_rresp,
    input  logic                                 bus_rlast,
    input  logic                                 bus_rvalid,
    output logic                                 bus_rready,

    // TB 外部 SRAM 写口 (加载 IFB / WB) — DMA 未接入时保留作为 $readmemh 后门
    input  logic                                 ifb_we_ext,
    input  logic [$clog2(SRAM_DEPTH)-1:0]        ifb_waddr_ext,
    input  logic [NUM_PE*DATA_WIDTH-1:0]         ifb_wdata_ext,

    input  logic                                 wb_we_ext,
    input  logic [$clog2(SRAM_DEPTH)-1:0]        wb_waddr_ext,
    input  logic [NUM_COL*NUM_PE*DATA_WIDTH-1:0] wb_wdata_ext,

    input  logic                                 ofb_re_ext,
    input  logic [$clog2(SRAM_DEPTH)-1:0]        ofb_raddr_ext,
    output logic [NUM_COL*DATA_WIDTH-1:0]        ofb_rdata_ext,

    // TB 调试观测 (mac_array 实时 psum)
    output logic signed [NUM_COL*PSUM_WIDTH-1:0] psum_out_vec,

    // 顶层 done
    output logic                                 done
);

    localparam int ADDR_W    = 20;
    localparam int IFB_WIDTH = NUM_PE * DATA_WIDTH;
    localparam int WB_WIDTH  = NUM_COL * NUM_PE * DATA_WIDTH;
    localparam int OFB_WIDTH = NUM_COL * DATA_WIDTH;
    localparam int AW        = $clog2(SRAM_DEPTH);

    // =========================================================================
    // 1. AXI-Lite CSR bridge + 共享配置寄存器
    //    axi_lite_csr 解码 AXI-Lite → (reg_w_en/addr/data/strb, reg_r_addr/data)
    //    cfg_regs 消费这些信号，产生 cfg_* 输出 + start_pulse + DMA 描述符
    //    w_out 目前无下游消费（多核融合时用），空连接避免 unused-wire lint。
    // =========================================================================
    logic                   reg_w_en;
    logic [CSR_ADDR_W-1:0]  reg_w_addr;
    logic [CSR_DATA_W-1:0]  reg_w_data;
    logic [CSR_DATA_W/8-1:0]reg_w_strb;
    logic [CSR_ADDR_W-1:0]  reg_r_addr;
    logic [CSR_DATA_W-1:0]  reg_r_data;

    axi_lite_csr #(
        .ADDR_W(CSR_ADDR_W),
        .DATA_W(CSR_DATA_W)
    ) u_csr_bridge (
        .clk(clk), .rstn(rst_n),
        .AWADDR(csr_awaddr),   .AWVALID(csr_awvalid), .AWREADY(csr_awready),
        .WDATA(csr_wdata),     .WSTRB(csr_wstrb),     .WVALID(csr_wvalid),  .WREADY(csr_wready),
        .BRESP(csr_bresp),     .BVALID(csr_bvalid),   .BREADY(csr_bready),
        .ARADDR(csr_araddr),   .ARVALID(csr_arvalid), .ARREADY(csr_arready),
        .RDATA(csr_rdata),     .RRESP(csr_rresp),     .RVALID(csr_rvalid),  .RREADY(csr_rready),
        .reg_w_en(reg_w_en),   .reg_w_addr(reg_w_addr), .reg_w_data(reg_w_data), .reg_w_strb(reg_w_strb),
        .reg_r_addr(reg_r_addr), .reg_r_data(reg_r_data)
    );

    logic [15:0]       cfg_h_out, cfg_w_out, cfg_w_in;
    logic [3:0]        cfg_k;
    logic [3:0]        cfg_ky;
    logic [2:0]        cfg_stride;
    logic [5:0]        cfg_cin_slices, cfg_cout_slices, cfg_tile_w, cfg_last_valid_w;
    logic [7:0]        cfg_num_tiles;
    logic [9:0]        cfg_total_wrf, cfg_kk;
    logic [2:0]        cfg_rounds_per_cins;
    logic [5:0]        cfg_round_len_last;
    logic [ADDR_W-1:0] cfg_ifb_base, cfg_wb_base, cfg_ofb_base;
    logic [ADDR_W-1:0] cfg_ifb_row_step;
    logic [ADDR_W-1:0] cfg_wb_cout_step;
    logic [ADDR_W-1:0] cfg_tile_in_step;
    // Phase D 新增：NHWC + 方式 1 预算值
    logic [ADDR_W-1:0] cfg_ifb_ring_words;
    logic [ADDR_W-1:0] cfg_ofb_row_words;
    logic [ADDR_W-1:0] cfg_ofb_ring_words;
    logic [ADDR_W-1:0] cfg_iss_step;
    logic [ADDR_W-1:0] cfg_ifb_ky_step;
    logic [15:0]       cfg_tile_pix_step;
    logic              cfg_arf_reuse_en;
    logic [5:0]        cfg_sdp_shift;
    logic              cfg_sdp_relu_en;

    // Descriptor list + new SDP fields
    logic [31:0]       cfg_desc_list_base;
    logic [15:0]       cfg_desc_count;
    logic signed [31:0] cfg_sdp_mult;
    logic signed [8:0]  cfg_sdp_zp_out, cfg_sdp_clip_min, cfg_sdp_clip_max;
    logic              cfg_sdp_round_en;

    // Streaming / ring cfg (J-2 起恒 streaming)
    logic [15:0]       cfg_h_in_total;
    logic [7:0]        cfg_ifb_strip_rows;
    logic [5:0]        cfg_ofb_strip_rows;
    logic [ADDR_W-1:0] cfg_ddr_ifm_row_stride;
    logic [ADDR_W-1:0] cfg_ddr_ofm_row_stride;

    // CTRL / STATUS
    logic              cfg_start_dfe_pulse, cfg_start_layer_pulse;
    logic [31:0]       dma_idma_src_base, dma_wdma_src_base, dma_odma_dst_base;
    logic [23:0]       dma_idma_byte_len, dma_wdma_byte_len, dma_odma_byte_len;
    logic [31:0]       dma_rdma_src_base;
    logic [23:0]       dma_rdma_byte_len;
    logic              idma_busy, idma_done, wdma_busy, wdma_done, odma_busy, odma_done;
    logic              rdma_busy, rdma_done;
    logic              dfe_busy, dfe_done, layer_busy, layer_done;

    // R.1: residual / shortcut / bias cfg
    logic              cfg_residual_en;
    logic signed [15:0] cfg_shortcut_mult;
    logic [4:0]        cfg_shortcut_shift;
    logic [12:0]       cfg_bias_base;

    // Sequencer → core pipeline + DMA
    logic              seq_start_core_pulse, seq_start_wgt_pulse;
    logic              seq_start_idma_pulse, seq_start_odma_pulse, seq_start_wdma_pulse;
    logic              seq_start_rdma_pulse;
    logic [15:0]       seq_strip_n_yout;
    logic [3:0]        seq_pad_top, seq_pad_bot, seq_pad_left, seq_pad_right;
    logic [15:0]       seq_strip_y_start;
    logic [19:0]       seq_strip_ifb_ddr_offset, seq_strip_ofb_ddr_offset;
    logic [23:0]       seq_strip_ifb_byte_len, seq_strip_ofb_byte_len;

    // IFB base 预扣 pad offset: ring-aware 减法
    //   eff = cfg_ifb_base + ring_words - pad_offset (保证落在 [0, 2×ring) 内,
    //   wrap_addr 可单次 mod 到正确 ring 位置)
    logic [ADDR_W-1:0] pad_offset;
    assign pad_offset = ({{(ADDR_W-20){1'b0}}, {12'd0, seq_pad_top} * cfg_ifb_ky_step[15:0]})
                      + ({{(ADDR_W-10){1'b0}}, seq_pad_left * cfg_cin_slices});

    logic [ADDR_W-1:0] eff_ifb_base;
    assign eff_ifb_base = cfg_ifb_base + cfg_ifb_ring_words - pad_offset;

    // Per-strip DMA src_base / byte_len = 全局 base + descriptor offset
    logic [31:0]       eff_idma_src_base, eff_odma_dst_base;
    logic [23:0]       eff_idma_byte_len, eff_odma_byte_len;
    assign eff_idma_src_base = dma_idma_src_base + {12'd0, seq_strip_ifb_ddr_offset};
    assign eff_odma_dst_base = dma_odma_dst_base + {12'd0, seq_strip_ofb_ddr_offset};
    assign eff_idma_byte_len = seq_strip_ifb_byte_len;
    assign eff_odma_byte_len = seq_strip_ofb_byte_len;

    // desc FIFO
    logic [255:0]      desc_fifo_wdata, desc_fifo_rdata;
    logic              desc_fifo_we, desc_fifo_re;
    logic              desc_fifo_full, desc_fifo_empty;

    cfg_regs #(
        .ADDR_W(CSR_ADDR_W),
        .DATA_W(CSR_DATA_W),
        .CORE_ADDR_W(ADDR_W)
    ) u_cfg (
        .clk(clk), .rst_n(rst_n),
        .reg_w_en(reg_w_en), .reg_w_addr(reg_w_addr),
        .reg_w_data(reg_w_data), .reg_w_strb(reg_w_strb),
        .reg_r_addr(reg_r_addr), .reg_r_data(reg_r_data),
        .core_done(done),
        .idma_busy(idma_busy), .wdma_busy(wdma_busy), .odma_busy(odma_busy),
        .idma_done(idma_done), .wdma_done(wdma_done), .odma_done(odma_done),
        .dfe_busy(dfe_busy), .dfe_done(dfe_done),
        .layer_busy(layer_busy), .layer_done(layer_done),
        .start_dfe_pulse(cfg_start_dfe_pulse),
        .start_layer_pulse(cfg_start_layer_pulse),
        .h_out(cfg_h_out), .w_out(cfg_w_out), .w_in(cfg_w_in),
        .k(cfg_k), .ky(cfg_ky), .stride(cfg_stride),
        .cin_slices(cfg_cin_slices), .cout_slices(cfg_cout_slices),
        .tile_w(cfg_tile_w), .num_tiles(cfg_num_tiles), .last_valid_w(cfg_last_valid_w),
        .total_wrf(cfg_total_wrf),
        .kk(cfg_kk), .rounds_per_cins(cfg_rounds_per_cins), .round_len_last(cfg_round_len_last),
        .ifb_base(cfg_ifb_base), .wb_base(cfg_wb_base), .ofb_base(cfg_ofb_base),
        .ifb_row_step(cfg_ifb_row_step),
        .wb_cout_step(cfg_wb_cout_step),
        .tile_in_step(cfg_tile_in_step),
        .ifb_ring_words(cfg_ifb_ring_words),
        .ofb_row_words(cfg_ofb_row_words),
        .ofb_ring_words(cfg_ofb_ring_words),
        .ifb_iss_step(cfg_iss_step),
        .ifb_ky_step(cfg_ifb_ky_step),
        .tile_pix_step(cfg_tile_pix_step),
        .arf_reuse_en(cfg_arf_reuse_en),
        .sdp_shift(cfg_sdp_shift), .sdp_relu_en(cfg_sdp_relu_en),
        .h_in_total(cfg_h_in_total),
        .ifb_strip_rows(cfg_ifb_strip_rows),
        .ofb_strip_rows(cfg_ofb_strip_rows),
        .ddr_ifm_row_stride(cfg_ddr_ifm_row_stride),
        .ddr_ofm_row_stride(cfg_ddr_ofm_row_stride),
        .idma_src_base(dma_idma_src_base), .idma_byte_len(dma_idma_byte_len),
        .wdma_src_base(dma_wdma_src_base), .wdma_byte_len(dma_wdma_byte_len),
        .odma_dst_base(dma_odma_dst_base), .odma_byte_len(dma_odma_byte_len),
        .desc_list_base(cfg_desc_list_base), .desc_count(cfg_desc_count),
        .sdp_mult(cfg_sdp_mult), .sdp_zp_out(cfg_sdp_zp_out),
        .sdp_clip_min(cfg_sdp_clip_min), .sdp_clip_max(cfg_sdp_clip_max),
        .sdp_round_en(cfg_sdp_round_en),
        // R.1: residual / shortcut / bias / rdma cfg
        .residual_en(cfg_residual_en),
        .shortcut_mult(cfg_shortcut_mult),
        .shortcut_shift(cfg_shortcut_shift),
        .bias_base(cfg_bias_base),
        .rdma_src_base(dma_rdma_src_base),
        .rdma_byte_len(dma_rdma_byte_len)
    );

    // =========================================================================
    // 1b. Descriptor FIFO + Sequencer (Phase C-1)
    // =========================================================================
    desc_fifo #(.DEPTH(32), .WIDTH(256)) u_desc_fifo (
        .clk(clk), .rst_n(rst_n),
        .wr_en  (desc_fifo_we),
        .wr_data(desc_fifo_wdata),
        .full   (desc_fifo_full),
        .rd_en  (desc_fifo_re),
        .rd_data(desc_fifo_rdata),
        .empty  (desc_fifo_empty),
        .count  ()
    );

    sequencer u_sequencer (
        .clk(clk), .rst_n(rst_n),
        .start_layer_pulse     (cfg_start_layer_pulse),
        .layer_busy            (layer_busy),
        .layer_done            (layer_done),
        .fifo_rd_data          (desc_fifo_rdata),
        .fifo_empty            (desc_fifo_empty),
        .fifo_rd_en            (desc_fifo_re),
        .cfg_h_out_total       (cfg_h_out),
        .strip_n_yout          (seq_strip_n_yout),
        .strip_pad_top         (seq_pad_top),
        .strip_pad_bot         (seq_pad_bot),
        .strip_pad_left        (seq_pad_left),
        .strip_pad_right       (seq_pad_right),
        .strip_y_start         (seq_strip_y_start),
        .strip_ifb_ddr_offset  (seq_strip_ifb_ddr_offset),
        .strip_ifb_byte_len    (seq_strip_ifb_byte_len),
        .strip_ofb_ddr_offset  (seq_strip_ofb_ddr_offset),
        .strip_ofb_byte_len    (seq_strip_ofb_byte_len),
        .start_core_pulse      (seq_start_core_pulse),
        .start_wgt_pulse       (seq_start_wgt_pulse),
        .start_idma_pulse      (seq_start_idma_pulse),
        .start_odma_pulse      (seq_start_odma_pulse),
        .start_wdma_pulse      (seq_start_wdma_pulse),
        .start_rdma_pulse      (seq_start_rdma_pulse),
        .core_strip_done       (done),
        .idma_strip_done       (idma_done),
        .odma_strip_done       (odma_done),
        .wdma_done             (wdma_done),
        .rdma_done             (rdma_done)
    );

    // =========================================================================
    // 2. 三块 SRAM (IFB / WB / OFB) + DMA 写/读端口 mux
    //    IFB: IDMA 写，line_buffer 读；WB: WDMA 写，wgt_buffer 读；
    //    OFB: ofb_writer 写，ODMA + TB ext 读（共享 read 端口，DMA 优先）
    // =========================================================================
    logic                   ifb_re;
    logic [AW-1:0]          ifb_raddr;
    logic [IFB_WIDTH-1:0]   ifb_rdata;

    logic                   wb_re;
    logic [AW-1:0]          wb_raddr;
    logic [WB_WIDTH-1:0]    wb_rdata;

    logic                   ofb_we;
    logic [AW-1:0]          ofb_waddr;
    logic [OFB_WIDTH-1:0]   ofb_wdata;

    // DMA → SRAM 内部接口
    logic                   idma_ifb_we;
    logic [AW-1:0]          idma_ifb_waddr;
    logic [IFB_WIDTH-1:0]   idma_ifb_wdata;

    logic                   wdma_wb_we;
    logic [AW-1:0]          wdma_wb_waddr;
    logic [WB_WIDTH-1:0]    wdma_wb_wdata;

    logic                   odma_ofb_re;
    logic [AW-1:0]          odma_ofb_raddr;
    logic [OFB_WIDTH-1:0]   odma_ofb_rdata;

    // IFB 写口：DMA 优先（无冲突默认下 DMA we 和 ext we 不会同时高）
    logic                   ifb_we_mux;
    logic [AW-1:0]          ifb_waddr_mux;
    logic [IFB_WIDTH-1:0]   ifb_wdata_mux;
    assign ifb_we_mux    = idma_ifb_we | ifb_we_ext;
    assign ifb_waddr_mux = idma_ifb_we ? idma_ifb_waddr : ifb_waddr_ext;
    assign ifb_wdata_mux = idma_ifb_we ? idma_ifb_wdata : ifb_wdata_ext;

    logic                   wb_we_mux;
    logic [AW-1:0]          wb_waddr_mux;
    logic [WB_WIDTH-1:0]    wb_wdata_mux;
    assign wb_we_mux    = wdma_wb_we | wb_we_ext;
    assign wb_waddr_mux = wdma_wb_we ? wdma_wb_waddr : wb_waddr_ext;
    assign wb_wdata_mux = wdma_wb_we ? wdma_wb_wdata : wb_wdata_ext;

    // OFB 读口：DMA 优先，rdata 分发给 DMA 和 ext
    logic                   ofb_re_mux;
    logic [AW-1:0]          ofb_raddr_mux;
    logic [OFB_WIDTH-1:0]   ofb_rdata_shared;
    assign ofb_re_mux    = odma_ofb_re | ofb_re_ext;
    assign ofb_raddr_mux = odma_ofb_re ? odma_ofb_raddr : ofb_raddr_ext;
    assign odma_ofb_rdata = ofb_rdata_shared;
    assign ofb_rdata_ext  = ofb_rdata_shared;

    sram_model #(.DEPTH(SRAM_DEPTH), .DATA_WIDTH(IFB_WIDTH)) u_ifb (
        .clk(clk), .we(ifb_we_mux), .waddr(ifb_waddr_mux), .wdata(ifb_wdata_mux),
        .re(ifb_re), .raddr(ifb_raddr), .rdata(ifb_rdata)
    );

    sram_model #(.DEPTH(SRAM_DEPTH), .DATA_WIDTH(WB_WIDTH)) u_wb (
        .clk(clk), .we(wb_we_mux), .waddr(wb_waddr_mux), .wdata(wb_wdata_mux),
        .re(wb_re), .raddr(wb_raddr), .rdata(wb_rdata)
    );

    sram_model #(.DEPTH(SRAM_DEPTH), .DATA_WIDTH(OFB_WIDTH)) u_ofb (
        .clk(clk), .we(ofb_we), .waddr(ofb_waddr), .wdata(ofb_wdata),
        .re(ofb_re_mux), .raddr(ofb_raddr_mux), .rdata(ofb_rdata_shared)
    );

    // =========================================================================
    // 3. Handshake buses
    // =========================================================================
    // line_buffer → mac_array (act)
    logic                       act_valid, act_ready;
    logic [IFB_WIDTH-1:0]       act_vec;

    // wgt_buffer → mac_array (wgt index)
    logic                       wgt_valid, wgt_ready;
    logic [$clog2(WRF_DEPTH)-1:0] wrf_raddr;

    // wgt_buffer → mac_array (WRF write 端口)
    logic [NUM_COL*NUM_PE-1:0]  wrf_we;
    logic [$clog2(WRF_DEPTH)-1:0] wrf_waddr;
    logic [WB_WIDTH-1:0]        wrf_wdata;

    // mac_array → parf_accum (psum 流)
    logic                       psum_out_valid;
    logic                       psum_in_ready;
    // psum_out_vec 是顶层 debug output，不再本地声明

    // parf_accum → ofb_writer (累加结果流)
    logic                       acc_out_valid, acc_out_ready;
    logic signed [NUM_COL*PSUM_WIDTH-1:0] acc_out_vec;

    // =========================================================================
    // 4. line_buffer
    //    lb_done / wb_done 仅供调试观测；顶层 done 只跟踪 ofb_writer（数据通路
    //    最末端 → 前端模块必然已完成）。
    // =========================================================================
    logic lb_done;
    logic [15:0] rows_consumed;
    logic [15:0] rows_available;   // IDMA rows_written → line_buffer 做 forward-pressure

    line_buffer #(
        .NUM_PE    (NUM_PE),
        .DATA_WIDTH(DATA_WIDTH),
        .ARF_DEPTH (ARF_DEPTH),
        .SRAM_DEPTH(SRAM_DEPTH),
        .ADDR_W    (ADDR_W)
    ) u_line_buffer (
        .clk              (clk),
        .rst_n            (rst_n),
        .start            (seq_start_core_pulse),
        .done             (lb_done),
        .cfg_h_out        (seq_strip_n_yout),
        .cfg_w_in         (cfg_w_in),
        .cfg_k            (cfg_k),
        .cfg_ky           (cfg_ky),
        .cfg_stride       (cfg_stride),
        .cfg_cin_slices   (cfg_cin_slices),
        .cfg_cout_slices  (cfg_cout_slices),
        .cfg_tile_w       (cfg_tile_w),
        .cfg_num_tiles    (cfg_num_tiles),
        .cfg_last_valid_w (cfg_last_valid_w),
        .cfg_ifb_base     (eff_ifb_base),
        .cfg_ifb_ring_words(cfg_ifb_ring_words),
        .cfg_ifb_row_step (cfg_ifb_row_step),
        .cfg_tile_in_step (cfg_tile_in_step),
        .cfg_iss_step     (cfg_iss_step),
        .cfg_ifb_ky_step  (cfg_ifb_ky_step),
        .cfg_tile_pix_step(cfg_tile_pix_step),
        .cfg_arf_reuse_en (cfg_arf_reuse_en),
        .cfg_pad_top      (seq_pad_top),
        .cfg_pad_left     (seq_pad_left),
        .cfg_h_in         (cfg_h_in_total),
        .ifb_re           (ifb_re),
        .ifb_raddr        (ifb_raddr),
        .ifb_rdata        (ifb_rdata),
        .act_valid        (act_valid),
        .act_vec          (act_vec),
        .act_ready        (act_ready),
        .rows_consumed    (rows_consumed),
        .rows_available   (rows_available)
    );

    // =========================================================================
    // 5. wgt_buffer
    // =========================================================================
    logic wb_done;

    // R.1: bias 已挪到 SDP, mac_array 不再需要 bias_vec; 仅留 old_psum 回写路径
    logic signed [NUM_COL*PSUM_WIDTH-1:0] old_psum_wire;
    logic                                 is_first_round_fill_wire;

    wgt_buffer #(
        .NUM_COL        (NUM_COL),
        .NUM_PE         (NUM_PE),
        .DATA_WIDTH     (DATA_WIDTH),
        .PSUM_WIDTH     (PSUM_WIDTH),
        .WRF_DEPTH      (WRF_DEPTH),
        .SRAM_DEPTH     (SRAM_DEPTH),
        .ADDR_W         (ADDR_W)
    ) u_wgt_buffer (
        .clk              (clk),
        .rst_n            (rst_n),
        .start            (seq_start_wgt_pulse),
        .done             (wb_done),
        .cfg_h_out        (cfg_h_out),
        .cfg_cin_slices   (cfg_cin_slices),
        .cfg_cout_slices  (cfg_cout_slices),
        .cfg_tile_w       (cfg_tile_w),
        .cfg_num_tiles    (cfg_num_tiles),
        .cfg_last_valid_w (cfg_last_valid_w),
        .cfg_k               (cfg_k),
        .cfg_kk              (cfg_kk),
        .cfg_total_wrf       (cfg_total_wrf),
        .cfg_rounds_per_cins (cfg_rounds_per_cins),
        .cfg_round_len_last  (cfg_round_len_last),
        .cfg_wb_base         (cfg_wb_base),
        .cfg_wb_cout_step    (cfg_wb_cout_step),
        .wb_re            (wb_re),
        .wb_raddr         (wb_raddr),
        .wb_rdata         (wb_rdata),
        .wrf_we           (wrf_we),
        .wrf_waddr        (wrf_waddr),
        .wrf_wdata        (wrf_wdata),
        .wgt_valid        (wgt_valid),
        .wrf_raddr        (wrf_raddr),
        .wgt_ready        (wgt_ready)
    );

    // =========================================================================
    // 6. mac_array
    // =========================================================================
    mac_array #(
        .NUM_COL   (NUM_COL),
        .NUM_PE    (NUM_PE),
        .DATA_WIDTH(DATA_WIDTH),
        .PSUM_WIDTH(PSUM_WIDTH),
        .WRF_DEPTH (WRF_DEPTH)
    ) u_mac_array (
        .clk            (clk),
        .rst_n          (rst_n),
        .wrf_we         (wrf_we),
        .wrf_waddr      (wrf_waddr),
        .wrf_wdata      (wrf_wdata),
        .act_in_vec     (act_vec),
        .act_valid      (act_valid),
        .act_ready      (act_ready),
        .wrf_raddr      (wrf_raddr),
        .wgt_valid      (wgt_valid),
        .wgt_ready      (wgt_ready),
        .psum_out_valid (psum_out_valid),
        .psum_out_vec   (psum_out_vec),
        .psum_in_ready  (psum_in_ready),
        .is_first_round_fill(is_first_round_fill_wire),
        .old_psum_vec       (old_psum_wire)
    );

    // =========================================================================
    // 7. parf_accum
    // =========================================================================
    parf_accum #(
        .NUM_COL   (NUM_COL),
        .PSUM_WIDTH(PSUM_WIDTH),
        .PARF_DEPTH(PARF_DEPTH)
    ) u_parf_accum (
        .clk              (clk),
        .rst_n            (rst_n),
        .cfg_tile_w       (cfg_tile_w),
        .cfg_last_valid_w (cfg_last_valid_w),
        .cfg_num_tiles    (cfg_num_tiles),
        .cfg_cin_slices   (cfg_cin_slices),
        .cfg_kk           (cfg_kk),
        .psum_in_valid    (psum_out_valid),
        .psum_in_vec      (psum_out_vec),
        .psum_in_ready    (psum_in_ready),
        .acc_out_valid    (acc_out_valid),
        .acc_out_vec      (acc_out_vec),
        .acc_out_ready    (acc_out_ready),
        .is_first_round_fill_out(is_first_round_fill_wire),
        .old_psum_at_wr         (old_psum_wire)
    );

    // =========================================================================
    // 8. ofb_writer (含 SDP)
    //    streaming 下 row_done_pulse → ODMA，ODMA 的 rows_drained 反馈回来做
    //    OFB ring 反压
    // =========================================================================
    logic ow_done;
    logic row_done_pulse;
    logic [15:0] rows_written;   // debug only
    logic [15:0] rows_drained;

    // R.1: ofb_writer ↔ Shortcut Bank ↔ bias_rf 之间的接线
    logic                  ow_sb_re;
    logic [AW-1:0]         ow_sb_raddr;
    logic [OFB_WIDTH-1:0]  ow_sb_rdata;          // shortcut data (1 拍延迟回到 ofb_writer)
    logic [5:0]            ow_cs_cnt;            // ofb_writer 当前 cs, 给 bias_rf
    logic [NUM_COL*PSUM_WIDTH-1:0] bias_vec_wire; // bias_rf → ofb_writer (= SDP bias_in)
    logic                  bias_ready_wire;

    ofb_writer #(
        .NUM_COL   (NUM_COL),
        .DATA_WIDTH(DATA_WIDTH),
        .PSUM_WIDTH(PSUM_WIDTH),
        .SRAM_DEPTH(SRAM_DEPTH),
        .ADDR_W    (ADDR_W)
    ) u_ofb_writer (
        .clk              (clk),
        .rst_n            (rst_n),
        .start            (seq_start_core_pulse),
        .done             (ow_done),
        .cfg_h_out        (seq_strip_n_yout),
        .cfg_tile_w       (cfg_tile_w),
        .cfg_last_valid_w (cfg_last_valid_w),
        .cfg_num_tiles    (cfg_num_tiles),
        .cfg_cout_slices  (cfg_cout_slices),
        .cfg_ofb_base     (cfg_ofb_base),
        .cfg_sdp_shift    (cfg_sdp_shift),
        .cfg_sdp_relu_en  (cfg_sdp_relu_en),
        .cfg_sdp_mult     (cfg_sdp_mult),
        .cfg_sdp_zp_out   (cfg_sdp_zp_out),
        .cfg_sdp_clip_min (cfg_sdp_clip_min),
        .cfg_sdp_clip_max (cfg_sdp_clip_max),
        .cfg_sdp_round_en (cfg_sdp_round_en),
        .cfg_ofb_ring_words(cfg_ofb_ring_words),
        .cfg_ofb_row_words (cfg_ofb_row_words),
        .cfg_ofb_strip_rows(cfg_ofb_strip_rows),
        .rows_drained      (rows_drained),
        // R.1: SDP residual / bias / shortcut 接线
        .cfg_residual_en   (cfg_residual_en),
        .cfg_shortcut_mult (cfg_shortcut_mult),
        .cfg_shortcut_shift(cfg_shortcut_shift),
        .bias_in           (bias_vec_wire),
        .bias_ready        (bias_ready_wire),
        .cs_cnt_out        (ow_cs_cnt),
        .sb_re             (ow_sb_re),
        .sb_raddr          (ow_sb_raddr),
        .sb_rdata          (ow_sb_rdata),
        .acc_out_valid    (acc_out_valid),
        .acc_out_vec      (acc_out_vec),
        .acc_out_ready    (acc_out_ready),
        .ofb_we           (ofb_we),
        .ofb_waddr        (ofb_waddr),
        .ofb_wdata        (ofb_wdata),
        .row_done_pulse   (row_done_pulse),
        .rows_written     (rows_written)
    );

    // =========================================================================
    // R.1: Shortcut Bank SRAM (128 bit wide, 复用 sram_model)
    //   写: rdma_ctrl 一次性 batch 装载 [bias 段 + shortcut 段]
    //   读端口仲裁: bias_rf 优先 (cs 切换时它需要 4 拍 prefetch),
    //               否则 ofb_writer 的 shortcut read
    //   bias_ready=0 时 ofb_writer 已被 stall, 不会同拍发 sb_re
    // =========================================================================
    logic                  rdma_sb_we;
    logic [AW-1:0]         rdma_sb_waddr;
    logic [OFB_WIDTH-1:0]  rdma_sb_wdata;

    logic                  br_re;
    logic [AW-1:0]         br_raddr;
    logic [OFB_WIDTH-1:0]  br_rdata;

    // 读端口 mux: bias_rf 用 br_re, ofb_writer 用 ow_sb_re; 两路同拍 mutually exclusive
    // (bias_ready=0 → ow_sb_re=0; bias_ready=1 → br_re=0)
    logic                  sb_re_mux;
    logic [AW-1:0]         sb_raddr_mux;
    logic [OFB_WIDTH-1:0]  sb_rdata_shared;
    assign sb_re_mux    = br_re | ow_sb_re;
    assign sb_raddr_mux = br_re ? br_raddr : ow_sb_raddr;
    // sb_rdata 1 拍后到达, 两个消费者都拿同一份, 各自 latching 区分
    assign br_rdata     = sb_rdata_shared;
    assign ow_sb_rdata  = sb_rdata_shared;

    sram_model #(.DEPTH(SRAM_DEPTH), .DATA_WIDTH(OFB_WIDTH)) u_shortcut_bank (
        .clk(clk),
        .we(rdma_sb_we),
        .waddr(rdma_sb_waddr),
        .wdata(rdma_sb_wdata),
        .re(sb_re_mux),
        .raddr(sb_raddr_mux),
        .rdata(sb_rdata_shared)
    );

    // =========================================================================
    // R.1: bias_rf — 16 × INT32 RF, 跨 cs 自动 prefetch 4 拍
    // =========================================================================
    bias_rf #(
        .NUM_COL    (NUM_COL),
        .PSUM_WIDTH (PSUM_WIDTH),
        .SRAM_DATA_W(OFB_WIDTH),
        .SRAM_ADDR_W(AW)
    ) u_bias_rf (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (seq_start_core_pulse),
        .cs_cnt        (ow_cs_cnt),
        .cfg_bias_base (cfg_bias_base),
        .br_re         (br_re),
        .br_raddr      (br_raddr),
        .br_rdata      (br_rdata),
        .bias_vec      (bias_vec_wire),
        .bias_ready    (bias_ready_wire)
    );

    // =========================================================================
    // 9. done
    // =========================================================================
    assign done = ow_done;

    // =========================================================================
    // 10. DMA 引擎 (idma_ctrl / wdma_ctrl / odma_ctrl) + AXI DataMover + axi_m_mux
    //
    //   idma_ctrl + wdma_ctrl 共享 axi_dm 的 MM2S 通道, 经 mm2s_arb 串行仲裁.
    //   odma_ctrl 独占 axi_dm 的 S2MM 通道.
    //   axi_dm 对外暴露 2 个 AXI4 master (mm2s 读 / s2mm 写), 经 axi_m_mux 与
    //   DFE 一起聚合到外部 1 个 AXI4 master 接口.
    //
    //   M[0] = axi_dm.MM2S read  (AR/R), AW/W/B tied 0
    //   M[1] = axi_dm.S2MM write (AW/W/B), AR/R tied 0
    //   M[2] = unused (was old WDMA), all tied
    //   M[3] = DFE (read only), AW/W/B tied 0
    // =========================================================================
    localparam int N_MST = 2**AXI_M_WIDTH;

    // Master-side 打包信号 (给 axi_m_mux)
    logic [N_MST-1:0] [AXI_M_ID-1:0]      m_awid;
    logic [N_MST-1:0] [BUS_ADDR_W-1:0]    m_awaddr;
    logic [N_MST-1:0] [7:0]               m_awlen;
    logic [N_MST-1:0] [1:0]               m_awburst;
    logic [N_MST-1:0]                     m_awvalid;
    logic [N_MST-1:0]                     m_awready;
    logic [N_MST-1:0] [BUS_DATA_W-1:0]    m_wdata;
    logic [N_MST-1:0] [BUS_DATA_W/8-1:0]  m_wstrb;
    logic [N_MST-1:0]                     m_wlast;
    logic [N_MST-1:0]                     m_wvalid;
    logic [N_MST-1:0]                     m_wready;
    logic [N_MST-1:0] [AXI_M_ID-1:0]      m_bid;
    logic [N_MST-1:0] [1:0]               m_bresp;
    logic [N_MST-1:0]                     m_bvalid;
    logic [N_MST-1:0]                     m_bready;
    logic [N_MST-1:0] [AXI_M_ID-1:0]      m_arid;
    logic [N_MST-1:0] [BUS_ADDR_W-1:0]    m_araddr;
    logic [N_MST-1:0] [7:0]               m_arlen;
    logic [N_MST-1:0] [1:0]               m_arburst;
    logic [N_MST-1:0]                     m_arvalid;
    logic [N_MST-1:0]                     m_arready;
    logic [N_MST-1:0] [AXI_M_ID-1:0]      m_rid;
    logic [N_MST-1:0] [BUS_DATA_W-1:0]    m_rdata;
    logic [N_MST-1:0] [1:0]               m_rresp;
    logic [N_MST-1:0]                     m_rlast;
    logic [N_MST-1:0]                     m_rvalid;
    logic [N_MST-1:0]                     m_rready;

    // ---- M[0] 写通道 / M[1] 读通道 / M[2] 全部 / M[3] 写通道 tie 0 ----
    // M[0] = MM2S (read only)
    assign m_awid   [0] = '0;
    assign m_awaddr [0] = '0;
    assign m_awlen  [0] = '0;
    assign m_awburst[0] = 2'b01;
    assign m_awvalid[0] = 1'b0;
    assign m_wdata  [0] = '0;
    assign m_wstrb  [0] = '0;
    assign m_wlast  [0] = 1'b0;
    assign m_wvalid [0] = 1'b0;
    assign m_bready [0] = 1'b1;
    // M[1] = S2MM (write only)
    assign m_arid   [1] = '0;
    assign m_araddr [1] = '0;
    assign m_arlen  [1] = '0;
    assign m_arburst[1] = 2'b01;
    assign m_arvalid[1] = 1'b0;
    assign m_rready [1] = 1'b1;
    // M[2] unused (DataMover 把原 wdma 的 master 槽合并掉了)
    assign m_awid   [2] = '0;
    assign m_awaddr [2] = '0;
    assign m_awlen  [2] = '0;
    assign m_awburst[2] = 2'b01;
    assign m_awvalid[2] = 1'b0;
    assign m_wdata  [2] = '0;
    assign m_wstrb  [2] = '0;
    assign m_wlast  [2] = 1'b0;
    assign m_wvalid [2] = 1'b0;
    assign m_bready [2] = 1'b1;
    assign m_arid   [2] = '0;
    assign m_araddr [2] = '0;
    assign m_arlen  [2] = '0;
    assign m_arburst[2] = 2'b01;
    assign m_arvalid[2] = 1'b0;
    assign m_rready [2] = 1'b1;
    // M[3] = DFE (read only)
    assign m_awid   [3] = '0;
    assign m_awaddr [3] = '0;
    assign m_awlen  [3] = '0;
    assign m_awburst[3] = 2'b01;
    assign m_awvalid[3] = 1'b0;
    assign m_wdata  [3] = '0;
    assign m_wstrb  [3] = '0;
    assign m_wlast  [3] = 1'b0;
    assign m_wvalid [3] = 1'b0;
    assign m_bready [3] = 1'b1;

    dfe #(
        .ADDR_W(BUS_ADDR_W), .DATA_W(BUS_DATA_W),
        .M_ID(AXI_M_ID), .FIFO_W(256)
    ) u_dfe (
        .clk(clk), .rst_n(rst_n),
        .start          (cfg_start_dfe_pulse),
        .done           (dfe_done),
        .busy           (dfe_busy),
        .desc_list_base (cfg_desc_list_base),
        .desc_count     (cfg_desc_count),
        .M_ARID(m_arid[3]), .M_ARADDR(m_araddr[3]), .M_ARLEN(m_arlen[3]),
        .M_ARBURST(m_arburst[3]), .M_ARVALID(m_arvalid[3]), .M_ARREADY(m_arready[3]),
        .M_RID(m_rid[3]), .M_RDATA(m_rdata[3]), .M_RRESP(m_rresp[3]),
        .M_RLAST(m_rlast[3]), .M_RVALID(m_rvalid[3]), .M_RREADY(m_rready[3]),
        .fifo_wdata(desc_fifo_wdata), .fifo_we(desc_fifo_we), .fifo_full(desc_fifo_full)
    );

    // =========================================================================
    // axi_dm IP cmd/data/sts wires
    // =========================================================================
    logic                       arb_mm2s_cmd_tvalid, arb_mm2s_cmd_tready;
    logic [71:0]                arb_mm2s_cmd_tdata;
    logic                       arb_mm2s_data_tvalid, arb_mm2s_data_tready;
    logic [BUS_DATA_W-1:0]      arb_mm2s_data_tdata;
    logic [BUS_DATA_W/8-1:0]    arb_mm2s_data_tkeep;
    logic                       arb_mm2s_data_tlast;
    logic                       arb_mm2s_sts_tvalid, arb_mm2s_sts_tready;
    logic [7:0]                 arb_mm2s_sts_tdata;
    logic                       odma_s2mm_cmd_tvalid, odma_s2mm_cmd_tready;
    logic [71:0]                odma_s2mm_cmd_tdata;
    logic                       odma_s2mm_data_tvalid, odma_s2mm_data_tready;
    logic [BUS_DATA_W-1:0]      odma_s2mm_data_tdata;
    logic [BUS_DATA_W/8-1:0]    odma_s2mm_data_tkeep;
    logic                       odma_s2mm_data_tlast;
    logic                       odma_s2mm_sts_tvalid, odma_s2mm_sts_tready;
    logic [7:0]                 odma_s2mm_sts_tdata;

    // idma_ctrl <→ mm2s_arb 之间的 cmd/data/sts
    logic                       idma_cmd_tvalid, idma_cmd_tready;
    logic [71:0]                idma_cmd_tdata;
    logic                       idma_data_tvalid, idma_data_tready;
    logic [BUS_DATA_W-1:0]      idma_data_tdata;
    logic [BUS_DATA_W/8-1:0]    idma_data_tkeep;
    logic                       idma_data_tlast;
    logic                       idma_sts_tvalid, idma_sts_tready;
    logic [7:0]                 idma_sts_tdata;
    logic                       idma_err_w;
    // wdma_ctrl <→ mm2s_arb
    logic                       wdma_cmd_tvalid, wdma_cmd_tready;
    logic [71:0]                wdma_cmd_tdata;
    logic                       wdma_data_tvalid, wdma_data_tready;
    logic [BUS_DATA_W-1:0]      wdma_data_tdata;
    logic [BUS_DATA_W/8-1:0]    wdma_data_tkeep;
    logic                       wdma_data_tlast;
    logic                       wdma_sts_tvalid, wdma_sts_tready;
    logic [7:0]                 wdma_sts_tdata;
    logic                       wdma_err_w;
    logic                       odma_err_w;
    // R.1: rdma_ctrl <→ mm2s_arb
    logic                       rdma_cmd_tvalid, rdma_cmd_tready;
    logic [71:0]                rdma_cmd_tdata;
    logic                       rdma_data_tvalid, rdma_data_tready;
    logic [BUS_DATA_W-1:0]      rdma_data_tdata;
    logic [BUS_DATA_W/8-1:0]    rdma_data_tkeep;
    logic                       rdma_data_tlast;
    logic                       rdma_sts_tvalid, rdma_sts_tready;
    logic [7:0]                 rdma_sts_tdata;
    logic                       rdma_err_w;

    // =========================================================================
    // idma_ctrl
    // =========================================================================
    idma_ctrl #(
        .ADDR_W(BUS_ADDR_W), .DATA_W(BUS_DATA_W),
        .SRAM_ADDR_W(AW), .LEN_W(DMA_LEN_W)
    ) u_idma (
        .clk(clk), .rst_n(rst_n),
        .start(seq_start_idma_pulse), .done(idma_done), .busy(idma_busy), .err(idma_err_w),
        .src_base(eff_idma_src_base), .byte_len(eff_idma_byte_len),
        .cfg_h_in_total    (cfg_h_in_total),
        .cfg_ifb_strip_rows(cfg_ifb_strip_rows),
        .cfg_ifb_ky_step   (cfg_ifb_ky_step),
        .cfg_ifb_ring_words(cfg_ifb_ring_words),
        .rows_consumed     (rows_consumed),
        .rows_available    (rows_available),
        .mm2s_cmd_tvalid (idma_cmd_tvalid),  .mm2s_cmd_tready (idma_cmd_tready),  .mm2s_cmd_tdata  (idma_cmd_tdata),
        .mm2s_data_tvalid(idma_data_tvalid), .mm2s_data_tready(idma_data_tready),
        .mm2s_data_tdata (idma_data_tdata),  .mm2s_data_tkeep (idma_data_tkeep),  .mm2s_data_tlast (idma_data_tlast),
        .mm2s_sts_tvalid (idma_sts_tvalid),  .mm2s_sts_tready (idma_sts_tready),  .mm2s_sts_tdata  (idma_sts_tdata),
        .ifb_we(idma_ifb_we), .ifb_waddr(idma_ifb_waddr), .ifb_wdata(idma_ifb_wdata)
    );

    // =========================================================================
    // wdma_ctrl
    // =========================================================================
    wdma_ctrl #(
        .ADDR_W(BUS_ADDR_W), .DATA_W(BUS_DATA_W), .WB_DATA_W(WB_WIDTH),
        .SRAM_ADDR_W(AW), .LEN_W(DMA_LEN_W)
    ) u_wdma (
        .clk(clk), .rst_n(rst_n),
        .start(seq_start_wdma_pulse), .done(wdma_done), .busy(wdma_busy), .err(wdma_err_w),
        .src_base(dma_wdma_src_base), .byte_len(dma_wdma_byte_len),
        .mm2s_cmd_tvalid (wdma_cmd_tvalid),  .mm2s_cmd_tready (wdma_cmd_tready),  .mm2s_cmd_tdata  (wdma_cmd_tdata),
        .mm2s_data_tvalid(wdma_data_tvalid), .mm2s_data_tready(wdma_data_tready),
        .mm2s_data_tdata (wdma_data_tdata),  .mm2s_data_tkeep (wdma_data_tkeep),  .mm2s_data_tlast (wdma_data_tlast),
        .mm2s_sts_tvalid (wdma_sts_tvalid),  .mm2s_sts_tready (wdma_sts_tready),  .mm2s_sts_tdata  (wdma_sts_tdata),
        .wb_we(wdma_wb_we), .wb_waddr(wdma_wb_waddr), .wb_wdata(wdma_wb_wdata)
    );

    // =========================================================================
    // rdma_ctrl (R.1: bias + shortcut 加载到 Shortcut Bank)
    // =========================================================================
    rdma_ctrl #(
        .ADDR_W(BUS_ADDR_W), .DATA_W(BUS_DATA_W),
        .SRAM_ADDR_W(AW), .LEN_W(DMA_LEN_W)
    ) u_rdma (
        .clk(clk), .rst_n(rst_n),
        .start(seq_start_rdma_pulse), .done(rdma_done), .busy(rdma_busy), .err(rdma_err_w),
        .src_base(dma_rdma_src_base), .byte_len(dma_rdma_byte_len),
        .mm2s_cmd_tvalid (rdma_cmd_tvalid),  .mm2s_cmd_tready (rdma_cmd_tready),  .mm2s_cmd_tdata  (rdma_cmd_tdata),
        .mm2s_data_tvalid(rdma_data_tvalid), .mm2s_data_tready(rdma_data_tready),
        .mm2s_data_tdata (rdma_data_tdata),  .mm2s_data_tkeep (rdma_data_tkeep),  .mm2s_data_tlast (rdma_data_tlast),
        .mm2s_sts_tvalid (rdma_sts_tvalid),  .mm2s_sts_tready (rdma_sts_tready),  .mm2s_sts_tdata  (rdma_sts_tdata),
        .sb_we   (rdma_sb_we),
        .sb_waddr(rdma_sb_waddr),
        .sb_wdata(rdma_sb_wdata)
    );

    // =========================================================================
    // mm2s_arb (idma + wdma + rdma → 单条 axi_dm.MM2S)
    // =========================================================================
    mm2s_arb #(.DATA_W(BUS_DATA_W)) u_mm2s_arb (
        .clk(clk), .rst_n(rst_n),
        .idma_cmd_tvalid (idma_cmd_tvalid),  .idma_cmd_tready (idma_cmd_tready),  .idma_cmd_tdata  (idma_cmd_tdata),
        .idma_data_tvalid(idma_data_tvalid), .idma_data_tready(idma_data_tready),
        .idma_data_tdata (idma_data_tdata),  .idma_data_tkeep (idma_data_tkeep),  .idma_data_tlast (idma_data_tlast),
        .idma_sts_tvalid (idma_sts_tvalid),  .idma_sts_tready (idma_sts_tready),  .idma_sts_tdata  (idma_sts_tdata),
        .wdma_cmd_tvalid (wdma_cmd_tvalid),  .wdma_cmd_tready (wdma_cmd_tready),  .wdma_cmd_tdata  (wdma_cmd_tdata),
        .wdma_data_tvalid(wdma_data_tvalid), .wdma_data_tready(wdma_data_tready),
        .wdma_data_tdata (wdma_data_tdata),  .wdma_data_tkeep (wdma_data_tkeep),  .wdma_data_tlast (wdma_data_tlast),
        .wdma_sts_tvalid (wdma_sts_tvalid),  .wdma_sts_tready (wdma_sts_tready),  .wdma_sts_tdata  (wdma_sts_tdata),
        .rdma_cmd_tvalid (rdma_cmd_tvalid),  .rdma_cmd_tready (rdma_cmd_tready),  .rdma_cmd_tdata  (rdma_cmd_tdata),
        .rdma_data_tvalid(rdma_data_tvalid), .rdma_data_tready(rdma_data_tready),
        .rdma_data_tdata (rdma_data_tdata),  .rdma_data_tkeep (rdma_data_tkeep),  .rdma_data_tlast (rdma_data_tlast),
        .rdma_sts_tvalid (rdma_sts_tvalid),  .rdma_sts_tready (rdma_sts_tready),  .rdma_sts_tdata  (rdma_sts_tdata),
        .mm2s_cmd_tvalid (arb_mm2s_cmd_tvalid),  .mm2s_cmd_tready (arb_mm2s_cmd_tready),  .mm2s_cmd_tdata  (arb_mm2s_cmd_tdata),
        .mm2s_data_tvalid(arb_mm2s_data_tvalid), .mm2s_data_tready(arb_mm2s_data_tready),
        .mm2s_data_tdata (arb_mm2s_data_tdata),  .mm2s_data_tkeep (arb_mm2s_data_tkeep),  .mm2s_data_tlast (arb_mm2s_data_tlast),
        .mm2s_sts_tvalid (arb_mm2s_sts_tvalid),  .mm2s_sts_tready (arb_mm2s_sts_tready),  .mm2s_sts_tdata  (arb_mm2s_sts_tdata)
    );

    // =========================================================================
    // odma_ctrl
    // =========================================================================
    odma_ctrl #(
        .ADDR_W(BUS_ADDR_W), .DATA_W(BUS_DATA_W),
        .SRAM_ADDR_W(AW), .LEN_W(DMA_LEN_W)
    ) u_odma (
        .clk(clk), .rst_n(rst_n),
        .start(seq_start_odma_pulse), .done(odma_done), .busy(odma_busy), .err(odma_err_w),
        .dst_base(eff_odma_dst_base), .byte_len(eff_odma_byte_len),
        .cfg_h_out_total       (cfg_h_out),
        .cfg_w_out             (cfg_w_out),
        .cfg_cout_slices       (cfg_cout_slices),
        .cfg_ofb_row_words     (cfg_ofb_row_words),
        .cfg_ddr_ofm_row_stride({{(BUS_ADDR_W-ADDR_W){1'b0}}, cfg_ddr_ofm_row_stride}),
        .cfg_ofb_ring_words    (cfg_ofb_ring_words),
        .row_done_pulse        (row_done_pulse),
        .rows_drained          (rows_drained),
        .s2mm_cmd_tvalid (odma_s2mm_cmd_tvalid),  .s2mm_cmd_tready (odma_s2mm_cmd_tready),  .s2mm_cmd_tdata  (odma_s2mm_cmd_tdata),
        .s2mm_data_tvalid(odma_s2mm_data_tvalid), .s2mm_data_tready(odma_s2mm_data_tready),
        .s2mm_data_tdata (odma_s2mm_data_tdata),  .s2mm_data_tkeep (odma_s2mm_data_tkeep),  .s2mm_data_tlast (odma_s2mm_data_tlast),
        .s2mm_sts_tvalid (odma_s2mm_sts_tvalid),  .s2mm_sts_tready (odma_s2mm_sts_tready),  .s2mm_sts_tdata  (odma_s2mm_sts_tdata),
        .ofb_re(odma_ofb_re), .ofb_raddr(odma_ofb_raddr), .ofb_rdata(odma_ofb_rdata)
    );

    // =========================================================================
    // axi_dm IP (Xilinx AXI DataMover): MM2S + S2MM
    //   MM2S 内部 read master 接 m_axi[0]; S2MM 内部 write master 接 m_axi[1].
    //   axi_dm 端口名照 Vivado 生成的 axi_dm.veo (BUS_DATA_W=128 时 tkeep=16-bit).
    // =========================================================================
    axi_dm u_axi_dm (
        // ---- 时钟 / 复位 (mm2s + s2mm + 各自 cmdsts 全部用同一时钟) ----
        .m_axi_mm2s_aclk            (clk),
        .m_axi_mm2s_aresetn         (rst_n),
        .mm2s_err                   (),
        .m_axis_mm2s_cmdsts_aclk    (clk),
        .m_axis_mm2s_cmdsts_aresetn (rst_n),
        // ---- MM2S cmd / data / sts (来自 mm2s_arb) ----
        .s_axis_mm2s_cmd_tvalid     (arb_mm2s_cmd_tvalid),
        .s_axis_mm2s_cmd_tready     (arb_mm2s_cmd_tready),
        .s_axis_mm2s_cmd_tdata      (arb_mm2s_cmd_tdata),
        .m_axis_mm2s_sts_tvalid     (arb_mm2s_sts_tvalid),
        .m_axis_mm2s_sts_tready     (arb_mm2s_sts_tready),
        .m_axis_mm2s_sts_tdata      (arb_mm2s_sts_tdata),
        .m_axis_mm2s_sts_tkeep      (),
        .m_axis_mm2s_sts_tlast      (),
        // ---- MM2S 内部 AXI4 read master → m_axi[0] ----
        .m_axi_mm2s_arid            (m_arid   [0]),
        .m_axi_mm2s_araddr          (m_araddr [0]),
        .m_axi_mm2s_arlen           (m_arlen  [0]),
        .m_axi_mm2s_arsize          (),
        .m_axi_mm2s_arburst         (m_arburst[0]),
        .m_axi_mm2s_arprot          (),
        .m_axi_mm2s_arcache         (),
        .m_axi_mm2s_aruser          (),
        .m_axi_mm2s_arvalid         (m_arvalid[0]),
        .m_axi_mm2s_arready         (m_arready[0]),
        .m_axi_mm2s_rdata           (m_rdata  [0]),
        .m_axi_mm2s_rresp           (m_rresp  [0]),
        .m_axi_mm2s_rlast           (m_rlast  [0]),
        .m_axi_mm2s_rvalid          (m_rvalid [0]),
        .m_axi_mm2s_rready          (m_rready [0]),
        // ---- MM2S 数据流出 → mm2s_arb ----
        .m_axis_mm2s_tdata          (arb_mm2s_data_tdata),
        .m_axis_mm2s_tkeep          (arb_mm2s_data_tkeep),
        .m_axis_mm2s_tlast          (arb_mm2s_data_tlast),
        .m_axis_mm2s_tvalid         (arb_mm2s_data_tvalid),
        .m_axis_mm2s_tready         (arb_mm2s_data_tready),
        // ---- S2MM 时钟 / 复位 ----
        .m_axi_s2mm_aclk            (clk),
        .m_axi_s2mm_aresetn         (rst_n),
        .s2mm_err                   (),
        .m_axis_s2mm_cmdsts_awclk   (clk),
        .m_axis_s2mm_cmdsts_aresetn (rst_n),
        // ---- S2MM cmd / data / sts (来自 odma_ctrl) ----
        .s_axis_s2mm_cmd_tvalid     (odma_s2mm_cmd_tvalid),
        .s_axis_s2mm_cmd_tready     (odma_s2mm_cmd_tready),
        .s_axis_s2mm_cmd_tdata      (odma_s2mm_cmd_tdata),
        .m_axis_s2mm_sts_tvalid     (odma_s2mm_sts_tvalid),
        .m_axis_s2mm_sts_tready     (odma_s2mm_sts_tready),
        .m_axis_s2mm_sts_tdata      (odma_s2mm_sts_tdata),
        .m_axis_s2mm_sts_tkeep      (),
        .m_axis_s2mm_sts_tlast      (),
        // ---- S2MM 内部 AXI4 write master → m_axi[1] ----
        .m_axi_s2mm_awid            (m_awid   [1]),
        .m_axi_s2mm_awaddr          (m_awaddr [1]),
        .m_axi_s2mm_awlen           (m_awlen  [1]),
        .m_axi_s2mm_awsize          (),
        .m_axi_s2mm_awburst         (m_awburst[1]),
        .m_axi_s2mm_awprot          (),
        .m_axi_s2mm_awcache         (),
        .m_axi_s2mm_awuser          (),
        .m_axi_s2mm_awvalid         (m_awvalid[1]),
        .m_axi_s2mm_awready         (m_awready[1]),
        .m_axi_s2mm_wdata           (m_wdata  [1]),
        .m_axi_s2mm_wstrb           (m_wstrb  [1]),
        .m_axi_s2mm_wlast           (m_wlast  [1]),
        .m_axi_s2mm_wvalid          (m_wvalid [1]),
        .m_axi_s2mm_wready          (m_wready [1]),
        .m_axi_s2mm_bresp           (m_bresp  [1]),
        .m_axi_s2mm_bvalid          (m_bvalid [1]),
        .m_axi_s2mm_bready          (m_bready [1]),
        // ---- S2MM 数据流入 (来自 odma_ctrl) ----
        .s_axis_s2mm_tdata          (odma_s2mm_data_tdata),
        .s_axis_s2mm_tkeep          (odma_s2mm_data_tkeep),
        .s_axis_s2mm_tlast          (odma_s2mm_data_tlast),
        .s_axis_s2mm_tvalid         (odma_s2mm_data_tvalid),
        .s_axis_s2mm_tready         (odma_s2mm_data_tready)
    );

    // N→1 AXI M 合并
    axi_m_mux #(
        .M_WIDTH(AXI_M_WIDTH), .M_ID(AXI_M_ID),
        .ADDR_W(BUS_ADDR_W), .DATA_W(BUS_DATA_W)
    ) u_axi_mux (
        .clk(clk), .rstn(rst_n),
        .M_AWID(m_awid), .M_AWADDR(m_awaddr), .M_AWLEN(m_awlen), .M_AWBURST(m_awburst),
        .M_AWVALID(m_awvalid), .M_AWREADY(m_awready),
        .M_WDATA(m_wdata), .M_WSTRB(m_wstrb), .M_WLAST(m_wlast),
        .M_WVALID(m_wvalid), .M_WREADY(m_wready),
        .M_BID(m_bid), .M_BRESP(m_bresp), .M_BVALID(m_bvalid), .M_BREADY(m_bready),
        .M_ARID(m_arid), .M_ARADDR(m_araddr), .M_ARLEN(m_arlen), .M_ARBURST(m_arburst),
        .M_ARVALID(m_arvalid), .M_ARREADY(m_arready),
        .M_RID(m_rid), .M_RDATA(m_rdata), .M_RRESP(m_rresp), .M_RLAST(m_rlast),
        .M_RVALID(m_rvalid), .M_RREADY(m_rready),
        .B_AWID(bus_awid), .B_AWADDR(bus_awaddr), .B_AWLEN(bus_awlen), .B_AWBURST(bus_awburst),
        .B_AWVALID(bus_awvalid), .B_AWREADY(bus_awready),
        .B_WDATA(bus_wdata), .B_WSTRB(bus_wstrb), .B_WLAST(bus_wlast),
        .B_WVALID(bus_wvalid), .B_WREADY(bus_wready),
        .B_BID(bus_bid), .B_BRESP(bus_bresp), .B_BVALID(bus_bvalid), .B_BREADY(bus_bready),
        .B_ARID(bus_arid), .B_ARADDR(bus_araddr), .B_ARLEN(bus_arlen), .B_ARBURST(bus_arburst),
        .B_ARVALID(bus_arvalid), .B_ARREADY(bus_arready),
        .B_RID(bus_rid), .B_RDATA(bus_rdata), .B_RRESP(bus_rresp), .B_RLAST(bus_rlast),
        .B_RVALID(bus_rvalid), .B_RREADY(bus_rready)
    );

endmodule
