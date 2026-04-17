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
// TB 通过层次引用 u_core_top.u_cfg.r_* 写入配置，随后拉 start 一拍启动；
// done = ofb_writer.done（OFB 最后 1 拍写完后拉高）。
// =============================================================================

module core_top #(
    parameter int NUM_COL     = 16,
    parameter int NUM_PE      = 16,
    parameter int DATA_WIDTH  = 8,
    parameter int PSUM_WIDTH  = 32,
    parameter int WRF_DEPTH   = 32,
    parameter int ARF_DEPTH   = 32,
    parameter int PARF_DEPTH  = 32,
    parameter int SRAM_DEPTH  = 8192
)(
    input  logic                                clk,
    input  logic                                rst_n,

    input  logic                                start,

    // TB 外部 SRAM 写口 (加载 IFB / WB)
    input  logic                                ifb_we_ext,
    input  logic [$clog2(SRAM_DEPTH)-1:0]       ifb_waddr_ext,
    input  logic [NUM_PE*DATA_WIDTH-1:0]        ifb_wdata_ext,

    input  logic                                wb_we_ext,
    input  logic [$clog2(SRAM_DEPTH)-1:0]       wb_waddr_ext,
    input  logic [NUM_COL*NUM_PE*DATA_WIDTH-1:0]wb_wdata_ext,

    // TB 外部 OFB 读口 (提取结果)
    input  logic                                ofb_re_ext,
    input  logic [$clog2(SRAM_DEPTH)-1:0]       ofb_raddr_ext,
    output logic [NUM_COL*DATA_WIDTH-1:0]       ofb_rdata_ext,

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
    // 1. 共享配置寄存器
    //    w_out / wb_cin_step / ofb_cout_step 目前无下游消费（padding / chunked
    //    sliding 等 phase-2 特性会用到），先空端口连接避免触发 unused-wire lint。
    // =========================================================================
    logic [15:0]       cfg_h_out, cfg_w_in;
    logic [3:0]        cfg_k;
    logic [2:0]        cfg_stride;
    logic [5:0]        cfg_cin_slices, cfg_cout_slices, cfg_tile_w, cfg_last_valid_w;
    logic [7:0]        cfg_num_tiles;
    logic [9:0]        cfg_total_wrf, cfg_kk;
    logic              cfg_wrf_packed;
    logic [2:0]        cfg_rounds_per_cins;
    logic [5:0]        cfg_round_len_last;
    logic [ADDR_W-1:0] cfg_ifb_base, cfg_wb_base, cfg_ofb_base;
    logic [ADDR_W-1:0] cfg_ifb_cin_step, cfg_ifb_row_step;
    logic [ADDR_W-1:0] cfg_wb_cout_step;
    logic [ADDR_W-1:0] cfg_tile_in_step;
    logic [4:0]        cfg_sdp_shift;
    logic              cfg_sdp_relu_en;

    cfg_regs #(.ADDR_W(ADDR_W)) u_cfg (
        .clk               (clk),
        .rst_n             (rst_n),
        .h_out             (cfg_h_out),
        .w_out             (/* unused */),
        .w_in              (cfg_w_in),
        .k                 (cfg_k),
        .stride            (cfg_stride),
        .cin_slices        (cfg_cin_slices),
        .cout_slices       (cfg_cout_slices),
        .tile_w            (cfg_tile_w),
        .num_tiles         (cfg_num_tiles),
        .last_valid_w      (cfg_last_valid_w),
        .total_wrf         (cfg_total_wrf),
        .wrf_packed        (cfg_wrf_packed),
        .kk                (cfg_kk),
        .rounds_per_cins   (cfg_rounds_per_cins),
        .round_len_last    (cfg_round_len_last),
        .ifb_base          (cfg_ifb_base),
        .wb_base           (cfg_wb_base),
        .ofb_base          (cfg_ofb_base),
        .ifb_cin_step      (cfg_ifb_cin_step),
        .ifb_row_step      (cfg_ifb_row_step),
        .wb_cin_step       (/* unused (reserved for chunked sliding) */),
        .wb_cout_step      (cfg_wb_cout_step),
        .ofb_cout_step     (/* unused (OFB 物理连续，地址递增即可) */),
        .tile_in_step      (cfg_tile_in_step),
        .sdp_shift         (cfg_sdp_shift),
        .sdp_relu_en       (cfg_sdp_relu_en)
    );

    // =========================================================================
    // 2. 三块 SRAM (IFB / WB / OFB)
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

    sram_model #(.DEPTH(SRAM_DEPTH), .DATA_WIDTH(IFB_WIDTH)) u_ifb (
        .clk(clk), .we(ifb_we_ext), .waddr(ifb_waddr_ext), .wdata(ifb_wdata_ext),
        .re(ifb_re), .raddr(ifb_raddr), .rdata(ifb_rdata)
    );

    sram_model #(.DEPTH(SRAM_DEPTH), .DATA_WIDTH(WB_WIDTH)) u_wb (
        .clk(clk), .we(wb_we_ext), .waddr(wb_waddr_ext), .wdata(wb_wdata_ext),
        .re(wb_re), .raddr(wb_raddr), .rdata(wb_rdata)
    );

    sram_model #(.DEPTH(SRAM_DEPTH), .DATA_WIDTH(OFB_WIDTH)) u_ofb (
        .clk(clk), .we(ofb_we), .waddr(ofb_waddr), .wdata(ofb_wdata),
        .re(ofb_re_ext), .raddr(ofb_raddr_ext), .rdata(ofb_rdata_ext)
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

    line_buffer #(
        .NUM_PE    (NUM_PE),
        .DATA_WIDTH(DATA_WIDTH),
        .ARF_DEPTH (ARF_DEPTH),
        .SRAM_DEPTH(SRAM_DEPTH),
        .ADDR_W    (ADDR_W)
    ) u_line_buffer (
        .clk              (clk),
        .rst_n            (rst_n),
        .start            (start),
        .done             (lb_done),
        .cfg_h_out        (cfg_h_out),
        .cfg_w_in         (cfg_w_in),
        .cfg_k            (cfg_k),
        .cfg_stride       (cfg_stride),
        .cfg_cin_slices   (cfg_cin_slices),
        .cfg_cout_slices  (cfg_cout_slices),
        .cfg_tile_w       (cfg_tile_w),
        .cfg_num_tiles    (cfg_num_tiles),
        .cfg_last_valid_w (cfg_last_valid_w),
        .cfg_ifb_base     (cfg_ifb_base),
        .cfg_ifb_cin_step (cfg_ifb_cin_step),
        .cfg_ifb_row_step (cfg_ifb_row_step),
        .cfg_tile_in_step (cfg_tile_in_step),
        .ifb_re           (ifb_re),
        .ifb_raddr        (ifb_raddr),
        .ifb_rdata        (ifb_rdata),
        .act_valid        (act_valid),
        .act_vec          (act_vec),
        .act_ready        (act_ready)
    );

    // =========================================================================
    // 5. wgt_buffer
    // =========================================================================
    logic wb_done;

    wgt_buffer #(
        .NUM_COL   (NUM_COL),
        .NUM_PE    (NUM_PE),
        .DATA_WIDTH(DATA_WIDTH),
        .WRF_DEPTH (WRF_DEPTH),
        .SRAM_DEPTH(SRAM_DEPTH),
        .ADDR_W    (ADDR_W)
    ) u_wgt_buffer (
        .clk              (clk),
        .rst_n            (rst_n),
        .start            (start),
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
        .cfg_wrf_packed      (cfg_wrf_packed),
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
        .psum_in_ready  (psum_in_ready)
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
        .acc_out_ready    (acc_out_ready)
    );

    // =========================================================================
    // 8. ofb_writer (含 SDP)
    // =========================================================================
    logic ow_done;

    ofb_writer #(
        .NUM_COL   (NUM_COL),
        .DATA_WIDTH(DATA_WIDTH),
        .PSUM_WIDTH(PSUM_WIDTH),
        .SRAM_DEPTH(SRAM_DEPTH),
        .ADDR_W    (ADDR_W)
    ) u_ofb_writer (
        .clk              (clk),
        .rst_n            (rst_n),
        .start            (start),
        .done             (ow_done),
        .cfg_h_out        (cfg_h_out),
        .cfg_tile_w       (cfg_tile_w),
        .cfg_last_valid_w (cfg_last_valid_w),
        .cfg_num_tiles    (cfg_num_tiles),
        .cfg_cout_slices  (cfg_cout_slices),
        .cfg_ofb_base     (cfg_ofb_base),
        .cfg_sdp_shift    (cfg_sdp_shift),
        .cfg_sdp_relu_en  (cfg_sdp_relu_en),
        .acc_out_valid    (acc_out_valid),
        .acc_out_vec      (acc_out_vec),
        .acc_out_ready    (acc_out_ready),
        .ofb_we           (ofb_we),
        .ofb_waddr        (ofb_waddr),
        .ofb_wdata        (ofb_wdata)
    );

    // =========================================================================
    // 9. done
    // =========================================================================
    assign done = ow_done;

endmodule
