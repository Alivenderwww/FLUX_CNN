`timescale 1ns/1ps
`include "flux_cnn_params.svh"

// =============================================================================
// cfg_regs.sv  --  Shared Configuration + DMA Descriptor Register Bank
//
// 写口分两条:
//   1. csr_w_*  (AXI-Lite, host 写) -- 只接受 boot 寄存器:
//                 CTRL (pulse), DESC_LIST_BASE, DESC_COUNT, DMA_MODE
//   2. seq_w_*  (sequencer CFG_WRITE descriptor) -- 所有 layer cfg 字段
//                 (H_OUT, K, IDMA_SRC_BASE, ... 共 ~50 个)
//
// host 流程: 写 DESC_LIST_BASE/DESC_COUNT → 写 CTRL[4]=1 拉 desc → 写 CTRL[5]=1
//   sequencer 自己消费 CFG_WRITE descriptors 把 cfg 寄存器写齐, 再跑 CONV
//   strips, 最后 END 触发 layer_done. host 完全不碰 layer cfg.
//
// 读口 (reg_r_addr/reg_r_data) 仍然 AXI-Lite, host poll STATUS 用.
// Bridge 在 axi_lite_csr.sv 里.
//
// Register map（4 字节对齐）：
//   0x000  CTRL              [4]=start_dfe    [5]=start_layer
//   0x004  STATUS  (RO)      [0]=core_done(sticky, 写 start_layer 自清) [1]=core_busy
//                             [2]=idma_busy   [3]=wdma_busy [4]=odma_busy
//                             [5]=idma_done   [6]=wdma_done [7]=odma_done
//                             [8]=dfe_busy    [9]=dfe_done
//                             [10]=layer_busy [11]=layer_done(电平, 仅 S_END 那拍高)
//   0x100  H_OUT             [15:0]
//   0x104  W_OUT             [15:0]
//   0x108  W_IN              [15:0]
//   0x10C  K                 [3:0]
//   0x110  STRIDE            [2:0]
//   0x114  CIN_SLICES        [5:0]
//   0x118  COUT_SLICES       [5:0]
//   0x11C  TILE_W            [5:0]
//   0x120  NUM_TILES         [7:0]
//   0x124  LAST_VALID_W      [5:0]
//   0x128  TOTAL_WRF         [9:0]
//   0x12C  (reserved, 原 WRF_PACKED 废弃)
//   0x130  KK                [9:0]
//   0x134  ROUNDS_PER_CINS   [2:0]
//   0x138  ROUND_LEN_LAST    [5:0]
//   0x13C  IFB_BASE          [19:0]
//   0x140  WB_BASE           [19:0]
//   0x144  OFB_BASE          [19:0]
//   0x14C  IFB_ROW_STEP      [19:0]            (NHWC: stride × W_IN × cin_slices)
//   0x154  WB_COUT_STEP      [19:0]
//   0x15C  TILE_IN_STEP      [19:0]            (NHWC: TILE_W × stride × cin_slices)
//   0x160  SDP_SHIFT         [5:0]
//   0x164  SDP_RELU_EN       [0]
//   0x168  H_IN_TOTAL        [15:0]            整图输入高度
//   0x16C  IFB_STRIP_ROWS    [7:0]             IFB ring 容纳输入行数
//   0x170  OFB_STRIP_ROWS    [7:0]             OFB ring 容纳输出行数
//   0x174  DDR_IFM_ROW_STRIDE[19:0]            DDR 相邻输入行跨度（字节）
//   0x178  DDR_OFM_ROW_STRIDE[19:0]            DDR 相邻输出行跨度（字节）
//   0x17C  DMA_MODE          [0]=idma_stream [1]=odma_stream
//   0x180  DESC_LIST_BASE    [31:0]
//   0x184  DESC_COUNT        [15:0]
//   0x188  SDP_MULT          [31:0]  signed
//   0x18C  SDP_ZP_OUT        [8:0]   signed
//   0x190  SDP_CLIP_MIN      [8:0]   signed
//   0x194  SDP_CLIP_MAX      [8:0]   signed
//   0x198  SDP_ROUND_EN      [0]
//   0x1A0  IFB_RING_WORDS    [19:0]            (ifb_strip_rows × W_IN × cin_slices, word 单位)
//   0x1A4  OFB_ROW_WORDS     [19:0]            (W_OUT × cout_slices, word 单位)
//   0x1A8  OFB_RING_WORDS    [19:0]            (ofb_strip_rows × W_OUT × cout_slices)
//   0x1AC  IFB_ISS_STEP      [19:0]            (stride × cin_slices, IFB 跨像素 word 步长)
//   0x1B0  IFB_KY_STEP       [19:0]            (W_IN × cin_slices, IFB 跨 ky 行 word 步长)
//   0x1B4  TILE_PIX_STEP     [15:0]            (TILE_W × stride, 像素域 tile 步长 for pad 判定)
//   0x1B8  ARF_REUSE_EN      [0]               (1: kx sliding-window reuse；仅 stride==1 && K>1)
//   0x1BC  RESIDUAL_EN       [0]               (1: SDP enable shortcut add path)
//   0x1C0  SHORTCUT_MULT     [15:0] signed     (shortcut scale align: shortcut × mult >> shift)
//   0x1C4  SHORTCUT_SHIFT    [4:0]             (右移量, 0..31)
//   0x1C8  BIAS_BASE         [12:0]            (bias 在 Shortcut Bank 中的起始 word-index, 0 表示 SRAM 头)
//   0x200  IDMA_SRC_BASE     [31:0]
//   0x204  IDMA_BYTE_LEN     [23:0]             (保留，WDMA 用；IDMA 实际 len 由 descriptor 覆盖)
//   0x210  WDMA_SRC_BASE     [31:0]
//   0x214  WDMA_BYTE_LEN     [23:0]
//   0x220  ODMA_DST_BASE     [31:0]
//   0x224  ODMA_BYTE_LEN     [23:0]             (保留；ODMA 实际 len 由 descriptor 覆盖)
//   0x230  RDMA_SRC_BASE     [31:0]             (一条 batch cmd 拉 [bias][shortcut] 进 SB)
//   0x234  RDMA_BYTE_LEN     [23:0]
//
// 复位：按 §6，数据路径 cfg / DMA 寄存器不加复位。控制路径 DMA_MODE 有复位。
// =============================================================================

module cfg_regs #(
    parameter int ADDR_W   = 12,
    parameter int DATA_W   = 32,
    parameter int CORE_ADDR_W = 20   // 内部 SRAM 地址位宽（同 core_top）
)(
    input  logic                     clk,
    input  logic                     rst_n,

    // ---- AXI-Lite 写口: host → boot regs (CTRL/DESC_LIST_BASE/DESC_COUNT/DMA_MODE) ----
    input  logic                     csr_w_en,
    input  logic [ADDR_W-1:0]        csr_w_addr,
    input  logic [DATA_W-1:0]        csr_w_data,
    input  logic [DATA_W/8-1:0]      csr_w_strb,     // 暂时不解析，全 32-bit 写

    // ---- 读口 (host poll STATUS / debug) ----
    input  logic [ADDR_W-1:0]        reg_r_addr,
    output logic [DATA_W-1:0]        reg_r_data,

    // ---- Sequencer CFG_WRITE 写口: 所有 layer cfg 字段 ----
    input  logic                     seq_w_en,
    input  logic [ADDR_W-1:0]        seq_w_addr,
    input  logic [DATA_W-1:0]        seq_w_data,

    // ---- Status 输入（来自 core 内部）----
    input  logic                     idma_busy,
    input  logic                     wdma_busy,
    input  logic                     odma_busy,
    input  logic                     idma_done,
    input  logic                     wdma_done,
    input  logic                     odma_done,
    input  logic                     dfe_busy,
    input  logic                     dfe_done,
    input  logic                     layer_busy,
    input  logic                     layer_done,

    // ---- VD100 dbg 输入 (RO 寄存器 0x008 expose) ----
    input  logic [3:0]               seq_state,
    input  logic [7:0]               fifo_count,
    input  logic [3:0]               master_arvalid,
    input  logic [3:0]               master_rvalid,

    // ---- CTRL 输出：host 触发 DFE 拉 descriptor 和 Sequencer 启动 ----
    output logic                     start_dfe_pulse,    // CTRL[4] 写 1
    output logic                     start_layer_pulse,  // CTRL[5] 写 1
    // CTRL[7] 写 1 → stretched 软 reset (持续 16 拍低) 给所有 sub-module
    // 用途: 任何 dispatcher / sequencer stuck 时, host 写 CTRL=0x80 拉回 IDLE.
    // cfg_regs 自身不响应这个 reset, cfg 寄存器值在软 reset 后保留.
    output logic                     soft_reset_n,       // stretched, low-active

    // ---- 整层完成 sticky 标志 (用于顶层 done 端口 / GIC IRQ / STATUS[0]) ----
    //   layer_done 一上升 → set, start_layer_pulse 自清.
    //   host 不需要 W1C, 写下一层 start 那拍就清掉了, 中断/poll 都简洁.
    output logic                     core_done_sticky,

    // ---- 配置输出 ----
    output logic [15:0]              h_out,
    output logic [15:0]              w_out,
    output logic [15:0]              w_in,
    output logic [3:0]               k,
    output logic [3:0]               ky,
    output logic [2:0]               stride,
    output logic [2:0]               stride_h,           // Round I: H 维 stride (跟 W stride 分离, 默认 = stride)
    output logic [5:0]               cin_slices,
    output logic [5:0]               cout_slices,
    output logic [5:0]               tile_w,
    output logic [7:0]               num_tiles,
    output logic [5:0]               last_valid_w,
    output logic [9:0]               total_wrf,
    output logic [9:0]               kk,
    output logic [2:0]               rounds_per_cins,
    output logic [5:0]               round_len_last,
    output logic [CORE_ADDR_W-1:0]   ifb_base,
    output logic [CORE_ADDR_W-1:0]   wb_base,
    output logic [CORE_ADDR_W-1:0]   ofb_base,
    output logic [CORE_ADDR_W-1:0]   ifb_row_step,
    output logic [CORE_ADDR_W-1:0]   wb_cout_step,
    output logic [CORE_ADDR_W-1:0]   tile_in_step,
    output logic [5:0]               sdp_shift,
    output logic                     sdp_relu_en,

    // ---- Streaming / ring 配置输出 ----
    output logic [15:0]              h_in_total,
    output logic [15:0]              ifb_strip_rows,  // VD100 fix 2026-05-15: 8→16 bit, 任意 H 都 fit
    output logic [15:0]              ofb_strip_rows,  // VD100 fix 2026-05-15: 6→8→16 bit, 跟 h_in_total 对齐
    output logic [CORE_ADDR_W-1:0]   ddr_ifm_row_stride,
    output logic [CORE_ADDR_W-1:0]   ddr_ofm_row_stride,
    // J-2: idma_streaming / odma_streaming outputs removed (hardware 恒 streaming).
    // ADDR_DMA_MODE register 保留但硬件不读, 软件可写入无副作用。

    // ---- DMA 描述符输出（layer-level base；per-strip offset/len 由 Sequencer 叠加）----
    output logic [31:0]              idma_src_base,
    output logic [23:0]              idma_byte_len,
    output logic [31:0]              wdma_src_base,
    output logic [23:0]              wdma_byte_len,
    output logic [31:0]              odma_dst_base,
    output logic [23:0]              odma_byte_len,

    // ---- Descriptor list + new SDP fields ----
    output logic [31:0]              desc_list_base,
    output logic [15:0]              desc_count,
    output logic signed [31:0]       sdp_mult,
    output logic signed [8:0]        sdp_zp_out,
    output logic signed [8:0]        sdp_clip_min,
    output logic signed [8:0]        sdp_clip_max,
    output logic                     sdp_round_en,

    // ---- Phase D: NHWC + 方式 1 预算值 ----
    output logic [CORE_ADDR_W-1:0]   ifb_ring_words,
    output logic [CORE_ADDR_W-1:0]   ofb_row_words,
    output logic [CORE_ADDR_W-1:0]   ofb_ring_words,
    output logic [CORE_ADDR_W-1:0]   ifb_iss_step,
    output logic [CORE_ADDR_W-1:0]   ifb_ky_step,
    output logic [15:0]              tile_pix_step,
    output logic                     arf_reuse_en,

    // ---- Residual / shortcut / bias (R.1) ----
    output logic                     residual_en,
    output logic signed [15:0]       shortcut_mult,
    output logic [4:0]               shortcut_shift,
    output logic [12:0]              bias_base,           // word-index in Shortcut Bank

    // ---- M2 Multi-core ----
    output logic                     skip_idma,           // 1 = consumer 等远端核 push 进 IFB, 不启本地 IDMA
    output logic [31:0]              rdma_src_base,
    output logic [23:0]              rdma_byte_len,

    // ---- Mesh: ODMA OFM 包目的节点 / opcode (host csr_w 配置) ----
    output logic [7:0]               ofm_tdest,           // [7:4]=dst_y, [3:0]=dst_x
    output logic [3:0]               ofm_opcode,

    // ---- SMC + NUMA: IDMA SG cmd list (Phase 7) ----
    //   driver 编译期把 SG cmd list 写到 mem 某段, ConvCore.idma_sg_dispatcher
    //   按 cmd_list_ptr 拉 cmd, 顺序跑 burst. 由 seq_w (CFG_WRITE descriptor) 写,
    //   每层独立配置 (不同 layer cmd list 起点 / 总数 不同).
    output logic [31:0]              idma_cmd_list_ptr,   // 全局地址 byte
    output logic [15:0]              idma_cmd_count,
    output logic [7:0]               idma_cmds_per_row,   // W slice 跨 mem 时 > 1

    // ---- SMC + NUMA: ODMA SG cmd list (Phase 7) ----
    output logic [31:0]              odma_cmd_list_ptr,
    output logic [15:0]              odma_cmd_count,
    output logic [7:0]               odma_cmds_per_row    // W slice / cout slice 跨 mem 时 > 1
);

    // =========================================================================
    // 地址常量 (来自 flux_cnn_params.svh, 由 params.py codegen 生成)
    // =========================================================================
    localparam [ADDR_W-1:0] ADDR_CTRL             = `FLUX_ADDR_CTRL;
    localparam [ADDR_W-1:0] ADDR_STATUS           = `FLUX_ADDR_STATUS;
    localparam [ADDR_W-1:0] ADDR_SEQ_DBG          = 12'h008;   // VD100 dbg RO

    localparam [ADDR_W-1:0] ADDR_H_OUT            = `FLUX_ADDR_H_OUT;
    localparam [ADDR_W-1:0] ADDR_W_OUT            = `FLUX_ADDR_W_OUT;
    localparam [ADDR_W-1:0] ADDR_W_IN             = `FLUX_ADDR_W_IN;
    localparam [ADDR_W-1:0] ADDR_K                = `FLUX_ADDR_K;
    localparam [ADDR_W-1:0] ADDR_STRIDE           = `FLUX_ADDR_STRIDE;
    localparam [ADDR_W-1:0] ADDR_STRIDE_H         = `FLUX_ADDR_STRIDE_H;
    localparam [ADDR_W-1:0] ADDR_CIN_SLICES       = `FLUX_ADDR_CIN_SLICES;
    localparam [ADDR_W-1:0] ADDR_COUT_SLICES      = `FLUX_ADDR_COUT_SLICES;
    localparam [ADDR_W-1:0] ADDR_TILE_W           = `FLUX_ADDR_TILE_W;
    localparam [ADDR_W-1:0] ADDR_NUM_TILES        = `FLUX_ADDR_NUM_TILES;
    localparam [ADDR_W-1:0] ADDR_LAST_VALID_W     = `FLUX_ADDR_LAST_VALID_W;
    localparam [ADDR_W-1:0] ADDR_TOTAL_WRF        = `FLUX_ADDR_TOTAL_WRF;
    localparam [ADDR_W-1:0] ADDR_KY               = `FLUX_ADDR_KY;
    localparam [ADDR_W-1:0] ADDR_KK               = `FLUX_ADDR_KK;
    localparam [ADDR_W-1:0] ADDR_ROUNDS_PER_CINS  = `FLUX_ADDR_ROUNDS_PER_CINS;
    localparam [ADDR_W-1:0] ADDR_ROUND_LEN_LAST   = `FLUX_ADDR_ROUND_LEN_LAST;
    localparam [ADDR_W-1:0] ADDR_IFB_BASE         = `FLUX_ADDR_IFB_BASE;
    localparam [ADDR_W-1:0] ADDR_WB_BASE          = `FLUX_ADDR_WB_BASE;
    localparam [ADDR_W-1:0] ADDR_OFB_BASE         = `FLUX_ADDR_OFB_BASE;
    localparam [ADDR_W-1:0] ADDR_IFB_ROW_STEP     = `FLUX_ADDR_IFB_ROW_STEP;
    localparam [ADDR_W-1:0] ADDR_WB_COUT_STEP     = `FLUX_ADDR_WB_COUT_STEP;
    localparam [ADDR_W-1:0] ADDR_TILE_IN_STEP     = `FLUX_ADDR_TILE_IN_STEP;
    localparam [ADDR_W-1:0] ADDR_SDP_SHIFT        = `FLUX_ADDR_SDP_SHIFT;
    localparam [ADDR_W-1:0] ADDR_SDP_RELU_EN      = `FLUX_ADDR_SDP_RELU_EN;

    localparam [ADDR_W-1:0] ADDR_H_IN_TOTAL       = `FLUX_ADDR_H_IN_TOTAL;
    localparam [ADDR_W-1:0] ADDR_IFB_STRIP_ROWS   = `FLUX_ADDR_IFB_STRIP_ROWS;
    localparam [ADDR_W-1:0] ADDR_OFB_STRIP_ROWS   = `FLUX_ADDR_OFB_STRIP_ROWS;
    localparam [ADDR_W-1:0] ADDR_DDR_IFM_ROW_STR  = `FLUX_ADDR_DDR_IFM_ROW_STRIDE;
    localparam [ADDR_W-1:0] ADDR_DDR_OFM_ROW_STR  = `FLUX_ADDR_DDR_OFM_ROW_STRIDE;
    localparam [ADDR_W-1:0] ADDR_DMA_MODE         = `FLUX_ADDR_DMA_MODE;

    localparam [ADDR_W-1:0] ADDR_DESC_LIST_BASE   = `FLUX_ADDR_DESC_LIST_BASE;
    localparam [ADDR_W-1:0] ADDR_DESC_COUNT       = `FLUX_ADDR_DESC_COUNT;
    localparam [ADDR_W-1:0] ADDR_SDP_MULT         = `FLUX_ADDR_SDP_MULT;
    localparam [ADDR_W-1:0] ADDR_SDP_ZP_OUT       = `FLUX_ADDR_SDP_ZP_OUT;
    localparam [ADDR_W-1:0] ADDR_SDP_CLIP_MIN     = `FLUX_ADDR_SDP_CLIP_MIN;
    localparam [ADDR_W-1:0] ADDR_SDP_CLIP_MAX     = `FLUX_ADDR_SDP_CLIP_MAX;
    localparam [ADDR_W-1:0] ADDR_SDP_ROUND_EN     = `FLUX_ADDR_SDP_ROUND_EN;
    localparam [ADDR_W-1:0] ADDR_IFB_RING_WORDS   = `FLUX_ADDR_IFB_RING_WORDS;
    localparam [ADDR_W-1:0] ADDR_OFB_ROW_WORDS    = `FLUX_ADDR_OFB_ROW_WORDS;
    localparam [ADDR_W-1:0] ADDR_OFB_RING_WORDS   = `FLUX_ADDR_OFB_RING_WORDS;
    localparam [ADDR_W-1:0] ADDR_IFB_ISS_STEP     = `FLUX_ADDR_IFB_ISS_STEP;
    localparam [ADDR_W-1:0] ADDR_IFB_KY_STEP      = `FLUX_ADDR_IFB_KY_STEP;
    localparam [ADDR_W-1:0] ADDR_TILE_PIX_STEP    = `FLUX_ADDR_TILE_PIX_STEP;
    localparam [ADDR_W-1:0] ADDR_ARF_REUSE_EN     = `FLUX_ADDR_ARF_REUSE_EN;
    localparam [ADDR_W-1:0] ADDR_RESIDUAL_EN      = `FLUX_ADDR_RESIDUAL_EN;
    localparam [ADDR_W-1:0] ADDR_SHORTCUT_MULT    = `FLUX_ADDR_SHORTCUT_MULT;
    localparam [ADDR_W-1:0] ADDR_SHORTCUT_SHIFT   = `FLUX_ADDR_SHORTCUT_SHIFT;
    localparam [ADDR_W-1:0] ADDR_BIAS_BASE        = `FLUX_ADDR_BIAS_BASE;
    localparam [ADDR_W-1:0] ADDR_SKIP_IDMA        = `FLUX_ADDR_SKIP_IDMA;
    localparam [ADDR_W-1:0] ADDR_OFM_TDEST        = `FLUX_ADDR_OFM_TDEST;
    localparam [ADDR_W-1:0] ADDR_OFM_OPCODE       = `FLUX_ADDR_OFM_OPCODE;
    localparam [ADDR_W-1:0] ADDR_IDMA_CMD_LIST_PTR  = `FLUX_ADDR_IDMA_CMD_LIST_PTR;
    localparam [ADDR_W-1:0] ADDR_IDMA_CMD_COUNT     = `FLUX_ADDR_IDMA_CMD_COUNT;
    localparam [ADDR_W-1:0] ADDR_IDMA_CMDS_PER_ROW  = `FLUX_ADDR_IDMA_CMDS_PER_ROW;
    localparam [ADDR_W-1:0] ADDR_ODMA_CMD_LIST_PTR  = `FLUX_ADDR_ODMA_CMD_LIST_PTR;
    localparam [ADDR_W-1:0] ADDR_ODMA_CMD_COUNT     = `FLUX_ADDR_ODMA_CMD_COUNT;
    localparam [ADDR_W-1:0] ADDR_ODMA_CMDS_PER_ROW  = `FLUX_ADDR_ODMA_CMDS_PER_ROW;

    localparam [ADDR_W-1:0] ADDR_IDMA_SRC_BASE    = `FLUX_ADDR_IDMA_SRC_BASE;
    localparam [ADDR_W-1:0] ADDR_IDMA_BYTE_LEN    = `FLUX_ADDR_IDMA_BYTE_LEN;
    localparam [ADDR_W-1:0] ADDR_WDMA_SRC_BASE    = `FLUX_ADDR_WDMA_SRC_BASE;
    localparam [ADDR_W-1:0] ADDR_WDMA_BYTE_LEN    = `FLUX_ADDR_WDMA_BYTE_LEN;
    localparam [ADDR_W-1:0] ADDR_ODMA_DST_BASE    = `FLUX_ADDR_ODMA_DST_BASE;
    localparam [ADDR_W-1:0] ADDR_ODMA_BYTE_LEN    = `FLUX_ADDR_ODMA_BYTE_LEN;
    localparam [ADDR_W-1:0] ADDR_RDMA_SRC_BASE    = `FLUX_ADDR_RDMA_SRC_BASE;
    localparam [ADDR_W-1:0] ADDR_RDMA_BYTE_LEN    = `FLUX_ADDR_RDMA_BYTE_LEN;

    // =========================================================================
    // start pulse 生成 (csr_w only): CTRL 写 1 → 当拍 pulse
    // =========================================================================
    assign start_dfe_pulse   = csr_w_en && (csr_w_addr == ADDR_CTRL) && csr_w_data[4];
    assign start_layer_pulse = csr_w_en && (csr_w_addr == ADDR_CTRL) && csr_w_data[5];

    // -------------------------------------------------------------------------
    // soft reset stretcher: host 写 CTRL.bit7 = 1 → r_soft_rst_cnt 装 64 拍 → 持续
    // 64 拍 soft_reset_n=0 → 所有 sub-module + axi_dm IP 内部全部 reset 回 IDLE.
    // VD100 fix 2026-05-15: 拉长 16 → 64 拍, 因为 axi_dm IP 内部有多个 reset domain
    // (mm2s/s2mm/cmdsts), 16 拍可能不够完全 sync. axi_dm aresetn 之前用 hard rst_n,
    // 现已改为 core_rst_n 跟其他 sub-module 同步.
    // 仅 csr_w 触发, sequencer 自己写 CFG_WRITE 不会触发 (seq_w_addr 只能写 0x100+).
    // -------------------------------------------------------------------------
    logic [5:0] r_soft_rst_cnt;
    logic       soft_reset_trigger;
    assign soft_reset_trigger = csr_w_en && (csr_w_addr == ADDR_CTRL) && csr_w_data[7];

    always_ff @(posedge clk) begin
        if      (!rst_n)              r_soft_rst_cnt <= 6'd0;
        else if (soft_reset_trigger)  r_soft_rst_cnt <= 6'd63;
        else if (r_soft_rst_cnt != 0) r_soft_rst_cnt <= r_soft_rst_cnt - 6'd1;
    end
    assign soft_reset_n = (r_soft_rst_cnt == 6'd0);

    // -------------------------------------------------------------------------
    // core_done sticky: layer_done 上升沿 set, start_layer_pulse 清掉
    //   - host poll: STATUS[0] = 1 表示有"未消费"的 done; 写 CTRL[5]=1 启下一层会自清
    //   - GIC level IRQ: 拉到顶层 done 端口, 一直高直到 host 启下一层
    // -------------------------------------------------------------------------
    logic r_core_done_sticky, layer_done_d1;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            r_core_done_sticky <= 1'b0;
            layer_done_d1      <= 1'b0;
        end else begin
            layer_done_d1 <= layer_done;
            if      (start_layer_pulse)              r_core_done_sticky <= 1'b0;
            else if (layer_done && !layer_done_d1)   r_core_done_sticky <= 1'b1;
        end
    end
    assign core_done_sticky = r_core_done_sticky;

    // =========================================================================
    // DMA_MODE 是控制路径 (boot reg, csr_w only), 必须复位（§6.1）
    // =========================================================================
    logic [1:0] r_dma_mode_ctrl;
    always_ff @(posedge clk) begin
        if      (!rst_n)                                    r_dma_mode_ctrl <= 2'b00;
        else if (csr_w_en && csr_w_addr == ADDR_DMA_MODE)   r_dma_mode_ctrl <= csr_w_data[1:0];
    end

    // SKIP_IDMA (M2 控制路径, 跨核 consumer 标志):
    //   原本只 seq_w_en 写 (desc 内 CFG_WRITE).
    //   mesh 模式新增 host (csr_w_en) 也能写, host 在 start_layer 前直接配 1.
    logic r_skip_idma;
    always_ff @(posedge clk) begin
        if      (!rst_n)                                       r_skip_idma <= 1'b0;
        else if (csr_w_en && csr_w_addr == ADDR_SKIP_IDMA)     r_skip_idma <= csr_w_data[0];
        else if (seq_w_en && seq_w_addr == ADDR_SKIP_IDMA)     r_skip_idma <= seq_w_data[0];
    end

    // OFM_TDEST / OFM_OPCODE (mesh 控制路径):
    //   ODMA 出包目的节点和 opcode, host start_layer 前 csr_w 配置一次, 整层用同一目标.
    //   不进 desc CFG_WRITE 流, 不需要 seq_w 路径.
    logic [7:0] r_ofm_tdest;
    logic [3:0] r_ofm_opcode;
    always_ff @(posedge clk) begin
        if      (!rst_n)                                       r_ofm_tdest  <= 8'd0;
        else if (csr_w_en && csr_w_addr == ADDR_OFM_TDEST)     r_ofm_tdest  <= csr_w_data[7:0];
    end
    always_ff @(posedge clk) begin
        if      (!rst_n)                                       r_ofm_opcode <= 4'd0;
        else if (csr_w_en && csr_w_addr == ADDR_OFM_OPCODE)    r_ofm_opcode <= csr_w_data[3:0];
    end

    // IFB_STRIP_ROWS / IFB_RING_WORDS (mesh 模式 host 旁路写):
    //   原本只 seq_w_en 写 (desc CFG_WRITE 内, layer cfg 一部分).
    //   mesh 模式跨核 consumer 不走自己的 desc list, host start_layer 前需要直接配置
    //   IFB ring 容量 (否则 ring full → AWREADY=0 → 远端 push 死锁).
    //   csr_w 优先级高于 seq_w (本核既被 host 配过又恰好有 desc 时取 host 值).
    //   start_layer_pulse 时清 vld: 多层 chain 时每层 desc 重新接管 (W slice 多核不同
    //   sub_W → 不同 ring_words, host 写一次不能共享).
    logic [15:0]             r_ifb_strip_rows_host;  // 8→16 bit
    logic [CORE_ADDR_W-1:0]  r_ifb_ring_words_host;
    logic                    r_ifb_strip_rows_host_vld;
    logic                    r_ifb_ring_words_host_vld;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            r_ifb_strip_rows_host_vld <= 1'b0;
            r_ifb_ring_words_host_vld <= 1'b0;
        end else begin
            if (start_layer_pulse) begin
                r_ifb_strip_rows_host_vld <= 1'b0;
                r_ifb_ring_words_host_vld <= 1'b0;
            end
            if (csr_w_en && csr_w_addr == ADDR_IFB_STRIP_ROWS) begin
                r_ifb_strip_rows_host     <= csr_w_data[15:0];  // 8→16 bit
                r_ifb_strip_rows_host_vld <= 1'b1;
            end
            if (csr_w_en && csr_w_addr == ADDR_IFB_RING_WORDS) begin
                r_ifb_ring_words_host     <= csr_w_data[CORE_ADDR_W-1:0];
                r_ifb_ring_words_host_vld <= 1'b1;
            end
        end
    end

    // =========================================================================
    // Boot 寄存器 (csr_w only): DESC_LIST_BASE / DESC_COUNT
    //   host 写完这两个 + CTRL 触发后, sequencer 自动消费 desc list 改 layer cfg.
    // =========================================================================
    logic [31:0]             r_desc_list_base;
    logic [15:0]             r_desc_count;
    always_ff @(posedge clk) begin
        if (csr_w_en) begin
            case (csr_w_addr)
                ADDR_DESC_LIST_BASE  : r_desc_list_base  <= csr_w_data[31:0];
                ADDR_DESC_COUNT      : r_desc_count      <= csr_w_data[15:0];
                default              : ;
            endcase
        end
    end

    // =========================================================================
    // 配置 / DMA 寄存器 —— 每寄存器独立 always_ff，数据路径无复位（§6）
    // =========================================================================
    logic [15:0]             r_h_out;
    logic [15:0]             r_w_out;
    logic [15:0]             r_w_in;
    logic [3:0]              r_k;
    logic [3:0]              r_ky;
    logic [2:0]              r_stride;
    logic [2:0]              r_stride_h;
    logic [5:0]              r_cin_slices;
    logic [5:0]              r_cout_slices;
    logic [5:0]              r_tile_w;
    logic [7:0]              r_num_tiles;
    logic [5:0]              r_last_valid_w;
    logic [9:0]              r_total_wrf;
    logic [9:0]              r_kk;
    logic [2:0]              r_rounds_per_cins;
    logic [5:0]              r_round_len_last;
    logic [CORE_ADDR_W-1:0]  r_ifb_base;
    logic [CORE_ADDR_W-1:0]  r_wb_base;
    logic [CORE_ADDR_W-1:0]  r_ofb_base;
    logic [CORE_ADDR_W-1:0]  r_ifb_row_step;
    logic [CORE_ADDR_W-1:0]  r_wb_cout_step;
    logic [CORE_ADDR_W-1:0]  r_tile_in_step;
    logic [CORE_ADDR_W-1:0]  r_ifb_ring_words;
    logic [CORE_ADDR_W-1:0]  r_ofb_row_words;
    logic [CORE_ADDR_W-1:0]  r_ofb_ring_words;
    logic [CORE_ADDR_W-1:0]  r_ifb_iss_step;
    logic [CORE_ADDR_W-1:0]  r_ifb_ky_step;
    logic [15:0]             r_tile_pix_step;
    logic                    r_arf_reuse_en;
    logic [5:0]              r_sdp_shift;
    logic                    r_sdp_relu_en;
    logic [15:0]             r_h_in_total;
    logic [15:0]             r_ifb_strip_rows;   // 8→16 bit (任意 H)
    logic [15:0]             r_ofb_strip_rows;   // 6→8→16 bit
    logic [CORE_ADDR_W-1:0]  r_ddr_ifm_row_stride;
    logic [CORE_ADDR_W-1:0]  r_ddr_ofm_row_stride;
    logic [31:0]             r_idma_src_base;
    logic [23:0]             r_idma_byte_len;
    logic [31:0]             r_wdma_src_base;
    logic [23:0]             r_wdma_byte_len;
    logic [31:0]             r_odma_dst_base;
    logic [23:0]             r_odma_byte_len;
    // r_desc_list_base / r_desc_count 已在上方 boot 块声明 (csr_w 写)
    logic signed [31:0]      r_sdp_mult;
    logic signed [8:0]       r_sdp_zp_out;
    logic signed [8:0]       r_sdp_clip_min;
    logic signed [8:0]       r_sdp_clip_max;
    logic                    r_sdp_round_en;
    // R.1: residual / shortcut / bias / rdma
    logic                    r_residual_en;
    logic signed [15:0]      r_shortcut_mult;
    logic [4:0]              r_shortcut_shift;
    logic [12:0]             r_bias_base;
    logic [31:0]             r_rdma_src_base;
    logic [23:0]             r_rdma_byte_len;
    // SMC + NUMA: IDMA SG cmd list (Phase 7)
    logic [31:0]             r_idma_cmd_list_ptr;
    logic [15:0]             r_idma_cmd_count;
    logic [7:0]              r_idma_cmds_per_row;
    // SMC + NUMA: ODMA SG cmd list (Phase 7)
    logic [31:0]             r_odma_cmd_list_ptr;
    logic [15:0]             r_odma_cmd_count;
    logic [7:0]              r_odma_cmds_per_row;

    // Layer cfg 寄存器 bank (seq_w only, sequencer 消费 CFG_WRITE descriptor 写入).
    //   §4.1 例外 2: 共享 seq_w_en 门控 + addr 解码, 所有 layer cfg 寄存器写在同一
    //   always_ff 内, 未命中时所有寄存器隐式保持。数据路径 §6 不复位。
    always_ff @(posedge clk) begin
        if (seq_w_en) begin
            case (seq_w_addr)
                ADDR_H_OUT           : r_h_out           <= seq_w_data[15:0];
                ADDR_W_OUT           : r_w_out           <= seq_w_data[15:0];
                ADDR_W_IN            : r_w_in            <= seq_w_data[15:0];
                ADDR_K               : r_k               <= seq_w_data[3:0];
                ADDR_STRIDE          : begin
                    r_stride          <= seq_w_data[2:0];
                    r_stride_h        <= seq_w_data[2:0];   // Round I: 写 STRIDE 同步设 stride_h (默认 = W stride). 后续 STRIDE_H 可单独 override.
                end
                ADDR_STRIDE_H        : r_stride_h        <= seq_w_data[2:0];
                ADDR_CIN_SLICES      : r_cin_slices      <= seq_w_data[5:0];
                ADDR_COUT_SLICES     : r_cout_slices     <= seq_w_data[5:0];
                ADDR_TILE_W          : r_tile_w          <= seq_w_data[5:0];
                ADDR_NUM_TILES       : r_num_tiles       <= seq_w_data[7:0];
                ADDR_LAST_VALID_W    : r_last_valid_w    <= seq_w_data[5:0];
                ADDR_TOTAL_WRF       : r_total_wrf       <= seq_w_data[9:0];
                ADDR_KY              : r_ky              <= seq_w_data[3:0];
                ADDR_KK              : r_kk              <= seq_w_data[9:0];
                ADDR_ROUNDS_PER_CINS : r_rounds_per_cins <= seq_w_data[2:0];
                ADDR_ROUND_LEN_LAST  : r_round_len_last  <= seq_w_data[5:0];
                ADDR_IFB_BASE        : r_ifb_base        <= seq_w_data[CORE_ADDR_W-1:0];
                ADDR_WB_BASE         : r_wb_base         <= seq_w_data[CORE_ADDR_W-1:0];
                ADDR_OFB_BASE        : r_ofb_base        <= seq_w_data[CORE_ADDR_W-1:0];
                ADDR_IFB_ROW_STEP    : r_ifb_row_step    <= seq_w_data[CORE_ADDR_W-1:0];
                ADDR_WB_COUT_STEP    : r_wb_cout_step    <= seq_w_data[CORE_ADDR_W-1:0];
                ADDR_TILE_IN_STEP    : r_tile_in_step    <= seq_w_data[CORE_ADDR_W-1:0];
                ADDR_IFB_RING_WORDS  : r_ifb_ring_words  <= seq_w_data[CORE_ADDR_W-1:0];
                ADDR_OFB_ROW_WORDS   : r_ofb_row_words   <= seq_w_data[CORE_ADDR_W-1:0];
                ADDR_OFB_RING_WORDS  : r_ofb_ring_words  <= seq_w_data[CORE_ADDR_W-1:0];
                ADDR_IFB_ISS_STEP    : r_ifb_iss_step    <= seq_w_data[CORE_ADDR_W-1:0];
                ADDR_IFB_KY_STEP     : r_ifb_ky_step     <= seq_w_data[CORE_ADDR_W-1:0];
                ADDR_TILE_PIX_STEP   : r_tile_pix_step   <= seq_w_data[15:0];
                ADDR_ARF_REUSE_EN    : r_arf_reuse_en    <= seq_w_data[0];
                ADDR_SDP_SHIFT       : r_sdp_shift       <= seq_w_data[5:0];
                ADDR_SDP_RELU_EN     : r_sdp_relu_en     <= seq_w_data[0];
                ADDR_H_IN_TOTAL      : r_h_in_total      <= seq_w_data[15:0];
                ADDR_IFB_STRIP_ROWS  : r_ifb_strip_rows  <= seq_w_data[15:0];  // 8→16 bit
                ADDR_OFB_STRIP_ROWS  : r_ofb_strip_rows  <= seq_w_data[15:0];  // 6→8→16 bit
                ADDR_DDR_IFM_ROW_STR : r_ddr_ifm_row_stride <= seq_w_data[CORE_ADDR_W-1:0];
                ADDR_DDR_OFM_ROW_STR : r_ddr_ofm_row_stride <= seq_w_data[CORE_ADDR_W-1:0];
                ADDR_IDMA_SRC_BASE   : r_idma_src_base   <= seq_w_data[31:0];
                ADDR_IDMA_BYTE_LEN   : r_idma_byte_len   <= seq_w_data[23:0];
                ADDR_WDMA_SRC_BASE   : r_wdma_src_base   <= seq_w_data[31:0];
                ADDR_WDMA_BYTE_LEN   : r_wdma_byte_len   <= seq_w_data[23:0];
                ADDR_ODMA_DST_BASE   : r_odma_dst_base   <= seq_w_data[31:0];
                ADDR_ODMA_BYTE_LEN   : r_odma_byte_len   <= seq_w_data[23:0];
                ADDR_SDP_MULT        : r_sdp_mult        <= $signed(seq_w_data[31:0]);
                ADDR_SDP_ZP_OUT      : r_sdp_zp_out      <= $signed(seq_w_data[8:0]);
                ADDR_SDP_CLIP_MIN    : r_sdp_clip_min    <= $signed(seq_w_data[8:0]);
                ADDR_SDP_CLIP_MAX    : r_sdp_clip_max    <= $signed(seq_w_data[8:0]);
                ADDR_SDP_ROUND_EN    : r_sdp_round_en    <= seq_w_data[0];
                ADDR_RESIDUAL_EN     : r_residual_en     <= seq_w_data[0];
                ADDR_SHORTCUT_MULT   : r_shortcut_mult   <= $signed(seq_w_data[15:0]);
                ADDR_SHORTCUT_SHIFT  : r_shortcut_shift  <= seq_w_data[4:0];
                ADDR_BIAS_BASE       : r_bias_base       <= seq_w_data[12:0];
                ADDR_RDMA_SRC_BASE   : r_rdma_src_base   <= seq_w_data[31:0];
                ADDR_RDMA_BYTE_LEN   : r_rdma_byte_len   <= seq_w_data[23:0];
                ADDR_IDMA_CMD_LIST_PTR : r_idma_cmd_list_ptr  <= seq_w_data[31:0];
                ADDR_IDMA_CMD_COUNT    : r_idma_cmd_count     <= seq_w_data[15:0];
                ADDR_IDMA_CMDS_PER_ROW : r_idma_cmds_per_row  <= seq_w_data[7:0];
                ADDR_ODMA_CMD_LIST_PTR : r_odma_cmd_list_ptr  <= seq_w_data[31:0];
                ADDR_ODMA_CMD_COUNT    : r_odma_cmd_count     <= seq_w_data[15:0];
                ADDR_ODMA_CMDS_PER_ROW : r_odma_cmds_per_row  <= seq_w_data[7:0];
                default              : ;   // 未使用地址
            endcase
        end
    end

    // =========================================================================
    // 输出直通
    // =========================================================================
    assign h_out           = r_h_out;
    assign w_out           = r_w_out;
    assign w_in            = r_w_in;
    assign k               = r_k;
    assign ky              = r_ky;
    assign stride          = r_stride;
    assign stride_h        = r_stride_h;
    assign cin_slices      = r_cin_slices;
    assign cout_slices     = r_cout_slices;
    assign tile_w          = r_tile_w;
    assign num_tiles       = r_num_tiles;
    assign last_valid_w    = r_last_valid_w;
    assign total_wrf       = r_total_wrf;
    assign kk              = r_kk;
    assign rounds_per_cins = r_rounds_per_cins;
    assign round_len_last  = r_round_len_last;
    assign ifb_base        = r_ifb_base;
    assign wb_base         = r_wb_base;
    assign ofb_base        = r_ofb_base;
    assign ifb_row_step    = r_ifb_row_step;
    assign wb_cout_step    = r_wb_cout_step;
    assign tile_in_step    = r_tile_in_step;
    assign ifb_ring_words  = r_ifb_ring_words_host_vld ? r_ifb_ring_words_host : r_ifb_ring_words;
    assign ofb_row_words   = r_ofb_row_words;
    assign ofb_ring_words  = r_ofb_ring_words;
    assign ifb_iss_step    = r_ifb_iss_step;
    assign ifb_ky_step     = r_ifb_ky_step;
    assign tile_pix_step   = r_tile_pix_step;
    assign arf_reuse_en    = r_arf_reuse_en;
    assign sdp_shift       = r_sdp_shift;
    assign sdp_relu_en     = r_sdp_relu_en;
    assign h_in_total         = r_h_in_total;
    assign ifb_strip_rows     = r_ifb_strip_rows_host_vld ? r_ifb_strip_rows_host : r_ifb_strip_rows;
    assign ofb_strip_rows     = r_ofb_strip_rows;
    assign ddr_ifm_row_stride = r_ddr_ifm_row_stride;
    assign ddr_ofm_row_stride = r_ddr_ofm_row_stride;
    // J-2: r_dma_mode_ctrl 保留寄存器不 drive 输出 (硬件恒 streaming)
    assign idma_src_base   = r_idma_src_base;
    assign idma_byte_len   = r_idma_byte_len;
    assign wdma_src_base   = r_wdma_src_base;
    assign wdma_byte_len   = r_wdma_byte_len;
    assign odma_dst_base   = r_odma_dst_base;
    assign odma_byte_len   = r_odma_byte_len;
    assign desc_list_base  = r_desc_list_base;
    assign desc_count      = r_desc_count;
    assign sdp_mult        = r_sdp_mult;
    assign sdp_zp_out      = r_sdp_zp_out;
    assign sdp_clip_min    = r_sdp_clip_min;
    assign sdp_clip_max    = r_sdp_clip_max;
    assign sdp_round_en    = r_sdp_round_en;
    assign residual_en     = r_residual_en;
    assign skip_idma       = r_skip_idma;
    assign shortcut_mult   = r_shortcut_mult;
    assign shortcut_shift  = r_shortcut_shift;
    assign bias_base       = r_bias_base;
    assign rdma_src_base   = r_rdma_src_base;
    assign rdma_byte_len   = r_rdma_byte_len;
    assign ofm_tdest       = r_ofm_tdest;
    assign ofm_opcode      = r_ofm_opcode;
    assign idma_cmd_list_ptr  = r_idma_cmd_list_ptr;
    assign idma_cmd_count     = r_idma_cmd_count;
    assign idma_cmds_per_row  = r_idma_cmds_per_row;
    assign odma_cmd_list_ptr  = r_odma_cmd_list_ptr;
    assign odma_cmd_count     = r_odma_cmd_count;
    assign odma_cmds_per_row  = r_odma_cmds_per_row;

    // =========================================================================
    // 读 mux：按 reg_r_addr 选择返回数据（组合）
    // =========================================================================
    logic [DATA_W-1:0] status_word;
    // STATUS[0] = sticky core_done (host poll: 1 = 上一层完成且未启下一层)
    // STATUS[11] = layer_done (instantaneous level; 仅 sequencer S_END 那 1-2 拍高)
    assign status_word = {20'd0,
                          layer_done, layer_busy, dfe_done, dfe_busy,
                          odma_done,  wdma_done,  idma_done,
                          odma_busy,  wdma_busy,  idma_busy,
                          layer_busy, r_core_done_sticky};

    // 优化 (2026-05-17): 读 mux 从 68 项削到 ~20 项, 只保留 host 实际 peek_csr 的地址.
    // 其它 layer cfg 寄存器全部不再 readback (sequencer 写完后用作硬件信号, 没人读).
    // 验证: grep "peek_csr.*0x" host/vd100_pc/*.py 实际只用 18 个地址.
    // 估省 LUT: 68→20 项 32-bit case mux ≈ 2000+ LUT.
    always_comb begin
        case (reg_r_addr)
            ADDR_CTRL              : reg_r_data = '0;
            ADDR_STATUS            : reg_r_data = status_word;
            ADDR_SEQ_DBG           : reg_r_data = {16'd0, master_rvalid, master_arvalid, fifo_count[3:0], seq_state};
            // 以下是 host debug 偶尔 peek 的: layer cfg sanity check / DMA cmd verify
            ADDR_H_OUT             : reg_r_data = {16'd0, r_h_out};
            ADDR_W_OUT             : reg_r_data = {16'd0, r_w_out};
            ADDR_SDP_SHIFT         : reg_r_data = {26'd0, r_sdp_shift};
            ADDR_H_IN_TOTAL        : reg_r_data = {16'd0, r_h_in_total};
            ADDR_IFB_STRIP_ROWS    : reg_r_data = {16'd0, r_ifb_strip_rows};
            ADDR_OFB_STRIP_ROWS    : reg_r_data = {16'd0, r_ofb_strip_rows};
            ADDR_DESC_LIST_BASE    : reg_r_data = r_desc_list_base;
            ADDR_DESC_COUNT        : reg_r_data = {16'd0, r_desc_count};
            ADDR_SDP_MULT          : reg_r_data = $unsigned(r_sdp_mult);
            ADDR_IFB_RING_WORDS    : reg_r_data = {12'd0, r_ifb_ring_words};
            ADDR_OFB_ROW_WORDS     : reg_r_data = {12'd0, r_ofb_row_words};
            ADDR_OFB_RING_WORDS    : reg_r_data = {12'd0, r_ofb_ring_words};
            ADDR_BIAS_BASE         : reg_r_data = {19'd0, r_bias_base};
            ADDR_IDMA_CMD_COUNT    : reg_r_data = {16'd0, r_idma_cmd_count};
            ADDR_IDMA_CMDS_PER_ROW : reg_r_data = {24'd0, r_idma_cmds_per_row};
            ADDR_ODMA_CMD_LIST_PTR : reg_r_data = r_odma_cmd_list_ptr;
            ADDR_ODMA_CMD_COUNT    : reg_r_data = {16'd0, r_odma_cmd_count};
            ADDR_ODMA_CMDS_PER_ROW : reg_r_data = {24'd0, r_odma_cmds_per_row};
            ADDR_IDMA_SRC_BASE     : reg_r_data = r_idma_src_base;
            default                : reg_r_data = '0;   // 其它 ~50 项 host 不读, 返 0
        endcase
    end

endmodule
