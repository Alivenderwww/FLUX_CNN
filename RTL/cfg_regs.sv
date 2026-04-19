`timescale 1ns/1ps

// =============================================================================
// cfg_regs.sv  --  Shared Configuration + DMA Descriptor Register Bank
//
// 通过 AXI-Lite decoded 端口（reg_w_en/addr/data + reg_r_addr/data）接受外部
// 读写。Bridge 在 axi_lite_csr.sv 里；本模块只做 address decode + 寄存器组。
//
// Register map（4 字节对齐）：
//   0x000  CTRL              [0]=start_core (legacy, 兼容老 TB 直接写 CTRL=0xB)
//                             [1]=start_idma  [2]=start_wdma  [3]=start_odma
//                             [4]=start_dfe   [5]=start_layer (new, Phase C-1)
//   0x004  STATUS  (RO)      [0]=core_done     [1]=core_busy
//                             [2]=idma_busy    [3]=wdma_busy [4]=odma_busy
//                             [5]=idma_done    [6]=wdma_done [7]=odma_done
//                             [8]=dfe_busy     [9]=dfe_done
//                             [10]=layer_busy  [11]=layer_done
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
//   0x12C  WRF_PACKED        [0]
//   0x130  KK                [9:0]
//   0x134  ROUNDS_PER_CINS   [2:0]
//   0x138  ROUND_LEN_LAST    [5:0]
//   0x13C  IFB_BASE          [19:0]
//   0x140  WB_BASE           [19:0]
//   0x144  OFB_BASE          [19:0]
//   0x148  IFB_CIN_STEP      [19:0]
//   0x14C  IFB_ROW_STEP      [19:0]
//   0x150  WB_CIN_STEP       [19:0]
//   0x154  WB_COUT_STEP      [19:0]
//   0x158  OFB_COUT_STEP     [19:0]
//   0x15C  TILE_IN_STEP      [19:0]
//   0x160  SDP_SHIFT         [5:0]   (Phase C: 5→6 bit)
//   0x164  SDP_RELU_EN       [0]
//   0x168  H_IN_TOTAL        [15:0]            整图输入高度（streaming 用）
//   0x16C  IFB_STRIP_ROWS    [7:0]             IFB ring 容纳输入行数
//   0x170  OFB_STRIP_ROWS    [5:0]             OFB ring 容纳输出行数
//   0x174  DDR_IFM_ROW_STRIDE[19:0]            DDR 相邻输入行跨度（字节）
//   0x178  DDR_OFM_ROW_STRIDE[19:0]            DDR 相邻输出行跨度（字节）
//   0x17C  DMA_MODE          [0]=idma_stream [1]=odma_stream
//   0x180  DESC_LIST_BASE    [31:0]  (Phase C)
//   0x184  DESC_COUNT        [15:0]  (Phase C)
//   0x188  SDP_MULT          [31:0]  signed (Phase C)
//   0x18C  SDP_ZP_OUT        [8:0]   signed (Phase C)
//   0x190  SDP_CLIP_MIN      [8:0]   signed (Phase C)
//   0x194  SDP_CLIP_MAX      [8:0]   signed (Phase C)
//   0x198  SDP_ROUND_EN      [0]            (Phase C)
//   0x1A0  PAD_TOP           [3:0]          (Phase C-2 临时全局 cfg，供老 CTRL 路径验证 pad)
//   0x1A4  PAD_LEFT          [3:0]          (Phase C-2 同上)
//   0x200  IDMA_SRC_BASE     [31:0]
//   0x204  IDMA_BYTE_LEN     [23:0]
//   0x210  WDMA_SRC_BASE     [31:0]
//   0x214  WDMA_BYTE_LEN     [23:0]
//   0x220  ODMA_DST_BASE     [31:0]
//   0x224  ODMA_BYTE_LEN     [23:0]
//
// 复位：按 §6，数据路径 cfg / DMA 寄存器不加复位（上电 X，靠上游 TB/host
// 先写再 start 保证 valid）。控制路径 core_busy 有复位。
// =============================================================================

module cfg_regs #(
    parameter int ADDR_W   = 12,
    parameter int DATA_W   = 32,
    parameter int CORE_ADDR_W = 20   // 内部 SRAM 地址位宽（同 core_top）
)(
    input  logic                     clk,
    input  logic                     rst_n,

    // ---- AXI-Lite decoded 读写端口 ----
    input  logic                     reg_w_en,
    input  logic [ADDR_W-1:0]        reg_w_addr,
    input  logic [DATA_W-1:0]        reg_w_data,
    input  logic [DATA_W/8-1:0]      reg_w_strb,     // 暂时不解析，全 32-bit 写
    input  logic [ADDR_W-1:0]        reg_r_addr,
    output logic [DATA_W-1:0]        reg_r_data,

    // ---- Status 输入（来自 core 内部）----
    input  logic                     core_done,
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

    // ---- CTRL 输出（6 个独立 1-拍脉冲） ----
    output logic                     start_core_pulse,  // CTRL[0] 写 1 (legacy)
    output logic                     start_idma_pulse,  // CTRL[1] 写 1 (legacy)
    output logic                     start_wdma_pulse,  // CTRL[2] 写 1 (legacy)
    output logic                     start_odma_pulse,  // CTRL[3] 写 1 (legacy)
    output logic                     start_dfe_pulse,   // CTRL[4] 写 1 (Phase C)
    output logic                     start_layer_pulse, // CTRL[5] 写 1 (Phase C)

    // ---- 配置输出 ----
    output logic [15:0]              h_out,
    output logic [15:0]              w_out,
    output logic [15:0]              w_in,
    output logic [3:0]               k,
    output logic [2:0]               stride,
    output logic [5:0]               cin_slices,
    output logic [5:0]               cout_slices,
    output logic [5:0]               tile_w,
    output logic [7:0]               num_tiles,
    output logic [5:0]               last_valid_w,
    output logic [9:0]               total_wrf,
    output logic                     wrf_packed,
    output logic [9:0]               kk,
    output logic [2:0]               rounds_per_cins,
    output logic [5:0]               round_len_last,
    output logic [CORE_ADDR_W-1:0]   ifb_base,
    output logic [CORE_ADDR_W-1:0]   wb_base,
    output logic [CORE_ADDR_W-1:0]   ofb_base,
    output logic [CORE_ADDR_W-1:0]   ifb_cin_step,
    output logic [CORE_ADDR_W-1:0]   ifb_row_step,
    output logic [CORE_ADDR_W-1:0]   wb_cin_step,
    output logic [CORE_ADDR_W-1:0]   wb_cout_step,
    output logic [CORE_ADDR_W-1:0]   ofb_cout_step,
    output logic [CORE_ADDR_W-1:0]   tile_in_step,
    output logic [5:0]               sdp_shift,
    output logic                     sdp_relu_en,

    // ---- Phase C: descriptor fetch + new SDP fields ----
    output logic [31:0]              desc_list_base,
    output logic [15:0]              desc_count,
    output logic signed [31:0]       sdp_mult,
    output logic signed [8:0]        sdp_zp_out,
    output logic signed [8:0]        sdp_clip_min,
    output logic signed [8:0]        sdp_clip_max,
    output logic                     sdp_round_en,
    output logic [3:0]               pad_top,        // Phase C-2: 老路径 pad 验证
    output logic [3:0]               pad_left,

    // ---- Streaming / ring 配置输出 ----
    output logic [15:0]              h_in_total,
    output logic [7:0]               ifb_strip_rows,
    output logic [5:0]               ofb_strip_rows,
    output logic [CORE_ADDR_W-1:0]   ddr_ifm_row_stride,
    output logic [CORE_ADDR_W-1:0]   ddr_ofm_row_stride,
    output logic                     idma_streaming,
    output logic                     odma_streaming,

    // ---- DMA 描述符输出 ----
    output logic [31:0]              idma_src_base,
    output logic [23:0]              idma_byte_len,
    output logic [31:0]              wdma_src_base,
    output logic [23:0]              wdma_byte_len,
    output logic [31:0]              odma_dst_base,
    output logic [23:0]              odma_byte_len
);

    // =========================================================================
    // 地址常量
    // =========================================================================
    localparam [ADDR_W-1:0] ADDR_CTRL             = 12'h000;
    localparam [ADDR_W-1:0] ADDR_STATUS           = 12'h004;

    localparam [ADDR_W-1:0] ADDR_H_OUT            = 12'h100;
    localparam [ADDR_W-1:0] ADDR_W_OUT            = 12'h104;
    localparam [ADDR_W-1:0] ADDR_W_IN             = 12'h108;
    localparam [ADDR_W-1:0] ADDR_K                = 12'h10C;
    localparam [ADDR_W-1:0] ADDR_STRIDE           = 12'h110;
    localparam [ADDR_W-1:0] ADDR_CIN_SLICES       = 12'h114;
    localparam [ADDR_W-1:0] ADDR_COUT_SLICES      = 12'h118;
    localparam [ADDR_W-1:0] ADDR_TILE_W           = 12'h11C;
    localparam [ADDR_W-1:0] ADDR_NUM_TILES        = 12'h120;
    localparam [ADDR_W-1:0] ADDR_LAST_VALID_W     = 12'h124;
    localparam [ADDR_W-1:0] ADDR_TOTAL_WRF        = 12'h128;
    localparam [ADDR_W-1:0] ADDR_WRF_PACKED       = 12'h12C;
    localparam [ADDR_W-1:0] ADDR_KK               = 12'h130;
    localparam [ADDR_W-1:0] ADDR_ROUNDS_PER_CINS  = 12'h134;
    localparam [ADDR_W-1:0] ADDR_ROUND_LEN_LAST   = 12'h138;
    localparam [ADDR_W-1:0] ADDR_IFB_BASE         = 12'h13C;
    localparam [ADDR_W-1:0] ADDR_WB_BASE          = 12'h140;
    localparam [ADDR_W-1:0] ADDR_OFB_BASE         = 12'h144;
    localparam [ADDR_W-1:0] ADDR_IFB_CIN_STEP     = 12'h148;
    localparam [ADDR_W-1:0] ADDR_IFB_ROW_STEP     = 12'h14C;
    localparam [ADDR_W-1:0] ADDR_WB_CIN_STEP      = 12'h150;
    localparam [ADDR_W-1:0] ADDR_WB_COUT_STEP     = 12'h154;
    localparam [ADDR_W-1:0] ADDR_OFB_COUT_STEP    = 12'h158;
    localparam [ADDR_W-1:0] ADDR_TILE_IN_STEP     = 12'h15C;
    localparam [ADDR_W-1:0] ADDR_SDP_SHIFT        = 12'h160;
    localparam [ADDR_W-1:0] ADDR_SDP_RELU_EN      = 12'h164;

    localparam [ADDR_W-1:0] ADDR_H_IN_TOTAL       = 12'h168;
    localparam [ADDR_W-1:0] ADDR_IFB_STRIP_ROWS   = 12'h16C;
    localparam [ADDR_W-1:0] ADDR_OFB_STRIP_ROWS   = 12'h170;
    localparam [ADDR_W-1:0] ADDR_DDR_IFM_ROW_STR  = 12'h174;
    localparam [ADDR_W-1:0] ADDR_DDR_OFM_ROW_STR  = 12'h178;
    localparam [ADDR_W-1:0] ADDR_DMA_MODE         = 12'h17C;

    localparam [ADDR_W-1:0] ADDR_DESC_LIST_BASE   = 12'h180;
    localparam [ADDR_W-1:0] ADDR_DESC_COUNT       = 12'h184;
    localparam [ADDR_W-1:0] ADDR_SDP_MULT         = 12'h188;
    localparam [ADDR_W-1:0] ADDR_SDP_ZP_OUT       = 12'h18C;
    localparam [ADDR_W-1:0] ADDR_SDP_CLIP_MIN     = 12'h190;
    localparam [ADDR_W-1:0] ADDR_SDP_CLIP_MAX     = 12'h194;
    localparam [ADDR_W-1:0] ADDR_SDP_ROUND_EN     = 12'h198;
    localparam [ADDR_W-1:0] ADDR_PAD_TOP          = 12'h1A0;
    localparam [ADDR_W-1:0] ADDR_PAD_LEFT         = 12'h1A4;

    localparam [ADDR_W-1:0] ADDR_IDMA_SRC_BASE    = 12'h200;
    localparam [ADDR_W-1:0] ADDR_IDMA_BYTE_LEN    = 12'h204;
    localparam [ADDR_W-1:0] ADDR_WDMA_SRC_BASE    = 12'h210;
    localparam [ADDR_W-1:0] ADDR_WDMA_BYTE_LEN    = 12'h214;
    localparam [ADDR_W-1:0] ADDR_ODMA_DST_BASE    = 12'h220;
    localparam [ADDR_W-1:0] ADDR_ODMA_BYTE_LEN    = 12'h224;

    // =========================================================================
    // start_*_pulse 生成：CTRL 写 1 → 当拍 pulse（reg_w_en 本身就是 1-拍脉冲）
    // =========================================================================
    assign start_core_pulse  = reg_w_en && (reg_w_addr == ADDR_CTRL) && reg_w_data[0];
    assign start_idma_pulse  = reg_w_en && (reg_w_addr == ADDR_CTRL) && reg_w_data[1];
    assign start_wdma_pulse  = reg_w_en && (reg_w_addr == ADDR_CTRL) && reg_w_data[2];
    assign start_odma_pulse  = reg_w_en && (reg_w_addr == ADDR_CTRL) && reg_w_data[3];
    assign start_dfe_pulse   = reg_w_en && (reg_w_addr == ADDR_CTRL) && reg_w_data[4];
    assign start_layer_pulse = reg_w_en && (reg_w_addr == ADDR_CTRL) && reg_w_data[5];

    // =========================================================================
    // core_busy：start_core_pulse 置位，core_done 清零（控制路径，复位必须）
    // =========================================================================
    logic r_core_busy;
    always_ff @(posedge clk) begin
        if      (!rst_n)             r_core_busy <= 1'b0;
        else if (start_core_pulse)   r_core_busy <= 1'b1;
        else if (core_done)          r_core_busy <= 1'b0;
        else                         r_core_busy <= r_core_busy;
    end

    // DMA_MODE 是控制路径，必须复位（§6.1）。从 bank always_ff 里拉出来单独维护。
    // 默认 0 → batch 模式，兼容 v1 所有测试。
    logic [1:0] r_dma_mode_ctrl;
    always_ff @(posedge clk) begin
        if      (!rst_n)                                    r_dma_mode_ctrl <= 2'b00;
        else if (reg_w_en && reg_w_addr == ADDR_DMA_MODE)   r_dma_mode_ctrl <= reg_w_data[1:0];
        else                                                r_dma_mode_ctrl <= r_dma_mode_ctrl;
    end

    // =========================================================================
    // 配置 / DMA 寄存器 —— 每寄存器独立 always_ff，数据路径无复位（§6）
    // =========================================================================
    logic [15:0]             r_h_out;
    logic [15:0]             r_w_out;
    logic [15:0]             r_w_in;
    logic [3:0]              r_k;
    logic [2:0]              r_stride;
    logic [5:0]              r_cin_slices;
    logic [5:0]              r_cout_slices;
    logic [5:0]              r_tile_w;
    logic [7:0]              r_num_tiles;
    logic [5:0]              r_last_valid_w;
    logic [9:0]              r_total_wrf;
    logic                    r_wrf_packed;
    logic [9:0]              r_kk;
    logic [2:0]              r_rounds_per_cins;
    logic [5:0]              r_round_len_last;
    logic [CORE_ADDR_W-1:0]  r_ifb_base;
    logic [CORE_ADDR_W-1:0]  r_wb_base;
    logic [CORE_ADDR_W-1:0]  r_ofb_base;
    logic [CORE_ADDR_W-1:0]  r_ifb_cin_step;
    logic [CORE_ADDR_W-1:0]  r_ifb_row_step;
    logic [CORE_ADDR_W-1:0]  r_wb_cin_step;
    logic [CORE_ADDR_W-1:0]  r_wb_cout_step;
    logic [CORE_ADDR_W-1:0]  r_ofb_cout_step;
    logic [CORE_ADDR_W-1:0]  r_tile_in_step;
    logic [5:0]              r_sdp_shift;
    logic                    r_sdp_relu_en;
    logic [31:0]             r_desc_list_base;
    logic [15:0]             r_desc_count;
    logic signed [31:0]      r_sdp_mult;
    logic signed [8:0]       r_sdp_zp_out;
    logic signed [8:0]       r_sdp_clip_min;
    logic signed [8:0]       r_sdp_clip_max;
    logic                    r_sdp_round_en;
    logic [3:0]              r_pad_top;
    logic [3:0]              r_pad_left;
    logic [15:0]             r_h_in_total;
    logic [7:0]              r_ifb_strip_rows;
    logic [5:0]              r_ofb_strip_rows;
    logic [CORE_ADDR_W-1:0]  r_ddr_ifm_row_stride;
    logic [CORE_ADDR_W-1:0]  r_ddr_ofm_row_stride;
    // r_dma_mode_ctrl 声明在上面（有复位）
    logic [31:0]             r_idma_src_base;
    logic [23:0]             r_idma_byte_len;
    logic [31:0]             r_wdma_src_base;
    logic [23:0]             r_wdma_byte_len;
    logic [31:0]             r_odma_dst_base;
    logic [23:0]             r_odma_byte_len;

    // 寄存器 bank 写入（§4.1 例外 2：共享 reg_w_en 门控 + addr 解码，
    // 所有寄存器写在同一 always_ff 内，不计 4-reg 限制；未命中时所有寄存器
    // 按 flop 语义隐式保持）。
    always_ff @(posedge clk) begin
        if (reg_w_en) begin
            case (reg_w_addr)
                ADDR_H_OUT           : r_h_out           <= reg_w_data[15:0];
                ADDR_W_OUT           : r_w_out           <= reg_w_data[15:0];
                ADDR_W_IN            : r_w_in            <= reg_w_data[15:0];
                ADDR_K               : r_k               <= reg_w_data[3:0];
                ADDR_STRIDE          : r_stride          <= reg_w_data[2:0];
                ADDR_CIN_SLICES      : r_cin_slices      <= reg_w_data[5:0];
                ADDR_COUT_SLICES     : r_cout_slices     <= reg_w_data[5:0];
                ADDR_TILE_W          : r_tile_w          <= reg_w_data[5:0];
                ADDR_NUM_TILES       : r_num_tiles       <= reg_w_data[7:0];
                ADDR_LAST_VALID_W    : r_last_valid_w    <= reg_w_data[5:0];
                ADDR_TOTAL_WRF       : r_total_wrf       <= reg_w_data[9:0];
                ADDR_WRF_PACKED      : r_wrf_packed      <= reg_w_data[0];
                ADDR_KK              : r_kk              <= reg_w_data[9:0];
                ADDR_ROUNDS_PER_CINS : r_rounds_per_cins <= reg_w_data[2:0];
                ADDR_ROUND_LEN_LAST  : r_round_len_last  <= reg_w_data[5:0];
                ADDR_IFB_BASE        : r_ifb_base        <= reg_w_data[CORE_ADDR_W-1:0];
                ADDR_WB_BASE         : r_wb_base         <= reg_w_data[CORE_ADDR_W-1:0];
                ADDR_OFB_BASE        : r_ofb_base        <= reg_w_data[CORE_ADDR_W-1:0];
                ADDR_IFB_CIN_STEP    : r_ifb_cin_step    <= reg_w_data[CORE_ADDR_W-1:0];
                ADDR_IFB_ROW_STEP    : r_ifb_row_step    <= reg_w_data[CORE_ADDR_W-1:0];
                ADDR_WB_CIN_STEP     : r_wb_cin_step     <= reg_w_data[CORE_ADDR_W-1:0];
                ADDR_WB_COUT_STEP    : r_wb_cout_step    <= reg_w_data[CORE_ADDR_W-1:0];
                ADDR_OFB_COUT_STEP   : r_ofb_cout_step   <= reg_w_data[CORE_ADDR_W-1:0];
                ADDR_TILE_IN_STEP    : r_tile_in_step    <= reg_w_data[CORE_ADDR_W-1:0];
                ADDR_SDP_SHIFT       : r_sdp_shift       <= reg_w_data[5:0];
                ADDR_SDP_RELU_EN     : r_sdp_relu_en     <= reg_w_data[0];
                ADDR_DESC_LIST_BASE  : r_desc_list_base  <= reg_w_data[31:0];
                ADDR_DESC_COUNT      : r_desc_count      <= reg_w_data[15:0];
                ADDR_SDP_MULT        : r_sdp_mult        <= $signed(reg_w_data[31:0]);
                ADDR_SDP_ZP_OUT      : r_sdp_zp_out      <= $signed(reg_w_data[8:0]);
                ADDR_SDP_CLIP_MIN    : r_sdp_clip_min    <= $signed(reg_w_data[8:0]);
                ADDR_SDP_CLIP_MAX    : r_sdp_clip_max    <= $signed(reg_w_data[8:0]);
                ADDR_SDP_ROUND_EN    : r_sdp_round_en    <= reg_w_data[0];
                ADDR_PAD_TOP         : r_pad_top         <= reg_w_data[3:0];
                ADDR_PAD_LEFT        : r_pad_left        <= reg_w_data[3:0];
                ADDR_H_IN_TOTAL      : r_h_in_total      <= reg_w_data[15:0];
                ADDR_IFB_STRIP_ROWS  : r_ifb_strip_rows  <= reg_w_data[7:0];
                ADDR_OFB_STRIP_ROWS  : r_ofb_strip_rows  <= reg_w_data[5:0];
                ADDR_DDR_IFM_ROW_STR : r_ddr_ifm_row_stride <= reg_w_data[CORE_ADDR_W-1:0];
                ADDR_DDR_OFM_ROW_STR : r_ddr_ofm_row_stride <= reg_w_data[CORE_ADDR_W-1:0];
                // ADDR_DMA_MODE 已单独 r_dma_mode_ctrl 维护（含复位），此处不需要
                ADDR_IDMA_SRC_BASE   : r_idma_src_base   <= reg_w_data[31:0];
                ADDR_IDMA_BYTE_LEN   : r_idma_byte_len   <= reg_w_data[23:0];
                ADDR_WDMA_SRC_BASE   : r_wdma_src_base   <= reg_w_data[31:0];
                ADDR_WDMA_BYTE_LEN   : r_wdma_byte_len   <= reg_w_data[23:0];
                ADDR_ODMA_DST_BASE   : r_odma_dst_base   <= reg_w_data[31:0];
                ADDR_ODMA_BYTE_LEN   : r_odma_byte_len   <= reg_w_data[23:0];
                default              : ;   // CTRL / STATUS / 未使用地址：不写入 bank
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
    assign stride          = r_stride;
    assign cin_slices      = r_cin_slices;
    assign cout_slices     = r_cout_slices;
    assign tile_w          = r_tile_w;
    assign num_tiles       = r_num_tiles;
    assign last_valid_w    = r_last_valid_w;
    assign total_wrf       = r_total_wrf;
    assign wrf_packed      = r_wrf_packed;
    assign kk              = r_kk;
    assign rounds_per_cins = r_rounds_per_cins;
    assign round_len_last  = r_round_len_last;
    assign ifb_base        = r_ifb_base;
    assign wb_base         = r_wb_base;
    assign ofb_base        = r_ofb_base;
    assign ifb_cin_step    = r_ifb_cin_step;
    assign ifb_row_step    = r_ifb_row_step;
    assign wb_cin_step     = r_wb_cin_step;
    assign wb_cout_step    = r_wb_cout_step;
    assign ofb_cout_step   = r_ofb_cout_step;
    assign tile_in_step    = r_tile_in_step;
    assign sdp_shift       = r_sdp_shift;
    assign sdp_relu_en     = r_sdp_relu_en;
    assign desc_list_base  = r_desc_list_base;
    assign desc_count      = r_desc_count;
    assign sdp_mult        = r_sdp_mult;
    assign sdp_zp_out      = r_sdp_zp_out;
    assign sdp_clip_min    = r_sdp_clip_min;
    assign sdp_clip_max    = r_sdp_clip_max;
    assign sdp_round_en    = r_sdp_round_en;
    assign pad_top         = r_pad_top;
    assign pad_left        = r_pad_left;
    assign h_in_total         = r_h_in_total;
    assign ifb_strip_rows     = r_ifb_strip_rows;
    assign ofb_strip_rows     = r_ofb_strip_rows;
    assign ddr_ifm_row_stride = r_ddr_ifm_row_stride;
    assign ddr_ofm_row_stride = r_ddr_ofm_row_stride;
    assign idma_streaming     = r_dma_mode_ctrl[0];
    assign odma_streaming     = r_dma_mode_ctrl[1];
    assign idma_src_base   = r_idma_src_base;
    assign idma_byte_len   = r_idma_byte_len;
    assign wdma_src_base   = r_wdma_src_base;
    assign wdma_byte_len   = r_wdma_byte_len;
    assign odma_dst_base   = r_odma_dst_base;
    assign odma_byte_len   = r_odma_byte_len;

    // =========================================================================
    // 读 mux：按 reg_r_addr 选择返回数据（组合）
    // =========================================================================
    logic [DATA_W-1:0] status_word;
    assign status_word = {20'd0,
                          layer_done, layer_busy, dfe_done, dfe_busy,
                          odma_done,  wdma_done,  idma_done,
                          odma_busy,  wdma_busy,  idma_busy,
                          r_core_busy, core_done};

    always_comb begin
        case (reg_r_addr)
            ADDR_CTRL            : reg_r_data = '0;
            ADDR_STATUS          : reg_r_data = status_word;
            ADDR_H_OUT           : reg_r_data = {16'd0, r_h_out};
            ADDR_W_OUT           : reg_r_data = {16'd0, r_w_out};
            ADDR_W_IN            : reg_r_data = {16'd0, r_w_in};
            ADDR_K               : reg_r_data = {28'd0, r_k};
            ADDR_STRIDE          : reg_r_data = {29'd0, r_stride};
            ADDR_CIN_SLICES      : reg_r_data = {26'd0, r_cin_slices};
            ADDR_COUT_SLICES     : reg_r_data = {26'd0, r_cout_slices};
            ADDR_TILE_W          : reg_r_data = {26'd0, r_tile_w};
            ADDR_NUM_TILES       : reg_r_data = {24'd0, r_num_tiles};
            ADDR_LAST_VALID_W    : reg_r_data = {26'd0, r_last_valid_w};
            ADDR_TOTAL_WRF       : reg_r_data = {22'd0, r_total_wrf};
            ADDR_WRF_PACKED      : reg_r_data = {31'd0, r_wrf_packed};
            ADDR_KK              : reg_r_data = {22'd0, r_kk};
            ADDR_ROUNDS_PER_CINS : reg_r_data = {29'd0, r_rounds_per_cins};
            ADDR_ROUND_LEN_LAST  : reg_r_data = {26'd0, r_round_len_last};
            ADDR_IFB_BASE        : reg_r_data = {12'd0, r_ifb_base};
            ADDR_WB_BASE         : reg_r_data = {12'd0, r_wb_base};
            ADDR_OFB_BASE        : reg_r_data = {12'd0, r_ofb_base};
            ADDR_IFB_CIN_STEP    : reg_r_data = {12'd0, r_ifb_cin_step};
            ADDR_IFB_ROW_STEP    : reg_r_data = {12'd0, r_ifb_row_step};
            ADDR_WB_CIN_STEP     : reg_r_data = {12'd0, r_wb_cin_step};
            ADDR_WB_COUT_STEP    : reg_r_data = {12'd0, r_wb_cout_step};
            ADDR_OFB_COUT_STEP   : reg_r_data = {12'd0, r_ofb_cout_step};
            ADDR_TILE_IN_STEP    : reg_r_data = {12'd0, r_tile_in_step};
            ADDR_SDP_SHIFT       : reg_r_data = {26'd0, r_sdp_shift};
            ADDR_SDP_RELU_EN     : reg_r_data = {31'd0, r_sdp_relu_en};
            ADDR_DESC_LIST_BASE  : reg_r_data = r_desc_list_base;
            ADDR_DESC_COUNT      : reg_r_data = {16'd0, r_desc_count};
            ADDR_SDP_MULT        : reg_r_data = $unsigned(r_sdp_mult);
            ADDR_SDP_ZP_OUT      : reg_r_data = {{23{r_sdp_zp_out[8]}}, r_sdp_zp_out};
            ADDR_SDP_CLIP_MIN    : reg_r_data = {{23{r_sdp_clip_min[8]}}, r_sdp_clip_min};
            ADDR_SDP_CLIP_MAX    : reg_r_data = {{23{r_sdp_clip_max[8]}}, r_sdp_clip_max};
            ADDR_SDP_ROUND_EN    : reg_r_data = {31'd0, r_sdp_round_en};
            ADDR_PAD_TOP         : reg_r_data = {28'd0, r_pad_top};
            ADDR_PAD_LEFT        : reg_r_data = {28'd0, r_pad_left};
            ADDR_H_IN_TOTAL      : reg_r_data = {16'd0, r_h_in_total};
            ADDR_IFB_STRIP_ROWS  : reg_r_data = {24'd0, r_ifb_strip_rows};
            ADDR_OFB_STRIP_ROWS  : reg_r_data = {26'd0, r_ofb_strip_rows};
            ADDR_DDR_IFM_ROW_STR : reg_r_data = {12'd0, r_ddr_ifm_row_stride};
            ADDR_DDR_OFM_ROW_STR : reg_r_data = {12'd0, r_ddr_ofm_row_stride};
            ADDR_DMA_MODE        : reg_r_data = {30'd0, r_dma_mode_ctrl};
            ADDR_IDMA_SRC_BASE   : reg_r_data = r_idma_src_base;
            ADDR_IDMA_BYTE_LEN   : reg_r_data = {8'd0, r_idma_byte_len};
            ADDR_WDMA_SRC_BASE   : reg_r_data = r_wdma_src_base;
            ADDR_WDMA_BYTE_LEN   : reg_r_data = {8'd0, r_wdma_byte_len};
            ADDR_ODMA_DST_BASE   : reg_r_data = r_odma_dst_base;
            ADDR_ODMA_BYTE_LEN   : reg_r_data = {8'd0, r_odma_byte_len};
            default              : reg_r_data = '0;
        endcase
    end

endmodule
