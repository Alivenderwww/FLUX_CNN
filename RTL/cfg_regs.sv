`timescale 1ns/1ps

// =============================================================================
// cfg_regs.sv  --  Shared Configuration + DMA Descriptor Register Bank
//
// 通过 AXI-Lite decoded 端口（reg_w_en/addr/data + reg_r_addr/data）接受外部
// 读写。Bridge 在 axi_lite_csr.sv 里；本模块只做 address decode + 寄存器组。
//
// Register map（4 字节对齐）：
//   0x000  CTRL              [0]=start (自动清零，写 1 一拍 start_pulse)
//                             [1]=soft_rst (保留)
//   0x004  STATUS  (RO)      [0]=core_done     [1]=core_busy
//                             [2]=idma_busy    [3]=wdma_busy [4]=odma_busy
//                             [5]=idma_done    [6]=wdma_done [7]=odma_done
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
//   0x160  SDP_SHIFT         [4:0]
//   0x164  SDP_RELU_EN       [0]
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

    // ---- CTRL 输出 ----
    output logic                     start_pulse,    // 写 CTRL.start 时的 1 拍脉冲

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
    output logic [4:0]               sdp_shift,
    output logic                     sdp_relu_en,

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

    localparam [ADDR_W-1:0] ADDR_IDMA_SRC_BASE    = 12'h200;
    localparam [ADDR_W-1:0] ADDR_IDMA_BYTE_LEN    = 12'h204;
    localparam [ADDR_W-1:0] ADDR_WDMA_SRC_BASE    = 12'h210;
    localparam [ADDR_W-1:0] ADDR_WDMA_BYTE_LEN    = 12'h214;
    localparam [ADDR_W-1:0] ADDR_ODMA_DST_BASE    = 12'h220;
    localparam [ADDR_W-1:0] ADDR_ODMA_BYTE_LEN    = 12'h224;

    // =========================================================================
    // start_pulse 生成：CTRL.start 写 1 → 当拍 pulse（reg_w_en 本身就是 1-拍脉冲）
    // =========================================================================
    assign start_pulse = reg_w_en && (reg_w_addr == ADDR_CTRL) && reg_w_data[0];

    // =========================================================================
    // core_busy：start_pulse 置位，core_done 清零（控制路径，复位必须）
    // =========================================================================
    logic r_core_busy;
    always_ff @(posedge clk) begin
        if      (!rst_n)      r_core_busy <= 1'b0;
        else if (start_pulse) r_core_busy <= 1'b1;
        else if (core_done)   r_core_busy <= 1'b0;
        else                  r_core_busy <= r_core_busy;
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
    logic [4:0]              r_sdp_shift;
    logic                    r_sdp_relu_en;
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
                ADDR_SDP_SHIFT       : r_sdp_shift       <= reg_w_data[4:0];
                ADDR_SDP_RELU_EN     : r_sdp_relu_en     <= reg_w_data[0];
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
    assign status_word = {24'd0,
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
            ADDR_SDP_SHIFT       : reg_r_data = {27'd0, r_sdp_shift};
            ADDR_SDP_RELU_EN     : reg_r_data = {31'd0, r_sdp_relu_en};
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
