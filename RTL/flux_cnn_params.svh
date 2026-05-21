// =============================================================================
// flux_cnn_params.svh  --  自动生成, 不要手改
// 来源: hardware/*.json + toolchain/hardware/codegen.py
// 改硬件参数: 改 hardware/vd100.json 后跑 `python -m toolchain.hardware.codegen`
// =============================================================================

`ifndef FLUX_CNN_PARAMS_SVH
`define FLUX_CNN_PARAMS_SVH
// ---- Datapath ----
`define FLUX_NUM_PE                   16  // PE per column (= IFB cin byte 数)
`define FLUX_NUM_COL                  16  // mac_array 列数 (= cout 并行度)
`define FLUX_DATA_WIDTH               8  // INT8
`define FLUX_PSUM_WIDTH               32  // INT32 累加
`define FLUX_WRF_DEPTH                32  // 每 PE 内权重 RF 深度
`define FLUX_ARF_DEPTH                32  // line_buffer 内 act ring depth
`define FLUX_PARF_DEPTH               32  // parf_accum partial sum depth

// ---- SRAM 容量 ----
`define FLUX_IFB_DEPTH                1024  // 128 KB (1 word = 128 bit)
`define FLUX_WB_DEPTH                 640  // 256 KB (1 word = 2048 bit)
`define FLUX_OFB_DEPTH                1024  // 32 KB (1 word = 128 bit)
`define FLUX_SHORTCUT_DEPTH           2048  // 128 KB Shortcut Bank

// ---- AXI / CSR ----
`define FLUX_BUS_ADDR_W               32
`define FLUX_BUS_DATA_W               128
`define FLUX_AXI_M_ID                 2  // per-master ID 宽
`define FLUX_AXI_M_WIDTH              2  // log2(masters/core)
`define FLUX_DMA_LEN_W                24
`define FLUX_CSR_ADDR_W               12
`define FLUX_CSR_DATA_W               32
`define FLUX_CORE_BUS_ID              4  // = AXI_M_ID + AXI_M_WIDTH

// ---- 全局地址映射 (多核) ----
`define FLUX_DDR_BASE                 0
`define FLUX_DDR_ADDR_WIDTH           31
`define FLUX_CORE_IFB_BASE            32'h80000000
`define FLUX_CORE_IFB_ADDR_WIDTH      28
`define FLUX_CORE_IFB_STRIDE          32'h10000000

// ---- CSR address map (cfg_regs reg_addr) ----
`define FLUX_ADDR_CTRL                12'h000
`define FLUX_ADDR_STATUS              12'h004
`define FLUX_ADDR_DMA_MODE            12'h17C
`define FLUX_ADDR_DESC_LIST_BASE      12'h180
`define FLUX_ADDR_DESC_COUNT          12'h184
`define FLUX_ADDR_H_OUT               12'h100
`define FLUX_ADDR_W_OUT               12'h104
`define FLUX_ADDR_W_IN                12'h108
`define FLUX_ADDR_K                   12'h10C
`define FLUX_ADDR_STRIDE              12'h110
`define FLUX_ADDR_CIN_SLICES          12'h114
`define FLUX_ADDR_COUT_SLICES         12'h118
`define FLUX_ADDR_TILE_W              12'h11C
`define FLUX_ADDR_NUM_TILES           12'h120
`define FLUX_ADDR_LAST_VALID_W        12'h124
`define FLUX_ADDR_TOTAL_WRF           12'h128
`define FLUX_ADDR_KY                  12'h12C
`define FLUX_ADDR_KK                  12'h130
`define FLUX_ADDR_ROUNDS_PER_CINS     12'h134
`define FLUX_ADDR_ROUND_LEN_LAST      12'h138
`define FLUX_ADDR_IFB_BASE            12'h13C
`define FLUX_ADDR_WB_BASE             12'h140
`define FLUX_ADDR_OFB_BASE            12'h144
`define FLUX_ADDR_IFB_ROW_STEP        12'h14C
`define FLUX_ADDR_WB_COUT_STEP        12'h154
`define FLUX_ADDR_TILE_IN_STEP        12'h15C
`define FLUX_ADDR_SDP_SHIFT           12'h160
`define FLUX_ADDR_SDP_RELU_EN         12'h164
`define FLUX_ADDR_H_IN_TOTAL          12'h168
`define FLUX_ADDR_IFB_STRIP_ROWS      12'h16C
`define FLUX_ADDR_OFB_STRIP_ROWS      12'h170
`define FLUX_ADDR_DDR_IFM_ROW_STRIDE  12'h174
`define FLUX_ADDR_DDR_OFM_ROW_STRIDE  12'h178
`define FLUX_ADDR_SDP_MULT            12'h188
`define FLUX_ADDR_SDP_ZP_OUT          12'h18C
`define FLUX_ADDR_SDP_CLIP_MIN        12'h190
`define FLUX_ADDR_SDP_CLIP_MAX        12'h194
`define FLUX_ADDR_SDP_ROUND_EN        12'h198
`define FLUX_ADDR_IFB_RING_WORDS      12'h1A0
`define FLUX_ADDR_OFB_ROW_WORDS       12'h1A4
`define FLUX_ADDR_OFB_RING_WORDS      12'h1A8
`define FLUX_ADDR_IFB_ISS_STEP        12'h1AC
`define FLUX_ADDR_IFB_KY_STEP         12'h1B0
`define FLUX_ADDR_TILE_PIX_STEP       12'h1B4
`define FLUX_ADDR_ARF_REUSE_EN        12'h1B8
`define FLUX_ADDR_RESIDUAL_EN         12'h1BC
`define FLUX_ADDR_SHORTCUT_MULT       12'h1C0
`define FLUX_ADDR_SHORTCUT_SHIFT      12'h1C4
`define FLUX_ADDR_BIAS_BASE           12'h1C8
`define FLUX_ADDR_SKIP_IDMA           12'h1CC
`define FLUX_ADDR_OFM_TDEST           12'h1D0
`define FLUX_ADDR_OFM_OPCODE          12'h1D4
`define FLUX_ADDR_IDMA_CMD_LIST_PTR   12'h1D8
`define FLUX_ADDR_IDMA_CMD_COUNT      12'h1DC
`define FLUX_ADDR_IDMA_CMDS_PER_ROW   12'h1E0
`define FLUX_ADDR_STRIDE_H            12'h1F0
`define FLUX_ADDR_ODMA_CMD_LIST_PTR   12'h1E4
`define FLUX_ADDR_ODMA_CMD_COUNT      12'h1E8
`define FLUX_ADDR_ODMA_CMDS_PER_ROW   12'h1EC
`define FLUX_ADDR_IDMA_SRC_BASE       12'h200
`define FLUX_ADDR_IDMA_BYTE_LEN       12'h204
`define FLUX_ADDR_WDMA_SRC_BASE       12'h210
`define FLUX_ADDR_WDMA_BYTE_LEN       12'h214
`define FLUX_ADDR_ODMA_DST_BASE       12'h220
`define FLUX_ADDR_ODMA_BYTE_LEN       12'h224
`define FLUX_ADDR_RDMA_SRC_BASE       12'h230
`define FLUX_ADDR_RDMA_BYTE_LEN       12'h234


`endif // FLUX_CNN_PARAMS_SVH
