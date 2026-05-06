#!/usr/bin/env python3
"""
flux_cnn params  --  唯一参数表 (Single Source of Truth)

RTL 和 Python toolchain 都从这里取参数, 避免双份维护.

用法:
  - Python:  from params import NUM_PE, IFB_DEPTH, ...
             from params import CSR_ADDR_MAP, core_ifb_axi_base
  - RTL:     `include "flux_cnn_params.svh"  (自动生成)
             跑 `python params.py` 重新生成 svh

约定: 改完 params.py **必须** 重跑 codegen 同步 svh, 否则 RTL/Python 不一致.
建议放进 git pre-commit hook 或 build 流程里.
"""

import os
import sys
from pathlib import Path

# =============================================================================
# 1. 核内 datapath 尺寸
# =============================================================================
NUM_PE      = 16        # 一列 PE 数 (= IFB SRAM 一行 cin byte 数)
NUM_COL     = 16        # mac_array 列数 (= cout 并行度)
DATA_WIDTH  = 8         # INT8
PSUM_WIDTH  = 32        # INT32 累加
WRF_DEPTH   = 32        # 每 PE 内权重 RF 深度
ARF_DEPTH   = 32        # line_buffer 内 act ring depth
PARF_DEPTH  = 32        # parf_accum 内 partial sum depth

# =============================================================================
# 2. SRAM 容量 (word, 1 word = 不同 SRAM 不同位宽, 见注释)
# =============================================================================
# -- 路径 A 修剪 (2026-05-05): xc7k325t N=4 BRAM 适配 --
#   ResNet11 N=4 SMC 实际需求 (per core):
#     IFB  ring max = 396 words (Layer 0 Patch s2d sub_W=33 cs=4, K+3 strip).
#     WB   max      = 528 words (FC layer 256×522 ÷ 256 cells per word = 528).
#     OFB  ring max = 264 words (Layer 0 sub_W=33 ofb_strip=8).
#     SHB  sliced max = 2044 words (Layer 3 ds residual sliced shortcut).
#   旧值 (8192/1024/2048/8192) 4 核合计 484 BRAM36 超 xc7k325t (445).
#   修剪后 (1024/640/1024/2560) 4 核估 ~232 BRAM36 = 52% util.
#
# 2026-05-05 IFB 1024 必须配合 line_buffer.sv rows_consumed_raw fix:
#   yout_wrap 不再 reset rows_consumed (跨 strip 累计才能跟 dispatcher 同步).
#   未修前 ResNet11 layer 3 (K=1 stride=2 cross-mem chunk) 死锁.
IFB_DEPTH       = 2048  # 8192→2048 省 24 BRAM36/core. RTL SHB 跟 IFB 共用 SRAM_DEPTH,
                        # 必须 ≥ max(IFB ring 396, SHB segment 2044) → 取 2048 (= 2^11)
                        # IFB 实际 ring 模式只需 ≤ 396 word, SHB 整段保活需 ≤ 2044 word
WB_DEPTH        = 640   # 1024→640 (distributed RAM, 不省 BRAM 仅省 LUTRAM)
OFB_DEPTH       = 1024  # 2048→1024 (LUTRAM, 仅省 LUTRAM)
SHORTCUT_DEPTH  = 2048  # 跟 IFB_DEPTH 对齐 (RTL 共用 SRAM_DEPTH)

# 兼容: RTL 不少地方用 SRAM_DEPTH 默认指代 IFB
SRAM_DEPTH = IFB_DEPTH

# 派生: word size in bytes
IFB_WORD_BYTES      = NUM_PE  * DATA_WIDTH // 8           # 16
WB_WORD_BYTES       = NUM_COL * NUM_PE * DATA_WIDTH // 8  # 256
OFB_WORD_BYTES      = NUM_COL * DATA_WIDTH // 8           # 16
SHORTCUT_WORD_BYTES = OFB_WORD_BYTES                       # 16 (跟 OFB 共享 layout)

# =============================================================================
# 3. AXI / CSR 接口
# =============================================================================
BUS_ADDR_W   = 32
BUS_DATA_W   = 128
AXI_M_ID     = 2          # 每核内 master per-master ID 宽 (axi_dm + DFE 等共享)
AXI_M_WIDTH  = 2          # log2(masters per core) = 4 个 master
DMA_LEN_W    = 24         # IDMA/ODMA byte_len 宽度
CSR_ADDR_W   = 12         # cfg_regs 寻址空间 = 4 KB
CSR_DATA_W   = 32

# 派生
CORE_BUS_ID  = AXI_M_ID + AXI_M_WIDTH    # 4: 核内 bus_*id 位宽

# =============================================================================
# 4. 全局地址映射 (多核 wrapper 视角)
#    - DDR:        0x0000_0000 - 0x7FFF_FFFF  (31 bit, 2 GB)
#    - Core[i] IFB: 0x8000_0000 + i × 0x1000_0000  (28 bit per region, 256 MB/core)
# =============================================================================
DDR_BASE                = 0x00000000
DDR_ADDR_WIDTH          = 31           # bit, 2 GB
CORE_IFB_BASE           = 0x80000000
CORE_IFB_ADDR_WIDTH     = 28           # bit, 256 MB per core
CORE_IFB_STRIDE         = 0x10000000   # = 1 << CORE_IFB_ADDR_WIDTH


def core_ifb_axi_base(core_id: int) -> int:
    """返回 core[core_id] 的 IFB AXI region base 地址."""
    return CORE_IFB_BASE + core_id * CORE_IFB_STRIDE


# =============================================================================
# 5. cfg_regs CSR 地址映射 (12 bit reg_addr 空间)
#    cfg_regs.sv 的 localparam ADDR_* 跟这里必须一致.
# =============================================================================
CSR_ADDR_MAP = {
    # ---- Boot regs (csr_w only, host AXI-Lite) ----
    'CTRL'              : 0x000,
    'STATUS'            : 0x004,
    'DMA_MODE'          : 0x17C,
    'DESC_LIST_BASE'    : 0x180,
    'DESC_COUNT'        : 0x184,

    # ---- Layer cfg (seq_w only, sequencer 消费 CFG_WRITE descriptor 写入) ----
    'H_OUT'             : 0x100,
    'W_OUT'             : 0x104,
    'W_IN'              : 0x108,
    'K'                 : 0x10C,
    'STRIDE'            : 0x110,
    'CIN_SLICES'        : 0x114,
    'COUT_SLICES'       : 0x118,
    'TILE_W'            : 0x11C,
    'NUM_TILES'         : 0x120,
    'LAST_VALID_W'      : 0x124,
    'TOTAL_WRF'         : 0x128,
    'KY'                : 0x12C,
    'KK'                : 0x130,
    'ROUNDS_PER_CINS'   : 0x134,
    'ROUND_LEN_LAST'    : 0x138,
    'IFB_BASE'          : 0x13C,
    'WB_BASE'           : 0x140,
    'OFB_BASE'          : 0x144,
    'IFB_ROW_STEP'      : 0x14C,
    'WB_COUT_STEP'      : 0x154,
    'TILE_IN_STEP'      : 0x15C,
    'SDP_SHIFT'         : 0x160,
    'SDP_RELU_EN'       : 0x164,
    'H_IN_TOTAL'        : 0x168,
    'IFB_STRIP_ROWS'    : 0x16C,
    'OFB_STRIP_ROWS'    : 0x170,
    'DDR_IFM_ROW_STRIDE': 0x174,
    'DDR_OFM_ROW_STRIDE': 0x178,
    'SDP_MULT'          : 0x188,
    'SDP_ZP_OUT'        : 0x18C,
    'SDP_CLIP_MIN'      : 0x190,
    'SDP_CLIP_MAX'      : 0x194,
    'SDP_ROUND_EN'      : 0x198,
    'IFB_RING_WORDS'    : 0x1A0,
    'OFB_ROW_WORDS'     : 0x1A4,
    'OFB_RING_WORDS'    : 0x1A8,
    'IFB_ISS_STEP'      : 0x1AC,
    'IFB_KY_STEP'       : 0x1B0,
    'TILE_PIX_STEP'     : 0x1B4,
    'ARF_REUSE_EN'      : 0x1B8,
    'RESIDUAL_EN'       : 0x1BC,
    'SHORTCUT_MULT'     : 0x1C0,
    'SHORTCUT_SHIFT'    : 0x1C4,
    'BIAS_BASE'         : 0x1C8,
    'SKIP_IDMA'         : 0x1CC,    # M2: 跨核 consumer 跳过本地 IDMA
    'OFM_TDEST'         : 0x1D0,    # mesh: ODMA OFM 包目的节点 tdest
    'OFM_OPCODE'        : 0x1D4,    # mesh: ODMA OFM 包 opcode
    'IDMA_CMD_LIST_PTR' : 0x1D8,    # SMC: IDMA SG cmd list 在全局地址起点 (32-bit)
    'IDMA_CMD_COUNT'    : 0x1DC,    # SMC: IDMA SG cmd 总数 (16-bit)
    'IDMA_CMDS_PER_ROW' : 0x1E0,    # SMC: 每行 IFM 由几条 cmd 组成 (W slice 跨 mem 时 > 1)
    'ODMA_CMD_LIST_PTR' : 0x1E4,    # SMC: ODMA SG cmd list 在全局地址起点
    'ODMA_CMD_COUNT'    : 0x1E8,    # SMC: ODMA SG cmd 总数
    'ODMA_CMDS_PER_ROW' : 0x1EC,    # SMC: 每行 OFM 由几条 cmd 组成 (W/cout 跨 mem 时 > 1)

    # ---- DMA cfg (seq_w only) ----
    'IDMA_SRC_BASE'     : 0x200,
    'IDMA_BYTE_LEN'     : 0x204,
    'WDMA_SRC_BASE'     : 0x210,
    'WDMA_BYTE_LEN'     : 0x214,
    'ODMA_DST_BASE'     : 0x220,
    'ODMA_BYTE_LEN'     : 0x224,
    'RDMA_SRC_BASE'     : 0x230,
    'RDMA_BYTE_LEN'     : 0x234,
}

# 走 host AXI-Lite (csr_w 路径), 不进 CFG_WRITE descriptor 的 reg
BOOT_REG_ADDRS = {0x000, 0x17C, 0x180, 0x184}


# =============================================================================
# 6. TB / 编译器 default DDR layout (single-core / multicore TB 共享)
#    多核 chain 模式下 toolchain 会用 stage-by-stage 计算实际 base, 这里只给默认值.
# =============================================================================
DEFAULT_DDR_LAYOUT = {
    'IFB_BASE'   : 0x000000,
    'WB_BASE'    : 0x800000,
    'OFB_BASE'   : 0x900000,
    'RDMA_BASE'  : 0xD00000,
    'DESC_BASE'  : 0xB00000,
}


# =============================================================================
# 7. RTL header 生成器
#    跑 `python params.py` 生成 RTL/flux_cnn_params.svh.
# =============================================================================
SVH_HEADER = """\
// =============================================================================
// flux_cnn_params.svh  --  自动生成, 不要手改
// 来源: params.py (single source of truth)
// 改完 params.py 跑 `python params.py` 重生成此文件.
// =============================================================================

`ifndef FLUX_CNN_PARAMS_SVH
`define FLUX_CNN_PARAMS_SVH

"""

SVH_FOOTER = """\

`endif // FLUX_CNN_PARAMS_SVH
"""


def _macro_lines(items, group_name):
    """把 (name, value, comment) tuple list 转 `define 宏行."""
    out = [f"// ---- {group_name} ----"]
    for entry in items:
        if len(entry) == 3:
            name, value, comment = entry
        else:
            name, value = entry
            comment = ''
        # value 是 int, hex 大值用 0x 格式
        if isinstance(value, int):
            if value >= 0x10000:
                vs = f"32'h{value:08X}"
            elif value < 0:
                vs = str(value)
            else:
                vs = str(value)
        else:
            vs = str(value)
        line = f"`define FLUX_{name:<24} {vs}"
        if comment:
            line += f"  // {comment}"
        out.append(line)
    out.append('')
    return out


def gen_svh(out_path: str = 'RTL/flux_cnn_params.svh') -> str:
    """生成 RTL/flux_cnn_params.svh, 返回内容字符串."""
    lines = [SVH_HEADER.rstrip()]

    # Datapath
    lines.extend(_macro_lines([
        ('NUM_PE',      NUM_PE,      'PE per column (= IFB cin byte 数)'),
        ('NUM_COL',     NUM_COL,     'mac_array 列数 (= cout 并行度)'),
        ('DATA_WIDTH',  DATA_WIDTH,  'INT8'),
        ('PSUM_WIDTH',  PSUM_WIDTH,  'INT32 累加'),
        ('WRF_DEPTH',   WRF_DEPTH,   '每 PE 内权重 RF 深度'),
        ('ARF_DEPTH',   ARF_DEPTH,   'line_buffer 内 act ring depth'),
        ('PARF_DEPTH',  PARF_DEPTH,  'parf_accum partial sum depth'),
    ], 'Datapath'))

    # SRAM
    lines.extend(_macro_lines([
        ('IFB_DEPTH',       IFB_DEPTH,      '128 KB (1 word = 128 bit)'),
        ('WB_DEPTH',        WB_DEPTH,       '256 KB (1 word = 2048 bit)'),
        ('OFB_DEPTH',       OFB_DEPTH,      '32 KB (1 word = 128 bit)'),
        ('SHORTCUT_DEPTH',  SHORTCUT_DEPTH, '128 KB Shortcut Bank'),
        ('SRAM_DEPTH',      SRAM_DEPTH,     'legacy alias for IFB_DEPTH'),
    ], 'SRAM 容量'))

    # AXI
    lines.extend(_macro_lines([
        ('BUS_ADDR_W',  BUS_ADDR_W),
        ('BUS_DATA_W',  BUS_DATA_W),
        ('AXI_M_ID',    AXI_M_ID,    'per-master ID 宽'),
        ('AXI_M_WIDTH', AXI_M_WIDTH, 'log2(masters/core)'),
        ('DMA_LEN_W',   DMA_LEN_W),
        ('CSR_ADDR_W',  CSR_ADDR_W),
        ('CSR_DATA_W',  CSR_DATA_W),
        ('CORE_BUS_ID', CORE_BUS_ID, '= AXI_M_ID + AXI_M_WIDTH'),
    ], 'AXI / CSR'))

    # Address map
    lines.extend(_macro_lines([
        ('DDR_BASE',                DDR_BASE),
        ('DDR_ADDR_WIDTH',          DDR_ADDR_WIDTH),
        ('CORE_IFB_BASE',           CORE_IFB_BASE),
        ('CORE_IFB_ADDR_WIDTH',     CORE_IFB_ADDR_WIDTH),
        ('CORE_IFB_STRIDE',         CORE_IFB_STRIDE),
    ], '全局地址映射 (多核)'))

    # CSR addr map (前缀 ADDR_)
    csr_items = [(f'ADDR_{name}', addr) for name, addr in CSR_ADDR_MAP.items()]
    csr_items_with_comment = []
    for name, addr in csr_items:
        # value 用 12'hXXX 格式
        addr_hex = f"12'h{addr:03X}"
        csr_items_with_comment.append((name, addr_hex, ''))
    lines.append("// ---- CSR address map (cfg_regs reg_addr) ----")
    for name, val_str, _ in csr_items_with_comment:
        lines.append(f"`define FLUX_{name:<24} {val_str}")
    lines.append('')

    lines.append(SVH_FOOTER.rstrip())

    content = '\n'.join(lines) + '\n'

    # 写文件
    proj_root = Path(__file__).resolve().parent
    out_full = proj_root / out_path
    out_full.parent.mkdir(parents=True, exist_ok=True)
    out_full.write_text(content, encoding='utf-8')
    return str(out_full)


# =============================================================================
# CLI
# =============================================================================
if __name__ == '__main__':
    out = gen_svh()
    print(f"Generated {out}")
    print(f"  - {len(CSR_ADDR_MAP)} CSR addr entries")
    print(f"  - Datapath: NUM_PE={NUM_PE}, NUM_COL={NUM_COL}, DATA_WIDTH={DATA_WIDTH}")
    print(f"  - SRAM: IFB={IFB_DEPTH}, WB={WB_DEPTH}, OFB={OFB_DEPTH}, Shortcut={SHORTCUT_DEPTH}")
    print(f"  - AXI: ADDR={BUS_ADDR_W}, DATA={BUS_DATA_W}")
    print(f"  - Multicore: CORE_IFB_BASE=0x{CORE_IFB_BASE:08X}, STRIDE=0x{CORE_IFB_STRIDE:08X}")
