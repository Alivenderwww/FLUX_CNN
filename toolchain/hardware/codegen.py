"""codegen.py — 从 hardware.json 生成 flux_cnn_params.svh.

Phase A: hardware.json 是 single source of truth for arch 字段.
params.py 现在改成 thin loader (顶部 import HardwareConfig + 暴露 NUM_PE 等同名全局).
svh 仍需独立生成 (RTL 用), 由本脚本负责.

CSR_ADDR_MAP 是 RTL+Python 同步约定 (跟 cfg_regs.sv localparam 一一对应),
不在 hardware.json 里, 而是写在 codegen.py 模板里 (single source for RTL+toolchain).

Usage:
    python -m toolchain.hardware.codegen [--hw vd100.json] [--check]
        --check: 生成内容跟现有 svh 比对, 不一致返回非 0 (CI 用)
"""
from __future__ import annotations

import argparse
import difflib
import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_TOOLCHAIN_DIR = os.path.dirname(_THIS_DIR)
_PROJECT_ROOT = os.path.dirname(_TOOLCHAIN_DIR)

if _TOOLCHAIN_DIR not in sys.path:
    sys.path.insert(0, _TOOLCHAIN_DIR)

from hardware.hardware_cfg import HardwareConfig


# ---------- CSR ADDR MAP (硬件 CSR 寄存器布局, RTL+toolchain 同步约定) ----------
# 跟 cfg_regs.sv localparam ADDR_* 一一对应. 改这里要同步改 cfg_regs.sv.
# 由于 RTL 也要它 (生成 svh), 不放 hardware.json (后者是 arch + deployment).
CSR_ADDR_MAP = {
    'CTRL':              0x000, 'STATUS':            0x004, 'DMA_MODE':          0x17C,
    'DESC_LIST_BASE':    0x180, 'DESC_COUNT':        0x184,
    'H_OUT':             0x100, 'W_OUT':             0x104, 'W_IN':              0x108,
    'K':                 0x10C, 'STRIDE':            0x110, 'CIN_SLICES':        0x114,
    'COUT_SLICES':       0x118, 'TILE_W':            0x11C, 'NUM_TILES':         0x120,
    'LAST_VALID_W':      0x124, 'TOTAL_WRF':         0x128, 'KY':                0x12C,
    'KK':                0x130, 'ROUNDS_PER_CINS':   0x134, 'ROUND_LEN_LAST':    0x138,
    'IFB_BASE':          0x13C, 'WB_BASE':           0x140, 'OFB_BASE':          0x144,
    'IFB_ROW_STEP':      0x14C, 'WB_COUT_STEP':      0x154, 'TILE_IN_STEP':      0x15C,
    'SDP_SHIFT':         0x160, 'SDP_RELU_EN':       0x164, 'H_IN_TOTAL':        0x168,
    'IFB_STRIP_ROWS':    0x16C, 'OFB_STRIP_ROWS':    0x170, 'DDR_IFM_ROW_STRIDE':0x174,
    'DDR_OFM_ROW_STRIDE':0x178, 'SDP_MULT':          0x188, 'SDP_ZP_OUT':        0x18C,
    'SDP_CLIP_MIN':      0x190, 'SDP_CLIP_MAX':      0x194, 'SDP_ROUND_EN':      0x198,
    'IFB_RING_WORDS':    0x1A0, 'OFB_ROW_WORDS':     0x1A4, 'OFB_RING_WORDS':    0x1A8,
    'IFB_ISS_STEP':      0x1AC, 'IFB_KY_STEP':       0x1B0, 'TILE_PIX_STEP':     0x1B4,
    'ARF_REUSE_EN':      0x1B8, 'RESIDUAL_EN':       0x1BC, 'SHORTCUT_MULT':     0x1C0,
    'SHORTCUT_SHIFT':    0x1C4, 'BIAS_BASE':         0x1C8, 'SKIP_IDMA':         0x1CC,
    'OFM_TDEST':         0x1D0, 'OFM_OPCODE':        0x1D4, 'IDMA_CMD_LIST_PTR': 0x1D8,
    'IDMA_CMD_COUNT':    0x1DC, 'IDMA_CMDS_PER_ROW': 0x1E0, 'STRIDE_H':          0x1F0,
    'ODMA_CMD_LIST_PTR': 0x1E4, 'ODMA_CMD_COUNT':    0x1E8, 'ODMA_CMDS_PER_ROW': 0x1EC,
    'IDMA_SRC_BASE':     0x200, 'IDMA_BYTE_LEN':     0x204,
    'WDMA_SRC_BASE':     0x210, 'WDMA_BYTE_LEN':     0x214,
    'ODMA_DST_BASE':     0x220, 'ODMA_BYTE_LEN':     0x224,
    'RDMA_SRC_BASE':     0x230, 'RDMA_BYTE_LEN':     0x234,
}


def gen_svh(cfg: HardwareConfig) -> str:
    """从 HardwareConfig 生成 RTL/flux_cnn_params.svh 内容."""
    arch = cfg.arch

    lines = []
    lines.append('// =============================================================================')
    lines.append('// flux_cnn_params.svh  --  自动生成, 不要手改')
    lines.append('// 来源: hardware/*.json + toolchain/hardware/codegen.py')
    lines.append('// 改硬件参数: 改 hardware/vd100.json 后跑 `python -m toolchain.hardware.codegen`')
    lines.append('// =============================================================================')
    lines.append('')
    lines.append('`ifndef FLUX_CNN_PARAMS_SVH')
    lines.append('`define FLUX_CNN_PARAMS_SVH')

    def emit_group(name, items):
        lines.append(f"// ---- {name} ----")
        for entry in items:
            if len(entry) == 3:
                n, v, c = entry
            else:
                n, v = entry; c = ''
            if isinstance(v, int):
                if v >= 0x10000:
                    vs = f"32'h{v:08X}"
                else:
                    vs = str(v)
            else:
                vs = str(v)
            line = f"`define FLUX_{n:<24} {vs}"
            if c:
                line += f"  // {c}"
            lines.append(line)
        lines.append('')

    emit_group('Datapath', [
        ('NUM_PE',      arch.num_pe,      'PE per column (= IFB cin byte 数)'),
        ('NUM_COL',     arch.num_col,     'mac_array 列数 (= cout 并行度)'),
        ('DATA_WIDTH',  arch.data_width,  'INT8'),
        ('PSUM_WIDTH',  arch.psum_width,  'INT32 累加'),
        ('WRF_DEPTH',   arch.wrf_depth,   '每 PE 内权重 RF 深度'),
        ('ARF_DEPTH',   arch.arf_depth,   'line_buffer 内 act ring depth'),
        ('PARF_DEPTH',  arch.parf_depth,  'parf_accum partial sum depth'),
    ])

    emit_group('SRAM 容量', [
        ('IFB_DEPTH',      arch.ifb_depth,      '128 KB (1 word = 128 bit)'),
        ('WB_DEPTH',       arch.wb_depth,       '256 KB (1 word = 2048 bit)'),
        ('OFB_DEPTH',      arch.ofb_depth,      '32 KB (1 word = 128 bit)'),
        ('SHORTCUT_DEPTH', arch.shortcut_depth, '128 KB Shortcut Bank'),
    ])

    emit_group('AXI / CSR', [
        ('BUS_ADDR_W',  arch.bus_addr_width),
        ('BUS_DATA_W',  arch.bus_data_width),
        ('AXI_M_ID',    arch.axi_m_id,    'per-master ID 宽'),
        ('AXI_M_WIDTH', arch.axi_m_width, 'log2(masters/core)'),
        ('DMA_LEN_W',   arch.dma_len_width),
        ('CSR_ADDR_W',  arch.csr_addr_width),
        ('CSR_DATA_W',  arch.csr_data_width),
        ('CORE_BUS_ID', arch.core_bus_id, '= AXI_M_ID + AXI_M_WIDTH'),
    ])

    # 全局地址映射 (RTL 多核 wrapper 视角, 不依赖 deployment)
    DDR_BASE = 0x00000000
    DDR_ADDR_WIDTH = 31
    CORE_IFB_BASE = 0x80000000
    CORE_IFB_ADDR_WIDTH = 28
    CORE_IFB_STRIDE = 1 << CORE_IFB_ADDR_WIDTH
    emit_group('全局地址映射 (多核)', [
        ('DDR_BASE',                DDR_BASE),
        ('DDR_ADDR_WIDTH',          DDR_ADDR_WIDTH),
        ('CORE_IFB_BASE',           CORE_IFB_BASE),
        ('CORE_IFB_ADDR_WIDTH',     CORE_IFB_ADDR_WIDTH),
        ('CORE_IFB_STRIDE',         CORE_IFB_STRIDE),
    ])

    # CSR addr map
    lines.append("// ---- CSR address map (cfg_regs reg_addr) ----")
    for name, addr in CSR_ADDR_MAP.items():
        n = f'ADDR_{name}'
        vs = f"12'h{addr:03X}"
        lines.append(f"`define FLUX_{n:<24} {vs}")
    lines.append('')
    lines.append('')
    lines.append('`endif // FLUX_CNN_PARAMS_SVH')
    lines.append('')

    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hw', default=os.path.join(_THIS_DIR, 'vd100.json'))
    ap.add_argument('--check', action='store_true', help='生成内容跟现有 svh 比对, 不一致返回非 0')
    ap.add_argument('--out-svh', default=os.path.join(_PROJECT_ROOT, 'RTL', 'flux_cnn_params.svh'))
    args = ap.parse_args()

    cfg = HardwareConfig.load(args.hw)
    new_svh = gen_svh(cfg)

    if args.check:
        try:
            with open(args.out_svh, 'r', encoding='utf-8') as f:
                old = f.read()
        except FileNotFoundError:
            print(f"[ERR] {args.out_svh} missing")
            sys.exit(1)
        if old == new_svh:
            print(f"[OK] svh 字节一致: {args.out_svh}")
            return
        diff = list(difflib.unified_diff(
            old.splitlines(keepends=True),
            new_svh.splitlines(keepends=True),
            fromfile=args.out_svh, tofile='<generated>', n=2))
        print(f"[DIFF] {len(diff)} lines:")
        sys.stdout.writelines(diff[:80])
        sys.exit(1)
    else:
        with open(args.out_svh, 'w', encoding='utf-8') as f:
            f.write(new_svh)
        print(f"Wrote {args.out_svh}")


if __name__ == '__main__':
    main()
