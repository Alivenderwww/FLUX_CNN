"""
deploy_smc_case.py — 把 sim/tb_smc/cases/<name>/ 通过 RPC 部署到 VD100 板跑

行为跟 sim/tb_smc/tb_smc_chain.sv 完全等价:
  sim TB                                  本脚本
  hier ref preload_ifb_smc       ↔        load_ifb_smc (RPC LOAD_DDR W slice 散布)
  hier ref preload_wb_smc        ↔        load_wb_smc (broadcast 各 mem)
  hier ref preload_rdma_smc      ↔        load_rdma_smc (per-core sliced 或 broadcast)
  hier ref preload_desc_smc      ↔        load_desc_smc (LOAD_DDR per-(core,layer) hex)
  hier ref preload_sg_cmd        ↔        load_sg_cmd_smc (LOAD_DDR per-(core,layer) idma/odma SG)
  axi_lite_write + write_dfe_start ↔     RUN_LAYERS RPC (a72 自跑 11 层)
  read_mem_word + check_layer_ofm ↔      read_ofm_smc + compare (stitch 多核 W 段)

关键依赖:
  toolchain 必须用 FLUX_SMC_GLOBAL_BASE=0x10000000 (或其它 PS-firmware 安全 base) 重生成 case,
  否则 case 数据的 DDR 地址会撞 a72 ELF 区 (0x0-0x01000000).
  生成: cd toolchain && FLUX_SMC_GLOBAL_BASE=0x10000000 \
        python run_multicore_chain.py --smc --demo resnet11 --case_name vd100_resnet11_n3 --n_cores 3

用法:
  cd host/vd100_pc
  python deploy_smc_case.py --case-dir ../../sim/tb_smc/cases/vd100_resnet11_n3 --vd100-ip 169.254.111.10
"""
import argparse
import os
import re
import struct
import sys
import time

import vd100_rpc
from vd100_rpc import Vd100Rpc, NUM_CORES


# === F5: SMC layout 从 hardware/vd100.json 读 (废除 hardcode) ===
# 加 toolchain 到 sys.path 以便 import HardwareConfig
_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_TOOLCHAIN_DIR = os.path.join(_PROJECT_ROOT, 'toolchain')
if _TOOLCHAIN_DIR not in sys.path:
    sys.path.insert(0, _TOOLCHAIN_DIR)

try:
    from hardware.hardware_cfg import HardwareConfig
    _HW_PATH = os.environ.get('FLUX_HW_JSON',
                              os.path.join(_TOOLCHAIN_DIR, 'hardware', 'vd100.json'))
    _hw = HardwareConfig.load(_HW_PATH)
    _r = _hw.deployment.regions
    SMC_MEM_STRIDE         = _hw.deployment.smc_mem_stride
    SMC_IFM_OFM_BASE       = _r.ifm_ofm_base
    SMC_LAYER_DATA_OFFSET  = _r.layer_data_offset
    SMC_WB_BASE            = _r.wb_base
    SMC_LAYER_WB_OFFSET    = _r.layer_wb_offset
    SMC_RDMA_BASE          = _r.rdma_base
    SMC_LAYER_RDMA_OFFSET  = _r.layer_rdma_offset
    SMC_DESC_BASE          = _r.desc_base
    SMC_LAYER_DESC_OFFSET  = _r.layer_desc_offset
    SMC_INPUT_BASE         = _r.input_base
    SMC_LAYER_INPUT_OFFSET = _r.layer_input_offset
    SMC_FINAL_OFM_BASE     = _r.final_ofm_base
except Exception as _e:
    # Fallback hardcode (硬件 JSON 缺失时)
    print(f"[WARN] hardware.json load failed ({_e}); 用 hardcode 默认", file=sys.stderr)
    SMC_MEM_STRIDE        = 0x0100_0000
    SMC_IFM_OFM_BASE      = 0x0000_0000
    SMC_LAYER_DATA_OFFSET = 0x0008_0000
    SMC_WB_BASE           = 0x0080_0000
    SMC_LAYER_WB_OFFSET   = 0x0001_0000
    SMC_RDMA_BASE         = 0x0090_0000
    SMC_LAYER_RDMA_OFFSET = 0x0001_0000
    SMC_DESC_BASE         = 0x00A0_0000
    SMC_LAYER_DESC_OFFSET = 0x0001_0000
    SMC_INPUT_BASE        = 0x00D0_0000
    SMC_LAYER_INPUT_OFFSET = 0x0008_0000
    SMC_FINAL_OFM_BASE    = 0x00F0_0000

NUM_COL = 16   # mac_array 列数 (= cout 并行度)
WORD_BYTES = 16   # BUS_DATA_W=128 bit


# -------------------- hex 文件 → bytes --------------------
def load_hex_file_words(path: str) -> list:
    """读 sim hex 文件 (每行 N hex char = N/2 byte big-endian, $readmemh 兼容).

    sim TB 用 $readmemh 读, hex string 是 big-endian (左边高位 right->left write).
    我们按行返回 bytes (big-endian → little-endian byte 翻转), 每个元素是 1 word.
    比如 IFB: 32 hex char/line = 16 byte = 1 word (BUS_DATA_W=128 bit)
    比如 WB:  512 hex char/line = 256 byte = 16 word (NUM_COL × NUM_PE × DATA_WIDTH = 2048 bit)
    """
    words = []
    if not os.path.exists(path):
        return words
    with open(path, 'r') as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith('//') or s.startswith('#'):
                continue   # 跳过注释 ($readmemh 兼容)
            # 去掉行尾注释
            if '//' in s:
                s = s.split('//', 1)[0].strip()
            if not s:
                continue
            if len(s) % 2 != 0:
                s = '0' + s   # pad 奇数 hex char
            big = bytes.fromhex(s)
            le = big[::-1]   # 翻成 little-endian
            # 拆 16 byte words (从低位到高位取 16 byte = 1 word)
            for i in range(0, len(le), WORD_BYTES):
                w = le[i:i + WORD_BYTES]
                if len(w) < WORD_BYTES:
                    w = w + b'\x00' * (WORD_BYTES - len(w))
                words.append(w)
    return words


# -------------------- meta 解析 --------------------
def parse_meta(case_dir: str) -> dict:
    """优先读 manifest.json (Phase B), 回退老 multicore_meta.txt.
    返回老格式 dict (兼容现有代码)."""
    manifest_path = os.path.join(case_dir, 'manifest.json')
    if os.path.exists(manifest_path):
        return _parse_meta_from_manifest(manifest_path, case_dir)
    return _parse_meta_legacy(case_dir)


def _parse_meta_from_manifest(manifest_path: str, case_dir: str) -> dict:
    """从 manifest.json 重建老 multicore_meta.txt 格式 dict (兼容当前 deploy 逻辑)."""
    import json
    with open(manifest_path, 'r', encoding='utf-8') as f:
        m = json.load(f)
    # 仍 fallback 老 meta 拿 SMC_* / DDR_* / LAYER_X_DIR 等老字段 (sim 兼容)
    legacy = _parse_meta_legacy(case_dir) if os.path.exists(os.path.join(case_dir, 'multicore_meta.txt')) else {}
    # manifest 字段映射回老 KV (host 用)
    def _hex2int(v):
        return int(v, 0) if isinstance(v, str) else int(v)
    legacy['NUM_CORES']           = int(m['n_cores'])
    legacy['NUM_LAYERS']          = int(m['n_layers'])
    dep = m.get('deploy', {})
    legacy['_DESC_LIST_BASE_PER_CORE'] = [_hex2int(v) for v in dep.get('desc_list_base_per_core', [])]
    legacy['_DESC_COUNT_PER_CORE']     = list(dep.get('desc_count_per_core', []))
    legacy['_FINAL_OFM_BASE']     = _hex2int(dep.get('final_ofm_base', 0))
    # 每 (core, layer) DESC_BASE / DESC_COUNT (host poke_csr 用)
    for layer in m['layers']:
        for cid_str, smc in layer.get('smc_layout', {}).items():
            cid = int(cid_str.split('_')[1]) if cid_str.startswith('core_') else int(cid_str)
            li = int(layer['idx'])
            legacy[f'CORE_{cid}_LAYER_{li}_DESC_BASE']  = _hex2int(smc['desc_base'])
            legacy[f'CORE_{cid}_LAYER_{li}_DESC_COUNT'] = int(smc['desc_count'])
    legacy['_manifest_source'] = manifest_path
    return legacy


def _parse_meta_legacy(case_dir: str) -> dict:
    """旧版 multicore_meta.txt KV 文本解析."""
    path = os.path.join(case_dir, 'multicore_meta.txt')
    meta = {}
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith(';'):
                continue
            m = re.match(r'^([\w_]+)\s*=\s*(.+)$', line)
            if not m:
                continue
            key, val = m.group(1), m.group(2).strip()
            # 类型识别
            if val.startswith('0x') or val.startswith('0X'):
                meta[key] = int(val, 16)
            elif re.match(r'^-?\d+$', val):
                meta[key] = int(val)
            elif re.match(r'^[\d\s\-]+$', val):  # 多个数 (空格分隔)
                meta[key] = [int(x) for x in val.split()]
            else:
                meta[key] = val
    return meta


def derive_smc_global_base(meta: dict) -> int:
    """从 CORE_0_LAYER_0_DESC_BASE 反推 SMC_GLOBAL_BASE.
    DESC_BASE = SMC_GLOBAL_BASE + 0*MEM_STRIDE + SMC_DESC_BASE + 0*SMC_LAYER_DESC_OFFSET
    """
    if 'CORE_0_LAYER_0_DESC_BASE' not in meta:
        raise KeyError("multicore_meta.txt 缺 CORE_0_LAYER_0_DESC_BASE")
    return meta['CORE_0_LAYER_0_DESC_BASE'] - SMC_DESC_BASE


def smc_addr(global_base: int, mem_id: int, mem_offset: int) -> int:
    return global_base + mem_id * SMC_MEM_STRIDE + mem_offset


# -------------------- 部署各 region --------------------
def load_ifb_smc(rpc: Vd100Rpc, case_dir: str, meta: dict, layer_idx: int, global_base: int):
    """layer_idx 的 IFB 散布到各 mem (W slice 模式) 或集中存 mem[my_core] (mode A/C).
    读 chain_data/layerXX/ifb.txt → 按 SMC_LAYER_X_IFM_SEG_WIDTHS/STARTS 切散布到 mem[i].
    """
    ldir_key = f'LAYER_{layer_idx}_DIR'
    fallback = os.path.join(case_dir, 'chain_data', f'layer{layer_idx:02d}')
    if ldir_key in meta and meta[ldir_key]:
        ldir = meta[ldir_key]
        if not os.path.isabs(ldir):
            ldir = os.path.join(case_dir, ldir)
        if not os.path.isdir(ldir):
            ldir = fallback   # case 移动后 meta 绝对路径失效, fallback
    else:
        ldir = fallback

    ifb_words = load_hex_file_words(os.path.join(ldir, 'ifb.txt'))
    if not ifb_words:
        print(f"    [warn] layer {layer_idx} ifb.txt empty")
        return

    h_in       = meta[f'SMC_LAYER_{layer_idx}_H_IN']
    w_in       = meta[f'SMC_LAYER_{layer_idx}_W_IN']
    cin_slices = meta[f'SMC_LAYER_{layer_idx}_C_IN_SLICES']
    mode       = meta[f'SMC_LAYER_{layer_idx}_MODE']

    # base offset within each mem (跟 sim TB preload_ifb_smc 一致)
    # 优先用 SMC_LAYER_X_IFB_OFFSET (新, 动态分配); fallback ROOT_SLOT × 固定 stride (老 case)
    ifb_off_key = f'SMC_LAYER_{layer_idx}_IFB_OFFSET'
    if ifb_off_key in meta and meta[ifb_off_key] >= 0:
        base_offset = SMC_INPUT_BASE + meta[ifb_off_key]
    else:
        root_slot = meta.get(f'SMC_LAYER_{layer_idx}_ROOT_SLOT', -1)
        if layer_idx == 0:
            base_offset = SMC_INPUT_BASE
        elif root_slot >= 0:
            base_offset = SMC_INPUT_BASE + (root_slot + 1) * SMC_LAYER_INPUT_OFFSET
        else:
            return   # 非 root layer 不需 preload

    if mode in ('A', 'C'):
        my_core = meta[f'SMC_LAYER_{layer_idx}_MODE_A_CORE']
        base = smc_addr(global_base, my_core, base_offset)
        # 整图紧凑
        n_words = h_in * w_in * cin_slices
        buf = b''.join(ifb_words[:n_words])
        rpc.load_ddr(base, buf)
        print(f"    IFB[{layer_idx}] mode {mode} → mem[{my_core}] @ 0x{base:08x} ({len(buf)/1024:.1f} KB)")
    else:  # W slice
        seg_widths = meta[f'SMC_LAYER_{layer_idx}_IFM_SEG_WIDTHS']
        seg_starts = meta[f'SMC_LAYER_{layer_idx}_IFM_SEG_STARTS']
        if isinstance(seg_widths, int): seg_widths = [seg_widths]
        if isinstance(seg_starts, int): seg_starts = [seg_starts]
        for i in range(int(meta.get('NUM_CORES', NUM_CORES))):
            sw, ss = seg_widths[i], seg_starts[i]
            base = smc_addr(global_base, i, base_offset)
            # gather: r * sw * cs + x * cs + cs_idx ← src: r * w_in * cs + (ss+x) * cs + cs_idx
            buf = bytearray()
            for r in range(h_in):
                for x in range(sw):
                    src_base = (r * w_in + (ss + x)) * cin_slices
                    for cs in range(cin_slices):
                        buf.extend(ifb_words[src_base + cs])
            rpc.load_ddr(base, bytes(buf))
        print(f"    IFB[{layer_idx}] W slice → 3 mem (seg_widths={seg_widths}, h_in={h_in}, cs={cin_slices})")


def load_wb_smc(rpc: Vd100Rpc, case_dir: str, meta: dict, layer_idx: int, global_base: int):
    """layer wb broadcast 到各 mem (mode A/W) 或 切 cout 段散布 (mode C)."""
    ldir = os.path.join(case_dir, 'chain_data', f'layer{layer_idx:02d}')
    n_words = meta.get(f'LAYER_{layer_idx}_WB_WORDS', 0)
    if n_words == 0:
        return
    # WB hex 一行 = 16 word (BUS_DATA_W × 16 = 2048 bit). load_hex_file_words 拆完后, 16 个 word 是一个 wb-row.
    wb_all_words = load_hex_file_words(os.path.join(ldir, 'wb.txt'))
    mode = meta[f'SMC_LAYER_{layer_idx}_MODE']

    if mode == 'C':
        # cout slice: 每核装自己段
        cs_starts = meta[f'SMC_LAYER_{layer_idx}_COUT_SEG_STARTS']
        cs_widths = meta[f'SMC_LAYER_{layer_idx}_COUT_SEG_WIDTHS']
        if isinstance(cs_starts, int): cs_starts = [cs_starts]
        if isinstance(cs_widths, int): cs_widths = [cs_widths]
        kk = meta[f'SMC_LAYER_{layer_idx}_K'] ** 2
        cins = meta[f'SMC_LAYER_{layer_idx}_C_IN_SLICES']
        wb_per_cs = kk * cins   # 每 cs 占 wb-rows
        for i in range(int(meta.get('NUM_CORES', NUM_CORES))):
            my_cs_start = cs_starts[i] // NUM_COL
            my_cs_n = (cs_widths[i] + NUM_COL - 1) // NUM_COL
            my_wb_lo = my_cs_start * wb_per_cs
            my_wb_n  = my_cs_n * wb_per_cs
            base = smc_addr(global_base, i, SMC_WB_BASE + layer_idx * SMC_LAYER_WB_OFFSET)
            buf = b''.join(wb_all_words[my_wb_lo * 16 : (my_wb_lo + my_wb_n) * 16])
            rpc.load_ddr(base, buf)
        print(f"    WB[{layer_idx}] cout slice → 3 mem")
    else:
        # broadcast 整图
        buf = b''.join(wb_all_words[: n_words * 16])
        for i in range(int(meta.get('NUM_CORES', NUM_CORES))):
            base = smc_addr(global_base, i, SMC_WB_BASE + layer_idx * SMC_LAYER_WB_OFFSET)
            rpc.load_ddr(base, buf)
        print(f"    WB[{layer_idx}] broadcast → 3 mem ({len(buf)/1024:.1f} KB)")


def load_rdma_smc(rpc: Vd100Rpc, case_dir: str, meta: dict, layer_idx: int, global_base: int):
    """RDMA per-core sliced (residual + W slice) 或 broadcast (mode A/W 普通 bias)."""
    ldir = os.path.join(case_dir, 'chain_data', f'layer{layer_idx:02d}')
    # 优先 sliced
    any_sliced = False
    sliced_words = {}
    for c in range(int(meta.get('NUM_CORES', NUM_CORES))):
        n_w = meta.get(f'CORE_{c}_LAYER_{layer_idx}_RDMA_WORDS', 0)
        if n_w > 0:
            any_sliced = True
            words = load_hex_file_words(os.path.join(ldir, f'rdma_data_c{c}.txt'))
            sliced_words[c] = words[:n_w]
    if any_sliced:
        for c, wlist in sliced_words.items():
            base = smc_addr(global_base, c, SMC_RDMA_BASE + layer_idx * SMC_LAYER_RDMA_OFFSET)
            buf = b''.join(wlist)
            rpc.load_ddr(base, buf)
        print(f"    RDMA[{layer_idx}] sliced → 3 mem")
    else:
        n_words = meta.get(f'LAYER_{layer_idx}_RDMA_WORDS', 0)
        if n_words == 0:
            return
        rdma_path = os.path.join(ldir, 'rdma_data.txt')
        if not os.path.exists(rdma_path):
            return
        words = load_hex_file_words(rdma_path)
        mode = meta[f'SMC_LAYER_{layer_idx}_MODE']
        if mode == 'C':
            cs_starts = meta[f'SMC_LAYER_{layer_idx}_COUT_SEG_STARTS']
            cs_widths = meta[f'SMC_LAYER_{layer_idx}_COUT_SEG_WIDTHS']
            if isinstance(cs_starts, int): cs_starts = [cs_starts]
            if isinstance(cs_widths, int): cs_widths = [cs_widths]
            for i in range(int(meta.get('NUM_CORES', NUM_CORES))):
                my_cs_start = cs_starts[i] // NUM_COL
                my_cs_n = (cs_widths[i] + NUM_COL - 1) // NUM_COL
                base = smc_addr(global_base, i, SMC_RDMA_BASE + layer_idx * SMC_LAYER_RDMA_OFFSET)
                buf = b''.join(words[my_cs_start * 4 : (my_cs_start + my_cs_n) * 4])
                rpc.load_ddr(base, buf)
        else:
            buf = b''.join(words[: n_words])
            for i in range(int(meta.get('NUM_CORES', NUM_CORES))):
                base = smc_addr(global_base, i, SMC_RDMA_BASE + layer_idx * SMC_LAYER_RDMA_OFFSET)
                rpc.load_ddr(base, buf)
        print(f"    RDMA[{layer_idx}] broadcast → 3 mem ({len(buf)/1024:.1f} KB)")


def load_desc_smc(rpc: Vd100Rpc, case_dir: str, meta: dict, layer_idx: int):
    """desc list per (core, layer)."""
    for c in range(int(meta.get('NUM_CORES', NUM_CORES))):
        key_base = f'CORE_{c}_LAYER_{layer_idx}_DESC_BASE'
        key_count = f'CORE_{c}_LAYER_{layer_idx}_DESC_COUNT'
        if key_base not in meta or meta.get(key_count, 0) == 0:
            continue
        path = os.path.join(case_dir, f'core{c}', f'layer{layer_idx:02d}_desc_list.hex')
        words = load_hex_file_words(path)
        # FIX 2026-05-12: 每 desc 是 32 byte (256 bit) = 2 word (hex 文件每行 128 bit). 老代码 words[:n_descs] 只写一半 desc 数据
        n_descs = meta[key_count]
        buf = b''.join(words[: n_descs * 2])
        rpc.load_ddr(meta[key_base], buf)


def load_sg_cmd_smc(rpc: Vd100Rpc, case_dir: str, meta: dict, layer_idx: int):
    """IDMA + ODMA SG cmd lists per (core, layer). 每 cmd 32 byte = 2 word."""
    for c in range(int(meta.get('NUM_CORES', NUM_CORES))):
        for which in ('idma', 'odma'):
            key_base  = f'SMC_CORE_{c}_LAYER_{layer_idx}_{which.upper()}_CMD_BASE'
            key_count = f'SMC_CORE_{c}_LAYER_{layer_idx}_{which.upper()}_CMD_COUNT'
            if key_base not in meta or meta.get(key_count, 0) == 0:
                continue
            path = os.path.join(case_dir, f'core{c}', f'layer{layer_idx:02d}_{which}_sg.hex')
            words = load_hex_file_words(path)
            n_cmds = meta[key_count]
            buf = b''.join(words[: n_cmds * 2])   # 2 word / cmd
            rpc.load_ddr(meta[key_base], buf)


# -------------------- run + verify --------------------
def build_layer_cfgs(meta: dict) -> list:
    """从 meta 构造 RUN_LAYERS 的 layer_cfg_t array (n_layers × 32 byte).
    每层带 per-core (desc_list_base, desc_count). desc_count==0 表示该核本层不参与.
    """
    n_layers = meta['NUM_LAYERS']
    cfgs = []
    for l in range(n_layers):
        base, count = [0]*NUM_CORES, [0]*NUM_CORES
        for c in range(int(meta.get('NUM_CORES', NUM_CORES))):
            base [c] = meta.get(f'CORE_{c}_LAYER_{l}_DESC_BASE', 0)
            count[c] = meta.get(f'CORE_{c}_LAYER_{l}_DESC_COUNT', 0)
        cfgs.append({'desc_list_base': base, 'desc_count': count})
    return cfgs


def read_ofm_smc(rpc: Vd100Rpc, meta: dict, layer_idx: int, global_base: int,
                 ofm_offset: int = None) -> bytes:
    """读 layer_idx 的 OFM (整图, stitch 多核). layer 维度从 meta 取.
    返回 NHWC int8 byte stream, len = h_out × w_out × c_out.

    ofm_offset: 该层 OFM 在每个 mem 内的偏移. 默认 None = SMC_FINAL_OFM_BASE (最后一层).
                中间层用 SMC_IFM_OFM_BASE + layer_idx * SMC_LAYER_DATA_OFFSET.
    """
    if ofm_offset is None:
        ofm_offset = SMC_FINAL_OFM_BASE
    h_out = meta[f'LAYER_{layer_idx}_H_OUT']
    w_out = meta[f'LAYER_{layer_idx}_W_OUT']
    c_out = meta[f'LAYER_{layer_idx}_C_OUT']
    cout_slices = (c_out + NUM_COL - 1) // NUM_COL
    mode = meta[f'SMC_LAYER_{layer_idx}_MODE']

    out = bytearray(h_out * w_out * c_out)

    if mode == 'A':
        my_core = meta[f'SMC_LAYER_{layer_idx}_MODE_A_CORE']
        base = smc_addr(global_base, my_core, ofm_offset)
        n_words = h_out * w_out * cout_slices
        raw = rpc.read_ddr(base, n_words * WORD_BYTES)
        # NHWC layout: r * w * cs + x * cs + cs_idx
        # 整图 c_out byte 段 = cs_idx 0..cs-1 各贡献 16 byte; 最后段尾 c_out%16 byte
        for r in range(h_out):
            for x in range(w_out):
                base_word_idx = (r * w_out + x) * cout_slices
                base_byte_idx = (r * w_out + x) * c_out
                for cs in range(cout_slices):
                    src = raw[(base_word_idx + cs) * WORD_BYTES : (base_word_idx + cs + 1) * WORD_BYTES]
                    take = min(NUM_COL, c_out - cs * NUM_COL)
                    out[base_byte_idx + cs * NUM_COL : base_byte_idx + cs * NUM_COL + take] = src[:take]
    elif mode == 'C':
        cs_starts = meta[f'SMC_LAYER_{layer_idx}_COUT_SEG_STARTS']
        cs_widths = meta[f'SMC_LAYER_{layer_idx}_COUT_SEG_WIDTHS']
        if isinstance(cs_starts, int): cs_starts = [cs_starts]
        if isinstance(cs_widths, int): cs_widths = [cs_widths]
        for i in range(int(meta.get('NUM_CORES', NUM_CORES))):
            my_cs_start = cs_starts[i] // NUM_COL
            my_cs_n = (cs_widths[i] + NUM_COL - 1) // NUM_COL
            base = smc_addr(global_base, i, ofm_offset)
            n_words = h_out * w_out * my_cs_n
            raw = rpc.read_ddr(base, n_words * WORD_BYTES)
            for r in range(h_out):
                for x in range(w_out):
                    base_word_idx = (r * w_out + x) * my_cs_n
                    for local_cs in range(my_cs_n):
                        full_cs = my_cs_start + local_cs
                        src = raw[(base_word_idx + local_cs) * WORD_BYTES : (base_word_idx + local_cs + 1) * WORD_BYTES]
                        take = min(NUM_COL, c_out - full_cs * NUM_COL)
                        out_off = (r * w_out + x) * c_out + full_cs * NUM_COL
                        out[out_off : out_off + take] = src[:take]
    else:  # W slice
        seg_widths = meta[f'SMC_LAYER_{layer_idx}_OFM_SEG_WIDTHS']
        seg_starts = meta[f'SMC_LAYER_{layer_idx}_OFM_SEG_STARTS']
        if isinstance(seg_widths, int): seg_widths = [seg_widths]
        if isinstance(seg_starts, int): seg_starts = [seg_starts]
        for i in range(int(meta.get('NUM_CORES', NUM_CORES))):
            sw, ss = seg_widths[i], seg_starts[i]
            base = smc_addr(global_base, i, ofm_offset)
            n_words = h_out * sw * cout_slices
            raw = rpc.read_ddr(base, n_words * WORD_BYTES)
            for r in range(h_out):
                for x in range(sw):
                    base_word_idx = (r * sw + x) * cout_slices
                    for cs in range(cout_slices):
                        src = raw[(base_word_idx + cs) * WORD_BYTES : (base_word_idx + cs + 1) * WORD_BYTES]
                        take = min(NUM_COL, c_out - cs * NUM_COL)
                        out_off = (r * w_out + (ss + x)) * c_out + cs * NUM_COL
                        out[out_off : out_off + take] = src[:take]
    return bytes(out)


def load_expected_ofm(case_dir: str, meta: dict, layer_idx: int) -> bytes:
    """读 expected_ofm.txt → 整图 NHWC int8 bytes (跟 read_ofm_smc 输出对齐)."""
    h_out = meta[f'LAYER_{layer_idx}_H_OUT']
    w_out = meta[f'LAYER_{layer_idx}_W_OUT']
    c_out = meta[f'LAYER_{layer_idx}_C_OUT']
    cout_slices = (c_out + NUM_COL - 1) // NUM_COL
    ldir_key = f'LAYER_{layer_idx}_DIR'
    fallback = os.path.join(case_dir, 'chain_data', f'layer{layer_idx:02d}')
    if ldir_key in meta and meta[ldir_key]:
        ldir = meta[ldir_key]
        if not os.path.isabs(ldir):
            ldir = os.path.join(case_dir, ldir)
        # case 被移动后, meta 里的绝对路径可能失效, fallback 到 case_dir/chain_data/
        if not os.path.isdir(ldir):
            ldir = fallback
    else:
        ldir = fallback
    words = load_hex_file_words(os.path.join(ldir, 'expected_ofm.txt'))
    out = bytearray(h_out * w_out * c_out)
    for r in range(h_out):
        for x in range(w_out):
            base_word = (r * w_out + x) * cout_slices
            base_byte = (r * w_out + x) * c_out
            for cs in range(cout_slices):
                src = words[base_word + cs]
                take = min(NUM_COL, c_out - cs * NUM_COL)
                out[base_byte + cs * NUM_COL : base_byte + cs * NUM_COL + take] = src[:take]
    return bytes(out)


def compare_ofm(got: bytes, expected: bytes, label: str = "OFM", limit: int = 10) -> int:
    """逐字节比对, 返回 mismatch 数, 打印前 limit 个差异."""
    if len(got) != len(expected):
        print(f"  [{label}] LEN MISMATCH: got {len(got)} expected {len(expected)}")
        return -1
    mm = 0
    for i, (g, e) in enumerate(zip(got, expected)):
        if g != e:
            if mm < limit:
                print(f"  [{label}] byte[{i}] got=0x{g:02x} expected=0x{e:02x}")
            mm += 1
    return mm


# -------------------- 主流程 --------------------
def deploy_and_run(rpc: Vd100Rpc, case_dir: str, override_ifm: bytes = None,
                   verify_layer: int = -1, skip_verify: bool = False,
                   skip_run: bool = False) -> dict:
    """部署 case 到板 + 跑推理 + 读 OFM 校验. 返回 stats dict.

    verify_layer: 比对哪一层 OFM (-1 = 最后一层). 中间层校验需要 read_ofm_smc 改造支持
                  从 SMC_IFM_OFM_BASE 读, 当前只支持 verify_layer=-1 (FINAL OFM).
    skip_verify:  True = 不跟 expected_ofm.txt 比对 (真图模式: expected 是 sim random IFB 算的).
    """
    print(f"== Deploy SMC case: {case_dir} ==")
    meta = parse_meta(case_dir)
    n_layers = meta['NUM_LAYERS']
    n_cores  = meta['NUM_CORES']
    if n_cores != NUM_CORES:
        print(f"  [warn] case n_cores={n_cores} != deploy NUM_CORES={NUM_CORES}, may misbehave")
    global_base = derive_smc_global_base(meta)
    print(f"  n_layers={n_layers}, n_cores={n_cores}, SMC_GLOBAL_BASE=0x{global_base:08x}")
    if global_base < 0x01000000:
        print(f"  [WARN] SMC_GLOBAL_BASE 太低, 会撞 a72 ELF (0x0-0x01000000). 重新生成 case "
              f"with FLUX_SMC_GLOBAL_BASE=0x10000000.")

    # ---- ping 验证连接 ----
    rpc.ping()

    # ---- 1. preload 所有 layer 数据 ----
    print("[1/3] preload SMC mem ...")
    t0 = time.time()
    for l in range(n_layers):
        # IFB only for root layers (layer 0 + ifb_is_root)
        is_root = (l == 0) or (meta.get(f'LAYER_{l}_PRELOAD_IFB', 0) == 1)
        if is_root:
            if l == 0 and override_ifm is not None:
                # 真图模式: override layer 0 IFM
                _load_ifb_override(rpc, override_ifm, meta, global_base)
            else:
                load_ifb_smc(rpc, case_dir, meta, l, global_base)
        load_wb_smc  (rpc, case_dir, meta, l, global_base)
        load_rdma_smc(rpc, case_dir, meta, l, global_base)
        load_desc_smc(rpc, case_dir, meta, l)
        load_sg_cmd_smc(rpc, case_dir, meta, l)
    t_preload = time.time() - t0
    print(f"  preload total {t_preload:.1f}s")

    if skip_run:
        print("[2/3] RUN_LAYERS SKIPPED (--skip-run)")
        return {'preload_s': t_preload, 'cycles': 0}

    # ---- 2. 跑 N 层推理 ----
    print("[2/3] RUN_LAYERS ...")
    layer_cfgs = build_layer_cfgs(meta)
    t0 = time.time()
    cycles = rpc.run_layers(layer_cfgs)
    t_run = time.time() - t0
    print(f"  done, hw cycles={cycles} ({cycles/100e6*1000:.2f} ms @ 100 MHz), "
          f"wall {t_run*1000:.1f} ms")

    # ---- 3. 读 + 校验 OFM ----
    print("[3/3] read OFM ...")
    last_l = n_layers - 1 if verify_layer < 0 else verify_layer
    got = read_ofm_smc(rpc, meta, last_l, global_base)
    expected = None
    mm = -1
    if not skip_verify:
        expected = load_expected_ofm(case_dir, meta, last_l)
        mm = compare_ofm(got, expected, label=f"layer{last_l}")
        if mm == 0:
            print(f"  [OK] layer {last_l} OFM bit-exact ({len(got)} byte)")
        else:
            print(f"  [FAIL] layer {last_l} OFM mismatch: {mm}/{len(got)} byte differ")
    else:
        print(f"  [info] skip_verify=True (真图模式), 读 OFM {len(got)} byte 不比对")

    return {
        'cycles': cycles,
        'preload_s': t_preload,
        'run_s': t_run,
        'mm': mm,
        'ofm': got,
        'expected_ofm': expected,
    }


def _load_ifb_override(rpc: Vd100Rpc, ifm: bytes, meta: dict, global_base: int):
    """真图模式: layer 0 IFM 用 PC 端给的 bytes (NHWC int8)."""
    h_in       = meta['SMC_LAYER_0_H_IN']
    w_in       = meta['SMC_LAYER_0_W_IN']
    cin_slices = meta['SMC_LAYER_0_C_IN_SLICES']
    mode       = meta['SMC_LAYER_0_MODE']
    expected_len = h_in * w_in * cin_slices * NUM_COL
    if len(ifm) != expected_len:
        raise ValueError(f"override IFM len {len(ifm)} != expected {expected_len} "
                         f"(H={h_in} W={w_in} cs={cin_slices})")

    # 转成 word array (cin_slices × 16 byte / pixel)
    if mode in ('A', 'C'):
        my_core = meta['SMC_LAYER_0_MODE_A_CORE']
        base = smc_addr(global_base, my_core, SMC_INPUT_BASE)
        rpc.load_ddr(base, ifm)
        print(f"    IFB[0 override] mode {mode} → mem[{my_core}] @ 0x{base:08x} ({len(ifm)/1024:.1f} KB)")
    else:
        seg_widths = meta['SMC_LAYER_0_IFM_SEG_WIDTHS']
        seg_starts = meta['SMC_LAYER_0_IFM_SEG_STARTS']
        if isinstance(seg_widths, int): seg_widths = [seg_widths]
        if isinstance(seg_starts, int): seg_starts = [seg_starts]
        # gather per-mem
        for i in range(int(meta.get('NUM_CORES', NUM_CORES))):
            sw, ss = seg_widths[i], seg_starts[i]
            base = smc_addr(global_base, i, SMC_INPUT_BASE)
            buf = bytearray()
            for r in range(h_in):
                for x in range(sw):
                    src_byte = (r * w_in + (ss + x)) * cin_slices * NUM_COL
                    buf.extend(ifm[src_byte : src_byte + cin_slices * NUM_COL])
            rpc.load_ddr(base, bytes(buf))
        print(f"    IFB[0 override] W slice → 3 mem ({len(ifm)/1024:.1f} KB total)")


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--case-dir', required=True, help='sim/tb_smc/cases/<name>/ 路径')
    p.add_argument('--vd100-ip', default='169.254.111.10')
    p.add_argument('--port', type=int, default=5000)
    p.add_argument('--repeat', type=int, default=1, help='重复推理 N 次 (复用已 LOAD 数据测吞吐)')
    p.add_argument('--skip-run', action='store_true', help='只 preload 数据, 不 RUN_LAYERS (调试用)')
    args = p.parse_args()

    if not os.path.isdir(args.case_dir):
        sys.exit(f"ERROR: case dir not found: {args.case_dir}")

    print(f"Connecting VD100 @ {args.vd100_ip}:{args.port}")
    with Vd100Rpc(args.vd100_ip, args.port) as rpc:
        stats = deploy_and_run(rpc, args.case_dir, skip_run=args.skip_run)
        if args.repeat > 1 and stats['mm'] == 0:
            print(f"\n=== Repeat × {args.repeat - 1} (复用已 LOAD 数据, 只跑 RUN_LAYERS) ===")
            meta = parse_meta(args.case_dir)
            cfgs = build_layer_cfgs(meta)
            for i in range(args.repeat - 1):
                t0 = time.time()
                cy = rpc.run_layers(cfgs)
                t_rt = time.time() - t0
                print(f"  run #{i+1}: cy={cy} ({cy/100e6*1000:.2f} ms HW), wall {t_rt*1000:.1f} ms")
    return 0 if stats['mm'] == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
