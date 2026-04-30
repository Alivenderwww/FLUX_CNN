"""
run_regression.py -- FLUX CNN 链式回归测试脚本

CASES 用 dict 描述, 字段:
  - name         : 唯一标识 (字符串, 也供 --case 子串过滤)
  - input_src    : '' = 链根 (随机 ifm); 否则 = 上层 case name (上层 ofm 当本层 ifm)
  - shortcut_src : '' = 不开 SDP 残差; 否则 = 上层 case name (上层 ofm 当 shortcut)
  - c_in/c_out/k/h_in/w_in/stride/pad : conv 形状
  - sdp_*        : 该层 SDP 参数 (mult/shift/zp_out/clip_min/clip_max/round_en/relu_en)

链式语义:
  case K 引用 case J (J<K) 的 ofm 作为输入或 shortcut. DDR 上 K.ifb_base = J.ofb_base
  (FM 共享, 由 plan_chain_ddr 分配). 软件层把 J 算出来的 expected_ofm 直接做 K 的 ifm
  参数 (bit-exact, 跟硬件链上一致).

输出:
  regression_report_<ts>.txt 单份报告, sim/tb_core_dma/ 下.
"""

import subprocess
import re
import os
import sys
import shutil
import datetime
import argparse

# Path: 脚本在 toolchain/，仿真在 sim/tb_core_dma/
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR     = os.path.normpath(os.path.join(_SCRIPT_DIR, "..", "sim", "tb_core_dma"))

if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

import hw_files          # noqa: E402
import gen_isa_test      # noqa: E402

# Report 输出到 sim dir（和生成的 .txt 放一起）。
# 默认文件名带时间戳: regression_report_YYYY-MM-DD_HH-MM-SS.txt
OUTPUT_FILE = None

# DDR layout (复用 16MB DDR mock; Patch 单层 ifb=8 MB 把 FM 区撑很大)
DDR_FM_REGION_BASE   = 0x0000_0000
DDR_FM_REGION_LIMIT  = 0x00C0_0000   # 12 MB FM 区 (Patch ifb 单独占 ~8 MB)
DDR_WB_REGION_BASE   = 0x00C0_0000
DDR_WB_REGION_LIMIT  = 0x00D0_0000   # 1 MB WB 区
DDR_RDMA_REGION_BASE = 0x00D0_0000
DDR_RDMA_REGION_LIMIT= 0x00F0_0000   # 2 MB RDMA 区 (bias + shortcut)
DDR_DESC_REGION_BASE = 0x00F0_0000
DDR_DESC_REGION_LIMIT= 0x0100_0000
ALIGN_BYTES          = 0x1000        # 4KB
AXI_BYTES            = 16
DESC_BYTES_RESERVED  = 0x8000

# ---------------------------------------------------------------------------
# Chain builder DSL: 链式书写 CASES 的 sugar.
#   - chain.conv(...) / node.conv(...) 自动从父节点推 h_in/w_in/c_in
#   - shortcut=node 用 Python 变量引用 (类型安全, 不用字符串名)
#   - SDP 默认 clip[0,127] / mult=1 / round=1 / relu=1; 只 shift 通常要改
#
# 用法:
#   chain  = Chain(h=960, w=540, c=4)
#   patch  = chain.conv('Patch', k=4, c=16, s=4, pad=0, shift=5)
#   l1_out = resnet_block(patch, 'L1', c=16, shifts=(11,11,7))
#   ...
#   fc     = chain.root('FC', h=1, w=1, c_in=256, c_out=522, k=1, shift=8)
#   CASES  = chain.cases
# ---------------------------------------------------------------------------

class _Node:
    """链上一层的 handle. 持有指向 case dict 的引用 + 计算好的 H_OUT/W_OUT."""
    def __init__(self, chain, case_dict, h_out, w_out):
        self._chain = chain
        self._case  = case_dict
        self.h_out  = h_out
        self.w_out  = w_out

    @property
    def name(self):  return self._case['name']
    @property
    def c_out(self): return self._case['c_out']

    def conv(self, name, k, c, s=1, pad=None, *, shift=0,
             clip_max=127, clip_min=0, zp_out=0, mult=1,
             round_en=1, relu_en=1, shortcut=None):
        return self._chain._add(name=name, input_node=self, k=k, c_out=c,
                                 stride=s, pad=pad, shift=shift,
                                 clip_max=clip_max, clip_min=clip_min,
                                 zp_out=zp_out, mult=mult,
                                 round_en=round_en, relu_en=relu_en,
                                 shortcut=shortcut)


class Chain:
    """整条链的容器. 持有 cases list (最终给 validate_chain / plan_chain_ddr 用)."""
    def __init__(self, h, w, c):
        self.cases = []         # list[dict] 按顺序
        self._initial = (h, w, c)

    def conv(self, name, k, c, s=1, pad=None, **kw):
        """从 chain 初始 (h, w, c_in) 起始的链根."""
        h, w, c_in = self._initial
        return self.root(name=name, k=k, c_out=c, s=s, pad=pad,
                          h=h, w=w, c_in=c_in, **kw)

    def root(self, name, k, c_out, h, w, c_in, s=1, pad=None, *, shift=0,
             clip_max=127, clip_min=0, zp_out=0, mult=1,
             round_en=1, relu_en=1, shortcut=None):
        """显式指定输入维度的链根 (例如 FC 输入跟前面链对不上)."""
        # 临时建一个伪 node 来描述输入 (不进 cases list)
        fake = _Node.__new__(_Node)
        fake._chain = self
        fake._case  = {'name': '', 'c_out': c_in}
        fake.h_out  = h
        fake.w_out  = w
        return self._add(name=name, input_node=None, _root_input=fake,
                          k=k, c_out=c_out, stride=s, pad=pad, shift=shift,
                          clip_max=clip_max, clip_min=clip_min,
                          zp_out=zp_out, mult=mult,
                          round_en=round_en, relu_en=relu_en,
                          shortcut=shortcut)

    def _add(self, name, input_node, k, c_out, stride, pad, shift,
             clip_max, clip_min, zp_out, mult, round_en, relu_en,
             shortcut, _root_input=None):
        # 输入维度: input_node (链式) 或 _root_input (root 自带)
        in_node = input_node if input_node is not None else _root_input
        h_in, w_in, c_in = in_node.h_out, in_node.w_out, in_node.c_out
        if pad is None:
            pad = (k - 1) // 2 if k > 1 else 0
        h_out = (h_in + 2 * pad - k) // stride + 1
        w_out = (w_in + 2 * pad - k) // stride + 1
        case = {
            'name': name,
            'input_src'   : input_node.name if input_node is not None else '',
            'shortcut_src': shortcut.name   if shortcut   is not None else '',
            'c_in': c_in, 'c_out': c_out, 'k': k,
            'h_in': h_in, 'w_in': w_in,
            'stride': stride, 'pad': pad,
            'sdp_mult': mult, 'sdp_shift': shift, 'sdp_zp_out': zp_out,
            'sdp_clip_min': clip_min, 'sdp_clip_max': clip_max,
            'sdp_round_en': round_en, 'sdp_relu_en': relu_en,
        }
        self.cases.append(case)
        return _Node(self, case, h_out, w_out)


def resnet_block(node, name, c, shifts, ds_stride=2):
    """ResNet block: B1.C1 → B1.C2 + (parallel) B2.ds  → 残差融合.
    shifts = (c1_shift, c2_shift, ds_shift). 返回 ds (= block 的输出)."""
    c1_sh, c2_sh, ds_sh = shifts
    c1 = node.conv(f'{name}.B1.C1', k=3, c=c, s=ds_stride, pad=1, shift=c1_sh)
    c2 = c1  .conv(f'{name}.B1.C2', k=3, c=c, s=1,         pad=1, shift=c2_sh)
    ds = node.conv(f'{name}.B2.ds', k=1, c=c, s=ds_stride, pad=0,
                   shift=ds_sh, shortcut=c2)
    return ds


# ---------------------------------------------------------------------------
# CASES 定义 (链式 builder)
#
# 网络: ResNet-18-like (输入 960×540×4)
#   Patch embed (4×4, s=4)               → 240×135×16
#   L1 (16→16, ds=2): B1.C1+B1.C2+B2.ds  → 120×68×16
#   L2 (16→32, ds=2)                     → 60×34×32
#   L3 (32→64, ds=2)                     → 30×17×64
#   FC merged (256→522, 1×1)             [独立 root, 没 AvgPool 硬件]
#
# SDP 默认: clip[0,127] + ReLU on (输出落 INT8 非负范围, 下层 MAC 当符号读 ok),
#          mult=1, round=1; per-layer shift 按 K²·Cin·psum_max 粗估.
# ---------------------------------------------------------------------------
_chain = Chain(h=960, w=540, c=4)
_patch = _chain.conv     ('Patch',  k=4, c=16, s=4, pad=0, shift=5)
_l1    = resnet_block    (_patch, 'L1', c=16, shifts=(11, 11, 7))
_l2    = resnet_block    (_l1,    'L2', c=32, shifts=(11, 12, 7))
_l3    = resnet_block    (_l2,    'L3', c=64, shifts=(12, 13, 8))
_fc    = _chain.root     ('FC',  k=1, c_in=256, c_out=522, h=1, w=1, shift=8)

# 鲁棒性 corner case (用独立 root, 不依赖前面 chain): 覆盖 K/stride/pad/cin/cout/h/w 边界
# 每个 case 用自己的 random IFM (gen_isa_test seed 互不相同)
_chain.root('R.K1_C16',     k=1, c_out=16, c_in=16,  h=16,  w=16, pad=0, shift=2)
_chain.root('R.K2_C16',     k=2, c_out=16, c_in=16,  h=16,  w=16, pad=0, shift=3)
_chain.root('R.K5_C16',     k=5, c_out=16, c_in=16,  h=16,  w=16, pad=2, shift=7)
_chain.root('R.K7_C16',     k=7, c_out=16, c_in=16,  h=16,  w=16, pad=3, shift=8)
# _chain.root('R.K3S3',     k=3, ..., s=3) -- skipped: stride>=3 触发 chain 的 force_s2d, 跟原意 (不 fold) 冲突
_chain.root('R.K3S2',       k=3, c_out=16, c_in=16,  h=32,  w=32, s=2, pad=1, shift=5)
_chain.root('R.K5S2',       k=5, c_out=16, c_in=16,  h=32,  w=32, s=2, pad=2, shift=6)
_chain.root('R.Cin4',       k=3, c_out=16, c_in=4,   h=16,  w=16, pad=1, shift=4)
_chain.root('R.Cin8',       k=3, c_out=16, c_in=8,   h=16,  w=16, pad=1, shift=4)
_chain.root('R.Cin12',      k=3, c_out=16, c_in=12,  h=16,  w=16, pad=1, shift=4)
_chain.root('R.Cout24',     k=3, c_out=24, c_in=16,  h=16,  w=16, pad=1, shift=5)
_chain.root('R.Cout32',     k=3, c_out=32, c_in=16,  h=16,  w=16, pad=1, shift=5)
_chain.root('R.H15W17',     k=3, c_out=16, c_in=16,  h=15,  w=17, pad=1, shift=5)
_chain.root('R.H33W33',     k=3, c_out=16, c_in=16,  h=33,  w=33, pad=1, shift=5)
_chain.root('R.H64W32',     k=3, c_out=16, c_in=16,  h=64,  w=32, pad=1, shift=5)  # OFB ring boundary
_chain.root('R.1x1FC',      k=1, c_out=32, c_in=64,  h=1,   w=1,  pad=0, shift=6)

CASES = _chain.cases


# ---------------------------------------------------------------------------
# Chain validation: dim 匹配 + 引用解析 + 命名唯一
# ---------------------------------------------------------------------------
def validate_chain(cases):
    """检查 input_src/shortcut_src 引用维度匹配, 且只能引用更早的 case."""
    seen = {}
    for i, c in enumerate(cases):
        nm = c['name']
        if nm in seen:
            raise ValueError(f"重复 case 名: '{nm}' (case {seen[nm]} 和 {i})")
        seen[nm] = i

        # input_src 维度匹配 (上游 H_OUT/W_OUT/c_out == 本层 h_in/w_in/c_in)
        if c['input_src']:
            if c['input_src'] not in seen:
                raise ValueError(f"case '{nm}': input_src='{c['input_src']}' "
                                 f"未定义或不在更早的 case 中")
            src = cases[seen[c['input_src']]]
            sH, sW = _layer_out_dims(src)
            if (sH, sW, src['c_out']) != (c['h_in'], c['w_in'], c['c_in']):
                raise ValueError(
                    f"case '{nm}': input_src='{c['input_src']}' 维度不匹配: "
                    f"上层 out={sH}x{sW}x{src['c_out']} vs 本层 in={c['h_in']}x{c['w_in']}x{c['c_in']}")

        # shortcut_src 维度匹配 (上游 H_OUT/W_OUT/c_out == 本层 H_OUT/W_OUT/c_out)
        if c['shortcut_src']:
            if c['shortcut_src'] not in seen:
                raise ValueError(f"case '{nm}': shortcut_src='{c['shortcut_src']}' "
                                 f"未定义或不在更早的 case 中")
            src = cases[seen[c['shortcut_src']]]
            sH, sW = _layer_out_dims(src)
            cH, cW = _layer_out_dims(c)
            if (sH, sW, src['c_out']) != (cH, cW, c['c_out']):
                raise ValueError(
                    f"case '{nm}': shortcut_src='{c['shortcut_src']}' 维度不匹配: "
                    f"上层 out={sH}x{sW}x{src['c_out']} vs 本层 out={cH}x{cW}x{c['c_out']}")


def _layer_out_dims(c):
    """从 case 字段算 H_OUT/W_OUT (对称 pad)."""
    H_OUT = (c['h_in'] + 2 * c['pad'] - c['k']) // c['stride'] + 1
    W_OUT = (c['w_in'] + 2 * c['pad'] - c['k']) // c['stride'] + 1
    return H_OUT, W_OUT


# ---------------------------------------------------------------------------
# DDR 布局: 每个链根独占输入 FM, 链内每层 ifb_base = 父层 ofb_base
# ---------------------------------------------------------------------------
def _align(x, a=ALIGN_BYTES):
    return (x + a - 1) & ~(a - 1)


def plan_chain_ddr(cases, hw_pe=16, hw_col=16):
    """
    返回 list[dict]:
      {ifb_base, ofb_base, wb_base, desc_base, rdma_base}  per case (byte address)

    FM 共享:
      - 链根 (input_src=''): 自己拿一段 FM 区放输入 (大小=ifb_words×16 byte)
      - 非根: ifb_base = parent.ofb_base (FM 共享)
      所有 case 都新分配一段 FM 区放自己的 OFB (用 fresh slot, FM_idx++)
    WB / DESC / RDMA: 每 case 独立, 顺序分配
    """
    name_to_idx = {c['name']: i for i, c in enumerate(cases)}

    # 第一步: 为每个 case 分配一个 OFB FM slot
    fm_cur = DDR_FM_REGION_BASE
    fm_slots = {}      # case_idx -> ofb_base
    fm_input_slots = {}   # case_idx -> 链根的输入 FM base (仅根)
    for i, c in enumerate(cases):
        if not c['input_src']:
            # 根 case 需要一段输入 FM
            cin_slices = (c['c_in'] + hw_pe - 1) // hw_pe
            ifb_size = c['h_in'] * c['w_in'] * cin_slices * AXI_BYTES
            fm_input_slots[i] = fm_cur
            fm_cur = _align(fm_cur + ifb_size)

        # 每 case 自己的 OFB FM slot
        cout_slices = (c['c_out'] + hw_col - 1) // hw_col
        H_OUT, W_OUT = _layer_out_dims(c)
        ofb_size = H_OUT * W_OUT * cout_slices * AXI_BYTES
        fm_slots[i] = fm_cur
        fm_cur = _align(fm_cur + ofb_size)

    if fm_cur > DDR_FM_REGION_LIMIT:
        raise ValueError(f"FM chain {fm_cur:#x} > limit {DDR_FM_REGION_LIMIT:#x}")

    # 第二步: WB / DESC / RDMA 按 case 顺序独立分配
    plan = []
    wb_cur   = DDR_WB_REGION_BASE
    desc_cur = DDR_DESC_REGION_BASE
    rdma_cur = DDR_RDMA_REGION_BASE
    for i, c in enumerate(cases):
        # ifb_base: 根 = 自己的 input slot, 非根 = 父层 ofb_base
        if c['input_src']:
            ifb_base = fm_slots[name_to_idx[c['input_src']]]
        else:
            ifb_base = fm_input_slots[i]

        # cfg 估算 wb / rdma 大小
        cin_slices  = (c['c_in']  + hw_pe  - 1) // hw_pe
        cout_slices = (c['c_out'] + hw_col - 1) // hw_col
        kk = c['k'] * c['k']
        wb_words = kk * cout_slices * cin_slices         # 每 word = 2048 bit = 256 byte
        wb_bytes = wb_words * 256

        # rdma: bias section (cout_slices*4 个 128-bit) + 可选 shortcut (H_OUT*W_OUT*cout_slices)
        rdma_words = cout_slices * 4
        if c['shortcut_src']:
            H_OUT, W_OUT = _layer_out_dims(c)
            rdma_words += H_OUT * W_OUT * cout_slices
        rdma_bytes = rdma_words * 16

        plan.append({
            'ifb_base' : ifb_base,
            'ofb_base' : fm_slots[i],
            'wb_base'  : wb_cur,
            'desc_base': desc_cur,
            'rdma_base': rdma_cur,
        })
        wb_cur   = _align(wb_cur   + wb_bytes)
        desc_cur = _align(desc_cur + DESC_BYTES_RESERVED)
        rdma_cur = _align(rdma_cur + rdma_bytes)

    if wb_cur > DDR_WB_REGION_LIMIT:
        raise ValueError(f"WB chain {wb_cur:#x} > limit {DDR_WB_REGION_LIMIT:#x}")
    if desc_cur > DDR_DESC_REGION_LIMIT:
        raise ValueError(f"DESC chain {desc_cur:#x} > limit {DDR_DESC_REGION_LIMIT:#x}")
    if rdma_cur > DDR_RDMA_REGION_LIMIT:
        raise ValueError(f"RDMA chain {rdma_cur:#x} > limit {DDR_RDMA_REGION_LIMIT:#x}")
    return plan


# ---------------------------------------------------------------------------
# 单 case 超时估算 (ns) -- 用于 watchdog
# ---------------------------------------------------------------------------
def estimate_case_timeout_ns(c):
    h_out, w_out = _layer_out_dims(c)
    cin_tiles  = max(1, (c['c_in']  + 15) // 16)
    cout_tiles = max(1, (c['c_out'] + 15) // 16)
    cycles = h_out * w_out * cin_tiles * cout_tiles * 100 + 500_000
    return cycles * 10 * 5


# ---------------------------------------------------------------------------
# 链式 case 生成: 在进程内调 gen_isa_test.generate_random, 链式传 ofm
# ---------------------------------------------------------------------------
def gen_chained_cases(cases, plan, ky_fold_global=False, s2d_global=False):
    """
    顺序调 generate_random 生成每个 case. 用 ofm_cache 把上层 ofm 当下层 ifm.
    返回 dict:
      idx  -> {case_dir, opt_tag, h_out, w_out}
    """
    ofm_cache = {}    # name -> ofm_arr  (生成完后保存供下游用)
    case_meta = {}    # idx  -> {case_dir, opt_tag, h_out, w_out}

    name_to_idx = {c['name']: i for i, c in enumerate(cases)}
    n = len(cases)

    for i, c in enumerate(cases):
        case_dir = os.path.join(SIM_DIR, "cases", f"case{i:02d}")
        os.makedirs(case_dir, exist_ok=True)

        # 链上输入 ifm: 父层 ofm. 根 case ifm_arr_in=None → generate_random 自己随机.
        ifm_arr_in = None
        skip_ifb_preload = False
        if c['input_src']:
            ifm_arr_in = ofm_cache[c['input_src']]
            skip_ifb_preload = True   # 该层 ifb 已由父层 OFB 写到 DDR, TB 不要 readmemh 覆盖

        # shortcut: 父层 ofm
        shortcut_arr_in = None
        if c['shortcut_src']:
            shortcut_arr_in = ofm_cache[c['shortcut_src']]

        # S2D / Ky-fold: 跟旧逻辑一样, 按 stride/Cin 决定;
        # 但 stride>=3 line_buffer 不原生支持 → 强制 s2d (即使 c_in>=16)
        force_s2d = c['stride'] >= 3 and c['k'] >= c['stride']
        use_s2d = force_s2d or (
            s2d_global and c['stride'] >= 2 and c['k'] >= c['stride']
            and c['c_in'] < 16)
        c_in_eff = (c['stride']*c['stride']*c['c_in']) if use_s2d else c['c_in']
        k_eff    = ((c['k'] + c['stride'] - 1) // c['stride']) if use_s2d else c['k']
        use_ky   = ky_fold_global and (k_eff > 1) and (c_in_eff < 16)

        opt_tags = []
        if use_s2d: opt_tags.append('s2d')
        if use_ky:  opt_tags.append('ky')
        if c['shortcut_src']: opt_tags.append('res')
        if c['input_src']:    opt_tags.append('chn')
        opt_tag = '+'.join(opt_tags) if opt_tags else '-'

        # case-name 里不能有空格 (TB $sscanf %s 在空格停); 用 _ 替代
        full_name = f"{c['name'].replace(' ', '_')}|conv"

        bases = plan[i]
        try:
            ret = gen_isa_test.generate_random(
                H_IN=c['h_in'], W_IN=c['w_in'], K=c['k'],
                NUM_CIN=c['c_in'], NUM_COUT=c['c_out'],
                TILE_W=32, seed=42 + i,
                shift_amt=c['sdp_shift'], stride=c['stride'],
                HW_PE=16, HW_COL=16,
                streaming=True,
                pad_top=c['pad'], pad_left=c['pad'],
                strip_rows=0,
                out_dir=case_dir, case_name=full_name,
                ky_fold=use_ky, s2d=use_s2d,
                residual=bool(c['shortcut_src']),
                ifm_arr_in=ifm_arr_in,
                shortcut_arr_in=shortcut_arr_in,
                sdp_mult     =c['sdp_mult'],
                sdp_zp_out   =c['sdp_zp_out'],
                sdp_clip_min =c['sdp_clip_min'],
                sdp_clip_max =c['sdp_clip_max'],
                sdp_round_en =c['sdp_round_en'],
                sdp_relu_en  =c['sdp_relu_en'],
                shortcut_mult =1 if c['shortcut_src'] else 0,
                shortcut_shift=0 if c['shortcut_src'] else 0,
                ddr_ifb_base =bases['ifb_base'],
                ddr_wb_base  =bases['wb_base'],
                ddr_ofb_base =bases['ofb_base'],
                ddr_desc_base=bases['desc_base'],
                ddr_rdma_base=bases['rdma_base'],
                skip_ifb_preload=skip_ifb_preload,
                skip_ofb_clear  =False,
            )
        except (ValueError, AssertionError) as e:
            print(f"  case {i} ({c['name']}) gen ERROR: {e}")
            sys.exit(1)

        ofm_cache[c['name']] = ret['ofm_arr']
        case_meta[i] = {
            'case_dir': case_dir,
            'opt_tag' : opt_tag,
            'h_out'   : ret['H_OUT'],
            'w_out'   : ret['W_OUT'],
        }
        print(f"  [{i+1}/{n}] {c['name']:<12} → {ret['H_OUT']}x{ret['W_OUT']}x{c['c_out']:<4} "
              f"[{opt_tag}]  ifb={bases['ifb_base']:#08x} ofb={bases['ofb_base']:#08x}")

    return case_meta


# ---------------------------------------------------------------------------
# 解析单次 vsim 日志里的所有 CASE_RESULT 行
# ---------------------------------------------------------------------------
CASE_RE = re.compile(
    r"CASE_RESULT\s+(\d+)\s+(PASS|FAIL)\s+cycles=(\d+)"
    r"(?:\s+mac_fire=(\d+))?"
    r"(?:\s+mac_util=([\d.]+)%)?"
    r"(?:\s+arf_w=(\d+))?"
    r"(?:\s+arf_r=(\d+))?"
    r"(?:\s+parf_f=(\d+))?"
    r"(?:\s+parf_d=(\d+))?"
    r"(?:\s+ifb_r=(\d+))?"
    r"(?:\s+wb_r=(\d+))?"
    r"(?:\s+ofb_w=(\d+))?"
    r"(?:\s+wall_ns=(\d+))?"
    r"(?:\s+mismatches=(\d+))?"
    r"\s+name=([^\n]+)")

PROFILE_RE = re.compile(
    r"CASE_PROFILE\s+(\d+)\s+cycles=(\d+)"
    r"\s+act_fire=(\d+)\s+act_stall=(\d+)\s+act_idle=(\d+)"
    r"\s+wgt_fire=(\d+)\s+wgt_stall=(\d+)\s+wgt_idle=(\d+)"
    r"\s+psum_fire=(\d+)\s+psum_stall=(\d+)\s+psum_idle=(\d+)"
    r"\s+acc_fire=(\d+)\s+acc_stall=(\d+)\s+acc_idle=(\d+)"
    r"\s+name=([^\n]+)")


def parse_sim_log(text):
    results = {}
    for line in text.splitlines():
        m = CASE_RE.search(line)
        if m:
            idx = int(m.group(1))
            results.setdefault(idx, {})
            results[idx].update({
                "idx":        idx,
                "passed":     m.group(2) == "PASS",
                "cycles":     int(m.group(3)),
                "mac_fire":   int(m.group(4) or 0),
                "mac_util":   float(m.group(5) or 0.0),
                "arf_w":      int(m.group(6) or 0),
                "arf_r":      int(m.group(7) or 0),
                "parf_f":     int(m.group(8) or 0),
                "parf_d":     int(m.group(9) or 0),
                "ifb_r":      int(m.group(10) or 0),
                "wb_r":       int(m.group(11) or 0),
                "ofb_w":      int(m.group(12) or 0),
                "wall_ns":    int(m.group(13) or 0),
                "name":       m.group(15).strip(),
                "mismatches": int(m.group(14) or 0),
            })
            continue
        mp = PROFILE_RE.search(line)
        if mp:
            idx = int(mp.group(1))
            results.setdefault(idx, {})
            results[idx].update({
                "act_fire":   int(mp.group(3)),  "act_stall":  int(mp.group(4)),  "act_idle":   int(mp.group(5)),
                "wgt_fire":   int(mp.group(6)),  "wgt_stall":  int(mp.group(7)),  "wgt_idle":   int(mp.group(8)),
                "psum_fire":  int(mp.group(9)),  "psum_stall": int(mp.group(10)), "psum_idle":  int(mp.group(11)),
                "acc_fire":   int(mp.group(12)), "acc_stall":  int(mp.group(13)), "acc_idle":   int(mp.group(14)),
            })
    return [results[i] for i in sorted(results.keys())]


# ---------------------------------------------------------------------------
# 报告生成
# ---------------------------------------------------------------------------
def write_report(results, case_opts=None, label=""):
    lines = []
    def L(s=""): lines.append(s)

    def split_name(r):
        if "|" in r["name"]:
            return r["name"].rsplit("|", 1)[0]
        return r["name"]

    def opt_of(r):
        if case_opts is None:
            return "-"
        return case_opts.get(r["idx"], "-")

    case_w = max([len(split_name(r)) for r in results] + [len("Case")])
    opt_w  = max([len(opt_of(r))     for r in results] + [len("Opt"), 6])

    total_cycles  = sum(r['cycles']  for r in results)
    total_wall_ns = sum(r['wall_ns'] for r in results)
    case_waste    = {r['idx']: (1.0 - r['mac_util']/100.0) * r['cycles'] for r in results}
    total_waste   = sum(case_waste.values())
    avg_mac_pct   = 100.0 * (total_cycles - total_waste) / max(total_cycles, 1)
    sum_ifb_r     = sum(r['ifb_r']   for r in results)
    sum_ofb_w     = sum(r['ofb_w']   for r in results)
    sum_arf_w     = sum(r['arf_w']   for r in results)
    sum_arf_r     = sum(r['arf_r']   for r in results)
    sum_parf_f    = sum(r['parf_f']  for r in results)
    sum_parf_d    = sum(r['parf_d']  for r in results)
    avg_ratio     = (sum_arf_r / sum_arf_w) if sum_arf_w > 0 else 0.0

    hdr = (f"  {'Case':<{case_w}} {'Opt':^{opt_w}} {'Res':^5} "
           f"{'Cycles':>11} {'Wall_us':>10} {'Cyc%':>6} {'MAC%':>6} {'Wst%':>6}  "
           f"{'IFB_R':>11} {'WB_R':>9} {'OFB_W':>11}  "
           f"{'ARF_W':>11} {'ARF_R':>11} {'Ratio':>6}  "
           f"{'PARF_F':>11} {'PARF_D':>11}")

    bar_w = max(len(hdr), 140)
    SEP = "=" * bar_w
    SUB = "-" * bar_w

    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    L(SEP)
    L(f"  FLUX CNN Regression (chained CASES)  {label}")
    L(f"  Generated: {now}")
    L(SEP)
    L()
    L(hdr)
    L(SUB)
    all_pass = True
    for r in results:
        status = "PASS" if r["passed"] else "FAIL"
        if not r["passed"]:
            all_pass = False
        cname   = split_name(r)
        opt     = opt_of(r)
        ratio   = (r['arf_r'] / r['arf_w']) if r['arf_w'] > 0 else 0.0
        cyc_pct = 100.0 * r['cycles'] / max(total_cycles, 1)
        wst_pct = (100.0 * case_waste[r['idx']] / total_waste) if total_waste > 0 else 0.0
        wall_us = r['wall_ns'] / 1000.0
        row = (f"  {cname:<{case_w}} {opt:^{opt_w}} {status:^5} "
               f"{r['cycles']:>11,} {wall_us:>10,.2f} {cyc_pct:>5.1f}% {r['mac_util']:>5.1f}% {wst_pct:>5.1f}%  "
               f"{r['ifb_r']:>11,} {'-':>9} {r['ofb_w']:>11,}  "
               f"{r['arf_w']:>11,} {r['arf_r']:>11,} {ratio:>5.2f}x  "
               f"{r['parf_f']:>11,} {r['parf_d']:>11,}")
        L(row)
    L(SUB)
    total_wall_us = total_wall_ns / 1000.0
    total_row = (f"  {'TOTAL':<{case_w}} {'':<{opt_w}} {'':<5} "
                 f"{total_cycles:>11,} {total_wall_us:>10,.2f} {'100.0%':>6} {avg_mac_pct:>5.1f}% {'100.0%':>6}  "
                 f"{sum_ifb_r:>11,} {'-':>9} {sum_ofb_w:>11,}  "
                 f"{sum_arf_w:>11,} {sum_arf_r:>11,} {avg_ratio:>5.2f}x  "
                 f"{sum_parf_f:>11,} {sum_parf_d:>11,}")
    L(total_row)
    L(SUB)
    L("  Opt: 编译器对该层做的特殊处理. -=baseline; s2d=Space-to-Depth; ky=Ky-fold;")
    L("       chn=链式 (上层 ofm 当本层 ifm); res=SDP 残差 (上层 ofm 当 shortcut). 多个 + 拼")
    L("  Wall_us: 单层端到端墙钟 (µs) = host 启动 setup → host 可读 DDR 最终 OFB.")
    L("           = boot AXI-Lite 写 + DFE 拉 desc + sequencer CFG_WRITE + CONV + ODMA flush.")
    L("           Cycles 是 layer_busy 期间核内拍数, 不含 host 写 boot regs / DFE 拉 desc 的开销.")
    L("  MAC%: useful_mac_ops / peak_mac_ops; useful = H_out×W_out×K²×Cin×Cout (pre-fold原始 conv)")
    L("        peak = cycles×NUM_COL×NUM_PE. Cout<16 (PE 列空转) + Ky-fold pad 都会扣利用率")
    L("  Cyc%: 该 case cycles 占总 cycles 比例.  Wst%: 该 case MAC 浪费占总浪费比例 (定位瓶颈层)")
    L()
    # 网络端到端延迟 (host 视角): 从启第一层到能读最后一层 DDR 输出
    L(SEP)
    L(f"  整网端到端墙钟 (host 启动 → host 可从 DDR 取最终结果):  "
      f"{total_wall_us:,.2f} µs  ({total_wall_ns:,} ns @10ns/cycle)")
    L(f"  对应实时 FPS 上限 (理想, 无 host 后处理 / 下帧准备):  "
      f"{1_000_000.0 / max(total_wall_us, 0.001):,.1f} fps")
    L(SEP)
    L()

    if any('act_fire' in r for r in results):
        L(SEP)
        L("  Handshake PROFILE — 各 V/R 接口 {fire / stall / idle} 占总 cycle %")
        L(SEP)
        L()
        hdr2 = (f"  {'Case':<{case_w}} {'Cycles':>11}  "
                f"{'act F/S/I':>22}  "
                f"{'wgt F/S/I':>22}  "
                f"{'psum F/S/I':>22}  "
                f"{'acc F/S/I':>22}")
        L(hdr2)
        L(SUB)
        for r in results:
            if 'act_fire' not in r:
                continue
            cname = split_name(r)
            cy = max(r['cycles'], 1)
            def fsi(prefix):
                f = 100.0 * r[f'{prefix}_fire']  / cy
                s = 100.0 * r[f'{prefix}_stall'] / cy
                i = 100.0 * r[f'{prefix}_idle']  / cy
                return f"{f:>5.1f}/{s:>5.1f}/{i:>5.1f}"
            row = (f"  {cname:<{case_w}} {r['cycles']:>11,}  "
                   f"{fsi('act'):>22}  "
                   f"{fsi('wgt'):>22}  "
                   f"{fsi('psum'):>22}  "
                   f"{fsi('acc'):>22}")
            L(row)
        L(SUB)
        L()

    L(SEP)
    L(f"  Overall: {'All PASS [OK]' if all_pass else 'Some FAILED [!!]'}  ({len(results)} cases)")
    L(SEP)

    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    return all_pass


# ---------------------------------------------------------------------------
# 主入口
# ---------------------------------------------------------------------------
def main():
    global OUTPUT_FILE
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", default="", help="Report label")
    parser.add_argument("--out",   default=None,
                        help="Output file (默认: regression_report_<timestamp>.txt)")
    parser.add_argument("--case",  default=None,
                        help="只跑 name 包含此子串的 case (链上游的依赖也自动包含). "
                             "例: --case L1.B1")
    parser.add_argument("--timeout-ns", type=int, default=0,
                        help="vsim watchdog 超时 (ns); 0=按每 case 尺寸自动估算")
    parser.add_argument("--fold", action="store_true",
                        help="对所有 K>1 且 Cin<16 的 case 启用 Ky fold")
    parser.add_argument("--s2d", action="store_true",
                        help="对所有 stride>=2 且 K>=stride 的 case 启用 Space-to-Depth")
    args = parser.parse_args()
    if args.out is not None:
        OUTPUT_FILE = args.out
    else:
        ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        OUTPUT_FILE = os.path.join(SIM_DIR, f"regression_report_{ts}.txt")

    # ---- Step 0: validate ----
    print("=" * 60)
    print(f"  FLUX CNN Chained Regression  {args.label}")
    print("=" * 60)
    print(f"\n[Step 0] 校验 CASE 链 ({len(CASES)} cases) ...")
    validate_chain(CASES)
    print("  chain ok")

    # ---- 选 case (含上游依赖) ----
    cases = list(CASES)
    if args.case is not None:
        sel_names = set()
        for c in cases:
            if args.case in c['name']:
                sel_names.add(c['name'])
        if not sel_names:
            print(f"  no case name contains '{args.case}'; aborted.")
            sys.exit(1)
        # 闭包: 把上游依赖也拉进来
        added = True
        while added:
            added = False
            for c in cases:
                if c['name'] in sel_names:
                    for src in (c['input_src'], c['shortcut_src']):
                        if src and src not in sel_names:
                            sel_names.add(src)
                            added = True
        cases = [c for c in cases if c['name'] in sel_names]
    n_cases = len(cases)
    print(f"  selected {n_cases} cases")

    # ---- Step 1: DDR 布局 ----
    print(f"\n[Step 1] 分配 DDR 链布局 ...")
    plan = plan_chain_ddr(cases)

    # ---- Step 2: 生成每 case 数据 (链式) ----
    print(f"\n[Step 2] 生成 {n_cases} 个 case 数据 (链式) ...")
    case_meta = gen_chained_cases(cases, plan,
                                   ky_fold_global=args.fold,
                                   s2d_global=args.s2d)
    case_opts = {i: case_meta[i]['opt_tag'] for i in case_meta}

    # 把 case0 的 sim_params.f 复制到 SIM_DIR/ 给 vsim 启动
    src = os.path.join(SIM_DIR, "cases", "case00", "sim_params.f")
    dst = os.path.join(SIM_DIR, "sim_params.f")
    shutil.copy(src, dst)

    if args.timeout_ns > 0:
        timeout_ns = args.timeout_ns
        t_src = "user --timeout-ns"
    else:
        est_total = sum(estimate_case_timeout_ns(c) for c in cases)
        timeout_ns = est_total + 100_000_000
        t_src = "auto-est"

    with open(dst, "a", encoding="utf-8") as f:
        f.write(f"+N_CASES={n_cases}\n")
        f.write(f"+TIMEOUT_NS={timeout_ns}\n")

    print(f"  watchdog timeout = {timeout_ns:,} ns "
          f"(~{timeout_ns // 10:,} cycles @10 ns)  [{t_src}]")

    # ---- Step 3: 单次 vsim 跑全部 case ----
    print(f"\n[Step 3] 启动单次 vsim (N_CASES={n_cases}) ...")
    r_sim = subprocess.run("vsim -c -do run.tcl", shell=True,
                           capture_output=True, text=True, cwd=SIM_DIR)
    out = r_sim.stdout + r_sim.stderr
    if r_sim.returncode != 0 and "ALL " not in out:
        print("  vsim failed to finish normally.")

    with open(os.path.join(SIM_DIR, "vsim_all.log"), "w", encoding="utf-8", errors="replace") as f:
        f.write(out)

    # ---- Step 4: parse 日志 ----
    print(f"\n[Step 4] 解析 vsim 日志 ...")
    results = parse_sim_log(out)
    if len(results) != n_cases:
        print(f"  WARNING: expected {n_cases} CASE_RESULT, got {len(results)}")
        missing = set(range(n_cases)) - set(r['idx'] for r in results)
        if missing:
            print(f"  missing case idx: {sorted(missing)}")

    for r in results:
        status = "PASS" if r["passed"] else "FAIL"
        print(f"  [{r['idx']:>2}] {status}  cycles={r['cycles']:>9,}  "
              f"MAC={r['mac_util']:>5.1f}%  {r['name']}")

    all_pass = write_report(results, case_opts=case_opts, label=args.label)

    print(f"\n报告已写入: {OUTPUT_FILE}")
    print(f"总体结论  : {'All PASS [OK]' if all_pass else 'Some FAILED [!!]'}")
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
