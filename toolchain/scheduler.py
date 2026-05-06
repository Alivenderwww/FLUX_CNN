"""
scheduler.py  --  多核调度器 (M2.5)

输入: layer 序列 (从 PyTorch model 或 Chain DSL 获取)
输出: per-core descriptor list, 实现 N 核协同跑整网

支持 4 种核协同模式:
  A. 单核单层      — 一个核独占跑完一层
  B. 单核多层      — 一个核连续跑多层 (核内串行)
  C.1 多核 Cout 切片 — 多核分担一层, 沿 cout 维度切, input 广播, output 拼接
  C.2 多核 W 切片  — 多核分担一层, 沿 W 维度切, weight 广播 + halo, output 拼接

调度算法 (5 阶段):
  Phase 1: 计算量分析 (per-layer MACs + 资源 footprint)
  Phase 2: stage 划分 (网络切多个 segment, segment 间走 DDR)
  Phase 3: per-layer 模式选择 (A/B/C.1/C.2)
  Phase 4: stage 内核 mapping (具体核编号分配, 优化跨核 push)
  Phase 5: per-core descriptor 生成

详见 docs/multicore_scheduling.md.
"""

from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional
import os
import sys

# 引入 params.py 作为参数 single source of truth
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_THIS_DIR))
from params import (
    NUM_PE, NUM_COL, DATA_WIDTH, PSUM_WIDTH,
    IFB_DEPTH as IFB_SRAM_WORDS,
    OFB_DEPTH as OFB_SRAM_WORDS,
    WB_DEPTH as WB_SRAM_WORDS,
)


# ---------------------------------------------------------------------------
# Layer 抽象 (从 Chain DSL / PyTorch model 转来)
# ---------------------------------------------------------------------------
@dataclass
class Layer:
    name: str
    k: int             # kernel size (KxK 方形核)
    c_in: int
    c_out: int
    h_in: int
    w_in: int
    stride: int = 1
    pad: int = 0
    has_residual: bool = False
    sdp_shift: int = 0
    # 链 / residual 数据流: ResNet 不是线性 chain. 每 layer 独立指明输入跟 (可选) shortcut
    # 来源 (按 layer name). input_src=='' 表示整网入口 (gen 用 random ifm).
    input_src: str = ''
    shortcut_src: str = ''
    # SDP per-layer 量化参数 (默认 multicore_chain 风格 clip[0,255] no round, 跟 RTL 对齐)
    sdp_mult: int = 1
    sdp_zp_out: int = 0
    sdp_clip_min: int = 0
    sdp_clip_max: int = 255
    sdp_round_en: int = 0
    sdp_relu_en: int = 1
    # Residual 缩放
    shortcut_mult: int = 0
    shortcut_shift: int = 0

    @property
    def force_s2d(self) -> bool:
        """stride>=3 + K>=stride: line_buffer 不原生支持, 强制 S2D 转 stride=1 conv.
        Patch (K=4 s=4) 这种触发, 跟 run_regression force_s2d 同条件."""
        return self.stride >= 3 and self.k >= self.stride

    def s2d_eff(self) -> tuple:
        """S2D 后的等效维度 (h_in, w_in, k, c_in, stride). 没 s2d 就返回原值."""
        if not self.force_s2d:
            return (self.h_in, self.w_in, self.k, self.c_in, self.stride)
        # K_new = ceil(K/stride), c_in_new = stride² × c_in, stride_new = 1
        # H_in_eff = H_in (s2d 把空间折到 cin, 但实际 RTL 用的还是 pre-s2d 的 H_in
        # 因为 hw_files.s2d_input 重排时 pad 后再展开). 看 hw_files.compute_s2d_params 实现.
        # 实际 gen_isa_test 内部把 ifm 重排成新维度: H_IN, W_IN 都被 hw_files.s2d_input
        # 改成 s2d 后的 H/W. 这里返回 s2d 后的等效值.
        k_new = (self.k + self.stride - 1) // self.stride
        cin_new = self.stride * self.stride * self.c_in
        # h/w_in 经 s2d_input 重排后的实际尺寸: ceil((h+2*pad)/stride) (后续 pad=0)
        h_eff = (self.h_in + 2 * self.pad + self.stride - 1) // self.stride
        w_eff = (self.w_in + 2 * self.pad + self.stride - 1) // self.stride
        return (h_eff, w_eff, k_new, cin_new, 1)

    @property
    def h_out(self) -> int:
        return (self.h_in + 2 * self.pad - self.k) // self.stride + 1

    @property
    def w_out(self) -> int:
        return (self.w_in + 2 * self.pad - self.k) // self.stride + 1

    @property
    def cin_slices(self) -> int:
        return (self.c_in + NUM_PE - 1) // NUM_PE

    @property
    def cout_slices(self) -> int:
        return (self.c_out + NUM_COL - 1) // NUM_COL

    @property
    def macs(self) -> int:
        """理论 MAC 数 (假设所有 PE 都干活, 即 cin 跟 cout 都是 16 倍数)."""
        return self.h_out * self.w_out * self.c_out * self.k * self.k * self.c_in

    @property
    def cycles_estimate(self) -> int:
        """单核运行 cycle 估计 (满载理想). 实际有反压损失."""
        # 每 cycle 16×16 = 256 MAC. cycles ≈ MAC / 256
        # 加上 idle/stall 损失 ~10-20% (depending on mac_util)
        # 这里用 MAC / (NUM_COL × NUM_PE) 作为下界
        return self.macs // (NUM_COL * NUM_PE)

    @property
    def ifb_words(self) -> int:
        """单核装载整个 IFM 需要的 IFB SRAM word 数."""
        return self.h_in * self.w_in * self.cin_slices

    @property
    def wb_words(self) -> int:
        """单核装载整个 weight 需要的 WB SRAM word 数."""
        return self.k * self.k * self.cout_slices * self.cin_slices

    @property
    def ofb_words(self) -> int:
        """单核装载整个 OFM 需要的 OFB SRAM word 数."""
        return self.h_out * self.w_out * self.cout_slices

    def fits_in_one_core_full(self) -> bool:
        """整图能不能装单核 SRAM (batch 模式)? streaming 模式不需要这个."""
        return (self.ifb_words <= IFB_SRAM_WORDS
                and self.wb_words <= WB_SRAM_WORDS
                and self.ofb_words <= OFB_SRAM_WORDS - 1)

    def fits_in_one_core_streaming(self) -> bool:
        """streaming 模式: 只要 IFB strip + WB + OFB strip 装得下就行 (ring 模式).
        IFB strip 至少 K 行, OFB strip 至少 1 行. 大网络都是 streaming 模式."""
        ifb_strip_min = (self.k + 1) * self.w_in * self.cin_slices
        ofb_strip_min = 1 * self.w_out * self.cout_slices
        return (ifb_strip_min <= IFB_SRAM_WORDS
                and self.wb_words <= WB_SRAM_WORDS
                and ofb_strip_min <= OFB_SRAM_WORDS - 1)


# ---------------------------------------------------------------------------
# 切片模式 + 切片后 sublayer
# ---------------------------------------------------------------------------
class Mode:
    A_SINGLE      = 'A_single'      # 一核独占
    B_PACK        = 'B_pack'        # 多层打包给一核 (上下文层是同核)
    C_COUT_SLICE  = 'C_cout_slice'  # 多核 cout 切片
    C_W_SLICE     = 'C_w_slice'     # 多核 W 切片


@dataclass
class LayerAssignment:
    layer: Layer
    mode: str
    cores: List[int]                  # 参与核 id 列表
    cout_segments: Optional[List[Tuple[int, int]]] = None  # mode C.1: per-core cout 区间
    w_segments: Optional[List[Tuple[int, int]]] = None     # mode C.2: per-core w 区间 (含 halo)

    def per_core_cycles(self) -> int:
        """切片后单核需跑的 cycle (理想), N 核分担时 ≈ 单核 cycle / N."""
        n = len(self.cores)
        return self.layer.cycles_estimate // max(n, 1)


# ---------------------------------------------------------------------------
# Phase 1: 计算量分析
# ---------------------------------------------------------------------------
def analyze(layers: List[Layer]) -> Dict:
    """返回整网 + per-layer 的 MAC 跟 footprint 报告."""
    total_macs = sum(l.macs for l in layers)
    total_cycles = sum(l.cycles_estimate for l in layers)

    report = {
        'total_macs'   : total_macs,
        'total_cycles' : total_cycles,
        'n_layers'     : len(layers),
        'layers'       : [],
    }
    for l in layers:
        report['layers'].append({
            'name'     : l.name,
            'k'        : l.k,
            'h_in'     : l.h_in, 'w_in'  : l.w_in,
            'h_out'    : l.h_out, 'w_out' : l.w_out,
            'c_in'     : l.c_in, 'c_out' : l.c_out,
            'macs'     : l.macs,
            'cycles'   : l.cycles_estimate,
            'pct_macs' : 100.0 * l.macs / total_macs if total_macs else 0,
            'ifb_words': l.ifb_words,
            'wb_words' : l.wb_words,
            'ofb_words': l.ofb_words,
            'fits_full': l.fits_in_one_core_full(),
            'fits_strm': l.fits_in_one_core_streaming(),
        })
    return report


def print_analysis(report: Dict) -> None:
    print(f"=== Network analysis ===")
    print(f"  Total MACs:    {report['total_macs']:>15,}")
    print(f"  Total cycles:  {report['total_cycles']:>15,} (理想满载下界, 单核)")
    print(f"  Layer count:   {report['n_layers']}")
    print()
    print(f"{'#':>3} {'name':<14} {'dim':<14} {'cin→cout':<10} "
          f"{'MACs':>12} {'cycles':>10} {'%':>5} "
          f"{'ifb':>6} {'wb':>5} {'ofb':>5} {'strm?':>6}")
    print('-' * 110)
    for i, r in enumerate(report['layers']):
        dim = f"{r['h_out']}x{r['w_out']}"
        cic = f"{r['c_in']}->{r['c_out']}"
        strm = 'Y' if r['fits_strm'] else 'N'
        print(f"{i:>3} {r['name']:<14} {dim:<14} {cic:<10} "
              f"{r['macs']:>12,} {r['cycles']:>10,} {r['pct_macs']:>5.1f} "
              f"{r['ifb_words']:>6} {r['wb_words']:>5} {r['ofb_words']:>5} {strm:>6}")


# ---------------------------------------------------------------------------
# Phase 2: Stage 划分
#   一个 stage = 一组连续 layer, stage 内多核流水线 (跨核 push), stage 间 DDR 同步.
#   贪心策略: 从前往后累加 layer.cycles, 累计到 target_stage_cycles 就切.
# ---------------------------------------------------------------------------
@dataclass
class Stage:
    layers: List[Layer]
    layer_idx: List[int]              # layers 在原网络中的 idx
    assignments: List['LayerAssignment'] = field(default_factory=list)

    @property
    def total_cycles(self) -> int:
        return sum(l.cycles_estimate for l in self.layers)


def split_into_stages(layers: List[Layer], n_cores: int,
                      target_per_core_cycles: Optional[int] = None) -> List[Stage]:
    """
    把网络按 cycles 平均切多个 stage, 每 stage 内多核流水线协同跑.

    规则:
      - 每 stage 总 cycles ≈ N × per-core 期望 cycles
      - 如果 target_per_core_cycles=None, 默认每 stage 至少 N 层 (流水线满管)
      - stage 内 layer 数 ≥ N (让每个核都能拿到一个 layer 干, 形成 ping-pong 流水)
    """
    total_cycles = sum(l.cycles_estimate for l in layers)
    if target_per_core_cycles is None:
        # 默认: 整网均摊到 N 核理想 cycle
        target_per_core_cycles = total_cycles // n_cores

    target_stage_cycles = n_cores * target_per_core_cycles  # 每 stage 总 cycles

    stages = []
    cur_layers = []
    cur_idx = []
    cur_cycles = 0

    for i, layer in enumerate(layers):
        cur_layers.append(layer)
        cur_idx.append(i)
        cur_cycles += layer.cycles_estimate

        # 到达 target 且 stage 内 layer 数 ≥ n_cores 时切
        if cur_cycles >= target_stage_cycles and len(cur_layers) >= n_cores:
            stages.append(Stage(cur_layers, cur_idx))
            cur_layers, cur_idx, cur_cycles = [], [], 0

    if cur_layers:
        stages.append(Stage(cur_layers, cur_idx))

    return stages


# ---------------------------------------------------------------------------
# Phase 3: per-layer mode 选择 + Phase 4: 核 mapping
#   stage 内 N 个核 ping-pong 跑流水线: 给每层选 mode + 分核.
#   简化版策略 (M2.5 第一步): 优先 mode A (单核独占), 大层考虑 mode C.
# ---------------------------------------------------------------------------
def can_w_slice(layer: Layer, n_split: int) -> bool:
    """W 切片可行性: w_in >= n_split * (K+1) 留 halo."""
    return layer.w_in >= n_split * (layer.k + 1)


def can_cout_slice(layer: Layer, n_split: int) -> bool:
    """Cout 切片可行性: c_out >= n_split * NUM_COL."""
    return layer.c_out >= n_split * NUM_COL


def choose_mode(layer: Layer, n_cores: int, target_per_core_cycles: int,
                prefer_w_slice: bool = True,
                force_multicore: bool = False) -> Tuple[str, int]:
    """
    判定 layer 用哪种模式 + 切多少核. 返回 (mode, n_split).

    prefer_w_slice=True 时优先 W 切片 (跨 stage 数据流局部, 可片上 push 跟下层兼容).
    Cout 切片只在 W 切不下时退化.

    全 N 核切片永远是 default 选择 (mode C, n_split=N), 无论 layer 大小.
    这样保证每 stage 内所有核都干活, 且每 stage = 一个 layer (简单)..

    例外: 极小 layer (cycle < target/4) 退化为 mode A 单核 (避免切片 overhead 占比高).

    force_multicore=True: BW-PoC 实验用, 不让任何 layer 走单核 (即使 cycle 很小).
        让所有 layer 占满 4 个 DDR slot, 量化"BW × 4 + 无空转" 上限.
    """
    cyc = layer.cycles_estimate

    # 极小 layer: mode A 单核 (force_multicore=True 时跳过此分支)
    if not force_multicore and cyc < target_per_core_cycles // 4:
        return (Mode.A_SINGLE, 1)

    # force_multicore 模式下: W slice 不可行的 layer (W=1 FC) 保持单核
    # (cout slice driver 路径未实现, 暂避开)
    if force_multicore and not can_w_slice(layer, n_cores):
        return (Mode.A_SINGLE, 1)

    # 默认全 N 核切片. 优先 W 切片, 兜底 cout.
    n_split = n_cores

    if prefer_w_slice:
        if can_w_slice(layer, n_split):
            return (Mode.C_W_SLICE, n_split)
        if can_cout_slice(layer, n_split):
            return (Mode.C_COUT_SLICE, n_split)
    else:
        if can_cout_slice(layer, n_split):
            return (Mode.C_COUT_SLICE, n_split)
        if can_w_slice(layer, n_split):
            return (Mode.C_W_SLICE, n_split)

    # 都不行: 减少 n_split
    for ns in range(n_cores - 1, 1, -1):
        if can_w_slice(layer, ns):
            return (Mode.C_W_SLICE, ns)
        if can_cout_slice(layer, ns):
            return (Mode.C_COUT_SLICE, ns)

    # 实在不能切, 单核硬扛
    return (Mode.A_SINGLE, 1)


def slicing_compatible(prev_mode: str, prev_n: int, curr_mode: str, curr_n: int,
                        prev_layer: Layer, curr_layer: Layer) -> bool:
    """
    切片维度兼容性:
      - W slice → W slice (相同 n_split): ✅ 可直接 push (含 halo)
      - 其他: 必须 DDR 中转 (broadcast 通过 DDR 等效)
    """
    if prev_mode == Mode.A_SINGLE or curr_mode == Mode.A_SINGLE:
        return False  # 单核 layer 边界一定走 DDR
    if prev_mode == Mode.C_W_SLICE and curr_mode == Mode.C_W_SLICE and prev_n == curr_n:
        return True
    return False


def make_cout_segments(c_out: int, n: int) -> List[Tuple[int, int]]:
    """把 c_out 沿 NUM_COL=16 倍数切 n 段 (尽量均匀)."""
    base = (c_out // NUM_COL) // n        # 每核多少 col
    extra = (c_out // NUM_COL) % n         # 余 col 给前几个核
    segs = []
    cursor = 0
    for i in range(n):
        cols = base + (1 if i < extra else 0)
        end = cursor + cols * NUM_COL
        if i == n - 1:
            end = c_out                  # 最后一段含尾巴 (非 16 倍数)
        segs.append((cursor, end))
        cursor = end
    return segs


def make_w_segments(w_in: int, n: int, k: int) -> List[Tuple[int, int]]:
    """把 w_in 切 n 段 (含 halo)."""
    base = w_in // n
    halo = (k - 1) // 2
    segs = []
    for i in range(n):
        w_core_lo = i * base
        w_core_hi = (i + 1) * base if i < n - 1 else w_in
        w_lo = max(0, w_core_lo - halo)
        w_hi = min(w_in, w_core_hi + halo)
        segs.append((w_lo, w_hi))
    return segs


def analyze_slicing(layers: List[Layer], n_cores: int,
                    force_multicore: bool = False) -> List[Dict]:
    """
    给出每 layer 的最佳切片决策, 并标记 layer 间是否兼容片上 push 链.
    force_multicore=True: BW-PoC 用, 强制所有 layer 多核切片 (即使 cycle 小).
    返回 list of dict.
    """
    target = sum(l.cycles_estimate for l in layers) // n_cores
    decisions = []
    prev_mode = None
    prev_n = 0
    for i, layer in enumerate(layers):
        mode, n_split = choose_mode(layer, n_cores, target, prefer_w_slice=True,
                                     force_multicore=force_multicore)
        compat = False
        if prev_mode is not None:
            compat = slicing_compatible(prev_mode, prev_n, mode, n_split,
                                        layers[i-1] if i > 0 else None, layer)
        decisions.append({
            'idx'      : i,
            'name'     : layer.name,
            'cycles'   : layer.cycles_estimate,
            'cycles_per_core': layer.cycles_estimate // max(n_split, 1),
            'mode'     : mode,
            'n_split'  : n_split,
            'h_out'    : layer.h_out,
            'w_out'    : layer.w_out,
            'c_out'    : layer.c_out,
            'compat_with_prev': compat,
        })
        prev_mode = mode
        prev_n = n_split
    return decisions


def print_slicing(decisions: List[Dict], n_cores: int) -> None:
    print(f"\n=== Slicing decisions (N={n_cores}, prefer W slice) ===")
    print(f"{'#':>3} {'name':<14} {'cyc':>8} {'cyc/N':>8} {'mode':<14} {'split':>5} "
          f"{'h_out':>5} {'w_out':>5} {'c_out':>5} {'compat?':>8}")
    print('-' * 90)
    sum_per_core = 0
    n_ddr_boundaries = 0
    for d in decisions:
        compat = '[Y] push' if d['compat_with_prev'] else '[N] DDR'
        if not d['compat_with_prev'] and d['idx'] > 0:
            n_ddr_boundaries += 1
        print(f"{d['idx']:>3} {d['name']:<14} {d['cycles']:>8,} {d['cycles_per_core']:>8,} "
              f"{d['mode']:<14} {d['n_split']:>5} {d['h_out']:>5} {d['w_out']:>5} {d['c_out']:>5} "
              f"{compat:>8}")
        sum_per_core += d['cycles_per_core']
    total_cycles = sum(d['cycles'] for d in decisions)
    print(f"\n  Sum cycles per core: {sum_per_core:,}")
    print(f"  Single-core baseline: {total_cycles:,}")
    print(f"  Speedup: {total_cycles/max(sum_per_core,1):.2f}× (ideal {n_cores}×)")
    print(f"  DDR boundaries: {n_ddr_boundaries} (片上 push: {len(decisions)-1-n_ddr_boundaries})")
    print(f"  Wall ≈ {sum_per_core:,} cycles → {1e9/(sum_per_core*10):.0f} fps @ 100MHz "
          f"(忽略 stage overhead)")


def assign_cores_in_stage(stage: Stage, n_cores: int,
                          target_per_core_cycles: int,
                          force_multicore: bool = False) -> None:
    """
    LPT 算法 (Longest Processing Time first):
    1. 先决定每层 mode + n_split, 算出该层"单核负担" (cycle/n_split if 切片, 否则全 cycle)
    2. 按"单核负担"降序排序 layer
    3. 依次给每层选最闲核(mode A) 或全核(mode C)

    layer 顺序在流水线 steady state 下不影响 wallclock = max(per-core load),
    所以可以自由 reorder 给最闲核.
    force_multicore=True: BW-PoC 用, 强制 ds 等小 layer 也走多核 (W slice 4 核陪算).
    """
    # 第一步: 决定每层 mode + n_split + 单核负担
    layer_meta = []  # list of (layer, mode, n_split, per_core_cycle)
    for layer in stage.layers:
        mode, n_split = choose_mode(layer, n_cores, target_per_core_cycles,
                                     force_multicore=force_multicore)
        if mode == Mode.A_SINGLE:
            per_core = layer.cycles_estimate
        else:
            per_core = layer.cycles_estimate // n_split
        layer_meta.append((layer, mode, n_split, per_core))

    # 第二步: LPT — 按 per_core 负担降序排序, 大的先分 (经典 4/3 近似比)
    layer_meta_sorted = sorted(layer_meta, key=lambda x: -x[3])

    # 第三步: 按 sorted 顺序分配核
    core_load = [0] * n_cores
    layer_assignments = {}    # layer.name → LayerAssignment

    for layer, mode, n_split, per_core in layer_meta_sorted:
        if mode == Mode.C_COUT_SLICE:
            cores = sorted(range(n_cores), key=lambda c: core_load[c])[:n_split]
            segments = make_cout_segments(layer.c_out, n_split)
            for c in cores:
                core_load[c] += per_core
            layer_assignments[layer.name] = LayerAssignment(
                layer=layer, mode=mode, cores=sorted(cores),
                cout_segments=segments)

        elif mode == Mode.C_W_SLICE:
            cores = sorted(range(n_cores), key=lambda c: core_load[c])[:n_split]
            segments = make_w_segments(layer.w_in, n_split, layer.k)
            for c in cores:
                core_load[c] += per_core
            layer_assignments[layer.name] = LayerAssignment(
                layer=layer, mode=mode, cores=sorted(cores),
                w_segments=segments)

        else:  # mode A
            target_core = min(range(n_cores), key=lambda c: core_load[c])
            core_load[target_core] += per_core
            layer_assignments[layer.name] = LayerAssignment(
                layer=layer, mode=mode, cores=[target_core])

    # 第四步: 按原 layer 顺序写回 stage.assignments (保留输出顺序)
    for layer in stage.layers:
        stage.assignments.append(layer_assignments[layer.name])


# ---------------------------------------------------------------------------
# Phase 5: per-core layer plan 生成
#   把 stage assignments 转成"每个核要跑什么", 同时确定跨层 push 链上下游关系.
#
#   返回 dict: core_id → list of LayerStep, 每 step 描述:
#     - layer: 原 Layer
#     - mode: A_SINGLE / C_COUT_SLICE / C_W_SLICE
#     - cores: 该 layer 全部参与核 (用于 producer 决定 ODMA dst)
#     - role: 'standalone' / 'producer' / 'consumer'
#     - producer_layer / consumer_layer: 跨核 push 链中前后层引用
#     - cout_segment / w_segment: mode C 切片下本核负责的段
# ---------------------------------------------------------------------------
@dataclass
class LayerStep:
    """单核单层的执行计划. 从核视角看 input/output 数据流向."""
    layer: Layer
    mode: str
    layer_idx: int                                # 在原网络中的层 idx (0..N-1)
    cores_all: List[int]                          # 这层所有参与核
    my_core: int                                  # 当前核 id
    cout_segment: Optional[Tuple[int, int]] = None
    w_segment:    Optional[Tuple[int, int]] = None
    # 数据流向 (编译器 cfg 生成依据)
    input_from:  str = 'ddr'                      # 'ddr' / 'push'
    output_to:   str = 'ddr'                      # 'ddr' / 'push'
    push_from_core: Optional[int] = None          # input_from='push' 时上层 producer 核
    push_to_core:   Optional[int] = None          # output_to='push' 时下层 consumer 核
    skip_idma:   bool = False                     # = (input_from == 'push')


def gen_per_core_plan(stages: List[Stage], n_cores: int,
                       enable_push: bool = False) -> Dict[int, List[LayerStep]]:
    """
    生成每核 layer 执行计划 (Phase 5).

    enable_push=True 时启用跨核 push 优化 (要求严格 ABAB ping-pong 模式, 否则死锁).
    enable_push=False 时全部走 DDR 中转 (功能稳妥, fps 略降).

    数据流决策:
      - 一层 mode A (单核独占) → 下层 mode A 不同核 + enable_push: 跨核 push
      - 一层 mode A → 下层 mode A 同核: DDR 中转
      - mode C 切片层: 都走 DDR (跨核 push 切片需要 broadcast, P3 再做)
      - enable_push=True 时遇到 ABA pattern 自动降级 DDR (双边检测保证一致性)
    """
    per_core: Dict[int, List[LayerStep]] = {c: [] for c in range(n_cores)}

    # 摊平所有层
    flat_assignments = []
    for stage in stages:
        for a in stage.assignments:
            flat_assignments.append(a)

    for i, a in enumerate(flat_assignments):
        next_a = flat_assignments[i + 1] if i + 1 < len(flat_assignments) else None
        prev_a = flat_assignments[i - 1] if i > 0 else None

        # 判断这层的 cores (mode A: 1 核, mode C: N 核)
        layer_cores = a.cores

        # 判断本层每核的 input_from / output_to
        # 当前简化: 只有 mode A → mode A 不同核时跨核 push
        input_is_push = False
        output_is_push = False
        push_from = None
        push_to = None

        if a.mode == Mode.A_SINGLE:
            my_core = a.cores[0]
            if enable_push:
                # 上一层 mode A 单核且不同核 → 我是 consumer
                if (prev_a is not None and prev_a.mode == Mode.A_SINGLE
                        and prev_a.cores[0] != my_core):
                    input_is_push = True
                    push_from = prev_a.cores[0]
                # 下一层 mode A 单核且不同核 → 我是 producer
                if (next_a is not None and next_a.mode == Mode.A_SINGLE
                        and next_a.cores[0] != my_core):
                    output_is_push = True
                    push_to = next_a.cores[0]
                # ABA/BAB 死锁防护: 三连层中, 如果 i-1 和 i+1 都不是我, 但其中一边
                # 是同核, 当前层的 push 链会跟核内 sequencer 顺序冲突死锁.
                # 简化: 只在 N=2 严格 ABAB 模式启用 push, 否则降级 DDR.
                # 检查规则: 如果 prev 和 next 都是同一核 (都不是我), 那就是 ABA. 当前
                # 层的 push 关系会让对方核出现 sequencer 卡死.
                if (prev_a is not None and next_a is not None
                        and prev_a.mode == Mode.A_SINGLE and next_a.mode == Mode.A_SINGLE
                        and prev_a.cores[0] == next_a.cores[0]
                        and prev_a.cores[0] != my_core):
                    # ABA detected, 双边降级 DDR
                    input_is_push = False
                    output_is_push = False
                    push_from = None
                    push_to = None

            step = LayerStep(
                layer=a.layer, mode=a.mode, layer_idx=i,
                cores_all=layer_cores, my_core=my_core,
                input_from='push' if input_is_push else 'ddr',
                output_to='push' if output_is_push else 'ddr',
                push_from_core=push_from,
                push_to_core=push_to,
                skip_idma=input_is_push,
            )
            per_core[my_core].append(step)

        else:  # mode C 切片: 各核独立从 DDR 读, 写 DDR (cout/w 段拼接)
            for idx, c in enumerate(layer_cores):
                step = LayerStep(
                    layer=a.layer, mode=a.mode, layer_idx=i,
                    cores_all=layer_cores, my_core=c,
                    cout_segment=a.cout_segments[idx] if a.cout_segments else None,
                    w_segment=a.w_segments[idx] if a.w_segments else None,
                    input_from='ddr', output_to='ddr', skip_idma=False,
                )
                per_core[c].append(step)

    return per_core


def print_per_core_plan(per_core_plan: Dict[int, List[LayerStep]]) -> None:
    print("\n=== Per-core layer plan (Phase 5) ===")
    for core_id in sorted(per_core_plan.keys()):
        steps = per_core_plan[core_id]
        print(f"  Core {core_id}: {len(steps)} layers")
        for s in steps:
            in_str  = f"in={s.input_from}"
            if s.input_from == 'push':
                in_str += f"(c{s.push_from_core})"
            out_str = f"out={s.output_to}"
            if s.output_to == 'push':
                out_str += f"(c{s.push_to_core})"
            extras = [in_str, out_str]
            if s.skip_idma:
                extras.append('SKIP_IDMA')
            if s.cout_segment:
                extras.append(f"cout=[{s.cout_segment[0]}:{s.cout_segment[1]}]")
            if s.w_segment:
                extras.append(f"w=[{s.w_segment[0]}:{s.w_segment[1]}]")
            extra_s = ' ' + ' '.join(extras)
            print(f"    [{s.layer_idx:>2}] {s.layer.name:<14} {s.mode:<12} "
                  f"cyc={s.layer.cycles_estimate:>8,}{extra_s}")


# ---------------------------------------------------------------------------
# 主入口: schedule 整网 → stages
# ---------------------------------------------------------------------------
def schedule(layers: List[Layer], n_cores: int,
             force_multicore: bool = False) -> List[Stage]:
    """force_multicore=True: BW-PoC 用, 让所有 layer 走多核 (无单核空转)."""
    total_cycles = sum(l.cycles_estimate for l in layers)
    target_per_core_cycles = total_cycles // n_cores

    stages = split_into_stages(layers, n_cores, target_per_core_cycles)
    for stage in stages:
        assign_cores_in_stage(stage, n_cores, target_per_core_cycles,
                               force_multicore=force_multicore)
    return stages


def print_schedule(stages: List[Stage], n_cores: int) -> None:
    print(f"\n=== Schedule (N={n_cores}) ===")
    total_cycles = sum(s.total_cycles for s in stages)
    print(f"  Stages: {len(stages)}, total cycles (单核基线): {total_cycles:,}")
    ideal_per_core = total_cycles // n_cores
    print(f"  Ideal per-core cycles: {ideal_per_core:,}")
    print()

    # 每核累计 cycles (估算多核流水后的 wallclock)
    per_core_load = [0] * n_cores

    for s_idx, stage in enumerate(stages):
        print(f"  --- Stage {s_idx} ({len(stage.layers)} layers, {stage.total_cycles:,} cycles)")
        stage_per_core = [0] * n_cores
        for a in stage.assignments:
            mode_str = a.mode
            cores_str = ','.join(str(c) for c in a.cores)
            cyc = a.per_core_cycles()
            for c in a.cores:
                stage_per_core[c] += cyc
            extra = ''
            if a.mode == Mode.C_COUT_SLICE:
                extra = f' cout_seg={a.cout_segments}'
            elif a.mode == Mode.C_W_SLICE:
                extra = f' w_seg={a.w_segments}'
            print(f"    {a.layer.name:<14} {mode_str:<14} core={cores_str:<6} "
                  f"cyc/core={cyc:>8,}{extra}")
        # stage 等价 wall = max(per-core load this stage)
        stage_wall = max(stage_per_core)
        print(f"    stage wall ≈ {stage_wall:,} (max per-core)")
        for c in range(n_cores):
            per_core_load[c] += stage_per_core[c]

    print(f"\n  Per-core total load: {per_core_load}")
    wall = max(per_core_load)
    print(f"  Wallclock estimate: {wall:,} cycles ({wall*10/1000:.2f} us @ 100 MHz, "
          f"{1e9/(wall*10):.1f} fps)")
    print(f"  Speedup vs 1 core: {total_cycles/wall:.2f}x (ideal {n_cores}x)")
def chain_to_layers(chain_cases: list) -> List[Layer]:
    """把 run_regression.py 的 Chain.cases (list of dict) 转成 Layer 序列.
    保留 input_src / shortcut_src / 全部 SDP 参数 — ResNet residual + 维度变化都靠它链起来.
    """
    layers = []
    for c in chain_cases:
        has_res = bool(c.get('shortcut_src', ''))
        layers.append(Layer(
            name           = c['name'],
            k              = c['k'],
            c_in           = c['c_in'],
            c_out          = c['c_out'],
            h_in           = c['h_in'],
            w_in           = c['w_in'],
            stride         = c['stride'],
            pad            = c['pad'],
            has_residual   = has_res,
            sdp_shift      = c['sdp_shift'],
            input_src      = c.get('input_src', ''),
            shortcut_src   = c.get('shortcut_src', ''),
            sdp_mult       = c.get('sdp_mult', 1),
            sdp_zp_out     = c.get('sdp_zp_out', 0),
            sdp_clip_min   = c.get('sdp_clip_min', 0),
            sdp_clip_max   = c.get('sdp_clip_max', 255),
            sdp_round_en   = c.get('sdp_round_en', 0),
            sdp_relu_en    = c.get('sdp_relu_en', 1),
            # 用户可选; ResNet 默认 mult=1 shift=0
            shortcut_mult  = 1 if has_res else 0,
            shortcut_shift = 0,
        ))
    return layers


# ---------------------------------------------------------------------------
# 网络定义: YOLOv3-tiny (典型大网络示例)
#
# 13 个 conv layer + maxpool/upsample/concat (后者当前 RTL 不支持, 这里仅 conv 分析).
# 输入 416×416×3, 输出双 head (13×13 + 26×26) for COCO 80 class.
# ---------------------------------------------------------------------------
def yolov3_tiny_layers() -> List[Layer]:
    """YOLOv3-tiny 13 conv layers (忽略 maxpool/upsample/concat 后处理)."""
    return [
        # backbone
        Layer('conv1',  k=3, c_in=3,    c_out=16,   h_in=416, w_in=416, stride=1, pad=1),
        # maxpool 416→208
        Layer('conv2',  k=3, c_in=16,   c_out=32,   h_in=208, w_in=208, stride=1, pad=1),
        # maxpool 208→104
        Layer('conv3',  k=3, c_in=32,   c_out=64,   h_in=104, w_in=104, stride=1, pad=1),
        # maxpool 104→52
        Layer('conv4',  k=3, c_in=64,   c_out=128,  h_in=52,  w_in=52,  stride=1, pad=1),
        # maxpool 52→26
        Layer('conv5',  k=3, c_in=128,  c_out=256,  h_in=26,  w_in=26,  stride=1, pad=1),
        # maxpool 26→13
        Layer('conv6',  k=3, c_in=256,  c_out=512,  h_in=13,  w_in=13,  stride=1, pad=1),
        # maxpool 2x2 s=1 (no dim change)
        Layer('conv7',  k=3, c_in=512,  c_out=1024, h_in=13,  w_in=13,  stride=1, pad=1),
        # head 1
        Layer('conv8',  k=1, c_in=1024, c_out=256,  h_in=13,  w_in=13,  stride=1, pad=0),
        Layer('conv9',  k=3, c_in=256,  c_out=512,  h_in=13,  w_in=13,  stride=1, pad=1),
        Layer('conv10', k=1, c_in=512,  c_out=255,  h_in=13,  w_in=13,  stride=1, pad=0),
        # route from conv8 → upsample → concat
        Layer('conv11', k=1, c_in=256,  c_out=128,  h_in=13,  w_in=13,  stride=1, pad=0),
        # upsample 13→26, concat with conv5 → 384 channel
        Layer('conv12', k=3, c_in=384,  c_out=256,  h_in=26,  w_in=26,  stride=1, pad=1),
        Layer('conv13', k=1, c_in=256,  c_out=255,  h_in=26,  w_in=26,  stride=1, pad=0),
    ]


# ---------------------------------------------------------------------------
# CLI: 跑 11-case ResNet + YOLOv3-tiny 分析
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from run_regression import CASES

    print("\n" + "=" * 60)
    print("Network 1: ResNet-18-like 11-case (240×135×4 → 522)")
    print("=" * 60)
    layers = chain_to_layers(CASES[:11])
    report = analyze(layers)
    print_analysis(report)
    for n in [2, 4]:
        decisions = analyze_slicing(layers, n_cores=n)
        print_slicing(decisions, n_cores=n)

    print("\n" + "=" * 60)
    print("Network 2: YOLOv3-tiny (416×416×3, 13 conv layers)")
    print("=" * 60)
    yolo_layers = yolov3_tiny_layers()
    yolo_report = analyze(yolo_layers)
    print_analysis(yolo_report)
    for n in [2, 4, 8]:
        stages = schedule(yolo_layers, n_cores=n)
        print_schedule(stages, n_cores=n)
