"""
compile_model.py — PyTorch nn.Module → 多层硬件编译流水 (Phase G)

扫描 Conv2d (+ ReLU) 链 → calibration 量化 → 分配 DDR layout → 逐层派发到
compile_layer。生成 model_plan.json 供 run_regression / 驱动脚本用。

当前范围：
  - 只支持 Conv2d + ReLU 串联链（Sequential / 嵌套 Module 均可）
  - Linear / Pooling / BN / Residual 未支持（roadmap Phase H+）
  - 每层都需满足现有硬件约束（streaming 下 Cin/Cout ≤ 16 * slices 有效）

DDR layout：
  - Feature-map 区（FM_0..FM_L）：layer k 的 OFB = layer k+1 的 IFB，共享 base
  - Weight 区：每层独立
  - Descriptor 区：每层独立
  - 所有 base 4KB 对齐；base 写入 config.txt 的 _META_DDR_*_BASE，TB 运行期读

用法：
    from compile_model import compile_and_emit_model
    compile_and_emit_model(model, x_calib, out_dir=..., streaming=True)
"""

import os
import json
import argparse

import hw_files
import compile_layer
from model_zoo import MODELS, build_model


_SCRIPT_DIR     = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT_DIR = os.path.normpath(
    os.path.join(_SCRIPT_DIR, "..", "sim", "tb_core_dma", "cases", "model"))

# DDR layout 起点（byte 地址，需落在 TB DDR_DEPTH=16MB 内）
# 三段：FM [0, 8MB) / WB [8MB, 15MB) / DESC [15MB, 16MB)
DDR_FM_REGION_BASE   = 0x0000_0000
DDR_FM_REGION_LIMIT  = 0x0080_0000
DDR_WB_REGION_BASE   = 0x0080_0000
DDR_WB_REGION_LIMIT  = 0x00F0_0000
DDR_DESC_REGION_BASE = 0x00F0_0000
DDR_DESC_REGION_LIMIT= 0x0100_0000
ALIGN_BYTES          = 0x1000   # 4KB
AXI_BYTES            = 16
DESC_BYTES_RESERVED  = 0x8000   # 每层 32KB descriptor 区够用（远超 2048 desc）


def _align(x, a=ALIGN_BYTES):
    return (x + a - 1) & ~(a - 1)


# ---------------------------------------------------------------------------
# Conv 链提取 + calibration
# ---------------------------------------------------------------------------
def _extract_conv_chain(model):
    """
    按 forward 顺序提取 Conv2d。返回 list of (conv, relu_en)。
    若某 Conv2d 后紧跟 ReLU（中间可有 Identity 类无参模块），relu_en=True。
    最后一个 Conv 之后无 ReLU → relu_en=False（分类 logits 层）。
    """
    torch = compile_layer._require_torch()
    # 先按顺序收集 (kind, module)
    flat = []
    def walk(m):
        for child in m.children():
            if isinstance(child, (torch.nn.Sequential, torch.nn.ModuleList)):
                walk(child)
            elif isinstance(child, torch.nn.Conv2d):
                flat.append(('conv', child))
            elif isinstance(child, torch.nn.ReLU):
                flat.append(('relu', child))
            elif len(list(child.children())) > 0:
                walk(child)
            else:
                raise NotImplementedError(
                    f"compile_model: 暂不支持 {type(child).__name__}; "
                    f"当前只处理 Conv2d + ReLU 链")
    walk(model)

    convs = []
    i = 0
    while i < len(flat):
        kind, mod = flat[i]
        if kind == 'conv':
            has_relu = (i + 1 < len(flat) and flat[i+1][0] == 'relu')
            convs.append((mod, has_relu))
            i += 2 if has_relu else 1
        elif kind == 'relu':
            raise ValueError("出现孤立 ReLU（未跟在 Conv2d 后）")
        else:
            i += 1
    if not convs:
        raise ValueError("未找到任何 Conv2d")
    return convs


def _calibrate_activations(model, x):
    """
    forward + hook 采样每 Conv2d 前输入 + 最终输出，计算 per-tensor scale。
    返回 (convs, scales[L+1], acts[L+1])。scales[k] = layer k 输入的对称量化 scale。
    """
    torch = compile_layer._require_torch()
    conv_chain = _extract_conv_chain(model)   # [(conv, has_relu), ...]
    acts = []
    hooks = []

    def pre_hook(m, inp):
        acts.append(inp[0].detach().clone())
    for conv, _ in conv_chain:
        hooks.append(conv.register_forward_pre_hook(pre_hook))

    def final_hook(m, inp, out):
        acts.append(out.detach().clone())
    hooks.append(model.register_forward_hook(final_hook))

    with torch.no_grad():
        _ = model(x)
    for h in hooks:
        h.remove()

    if len(acts) != len(conv_chain) + 1:
        raise RuntimeError(f"calibration: acts={len(acts)} vs convs={len(conv_chain)}+1")
    scales = [compile_layer.estimate_activation_scale(a) for a in acts]
    return conv_chain, scales, acts


# ---------------------------------------------------------------------------
# DDR layout
# ---------------------------------------------------------------------------
def _plan_ddr(layer_cfgs):
    """
    输入每层 derive_layer_cfg；输出 list[dict] per layer:
      {ifb_base, ofb_base, wb_base, desc_base} (byte address)。

    FM 共享链：layer k 的 OFB 写到 fm_bases[k+1]，作为 layer k+1 的 IFB 读。
    关键约束：fm_bases[k+1] - fm_bases[k] 必须 ≥ fm_size[k] (= layer k 的输入大小)，
              否则 layer k 写 OFB 会覆盖自己还没读完的 IFB（streaming 下尤其致命）。
              stride>1 的层 output size < input size，过去用 ofb_bytes 作步长会
              让下层 OFB 落在本层 IFB 区间内 → DDR 被踩。
    """
    L = len(layer_cfgs)
    fm_bases = [DDR_FM_REGION_BASE]
    # 第 0 个 FM (layer 0 输入) 大小 = L0 的 IFB bytes
    cur_fm_size = (layer_cfgs[0]['H_IN'] * layer_cfgs[0]['W_IN']
                   * layer_cfgs[0]['cin_slices'] * AXI_BYTES)
    for cfg in layer_cfgs:
        # layer k 的 IFB 占 fm_bases[k]..fm_bases[k]+cur_fm_size。下一层 OFB 必须跳过这整段。
        fm_bases.append(_align(fm_bases[-1] + cur_fm_size))
        # 下一个 FM (layer k 的 output = layer k+1 的 input)
        cur_fm_size = cfg['H_OUT'] * cfg['W_OUT'] * cfg['cout_slices'] * AXI_BYTES
    if fm_bases[-1] > DDR_FM_REGION_LIMIT:
        raise ValueError(
            f"FM chain 超过 FM region: {fm_bases[-1]:#x} > {DDR_FM_REGION_LIMIT:#x}")

    wb_bases = []
    cur = DDR_WB_REGION_BASE
    for cfg in layer_cfgs:
        # wb_words 每 word = 16*16*8 bit = 256 byte（一行 2048-bit WB line）
        wb_bytes = cfg['wb_words'] * 256
        wb_bases.append(cur)
        cur = _align(cur + wb_bytes)
    if cur > DDR_WB_REGION_LIMIT:
        raise ValueError(f"WB chain 超过 WB region: {cur:#x} > {DDR_WB_REGION_LIMIT:#x}")

    desc_bases = []
    cur = DDR_DESC_REGION_BASE
    for _ in layer_cfgs:
        desc_bases.append(cur)
        cur = _align(cur + DESC_BYTES_RESERVED)
    if cur > DDR_DESC_REGION_LIMIT:
        raise ValueError(f"DESC chain 超过 DESC region: {cur:#x} > {DDR_DESC_REGION_LIMIT:#x}")

    return [
        {
            'ifb_base' : fm_bases[k],
            'ofb_base' : fm_bases[k+1],
            'wb_base'  : wb_bases[k],
            'desc_base': desc_bases[k],
        }
        for k in range(L)
    ]


# ---------------------------------------------------------------------------
# 主 API
# ---------------------------------------------------------------------------
def compile_and_emit_model(
    model, x_calib,
    out_dir=DEFAULT_OUT_DIR, streaming=True,
    model_name="model",
    layer_dir_fn=None,
):
    """
    layer_dir_fn: callable(k) -> per-layer 输出目录。None 时 = out_dir/layer_KK。
                  run_regression 用它把每层直接落到 cases/caseNN/。
    """
    """
    """
    torch = compile_layer._require_torch()
    conv_chain, scales, acts = _calibrate_activations(model, x_calib)
    L = len(conv_chain)

    # ---- Pass 1: derive 每层 cfg 拿尺寸 ----
    layer_cfgs = []
    h, w = x_calib.shape[2], x_calib.shape[3]
    for k, (conv, _relu) in enumerate(conv_chain):
        K      = conv.kernel_size[0]
        stride = conv.stride[0]
        pad    = conv.padding[0]
        cfg = hw_files.derive_layer_cfg(
            H_IN=h, W_IN=w, K=K,
            NUM_CIN=conv.in_channels, NUM_COUT=conv.out_channels,
            stride=stride, pad_top=pad, pad_left=pad, streaming=streaming)
        layer_cfgs.append(cfg)
        h, w = cfg['H_OUT'], cfg['W_OUT']

    # ---- Pass 2: 分配 DDR ----
    bases = _plan_ddr(layer_cfgs)

    # ---- Pass 3: 逐层编译 ----
    # 关键：第 k>0 层的整数输入 = 上一层的硬件 bit-exact 输出（ofm_expected_q）
    # 不能用 quantize_symmetric(acts[k], scales[k])，否则硬件 int 定点 vs PyTorch float
    # round 会有 off-by-1 偏差导致层间比对不一致
    os.makedirs(out_dir, exist_ok=True)
    summaries = []
    for k, (conv, relu_en) in enumerate(conv_chain):
        layer_out = (layer_dir_fn(k) if layer_dir_fn is not None
                     else os.path.join(out_dir, f"layer_{k:02d}"))
        ifm_override = summaries[k-1]['ofm_expected_q'] if k > 0 else None
        info = compile_layer.compile_and_emit_conv2d(
            conv, acts[k], scales[k], scales[k+1],
            out_dir=layer_out, streaming=streaming,
            seed_label=f"{model_name}_L{k}|model",
            ddr_ifb_base =bases[k]['ifb_base'],
            ddr_ofb_base =bases[k]['ofb_base'],
            ddr_wb_base  =bases[k]['wb_base'],
            ddr_desc_base=bases[k]['desc_base'],
            skip_ifb_preload=(k > 0),
            skip_ofb_clear  =(k < L - 1),
            emit_expected_ofm=(k == L - 1),
            ifm_int_override=ifm_override,
            relu_en=relu_en,
        )
        info['relu_en'] = relu_en
        info.update({
            'layer_idx': k, 'case_dir': layer_out,
            'is_first' : (k == 0), 'is_last' : (k == L - 1),
            'ddr_ifb_base' : bases[k]['ifb_base'],
            'ddr_ofb_base' : bases[k]['ofb_base'],
            'ddr_wb_base'  : bases[k]['wb_base'],
            'ddr_desc_base': bases[k]['desc_base'],
        })
        summaries.append(info)

    # ---- 写 model_plan.json ----
    plan = {
        'model_name': model_name,
        'n_layers'  : L,
        'streaming' : streaming,
        'layers': [
            {
                'idx': s['layer_idx'], 'case_dir': s['case_dir'],
                'K': s['K'], 'stride': s['stride'], 'pad': s['pad'],
                'Cin': s['Cin'], 'Cout': s['Cout'],
                'H_IN': s['H_IN'], 'W_IN': s['W_IN'],
                'H_OUT': s['H_OUT'], 'W_OUT': s['W_OUT'],
                's_x': s['s_x'], 's_y': s['s_y'],
                'sdp_mult': s['sdp_mult'], 'sdp_shift': s['sdp_shift'],
                'ddr_ifb_base' : s['ddr_ifb_base'],
                'ddr_ofb_base' : s['ddr_ofb_base'],
                'ddr_wb_base'  : s['ddr_wb_base'],
                'ddr_desc_base': s['ddr_desc_base'],
                'is_first': s['is_first'], 'is_last': s['is_last'],
            }
            for s in summaries
        ],
    }
    with open(os.path.join(out_dir, 'model_plan.json'), 'w') as f:
        json.dump(plan, f, indent=2)
    print(f"[compile_model] {model_name}: wrote {L} layers → {out_dir}")
    return summaries


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--model', choices=list(MODELS.keys()),
                   help='emit this model;')
    p.add_argument('--smoke', action='store_true')
    p.add_argument('--out-dir', default=DEFAULT_OUT_DIR,
                   help='plan 元数据目录 (model_plan.json)')
    p.add_argument('--sim-dir', default=None,
                   help='若设置，每层输出到 <sim-dir>/cases/caseNN (NN 从 --start-idx 起)')
    p.add_argument('--start-idx', type=int, default=0,
                   help='配合 --sim-dir 的起始 case 索引')
    args = p.parse_args()

    if args.smoke and not args.model:
        args.model = 'mnist2'

    if not args.model:
        print("compile_model.py: use --model <name> or --smoke")
        raise SystemExit(1)

    model, x, n_layers = build_model(args.model)

    layer_dir_fn = None
    if args.sim_dir is not None:
        def _mk_dir_fn(sim_dir, start):
            def inner(k):
                return os.path.join(sim_dir, "cases", f"case{start + k:02d}")
            return inner
        layer_dir_fn = _mk_dir_fn(args.sim_dir, args.start_idx)

    compile_and_emit_model(model, x, out_dir=args.out_dir, streaming=True,
                           model_name=args.model, layer_dir_fn=layer_dir_fn)
