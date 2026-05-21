"""compile.py — 编译链统一 CLI 入口 (Phase C).

把 frontend / midend / backend 串成一条 case 生成流水, 替代历史 run_multicore_chain.py.

Phase C 实现策略: thin wrapper, 内部仍调 run_multicore_chain.main_impl. 这样:
  - 新 CLI 立即可用
  - 老 CLI (run_multicore_chain.py) 不破坏
  - Phase D/E 重构 frontend / midend 内部时 compile.py 接口不变

Usage:
    # 1. 一站式: PyTorch / Chain DSL → case
    python -m toolchain.compile \\
        --hw toolchain/hardware/vd100.json \\
        --model wslice1 \\
        --out cases/vd100_wslice1_n1/ \\
        [--n-cores 1]

    # 2. 列出 demo / hardware
    python -m toolchain.compile --list-demos
    python -m toolchain.compile --list-hw

未来 (Phase D+):
    python -m toolchain.compile \\
        --hw vd100.json \\
        --model resnet11.quant.yaml \\
        --input runtime_test.png \\
        --out case/ \\
        --dump-ir front.yaml mid.yaml
"""
from __future__ import annotations

import argparse
import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)


def _list_demos():
    """从 run_multicore_chain.py argparse choices 提取 demo 列表."""
    import re
    rmc = os.path.join(_THIS_DIR, 'run_multicore_chain.py')
    with open(rmc, 'r', encoding='utf-8') as f:
        text = f.read()
    m = re.search(r"choices=\[(.*?)\]", text, re.DOTALL)
    if not m:
        return []
    items = re.findall(r"'([^']+)'", m.group(1))
    return items


def _list_hw():
    """列出 hardware/ 下所有 *.json."""
    hw_dir = os.path.join(_THIS_DIR, 'hardware')
    return [f for f in os.listdir(hw_dir) if f.endswith('.json') and not f.startswith('_')]


def main():
    ap = argparse.ArgumentParser(
        description="FLUX_CNN 编译链统一入口 (Phase C)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument('--hw', default='toolchain/hardware/vd100.json',
                    help='Hardware JSON path (default vd100.json)')
    ap.add_argument('--model', default=None,
                    help='Chain DSL demo name (e.g. wslice1, resnet11) 或 .yaml/.pt 路径')
    ap.add_argument('--out', default=None,
                    help='Output case dir (default sim/tb_smc/cases/<auto>)')
    ap.add_argument('--n-cores', type=int, default=None,
                    help='Override deployment.num_cores (default 用 hw json 里值)')
    ap.add_argument('--list-demos', action='store_true', help='列出所有内置 demo')
    ap.add_argument('--list-hw', action='store_true', help='列出 hardware/*.json')
    ap.add_argument('--dump-ir', action='store_true',
                    help='导出 frontend.yaml / midend.yaml 到 case dir (Phase D+ 完整支持)')
    args = ap.parse_args()

    if args.list_demos:
        demos = _list_demos()
        print(f"Available demos ({len(demos)}):")
        for d in demos:
            print(f"  - {d}")
        return 0
    if args.list_hw:
        hws = _list_hw()
        print("Available hardware configs:")
        for h in hws:
            print(f"  - toolchain/hardware/{h}")
        return 0

    if args.model is None:
        ap.error("--model 必填 (demo name 或 .yaml/.pt 路径). 见 --list-demos")

    # 加载硬件配置
    from hardware.hardware_cfg import HardwareConfig
    hw_path = args.hw
    if not os.path.isabs(hw_path):
        # 相对路径: 相对项目根
        proj_root = os.path.dirname(_THIS_DIR)
        hw_path = os.path.join(proj_root, hw_path)
    cfg = HardwareConfig.load(hw_path)
    n_cores = args.n_cores if args.n_cores is not None else cfg.deployment.num_cores
    print(f"[compile] hw={hw_path}  target={cfg.deployment.target}  n_cores={n_cores}")

    # 设置 SMC global base (用于 run_multicore_chain 的 SMC_GLOBAL_BASE 环境变量)
    os.environ['FLUX_HW_JSON'] = hw_path
    os.environ['FLUX_SMC_GLOBAL_BASE'] = f"0x{cfg.deployment.ddr_base:08x}"

    # 判断 model 类型: demo name / yaml / pt
    if args.model.endswith('.yaml') or args.model.endswith('.yml') or args.model.endswith('.json'):
        # Chain DSL 文件
        return _compile_from_dsl(args.model, args.out, n_cores, cfg)
    elif args.model.endswith('.pt'):
        # PyTorch checkpoint
        ap.error("Phase E 前 .pt 走 run_model.py, compile.py 暂不直接支持")
    else:
        # demo name (Chain DSL 已注册在 run_multicore_chain.py)
        return _compile_from_demo(args.model, args.out, n_cores, cfg)


def _compile_from_demo(demo: str, out_dir: str | None, n_cores: int, cfg):
    """直接调 run_multicore_chain.run_multicore_chain (in-process, F4)."""
    case_name = f"vd100_{demo}_n{n_cores}"
    if out_dir is None:
        # 跟 run_multicore_chain 默认输出路径一致 sim/tb_smc/cases/<name>
        out_root = os.path.normpath(os.path.join(_THIS_DIR, '..', 'sim', 'tb_smc', 'cases'))
        out_dir = os.path.join(out_root, case_name)
    os.makedirs(out_dir, exist_ok=True)

    # 加载 layer list (demo 名 → Chain DSL)
    import run_multicore_chain as rmc
    layers = rmc.load_demo_layers(demo)
    print(f"[compile] demo={demo} → {len(layers)} layers, case_dir={out_dir}")

    chain_data, per_core_plan = rmc.run_multicore_chain(
        layers=layers, n_cores=n_cores, out_dir=out_dir, smc=True)
    print(f"[compile] OK: {len(chain_data)} layers, {n_cores} cores")
    return 0


def _compile_from_dsl(yaml_path: str, out_dir: str | None, n_cores: int, cfg):
    """从 Chain DSL YAML 文件编译 (Phase D 完整支持). 当前 stub: 暂未实现."""
    print(f"[compile] DSL file: {yaml_path}")
    print("  [TODO] Phase D 后启用 frontend.dsl_loader → midend → backend 直跑路径.")
    print("  目前请用 demo name (见 --list-demos).")
    return 2


if __name__ == '__main__':
    sys.exit(main() or 0)
