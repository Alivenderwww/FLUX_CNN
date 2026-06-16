#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_perf_report.py — 从 sim log 的 PERF_* 机读 tag 行生成 CSV 性能报告.

架构无关 / 模型无关 / 输入无关:
  - cores / layers / ddrs 由 PERF_RUN 行自描述, parser 不写死任何维度.
  - 每种 PERF_* 行用通用 key=value 解析, 新增字段自动进 CSV header (无需改 parser).
  - 缺某类行 (如无 PERF_DDR) 则不出该 CSV. 任何遵循 PERF_* 约定的 TB 都能直接用.

TB 侧约定 (tb_smc_chain.sv 已实现, 其它 TB 照此打印即可被本脚本解析):
  PERF_RUN        result=PASS layers=N cores=N ddrs=N wall_cyc=N clk_mhz=N
  PERF_LAYER      l=N cyc=N [name=...]
  PERF_LAYER_CORE l=N c=N cyc=N fire=N util_pct=F act_stall=N ... acc_idle=N
  PERF_CORE       c=N idma_grant=N ... data_cyc=N done_cyc=N
  PERF_DDR        d=N busy_cyc=N aw_fire=N w_beats=N ar_fire=N r_beats=N

输出 CSV (规范化多表, 适合 pandas / excel):
  <case>_run.csv / _layer.csv / _layer_core.csv / _core.csv / _ddr.csv

用法:
  python gen_perf_report.py <sim_log> [--out-dir DIR] [--case NAME] [--pe-per-core 256]
"""
import argparse
import csv
import os
import re

TAGS = ['PERF_RUN', 'PERF_LAYER', 'PERF_LAYER_CORE', 'PERF_CORE', 'PERF_DDR']


def parse_kv(rest):
    """把 'k1=v1 k2=v2 ...' 解析成有序 dict (保留出现顺序)."""
    d = {}
    for tok in rest.split():
        if '=' in tok:
            k, v = tok.split('=', 1)
            d[k] = v
    return d


def collect(text):
    """扫一遍 log, 按 tag 收集所有行的 key=value dict. \b 词边界让
    PERF_LAYER 不会误吃 PERF_LAYER_CORE (R 与 _ 之间无 boundary)."""
    rows = {t: [] for t in TAGS}
    pats = {t: re.compile(r'\b' + t + r'\b\s+(.*)') for t in TAGS}
    for line in text.splitlines():
        for t in TAGS:
            m = pats[t].search(line)
            if m:
                rows[t].append(parse_kv(m.group(1)))
                break
    return rows


def write_csv(path, dicts, extra_cols=None, deriv=None):
    """写一个 CSV. header = 所有 key 按首次出现顺序 + extra_cols.
    deriv(row) 可就地往 row 加派生列. 返回是否写了文件."""
    if not dicts:
        return False
    keys = []
    for d in dicts:
        for k in d:
            if k not in keys:
                keys.append(k)
    for k in (extra_cols or []):
        if k not in keys:
            keys.append(k)
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(keys)
        for d in dicts:
            row = dict(d)
            if deriv:
                deriv(row)
            w.writerow([row.get(k, '') for k in keys])
    return True


def _i(d, k, default=0):
    try:
        return int(d.get(k, default))
    except (ValueError, TypeError):
        return default


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log', help='sim log / transcript 文件路径')
    ap.add_argument('--out-dir', default=None, help='CSV 输出目录 (默认 = log 所在目录)')
    ap.add_argument('--case', default=None, help='case 名 (CSV 文件名前缀, 默认从 log 文件名推)')
    args = ap.parse_args()

    if not os.path.exists(args.log):
        ap.error(f'log 不存在: {args.log}')
    with open(args.log, 'r', encoding='utf-8', errors='replace') as f:
        text = f.read()

    rows = collect(text)
    if not any(rows.values()):
        print(f'[PERF] 未找到任何 PERF_* tag 行 (TB 是否已打印? log={args.log})')
        return 1

    out_dir = args.out_dir or os.path.dirname(os.path.abspath(args.log))
    os.makedirs(out_dir, exist_ok=True)
    case = args.case or os.path.splitext(os.path.basename(args.log))[0]
    prefix = os.path.join(out_dir, case)

    run = rows['PERF_RUN'][0] if rows['PERF_RUN'] else {}
    wall = _i(run, 'wall_cyc')
    cores = _i(run, 'cores', 1) or 1
    try:
        clk = float(run.get('clk_mhz', 100))
    except ValueError:
        clk = 100.0
    total_fire = sum(_i(d, 'fire') for d in rows['PERF_LAYER_CORE'])

    written = []

    # ---- run.csv (+ 派生 fps / total_mac_fire / pe_util_pct) ----
    if run:
        run = dict(run)
        run['fps'] = round(clk * 1e6 / wall, 1) if wall else ''
        run['total_mac_fire'] = total_fire
        # 整体 MAC pipe 利用率 = Σ(各核各层 stage-0 fire 拍数) / (cores × wall).
        # fire 已是 pipe 占用拍数, 跟 layer.csv 的 mac_pipe_pct 同口径 (硬件 pipe 利用率,
        # 非 ops 算术利用率 — 后者需每层理论 MAC, TB 未提供).
        run['mac_pipe_pct'] = round(100.0 * total_fire / (cores * wall), 2) if wall else ''
        if write_csv(f'{prefix}_run.csv', [run]):
            written.append(f'{case}_run.csv')

    # ---- layer.csv (+ pct_of_wall + mac_pipe_pct = 该层各核平均利用率) ----
    layer_fire = {}
    for d in rows['PERF_LAYER_CORE']:
        l = d.get('l')
        layer_fire[l] = layer_fire.get(l, 0) + _i(d, 'fire')

    def layer_deriv(row):
        cyc = _i(row, 'cyc')
        row['pct_of_wall'] = round(100.0 * cyc / wall, 2) if wall else ''
        lf = layer_fire.get(row.get('l'), 0)
        row['mac_pipe_pct'] = round(100.0 * lf / (cores * cyc), 2) if cyc else ''

    if write_csv(f'{prefix}_layer.csv', rows['PERF_LAYER'],
                 extra_cols=['pct_of_wall', 'mac_pipe_pct'], deriv=layer_deriv):
        written.append(f'{case}_layer.csv')

    # ---- layer_core.csv (原始, 最细粒度) ----
    if write_csv(f'{prefix}_layer_core.csv', rows['PERF_LAYER_CORE']):
        written.append(f'{case}_layer_core.csv')

    # ---- core.csv (原始) ----
    if write_csv(f'{prefix}_core.csv', rows['PERF_CORE']):
        written.append(f'{case}_core.csv')

    # ---- ddr.csv (+ busy_pct) ----
    def ddr_deriv(row):
        row['busy_pct'] = round(100.0 * _i(row, 'busy_cyc') / wall, 2) if wall else ''

    if write_csv(f'{prefix}_ddr.csv', rows['PERF_DDR'],
                 extra_cols=['busy_pct'], deriv=ddr_deriv):
        written.append(f'{case}_ddr.csv')

    res = run.get('result', '?') if run else '?'
    fps = run.get('fps', '?') if run else '?'
    putil = run.get('mac_pipe_pct', '?') if run else '?'
    print(f'[PERF] {case}: result={res} wall={wall}cy fps={fps} mac_pipe={putil}% '
          f'layers={run.get("layers","?")} cores={cores} -> {len(written)} CSV in {out_dir}')
    for w in written:
        print(f'[PERF]   {w}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
