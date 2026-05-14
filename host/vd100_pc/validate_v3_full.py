#!/usr/bin/env python3
"""v3 PDI 全面验证 master script.

Phase A: 既有功能回归 (验证 ODMA fix 没 break)
  A1. Stage 3a Phase 1: 1×1×1×1 N=200 (mac bypass-equivalent saturate)
  A2. Stage 3a Phase 2: K=3 H=W=8 Cin=Cout=16 N=100

Phase B: Stage 4 真 MAC bit-exact 递进 (验证 ODMA fix 真修了 yout_base)
  B1. K=1 H=W=1 Cin=Cout=16
  B2. K=1 H=W=2 Cin=Cout=16 (历史 row 1 fail case)
  B3. K=1 H=W=4 Cin=Cout=16
  B4. K=1 H=W=8 Cin=Cout=16
  B5. K=3 H=W=8 Cin=Cout=16 (目标 case)
  B6. K=3 H=W=16 Cin=Cout=16 (大 case)

Phase C: 软 reset 通路 (验证 CTRL.bit7 救活 stuck board)
  C1. 跑正常 case → 故意 ODMA_CMD_COUNT=1 stuck → 软 reset → 重跑 PASS

Phase D: 多轮 bit-exact (验证持续稳定 + 跨轮无累计错误)
  D1. K=3 H=W=8 bit-exact N=50

Phase E: 容量边界
  E1. 测一个大 case 找 BRAM 上限
"""
import sys, subprocess, os, time
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')

PY = sys.executable
HOST_DIR = r'C:/_Project/FLUX_CNN/host/vd100_pc'

def run(label, cmd, timeout=120):
    print(f'\n{"="*70}\n[{label}]\n  cmd: {" ".join(cmd[2:])[:100]}\n{"-"*70}')
    t0 = time.perf_counter()
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        dt = time.perf_counter() - t0
        # 打印 stdout 最后 15 行
        lines = r.stdout.strip().split('\n')
        for line in lines[-15:]:
            print(f'  {line}')
        passed = ('PASS=' in r.stdout and 'FAIL=0' in r.stdout) or \
                 ('1/1 PASS' in r.stdout) or \
                 ('All' in r.stdout and 'PASS' in r.stdout)
        ok = (r.returncode == 0) and passed
        print(f'  → {"✅ PASS" if ok else "❌ FAIL"} ({dt:.1f}s, rc={r.returncode})')
        return ok, dt, r.stdout
    except subprocess.TimeoutExpired:
        print(f'  → ❌ TIMEOUT after {timeout}s')
        return False, timeout, ''


results = []

# ====== Phase A: 既有功能回归 ======
print('\n' + '#' * 70)
print('# Phase A: 既有功能回归 (验证 ODMA fix 没 break Stage 3a)')
print('#' * 70)

results.append(('A1: Stage 3a Phase 1 N=200', *run(
    'A1', [PY, f'{HOST_DIR}/test_stage3a_phase1_loop.py', '--n', '200'], timeout=60)))

results.append(('A2: Stage 3a Phase 2 K=3 H=W=8 N=100', *run(
    'A2', [PY, f'{HOST_DIR}/test_stage3a_phase2_larger.py', '--n', '100',
           '--k', '3', '--h', '8', '--w', '8', '--cin', '16', '--cout', '16',
           '--stride', '1', '--pad', '1'], timeout=60)))

# ====== Phase B: Stage 4 bit-exact 递进 ======
print('\n' + '#' * 70)
print('# Phase B: Stage 4 真 MAC bit-exact 递进')
print('#' * 70)

B_cases = [
    ('B1: K=1 H=W=1',  ['--k', '1', '--h', '1', '--w', '1',  '--cin', '16', '--cout', '16', '--stride', '1', '--pad', '0']),
    ('B2: K=1 H=W=2',  ['--k', '1', '--h', '2', '--w', '2',  '--cin', '16', '--cout', '16', '--stride', '1', '--pad', '0']),
    ('B3: K=1 H=W=4',  ['--k', '1', '--h', '4', '--w', '4',  '--cin', '16', '--cout', '16', '--stride', '1', '--pad', '0']),
    ('B4: K=1 H=W=8',  ['--k', '1', '--h', '8', '--w', '8',  '--cin', '16', '--cout', '16', '--stride', '1', '--pad', '0']),
    ('B5: K=3 H=W=8',  ['--k', '3', '--h', '8', '--w', '8',  '--cin', '16', '--cout', '16', '--stride', '1', '--pad', '1']),
    ('B6: K=3 H=W=16', ['--k', '3', '--h', '16', '--w', '16','--cin', '16', '--cout', '16', '--stride', '1', '--pad', '1']),
]
for name, args in B_cases:
    results.append((name, *run(
        name, [PY, f'{HOST_DIR}/test_stage4_bitexact.py', '--n', '1'] + args, timeout=60)))

# ====== Phase C: 软 reset 通路 ======
print('\n' + '#' * 70)
print('# Phase C: 软 reset 通路 (CTRL.bit7)')
print('#' * 70)

results.append(('C1: Soft reset 通路', *run(
    'C1', [PY, f'{HOST_DIR}/test_soft_reset.py'], timeout=60)))

# ====== Phase D: 多轮 bit-exact ======
print('\n' + '#' * 70)
print('# Phase D: 多轮 bit-exact (验证持续稳定)')
print('#' * 70)

results.append(('D1: bit-exact N=50 (K=3 H=W=8)', *run(
    'D1', [PY, f'{HOST_DIR}/test_stage4_bitexact.py', '--n', '50',
           '--k', '3', '--h', '8', '--w', '8', '--cin', '16', '--cout', '16',
           '--stride', '1', '--pad', '1'], timeout=120)))

# ====== Summary ======
print('\n' + '#' * 70)
print('# Summary')
print('#' * 70)
n_pass = sum(1 for r in results if r[1])
n_total = len(results)
for name, ok, dt, _ in results:
    print(f'  {"✅" if ok else "❌"} {name:50s}  {dt:5.1f}s')
print(f'\nTotal: {n_pass}/{n_total} PASS')

sys.exit(0 if n_pass == n_total else 1)
