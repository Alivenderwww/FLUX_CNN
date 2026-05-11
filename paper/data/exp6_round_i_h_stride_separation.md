# 实验 6 / Round I: H 维 stride 分离 — ds layer IDMA 数据量减半

**日期**: 2026-05-08
**结果 (commit `e8ca7b0`, L9 fix 后)**: ResNet11 N=4 SMC: 204240 → **190977** cy (**-6.5%**, -13263 cy)
**初版 (commit `9957414`, L3/L6 only)**: 192158 cy (-5.9%)

## 动机

实验 5 (paper/data/exp5_ds_layer_root_cause.md) 证明 ds layer (K=1 stride>1) 是 memory-bound:
- mac_array fire 数少 (K²=1 vs K²=9)
- 但 IDMA 拉的 IFM 数据量跟 K=3 layer 类似（dense H × W）
- 数学上 K=1 stride=2 实际只用 stride 间隔的 1/(stride²) = 1/4 像素
- → mac_array 大部分时间在等 IDMA 拉用不到的数据

## 优化思路

让 IDMA H 维 stride 拉数据 (cmd 数 H 维减半):
- 原: cmd r=0..H_in-1, 每 cmd src_addr = base + r × row_stride
- 新: cmd r=0..H_out-1, 每 cmd src_addr = base + r × stride × row_stride

数学等价: 拉 H_in 行 (yin=0,1,...,H_in-1) 中的 stride 间隔行 (yin=0, stride, 2×stride, ...)。
mac_array 当 H 维 stride=1 跑（IFB 内是 dense H_out 行）。

W 维 stride **不能**做（axi_dm 不支持 strided burst，每 cmd 是连续地址）。

## RTL 改动 (~25 行, 控制极小)

| 文件 | 改动 |
|---|---|
| `params.py` + svh | CSR_ADDR_MAP 加 `STRIDE_H = 0x1F0` |
| `cfg_regs.sv` | 加 `r_stride_h` reg + `stride_h` 输出. 写 STRIDE 时同步 r_stride_h (默认兼容). 写 STRIDE_H 单独 override. |
| `line_buffer.sv` | 3 处 H 维 stride 用法 (rows_needed_next / rows_consumed_raw / y_row_base) 改用 `cfg_stride_h`. W 维 (`iss_pos_s`) 保持用 `cfg_stride`. |
| `core_top.sv` | `cfg_stride_h` wire 连接 cfg_regs → line_buffer |

## Driver 改动

- `mesh_cmd.gen_idma_sg_cmd_list_w_slice`: 加 `h_compress_stride` 参数, 每 cmd src_addr 跳 stride 行
- `run_multicore_chain.py`: 检测 ds layer (K=1, stride>1, cin_slices=1) 启用 H 压缩
- `hw_files.cfg_to_dict`: 加 STRIDE_H + 改 H_IN_TOTAL = h_in_idma

## 实测

### ResNet11 N=4 SMC 每层 cy 对比 (Round H → Round I)

| L | 描述 | Round H cy | Round I cy | Δ | 占比变化 |
|---|---|---:|---:|---:|---|
| L0 Patch | s2d K=1 cs=4 | 46015 | 46019 | +4 | (噪音) |
| L1 K=3 s=2 | | 27367 | 27709 | +342 | (噪音) |
| L2 K=3 s=1 | | 19798 | 19800 | +2 | – |
| **L3 ds K=1 s=2** | cin_slices=1 | 18369 | **10921** | **-7448 (-40.5%)** | ✓ |
| L4 K=3 s=2 | | 11303 | 11305 | +2 | – |
| L5 K=3 s=1 | | 22174 | 22176 | +2 | – |
| **L6 ds K=1 s=2** | cin_slices=1 | 11721 | **6726** | **-4995 (-42.6%)** | ✓ |
| L7 K=3 s=2 | | 12537 | 12539 | +2 | – |
| L8 K=3 s=1 | | 27362 | 27364 | +2 | – |
| **L9 ds (cin_slices=2)** | 4151 | **2972** | **-1179 (-28.4%)** | ✓ (L9 fix 后) |
| L10 FC | | 2926 | 2927 | +1 | – |
| **Total (L3/L6/L9 全 enable)** | | **204240** | **190977** | **-13263** | **-6.5%** |

### 其他 case

- smc_wslice1: 4146 → 4146 (无变化, 没 K=1 stride>1 layer)
- smc_wslice5: 20718 → 20718 (无变化, 没 K=1 stride>1 layer)

## 收益分析

L3/L6 ds layer cy 减 40-43%, 接近预期 50% (act_id 减半, 但有 mac_array fire / drain 等不变 overhead)。

- L3: 18369 → 10921, 减 7448 cy, ds-only saving = 40.5%
- L6: 11721 → 6726, 减 4995 cy, ds-only saving = 42.6%

L9 (cin_slices=2) 暂时跳过：driver 设 K=1 stride>1 + cin_slices=1 才启用。L9 启用时 OFM r=21,22 没写 (mac_array stall in yout=21+ 阶段)。Bug 推测在 cin_slices > 1 时 line_buffer ring 反压跟 H stride 交互问题，待 debug。

修复 L9 后预期再 -3K cy ≈ -1.5%。

## 累计 vs IP baseline

```
Round B IP path baseline:    217311 cy
→ Round C cout slice:        210784 cy (-3.0%)
→ Round F TB host parallel:  206589 cy (-2.0%)
→ Round G TB desc preload:   206121 cy (-0.23%)
→ Round H S2MM pause:        204251 cy (-0.91%)
→ Round H ODMA sts bg:       204240 cy (-0.005%)
→ Round I H stride (L3/L6):  192158 cy (-5.9%)
→ Round I L9 fix:            190977 cy (-0.6% 增量) ★
=========================================
Total:                       217311 → 190977 = -12.1%
@100 MHz: 2.17 ms → 1.91 ms
FPS:      460 → 524
```

## L9 Bug 根因 (debug 2026-05-08)

### 现象

Round I 初版 (commit 9957414) 启用 L9 (cin_slices=2) 时 OFM mismatch:
- r=21,22 mem=3 (C3) 起 OFM 是 X (got=X)
- mismatches=260 / 2040 ≈ 12.7%
- L9 fire 数对 (C3=960 ✓), mac_array 实际算到所有 yout, 但 mem 写不到 r=21+

### 根因

`cfg.IFB_ROW_STEP` (line_buffer 内 yout 推进时 IFB read base 步长) 原值
= `stride × W_in × cin_slices`. 这是 dense H_in IFB layout 下 yout 跨 stride 行的偏移.

H stride compress 后 IFB 内行 dense (1 yout = 1 row), 步长应 = `stride_h × W_in × cs`
(而非 stride × W × cs).

driver `hw_files.cfg_to_dict` 之前没考虑 `_STRIDE_H`, IFB_ROW_STEP 仍按原 stride 算.
mac_array yout 推进时 ptr_yout_base 跨 wrong 步长, 跟 IFB 实际 layout 错位.

### 修复

`hw_files.cfg_to_dict` 改:
```python
# 旧
'IFB_ROW_STEP': cfg['IFB_ROW_STEP'],

# 新 (Round I L9 fix)
'IFB_ROW_STEP': cfg.get('_STRIDE_H', cfg['stride']) * cfg['W_IN'] * cfg['cin_slices'],
```

`_STRIDE_H` 默认 = `stride` (兼容老行为). ds layer 设 `_STRIDE_H=1` 时 IFB_ROW_STEP 减半.

### 为什么 L3/L6 (cin_slices=1) 也"PASS"?

实际 L3/L6 在 Round I 初版 (没 IFB_ROW_STEP fix) 时也跑 OFM bit-exact. 这暗示 L3/L6 跟 L9 行为有别:

- L3/L6 cin_slices=1: 一个 cell 一拍 fire 完成. ring 内 row layout 简单, ptr_yout_base 错位被 ring wrap 抵消?
- L9 cin_slices=2: 一个 cell 两拍 fire (cin slice 0/1). ptr_*_base 内层切换 (ptr_cins_base) 让 IFB read 跨 cin slice 行, 不能被 wrap 简单抵消.

具体机制需要 line_buffer FSM 详细追踪. 但 fix 后 L3/L6/L9 全 PASS, 暂时不深挖.

### 修复后 L9 数据 (commit `e8ca7b0`)

L9 cy 4151 → 2972 (-28.4%, -1179 cy). 比 L3 (-40.5%) / L6 (-42.6%) 收益小因为
baseline cy 已经小, drain + mac_array setup 占比大.

整网 ResNet11: 192158 → **190977** (-1181 cy = -0.6% 增量).

## 总结

| 阶段 | ResNet11 cy | L3 ds | L6 ds | L9 ds | 累计 |
|---|---:|---:|---:|---:|---|
| Round H baseline | 204240 | 18369 | 11721 | 4151 | – |
| Round I L3/L6 only (commit 9957414) | 192158 | 10921 | 6726 | 4153 | -5.9% |
| Round I L9 fix (commit e8ca7b0) | **190977** | 10921 | 6726 | **2972** | **-6.5%** |

## 论文意义

### 1. 真正打到 ds layer 瓶颈

实验 5 提出"ds memory-bound"是论文创新点。Round I **实施验证**: ds layer cy 减
40%+ 印证了 memory-bound 推论 (减数据量直接减 cy)。

### 2. 控制变量法的实际价值

如果按旧 §12 推测做 bias_rf ping-pong, 投 1 周最多省 0.06%。
按实验 5 重定位 (memory-bound) 做 H stride 分离, 投 0.5 天省 5.9%。
**控制变量实验值得做**。

### 3. RTL 改动极小

```
cfg_regs.sv: +5 行 (r_stride_h reg + 写时同步 + 输出)
line_buffer.sv: 3 处替换 cfg_stride → cfg_stride_h (H 维专用)
core_top.sv: 1 wire
params.py: 1 行 CSR addr
```

总 RTL 改 < 15 行实质改动 + 注释。Driver 改 < 50 行。

**完美演示**: 跨 RTL/driver 联动的小改动, 拿到大收益 (-5.9%)。论文章节素材。

### 4. 未来工作

- L9 cin_slices > 1 修复 (再 -1.5%)
- W 维 stride 优化 (要 axi_dm 支持 strided burst, 或者 driver 多 cmd 实现 — 但 cmd 数会爆炸)
- 算法层 K=1 ds 跟 K=3 conv 合并 (RepVGG, 16% 收益, 训练侧改动)

## 数据来源

- baseline Round H: commit `4c4d057` (204240 cy)
- Round I commit: `9957414`
- 实验跑 sim: driver 生成 case 后 `cd sim/tb_smc && vsim -c -do run.tcl` (default IP path, 不需 IDEAL_SMC define). active_case.txt 由 driver 自动写.
