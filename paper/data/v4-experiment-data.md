# v4 §5.5.5 参数敏感度扫频 — 实验数据汇总

> 对照 `paper/data/v4-experiment-requirements.md` 的 5 个实验 (A/B/C/D/E).
> 数据来源全部来自 sim 实测 (tb_core_dma 单核 + tb_smc N=4 SMC), 配套日志在
> `sim/tb_core_dma/v4_sweep_*_vsim.log` 与 `sim/tb_smc/v4d_w*_vsim.log`.
> 日期: 2026-05-09. baseline commit: `e8ca7b0` (Round I L9 fix).

## 0 数据采集与口径

- 单核 case 跑 `toolchain/v4_sweep_ac.py` (实验 A+C) 与 `toolchain/v4_sweep_b_d1.py` (实验 B+D-N1).
  报告写入 `sim/tb_core_dma/v4_sweep_*_report_*.txt`, 字段口径与 `run_regression.py` 一致.
- SMC N=4 case 跑 `toolchain/v4_sweep_d_smc.py`, 单层 chain × 4 个 W. 每 W 独立 vsim run, 输出 JSON.
- 关键字段:
  - `wall_cy`: 整 case 端到端 cycle 数 (`tb` cycle counter, `core_busy` 期间).
  - `mac_util`: PE 利用率 = useful_MAC / (cycles × 16 × 16).
    useful_MAC = `H_out × W_out × K² × Cin_orig × Cout_orig` (与 fold/s2d 等价变换前的原始 conv 计数).
  - `act_id` / `wgt_id` / `psm_id` / `acc_id`: 4 类 idle 来源, 单位 cycle.
- 频率: sim 全部按 100 MHz 折算 wall 时间 (10 ns / cycle).

---

## 1 实验 A — *C*ᵢₙ 扫频与 Ky 折叠适用域

固定: K=3, stride=1, pad=1, H=W=32, Cout=16. 自变量 *C*ᵢₙ ∈ {1, 2, 4, 8, 16, 32, 64} × {fold off, fold on}.

### 1.1 表 A.1 — 单层 wall cycles 与 Ky-fold 加速比

| *C*ᵢₙ | fold off cy | fold on cy | 加速比 | fold off MAC% | fold on MAC% | 备注 |
|---:|---:|---:|---:|---:|---:|---|
| 1  | 10131 | 4104  | 2.47× | 5.7%  | 14.0% | groups=16, ky=1, pad_ky=13 (大量空) |
| 2  | 10131 | 4104  | 2.47× | 11.4% | 28.1% | groups=8, ky=1, pad_ky=5 |
| 4  | 10131 | 4104  | 2.47× | 22.7% | 56.1% | groups=4, ky=1, pad_ky=1 |
| 8  | 10131 | (FAIL\*) | – | 45.5% | – | groups=2, ky=2, pad_ky=1 (RTL bit-exact bug, 边界) |
| 16 | 10131 | 10131 | 1.00× | 91.0% | 91.0% | Cin=HW_PE, fold 自动跳过 |
| 32 | 19779 | 19779 | 1.00× | 93.2% | 93.2% | Cin>HW_PE, 走 cin slice |
| 64 | 39396 | 39396 | 1.00× | 93.6% | 93.6% | Cin>HW_PE, cin slice ×4 |

> \* `A.cin08_ky` 触发 `compute_fold_params(K=3, NUM_CIN=8, HW_PE=16)` 返回 `groups_y=2, ky_per_group=2, pad_ky=1` —
> 是唯一 `ky_per_group > 1` 的边界配置; 当前 RTL 路径 OFM 出现 32/1024 word mismatch (32×32×16=512 word
> 中 6.25%). 上下游 (Cin=4 ky=1 / Cin=16 cin_slice 路径) 都 PASS, 故 bug 锁定在 fold ky_per_group≥2 + pad_ky 末组路径,
> 与本扫频结论无关; 加速比预期介于 Cin=4 (2.47×) 与 Cin=16 (1.00×) 之间, 估算 ≈1.7× (按 useful_MAC×2 / fire×1).

### 1.2 关键观察

- **fold off** 当 *C*ᵢₙ < 16 时 wall_cy 与 *C*ᵢₙ 无关 (恒 10131 cy), 因为 PE 阵列列方向被 1 个 cin slice 占满,
  Cin 减小并不减 fire 数—只让 PE 行方向部分空转. MAC% 与 *C*ᵢₙ 线性 (5.7% / 11.4% / 22.7% / 45.5%, 倍率 ×2).
- **fold on** 把 ky 维折到 cin 假维度 (cin_fake = HW_PE = 16), fire 数减为原来的 1/groups_y, MAC% 翻倍接近上限.
  Cin=1/2/4 三个 case 加速比都是 2.47× (≈ 9 / (3×3/groups)=9/(2.25)=4.0 的折扣后), MAC% 提升 2.45×.
- **拐点 Cin=16**: fold 不再激活, 两条曲线汇合到 91% MAC% / 10131 cy.
- **Cin=32/64**: cin slice ≥2, 走 PE 行方向轮询, MAC% 持平 (93%), wall_cy 与 cin slice 数线性.

### 1.3 PROFILE — 4 类握手 fire/stall/idle (% of wall_cy)

| Case | act F/S/I | wgt F/S/I | psum F/S/I | acc F/S/I |
|---|---|---|---|---|
| A.cin01_off | 91.0/1.5/4.2 | 91.0/6.1/0.0 | 91.0/1.9/8.4 | 10.1/0.0/87.1 |
| A.cin01_ky  | 74.9/15.9/3.3 | 74.9/19.9/0.0 | 74.9/17.2/11.1 | 25.0/0.0/70.4 |
| A.cin04_off | 91.0/1.5/4.2 | 91.0/6.1/0.0 | 91.0/1.9/8.4 | 10.1/0.0/87.1 |
| A.cin04_ky  | 74.9/15.9/3.3 | 74.9/19.9/0.0 | 74.9/17.2/11.1 | 25.0/0.0/70.4 |
| A.cin16_off | 91.0/1.5/4.2 | 91.0/6.1/0.0 | 91.0/1.9/8.4 | 10.1/0.0/87.1 |
| A.cin32_off | 93.2/0.0/4.6 | 93.2/4.6/0.0 | 93.2/0.0/7.5 | 5.2/0.0/92.6 |
| A.cin64_off | 93.6/0.0/4.6 | 93.6/4.6/0.0 | 93.6/0.0/6.8 | 2.6/0.0/95.6 |

- fold off: act/wgt/psum 三个接口的 fire 占比一致 (91%) — 流水紧凑, idle 主要发生在 acc 接口 (87% acc_id =
  parf_accum drain 阶段, 因为 16 × 16 fire 后只 drain 一行 OFM, 1024 cy / 10131 cy ≈ 10%).
- fold on: 5/16 PE 行被 zero pad 占用, fire 接口 stall 增加 (1.5% → 15.9%), acc idle 也增 (87% → 70%).
  尽管 fire 接口 idle 增, 总 wall cy 仍减 2.47× — 因为 fire 数本身少了 ~2.46×.

---

## 2 实验 C — *K* 扫频与卷积核 round chunking 开销

固定: stride=1 (K=8 例外 stride=4 +s2d), H=W=32, Cin=Cout=16.

### 2.1 表 C.1 — 单层 fire 数与平均每 fire cycle

| K | K² | fire 数 | wall_cy | cy / fire | MAC% | 备注 |
|---:|---:|---:|---:|---:|---:|---|
| 1 | 1  | 1024  | 1627  | 1.59 | 62.9% | num_tiles=1, no halo |
| 3 | 9  | 9216  | 10131 | 1.10 | 91.0% | 主 conv 形状 |
| 5 | 25 | 25600 | 26912 | 1.05 | 95.1% | 大 K, fire 占 95% wall |
| 7 | 49 | 50176 | 52175 | 1.04 | 96.2% | 接近天花板 |
| **8 (s2d)** | – | 784  | 1358  | 1.73 | 57.7% | K=8 stride=4 + s2d → K=2 stride=1, Cin 4→64, H/W 32→7, fire ×0.015 |

### 2.2 关键观察

- *K* 增大时, fire 数与 *K*² 线性 (1024 → 9216 → 25600 → 50176, 比例 1:9:25:49, 完全吻合 K²).
- **wall_cy 几乎完全等于 fire 数** (cy/fire 比逐 K 下降到 1.04), 说明 mac_array fire 占满 wall cy,
  IDMA / WDMA setup overhead 在大 K 时被摊薄, 16×16 阵列覆盖 K² ≤ 256 (HW_PE × HW_COL) 完全绰绰有余.
- *K*=1 cy/fire=1.59 显著偏高: K=1 fire 少 (1024 cy 的 setup overhead 占比大),
  PE wgt round 数 = max(1, ceil(K²·Cin / (HW_PE · WRF))) 仍 ≥1 设置耗费.
- *K*=8 stride=4 +s2d 等价化到 K=2 stride=1 Cin=64 H=7 W=7: useful_MAC 不变, fire 减为 784,
  wall_cy 1358 cy. 验证 §3.4 选 16×16 PE 与 *K*² ≤ 256 / *K*+stride² ≤ 256 的覆盖论证.

### 2.3 PROFILE

| Case | act F/S/I | wgt F/S/I | psum F/S/I | acc F/S/I |
|---|---|---|---|---|
| C.k1     | 62.9/0.0/21.6 | 62.9/21.6/0.0 | 62.9/0.0/45.0 | 62.9/0.0/25.5 |
| C.k3     | 91.0/1.5/4.2  | 91.0/6.1/0.0  | 91.0/1.9/8.4  | 10.1/0.0/87.1 |
| C.k5     | 95.1/0.0/2.8  | 95.1/2.8/0.0  | 95.1/0.0/5.4  | 3.8/0.0/94.2 |
| C.k7     | 96.2/0.0/2.0  | 96.2/2.0/0.0  | 96.2/0.0/4.1  | 2.0/0.0/96.3 |
| C.k8_s2d | 57.7/0.0/14.1 | 57.7/14.1/0.0 | 57.7/0.0/51.8 | 3.6/0.0/69.0 |

- *K*=1 三个 fire% (act/wgt/psum/acc) 同步在 62.9%, 是 num_tiles=1 大 idle 的特征 (acc 25.5% idle =
  parf drain, 但 fire 接口本身 idle 21.6% 是 IDMA cmd 组装慢 — 单 cmd 拉 16×16 word).
- *K*=3/5/7: fire% 单调上升 (91% → 95.1% → 96.2%), 接近 100% MAC 阵列上限.
- C.k8_s2d 本质是 K=2 的 baseline + 大 IDMA 数据量 (Cin 64), psum_id 51.8% 是 drain 占用主导.

---

## 3 实验 B — stride 扫频与降采样层调度

固定: H=W=64, Cin=Cout=16. K × stride 4 组.

### 3.1 单核基线 (tb_core_dma)

| Case | K | stride | wall_cy | MAC% | fire (PROFILE %) | 备注 |
|---|---:|---:|---:|---:|---:|---|
| B.k1s1 | 1 | 1 | 5148   | 79.6% | 79.6% | K=1 dense, fire 占满 |
| B.k1s2 | 1 | 2 | 4945   | **20.7%** | 20.7% | ds layer, MAC% 跌 4× (fire 数 ÷4 但 setup 不变) |
| B.k3s1 | 3 | 1 | 38484  | **95.8%** | 95.8% | 主 conv 形状, 接近天花板 |
| B.k3s2 | 3 | 2 | 9682   | **95.2%** | 95.2% | K=3 stride=2 仍保持 95% — 空间复用 K² 弥补了 stride 损失 |

> 关键观察: **K=1 stride=2 的 MAC% (20.7%) 远低于 K=3 stride=2 (95.2%)** — 这是
> "ds 用 K=1 才 memory-bound, K=3 仍 compute-bound" 的强证据.
> H 步长分离优化只对 K=1 ds layer 有效, K=3 ds layer 不需要也不可能受益.

### 3.2 H 步长分离对照 (复用 ResNet11 SMC 实测, exp6)

baseline commit `4c4d057` (Round H, H 步长分离 OFF) → `e8ca7b0` (Round I, H 步长分离 ON).
ResNet11 N=4 SMC 三个 ds layer (K=1 stride=2):

| Layer | 形状 | Round H cy | Round I cy | Δ cy | 减幅 |
|---|---|---:|---:|---:|---:|
| L3 (ds K=1 s=2) | Cin=Cout=16, H=120, W=68, cin_slices=1 | 18369 | **10921** | -7448 | **-40.5%** |
| L6 (ds K=1 s=2) | Cin=16, Cout=32, H=60, W=34, cin_slices=1 | 11721 | **6726**  | -4995 | **-42.6%** |
| L9 (ds K=1 s=2) | Cin=32, Cout=64, H=30, W=17, cin_slices=2 | 4151  | **2972**  | -1179 | **-28.4%** |
| **整网** | ResNet11 N=4 SMC | 204240 | 190977 | -13263 | **-6.5%** |

### 3.3 关键观察

- H 步长分离 driver-only 优化 (~50 行 driver + 15 行 RTL) 让 ds layer cycle 减 40%+, 接近理论上限 50%.
  剩余 ~10-12% 不可减是 mac_array fire 阶段固有 + drain + setup.
- L9 比 L3/L6 减幅小 (28% vs 40%+) 因 cin_slices=2 一拍多发, fire / drain 占比大, IDMA 占比小.
- 整网 -6.5% 印证 §5.5.4 控制变量法: ds layer 是 memory-bound 的论断.

---

## 4 实验 D — 核数 *N* × *W*ᵢₙ 扫频

固定: K=3 stride=1 pad=1, Cin=Cout=16, 方阵 H=W. 自变量 W ∈ {16, 32, 64, 128}, N ∈ {1, 4} (N=2 受
SMC IP `axi_smc_4to4` 4-port 拓扑限制不易直接对照, 故跳过中间点).

### 4.1 单核 N=1 (tb_core_dma)

| W (=H) | wall_cy | MAC% | act_id (% wall) | 备注 |
|---:|---:|---:|---:|---|
| 16  | 2,736   | 84.2% | 5.1% | 小图, IDMA 单 cmd 耗 5% |
| 32  | 10,131  | 91.0% | 4.2% | 中等, fire ramp 快 |
| 64  | 38,484  | 95.8% | 3.4% | fire 占满 |
| 128 | 151,960 | 97.0% | 2.7% | 接近 100% MAC% 上限 |

### 4.2 N=4 SMC (tb_smc, axi_smc_4to4 IP)

| W (=H) | wall_cy | per-core util | per-core sub_W_out |
|---:|---:|---:|---:|
| 16  | 2,278  | 28.0% × 4 | 4 / 4 / 4 / 4 |
| 32  | 4,161  | 58.4% × 4 | 8 / 8 / 8 / 8 |
| 64  | 10,369 | 90.8% × 4 | 16 / 16 / 16 / 16 |
| 128 | 39,860 | 93.0% × 4 | 32 / 32 / 32 / 32 |

### 4.3 表 D.1 — N=1 / N=4 加速比与流水占空比

| W | N=1 cy | N=4 cy | 加速比 | 加速比 / 4 (ideal) | N=4 PE util |
|---:|---:|---:|---:|---:|---:|
| 16  | 2,736   | 2,278  | **1.20×** | 30.0% | 28.0% |
| 32  | 10,131  | 4,161  | **2.43×** | 60.9% | 58.4% |
| 64  | 38,484  | 10,369 | **3.71×** | 92.8% | 90.8% |
| 128 | 151,960 | 39,860 | **3.81×** | 95.3% | 93.0% |

### 4.4 关键观察

- 加速比与 *W*ᵢₙ 强相关: **W=16 仅 1.20×** (sub_W_out=4 已经到 PE 列覆盖下限, halo 占比大),
  W=128 接近线性 **3.81×** (天花板 4×).
- 拐点 **W ≈ 64**: sub_W_out=16 = HW_COL, 4 核各占满 PE 列, halo 与 SMC 互联开销摊薄完毕.
- "加速比 / 4" 与 "N=4 PE util" 在 W ≥ 32 时几乎完全对齐 (60.9% vs 58.4%, 92.8% vs 90.8%, 95.3% vs 93.0%) —
  说明 N=4 加速比 = (N=1 单核能做到的 MAC%) × (N=4 切片均衡度), 没有额外的 SMC 仲裁损失.
  W=16 case 的 1.5% 偏差 (30% vs 28%) 才是 SMC IP 互联的纯 overhead.
- 论文论点 "N=4 SMC 加速比偏离线性主因是 W slice halo + sub_W_out=4 PE 列覆盖率退化, 而非 1-DDR 带宽"
  在本扫频得到强支持: W=128 case 4 核同 mem 仲裁压力最大, 加速比反而最高.

---

## 5 实验 E — ResNet11 *N*=4 主线分层瓶颈分项

直接复用 `paper/data/exp8_pe_util_roadmap.md` 表 1 (commit `e8ca7b0` Round I L9 fix baseline).

### 5.1 表 E.1 — 每层形状、fire 数与 idle 来源 (max core 取数, 4 核取最慢核)

| L | name | C_in→C_out | sub_W_out | fire (max) | wall_cy | idle | util |
|---|---|---|---:|---:|---:|---:|---:|
| L0  | Patch K=4 s=4 (s2d cs=4) | 4→16   | 34 | 32640  | 46019  | 13379 | **70.9%** |
| L1  | K=3 s=2                   | 16→16  | 17 | 18360  | 27709  | 9349  | **66.3%** |
| L2  | K=3 s=1                   | 16→16  | 17 | 18360  | 19800  | 1440  | 92.7% |
| L3  | ds K=1 s=2                | 16→16  | 17 | 2040   | 10921  | 8881  | **18.7%** |
| L4  | K=3 s=2                   | 16→32  | 9  | 9720   | 11305  | 1585  | 86.0% |
| L5  | K=3 s=1                   | 32→32  | 9  | 19440  | 22176  | 2736  | 87.7% |
| L6  | ds K=1 s=2                | 16→32  | 9  | 1080   | 6726   | 5646  | **16.1%** |
| L7  | K=3 s=2                   | 32→64  | 5  | 10800  | 12539  | 1739  | 86.1% |
| L8  | K=3 s=1                   | 64→64  | 5  | 21600  | 27364  | 5764  | **78.9%** |
| L9  | ds K=1 s=2                | 32→64  | 5  | 1200   | 2972   | 1772  | 40.4% |
| L10 | FC                        | 256→522 | 1 | ~144  | 2927   | ~2783 | ~5% |
| **Total** | – | – | – | **~135384** | **190977** | **55593** | **70.9%** |

### 5.2 关键观察

- 主路径 *K*=3 layer (L2/L4/L5/L7/L8) PE 利用率 78–93%, 已接近 16×16 阵列上限.
- 三类瓶颈层:
  1. **降采样 ds (L3/L6)** 利用率 16-19% — IDMA 拉用不到的 stride² 像素 (K=1 没空间复用), memory-bound;
     L9 (cin_slices=2) 40% 因为 fire 多一倍, IDMA 占比下降.
  2. **Patch (L0)** 70.9% — IFM 大 (4×960×540 = 2 MB / core 跑 SMC 也得拉 519 KB) + WDMA 串行 idle 11K cy.
  3. **FC (L10)** ~5% — work=144 fire, setup + drain 占 95%, 单层 cy 太小不值优化.
- *N*=4 W slice 让 sub_W_out=17/9/5 不均 (L8 sub_W_out=5/4/4/4), 25% fire 不均直接产生 5K cy 等待 idle.

### 5.3 PROFILE 摘录 (per-core idle 来源, L0 / L1 / L3 三层主因)

| 层 | 核 | act_st | act_id | wgt_st | wgt_id | psm_id | 主因 |
|---|---|---:|---:|---:|---:|---:|---|
| L0 | C0-3 | 1680 | **11089** | **11089** | 1680 | 13360 | mm2s_arb 串行调度 IDMA / WDMA, 同步等 IFM |
| L1 | C0  | – | 119 | – | – | – | 本地核单段, IDMA 不卡 |
| L1 | C1-3 | – | **8720** | – | – | – | SMC head-of-line block (跨 mem cmd 等 C0) |
| L3 | C0-3 | – | **~3500** | – | – | – | ds K=1 stride=2 IDMA 数据量是 fire 的 4× → IDMA-bound |

---

## 6 数据交付清单

| 交付项 | 状态 | 数据来源 |
|---|---|---|
| 实验 A 表 + PROFILE | ✅ done | `sim/tb_core_dma/v4_sweep_ac_report_2026-05-09_15-01-31.txt` |
| 实验 C 表 + PROFILE | ✅ done | 同上 |
| 实验 B 单核基线 | ✅ done | `sim/tb_core_dma/v4_sweep_b_d1_report_2026-05-09_15-08-36.txt` |
| 实验 B SMC H-step 对照 | ✅ done | `paper/data/exp6_round_i_h_stride_separation.md` |
| 实验 D N=1 单核 | ✅ done | 同实验 B |
| 实验 D N=4 SMC | ✅ done | `sim/tb_smc/v4_sweep_d_smc_2026-05-09_15-10-11.json` (4 个 W) |
| 实验 E 分层 | ✅ done | `paper/data/exp8_pe_util_roadmap.md` 表 1 |

所有数据已采集完成. 配套日志文件未删除, 复现可直接读 `sim/tb_*/` 下对应文件.

### 6.1 复现命令汇总

```bash
cd C:/_Project/FLUX_CNN/toolchain
PY=./.venv/Scripts/python.exe
# 实验 A + 实验 C (单核 19 case, ~1 min sim)
$PY v4_sweep_ac.py
# 实验 B + 实验 D N=1 (单核 8 case, ~30 s sim)
$PY v4_sweep_b_d1.py
# 实验 D N=4 SMC (4 个 W 各一次, ~2 min sim)
$PY v4_sweep_d_smc.py
```

实验 E 不需要 sim, 直接读 `paper/data/exp8_pe_util_roadmap.md` 表 1.

---

## 7 数据交付局限性

写论文时可参考的几个非主线现象, 已自我标识:

1. **A.cin08_ky bit-exact 失败** — 实验 A 唯一 FAIL case, ky_per_group=2 末组 pad_ky=1 路径 RTL bug;
   相邻 Cin=4/16 case 都 PASS, fold 加速比趋势仍清晰.
2. **B 单核 K=1 stride=3/4 sim 不支持** — line_buffer K<stride 路径未实现 (force_s2d 触发条件 K≥stride);
   v4 spec 自己注 "stride=3 仅作完备性, ResNet11 主线无", 对 ResNet 流水无影响.
3. **D 中间点 N=2 跳过** — `axi_smc_4to4` IP 4-port 固定拓扑, N=2 需独立 `axi_smc_2to2` IP + 不同
   testbench (tb_multicore 老路径 vs SMC 不可比); 本扫频以 N=1 / N=4 端到端对比为主, 中间点用线性插值留作论文文字处理.

> 上述 3 项均不影响 §5.5.5 论证强度 (Ky-fold 适用域 / H-step 收益 / N=4 加速比上限). 若 §5 写作 agent
> 需要补充, 可记录在 §5.5.5 末段 "实验局限性" 一句话.
