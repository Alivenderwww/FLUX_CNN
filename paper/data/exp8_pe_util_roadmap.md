# 实验 8: PE 利用率剖析 + 优化路线图

**日期**: 2026-05-08
**Baseline**: Round I L9 fix commit `e8ca7b0`, ResNet11 N=4 SMC = 190977 cy
**任务**: 拆 idle 来源, 量化理论最小 cy, 锁优先级最高的优化方向

## 1. 理论最小 cy 推导

PE array 16×16, 4 ConvCore W slice 并行. 整网串行 (layer i 的 OFM 给 i+1).

理论最小 cy = Σ (各 layer 4-core 中 max core 的 mac_array fire 数). 对应 PE 100% util,
不含 IDMA stall / 跨层 transition / FC drain 等 overhead.

### 每层 fire / cy / util (Round I L9 fix baseline)

| L | name | C_in→C_out | sub_W_out (max core) | fire (max) | cy | idle | util |
|---|---|---|---:|---:|---:|---:|---:|
| L0 | Patch K=4s=4 (s2d cs=4) | 4→16 | 34 | 32640 | 46019 | 13379 | **70.9%** |
| L1 | K=3 s=2 | 16→16 | 17 | 18360 | 27709 | 9349 | **66.3%** |
| L2 | K=3 s=1 | 16→16 | 17 | 18360 | 19800 | 1440 | 92.7% ✓ |
| L3 | ds K=1 s=2 | 16→16 | 17 | 2040 | 10921 | 8881 | **18.7%** ❌ |
| L4 | K=3 s=2 | 16→32 | 9 | 9720 | 11305 | 1585 | 86.0% ✓ |
| L5 | K=3 s=1 | 32→32 | 9 | 19440 | 22176 | 2736 | 87.7% ✓ |
| L6 | ds K=1 s=2 | 16→32 | 9 | 1080 | 6726 | 5646 | **16.1%** ❌ |
| L7 | K=3 s=2 | 32→64 | 5 | 10800 | 12539 | 1739 | 86.1% ✓ |
| L8 | K=3 s=1 | 64→64 | 5 | 21600 | 27364 | 5764 | **78.9%** |
| L9 | ds K=1 s=2 | 32→64 | 5 | 1200 | 2972 | 1772 | 40.4% |
| L10 | FC (cout slice) | 256→522 | 1 | ~144 | 2927 | ~2783 | ~5% |
| **Total** | | | | **~135384** | **190977** | **55593** | **70.9%** |

### 整网指标对比

| 指标 | 当前 | 理论上限 |
|---|---:|---:|
| Cycles | 190977 | 135384 |
| @100 MHz Latency | 1.91 ms | 1.35 ms |
| FPS | 524 | 740 |
| 整网 utilization | 70.9% | 100% |

剩余优化空间 41% cy / 216 FPS / 29% util gap.

## 2. Idle 来源拆解 (按 sim profile)

sim 输出每层每核的 stall 类型计数:
- `act_id` = mac_array 等 IFM 数据 (line_buffer 没数据 issue)
- `wgt_id` = mac_array 等 weight (wgt_buffer 没数据)
- `psm_id` = mac_array 等 psum (parf_accum 反压)
- `acc_id` = parf_accum 等 mac fire (drain)

### 优先级 A — L0 Patch (idle 13379 cy, 29%)

```
C0-3 L0: cy=46019 fire=32640 util=70.9%
         act_st=1680 act_id=11089 wgt_st=11089 wgt_id=1680 psm_id=13360
```

`act_id=wgt_id=11089` 同步 → mm2s_arb 串行调度 IDMA 跟 WDMA. L0 IFM 大 (519400B/core)
但 weight 才 1024B. mm2s_arb 仲裁让 IDMA 不让出, WDMA 等. mac_array 拉 weight 时 stall.

**优化**: mm2s_arb 让 IDMA / WDMA 真并行 (现共享 axi_dm.MM2S 通道串行). 1-2 天投入,
预期 ~3-5% 整网.

### 优先级 B — L1 K=3 s=2 第一个 conv (idle 9349 cy, 34%)

```
C0 L1: cy=27709 fire=18360 act_id=119 (几乎贴满)
C1-3 L1: cy=27709 fire=18360 act_id=8720 (大量 stall)
```

**C0 跟 C1-3 不对称** 是 SMC head-of-line block 的特征. 4 核同时 IDMA 拉跨 mem
数据时, SMC IP 仲裁让 C0 先吃饱, C1-3 等. 8720 cy / 27709 cy = 31% L1 cy 浪费.

**假设**: cmd 排序让"跨 mem 远端 cmd 先发, 本地 cmd 后发", 让 4 核同时启动远端访问,
减 head-of-line block.

**优化**: driver mesh_cmd 改 cmd 顺序 (mem 段顺序). 0.5-1 天投入, 预期 ~2-3% 整网.

### 优先级 C — L8 K=3 (idle 5764 cy, 21%)

```
C0 L8: fire=21600 (sub_W_out=5)
C1-3 L8: fire=17280 (sub_W_out=4)
```

C0 多算 25% (sub_W_out 5/4/4/4 不均, 17 W_out 切 4 核). cout=64 让不均放大.

**优化**: 重新分 sub_W_out (e.g. 5/4/4/4 → 试 4/4/4/5 或 cout slice). 1-2 天,
预期 ~2%. 但 cout slice 换 IFM 跨核压力, 净收益不一定.

### 优先级 D — ds layer L3/L6/L9 (idle 16299 cy)

Round I H 压缩已实施, util 18%/16%/40%. 软件 W 压缩失败 (Round J +37%, paper/data/exp7).
需硬件 2D 寻址 mem core 解码寻址 pattern. 1-2 周投入, ~2-3% 整网.

**当前论文阶段不投** (架构稳定性优先), 留作 future work.

### 优先级 E — L10 FC (idle 2783 cy)

work=144 fire 本身小, setup + drain 占 95%. 不值优化.

## 3. 优化路线图

| 优先级 | 优化点 | 投入 | 预期 | ROI |
|---|---|---|---|---|
| 🟢 B | L1 cmd 排序 (远端先发) | 0.5-1 天 | -2-3% | ★★★ |
| 🟢 A | L0 IDMA/WDMA 真并行 | 1-2 天 | -3-5% | ★★★ |
| 🟢 C | L8 W slice 均匀化 | 1-2 天 | -2% | ★★ |
| 🟡 D | ds 硬件 2D 寻址 | 1-2 周 | -2-3% | ★ (论文外) |
| 🔴 E | L10 FC | – | <1% | – |

A+B+C 全做后预估: 190977 → ~175000 cy = 1.75ms / 571 FPS, util 70.9% → ~77%.

本实验先攻 **优先级 B** (改动最小, 收益清晰, 验证 SMC head-of-line 假设).

---

## 4. Round K 实施: L1 cmd 排序 (本地 mem 段优先)

### 改动思路

每核 IDMA cmd 行内段顺序: 原 (col 顺序) **远端 halo → 本地 main**, 改成
**本地 main → 远端 halo**. sram_offset 仍按 col 顺序 (不改 IFB layout, dispatcher
按 cmds_per_row 计完成数, 跟 issue 顺序无关 — 见 idma_sg_dispatcher.sv:300).

**意图**: 4 核同时 hammer 4 不同 mem (并行无竞争), 远端 halo cmd 后发, 让 SMC
mem 0 slave 端不被 C0 + C1 同时抢 (head-of-line 假设).

### 实施

```python
# toolchain/mesh_cmd.py: gen_idma_sg_cmd_list_w_slice 加 local_seg_first 参数
# 行内 cmd 按 (seg_id == target_core_id) 排序: 本地段优先, 远端段排后
if local_seg_first and len(row_cmds) > 1:
    row_cmds.sort(key=lambda x: 0 if x[0] == target_core_id else 1)
```

```python
# toolchain/run_multicore_chain.py: FLUX_LOCAL_FIRST env 控制 (default=1 ON)
local_seg_first = os.environ.get('FLUX_LOCAL_FIRST', '1') == '1'
```

RTL 0 行, 纯 driver 改动 ~10 行.

### 实测 (commit `<待 commit>`)

#### Per-layer cy 对比 (Round I baseline → Round K)

| L | 描述 | Round I cy | Round K cy | Δ |
|---|---|---:|---:|---:|
| L0 | Patch (本地核, 单段) | 46019 | 46019 | 0 |
| **L1** | K3 s2 (4核跨 mem) | 27709 | **26866** | **-843** ✓ |
| L2 | K3 s1 (无 halo) | 19800 | 19817 | +17 (噪音) |
| L3 | ds K1 s2 | 10921 | 10921 | 0 |
| L4 | K3 s2 | 11305 | 11305 | 0 |
| L5 | K3 s1 | 22176 | 22160 | -16 |
| L6 | ds K1 s2 | 6726 | 6726 | 0 |
| L7 | K3 s2 | 12539 | 12537 | -2 |
| L8 | K3 s1 | 27364 | 27364 | 0 |
| L9 | ds K1 s2 | 2972 | 2972 | 0 |
| L10 | FC | 2927 | 2927 | 0 |
| **Total** | | **190977** | **190133** | **-844 (-0.44%)** |

L2 +17 噪音可忽略 (per-cell setup 浮动). 实质收益集中在 L1.

#### L1 各核 act_id 拆解

`act_id` = mac_array fire 时 line_buffer 没数据的等待 cycle.

| Core | data path | Round I act_id | Round K act_id | Δ |
|---|---|---:|---:|---:|
| C0 | mem 0 (本地, 单段) | 119 | 119 | 0 |
| C1 | mem 0 halo + mem 1 main | 8720 | **7799** | **-921 (-10.6%)** |
| C2 | mem 1 halo + mem 2 main | 8720 | 7799 | -921 (-10.6%) |
| C3 | mem 2 halo + mem 3 main | 8778 | 7935 | -843 (-9.6%) |

C1-3 act_id 一致减 ~10%. C0 没变 (单段无重排).

#### IDMA dispatcher state 拆解 (整 chain 累计 cy)

| Core | fetch | data | sum | Δ sum |
|---|---:|---:|---:|---:|
| C0 | 28662 → 28232 (-430) | 77717 → 77660 (-57) | 138959 → 139101 | +142 |
| C1 | 36383 → 36539 (+156) | 86463 → 85733 (-730) | 149059 → 148033 | -1026 |
| C2 | 36111 → 35703 (-408) | 86252 → 85662 (-590) | 144734 → 143791 | -943 |
| C3 | 30748 → 29699 (-1049) | 80674 → 78703 (-1971) | 139608 → 138578 | -1030 |

C1-3 dispatcher sum 减 ~1K cy/核, data 减为主 (channel utilization 提升).
跟 L1 cy -843 同数量级.

### 总结

✅ **head-of-line 假设方向对** — C1-3 act_id 减 10%, dispatcher data 减 1K cy/核.

⚠️ **收益比预期小 (-0.44% vs 预期 -2-3%)** — L1 idle 9349 cy 中只压了 843 cy.
剩 89% L1 idle 不是 head-of-line 引起, 而是:

1. **axi_dm IP cmd setup latency**: per-cmd 10-15 cy descriptor fetch + cmd parse +
   AXI burst issue. L1 跨 mem 480 cmd × ~10 cy = 4800 cy 是 IP 内部固定开销.
2. **SMC IP arbitration overhead**: 即使 4 核同时 hammer 4 mem, SMC IP 内部
   axi_crossbar arbiter 不是零延迟 (paper/data/exp2_ideal_smc.md 实测 -7%).
3. **AR/R channel deep pipeline**: axi_dm 单 cmd 在 R channel 占 burst_len + 几 cy
   turnaround, 多 cmd 不能 in-flight overlap.

**剩余优化路径** (按 ROI):

| 优先级 | 优化点 | 预期 | 投入 |
|---|---|---|---|
| A | L0 mm2s_arb IDMA/WDMA 真并行 | -3-5% | 1-2 天 (RTL) |
| C | L8 W slice 均匀化 (sub_W 5/4/4/4 → 4/4/4/5) | -2% | 1 天 (driver) |
| – | L1 axi_dm cmd batch / 多 cmd in-flight | -? | 2-3 天 (RTL) |
| D | ds 硬件 2D 寻址 | -2-3% | 1-2 周 (RTL+IP) |

### 论文意义

Round K 是控制变量法验证: head-of-line 假设的方向对 (act_id 减 10%), 但收益小.
反过来说明 L1 主要瓶颈不在 SMC 仲裁, 而在 axi_dm IP cmd setup latency. 这类
**"假设方向对但量级小"的实证** 帮论文区分 SMC 仲裁开销 vs axi_dm 单 cmd 开销.

跟 Round J 失败实验 (axi_dm cmd 颗粒度反噬) 形成互补:
- Round J: axi_dm cmd 颗粒度太小直接灾难性退化 (+37%)
- Round K: 软件 cmd 顺序优化只能拿到 axi_dm overhead 的"边际"部分 (-0.44%)

→ axi_dm IP 是性能天花板, 软件层 trade-off 上限有限.

### 数据来源

- Baseline Round I L9 fix: commit `e8ca7b0` (190977 cy)
- Round K commit: `6ba37ff` (driver only, RTL 0 行, FLUX_LOCAL_FIRST=1 默认 ON)
- 实验跑 sim: `python toolchain/run_multicore_chain.py --smc --demo resnet11 --case_name smc_resnet11 --n_cores 4`
  接 `cd sim/tb_smc && vsim -c -do run.tcl` (case 由 active_case.txt 自动确定)
- 还原 baseline: `FLUX_LOCAL_FIRST=0 python ...`

---

## 5. Round L 探针: L8 cout slice 可行性分析 (放弃)

### 动机

L8 (K=3 c64→64 H=30 W=17) W slice 不均: sub_W_out=5/4/4/4 让 C0 fire=21600 比
C1-3 fire=17280 多 25%. L8 cy=27364, 其中 C0 fully busy, C1-3 等 C0 idle 5764 cy.

直觉: 切 cout (64/4=16 cout/core, 各核全 W) 让 fire 完美均衡, 期望省 5K cy.

### 收益估算

| 维度 | W slice (当前) | cout slice (假设) |
|---|---|---|
| 各核 IFM 数据量 | sub_W × H × cin = ~9600 byte | W × H × cin = 32640 byte (×3.4) |
| 各核 fire | C0=21600, C1-3=17280 (25% 不均) | 17280 (完美均衡) |
| 各核 idle | 5764 (C0 没 idle, 其他等 C0) | ~4720 (假设) |
| 各核 L8 cy | 27364 | ~22000 (估) |
| **整网收益** | – | **~-2.6%** |

### 实施成本

driver 当前限制 (run_multicore_chain.py:712):
```
NotImplementedError: SMC cout slice layer L3.B1.C2: prev layer 7 必须 mode A
集中存放或 layer 是 root, 当前 W slice → cout slice 不支持
```

cout slice 当前只支持:
- 上层是 **mode A** (单核独占, IFM 集中存放)
- 或 layer 是 chain root (IFM 在 SMC_INPUT_BASE)
- L10 FC 命中条件 (L9 输出 stitch 后再 cout slice OK)

L7→L8→L9 双向 transition 需 driver 改造:
1. **L7 (W slice OFM 散布 4 mem) → L8 (cout slice 拉整图 IFM)**
   - cout slice IDMA cmd 生成器需支持 multi-seg base (复用 gen_idma_sg_cmd_list_w_slice
     传整图 W + 4 mem segments, ~30 行)
2. **L8 (cout slice OFM 各核 cout 段) → L9 (W slice IFM 4 mem W 段)**
   - 这是 stitch 难点: L8 各核写到自己 mem 一个 cout 段全 W, L9 各核需读 4 mem 拼整 cout
   - 当前 cout slice OFM 写 = 自核 mem 紧凑, L9 读跨 4 mem 但每 mem 是不同 cout 段 → 各核 IDMA cmd 生成需懂"跨 mem cout 段拼接 cin slice"
   - 实施: ~1 周 (mesh_cmd + driver layer transition)

总投入估 **1-2 周** driver 工作.

### ROI 评估

| 收益 | 成本 | 净 |
|---|---|---|
| -2.6% 整网 (-5K cy, 1.90→1.85 ms) | 1-2 周 driver | **不划算** |

跟其他选项对比:
- Round K (head-of-line): 0.5 天投入, -0.44%, 已做
- A (mm2s_arb 真并行): 1-2 天, -3-5%, RTL 改动
- 硬件 2D 寻址 ds layer: 1-2 周, -2-3%, 协议级别改造

cout slice for L8 投入跟硬件 2D 寻址同, 收益略高 (2.6 vs 2-3%) 但风险:
- IFM 数据量 ×3.4, SMC IP 可能因 4 master 同 hammer 4 mem 竞争吞吐塌
- 实测 cy 可能远低于理论估值

### 决定: 放弃实施, 留 hook 给 future work

加 driver hook (供后续实施时直接用):
- `scheduler.set_force_cout_layer_names(names)` — 强制特定 layer 走 cout slice
- `FLUX_FORCE_COUT_LAYERS="8"` env 控制
- 当前调用会触发 NotImplementedError (driver layer transition 未实现)

### 论文意义

Round L 是 **机会成本评估** 实证: 看似自然的优化 (cout slice 替 W slice 解 imbalance)
实际成本 > 收益. 未盲目实施, 通过收益模型 + 实施成本拆解提前决定不投.

跟 Round J 失败/Round K 边际收益形成完整链条:
- Round J: 软件 W 压缩 灾难性退化 (+37%) — axi_dm cmd 颗粒度天花板
- Round K: 软件 cmd 排序 边际收益 (-0.44%) — SMC 仲裁不是主瓶颈
- **Round L: cout slice 替代评估 ROI 不划算 — driver layer transition 复杂度天花板**

→ 软件层优化路径基本枚举完, 剩余空间需硬件改动 (mm2s_arb / ds 2D 寻址).

### 数据来源

- Driver hook commit: `<待 commit>` (FLUX_FORCE_COUT_LAYERS env, 当前会 raise NotImplementedError)
- 收益估算基于 sim 实测 fire 数 (Round I L9 fix baseline)



