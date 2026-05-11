# 实验 1: bias_rf 理想化 — 量化 bias prefetch idle 占比

**日期**: 2026-05-08
**Baseline (Round H)**: ResNet11 N=4 SMC = 204240 cy

## 方法

把 `RTL/bias_rf.sv` 的 `bias_ready` 信号改成永远 = 1（`+define+IDEAL_BIAS_RF`），
绕过 cs 切换时的 4 拍 SRAM read prefetch stall。bias_vec 输出可能错误，但 ResNet11
chain test 的 bias 全是 0（chain.cases 默认不设 bias），所以 OFM check 仍 PASS，
cy 数据有效。

## 结果

| 配置 | ResNet11 cy | Δ vs baseline |
|---|---:|---:|
| Baseline (real bias_rf) | 204240 | 0 |
| Ideal (bias_ready 永远 = 1) | **204108** | **-132 cy (-0.06%)** |

## 发现 (key insight)

**bias_rf cs 切换 stall 不是主要瓶颈**。仅占总 cy 的 **0.06%**。

之前 §12 把 bias_rf ping-pong 列为优化候选 D（预期 5-8% 收益），现在量化数据证明
预期严重高估。Round D 尝试 ping-pong 失败回退是对的（即使成功最多省 132 cy）。

## 对论文的启示

- `psum_idle` 在很多层占总 cy 的 20-30%（L0 13356/46015 = 29%, L3 16310/18369 = 89%），
  之前推测主因是 bias prefetch stall。**实际数据否定这个推测**。
- `psum_idle` 主因实际是 **mac_array 流水填充 + drain 同步**（K+几拍空泡 + parf_accum
  drain 跟 ofb_writer 异步），跟 bias_rf 无关。
- 优化 mac_array 流水深化才能减 psum_idle，bias_rf 改造不值得。

## 控制变量法的价值

不做这个实验，会把 bias_rf 改造列为高 ROI 优化项（基于对 psum_idle 来源的错误推测）。
实验数据揭示：**psum_idle ≠ bias prefetch idle**，两者来源不同。

## 数据来源

- baseline: commit `4c4d057` Round H step 2
- 实验代码: `RTL/bias_rf.sv` 加 `IDEAL_BIAS_RF` define 分支
- TB: `sim/tb_smc/run_chain.tcl` 加 `+define+IDEAL_BIAS_RF`
- 注: 实验代码已回退（不影响主代码）
