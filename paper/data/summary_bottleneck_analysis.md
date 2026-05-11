# ResNet11 N=4 SMC 性能瓶颈综合分析

**日期**: 2026-05-08
**baseline**: Round H IP path = 204240 cy (vs IP baseline 217311, 累计 -6.0%)
**PE util**: 62.85% (理论上限 100%, 还差 37.15%)

## 实验方法

控制变量法: 替换 RTL 模块为理想模型, 量化各因素对总 cy 的贡献.

5 组实验在 `paper/data/exp{1..5}_*.md`:
1. bias_rf 永远 ready: 量化 bias prefetch idle 占比
2. axi_smc IP → 0-latency sim model: 量化 SMC 互联 overhead
   2b. 每层 SMC overhead breakdown
3. mm2s_arb 占用率 counter: 量化 4 master 竞争
4. dispatcher state cy breakdown: 量化 fetch/data/ring_wait 时间分布
5. ds layer 根因: 数学证明 memory-bound 不是 compute-bound

## 关键发现 (按重要性)

### 1. SMC 互联 IP 占 7.0% 总 cy (最大单一可量化瓶颈)

`axi_smc_4to4` IP register stage latency 累计 14354 cy / 204240 = 7.0%。
- IDMA-bound layer (L0/L1/L3/L6/L9): 12-13% overhead
- compute-bound layer (L2/L5/L7/L8): 0.5-1.3% overhead

**论文 figure**: SMC overhead vs PE util 散点图, 双峰分布。

### 2. WDMA 从不饿死, mm2s_arb 仲裁不是瓶颈

Round E 加的 WDMA starve 防御机制 `starve_preempt = 0` 整网 0 触发。
当前 dispatcher 串行 (Round B), IDMA 在 fetch latency 间隙释放 mm2s, WDMA 拉得到。

**论文论证**: dispatcher prefetch + mm2s 仲裁是耦合的, 单做仲裁优化收益 0。

### 3. ds layer 是 memory-bound, 不是 compute-bound

ds layer (K=1 s=2) PE util 11% 但 PE 阵列 100% 用. 慢是因为 fire 数 9× 少 (K²=1 vs 9)
但数据传输量跟 K=3 layer 类似。

**论文创新点候选**: 算法层 K=1 ds + K=3 conv 合并 (math equiv), 整网 -16%。

### 4. dispatcher 内 fetch (17.5%) 跟 ring_wait (12.6%) 互为 trade-off

减 SMC IP latency (IDEAL_SMC) 让 fetch -25.7%, 但 ring_wait +18% 反弹。
mac_array 消费速度跟不上 IDMA push, 反压换地方表现。

**论证**: 减 IDMA latency 净收益 ≤ 2-3%, 真正瓶颈在 mac_array 内部。

### 5. bias_rf cs 切换 stall 不重要 (0.06% 总 cy)

之前推测 psm_idle 主因是 bias prefetch, 实验否定。psm_idle 来自 mac_array 流水填充
+ drain 同步, 跟 bias_rf 无关。

**论文教训**: 推测的瓶颈来源不一定对, 控制变量是关键。

## ResNet11 性能 Decomposition (每层 cy 占比)

baseline 204240 cy = sum:

| 类别 | layer | cy | 占比 | 瓶颈类型 |
|---|---|---:|---:|---|
| Patch | L0 | 46015 | 22.5% | IDMA preload + drain |
| K=3 主路径 | L1+L2 | 47165 | 23.1% | balanced (L2 92.7% util) |
| **ds (K=1)** | L3+L6+L9 | 34241 | **16.8%** | **memory-bound** |
| K=3 主路径 | L4+L5 | 33477 | 16.4% | balanced |
| K=3 主路径 | L7+L8 | 39899 | 19.5% | balanced |
| FC | L10 | 2926 | 1.4% | cout slice 多核 |

ds layer 占 16.8%, 是最大优化空间。

## 优化潜力排序 (基于实验数据)

| 优化方向 | 预期收益 | 工程量 | 论文价值 | 状态 |
|---|---:|---|---|---|
| Round A-H 已做 | -6.0% | 1 周 | 中 | ✅ 完成 |
| **SMC 互联简化** | -7.0% | 1-2 周 | **高** | 🟡 论文 |
| **ds IFB 复用** (skip_idma) | -7-10% | 2-3 周 | **高** | 🟡 论文 |
| **K=1 算法层合并** | **-16%** | 2-4 周 | **极高** | 🟡 论文 |
| mac_array K=3 流水深化 | -2-3% | 1 周 | 中 | 🟡 后续 |
| ~~bias_rf ping-pong~~ | -0.06% | 失败 | 低 | ❌ 放弃 |
| ~~dispatcher prefetch~~ | ≤-2% | 失败 | 低 | ❌ 放弃 |

## 论文章节素材建议

### 章节 1: 性能瓶颈分类

> ResNet11 N=4 SMC 性能瓶颈可分类成 4 大类:
> 1. **互联 overhead** (7%): SMC IP register stage
> 2. **memory-bound layer** (17% ds layer): 数据传输 dominant
> 3. **compute-bound layer** (60+% K=3 主路径): mac_array 已 70-90% util
> 4. **layer 间 host overhead** (10%): boot regs 串行 + dfe 拉 desc

### 章节 2: 控制变量法

> 通过替换 RTL 模块为理想模型 (axi_crossbar_4to4_sim 替代 axi_smc IP 等), 量化
> 各因素贡献。这种方法揭示了 §12 的早期推测错误 (bias_rf 跟 mac_array 阵列空转
> 都被否定)。

### 章节 3: ds layer 根因 (创新)

> ResNet residual ds layer (K=1 s=2) PE util 11%, 之前文献归因为"K=1 时 PE 阵列
> 利用率天花板 1/9"。本工作证明该归因错误: K=1 时 mac_array PE 阵列 100% 用 (fire
> 满 256 MAC), 慢的根因是 fire 数少但数据量不变 → memory-bound。

### 章节 4: 优化方向 (论文创新)

> 基于上述瓶颈分析, 提出 3 个优化方向:
> A. 互联简化 (替换 SmartConnect 为自写 crossbar): -7%
> B. ds layer IFB 复用 (residual path 跟主路径共享 input): -10%
> C. **算法层 K=1 合并** (K=1 ds + K=3 conv → 单 K=3 with bias)**: -16%**

## 数据来源

- 实验代码 (RTL counter / ifdef): commit `1e41af0`
- 5 个实验报告: paper/data/exp{1..5}*.md
- baseline: commit `4c4d057` (Round H step 2)
- PE profile dump: tb_smc_chain.sv 的 dump_pe_profile task
- mm2s_arb / dispatcher counter: tb_smc_chain.sv 末层 dump
