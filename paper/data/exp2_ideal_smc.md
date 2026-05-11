# 实验 2: SMC IP 替换为 0-latency sim model — 量化 SMC 互联 overhead

**日期**: 2026-05-08
**Baseline (Round H)**: ResNet11 N=4 SMC = 204240 cy (real Vivado axi_smc_4to4 IP)

## 方法

把 `RTL/multicore_top_smc.sv` 加 `\`ifdef IDEAL_SMC` 切换:
- IP path: Vivado `axi_smc_4to4`（基于 axi_crossbar IP, 4 SI : 4 MI, 4 × 16 MB region）
  内部每 AXI 通道带 register slice (默认 `LATENCY_SLAVE_INPUT` etc 全 enable)
- IDEAL path: `axi_crossbar_4to4_sim.sv` (我自己写的 sim model, 纯组合 mux 路由 + FSM 仲裁)
  **0 register stage**, 跨 SI/MI 0 cycle latency

跑 `+define+IDEAL_SMC` 重新跑 sim, 对比 cy 数。

## 结果

| Case | IP path (Round H) | IDEAL_SMC | Δ | overhead 占比 |
|---|---:|---:|---:|---:|
| **smc_resnet11** | 204240 | **189886** | **-14354** | **7.0%** |
| smc_wslice1 | 4146 | 3579 | -567 | 13.7% |
| smc_wslice5 | 20718 | 17727 | -2991 | 14.4% |

## 关键发现

### 1. SMC 互联 IP 吃了 7% 总 cy

**这是当前 ResNet11 N=4 性能瓶颈中可量化的最大单一来源**。Vivado `axi_smc_4to4` IP
内部每条 AXI 通道（AW/AR/W/R/B 5 条）默认有 register slice，让 cmd / data 路径
来回都吃 1-2 cy register stage latency。

### 2. cy 占比跟 layer 大小成反比

- ResNet11 (大网, 有大量 burst): 7.0% overhead
- wslice5 (5 小 layer): 14.4%
- wslice1 (1 小 layer): 13.7%

小 case 占比更高因为 dispatcher 跟 axi_dm IP 的 fetch latency 占总 cy 大头，
SMC IP register stage 在每条 AXI handshake 都加 1-2 cy，比例放大。

### 3. PE util 翻译

- IP path: 131.5M / (4 × 204240 × 256) = **62.85%**
- IDEAL_SMC: 131.5M / (4 × 189886 × 256) = **67.60%** (+4.75%)

如果完全消除 SMC overhead, PE util 能到 67.6%。剩 32.4% idle 是 mac_array 内部
流水填充 + drain + ds 层 PE 阵列空转的结构性问题。

## 对论文的启示

### 章节素材

- **互联 overhead 是真实 cost**: 论文不能只算 dispatcher / mac_array, 必须加上互联 register stage 的 7% cost。
- **N 核 SMC 越大越省**: 4 核 SMC 互联开销 7%, 真硬件部署 8 核或更多 SMC 可能开销更大（因为 crossbar 复杂度 N²）。
- **优化方向**: 互联的 register slice 数量是 IP config 选项, 可调（牺牲 timing 减 latency）。但实测 register slice 减少会让 Fmax 下降, 是 tradeoff。

### 跟既有优化对比

| 优化项 | 累计减 cy | 占总 cy 比例 |
|---|---:|---:|
| Round C/E/F/G/H | ~13K cy | -6.0% |
| **SMC IP overhead (理论上限)** | **14354 cy** | **-7.0%** |
| 加起来 | ~27K cy | -13.0% |

如果能找到办法减少 SMC IP overhead, 可以再吃 7% 性能。

### Future Work 候选

1. **改 IP config**: 选 `LATENCY_SLAVE_INPUT=0` 等（牺牲 Fmax 减 latency）。但综合时序可能违反。
2. **替换互联**: 自写综合质量好的 crossbar (难, 或者用 SmartConnect 替代)。
3. **减少互联跳数**: 让 ConvCore 直接接 mem (P2P), 不经过 crossbar。但 4 核交叉访问就需要 crossbar。
4. **接受 overhead, 增 ConvCore 算力**: 让 mac_array 算得更快, 互联 overhead 占比相对变小。

## 数据来源

- baseline: commit `4c4d057` (Round H step 2)
- 实验代码: `RTL/multicore_top_smc.sv` 加 `IDEAL_SMC` ifdef + 恢复 `axi_crossbar_4to4_sim.sv`
- TB: `sim/tb_smc/run_chain.tcl` 加 `+define+IDEAL_SMC` + 编译 sim crossbar
- **注**: 实验代码暂时保留（用作未来其他实验对比 baseline）
