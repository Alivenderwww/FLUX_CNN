# 实验 2b: SMC IP overhead 按层 breakdown

**日期**: 2026-05-08
**关联实验**: exp2_ideal_smc.md

## 数据

ResNet11 N=4 SMC 每层 cy: IP path vs IDEAL_SMC (sim model crossbar):

| L | 维度 / K / s | IP path | IDEAL_SMC | overhead | 占比 |
|---|---|---:|---:|---:|---:|
| L0 Patch | s2d K=1 cs=4 | 46015 | 40341 | **-5674** | **12.3%** |
| L1 | K=3 s=2 16→16 | 27367 | 24008 | **-3359** | **12.3%** |
| L2 | K=3 s=1 16→16 | 19798 | 19698 | -100 | 0.5% |
| L3 ds | K=1 s=2 | 18369 | 16555 | -1814 | 9.9% |
| L4 | K=3 s=2 16→32 | 11303 | 10419 | -884 | 7.8% |
| L5 | K=3 s=1 32→32 | 22174 | 21927 | -247 | 1.1% |
| L6 ds | K=1 s=2 | 11721 | 10278 | -1443 | 12.3% |
| L7 | K=3 s=2 32→64 | 12537 | 12372 | -165 | 1.3% |
| L8 | K=3 s=1 64→64 | 27362 | 27205 | -157 | 0.6% |
| L9 ds | K=1 s=2 | 4151 | 3638 | **-513** | **12.4%** |
| L10 FC | cout slice | 2926 | 2931 | +5 | -0.2% (noise) |
| **Total** | | **204240** | **189886** | **-14354** | **7.0%** |

## 关键发现

**SMC IP overhead 占比按层呈双峰分布**：

### 高 overhead 层 (12%): IDMA-bound layer

- L0 / L1 / L3 / L6 / L9: 12-13% overhead
- 共同特点: K=1 s=2 (ds layer) **或** patch s2d (cs=4 cross-mem)
- 这些 layer cycles 大头是 IDMA 拉数据 + cross-mem cmd 多, 每条 cmd 都吃 SMC IP 的 register stage latency

### 低 overhead 层 (~1%): Compute-bound layer

- L2 / L5 / L7 / L8: 0.5-1.3% overhead
- 共同特点: K=3 s=1, MAC 跑久 (fire 数大占总 cy 60-90%)
- IDMA prefetch 完成后 mac_array 持续算, IDMA 通道 idle, SMC IP register stage 不影响

### 中等 overhead 层 (~8%): 边缘场景

- L4 K=3 s=2: stride=2 让 IDMA 数据量减半, 但 mac 也少, IDMA 占比中等

## 论文素材点

### 1. ds layer 双重痛点

- 已知问题 (§12): ds layer PE 阵列空转 (K=1 cin=16 时只 16/256 PE 工作)
- **新发现**: ds layer 还吃 12% SMC overhead, 双重 cost

ds layer L3 cy=18369, fire=2040, util=11%. 其中:
- ~10% IP register stage (1814 cy)
- ~85% IDMA preload + drain (rebalance 后是 act_id + psum_id)

### 2. SMC overhead 跟 mac util 反相关

数据可视化 (论文图候选):

```
SMC overhead %  ┃
              14┤  L9●  L1●  L0●  L6●
              12┤
              10┤    L3●
               8┤      L4●
               6┤
               4┤
               2┤        L7●  L5●  L2●  L8●
               0┤___________________________________
                  10   30   50   70   90    PE util %
```

PE util 高的 layer (compute-bound) SMC overhead 自然小。
PE util 低的 layer (memory-bound) SMC overhead 大 → 双重痛点。

### 3. ResNet11 总体性能上限估算

| 优化场景 | 总 cy | 速度 | PE util |
|---|---:|---:|---:|
| 当前 IP path (Round H) | 204240 | 1.00× | 62.85% |
| + IDEAL_SMC (零互联) | 189886 | 1.08× | 67.60% |
| + IDEAL_BIAS_RF (零 bias stall) | 189754 | 1.08× | 67.65% |
| 假设 ds layer PE util 也到 70% | ~175K | ~1.17× | ~73% |
| 完美 mac_array (100% util on K=3 layer) | ~150K | ~1.36× | ~85% |

**当前优化 ROI 优先级 (基于 paper/data 实证)**:

1. **互联简化** (减 SMC overhead 7%): 改 IP config / 替换 SmartConnect / 自写 crossbar
2. **mac_array 改造** (ds layer PE util 11→70% 估省 ~7%): Ky-cout-fold 或小 K 引擎
3. ~~bias_rf ping-pong~~: 0.06% 收益, 放弃
4. ~~dispatcher prefetch~~: 跟 SMC overhead 重叠, 单独做收益小, 还要解决 mm2s 抢占问题
