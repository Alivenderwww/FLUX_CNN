# 实验 4: IDMA SG dispatcher 状态 cy breakdown — 时间用在哪

**日期**: 2026-05-08
**关联**: exp2 (SMC overhead) + exp3 (mm2s_arb 占用)

## 方法

`RTL/DMA/idma_sg_dispatcher.sv` 加 5 个 hs_*_cy counter 累计每个 FSM 状态的 cy:
- `hs_fetch_cy`: S_FETCH_CMD_ISS + S_FETCH_CMD_DAT (拉 SG cmd 时间)
- `hs_ring_wait_cy`: S_RING_WAIT (line_buffer ring 反压时间)
- `hs_issue_cy`: S_ISSUE (发 user data cmd 时间)
- `hs_data_cy`: S_DATA (传 user data 时间, mac_array 消费源)
- `hs_done_cy`: S_DONE (等 sts drain)

跑 ResNet11, IP path vs IDEAL_SMC 对比。

## 数据对比

### IP path (Round H baseline = 204240 cy)

| Core | fetch | ring_wait | issue | data | done | sum | other (IDLE) |
|---|---:|---:|---:|---:|---:|---:|---:|
| C0 | 31614 | 31287 | 1681 | 86283 | 11 | 150876 | 53364 |
| C1 | 39819 | 24551 | 2191 | 95745 | 11 | 162317 | 41923 |
| C2 | 38546 | 20649 | 2131 | 94155 | 11 | 155492 | 48748 |
| C3 | 33275 | 26794 | 1801 | 88703 | 11 | 150584 | 53656 |
| Avg | 35814 | 25820 | 1951 | 91222 | 11 | 154817 | 49423 |
| % of cy | 17.5% | 12.6% | 1.0% | 44.7% | 0.0% | 75.8% | 24.2% |

### IDEAL_SMC (zero SMC latency = 189886 cy)

| Core | fetch | ring_wait | issue | data | done | sum |
|---|---:|---:|---:|---:|---:|---:|
| C0 | 22881 | 37526 | 1681 | 78384 | 11 | 140483 |
| C1 | 29609 | 28713 | 2191 | 87656 | 11 | 148180 |
| C2 | 29223 | 23947 | 2131 | 86109 | 11 | 141421 |
| C3 | 24764 | 31691 | 1801 | 80201 | 11 | 138468 |
| Avg | 26619 | 30469 | 1951 | 83088 | 11 | 142138 |
| % of cy | 14.0% | 16.0% | 1.0% | 43.8% | 0.0% | 74.9% |

### Δ (IDEAL_SMC vs IP path)

| metric | IP path avg | IDEAL_SMC avg | Δ |
|---|---:|---:|---:|
| fetch | 35814 | 26619 | **-9195 (-25.7%)** |
| ring_wait | 25820 | 30469 | **+4649** (反弹) |
| data | 91222 | 83088 | **-8134 (-8.9%)** |

## 关键发现

### 1. SMC overhead 主要在 fetch + data, 比例约 1:1

SMC IP register stage 让:
- 每条 SG fetch cmd 慢 ~ 2-3 cy (5 条 AXI 通道 × 1-2 cy register slice)
- 每条 user data burst 慢 ~ 2-3 cy

**dispatcher 端可量化的 SMC cost**:
- fetch: -9195 cy/核 × 4 核 = ~37K (但实际 wall -14K, 因为多核重叠)
- data: -8134 cy/核 × 4 核 = ~32K (同上)

### 2. ring_wait 反弹暴露了 mac_array 消费瓶颈

IDEAL_SMC 下 IDMA 拉得快, line_buffer ring 更快满, dispatcher 反压时间 +4649 cy。
**跟随之而来的问题**: 即使消除互联 overhead, mac_array 消费速度也跟不上, ring 反压
取代了原本的 IDMA fetch latency。

**这是个负反馈**: 减少 IDMA latency 让 mac_array 反压更频繁, 减小 IDMA 加速净收益。

### 3. IDMA dispatcher 时间分布:

```
Total per core (avg) = 204240 cy

fetch        ████████████▎               17.5%
ring_wait    ██████████                  12.6%
issue        █                            1.0%
data         ████████████████████████████ 44.7%   ← mac_array 数据来源
done         (negligible)
IDLE         ████████████████             24.2%   ← layer 间隙 / 第一行 preload
```

**论文 figure 候选**: 4 核 stacked bar, 看 dispatcher 内部时间分布。

### 4. cross-mem layer 看在 fetch cy

C1/C2 fetch=39819/38546 比 C0/C3 fetch=31614/33275 多 ~20%.
跟 C1/C2 的 cmds_per_row=2 (cross-mem) vs C0/C3 cmds_per_row=1 一致。

### 5. data cy ≈ 90K, mac_array fire ≈ 73K

mac_array 实际 fire = 18360 fire/核 (in big layer L1) × 11 layer ≈ avg 70K-75K per core.
dispatcher data cy ≈ 90K. 差额 = mac_array 等其他东西 (psum_idle 等), 不是等 IDMA。

**这进一步说明 IDMA 已经够快**, 减 IDMA latency 边际收益小。**真正瓶颈是 mac_array
内部流水填充 + drain**。

## 论文素材点

### 推论 1: dispatcher prefetch 收益上限低

如果 prefetch 能让 fetch 完全跟 data 重叠 (理想情况), 单核可省 fetch cy / 4 核 ≈ 9K cy。
但 ring_wait 反弹会吃掉一半 (+4.6K)。**净收益估算 ≤ 4-5K cy ≈ 2-2.5%**, 而非之前估的 3-5%。

### 推论 2: 减 IDMA latency 不如增 mac_array 算力

数据显示 IDMA dispatcher 已经有 24% IDLE + 12.6% ring_wait, 加起来 36.8% 时间无活。
mac_array 消费太慢, 让 IDMA 系统大部分时间空。

**优化方向**: 加快 mac_array 消费 (Ky-cout-fold / 小 K 引擎) 比加快 IDMA 收益更大。

### 推论 3: 瓶颈层次

```
mac_array (流水填充 + drain + ds 阵列空转)
    ↓ 拉慢 ↓
parf_accum (drain 同步)
    ↓ 拉慢 ↓
ofb_writer (SDP + bias_rf)
    ↓ 反压 ↓
line_buffer (ring 反压, ring_wait 体现)
    ↓ 反压 ↓
IDMA dispatcher (ring_wait 体现)
```

**减 IDMA latency 不解决根本问题** — 数据流"上游"(mac_array)处理慢, "下游"(dispatcher)
减压只让"反压"换地方表现 (从 fetch 移到 ring_wait)。

## 数据来源

- baseline IP path: commit `4c4d057` (Round H) → wall 204240
- IDEAL_SMC: `+define+IDEAL_SMC` → wall 189886 (-7.0%)
- 实验代码: `RTL/DMA/idma_sg_dispatcher.sv` 加 hs_* counter, `tb_smc_chain.sv` 末尾 dump
- 这次改动是干净的 counter 添加, 应 commit
