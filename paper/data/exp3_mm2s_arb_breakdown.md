# 实验 3: mm2s_arb 占用率 counter — 量化 mm2s 通道使用模式

**日期**: 2026-05-08
**Baseline**: Round H IP path = 204240 cy

## 方法

`RTL/DMA/mm2s_arb.sv` 加 5 个 hs_*_grant counter:
- `hs_idma_grant`: IDMA 拿到 mm2s cmd 通道总 cy
- `hs_wdma_grant`: WDMA 拿到
- `hs_rdma_grant`: RDMA 拿到
- `hs_ocmd_grant`: ODMA cmd fetch 拿到
- `hs_starve_preempt`: WDMA starve 触发抢占次数 (Round E 机制)
- `hs_data_full_stall`: data FIFO 满阻塞 cmd_fire

`tb_smc_chain.sv` 在最后一层结束时 dump 4 核 counter snapshot。

## 数据 (ResNet11 N=4 SMC, IP path)

| Core | idma | wdma | rdma | ocmd | **starve_preempt** | data_full_stall |
|---|---:|---:|---:|---:|---:|---:|
| C0 | 3362 | 11 | 11 | 871 | **0** | 0 |
| C1 | 4382 | 11 | 11 | 871 | **0** | 0 |
| C2 | 4262 | 11 | 11 | 871 | **0** | 0 |
| C3 | 3602 | 11 | 11 | 871 | **0** | 0 |

## 关键发现

### 1. WDMA 从不饿死 — Round E 0 次触发

`starve_preempt = 0` 说明 Round E 加的 WDMA 饥饿提优先级机制 **一次都没触发**。
当前 dispatcher 串行 fetch+exec, IDMA 在 fetch latency 间隙释放 mm2s 通道, WDMA
在间隙拉到 weight, 从不超过 32 cycle 等待门槛。

**论文价值**: Round E 改动是纯防御性 (0 当前收益, 但 prefetch 改造时是兜底)。
单独评估 mm2s_arb 改动收益时, 可量化为 0.

### 2. cmd_fire 总数远小于 cy

每核 cmd 总数 = idma + wdma + rdma + ocmd ≈ 4500-5400. 跟 cy=204240 的比 ≈ 2.6%.

**说明**: mm2s 通道大部分时间在传 user data 或 idle, 不是 cmd 在抢资源。
反过来想: 如果只 cmd 数据传输瓶颈, dispatcher prefetch 收益空间 ≤ 2.6%。

### 3. IDMA cmd 数核间差异 — 标识 cross-mem layer

- C0: 3362, C1: 4382, C2: 4262, C3: 3602
- C1/C2 (中间核) 比 C0/C3 (边界核) 多 ~700 cmd
- 中间核的 W slice 段恰好横跨 mem 边界, IDMA 一行需要 2 条 cross-mem cmd (cmds_per_row=2)
- 边界核段在单 mem 内, cmds_per_row=1

跟 §12 瓶颈 B (中间核 act_idle 多) 一致。**mm2s_arb counter 直接量化了 cross-mem cmd 多 ~30% 的事实**。

### 4. WDMA / RDMA 各 11 cmd 一一对应 layer

- WDMA 11 cmd = 11 layer 各 1 次整层 weight 拉取
- RDMA 11 cmd = 11 layer 各 1 次 bias + shortcut 拉取
- 跟 RTL 设计 (整层一次性 DMA preload) 一致

### 5. data_full_stall = 0

owner FIFO (depth 8) 没出现 cmd 阻塞 stall. axi_dm IP 内部 cmd FIFO 处理流畅。
说明 dispatcher 当前发 cmd 速率不足以填满 owner FIFO, 还有 prefetch 加速空间
(但收益受限, 见上 #2)。

## 论文素材

### Figure 候选 (mm2s 通道使用模式 stacked bar)

```
mm2s 通道占用 % (per core, ResNet11)

              idma (~3500-4500)  ocmd (~870)   wdma (11)   rdma (11)
              [████████████████]  [██]         [.]         [.]
              ≈ 80%               ≈ 17%         ≈ 0.2%      ≈ 0.2%
```

注意: 这是 cmd_fire 次数, 不是 cycle 占用. cycle 占用要乘以每条 cmd 的处理时间。

### 跟 Round C+ 失败的关联

Round C+ dispatcher prefetch 让 L0 退化 +17K cy。当时归因 WDMA 饿死 (memory/feedback_prefetch_starves_wdma.md)。
但 **本实验显示 starve_preempt=0**, WDMA 不会饿死。

修正归因: Round C+ L0 暴涨可能是 **axi_dm IP 内部 cmd FIFO 排队让 user data 返回延迟**, 不是 mm2s_arb 仲裁问题。
prefetch 让 fetch_cmd 排到 user_data cmd 前面, axi_dm 顺序处理, user data 反而要等 fetch data 出完。

**重新评估 dispatcher prefetch**: 
- WDMA 不饿死, mm2s_arb 不是问题
- 真正问题是 axi_dm IP cmd 顺序处理: 多 outstanding cmd 不会让单个 cmd 处理更快, 反而 user data 被 fetch data 挡住
- 解决方案需要 axi_dm IP 内部支持多通道并发, 或者 dispatcher 用独立 axi master 拉 cmd (不抢 mm2s 通道)

### Future Work

1. **DFE 模式拉 SG cmd** (跟 axi_dm MM2S 通道分开): DFE 已有 m[3] 独立 master, 可让 dispatcher 借用 (但 DFE 也在用)。
2. **加 m[2] 给 dispatcher SG fetch** (分配剩下未用的 master 端口): core_top.sv 的 m[2] 当前 unused, 可拿来给 dispatcher.

## 数据来源

- baseline: commit `4c4d057` (Round H step 2)
- 实验代码: `RTL/DMA/mm2s_arb.sv` 加 hs_*_grant counter, `sim/tb_smc/tb_smc_chain.sv` 加 dump
- 这次改动是干净的添加 counter, 不影响功能, 应该 commit
