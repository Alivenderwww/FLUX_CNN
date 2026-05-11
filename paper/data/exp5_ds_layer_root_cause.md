# 实验 5: ds layer (K=1 residual) PE util 11% 根因 — memory-bound 不是 compute-bound

**日期**: 2026-05-08
**关联**: §12 PE 利用率分析 (列 ds 层为瓶颈 C, 推测是阵列空转)

## 问题

ResNet11 中 3 个 ds layer (residual downsample, K=1 s=2):
- L3: cy=18369, util=**11.1%**
- L6: cy=11721, util=**9.2%**
- L9: cy=4151, util=**28.9%**

之前 (§12) 推测主因是 **mac_array 阵列空转** (K=1 cin=cout=16 时 PE 阵列只用 16/256)。
本实验数学分析 + 数据证伪此推测。

## 数学分析

ResNet11 L3 (Patch s2d 后): H_in=240 W_in=135 (W slice 后 sub_W=34) cin=16 cout=16
H_out=120 W_out=68 (per core sub_W_out=17)。

### 假设 1 (旧推测): mac_array 阵列空转

K=1 时 mac_array 一拍 fire = `K² × cin × cout = 1 × 16 × 16 = 256 MAC`。**PE 阵列 100%
利用率** (NUM_PE=NUM_COL=16, cin slice 满 PE row, cout slice 满 PE col)。

→ K=1 不导致 PE 阵列空转！这个推测是错的。

### 假设 2 (本实验): memory-bound

ds layer 一拍 fire 跑 256 MAC, 要求一拍消耗 16 cin × 1 cell 数据 = **16 byte 输入**。
但 IDMA 拉数据吞吐受 axi_dm + axi_smc IP 限制, 实际拉一行 (sub_W=17) 数据需要:

```
data_per_row = sub_W × cin_slice × 16 byte = 17 × 1 × 16 = 272 byte
data_cy_per_row = 272 / 16 byte/cy = 17 cy/row + handshake + cross-mem 开销 ≈ 30-50 cy
```

mac_array 跑一行 fire = sub_W_out = 17 cy (理想流水)。

```
Best case dispatcher data_cy / mac_array fire = 30 / 17 = 1.76× slower
```

实际 **dispatcher data 时间 + ring_wait + fetch 串行** 让总 cy 比 fire 多 9× (实测 18369 / 2040)。

## 实测数据验证

| Layer | cy | fire | util | data_cy (per dispatcher) | act_id |
|---|---:|---:|---:|---:|---:|
| L2 (K=3) | 19798 | 18360 | **92.7%** | ~10K | 868 |
| **L3 (K=1)** | **18369** | **2040** | **11.1%** | ~10K | **13619** |

**关键**: L2 (K=3) 跟 L3 (K=1) 总 cy 几乎相同 (~18-20K), 但 fire 数差 9×。

- L2 一行 dispatcher 拉 17 × 1 × 16 = 272 byte → 约 30 cy
  mac_array 一行 fire = sub_W × K² × cin = 17 × 9 × 1 = 153 cy
  → 计算 5× 拉数据时间, **compute-bound**, util 92.7%

- L3 一行 dispatcher 拉相同 ~30 cy
  mac_array 一行 fire = sub_W × K² × cin = 17 × 1 × 1 = 17 cy
  → 拉数据 1.76× 计算时间, 但加上 cross-mem cmd 串行 + parf drain + ring_wait, 总
  cy 跟 L2 类似, fire 9× 少, util 9× 低 → **memory-bound**

## 结论

**ds layer 不是 PE 阵列空转, 是数据传输瓶颈**。

PE 阵列 100% 利用率 **per fire**, 但 fire 之间被数据传输阻塞。

## 论文素材点

### 推论 1: §12 推测 C 错了

§12 推荐"mac_array Ky-cout-fold / 小 K 引擎" 解决 ds layer。但根因是 memory-bound,
fold cin/cout 不能减少数据量 (cin 已经满 PE row), 反而增加内存压力。

**正确优化**: 减少数据传输量 / 时间, 不是改 mac_array。

### 推论 2: ds layer 跟主路径共享 IFM

ResNet block 结构:
```
input ──→ B1.C1 (K=3 s=2) ──→ B1.C2 (K=3 s=1) ──→ +──→ output
   └────→ B2.ds (K=1 s=2) ────────────────────────→
```

L3 (B2.ds) 跟 L1 (B1.C1) 输入相同 (block input)。当前 layer-sequential 架构下
L3 重新拉 input 浪费。

**优化方向**: `IFB 跨 layer 共享` — L1 跑完后, L3 用相同 IFB (不需重新 IDMA 拉)。
需要 driver + sequencer 改造支持 "skip_idma" 复用 IFB。

预期收益: ds layer act_id ≈ 50-70% 减小 → cy 减 30-40% → 单 ds layer 减 5K cy → 整网 -7.5%。

### 推论 3: 算法层合并 (论文创新点)

K=1 ds + K=3 conv 在数学上等价于 K=3 conv with shortcut + bias add。
编译器层把两者合并成单 K=3 layer (额外 add/scale path)。

实施挑战: SDP 需要支持双 input add (当前已支持 residual_en + shortcut_mult)。
关键是把 ds 的 **W matrix** 合并进 K=3 conv 的 W matrix (math: kernel reshape)。

预期收益: 完全消除 ds layer cycles → 整网 -16% (ds 占 17% 总 cy).

### 综合优化潜力 (基于实验数据)

| 优化项 | 预期收益 | 工程量 | 论文价值 |
|---|---:|---|---|
| Round C/F/G/H (已做) | **6.0%** | 1 周 | 中 |
| **互联简化** (减 SMC IP overhead) | 7.0% | 1-2 周 | 高 (NUMA 互联设计) |
| **ds layer IFB 复用** (skip_idma + sequencer) | 7-10% | 2-3 周 | 高 (residual 优化) |
| **算法层 K=1 合并** | 16% | 2-4 周 | **极高** (编译器创新) |
| **mac_array 流水深化** (K=3 PE util 92→98%) | 2-3% | 1 周 | 中 |
| **bias_rf ping-pong** | 0.06% | 失败放弃 | 低 |
| **dispatcher prefetch** | ≤2% | 失败放弃 | 低 |

**最大单一优化方向: 算法层 K=1 ds 跟 K=3 conv 合并** (16% 收益, 论文创新点)。

## 数据来源

- baseline IP path: 204240 cy
- 数据从 `dump_pe_profile` 提取 (per-core per-layer fire/idle/stall counter)
- dispatcher state breakdown (exp4) 显示 data_cy ≈ 90K/核 全网, 平均 8K/层
- ds layer act_id ≈ 等数据时间 ≈ 75% 总 cy (跟 dispatcher data_cy 一致)
