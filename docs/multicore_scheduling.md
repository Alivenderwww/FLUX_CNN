# 多核调度器设计 (M2.5 编译器扩展)

> 目标: PyTorch model → 多核 stage 划分 → per-core descriptor list, 让 N 核流水线
> 协同跑完整网络. 500fps 目标依赖此调度器把工作均匀分给 ≥3 核.

## 1. 支持的 4 种核协同模式

### 模式 A: 单核单层 (1×1)
最简单, 一个核独占跑完一层. 当前 single-core / cross-core toy case 都是这种模式.

### 模式 B: 单核多层 (1×N)
一个核连续跑 N 层 (核内串行). 用在 **小计算量层** (e.g. 1×1 conv, depthwise),
浪费一个核独占跑不划算. layer 间 OFM → DDR → 下层 IFM (本核 IDMA reload).

### 模式 C: 多核单层 (M×1)
M 个核同时分担一层, 适合 **大计算量层** (整核饱和或装不下).
两种切片方式 (用户拍板的限制):

#### C.1 Cout 切片 (推荐主路径)
- N 核各负责 cout 段: `core[i].cout ∈ [i*C/N, (i+1)*C/N)`
- **Input 广播**: 所有核 IDMA 同份 IFM 数据 (跨核 push 或 DDR 各拉一份)
- **Weight 独立**: 每核 WB 加载自己 cout 段的权重
- **Output 拼接**: 每核 ODMA 写到 DDR 中 cout 偏移位置 `OFM[..., i*C/N : (i+1)*C/N]`
- **适合**: cout 大且能被 N 整除整, e.g. ResNet-18 后段 (cout=64, 128, 256)

#### C.2 W 切片 (备路径)
- N 核各负责 W 段: `core[i].W ∈ [i*W/N - K/2, (i+1)*W/N + K/2)` (含 halo)
- **Weight 广播**: 所有核 WB 加载完整权重
- **Input 含 halo**: 每核 IDMA 拉自己段 + 边界 K-1 列
- **Output 拼接**: 每核 ODMA 写到 OFM 的 W 偏移位置
- **适合**: H/W 大但 cout 小, 早期层 (H=240, W=135, cout=16)

#### 不做的切片
- **H 切片**: 流水难形成 (line_buffer cold-start 需要 K 行, 切 H 后多核冷启同步代价高)
- **Cin 切片**: 多核结果做 shortcut 累加, 增加跨核同步复杂度

### 模式 D: 多核多层 (M×N) — 宏观策略
A/B/C 的组合. **stage 划分** 把网络切成多个 segment, 每 segment 内 N 核协同 (跨核流水线), segment 间走 DDR.

## 2. 编译器调度算法 (5 阶段)

### Phase 1: 计算量分析

```python
# 输入: PyTorch model (或我们的 chain layer list)
# 输出: per-layer compute (MAC count) + 资源 footprint
for layer in net.layers:
    layer.macs = H_out × W_out × C_out × K² × C_in
    layer.ifb_words = H_in × W_in × cin_slices         # 单核装载需要的 IFB SRAM
    layer.wb_words  = K² × cout_slices × cin_slices    # 单核装载需要的 WB SRAM
    layer.ofb_words = H_out × W_out × cout_slices

total_macs = sum(layer.macs)
```

例: ResNet-18-like 11-case 总 MAC ≈ 整网 P MACs (具体由 11-case profile 算).

### Phase 2: 计算理论最佳划分

```python
# 假设核数 N, 目标核利用率 U (e.g. 0.85)
target_per_core_macs = total_macs / N

# stage 数: 流水线 segment 数. 每 stage 内多核协同, segment 间 DDR sync.
# stage 数选取由 SRAM 容量 + 流水线效率决定:
#   - stage 太多 → DDR 流量大
#   - stage 太少 → 单 stage 太长, cold-start 占比高
# 推荐: num_stages = ceil(total_macs / (N × per_core_max_macs))
#   per_core_max_macs ~ 单核 SRAM 装满时一次能跑的最大 MAC 数

num_stages = compute_stage_count(total_macs, N, sram_constraints)
target_stage_macs = total_macs / num_stages
```

### Phase 3: 切分策略选择 (per-layer)

每层独立决定 mode A/B/C：

```python
def choose_mode(layer, N_avail):
    """N_avail = 当前 stage 可用核数"""
    macs = layer.macs

    # 资源约束: 一个核能不能装下整层?
    fits_in_one_core = (layer.ifb_words ≤ IFB_SRAM
                        and layer.wb_words ≤ WB_SRAM
                        and layer.ofb_words ≤ OFB_SRAM)

    if not fits_in_one_core:
        # 必须切片
        if layer.c_out >= N_avail × 16:       # cout 够大 N 核分得开
            return ('C.1_Cout_slice', N_avail)
        else:
            return ('C.2_W_slice', N_avail)

    # 装得下, 但是否需要多核来均衡负载?
    if macs > target_stage_macs / N_avail:
        # 大层, 多核分担
        return ('C.1_Cout_slice', N_avail) if layer.c_out >= N_avail × 16 else ('C.2_W_slice', N_avail)
    else:
        # 小层, 单核独占 (mode A) 或者打包多个小层进一个核 (mode B)
        return ('A_single', 1)
```

### Phase 4: stage 内核 mapping (微观)

stage 内决定: 这一层放在 **具体哪个核编号**? 优化目标 = **最小化跨核 DDR 流量**.

```python
# 跨核 push 路径: layer i 在 core[a] → ODMA push → layer i+1 在 core[b].
#   - 如果 a == b: 同核两层, 必须 OFB→DDR→IFB (本核 reload), 慢
#   - 如果 a != b: 跨核 push (axi crossbar 直送), 快
# 所以连续两层尽量分在不同核.

# 多核分一层时 (mode C), N 个核一起干, 分配明确.
# 单核多层时 (mode B), 选一个 "正在 idle 或负载轻" 的核打包.

def assign_cores(stage_layers, N):
    core_load = [0] * N           # 每核累计 cycle
    layer_to_cores = {}           # layer_id → list of core_id

    for layer in stage_layers:
        mode, n_cores = choose_mode(layer, N)
        if mode == 'A_single':
            # 选 load 最轻的核
            target_core = argmin(core_load)
            # 但要避免跟前一层同核 (跨核 push)
            if prev_layer.cores == [target_core] and len(unique_loads) > 1:
                target_core = second_min(core_load)
            layer_to_cores[layer.id] = [target_core]
            core_load[target_core] += layer.cycles
        elif mode == 'C.1_Cout_slice':
            # 分给所有 N 核, 每核 cout/N
            layer_to_cores[layer.id] = list(range(N))
            for c in range(N):
                core_load[c] += layer.cycles / N
        # ...

    return layer_to_cores
```

### Phase 5: per-core descriptor 生成

最后从核视角看:

```python
for core_id in range(N):
    desc_list = []
    for layer in net.layers:
        if core_id in layer_to_cores[layer.id]:
            cfg = build_cfg(layer, slice_info=...)
            #   cfg.ODMA_DST_BASE = (跨核 push 时) consumer 核 IFB region
            #                     = (DDR 中转时) DDR OFB 区
            #   cfg.SKIP_IDMA   = (跨核 push 时, 自己是 consumer) 1 否则 0
            #   cfg.IDMA_SRC_BASE = (Cout 切片时) DDR 整 IFM
            #                     = (W 切片时) DDR IFM 的本核 W 段+halo
            #   cfg.WDMA_SRC_BASE = (Cout 切片时) DDR 中本核 cout 段权重
            desc_list.append(cfg)
    write_desc_list(core_id, desc_list)
```

## 3. 关键设计点 + trade-off

### T.1 Stage 边界判定
stage 切换 = 必须经 DDR. 选 stage 边界的依据:
- **a) 核 SRAM 不够装**: 必须断流水放 DDR
- **b) Layer 类型变化**: 比如 conv → pool → conv (pool 在 SDP 后处理目前没有, 推迟)
- **c) 平衡每 stage MAC 数**: 让每 stage 跑时间接近, 才能流水 efficiency 高

当前推荐: 简单贪心 — **从前往后累加 layer.cycles, 累计到 target_stage_cycles 就切**.

### T.2 Cout 切片对齐约束
- N 核切 cout, 要求 cout % (N × 16) == 0 才整齐 (16 = NUM_COL).
- 不整齐时 padding 到对齐, 多余 cout 算空 (浪费 PE)
- 或者最后一核分多/少 (不整齐切, 编译器侧记录每核 cout 数)

简单实现: padding. 性能稍降但代码简单.

### T.3 W 切片 halo 大小
- halo = (K - 1) / 2 列, 跨核需要 share
- 实现: producer 核也写 halo 列的 OFM 给两个 consumer 核 (跨核 push 重复发送)
- 或者: consumer 核 IDMA 直接拉自己段 + halo (DDR 重读 halo 列)

简单实现: DDR 重读 halo. 节省跨核流量但 DDR 流量稍增.

### T.4 跨核 push vs DDR 中转的选择
- 跨核 push: layer i → layer i+1 在不同核, 数据走 axi crossbar 直送 IFB SRAM
- DDR 中转: 数据出 OFB → DDR → 下层 IDMA 重读

stage 内连续 layer **应该尽量** 跨核 push (省 DDR). 但要保证 consumer 核 SKIP_IDMA + 启动顺序. 编译器要正确生成 cfg.

### T.4.1 N 核拓扑 (重要澄清)

当前 RTL 用 Xilinx `axi_NtoM` IP, 是 **逻辑全连接星形 crossbar**:
- N 个核 master (SI[0..N-1]) ↔ 1 个 DDR + N 个核 IFB slave (MI[0..N])
- 任何核可以写任何核的 IFB region (地址路由)
- 物理上不是 2D mesh 也不是 chain
- 共享 1 × BUS_DATA_W (128 bit) 总线带宽, N 核同时 push 时**串行仲裁**

不同于 AMD-AIE 物理 mesh 拓扑. 带宽约束: N 核同时 push 时单核有效带宽 = 128/N bit/cycle.
但 ResNet 每层 OFM 数据量小 (~KB 级), 仲裁延迟一般不是瓶颈.

### T.4.2 两条独立的数据流路径

容易混淆的两条独立机制:

#### 路径 A: 跨层流水线 (mode A 多核 chain)
- producer (core a) ODMA push → axi crossbar → consumer (core b) ifb_axi_slave → IFB SRAM
- consumer SKIP_IDMA=1, **不通过 IDMA**, 完全被动接收
- 当前 M2 已实现 ✅

#### 路径 B: 单层多核切片内部 (mode C IFM 来源)
mode C.1/C.2 下 N 个核同跑一层, 每核需要拿到自己的输入数据:
- **每核独立 IDMA 从 DDR 读**自己段 (cfg.SKIP_IDMA=0)
- 各核 IDMA cfg 不同 (SRC_BASE / ROW_STRIDE / byte_len 各异)
- 这跟跨核 push 路径不冲突, 是 **stage 边界后** 的并发独立读

### T.4.3 mode A ↔ mode C 切换的 stage 边界

axi crossbar 不支持 1→N broadcast. mode A → mode C.1 (N 核都需要同一份 IFM) 时:
- **当前推荐**: 走 DDR 中转 (mode A producer 写 DDR, N 核 consumer 各自 IDMA 拉 DDR)
- 替代方案 (后续): producer 发 N 次 ODMA cmd 写 N 个 consumer IFB region (重复发送, 不经 DDR), 编译器侧实现, 不改 RTL

实用编译器策略: **stage 内统一 mode**, 要么全 mode A 流水, 要么全 mode C 切片. stage 边界走 DDR.

### T.4.4 mode C 切片的 OFM 拼接

#### Cout 切片
N 核各产 cout 的连续段, DDR 上 OFM 按 NHWC 排. 每核 ODMA cfg:
- `ODMA_DST_BASE = base + my_cout_start * 16` (cout segment 偏移)
- `DDR_OFM_ROW_STRIDE` = 全核共享 (整 OFM 一行)
- `byte_len = h_out × w_out × my_cout_segment_size × 16`

#### W 切片
N 核各产 W 的连续段 (含 halo, 但只写自己 W 段不含 halo). DDR 上 OFM 按行交错拼接:
- `ODMA_DST_BASE = base + my_w_start * cout_slices * 16` (W 段在行内偏移)
- `DDR_OFM_ROW_STRIDE` = 全核共享 (整 OFM 一行)
- `byte_len = h_out × my_w_segment_size × cout_slices × 16`

ODMA RTL **已支持** 这种 row-by-row 写 + arbitrary stride, 编译器侧把 base/stride 算对即可, 不改 RTL.

### T.5 反压 cold-start
跨核流水线 cold-start 时:
- consumer line_buffer 需要 ifb_strip_rows 行才开始 issue
- producer 写 ifb_strip_rows 行后反压 (ring 满)
- consumer 开始消费, 反压解, 流水线动起来

约束: `ifb_strip_rows >= K_consumer` (consumer 一次 issue 需要的最小行数).
编译器 validate 时必须 check, 否则死锁.

## 4. ResNet-18-like 11-case 上的 mapping 例子

整网 11 case 各 cycle 数 (单核基线):
| # | Name | cycles | dim | C_in→C_out | 备注 |
|---|---|---|---|---|---|
| 0 | Patch | 135K | 240×135 | 4→16 | s2d, 大尺寸 |
| 1 | L1.B1.C1 | 74K | 120×68 | 16→16 | stride=2 |
| 2 | L1.B1.C2 | 76K | 120×68 | 16→16 | |
| 3 | L1.B2.ds | 43K | 120×68 | 16→16 | 残差 |
| 4 | L2.B1.C1 | 39K | 60×34 | 16→32 | stride=2 |
| 5 | L2.B1.C2 | 77K | 60×34 | 32→32 | |
| 6 | L2.B2.ds | 14K | 60×34 | 16→32 | 残差 |
| 7 | L3.B1.C1 | 38K | 30×17 | 32→64 | stride=2 |
| 8 | L3.B1.C2 | 79K | 30×17 | 64→64 | |
| 9 | L3.B2.ds | 7K | 30×17 | 32→64 | 残差 |
| 10 | FC | 10K | 1×1 | 256→522 | |

总: 593K cycles.

### N=2 双核流水线 mapping (示例)

```
Stage 0 (core 0+1 流水): Patch(135K) + L1.B1.C1(74K) + L1.B1.C2(76K) + L1.B2.ds(43K)
   - Patch 用 mode C.2 (W 切片 240×135 沿 W 切, c_out=16 太小不能 cout 切)
     core 0 算 W[0:67], core 1 算 W[67:135]
   - L1.B1.C1 用 mode A 给 core 0 (74K)
   - L1.B1.C2 用 mode A 给 core 1 (76K) → 跨核 push 从 core 0 到 core 1
   - L1.B2.ds 用 mode A 给 core 0 (43K) → 跨核 push 从 core 1 到 core 0
   stage 0 总 cycles ≈ max(core 0 = 74+43+135/2, core 1 = 76+135/2) ≈ 184K

Stage 1: L2.B1.C1 + L2.B1.C2 + L2.B2.ds (130K)
Stage 2: L3.B1.C1 + L3.B1.C2 + L3.B2.ds (124K) + FC (10K)

理想 N=2 总时间: max(stage 0..3) × num_stages, 流水满管时 ≈ 整网/N = 297K cycles ≈ 2.97ms = 336fps
实际反压 + cold-start 损失 ~10%: ~300fps
```

### N=4 四核流水线 mapping

```
Stage 0: Patch + L1 全部 (4 layers) → 4 核每核一层, 形成 ping-pong
   - Patch (135K) 太大, 用 mode C.1 cout 切片到 4 核? cout=16 / 4 = 4, 太小
   - 用 mode C.2 W 切片 4 核, 每核 W ≈ 33 列 + halo
   - L1.B1.C1, C2, ds 也走 4 核切片或者各核一层 mode A pipeline
   stage 0 ≈ 整网 L1 cycles / 4 = (135+74+76+43)/4 ≈ 82K

Stage 1: L2 全部
Stage 2: L3 全部 + FC

理想 N=4 总时间: ~593/4 = 148K cycles ≈ 1.48ms = 670fps
扣损失: ~500fps ✓ 达成目标
```

## 5. 工作量分阶段

### M2.5 第一步 (1 周): 模式 A + 模式 B + 跨核 push (已有)
- 当前 toolchain 支持单核多层 (run_regression.py chain) ✓
- M2 跨核 push 已通 ✓
- **要补**: 把 11-case 拆成 2 stage 跑跨核 chain (实际上是 mode B 在两核上手工 map)
- TB: 扩 `tb_multicore_xcore.sv` 支持 N 层 chain 而非 2 层

### M2.5 第二步 (1-2 周): 模式 C.1 (Cout 切片)
- 编译器: 加 split_layer_cout(layer, N) 函数
  - 把 cout 维度切 N 段, 生成 N 份 cfg
  - 每份 cfg 的 ODMA_DST_BASE 偏移到 OFM 的 cout 段位置
  - 每份 cfg 的 WDMA_SRC_BASE 偏移到 weight 的 cout 段位置
- RTL 不需改 (核内本来就支持任意 cout_slices)
- TB: 验证 N 核同时跑同一层的 cout 切片版, OFM 拼接结果跟单核一致

### M2.5 第三步 (1-2 周): 模式 C.2 (W 切片)
- 编译器: 加 split_layer_w(layer, N) 函数
- RTL: 暂不改, 用 DDR halo 重读
- TB: 验证 N 核同时跑同一层的 W 切片版, OFM 拼接结果一致

### M2.5 第四步 (1 周): stage 调度器
- compute_stage_split(layers, N, sram_constraints) → list of stage
- per-stage assign_cores(stage_layers, N) → 核分配
- 端到端 11-case 在 N=2/4 跑通

### M3 (后期): MLIR/TVM 整合
roadmap.md 已记录, 推后做.

## 6. 代码组织建议

新建 `toolchain/scheduler.py`:
```python
class Layer:
    def __init__(self, name, k, c_in, c_out, h_in, w_in, stride, pad):
        ...
    def macs(self):
        return self.h_out * self.w_out * self.c_out * self.k**2 * self.c_in

def schedule(layers, N_cores, sram_constraints):
    """
    Returns: List[Stage], Stage = (layer, mode, core_assignment)
    """
    # Phase 1: compute MACs
    # Phase 2: stage split
    # Phase 3: per-layer mode selection
    # Phase 4: core mapping
    # Phase 5: gen per-core desc
    return stages

def gen_per_core_descs(stages, N_cores) -> Dict[int, DescList]:
    """从核视角生成 desc list"""
    for core_id in range(N_cores):
        ...
```

`run_regression.py` 现有 chain 路径作为 N=1 的 special case 保留.
新加 `run_multicore.py` 调用 scheduler + 多核 TB driver.
