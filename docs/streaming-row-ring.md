# Streaming Row-Ring 架构设计规范

**状态**：**已实现并回归通过**（Phase A→D，含 480×640 VGA 端到端验证，`sim/tb_core_dma/run_regression_stream.py` 11/11 PASS）。以下按 (目标) → (架构) → (实现结果) 的顺序组织。

**动机**：让单核能处理远超内部 SRAM 容量的图像（如 480×640 / 224×224 / 1024×1024），且：

- 一次 `start` 跑完整层，host 不再分条带调度
- 外部 AXI 输入数据按行流水、**不重复搬运**（重叠行靠内部 ring 保留）
- IDMA / 核 / ODMA 三阶段**同时活着**（真正流水），而不是 strip-tiling 的串行
- 天然匹配多核 mesh：两核间的 row-level 流直接衔接，等价于 AIE 的 cascade stream

---

## 1. 顶层数据流

```
DDR  ──AXI AR/R──► IDMA ─▶ IFB (row-ring) ─▶ line_buffer ─▶ mac_array
                     ▲                           │
                     │ row-credit                │
                     └───────────────────────────┘
                                                 │
                                                 ▼
                           parf_accum ─▶ ofb_writer ─▶ OFB (row-ring)
                                                                 │
                                                                 ▼
                                                               ODMA ──AXI AW/W──► DDR
                                                                 ▲
                                                                 │ row-credit
                                                                 └──────────────── ofb_writer
```

**反压双链**：
- **输入侧**：`line_buffer` 消费行 → 产生 `ifb_rows_consumed` credit → 控制 IDMA 何时可写下一行
- **输出侧**：`ofb_writer` 产出行 → 产生 `ofb_rows_produced` 信号 → 触发 ODMA 搬该行；ODMA 完成后释放 credit 给 ofb_writer

---

## 2. IFB 作为 row-level ring buffer

### 2.1 ring 参数（编译时 + cfg 时两级）

| 量 | 类型 | 说明 |
|---|---|---|
| `SRAM_DEPTH` | 编译参数 | IFB / WB / OFB 硬件深度（= 8192 word，不变）|
| `cfg_h_in_total` | cfg 寄存器 | 整张图输入高度（e.g. 480），IDMA 跑完这么多行算结束 |
| `cfg_strip_rows` | cfg 寄存器 | IFB ring 容纳的行数 N_ring（e.g. 34）|
| `cfg_w_in` | cfg 寄存器（已存在） | 每行宽度，决定每行占 `W_IN × cin_slices` 个字 |

**容量约束**（软件 check）：
```
cfg_strip_rows × cfg_w_in × cfg_cin_slices  ≤  SRAM_DEPTH
cfg_strip_rows  ≥  K + margin     // margin ≥ 1 即可形成流水
```

典型值：
- 480×640 Cin=3 K=3：`strip_rows = 12`（SRAM 装 12×640=7680 字 ≤ 8192），K+margin 需 ≥4，充裕
- 224×224 Cin=16 K=3：`strip_rows = 32`（32×224=7168 字），margin 29 行——非常充裕
- 1024×1024 Cin=1 (Cin=3 → cin_slices=1)：`strip_rows = 8`，K+margin=4 时刚够

### 2.2 地址生成

IFB 的物理地址**按行分桶**：

```
row_slot = (ifb_row_idx) mod cfg_strip_rows
word_offset_in_row = y_pixel * cfg_cin_slices + cin_slice_idx   // 行内索引
phys_addr = row_slot * cfg_w_in * cfg_cin_slices + word_offset_in_row
```

对 `line_buffer` 而言：原来 `ptr_ky_base` 走线性 += `cfg_w_in`，现在要按 `cfg_strip_rows` 取模：
```
ptr_ky_base 新值 = (ptr_ky_base + cfg_w_in) mod (strip_rows × W_IN × cin_slices)
```

实现上用一个追加的 row counter，用比较器识别环绕而非真做除法。

### 2.3 ring 尚未写满的启动阶段

核启动后，`line_buffer` 要等 IFB 至少存了 `K` 行才能开始发第一个 act——这是正常 warmup，由 row-credit 自然卡住。

---

## 3. Row-Credit 反压协议

### 3.1 两个全局计数器（位于 core_top，跨模块共享）

| 信号 | 驱动者 | 消费者 | 语义 |
|---|---|---|---|
| `rows_written`（log2(H_IN)-bit） | IDMA | IFB 写入控制 | IDMA 写完一整行时 +1 |
| `rows_consumed`（同） | line_buffer | IDMA 流控 | line_buffer 彻底用完某行时 +1 |

### 3.2 IDMA 流控（streaming mode）

IDMA 新增一条 S_WAIT 状态；发 AR 前检查：

```
ifb_full = (rows_written - rows_consumed) >= cfg_strip_rows
ifb_room = !ifb_full && (rows_written < cfg_h_in_total)

S_IDLE  → start               → S_AR
S_AR    → !ifb_room           → S_WAIT
S_AR    → ar_fire             → S_R
S_R     → r_last_fire         → 更新 rows_written；若 rows_written == h_in_total → S_DONE
                                 否则 → (ifb_room ? S_AR : S_WAIT)
S_WAIT  → ifb_room            → S_AR
```

每次 AR 发**一整行**（= `w_in × cin_slices × 16` 字节），不超 AXI burst 256-beat 上限则单 burst，否则自动切。`src_base` 随 `rows_written` 线性推进（`+= row_bytes_in_ddr`）。

### 3.3 line_buffer 的 "rows_consumed" 推进

一个输入行被 "彻底用完" 的判据（K=3, stride=1 为例）：
- 当计算 OFM 行 `y_out` 时，使用 IFB 行 `y_out, y_out+1, y_out+2`
- 当 `y_out` 推进到 `y_out+1` 时，IFB 行 `y_out` 不再被用 → `rows_consumed += 1`

实现：line_buffer 已有 `yout_cnt`；每 yout 推进（当前 `evt_iss_yout_wrap`？实际是 tile wrap 到下一 yout 时）→ `rows_consumed <= rows_consumed + 1`。

**跨 cin_slice**：cin_slices > 1 时，同一行的多个切片都要用完才能 release。但切片共享行编号；应按 `rows_consumed += 1` 时机 = "最后一个 cin_slice 也用完"。可简单地只在 cin_slice 最后一次的 yout 推进时 +1。

---

## 4. ODMA 侧 row-credit

### 4.1 OFB 作输出 ring（小）

- OFB 物理深度 8192（已有），足够存多行
- `cfg_ofb_strip_rows` 决定 OFB 可容纳的 OFM 行数（可小至 2 行即可形成流水）

### 4.2 ofb_writer 产出信号

新增：`ofb_row_done_pulse`——每写完一行 OFM（yout 推进时）一拍脉冲给 ODMA。

### 4.3 ODMA 流控

ODMA 新增 streaming mode：每收到一个 `ofb_row_done_pulse`，发一个 row-sized burst 把 OFB 这行写到 DDR；回来 BRESP 后 release 那行的 OFB 槽。

简化版：ODMA 维护 `ofb_rows_pending`（产出未搬走的行数），`ofb_rows_pending >= cfg_ofb_strip_rows` 时反压 `ofb_writer` 暂停推进 yout。

---

## 5. cfg_regs 新增字段

| 地址 | 字段 | 宽度 | 含义 |
|---|---|---|---|
| 0x168 | `H_IN_TOTAL` | 16 | 整张图输入高度（IDMA 总行数）|
| 0x16C | `IFB_STRIP_ROWS` | 8 | IFB ring 容纳的输入行数（K 到 ~64）|
| 0x170 | `OFB_STRIP_ROWS` | 6 | OFB ring 容纳的输出行数（≥ 2）|
| 0x174 | `DDR_IFM_ROW_STRIDE` | 20 | DDR 里相邻输入行跨度（字节），供 IDMA 地址发生器用 |
| 0x178 | `DDR_OFM_ROW_STRIDE` | 20 | DDR 里相邻输出行跨度，供 ODMA |

**注**：`IDMA_BYTE_LEN` / `ODMA_BYTE_LEN` 的语义在 streaming mode 下变为「总图像字节数」，DMA 按行切分。

新增一个 CTRL 位或独立的 `DMA_MODE` 字段：
- `DMA_MODE[0]` = `IDMA_STREAMING`（0=batch 原行为，1=streaming row-by-row）
- `DMA_MODE[1]` = `ODMA_STREAMING`

默认 batch 模式保证 v1 所有回归直接兼容。

---

## 5.5 v2 硬限制：Cout ≤ NUM_COL (= 16)

当前循环顺序 `cs` 最外层：对每 cout_slice 跑完整张图。要跑大图 streaming，要求每行 IFM 只过一次 → cs 不能外循环。

**两条路**：
- **v2：限制 Cout ≤ 16** → cs=1，天然不存在跨 cs 重读问题。streaming 完整生效。
- **v3：cs 内循环重构** → 需要改 line_buffer（yout→cs 调换）、wgt_buffer（每次 iteration 切换 WRF 读的 cs 偏移）、ofb_writer（per-cs 的 parf 维护）。工作量比 v2 chunked 那次还大。

**v2 实现时 cfg_regs 做 assertion**：`streaming_mode=1 && cfg_cout_slices > 1` → 报错或退回 batch。

---

## 6. 模块改动清单

| 模块 | 改动强度 | 要点 |
|---|---|---|
| `cfg_regs.sv` | 低 | 加 5 个新字段 |
| `line_buffer.sv` | 中 | `ptr_ky_base` / `ptr_kx_base` mod strip_rows；加 `rows_consumed` 输出 |
| `idma.sv` | 高 | 加 streaming mode（cfg 位选择）、S_WAIT、row-level 地址推进、row-credit 接收 |
| `odma.sv` | 中 | 加 streaming mode、row-level 地址推进、`ofb_row_done_pulse` 触发 |
| `ofb_writer.sv` | 低 | 加 `row_done_pulse` 输出 |
| `core_top.sv` | 低 | 连线 rows_written / rows_consumed / ofb_row_done |
| `gen_isa_test.py` | 中 | 新增 cfg 字段输出；support "streaming" 和 "batch" 两种 descriptor 模式 |
| TB | 中 | tb_core_dma 加 streaming 模式，大图测试用例 |

---

## 7. 边界 case / 风险点

### 7.1 stride > 1

OFM 行 y 用 IFB 行 `y*stride, y*stride+1, ..., y*stride+K-1`。stride=2 时 IFB 行推进速度翻倍。`rows_consumed` 的推进要按 "`yout_cnt` 推进 → `rows_consumed += stride`"。

### 7.2 cin_slices > 1

每个 IFB 行在物理上存 `cin_slices` 个切片（沿 cin 维度铺开）。`rows_written` 的含义是 "整行（所有切片）都 DMA 完毕"。IDMA 发一个 AR 就把一整行的所有切片都拉进来。

### 7.3 strip_rows 过小导致 deadlock

若 `strip_rows < K`，IFB 永远不够 K 行，line_buffer 永远 empty，core 卡死。软件必须保证 `strip_rows ≥ K`。运行时可加一个 assertion 检查。

### 7.4 首行 / 末行 padding

不支持（本设计和当前一致）。软件对 padding 要预处理 ddr 或加独立 padding 模块（phase 2）。

### 7.5 小图（图本身不大于 strip_rows）

软件可直接设 `strip_rows = h_in_total`，退化成 batch mode，完全兼容现有行为。

---

## 8. 多核扩展视角

### 8.1 核间流

两核级联（Core A → Core B）：
- Core A 的 ODMA 改为 "写到核间 stream 端口"（不是 DDR）
- Core B 的 IDMA 改为 "从 stream 端口读"
- 两者共用同一套 row-credit 协议：Core B 每消费一行，给 Core A credit 允许写下一行

物理上这条 "stream" 可以是：
- **片上 AXI-Stream 或自研 stream**（短距离，低延迟）
- **共享 MemTile SRAM**（类 AIE，两核共访问）
- **NoC 多跳**（多核 mesh）

### 8.2 权重分布

多核共同做一层时，`cout` 维度切分给不同核；每核各自 IFM 同相，输出聚合。权重 DMA 一次性从 DDR 拉到各核的 WB，不走 stream。

### 8.3 Psum cascade

深度可分（depth-split）场景下，多核在 cin 维度分片，每核产出 partial psum，通过 cascade stream 累加到下游核。和 AIE cascade 的用法一致。

---

## 9. 实现 Phase 划分

建议分 4 个 phase，每个 phase 做完能独立回归：

### Phase A：cfg_regs + line_buffer 地址 wrap

- cfg 新增字段（默认值让行为兼容 batch mode）
- line_buffer 地址推进改 mod 模式（当 `strip_rows == h_in_total` 时等价于原线性）
- 现有 tb_core_isa / tb_core_dma 回归必须全绿（cycles/MAC% 不变）

### Phase B：IDMA streaming mode

- 加 streaming mode 位，FSM 加 S_WAIT
- row-level 地址推进
- rows_consumed 信号接线（暂时从 line_buffer 直接喂，后续可做精细化）
- 新 TB 测试：中等大小图（96×128 Cin=3），`strip_rows=16`，完整跑通

### Phase C：ofb_writer + ODMA streaming

- ofb_writer 加 `row_done_pulse`
- ODMA streaming mode
- 端到端大图测试：480×640 Cin=3 K=3

### Phase D：stride=2、cin_slices>1、K=7 覆盖

- 完善边界 case
- 扩充回归用例集

---

## 10. 关键指标验证

对于 480×640 Cin=3 Cout=8 K=3 stride=1：
- 理论 MAC = 478×638×3×8×9 ≈ 65.9M MACs
- 单核 256 PE，理论最少 cycles = 65.9M / 256 ≈ 257k cycles
- 目标 MAC 利用率：≥ 99%（和当前 tile-local 跑一致）
- 新增 overhead：ring warmup (K-1 行等 IDMA) 约 100 cycles 级，可忽略

DDR 带宽需求（100 MHz 核时钟，128-bit AXI）：

AXI4 的 AR+R（读）和 AW+W+B（写）**在协议上互相独立**，每个方向峰值 128-bit × 100 MHz = **1.6 GB/s**。

- **读方向**：IDMA 输入 4.7 MB + WDMA 约 几 KB（一次性），257k cycles 内完成 → ~1.5 GB/s，**≤ 1.6** ✓
- **写方向**：ODMA 输出 4.7 MB 同样 ~1.5 GB/s，**≤ 1.6** ✓
- 两方向**同时跑**，当前规格刚好够，利用率 ~93%

这个余量对 HBM / DDR4 的实际 latency 偏紧，真实部署若 DDR 有效带宽低于 1.6 GB/s，需要把核时钟拉到 200 MHz 或 AXI 宽度拉到 256-bit。但协议层面没有 double-count 问题。

---

## 11. 设计决策（已定）

1. ✅ **strip_rows 分 IFB/OFB 两个独立 cfg 字段**：`IFB_STRIP_ROWS`、`OFB_STRIP_ROWS`
2. ✅ **DMA 通过 cfg 位切换 streaming/batch**：默认 batch 以兼容现有回归
3. ✅ **rows_consumed 粗颗粒（整行）足够**：line_buffer 循环中 `cins` 在 `yout` 内，整行所有切片天然一并用完，无需 per-slice credit
4. ✅ **stride 通用公式**：`rows_consumed += stride`（stride > K 时若干行未用，v1 仍按序灌，DDR 带宽略浪费，v3 可优化）
5. ✅ **v2 限制 Cout ≤ 16（cs=1）**：v3 再做 cs 内循环重构

**剩余可选优化**（v3+）：
- stride > K 的 DDR 行跳过
- cs 内循环重构支持大 Cout streaming
- Weight streaming（权重也按段流入）

---

## 12. 下一步

按 §9 的 Phase A 开工。先改 cfg_regs 和 line_buffer，保持向后兼容（batch mode 不变），回归过了再推进 Phase B。

---

## 13. 架构方向 B（v3 目标）：AXI Master/Slave 分离

当前（v1 + v2）Core 对外只有一个 AXI M（带所有 3 个 DMA 聚合）。下一代架构应拆成：

```
           AXI-Lite S (CSR)
               │
         ┌─────▼────┐
AXI-S ──►│   Core   │──► AXI-M (OFB 流出)
(input)  │          │
IFB+WB+  │          │
压入    │          │
         └──────────┘
```

**拆分后**：
- Core **对外 2 个端口**：1 S（入），1 M（出）——职责清晰
- IDMA / WDMA **移出 Core**，作为 SoC wrapper 的 peripheral；单核 + DDR 时在外部实例化，多核时直接省略
- Core 的 AXI-S 内部按地址 decode：IFB range / WB range / CFG range，分发写入
- 多核链：`CoreA.M ───► CoreB.S ───► CoreB.M ───► CoreC.S ...` 直连不过 DDR

**对当前 DMA 代码的影响**：
- `idma.sv` / `wdma.sv` / `odma.sv` 全部**保留、可直接复用**
- 只是实例化位置从 `core_top` 内移到 `soc_wrapper.sv`（新文件）
- `core_top` 简化：去掉 `axi_m_mux`，内部只留 `odma` + AXI-S 解码逻辑

v3 时再做；v2 阶段先把 streaming 功能在当前 all-master 拓扑上跑通，证明概念可行。

---

## 14. 实现结果（Phase A→D）

### 14.1 分阶段实施

| Phase | 内容 | 关键改动 |
|---|---|---|
| A-1 | cfg_regs 加 streaming 字段（H_IN_TOTAL / strip_rows / DDR row stride / DMA_MODE） | `r_dma_mode_ctrl` 独立 always_ff 加同步复位（避免 X 传播到 IDMA） |
| A-2 | `line_buffer` 加 ring wrap + `rows_consumed` 输出 | `wrap_addr()` 函数按 `cfg_ifb_cin_step` 取模 |
| B | `idma.sv` 加 streaming：S_WAIT 状态、`row_beats_remaining`、`rows_written`、ring wrap | 关键 fix：`M_ARVALID = (state==S_AR) && !ring_full`（否则 slave 接受无效 AR 导致死锁） |
| C-1 | `ofb_writer.sv` 加 `row_done_pulse` + `rows_written` 输出、ring wrap、`ring_full` 反压 | `r_rows_written` 控制路径加同步复位 |
| C-2 | `odma.sv` 加 streaming：S_AW_WAIT 状态、`row_beats_left` / `rows_produced` / `rows_drained`、按 `ddr_ofm_row_stride` 跨行 | `row_base_addr` 跟踪当前行 DDR base |
| C-3 | `core_top.sv` 串起 row-credit 环：`ofb_writer.row_done_pulse → odma`；`odma.rows_drained → ofb_writer` | `cfg_ddr_ofm_row_stride` 20-bit → 32-bit 零扩展绑定 |
| D-1 | `gen_isa_test.py --streaming`：emit H_IN_TOTAL / strip_rows / DDR stride / DMA_MODE；overwrite `IFB_CIN_STEP` / `OFB_COUT_STEP` 为 ring wrap 模数 | 同时修 `WB_WORDS` plusarg（之前写死 9 导致 K=7 挂） |
| D-2 | `tb_core_dma.sv` 双路流程：`DMA_MODE=0` 走 batch 三段；`=3` 走 streaming 单次 `CTRL=0xB` | 读 local `config.txt` / `sim_params.f`（不再 `../tb_core_isa/`） |
| D-3 | Streaming 回归（8 case 小图）验证 | 发现 line_buffer **forward-pressure** 缺失：未写入的 IFB 行被读出 X → 引入 `rows_available` 输入 + `rows_needed = yout*stride + K` 门控 |
| D-4 | 大图测试：128×128 / 240×320 / 480×640 VGA | DDR 扩到 16 MB；`gen_isa_test.py` PYTHON_SRAM_LIMIT 扩到 512K；streaming 下 `hw_sram_depth` 固定 8192 |

### 14.2 关键修复：两条反压链

设计文档只提到 `rows_consumed`（线路 consumer → producer，防 over-fill）。实测发现必须加 `rows_available`（线路 producer → consumer，防 read-ahead）：

```
IDMA  ──rows_written──▶ rows_available ──▶  line_buffer   (forward-pressure)
IDMA ◀──rows_consumed ◀──                     line_buffer   (back-pressure)

ofb_writer ──row_done_pulse──▶ rows_produced ──▶  odma     (forward-pressure)
ofb_writer ◀──rows_drained ◀──                     odma     (back-pressure)
```

否则 line_buffer 会在 IDMA 首 8 行未完成前就读出 `X`（SRAM uninitialized），传到 MAC 做 `X × w` 产出 `X`，最终 DDR OFM 里都是 `X`。

### 14.3 回归验证

**小图 8 case**（`tb_core_dma` 中复用 `tb_core_isa` cfg 参数，筛 `cin_slices=1 && cout_slices=1`）：
- K=3 C4C4 / C4C8 / C8C4 / C8C8 / C16C16 66×118
- K=5 C16C16 30×58 s=2
- K=7 C8C8 / C16C16 62×114（chunked kk=49）

**大图 3 case**：128×128 C3C8 / 240×320 C3C16 / **480×640 C3C16**

全部 PASS，MAC 利用率 98.56% – 99.93%。代表性能见 `README.md` 性能表。

### 14.4 已知限制（留给 v3）

- **v2 硬限**：`cin_slices == 1 && cout_slices == 1`（Cin/Cout ≤ 16）。cs 外层循环需要把 OFM 行多次产出，打破 row-producer 单调性 → v3 重构 parf_accum / ofb_writer 的 cs 循环位置
- IFB / OFB strip_rows 需满足 `strip_rows ≥ K + margin`（避免下溢）；当前默认 8，K=7 时留 1 行 slack 够用
- 多 burst / 行：W_IN > 256 时 IDMA/ODMA 自动切多 AR/AW burst（代码支持，回归覆盖 W_IN=640）

