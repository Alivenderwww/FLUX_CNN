# 写作评审报告 Phase 4

## 第 1 次评审

### 判定
PASS

### 评审范围

- 主产出：`paper/workspace/paragraph-skeleton.md`（170 段 / §1-§8 / 35 节）
- 上一阶段参考：`paper/workspace/section-summary.md`（重点抽 §6 / §7 节摘要核对一致性）
- 评审重点：主题句聚焦度、类型标签准确性、节内段落分布、跨章衔接（特别是 §6→§7 / §7→§8 三 Writer 边界）、风格一致性、narrative 主线、避免重复
- 不评：事实正确性 / 新颖性 / [CHECK]/[TBD] 标记内容

---

### 抽样 12 段主题句聚焦度

抽样覆盖 §1-§8 各章：

| 段位 | 主题句要点 | 聚焦度 | 备注 |
|------|----------|--------|------|
| §1.1.p2 | 浅层 PE 利用率仅 12.5%-50% 是 narrative A 核心 motivation | OK | 单一论断，配数据 |
| §1.4.p2 | (1) Ky-fold 在 Cin<16 浅层把 PE 利用率提升至接近 100%，零 RTL 改动 | OK | 单一 contribution claim |
| §2.2.p1 | ResNet-18 5 层在 16×16 阵列 chronically PE-underutilized | OK | 聚焦单一现象 |
| §3.4.p3 | Alwani Fused-layer 是 layer-level，FLUX_CNN row-level 只缓几行不缓几层 | OK | comparison 单点对位 |
| §4.1.p1 | 顶层由 5 模块 core pipeline + DMA 子系统构成；外部仅 1 AXI4 M + 1 AXI-Lite S | OK | 顶层结构 claim 单一 |
| §4.3.p1 | IFB/OFB row-ring + forward-pressure + ODMA credit + sequencer 三阶段并发 | **轻微复合** | 单段抛 4 个机制点（ring/forward-pressure/credit/三阶段并发），prose 阶段需拆 |
| §5.1.p1 | Cin<16 时把 Ky 折到 cin，硬件按普通 conv 跑 | OK | 单一 claim |
| §5.2.p1 | stride≥2 把 stride² 相位折到 cin，等价为 stride=1 K_new=⌈K/stride⌉ | OK | 单一变换定义 |
| §6.2.p1 | 自写 ~3000 行 RTL 替换为 Xilinx axi_dm IP + 轻量 *_ctrl，外部 1 M + 1 S | OK | 单一 vendor IP claim |
| §6.4.p1 | multicore_top 参数化 N 核 wrapper，N=4 W slice 20/20 case PASS | OK | 单一 multi-core claim |
| §7.4.p1 | 11-layer chain 593K cycles / 86.6% MAC% / 双 operating point latency | OK | 量化数据节 claim 自然合并 |
| §8.1.p1 | 编译器侧 Ky-fold + S2D 替代 hardware-reconfigurable + row-ring + N=4 multi-core | OK | conclusion 收束 claim 合并合理 |

**结论**：12 段中 11 段主题句聚焦单一论点；§4.3.p1 是机制密集段（4 个机制点串列），但因为它担任"row-ring 主章首段总览"的功能，作为 claim 段抛出全节 scope 是合理的，不算缺陷——只在 prose 阶段提示拆为 2-3 句即可。**无复合 claim 缺陷。**

---

### 类型标签准确性

抽样 8 段核标签：

| 段位 | 标签 | 内容 | 一致性 |
|------|------|------|--------|
| §1.1.p1 | setup | "固定空间阵列 CNN 加速器已成主流形态…" | OK，纯背景 |
| §1.4.p2 | claim | "(1) Ky-fold 把 PE 利用率提升至接近 100%" | OK，论断 + quantified |
| §2.1.p2 | evidence | "TPU/Eyeriss/ShiDianNao 是 WS/RS/OS 代表" | OK，举证支持 §2.1.p1 taxonomy |
| §3.4.p3 | comparison | "Alwani layer-level vs FLUX_CNN row-level" | OK，对位 |
| §4.2.p2 | evidence | "counter+ready 反压 / FILL/DRAIN overlap…" | OK，机制级证据支持 §4.2.p1 |
| §5.3.p2 | evidence | "22 case × 三种模式全 PASS" | OK，回归证据 |
| §6.4.p3 | evidence | "axi_2to3 / axi_4to5 实现跨核 SRAM 直送" | OK，实现级证据 |
| §7.5.p4 | claim | "1.45× 不到线性 2× 的原因可分解为…" | OK，归因 claim |

**结论**：8 段标签 100% 与内容匹配。无 evidence 段在抛 claim、无 claim 段在堆数据等错配。

---

### 节内段落分布合理性

抽样 6 节（覆盖不同章 + 不同段数）：

| 节 | 段数 | 类型分布 | 节内逻辑 | 评价 |
|----|------|---------|---------|------|
| §1.4 Contributions | 7 | setup/claim×5/transition | 总-分-总 | 合理，5 个 contribution claim 各占一段 |
| §2.4 Design Goals | 3 | setup/claim/transition | 短而紧 | 合理，节本身就是 §2.2+§2.3 的归约 |
| §3.4 Streaming/Line-Buffer | 6 | setup/evidence/comparison×3/transition | 1 setup + 1 总举证 + 3 个 prior-art 对位 + 1 收束 | 合理，6 段对应 §3.4 是 narrative B 主对照集 |
| §4.3 Row-Ring Datapath | 5 | claim/evidence×2/claim/transition | claim → 2 evidence → 二级 claim → transition | 合理 |
| §5.4 Compare HW-Reconfig | 4 | claim/evidence/comparison/transition | 标准 claim+evidence+comparison+transition | 合理 |
| §7.6 Compare Prior Art | 7 | claim/evidence×2/claim×2/comparison/transition | 1 总 claim → Tab.7 + Tab.8 → 2 精细论证 → 同器件对照 → transition | 合理，7 段在防御主战场不冗余 |

**结论**：6 节段数 3-7 范围合理；类型 mix 健康（无"全 claim 没 evidence"或"全 evidence 没 claim"）；段间承接自然。

---

### 跨章衔接（三 Writer 边界处）

#### §3.6 → §4.1（Writer B 内部）
- §3.6.p4: "这张表为 §4 详细架构论证铺好坐标系，下章进入 Architecture。"
- §4.1.p1: "FLUX_CNN 顶层由 5 模块 core pipeline + DMA 子系统两层构成…"
- **评价**：§3.6 末段明确 forwarding 到 §4，§4.1 首段开门见山给顶层结构。**衔接顺畅。**

#### §6.5 → §7.1（Writer B → Writer C 边界 — 关键）
- §6.5.p3: "这是工程实践细节而非主贡献，作为 system 完整性收束节出现；下章进入 quantified evaluation。"
- §7.1.p1: "评测平台为 Xilinx XC7K325T-FFG900-2 / Vivado 2023.1 综合…"
- **评价**：§6.5 末段明确 forwarding 到 §7 evaluation，§7.1 首段直接进入 setup 三件套（平台 + EDA + commit 锚点）。**Writer B/C 边界衔接顺畅，无突兀感。**

#### §7.7 → §8.1（Writer C 内部）
- §7.7.p5: "章末诚实标注收束 + future work 已给出，自然过渡到 §8 Conclusion 全文 high-level 总结。"
- §8.1.p1: "FLUX_CNN 论证了在固定 16×16 INT8 阵列 FPGA 加速器上，编译器侧 Ky-fold + S2D 可以替代 hardware-reconfigurable…"
- **评价**：§7.7 末段铺垫 high-level 总结，§8.1 首段直接 conclusion claim，承上良好。**衔接顺畅。**

**三个边界全部 PASS，无断层。**

---

### 风格一致性（三段拼接）

逐项核对：

| 维度 | §1-§3（Writer B 早段） | §4-§6（Writer B 中后段） | §7-§8（Writer C） | 一致性 |
|------|----------------------|---------------------|------------------|--------|
| 主题句长度 | 30-90 字 | 30-100 字 | 40-120 字（§7 量化节因数字密集略长） | **基本一致**，§7 因量化数据天然偏长但合理 |
| 类型标签使用 | setup/claim/evidence/comparison/transition 五类全用 | 五类全用 | 五类全用，未出现"§7-§8 完全不用 transition"现象 | **一致** |
| Contribution 编号 | C1.x / C2.x / C3.x | 同 | 同（如 §7.1 引 C3.3 / §8.1 引 C1.2/C2.1/C2.2/C3.5/C3.7） | **格式统一** |
| 文献条目 | `[Eyeriss ISCA'16]` `[NVDLA]` `[Alwani MICRO'16]` | 同 | `[Liu TNNLS'21]` `[Snowflake ISCAS'17]` `[Tangram ASPLOS'19]` | **会议+年份格式统一** |
| `[CHECK:...]` `[TBD:...]` 用法 | 散布 | 散布 | 散布 | **三段都用，习惯一致** |
| `[依赖: ...]` 字段使用 | 段尾用 | 同 | 同 | **统一** |
| 学术腔调 | 中文为主 + 英文术语 + quantified 数字 | 同 | 同 | **统一** |
| 章首口径声明使用 | §5 章首块引语 | §5 章首块引语 | §7 章首块引语 + §8 章首块引语 | **统一**（Writer C 沿用了 Writer B 在 §5 引入的 blockquote 章首口径声明模式） |

**关键观察**：Writer C 在 §7 / §8 章首沿用了 Writer B 在 §5 章首建立的 `> **章首口径声明**：...` blockquote 模式——这是非常好的风格延续信号。三段拼接在风格上**没有可观察的分裂**。

---

### narrative 主线一致性

追踪 narrative A（compiler-side PE utilization）+ B（system-side row-streaming + multi-core）从 §1 抛 claim → §5 主章展开 → §7 验证：

- §1.1.p2: "narrative A 的核心 motivation"（明确锚 narrative A）
- §1.1.p4: "narrative B streaming 必要性的硬约束"（明确锚 narrative B）
- §1.3.p2: "narrative A 主轴落在编译器侧 PE utilization"
- §1.3.p3: "narrative B 系统侧由 row-ring streaming 与 multi-core W slice 支撑"
- §1.4.p2-p4: 5 条 contribution 与 narrative A/B 一一对应
- §3.2.p1: "hardware-reconfigurable 是 narrative A 的反向 alternative"
- §3.3.p1: "compiler / loop-nest co-design 是 narrative A 的同语言谱系"
- §3.4.p1: "streaming 谱系是 narrative B 的主对照集"
- §3.6.p2: 5 维度 placement 表
- §5.x 全章: narrative A 主章展开（Ky-fold / S2D / 联合触发 / vs HW-reconfig）
- §6.4: narrative B 系统侧 multi-core
- §7.2: "narrative A 核心数据"
- §7.4: 整网 MAC% gap 解释（narrative A 单层 vs 整网口径论证）
- §7.5: "narrative B 系统侧主数据"
- §7.6.p1: "narrative A/B 防御主战场"
- §8.1.p1: "narrative A 主轴 + narrative B 系统侧"双线收束

**结论**：narrative A/B 标签从 §1 → §3 → §5 → §6 → §7 → §8 全程贯穿，无断点，无 narrative C（去中心化握手）反客为主。**主线一致性优秀。**

---

### 避免重复

抽查可能重复的概念：

| 概念 | 首次定义/详述 | 后续段落 | 是否重复 |
|------|-------------|---------|--------|
| Cin<16 各层 PE 利用率（12.5%/25%/50%）| §1.1.p3 简列 + §2.2.p2 详列（Tab.PE-Util-Breakdown）| §5.1.p3 / §7.2.p1 引用 | **不重复**——§1.1 motivation 简提 → §2.2 详述 → §5/§7 数据节引用，承上启下合理 |
| row-ring 容量 ~10 KB（VGA 480×640）| §1.1.p4 / §1.4.p4 / §2.3.p2 / §4.3.p2 | 几处都点 | **轻微复述**——§1 contributions / §2 motivation / §4 architecture 都点是合理的（每章侧重不同），未到信息冗余 |
| Alwani layer-fusion / Kang AoCStream | §1.2.p3 简提 | §2.3.p3 / §3.4.p3-p4 详述 | **不重复**——逐次加深 |
| narrative A/B 标签 | §1.3 设立 | 全文引用 | **不重复**——是有意贯穿 |
| 86.6% MAC% / 593K cycles | §1.4.p6 / §6.1.p3 / §7.4.p1 | 三处出现 | **可接受**——contribution 提一次、§6.1 编译栈节作为编译完整性证据、§7.4 evaluation 主数据节，每处叙事角度不同 |
| Cout<16 不优化 | §2.2.p4 / §5.3.p3 / §7.7.p3 | 三处出现 | **可接受**——motivation 提一次、§5 design rationale 详述、§7.7 局限收束 |

**结论**：无信息冗余 / 概念重复定义。所有重复出现均带"承上启下"或"不同侧重"的合理理由。

---

### 全文统计区状态（次要 metadata）

末尾 line 349-351:
```
## 全文统计

> 填充完所有章节后回填。
```

**建议**：Writer C 因 SSL 中断未回填 metadata。Phase 5 启动前可让 Writer 回填以下数字（次要建议，不影响 Phase 4 PASS）：

- 总段数：约 170 段
- 各章段数：§1×24 / §2×16 / §3×24 / §4×21 / §5×18 / §6×20 / §7×35 / §8×3
- 类型标签分布：setup / claim / evidence / comparison / transition 各类计数
- 平均节内段数：约 4.86 段/节

---

### 通过原因

1. **主题句聚焦度高**：12 段抽样中 11 段单一论点，1 段（§4.3.p1）虽机制密集但功能上是首段总览，prose 阶段拆句即可，不构成缺陷
2. **类型标签零错配**：8 段抽样标签 100% 与内容匹配
3. **节内段落分布健康**：6 节抽样段数 3-7 合理，类型 mix 平衡，无连续 evidence 无 claim 串接
4. **三 Writer 边界无可见分裂**：§3.6→§4.1 / §6.5→§7.1 / §7.7→§8.1 三个关键边界衔接顺畅；Writer C 沿用了 Writer B 在 §5 的 blockquote 章首口径声明模式，风格继承良好
5. **风格一致性优秀**：主题句长度、类型标签使用、contribution 编号格式、文献条目格式、`[CHECK]/[TBD]` 用法、`[依赖:...]` 字段使用全部统一
6. **narrative A/B 主线贯穿**：从 §1.3 设立 → §3.2-§3.4 谱系定位 → §5 主章 → §6.4 系统侧 → §7.2/§7.5 数据节 → §8.1 收束，全程标签清晰
7. **无概念重复定义**：所有重复出现均承担"逐层加深"或"不同侧重"功能

### 给 Writer 的轻微建议（不影响 PASS，Phase 5 写 prose 时参考）

1. **§4.3.p1（机制密集段）**：写 prose 时建议拆为 2-3 句——第一句给 row-ring + forward-pressure 主机制，第二句给 ODMA credit 反向通路，第三句给 sequencer 三阶段并发。整段一句话塞 4 个机制点 prose 阶段读者会喘不过气。
2. **全文统计区回填**：line 349-351 的 `> 填充完所有章节后回填。` 占位可在 Phase 5 启动前由 Writer 补 5 行 metadata（总段数 / 各章段数 / 类型标签分布），不影响 Phase 4 评审通过。
3. **§6.5.p3 transition 句过紧**：`这是工程实践细节而非主贡献，作为 system 完整性收束节出现；下章进入 quantified evaluation。`——分号串两个独立 claim，prose 阶段可拆为两句让节奏松一点（"...收束节出现。下章进入..."）。

### 提示给其他评审（不影响本 phase 判定）

- **§7.5.p1 / §7.4.p1 数字稠密**：包括 9057 cycles / 8808 cycles / 3833 cycles / 5569 cycles / 593K cycles / 86.6% MAC% / 5.95 ms / 8.69 ms / 168 fps / 115 fps —— 这些数字真伪由 **tech 评审**复核，写作侧只确认了形式一致和口径标注（"target vs Fmax"双 operating point）合理。
- **§5.2.p3 Sub-Pixel CVPR'16 引用**：S2D 算法侧出处、加速器领域引用谱系完整性由 **novelty 评审**复核。

---
