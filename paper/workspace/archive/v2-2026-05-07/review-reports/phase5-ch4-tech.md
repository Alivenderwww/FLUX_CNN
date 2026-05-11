# 技术评审报告 Phase 5 §4

## 第 1 次评审

### 判定：PASS

评审范围：paper.md line 249-337（§4 硬件实现与编译器优化，全文最长章 32 段 / 9100 字）。

---

### 抽样验证记录（≥3 处）

| # | 位置 | 摘录 | 验证结论 |
|---|------|------|---------|
| 1 | §4.3 line 281 Ky 折叠数学 | "ky = g · kyper + ky_local 分组...cin_fake = groups_y · Cin" | 与 docs/pe-fold.md §1 数学公式完全一致：I'/W' 定义、groups_y/kyper/cin_fake/pad_ky 派生公式逐一对得上 |
| 2 | §4.3 line 285 S2D 数学 | "ky = stride·ky' + a, kx = stride·kx' + b, p = a · stride + b...K_new = ⌈K / stride⌉、Cin_new = stride² · Cin" | 与 docs/pe-fold.md §2 完全一致；pad_waste 公式 (K_new² · stride² − K²) / (K_new² · stride²) 也一致 |
| 3 | §4.3 line 295 Liu 对位论证 | "Arria 10 GX1150 大型器件...PE 利用率约 97%；本工作 XC7K325T 中型器件...约 86.6%...资源量级约为 5~7 倍" | 论证含糊度合格——明确"不同器件规模下的可比数字"+ 数量级量化（约 5~7 倍）+ 86.6% 已挂 [CHECK] 待 model_analysis.md 与 STATUS.md 交叉核 |
| 4 | §4.2 line 263 / 267 关键参数 | "IFB 容量 8192 word、BUS_DATA_W = 128、OFB 容量 2048 word、PARF = 32 深度、NUM_COL = 16" | 与 CLAUDE.md "项目总览/核心参数" 段完全一致 |
| 5 | §4.5 line 319 多核地址映射 | "DDR 0x0000_0000—0x7FFF_FFFF + Core[i] IFB 0x8000_0000 + i × 0x1000_0000" | 与 CLAUDE.md "项目总览/多核地址映射" 完全一致 |

---

### narrative A 主战场专项核查（§4.3）

| 检查项 | 结论 |
|--------|------|
| Liu 86.6% vs 97% "不同器件规模可比数字"措辞 | ✅ line 295 完整论证：器件量级差异 + 路线选择差异 + 不声称"更优" |
| C2.1 Ky 折叠用 "据已知文献..." 不用 "first to..." | ✅ line 283 措辞合规："据已知文献，加速器领域尚无直接对应的纯编译器侧...方案" |
| C2.2 S2D 加速器领域引用谱系 [CHECK] | ✅ line 287 末已标 "[CHECK-1.2.4: 加速器领域 S2D 引用谱系待补查]"；同时点明超分辨率 Pixel-Shuffle / Sub-pixel 来源 |
| Cout < 16 disadvantage 不软化 | ✅ line 291 "这两处 disadvantage 不软化、不掩饰，作为本工作'硬件最简'取舍的诚实代价显式陈述" |
| vs Hardware-Reconfigurable 4 维度对位（MAERI / Eyeriss-v2 / Tangram） | ✅ line 293 完整 4 维度：硬件复杂度 / 编译器复杂度 / 内存代价 / 适用场景 |

---

### contribution 编号引用核查

| 贡献编号 | 出现位置 | 状态 |
|---------|---------|------|
| C1.1 OS+列广播 | §4.2 line 265 | ✅ |
| C1.2 5 模块去中心化 | §4.2 line 261 | ✅ |
| C1.3 行级流式行环 | §4.2 line 263 | ✅ |
| C1.4 PARF | §4.2 line 267 | ✅ |
| C1.5 SDP | §4.2 line 269 | ✅ |
| C1.6 7 层循环嵌套 | §4.2 line 271 | ✅ |
| C2.1 Ky 折叠 | §4.3 line 281 | ✅ |
| C2.2 S2D | §4.3 line 285 / 287 | ✅ |
| C2.3 联合触发 | §4.3 line 289 | ✅ |
| C2.4 PyTorch→ISA | §4.4 line 303 | ✅ |
| C2.5 链式 CASES | §4.4 line 305 | ✅ |
| C3.1 AXI-DMA 集成 | §4.4 line 307 | ✅ |
| C3.2 CFG_WRITE descriptor | §4.4 line 309 | ✅ |
| C3.3 双口 cfg_regs | §4.4 line 309 | ✅ |
| C3.4 多核 W 切片 | §4.5 line 319 / 321 / 323 | ✅ |
| C3.5 单源参数 | §4.5 line 325 | ✅ |
| C3.6 profile 报告 | §4.4 line 311 | ✅ |

§4.6 小结（line 333）按"6+3+5+2+1=17 项归类"对 16 条贡献完整收口（C1×6 / C2.1-C2.3 ×3 / C2.4-C2.5+C3.1-C3.3 ×5 / C3.4+C3.5 ×2 / C3.6 ×1），归类口径与 contributions.md 自洽。

---

### 数据真实性抽样（4 处关键数字）

| # | 数字 | 位置 | 来源标注 | 结论 |
|---|------|------|---------|------|
| 1 | 整网 PE 利用率 86.6% | line 295 | [CHECK: 整网利用率 86.6% 数字与 model_analysis.md 与 STATUS.md 交叉核 + 取数语义对齐] | ✅ 已显式 [CHECK] |
| 2 | N=1/2/4 cycles：596K → 354K → 220,824 | line 323 / 327 | line 323 引 STATUS.md §2.8；line 327 引 STATUS.md §2 + commit 5fe16b2 + [CHECK-Q5] | ✅ 关键 220,824 cy / 453 fps / DDR busy 84.7% 已挂 [CHECK-Q5] |
| 3 | Patch 层 S2D 5.05× 加速（654,404 → 129,594 cycles） | line 287 | <!-- 来自 STATUS.md §2.8 / contributions.md C2.2 --> | ✅ 来源标注完整 |
| 4 | 单核 Vivado 综合 Fmax 68.4 MHz | line 269 | <!-- 来自 STATUS.md §1 单核综合表 --> | ✅ 来源标注完整；并诚实陈述 critical path 在 SDP 量化组合链 |

---

### claim 强度核查

- 未发现 "first to..." / "novel architecture" 类违禁措辞
- line 277 "本论文 novelty 最强的主战场"措辞略强，但与 paragraph-skeleton.md §4.3 段 1 setup 原句一致，非新增 over-claim
- 多核加速比 1.68× 不掩饰，明确"偏离线性 4×"+ 根因（DDR 带宽）+ ROI 决策不投多 DDR 板（line 327）

### 诚实陈述核查

| limitation | 位置 | 结论 |
|-----------|------|------|
| Fmax 仅 68 MHz（critical path 在 SDP） | line 269 | ✅ 显式 + 指向 §6.3 future work（流水切分） |
| Cout < 16 列空转无法回收 | line 291 | ✅ 不软化，明确"当前架构的明确取舍" |
| Ky-fold IFB groups_y 倍膨胀 | line 283 / 291 | ✅ 显式代价陈述 |
| S2D 在 K 不被 stride 整除时的 pad_waste | line 287 | ✅ 给出 pad_waste 公式 + K=8 stride=2 时 pad_waste=0 的具体数 |
| 1-DDR 通道带宽是 N=4 加速比下降根因 | line 327 | ✅ DDR busy 84.7% 显式 |

---

### 段落骨架对齐核查

- 骨架要求：§4.1=2 / §4.2=8 / §4.3=8 / §4.4=6 / §4.5=6 / §4.6=2 = 32 段
- 实际段数：§4.1=2 / §4.2=8 / §4.3 ≥11 / §4.4=6 / §4.5=6 / §4.6=2
- §4.3 段数膨胀（8 → 约 11 段）：因 narrative A 主战场内容密度高，新增"硬件可重构 4 维度对位"+"Liu 对位"+"S2D vs Ky-fold 收益对比"等独立段；**属于内容深化而非偏题，不构成 FAIL**
- 关键依赖标记（Fig.4.1—Fig.4.11 / Tab.4.1—Tab.4.2）在对应段落均有 [依赖] 标注

---

### 回归性核查

- §4 未引入骨架未提的新 prior art（仅引用 MAERI / Eyeriss-v2 / Tangram / Liu Full-Stack TNNLS'21 / Pixel-Shuffle / Sub-pixel，均在骨架 §1.2 / §3.4 范围内）
- §4 数学推导符号体系与 §2.2 / docs/pe-fold.md 一致（N/Cout/Cin/H/W/Ky/Kx 7 层嵌套 + g/kyper/ky_local + a/b/p）
- §4 跨章引用（§3.4 / §3.5 / §5.4 / §5.5 / §5.6 / §6.2 / §6.3）的去向章节 paragraph-skeleton.md 中均有对应位

---

### 总结

§4 全章在 narrative A 主战场（§4.3）的论证强度、数学严谨度、参数与 docs/pe-fold.md 一致性、Liu 对位说理、Cout<16 诚实陈述、与硬件可重构路线 4 维度对位、C 编号引用完整性、关键数字 [CHECK] 标记、claim 强度控制全部合规。所有抽样数字均能追溯到 STATUS.md / contributions.md / model_analysis.md / docs/pe-fold.md / CLAUDE.md，无编造、无越界引用、无 over-claim "first to..."。

§4.3 段数较骨架略有膨胀（8 → ≥11 段），属内容深化而非偏题，按"灵魂层>格式层"原则不影响判定。

判定：**PASS**。
