# 图 1.3 FLUX_CNN 五大创新点思维导图
# Figure 1.3 FLUX_CNN five-pillar contributions mindmap

## 在论文中的角色
- 首次引入：§1.3 本论文的主要研究内容（paper.md L147-L163）
- 引用位置：§1.3 第一段总览，第二章理论基础前
- 论证作用：把 §1.3 罗列的"第一、第二、第三、第四、第五"五项研究内容用一张思维导图视觉化，让读者在阅读后续大章节前对论文整体贡献结构形成完整印象。该图同时作为"五大创新点"在结论 §6.2 的呼应锚点。

## 图类型
中心辐射式思维导图（中央节点 + 5 辐射分支 + 各分支子要点）。

## 设计要素

### 必含元素
1. **中央节点**：圆角矩形或圆形，标"FLUX_CNN 端侧分层调度加速器"（思源黑体 12 pt 加粗）。
2. **五条辐射分支**（从中心向外，按时钟方向 12 / 2 / 5 / 7 / 10 点位置放置）：
   - **分支 1（12 点）— 分层调度架构**：核内 cfg_regs + 6 层 FSM；核外 Task Descriptor 链表；2 次写 vs 50 次/层。子要点 3-4 项。
   - **分支 2（2 点）— 任意形状固定阵列**：16×16 INT8 MAC + 三级 RF（WRF/ARF/PARF）；零乘除法地址生成；任意 K×K / stride / Cin/Cout 切片；K² > WRF 时 round 分块。
   - **分支 3（5 点）— Ky-fold + S2D 编译器优化**：小 Cin 场景用 Ky-fold；大 stride 场景用 S2D；零 RTL 改动；DDR 友好（S2D 无复制）。
   - **分支 4（7 点）— W 切片 + SMC 多核扩展**：N=1/2/4 三档；跨核 SRAM 直送；NUMA 本地访问；ResNet11 整网 2.51× sim 加速。
   - **分支 5（10 点）— DSP 跨列复用映射**：DSP48E1 25×18 乘法器位宽冗余；A 端打包相邻列权重；B 端共享激活；硬件无关方案。
3. **辐射线**：从中央节点到每分支主标题用粗实线连接；每分支主标题向其子要点用细实线发散。
4. **辐射分支颜色编码**：5 分支用 5 种淡色背景区分（淡蓝 / 淡黄 / 淡绿 / 淡紫 / 淡灰）。

### 标注要求
- 每分支主标题前加章节锚点（如"§4.7 / §4.11 / §4.8-4.9 / §4.11 / §4.12"）
- 每分支子要点字号略小（Times New Roman 9 pt）
- 中央节点下方一行小字"五大创新点 (Five-pillar contributions)"

### 视觉层次
- 主角：中央节点
- 配角：5 条辐射分支与主标题
- 背景：各分支子要点小框（淡色填充）

## 数据来源
- paper.md §1.3 L149-L159（第一至第五点研究内容）
- contributions.md §一、二、三、四、五贡献清单
- paper.md §6.2 创新点（与本图分支对应）

## ASCII 示意稿

```
                  §4.7 / §3.4
              ┌─────────────────┐
              │ 1. 分层调度架构 │
              │ • 核内 cfg+FSM  │
              │ • 核外 TaskDesc │
              │ • 2 vs 50 写次  │
              └────────┬────────┘
                       │
   §4.12              │            §4.3 / §4.4
   ┌──────────┐       │       ┌──────────────────┐
   │5. DSP 跨 │       │       │ 2. 任意形状      │
   │  列复用  │◀──────┼──────▶│   固定阵列       │
   │ • 2 INT8 │       │       │ • 16×16 INT8     │
   │   /DSP   │   ┌───▼────┐  │ • WRF/ARF/PARF=32│
   │ • 99% UR │   │FLUX_CNN│  │ • 任意 K/s/C    │
   └──────────┘   │ 五大创新│  │ • round 分块     │
                  │  点    │  └──────────────────┘
                  └────┬───┘
                       │
   §4.11               │            §4.8-4.9
   ┌────────────┐      │       ┌─────────────────┐
   │4. 多核 W   │◀─────┴──────▶│ 3. Ky-fold + S2D│
   │  切片+SMC  │              │ • 小 Cin / 大s  │
   │ • N=1/2/4  │              │ • 零 RTL 改动   │
   │ • XSRAM直送│              │ • DDR 友好      │
   │ • NUMA     │              │ • 1.87×@Patch   │
   │ • 2.51×    │              └─────────────────┘
   └────────────┘
```

## 与正文的一致性检查
- [x] §1.3 "第一，提出微观寄存器配置 + 宏观指令流调度的分层架构方案" — 分支 1
- [x] §1.3 "第二，设计 16×16 INT8 MAC 阵列与三级寄存器堆" — 分支 2
- [x] §1.3 "第三，针对小通道场景下 PE 阵列利用率受限的问题 ... Ky-fold 与 S2D" — 分支 3
- [x] §1.3 "第四，针对端侧多核扩展中跨核数据搬运... W 维切片调度 ... 跨核 SRAM 直送" — 分支 4
- [x] §1.3 "第五，针对 FPGA 验证平台上 DSP 资源的有效利用 ... DSP48E1 块 25×18 乘法器" — 分支 5
- [x] §1.3 "第六、第七" 是工具链与综合评估，作为支撑性工作不纳入"创新点"主图

## 不确定项
- [TBD: 是否合并第六、第七为额外辐射分支] — 倾向不合并，保持"五大创新点"的清晰陈述
- [TBD: ResNet11 加速数字 2.51× 还是 3.13×] — 按 paper.md §5.5.1 主配置数据，最终以表 5.4 为准；论文修订后图同步

## image 生成提示词

### 中文版

科研论文配图，**FLUX_CNN 五大创新点思维导图**，IEEE 期刊配色风格，白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1:1（正方形），适合 A4 单栏插图。**

**版式（中心辐射式，五条分支按时钟方向均匀分布）**：
- **中央节点**：画面正中央绘制一个圆角矩形（淡黄色 CMYK 0/10/30/0 填充、深灰 #404040 实线边框），尺寸约占画面 22%×18%。内部上方一行"FLUX_CNN"（思源黑体 12 pt 加粗），下方一行"端侧分层调度加速器"（思源黑体 10 pt），再下方一行小字"五大创新点 / Five-pillar contributions"（Times New Roman 9 pt 斜体）。
- **五条辐射分支**：从中央节点向外辐射，分支末端为圆角矩形子框，按时钟方向 12 点、2 点、5 点、7 点、10 点均匀分布。每子框尺寸约 25%×18%。
  - **12 点 — 分支 1 分层调度架构**：淡蓝 CMYK 30/10/0/0 填充。内部三行：标题"1. 分层调度架构 (§3.4 / §4.7)"（思源黑体 10 pt 加粗）；子要点"核内 cfg_regs + 6 级 FSM""核外 Task Descriptor 链表""控制写次 50→4 / 层"。
  - **2 点 — 分支 2 任意形状固定阵列**：淡黄 CMYK 0/10/30/0 填充。三行：标题"2. 固定 16×16 阵列 (§3.3 / §4.3)"；子要点"16×16 INT8 MAC，三级 RF""零乘除法地址生成""任意 K × stride × Cin/Cout"。
  - **5 点 — 分支 3 Ky-fold + S2D 编译器优化**：淡绿（CMYK 30/0/30/0）填充。三行：标题"3. Ky-fold + S2D (§4.8-4.9)"；子要点"小 Cin 用 Ky-fold""大 stride 用 S2D""零 RTL 改动，DDR 友好"。
  - **7 点 — 分支 4 多核 W 切片 + SMC**：淡紫（CMYK 10/20/0/0）填充。三行：标题"4. 多核 W 切片 (§4.11)"；子要点"N=1/2/4 三档""跨核 SRAM 直送，NUMA""ResNet11 加速 2.5-3.1×"。
  - **10 点 — 分支 5 DSP 跨列复用**：淡灰 CMYK 0/0/0/10 填充。三行：标题"5. DSP 跨列复用 (§4.12)"；子要点"DSP48E1 25×18 位宽冗余""相邻列共享激活打包""DSP 利用率 99%"。
- **辐射连线**：从中央节点到每分支主框用深蓝色 #1f4e79 粗实线（线宽 1.5 pt）连接；可在线中段加一个细小箭头表示辐射方向。
- **图例（可选）**：右下角小字一行"颜色编码：架构 / 阵列 / 编译器 / 多核 / DSP 映射"。

**字体**：所有英文 Times New Roman 10 pt，标题加粗；中文思源黑体 10 pt。**严肃学术风格，IEEE 期刊配色，禁止卡通**：禁止手写体、彩色渐变、阴影、3D 立体、图标、emoji、卡通元素。整体保持工科论文严谨风格。

### English version

Scientific paper figure, **FLUX_CNN five-pillar contributions mindmap**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **Overall aspect ratio approximately 1:1 (square), suitable for a single-column A4 figure.**

**Layout (radial mindmap with five branches at clock positions)**:
- **Central node**: Place a rounded rectangle (light-yellow CMYK 0/10/30/0 fill, dark-gray #404040 border) at the canvas center, sized about 22%×18%. Inside: line 1 "FLUX_CNN" (Source Han Sans 12 pt bold); line 2 "edge hierarchical-scheduling accelerator" (Source Han Sans 10 pt); line 3 small italic "Five-pillar contributions" (Times New Roman 9 pt italic).
- **Five radial branches**: Radiate from the central node to rounded-rectangle child boxes at clock positions 12, 2, 5, 7, and 10. Each child box ~25%×18%.
  - **12 o'clock — Branch 1 Hierarchical scheduling**: Light-blue CMYK 30/10/0/0 fill. Three lines: title "1. Hierarchical scheduling (§3.4 / §4.7)" (Source Han Sans 10 pt bold); bullets "intra-core cfg_regs + 6-level FSM", "inter-core Task Descriptor list", "host writes 50→4 per layer".
  - **2 o'clock — Branch 2 Fixed shape-agnostic array**: Light-yellow fill. Three lines: title "2. Fixed 16×16 array (§3.3 / §4.3)"; bullets "16×16 INT8 MAC + 3-level RF", "zero mul/div address gen", "arbitrary K, stride, Cin/Cout".
  - **5 o'clock — Branch 3 Ky-fold + S2D compiler**: Light-green CMYK 30/0/30/0 fill. Three lines: title "3. Ky-fold + S2D (§4.8-4.9)"; bullets "small Cin → Ky-fold", "large stride → S2D", "zero RTL change, DDR-friendly".
  - **7 o'clock — Branch 4 Multi-core W-slice + SMC**: Light-purple CMYK 10/20/0/0 fill. Three lines: title "4. Multi-core W-slice (§4.11)"; bullets "N=1/2/4 configurations", "cross-core SRAM push, NUMA", "ResNet11 speedup 2.5-3.1×".
  - **10 o'clock — Branch 5 DSP cross-column reuse**: Light-gray CMYK 0/0/0/10 fill. Three lines: title "5. DSP cross-column reuse (§4.12)"; bullets "DSP48E1 25×18 bitwidth slack", "shared activation packing", "DSP utilization 99%".
- **Radial lines**: From the central node to each branch box, draw thick (1.5 pt) deep-blue #1f4e79 solid lines, with a small arrowhead at the box end.
- **Legend (optional)**: A small caption at the lower-right "Color code: architecture / array / compiler / multi-core / DSP mapping".

**Typography**: All English in Times New Roman 10 pt with bold titles; Chinese in Source Han Sans 10 pt. **Serious academic style, IEEE palette, no cartoon**: strictly forbid handwritten fonts, color gradients, drop shadows, 3D effects, icons, emoji, or cartoon elements. Maintain rigorous engineering-paper aesthetics throughout.
