# 图 2.4 NUMA 架构拓扑示意
# Figure 2.4 NUMA topology schematic

## 在论文中的角色
- 首次引入：§2.5 非一致性内存访问架构（paper.md L251-L255）
- 引用位置：§2.5
- 论证作用：作为体系结构基础概念图，用通用 OS / 数据库领域的 NUMA 原型（2-4 个 NUMA 节点 + QPI/UPI 互联 + 本地远端延迟对照）建立读者对"本地访问快、远端访问慢"的直观印象，为 §4.11 多核加速器中跨核 SRAM 直送策略提供理论锚点。**此图不画 FLUX_CNN 跨核 SRAM 拓扑，那是 §4.11 的事**。

## 图类型
体系结构拓扑示意图（多个 NUMA 节点 + 内部本地内存 + 互联网络 + 延迟标注）。

## 设计要素

### 必含元素
1. **N 个 NUMA 节点**（N = 4，对称布局占画面四角）：每节点是一个圆角矩形，内含：
   - 一个 CPU 方框（标"CPU Socket i"）
   - 一个本地内存方框（标"Local Memory / DRAM"）
   - CPU ↔ Local Memory 间一条粗实线（标"本地访问 ≈ 100 ns"）
2. **跨节点互联（QPI / UPI）**：四个节点之间用细实线连成"全连接"或"环形"拓扑，标"QPI / UPI Interconnect"。
3. **延迟对照标注**：在某条跨节点连线旁加粗显示"远端访问 ≈ 250 ns"。
4. **示例数据访问轨迹**：用红色虚线箭头从 Node 0 的 CPU 射出，经过互联网络到达 Node 2 的本地内存，标"跨节点访问示例"。
5. **图例**：右下角"● CPU / ■ Local Memory / ── 本地访问 / ─ ─ 跨节点访问"。
6. **底部小字总结**：本地访问与远端访问延迟比约 1:2.5，为多核加速器设计提供本地优先访问的设计动机。

### 标注要求
- 每节点编号 Node 0 / 1 / 2 / 3
- 本地访问与跨节点访问延迟分别用不同字号 / 颜色
- 互联网络可标"QPI (Intel) / UPI (Intel) / Infinity Fabric (AMD)"
- 底部一行小字"NUMA: 本地访问延迟 ≪ 远端访问延迟"

### 视觉层次
- 主角：4 个 NUMA 节点 + 互联网络
- 配角：延迟标注与跨节点访问轨迹（红色虚线）
- 背景：互联网络的全连接线条（淡灰色）

## 数据来源
- paper.md §2.5 L253（Hennessy/Patterson 教科书 [5] 引用）
- 文献 [5] Hennessy & Patterson 计算机体系结构
- 公开 NUMA 教学资源（Intel QPI、AMD Infinity Fabric 延迟数据）
- **本图为通用 OS / 数据库领域 NUMA 概念图**，不引入 FLUX_CNN 跨核 SRAM 内容

## ASCII 示意稿

```
   ┌────────────────────┐   QPI/UPI    ┌────────────────────┐
   │ NUMA Node 0        │◀────────────▶│ NUMA Node 1        │
   │                    │   远端访问    │                    │
   │  ┌───┐──本地100ns──│   ≈ 250 ns   │──本地100ns──┌───┐   │
   │  │CPU│         ┌───┤              ├───┐         │CPU│   │
   │  └───┘         │Mem│              │Mem│         └───┘   │
   │                └───┘              └───┘                 │
   └─────────┬──────────┘              └─────────┬──────────┘
             │                                   │
             │ QPI/UPI                  QPI/UPI  │
             │                                   │
   ┌─────────┴──────────┐              ┌─────────┴──────────┐
   │ NUMA Node 2        │◀────────────▶│ NUMA Node 3        │
   │                    │              │                    │
   │  ┌───┐──本地100ns──│              │──本地100ns──┌───┐   │
   │  │CPU│         ┌───┤              ├───┐         │CPU│   │
   │  └───┘         │Mem│              │Mem│         └───┘   │
   │                └───┘              └───┘                 │
   └────────────────────┘              └────────────────────┘
                          ─ ─ ─ ─ ─ ─ ─▶
                          跨节点访问示例
                         (Node 0 CPU → Node 2 Mem)

   图例: ● CPU  ■ Local Memory  ── 本地访问  ─ ─ 跨节点访问
   注: 本地访问延迟 ≈ 100 ns，远端访问延迟 ≈ 250 ns
```

## 与正文的一致性检查
- [x] §2.5 "处理核访问与之物理邻近的本地存储节点延迟较低" — 图中本地访问 100 ns
- [x] §2.5 "访问其他核所属的远端存储节点则需穿越互连网络，延迟显著上升" — 图中远端访问 250 ns
- [x] §2.5 "现代多核处理器与片上系统普遍采用分布式本地存储加片上互连的 NUMA 结构" — 图中 4 节点 + QPI 全连接
- [x] §2.5 "Hennessy 和 Patterson 在体系结构教科书中将 NUMA 与 SMP 相对照" — 图作为教科书概念图呈现
- [x] §2.5 "Eyeriss v2 ... 簇内访问延迟约 1 拍，簇间数据搬运需多拍" — 此结论的通用 NUMA 锚点

## 不确定项
- [TBD: 100 ns / 250 ns 延迟数据是否需精确文献支撑] — 倾向用"约"作教学口径，避免精确数字争议
- [TBD: 是否画 4 节点全连接还是环形] — 倾向全连接，与现代 Intel QPI / AMD Infinity Fabric 拓扑一致

## image 生成提示词

### 中文版

科研论文配图，**NUMA（非一致性内存访问）架构拓扑示意图**，IEEE 期刊配色风格，白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1:1（正方形），适合 A4 单栏插图。**

**版式（4 个对称 NUMA 节点 + 中央互联）**：
- **四角放 4 个 NUMA 节点**：画面四角各放一个大圆角矩形（淡蓝色 CMYK 30/10/0/0 填充、深灰 #404040 实线边框），尺寸约占画面 35%×35%。每节点内部水平排列两个子方框：
  - 左侧子方框：CPU Socket（淡黄色 CMYK 0/10/30/0 填充），标"CPU Socket *i*"（思源黑体 9 pt 加粗）。
  - 右侧子方框：Local Memory（淡灰色 CMYK 0/0/0/10 填充），标"Local Memory / DRAM"（思源黑体 9 pt）。
  - CPU 与 Local Memory 之间用一条深绿色 #2e7d32 粗实线连接（线宽 2 pt），中间标小字"本地访问 ≈ 100 ns / local access"（Times New Roman 8 pt）。
  - 节点顶部标"NUMA Node 0/1/2/3"（思源黑体 10 pt 加粗）。
- **跨节点互联**：4 个节点之间用细灰色 #404040 实线连成全连接拓扑（共 6 条线：上下左右 + 两条对角线，对角线为虚线以区分）。其中一条边（例如 Node 0 ↔ Node 2 对角线）用粗红色 #c00000 虚线表示"远端访问示例"，旁边标小字"远端访问 ≈ 250 ns / remote access"（Times New Roman 8 pt）。
- **互联标题**：画面正中央放一个小框（白底、深灰边框），标"QPI / UPI / Infinity Fabric Interconnect"（Times New Roman 9 pt 斜体）。
- **图例**：右下角放一个小矩形框（淡灰色填充），内含 4 行小字"● CPU""■ Local Memory""── 本地访问""─ ─ 跨节点访问"，Times New Roman 8 pt。
- **底部注释**：画面底部居中一行小字（Times New Roman 9 pt 斜体）"NUMA: 本地访问延迟 ≪ 远端访问延迟（约 1:2.5）"，再加一行英文"NUMA: local access latency ≪ remote access latency (~1 : 2.5)"。

**字体**：所有英文 Times New Roman 10 pt，标题加粗；中文思源黑体 10 pt。**严肃学术风格，IEEE 期刊配色，禁止卡通**：禁止手写体、彩色渐变、阴影、3D 立体、图标、emoji、卡通元素。整体保持工科论文严谨风格。本图为教科书风格的 NUMA 概念图，**不引入 FLUX_CNN 跨核 SRAM 直送**，与 §4.11 多核扩展图保持区分。

### English version

Scientific paper figure, **NUMA (Non-Uniform Memory Access) topology schematic**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **Overall aspect ratio approximately 1:1 (square), suitable for a single-column A4 figure.**

**Layout (4 symmetric NUMA nodes + central interconnect)**:
- **Four NUMA nodes at the corners**: At the canvas's four corners place large rounded rectangles (light-blue CMYK 30/10/0/0 fill, dark-gray #404040 solid border), each about 35%×35%. Inside each node, horizontally arrange two sub-boxes:
  - Left sub-box: CPU Socket (light-yellow CMYK 0/10/30/0 fill), label "CPU Socket *i*" (Source Han Sans 9 pt bold).
  - Right sub-box: Local Memory (light-gray CMYK 0/0/0/10 fill), label "Local Memory / DRAM" (Source Han Sans 9 pt).
  - Between CPU and Local Memory draw a deep-green #2e7d32 thick solid line (2 pt), labeled "local access ≈ 100 ns" (Times New Roman 8 pt).
  - Above each node: "NUMA Node 0/1/2/3" title (Source Han Sans 10 pt bold).
- **Cross-node interconnect**: Connect the 4 nodes with thin dark-gray #404040 solid lines forming a full mesh (6 edges total: 4 sides + 2 diagonals; diagonals dashed to distinguish). Highlight one edge (e.g., Node 0 ↔ Node 2 diagonal) with a thick red #c00000 dashed line representing a "remote access example", labeled "remote access ≈ 250 ns" (Times New Roman 8 pt).
- **Interconnect title**: At the canvas center place a small white-fill box with dark-gray border, label "QPI / UPI / Infinity Fabric Interconnect" (Times New Roman 9 pt italic).
- **Legend**: Lower-right small rectangle (light-gray fill) with 4 lines: "● CPU", "■ Local Memory", "── local access", "─ ─ remote access" (Times New Roman 8 pt).
- **Bottom caption**: At the bottom center, two italic lines: "NUMA: local access latency ≪ remote access latency (~1 : 2.5)" and a Chinese counterpart above it.

**Typography**: All English in Times New Roman 10 pt with bold titles; Chinese in Source Han Sans 10 pt. **Serious academic style, IEEE palette, no cartoon**: strictly forbid handwritten fonts, color gradients, drop shadows, 3D effects, icons, emoji, or cartoon elements. This figure is a textbook-style NUMA concept diagram and **does not introduce FLUX_CNN's cross-core SRAM push topology**, which is reserved for §4.11.
