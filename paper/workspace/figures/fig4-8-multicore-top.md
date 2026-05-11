# 图 4.8 多核扩展层结构图
# Figure 4.8 Multi-core extension layer structure

## 在论文中的角色
- 首次引入：§4.11 多核 W 切片扩展（paper.md L426 / L430）
- 引用位置：paper.md L426 / L430 / §4.12 本章小结
- 论证作用：展示多核扩展层（multicore_top）在加速核之上的封装：N=4 个加速核（按 W 维切片并行）+ AXI 互连交换（基于 Xilinx SmartConnect IP）+ AXI-Lite 一对多分发模块（axi_lite_1to4）+ 跨核 SRAM 直送（odma_ctrl 直写消费者核 IFB 物理地址）+ halo 列重叠输入。强调"单核 RTL 完全不变，扩展通过外层完成"的设计哲学。读者应理解 W 维切片分布、halo 列冗余、跨核 SRAM 直送优化绕开 DDR 双向带宽的关键收益。

## 图类型
矩阵式架构图：4 个加速核以 **2×2 矩阵布局**（不用 1×4 横排，避免过宽）；周围标注 W 切片 halo 重叠 + 跨核 SRAM 直送箭头 + 顶部 AXI 互连 + AXI-Lite 分发。整体接近 1:1 正方形。

## 设计要素

### 必含元素

#### 顶部 — 控制接口与互连
1. **AXI-Lite ← Control Core（最顶部）**：细线进入 axi_lite_1to4 顶部端口。
2. **axi_lite_1to4（顶部中央小方块）**：标 "axi_lite_1to4 (1-to-4 distributor)"，4 路输出分别送 4 个核的 AXI-Lite 端口。
3. **AXI Interconnect（顶部覆盖整图宽度）**：粗横线表示 "AXI Interconnect (Xilinx SmartConnect)" + 顶部接 "DDR (off-chip)" 端口（粗箭头朝上）。

#### 中央主体 — 2×2 N=4 加速核矩阵
4. **Core[0]（左上）**：方块标 "Core[0] (acc core, sub_W slice 0)"，内部小字 "IFB | core | OFB"，IFB 起始地址标 "IFB base = 0x80000000"。
5. **Core[1]（右上）**：方块标 "Core[1] (acc core, sub_W slice 1)"，IFB 标 "IFB base = 0x90000000"。
6. **Core[2]（左下）**：方块标 "Core[2] (acc core, sub_W slice 2)"，IFB 标 "IFB base = 0xA0000000"。
7. **Core[3]（右下）**：方块标 "Core[3] (acc core, sub_W slice 3)"，IFB 标 "IFB base = 0xB0000000"。

#### W 维切片调度示意
8. **W 切片条（最右侧侧栏）**：画一根代表 W_in 的水平条，分 4 段分别标 "sub_W₀ | sub_W₁ | sub_W₂ | sub_W₃"，相邻段之间画**重叠灰色区域**标 "halo (K=3, stride=1: 2 cols overlap)"。
9. **halo 重叠注**：在 W 切片条下方小字 "halo: each core stores overlap cols redundantly — no cross-core sync needed"。

#### 跨核 SRAM 直送
10. **跨核 push 箭头**：在 2×2 矩阵之间，画 4 根**深红色粗实线箭头**示意：
    - Core[0].odma → Core[1].IFB（横向，标 "x-core push (next layer)"）
    - Core[1].odma → Core[2].IFB（弯折，标 "x-core push"）
    - Core[2].odma → Core[3].IFB（横向，标 "x-core push"）
    - Core[3].odma → Core[0].IFB（弯折回左上，标 "x-core push"）
    （实际生产者→消费者映射在编译器侧由 layer dependency 决定，本图示意一种典型环式模式）
11. **ifb_axi_slave 端口**：每个 Core 的 IFB 旁边画一个小三角形端口标 "ifb_axi_slave (AXI Interconnect mount)"，强调"消费者核 IFB 通过 AXI Interconnect 接受写入"。

#### 层边界握手
12. **done_sticky 汇聚**：4 个 Core 的右下角 done_sticky 信号汇聚成一根细线，标 "done_sticky[0..3] → all-done → next layer enable"，回到顶部 AXI Interconnect 旁的小逻辑门符号 "AND"。

#### 关键标注（左下角侧栏）
13. **关键设计说明**：
    - "single-core RTL identical at N=1/2/4 — extension at top layer only"
    - "halo: computed redundancy, no cross-core sync"
    - "x-core SRAM push: avoids DDR round-trip between layers"
    - "address map: IFB base = 0x80000000 + i × 0x10000000"

### 标注要求
- N=4（已综合通过 N=1/2/4 三种配置）
- 多核地址映射：`0x80000000 + i × 0x10000000`（与 CLAUDE.md 一致）
- halo 列数：K=3 stride=1 时 2 列（与 §4.11 一致）
- AXI 互连：基于 Xilinx SmartConnect IP
- AXI-Lite 分发：1 → 4

### 视觉层次
- 主角：2×2 N=4 加速核矩阵 + 跨核 SRAM 直送红色箭头
- 配角：AXI Interconnect 顶部横线、axi_lite_1to4 分发、W 切片条
- 背景：地址映射小字、halo 注释

## 数据来源
- paper.md §4.11
- docs/modules/multicore_top.md（如存在）
- contributions.md（多核 W 切片扩展 + 跨核 SRAM 直送贡献）
- CLAUDE.md params.py：多核地址映射 `0x80000000 + i × 0x10000000`
- memory/project_4ddr_poc_result.md（4-DDR PoC 结果背景）

## ASCII 示意稿

```
              ▲ DDR (off-chip)
              │
   ╔══════════╧══════════════════════════════════════════════╗
   ║   AXI Interconnect (Xilinx SmartConnect)                ║
   ╚═════╤════════╤═════════════╤═══════════════╤════════════╝
         │        │             │               │
   AXI-Lite (Control Core)
         │
         ▼
   ┌──────────────────┐
   │ axi_lite_1to4    │ ──── 1-to-4 distribute ────┐
   │ (distributor)    │                            │
   └──────────────────┘                            │
                                                   │
   ┌─────────────────────────────────────────────┐ │   W slice strip:
   │   N=4 multicore (2×2 layout)                │ │   ┌─────────────┐
   │                                             │ │   │sub_W₀│sub_W₁│  ← halo overlap
   │   ┌─────────────────┐ ─x-core push─▶┌──────────────┐│sub_W₂│sub_W₃│
   │   │ Core[0]         │               │ Core[1]      ││ (gray cols
   │   │ sub_W slice 0   │               │ sub_W slice 1││  between)   │
   │   │ IFB|core|OFB    │               │ IFB|core|OFB │└─────────────┘
   │   │ IFB=0x80000000  │ ◀─x-core push─┤ IFB=0x90000000│ halo: K=3,
   │   │ ▶ifb_axi_slave  │               │ ▶ifb_axi_slave│ stride=1
   │   │ done_sticky[0]  │               │ done_sticky[1]│ → 2 cols
   │   └─────────┬───────┘               └────────┬─────┘  overlap
   │             │ x-core push                    │        (computed
   │             ▼                                ▼        redundancy,
   │   ┌─────────────────┐               ┌──────────────┐  no cross-
   │   │ Core[2]         │ ◀─x-core push─┤ Core[3]      │  core sync)
   │   │ sub_W slice 2   │               │ sub_W slice 3│
   │   │ IFB=0xA0000000  │               │ IFB=0xB0000000│
   │   │ ▶ifb_axi_slave  │ ─x-core push─▶│▶ifb_axi_slave │
   │   │ done_sticky[2]  │               │ done_sticky[3]│
   │   └─────────────────┘               └──────────────┘
   │                                             │
   └─────────────────┬───────────────────────────┘
                     │ done_sticky[0..3]
                     ▼
                  ┌──────┐
                  │ AND  │ ─── all-done → next layer enable
                  └──────┘

   key notes:
   - single-core RTL identical at N=1/2/4 — extension at top layer only
   - halo: computed redundancy, no cross-core sync
   - x-core SRAM push: avoids DDR round-trip between layers
   - address map: IFB base = 0x80000000 + i × 0x10000000
```

## 与正文的一致性检查
- [x] §4.11 "N 个加速核（N=1/2/4 已综合通过）" — 中央 2×2 矩阵画 N=4，旁注 "single-core RTL identical at N=1/2/4"
- [x] §4.11 "AXI 互连（基于 Xilinx SmartConnect IP）" — 顶部 AXI Interconnect 横线注 "Xilinx SmartConnect"
- [x] §4.11 "AXI-Lite 一对多分发模块（axi_lite_1to4）" — 顶部 axi_lite_1to4 方块
- [x] §4.11 "W 维切片，每核处理一段 sub_W" — 右侧侧栏 W 切片条 sub_W₀..₃
- [x] §4.11 "K=3 stride=1 时相邻核重叠 2 列输入" — halo 注 "K=3, stride=1: 2 cols overlap"
- [x] §4.11 "halo 由 computed redundancy 实现，不引入跨核同步开销" — 侧栏 "computed redundancy, no cross-core sync"
- [x] §4.11 "层边界握手：所有核 done_sticky 拉起后才允许下一层启动" — 底部 AND 门汇聚
- [x] §4.11 "跨核 SRAM 直送：odma_ctrl 直写消费者核 IFB 物理地址" — 4 根深红色 x-core push 箭头
- [x] §4.11 "多核地址映射 0x80000000 + i × 0x10000000" — 4 个 Core IFB 基址标注一致
- [x] §4.11 "ifb_axi_slave 模块挂载在 AXI 互连上" — 每个 Core 的 IFB 旁画 ifb_axi_slave 端口
- [x] §4.11 "单核 RTL 在 N=1/2/4 配置下完全相同" — 关键标注侧栏明示

## 不确定项
- [TBD: 跨核 push 的具体生产者 → 消费者映射依赖编译器 scheduler.analyze_slicing 决策（W 维切片连续性 + 层依赖），本图按"环式串联"画作示意。实际不同层可能不同，由编译器在 chain 阶段确定]
- [TBD: 是否每对 Core 之间都画 x-core push 箭头取决于网络层结构，本图按"4 核环式"示意。也可改为"任意两核间双向"由 AXI 互连支持，绘图时可由用户简化]
- [CHECK: rdma_ctrl 在多核场景下是否对每核都独立 — 按 §4.10/§4.11 推断每核独立，本图未单独画出 DMA 子系统（已在 fig4-7 详述），仅在 Core 内部小字 "IFB|core|OFB" 抽象示意]

## image 生成提示词

### 中文版

科研论文配图，**多核扩展层（multicore_top）结构图**，IEEE 期刊配色风格，白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1:1（正方形），避免整体过于竖向（高:宽 > 1.5:1）或过于横向（宽:高 > 1.5:1），以适合 A4 单栏插图。N=4 加速核必须用 2×2 矩阵布局（不用 1×4 横排），周围标注 W 切片 halo 重叠 + 跨核 SRAM 直送箭头。**

**版式**：
- **主框（圆角矩形大框）**：标题 "Multi-core Extension Layer (multicore_top, N=4)"（淡黄色 CMYK 0/10/30/0 填充、深灰色边框）。
- **顶部 — AXI Interconnect 横向粗线**：覆盖整图顶部宽度，标 "AXI Interconnect (Xilinx SmartConnect)" 双线方框（深灰色边框、白色填充）。顶部接一个 "DDR (off-chip)" 小方块（淡灰色 CMYK 0/0/0/15 填充），用粗黑色实线箭头朝上贯通。
- **顶部偏左 — AXI-Lite 分发**：在 AXI Interconnect 下方左侧画 "axi_lite_1to4 (1-to-4 distributor)" 小方框（淡橙色 CMYK 0/30/30/0），顶部一根细线标 "AXI-Lite ← Control Core"，下方分出 4 根细线分别送 4 个 Core 的 AXI-Lite 端口。
- **中央主体 — N=4 加速核 2×2 矩阵**（占整图主面积约 60%）：
  - 左上 "Core[0]" 圆角矩形方框（淡蓝色 CMYK 30/10/0/0 填充），内部小字 "sub_W slice 0  |  IFB | core | OFB  |  IFB base=0x80000000  |  ifb_axi_slave  |  done_sticky[0]"
  - 右上 "Core[1]" 同样格式，"sub_W slice 1  |  IFB base=0x90000000"
  - 左下 "Core[2]" "sub_W slice 2  |  IFB base=0xA0000000"
  - 右下 "Core[3]" "sub_W slice 3  |  IFB base=0xB0000000"
- **跨核 SRAM 直送箭头（深红色粗实线）**：在 4 个 Core 之间画 4 根深红色（CMYK 0/100/100/0）粗实线箭头：
  - Core[0] → Core[1]（水平箭头从右边缘进左边缘）
  - Core[1] → Core[2]（弯折，从右下到左上对角）
  - Core[2] → Core[3]（水平箭头）
  - Core[3] → Core[0]（弯折回左上）
  - 每根箭头旁标小字 "x-core push (odma → next-core IFB)"
- **右侧侧栏 — W 切片示意条**：画一个长条矩形（覆盖右侧约 15% 宽度），水平分 4 段标 "sub_W₀ | sub_W₁ | sub_W₂ | sub_W₃"，相邻段之间用淡灰色 CMYK 0/0/0/30 填充小重叠区，标 "halo (2 cols, K=3, stride=1)"。下方小字注 "halo: computed redundancy — no cross-core sync needed"。
- **底部 — done_sticky 汇聚**：4 个 Core 右下角 done_sticky[i] 引出细线汇聚到底部小逻辑门符号 "AND"，输出箭头标 "all-done → next layer enable"。
- **左下角 key notes 小字**：
  - "single-core RTL identical at N=1/2/4 — extension at top layer only"
  - "halo: computed redundancy, no cross-core sync"
  - "x-core SRAM push: avoids DDR round-trip between layers"
  - "address map: IFB base = 0x80000000 + i × 0x10000000"

**字体**：所有英文 Times New Roman 10 pt，模块标题加粗，地址映射数字（0x80000000 等）用等宽字体 Courier New 9 pt；中文（如有）思源黑体 10 pt。**禁止**手写体、彩色渐变、阴影、3D 立体、图标、卡通元素。整体保持工科论文严谨风格。

### English version

Scientific paper figure, **Multi-core extension layer (multicore_top) structure**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **The overall figure aspect ratio should be approximately 1:1 (square). Avoid overly tall layouts (height:width > 1.5:1) or overly wide layouts (width:height > 1.5:1), to fit a single-column A4 figure. The N=4 acceleration cores MUST be arranged in a 2×2 matrix (NOT a 1×4 horizontal row), surrounded by W-slice halo annotations and cross-core SRAM push arrows.**

**Layout**:
- **Main frame (large rounded rectangle)**: Titled "Multi-core Extension Layer (multicore_top, N=4)" (light-yellow CMYK 0/10/30/0 fill, dark-gray border).
- **Top — AXI Interconnect horizontal thick line**: Spans full width at top, labeled "AXI Interconnect (Xilinx SmartConnect)" as a double-line box (dark-gray border, white fill). On top connects to a "DDR (off-chip)" small box (light-gray CMYK 0/0/0/15 fill) via a thick black solid arrow going upward.
- **Top-left — AXI-Lite distributor**: Below AXI Interconnect, on the left, draw "axi_lite_1to4 (1-to-4 distributor)" small box (light-orange CMYK 0/30/30/0); from top a thin line labeled "AXI-Lite ← Control Core"; from bottom 4 thin lines fan out to AXI-Lite ports of the 4 cores.
- **Central main body — N=4 cores 2×2 matrix** (occupies ~60% of figure area):
  - Upper-left "Core[0]" rounded rectangle (light-blue CMYK 30/10/0/0 fill), internal small text: "sub_W slice 0  |  IFB | core | OFB  |  IFB base=0x80000000  |  ifb_axi_slave  |  done_sticky[0]"
  - Upper-right "Core[1]" same format: "sub_W slice 1  |  IFB base=0x90000000"
  - Lower-left "Core[2]": "sub_W slice 2  |  IFB base=0xA0000000"
  - Lower-right "Core[3]": "sub_W slice 3  |  IFB base=0xB0000000"
- **Cross-core SRAM push arrows (dark-red thick solid lines)**: Between the 4 cores draw 4 dark-red (CMYK 0/100/100/0) thick solid arrows:
  - Core[0] → Core[1] (horizontal arrow from right edge to left edge)
  - Core[1] → Core[2] (bent, from lower-right to upper-left diagonal)
  - Core[2] → Core[3] (horizontal arrow)
  - Core[3] → Core[0] (bent back to upper-left)
  - Each arrow labeled "x-core push (odma → next-core IFB)"
- **Right-side strip — W slice diagram**: Draw a long rectangle (covering ~15% width on the right), divided horizontally into 4 segments labeled "sub_W₀ | sub_W₁ | sub_W₂ | sub_W₃", with light-gray CMYK 0/0/0/30 fill in narrow overlap regions between adjacent segments, labeled "halo (2 cols, K=3, stride=1)". Below in small text: "halo: computed redundancy — no cross-core sync needed".
- **Bottom — done_sticky aggregation**: From lower-right of each core, done_sticky[i] thin lines converge to a bottom small logic gate symbol "AND", output arrow labeled "all-done → next layer enable".
- **Lower-left key notes (small text)**:
  - "single-core RTL identical at N=1/2/4 — extension at top layer only"
  - "halo: computed redundancy, no cross-core sync"
  - "x-core SRAM push: avoids DDR round-trip between layers"
  - "address map: IFB base = 0x80000000 + i × 0x10000000"

**Typography**: All English in Times New Roman 10 pt with bold module titles; address numbers (0x80000000 etc.) in monospace Courier New 9 pt; Chinese (if any) in Source Han Sans 10 pt. **Strictly forbidden**: handwritten fonts, color gradients, drop shadows, 3D effects, icons, cartoon elements. Maintain rigorous engineering-paper aesthetics throughout.
