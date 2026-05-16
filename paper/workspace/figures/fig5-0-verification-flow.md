# 图 5.0 验证流程图
# Figure 5.0 Verification flow

## 在论文中的角色
- 首次引入：§5.2 验证环境与方法（paper.md L538-L557）
- 引用位置：§5.2.2 验证方法（功能验证小节）
- 论证作用：用一张端到端流程图把"PyTorch 浮点训练 → INT8 量化 → 编译器生成 cfg + 数据 dump → ModelSim TB 加载 → RTL 仿真 → OFM dump → PyTorch 参考逐字节比对 → PASS/FAIL"完整链路可视化，让读者理解 FLUX_CNN 验证流程的两端锚点（PyTorch 浮点参考 + RTL 仿真）以及编译器在中间的桥接作用。

## 图类型
端到端流程框图（节点 + 数据张量类型标注 + 比对汇合点）。

## 设计要素

### 必含元素
1. **流程节点（自上而下或自左而右 8 节点）**：
   - **节点 1**：PyTorch FP32 训练模型（标"nn.Sequential" 输入 → 输出"）
   - **节点 2**：INT8 量化（PTQ / QAT）
   - **节点 3**：编译器（toolchain/gen_isa_test.py + scheduler）
   - **节点 4**：生成 cfg 寄存器值 + DDR 数据 dump（权重 / 偏置 / 输入 / 描述符链表）
   - **节点 5**：TB sim/tb_core_dma 加载 DDR 镜像
   - **节点 6**：ModelSim RTL 仿真（含 axi_dm IP）
   - **节点 7**：OFB dump（OFM INT8 张量）
   - **节点 8**：PyTorch 浮点参考逐字节比对（PASS / FAIL）
2. **并行参考通路**：节点 2 的右侧分叉一条线到"PyTorch INT8 参考模型"，与节点 7 的输出在节点 8 汇合做逐字节比对。
3. **数据张量类型标注**：每节点之间的箭头上方标数据格式：
   - 节点 1→2："FP32 weights & inputs"
   - 节点 2→3："INT8 weights, INT32 bias, scale/shift"
   - 节点 3→4："cfg_regs values + DDR layout"
   - 节点 4→5："binary dump (.hex / .bin)"
   - 节点 5→6："AXI4 transactions"
   - 节点 6→7："OFM INT8 tensor"
   - 节点 7→8："byte-by-byte compare"
4. **结果分支**：节点 8 输出分两路："PASS（绿色）"与"FAIL（红色）"。
5. **三层验证粒度图例**：底部加图例"单算子 / 多层链 / 整网（ResNet11）"三种验证粒度，每种走同一流程图但激励规模不同。

### 标注要求
- 每节点用对应工具 / 模块名 +（简体中文 + 英文）
- 数据张量类型用斜体小字
- PASS / FAIL 分支用颜色对比（绿 / 红）
- 左上角标"验证流程 / Verification flow"

### 视觉层次
- 主角：8 节点主流程线
- 配角：右侧 PyTorch 参考通路 + 节点 8 比对汇合
- 背景：底部三层验证粒度图例

## 数据来源
- paper.md §5.2.1 验证环境（硬件平台 / 软件工具 / 工具链）
- paper.md §5.2.2 验证方法（功能 / 性能 / 对比三层）
- paper.md §5.3 功能仿真（单算子 22 + 形状鲁棒 24 + 多核 20 + ResNet11 1）
- contributions.md C3.x 工具链层

## ASCII 示意稿

```
   ┌───────────────────┐
   │ 1. PyTorch FP32   │  FP32 weights & inputs
   │    nn.Sequential  │────────────────────────┐
   └─────────┬─────────┘                         │
             │                                   │
             ▼                                   ▼
   ┌───────────────────┐                ┌──────────────────┐
   │ 2. INT8 量化      │                │ PyTorch INT8     │
   │   (PTQ / QAT)     │─── scale ────▶│ 参考模型         │
   └─────────┬─────────┘                │ (逐层浮点模拟)   │
             │                          └────────┬─────────┘
             │ INT8 weights, INT32 bias          │
             ▼                                   │
   ┌───────────────────┐                         │
   │ 3. 编译器         │                         │
   │  gen_isa_test.py  │                         │
   │  + scheduler      │                         │
   └─────────┬─────────┘                         │
             │ cfg + DDR layout                  │
             ▼                                   │
   ┌───────────────────┐                         │
   │ 4. 生成 cfg + dump│                         │
   │  权重/偏置/输入/  │                         │
   │  描述符链表       │                         │
   └─────────┬─────────┘                         │
             │ binary dump                       │
             ▼                                   │
   ┌───────────────────┐                         │
   │ 5. TB tb_core_dma │                         │
   │   加载 DDR 镜像   │                         │
   └─────────┬─────────┘                         │
             │ AXI4 transactions                 │
             ▼                                   │
   ┌───────────────────┐                         │
   │ 6. ModelSim RTL   │                         │
   │   仿真 (axi_dm)   │                         │
   └─────────┬─────────┘                         │
             │ OFM INT8 tensor                   │
             ▼                                   │
   ┌───────────────────┐                         │
   │ 7. OFB dump       │                         │
   │   INT8 tensor     │                         │
   └─────────┬─────────┘                         │
             │                                   │
             ▼                                   ▼
            ┌──────────────────────────────────────┐
            │ 8. 逐字节比对 (byte-by-byte compare) │
            └────────────────┬─────────────────────┘
                             │
              ┌──────────────┴──────────────┐
              ▼                             ▼
        ┌──────────┐                ┌──────────┐
        │ PASS (绿)│                │ FAIL (红)│
        └──────────┘                └──────────┘

   验证粒度：[单算子 22 case] [多层链] [ResNet11 整网 11 层]
```

## 与正文的一致性检查
- [x] §5.2.1 "Python 侧的编译器、激励生成与参考模型三类工具" — 节点 3-4 编译器 + 参考通路
- [x] §5.2.1 "编译器将 PyTorch nn.Sequential 模型编译为硬件配置寄存器值与 DDR 数据布局" — 节点 3→4 标注
- [x] §5.2.1 "参考模型使用 PyTorch 在 CPU 上以 INT8 浮点逐层模拟硬件计算路径，逐元素生成参考输出" — 右侧 PyTorch 参考通路
- [x] §5.2.2 "ModelSim RTL 仿真输出为被验证对象，逐元素按位比较" — 节点 6 + 节点 8
- [x] §5.2.2 "任何字节不匹配判定为 FAIL，仅当全部输出字节完全一致时判定为 PASS" — 节点 8 PASS/FAIL 分支
- [x] §5.3 "单算子、多层链与整网三个层次" — 底部图例三种粒度
- [x] §5.2.1 "axi_dm 等加密 IP 可在 ModelSim 中正确精化与运行" — 节点 6 标注 (axi_dm)

## 不确定项
- [TBD: 是否把"compile_simlib"步骤单独画为节点] — 倾向不画，作为节点 6 的前置一行小注释（仿真前一次性运行）
- [TBD: 节点 4 的 dump 列表是否详尽（缺 odma 目的地址等）] — 倾向保持精简，正文 §5.2 已详述

## image 生成提示词

### 中文版

科研论文配图，**FLUX_CNN 端到端功能验证流程图**，IEEE 期刊配色风格，白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1:1.2（高略大于宽），适合 A4 单栏插图。**

**版式（主流程自上而下 + 右侧参考通路）**：
- **主流程线（占画面左 65% 宽度，自上而下 7 个节点）**：节点为圆角矩形（淡蓝色 CMYK 30/10/0/0 填充、深灰 #404040 边框），等宽（约画面 35% 宽）、垂直堆叠。节点间距均匀，间用细黑色实线箭头（线宽 1 pt）连接，箭头中段上方放数据张量类型标注（Times New Roman 8 pt 斜体）。节点内容：
  1. "1. PyTorch FP32 训练模型 / nn.Sequential"
  2. "2. INT8 量化 (PTQ / QAT)"
  3. "3. 编译器 toolchain (gen_isa_test.py + scheduler)"
  4. "4. 生成 cfg 寄存器值 + DDR dump (权重/偏置/输入/描述符)"
  5. "5. TB sim/tb_core_dma 加载 DDR 镜像"
  6. "6. ModelSim RTL 仿真 (含 axi_dm IP)"
  7. "7. OFB dump (OFM INT8 张量)"
- 箭头标注：1→2 "FP32 weights & inputs"；2→3 "INT8 weights, INT32 bias, scale/shift"；3→4 "cfg + DDR layout"；4→5 "binary dump (.hex / .bin)"；5→6 "AXI4 transactions"；6→7 "OFM INT8 tensor"。
- **右侧参考通路（占画面右 30% 宽度）**：从节点 2 右侧分叉一条向右的细灰色实线箭头到一个独立节点（淡黄色 CMYK 0/10/30/0 填充）："PyTorch INT8 参考模型 / Reference model (逐层浮点模拟)"。该节点向下伸出一条长虚线箭头穿过画面右侧到达底部比对节点。
- **底部比对汇合节点**：在主流程节点 7 下方，绘制一个大圆角矩形（淡绿色 CMYK 30/0/30/0 填充）"8. 逐字节比对 / byte-by-byte compare"（思源黑体 10 pt 加粗）。从节点 7 与右侧参考节点分别画箭头汇入此节点。
- **PASS / FAIL 分支**：比对节点底部分两路向下：
  - 左路 — 深绿色 #2e7d32 实线箭头 + 矩形 "PASS"（深绿填充）
  - 右路 — 深红色 #c00000 实线箭头 + 矩形 "FAIL"（深红填充）
- **底部图例**：画面最底端一行小字（Times New Roman 9 pt 斜体）"Verification granularity: 单算子 (22 case) / 多层链 / ResNet11 整网 (11 层)" 与英文 "single-op / multi-layer chain / ResNet11 full-net"。
- **顶部标题**：画面左上角小字"验证流程 / Verification flow"（思源黑体 10 pt 加粗）。

**字体**：所有英文 Times New Roman 10 pt，节点标题加粗；中文思源黑体 10 pt。**严肃学术风格，IEEE 期刊配色，禁止卡通**：禁止手写体、彩色渐变、阴影、3D 立体、图标、emoji、卡通元素。PASS 用深绿、FAIL 用深红，其他节点保持淡蓝 / 淡黄 / 淡绿三色协调。整体保持工科论文严谨风格。

### English version

Scientific paper figure, **FLUX_CNN end-to-end functional verification flow**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **Overall aspect ratio approximately 1:1.2 (slightly taller), suitable for a single-column A4 figure.**

**Layout (main pipeline top-to-bottom + right-side reference path)**:
- **Main pipeline (left 65% width, 7 vertical nodes)**: Rounded-rectangle nodes (light-blue CMYK 30/10/0/0 fill, dark-gray #404040 border), uniform width (~35% of canvas), vertically stacked with even spacing. Connect nodes with thin solid black arrows (1 pt). Place data-tensor-type labels above each arrow (Times New Roman 8 pt italic). Node contents:
  1. "1. PyTorch FP32 training model / nn.Sequential"
  2. "2. INT8 quantization (PTQ / QAT)"
  3. "3. Compiler toolchain (gen_isa_test.py + scheduler)"
  4. "4. Generate cfg registers + DDR dump (weights/bias/inputs/descriptors)"
  5. "5. TB sim/tb_core_dma loads DDR image"
  6. "6. ModelSim RTL simulation (with axi_dm IP)"
  7. "7. OFB dump (OFM INT8 tensor)"
- Arrow labels: 1→2 "FP32 weights & inputs"; 2→3 "INT8 weights, INT32 bias, scale/shift"; 3→4 "cfg + DDR layout"; 4→5 "binary dump (.hex / .bin)"; 5→6 "AXI4 transactions"; 6→7 "OFM INT8 tensor".
- **Right-side reference path (right 30% width)**: From node 2, branch a thin solid gray arrow rightward to an independent node (light-yellow CMYK 0/10/30/0 fill) "PyTorch INT8 reference model (per-layer float simulation)". From this node extend a long dashed arrow downward to the bottom compare node.
- **Bottom compare node**: Below node 7, place a large rounded rectangle (light-green CMYK 30/0/30/0 fill) "8. byte-by-byte compare" (Source Han Sans 10 pt bold). Arrows from node 7 and from the right reference node both merge into this compare node.
- **PASS / FAIL branches**: The compare node forks downward into two paths:
  - Left — deep-green #2e7d32 solid arrow + "PASS" rectangle (deep-green fill)
  - Right — deep-red #c00000 solid arrow + "FAIL" rectangle (deep-red fill)
- **Bottom legend**: A single italic line (Times New Roman 9 pt) "Verification granularity: single-op (22 case) / multi-layer chain / ResNet11 full-net (11 layers)".
- **Top-left title**: Small bold label "Verification flow" (Source Han Sans 10 pt bold).

**Typography**: All English in Times New Roman 10 pt with bold titles; Chinese in Source Han Sans 10 pt. **Serious academic style, IEEE palette, no cartoon**: strictly forbid handwritten fonts, color gradients, drop shadows, 3D effects, icons, emoji, or cartoon elements. Use deep-green for PASS and deep-red for FAIL; keep other nodes in coordinated light-blue / light-yellow / light-green palette. Maintain rigorous engineering-paper aesthetics throughout.
