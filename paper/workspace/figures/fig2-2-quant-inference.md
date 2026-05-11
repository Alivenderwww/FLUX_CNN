# 图 2.2 单层神经网络量化计算原理示意图
# Figure 2.2 Schematic of single-layer quantized inference dataflow

## 在论文中的角色
- 首次引入位置：§2.5 模型量化基础（paper.md L198）
- 引用位置：paper.md L198
- 论证作用：直观说明 INT8 量化推理通路上各张量的位宽分配（INT8 输入 / INT8 权重 / INT32 偏置 / INT32 PSUM / INT8 输出），是 §4.5 SDP 后处理模块结构的算法蓝本。

## 图类型
数据流示意图（含位宽标注）。

## 设计要素

### 必含元素
1. **输入张量节点**：左端方块，标注 "Input activation (INT8)"，下方注 "from previous layer"。
2. **权重节点**：上方方块，标注 "Weights (INT8)"，下方注 "loaded from DDR / WB"。
3. **乘加阵列节点**：中间大圆角方块，标注 "MAC array  (INT8 × INT8 → INT16 → INT32 PSUM)"。
4. **偏置节点**：右上方块，标注 "Bias (INT32)"。
5. **加偏置节点**：圆形或菱形，标注 "+ bias"。
6. **激活函数节点**：方块，标注 "Activation (ReLU)"。
7. **重量化节点**：方块，标注 "Requantize  (right-shift + saturate)"。
8. **输出张量节点**：右端方块，标注 "Output activation (INT8)"，下方注 "to next layer"。
9. **位宽箭头标注**：每条数据流箭头旁标 "INT8" / "INT32" 等位宽。

### 标注要求
- 乘法器面积比注："INT8 mult ≈ 1/10 area of FP32 mult"
- 偏置位宽 INT32 解释："avoid overflow during accumulation"
- 输出位宽 INT8 解释："directly feeds next layer"

### 视觉层次
- 主角：横向数据流主轴（输入 → MAC → +bias → ReLU → requantize → 输出）
- 配角：权重与偏置两路侧路输入
- 背景：浅灰色框注 "INT8 quantized inference path"

## 数据来源
- paper.md §2.5（量化白皮书 [27]）
- contributions.md C1.4（SDP 链）

## ASCII 示意稿

```
                          Weights (INT8)              Bias (INT32)
                          ┌────────────┐              ┌────────────┐
                          │  loaded    │              │  loaded    │
                          │  from DDR  │              │  from DDR  │
                          └─────┬──────┘              └─────┬──────┘
                                │ INT8                      │ INT32
                                ▼                           │
   ┌──────────────┐    ┌─────────────────────────────┐      │
   │  Input       │    │  MAC array                  │      │
   │  activation  │───▶│  INT8 × INT8 → INT16        │      │
   │  (INT8)      │    │              → INT32 PSUM   │      │
   └──────────────┘    └────────────┬────────────────┘      │
   from prev layer                  │ INT32 PSUM            │
                                    ▼                       │
                              ┌─────────────┐  ◀────────────┘
                              │  + bias     │
                              └──────┬──────┘
                                     │ INT32
                                     ▼
                              ┌─────────────┐
                              │  Activation │
                              │  (ReLU)     │
                              └──────┬──────┘
                                     │ INT32
                                     ▼
                              ┌─────────────────────┐
                              │  Requantize         │
                              │  right-shift +      │
                              │  saturate to INT8   │
                              └──────┬──────────────┘
                                     │ INT8
                                     ▼
                              ┌─────────────┐
                              │  Output     │
                              │  activation │ ──▶ next layer
                              │  (INT8)     │
                              └─────────────┘

   Note: INT8 multiplier ≈ 1/10 area of FP32 multiplier (paper.md §2.5)
```

## 与正文的一致性检查
- [x] §2.5 "权重通常被量化至 8 位，偏置量化至 32 位以避免大数累加溢出" — 图中权重 INT8、偏置 INT32
- [x] §2.5 "8 位定点乘法，其结果累加进 32 位部分和" — 图中 MAC 节点位宽 INT8×INT8→INT32 PSUM
- [x] §2.5 "累加完成后与 32 位偏置相加，再经过激活函数与重量化逻辑得到 8 位输出" — 图中 +bias → ReLU → Requantize → INT8
- [x] §2.5 "8 位乘法器的面积约为 32 位浮点乘法器的十分之一以下" — 图注说明

## 不确定项
- [TBD: 是否在重量化节点同时画出移位量参数 *shift*] — 倾向画出（与 §4.5 SDP 链对齐）

## image 生成提示词

### 中文版

科研论文配图，**INT8 量化推理数据流图**，IEEE 期刊配色风格，白底黑字，整体竖向流水线布局，可在版面允许时改为横向。**整体画面宽高比约 1:1（正方形）或 1:1.2（高略大于宽），避免整体过于竖向（高:宽 > 1.5:1）或过于横向（宽:高 > 1.5:1），以适合 A4 单栏插图；若主轴节点较多，请采用 2 列折行排列以压缩竖向高度。**

**版式**：从左到右或从上到下绘制 INT8 量化推理主轴。每个节点用矩形或圆角矩形方框（淡蓝色 CMYK 20/5/0/0 填充、深蓝色实线边框），节点之间用黑色实线带箭头连接，箭头旁标位宽（如 "INT8"、"INT32 PSUM"）。

**主轴节点**（按顺序）：
1. "Input activation (INT8)"，下方小字 "from prev layer"。
2. "MAC array  INT8 × INT8 → INT16 → INT32 PSUM"，节点稍大，居中。
3. "+ bias"，菱形或圆角方块。
4. "Activation (ReLU)"。
5. "Requantize  right-shift + saturate"。
6. "Output activation (INT8)"，下方小字 "to next layer"。

**侧路输入**（淡黄色 CMYK 0/10/40/0 填充）：
- "Weights (INT8) loaded from DDR" 接到 MAC array 节点上方
- "Bias (INT32) loaded from DDR" 接到 +bias 节点右侧

**位宽标注**：每条箭头旁标位宽，使用 Times New Roman 9 pt 等宽字体。整图右下角加图注 "Note: INT8 multiplier ≈ 1/10 area of FP32 multiplier"，字体 Times New Roman 8 pt。

**字体**：英文 Times New Roman 10 pt，中文思源黑体 10 pt。**禁止**手写字体、卡通元素、霓虹色彩、3D 阴影、装饰图标。保持工科论文专业排版。

### English version

Scientific paper figure, **INT8 quantized inference dataflow diagram**, IEEE-journal style, white background with black text, vertical pipeline layout (or horizontal if page allows). **The overall figure aspect ratio should be approximately 1:1 (square) or 1:1.2 (slightly taller than wide). Avoid overly tall layouts (height:width > 1.5:1) or overly wide layouts (width:height > 1.5:1), to fit a single-column A4 figure; if the main-axis nodes are too many, fold them into a 2-column arrangement to compress vertical height.**

**Layout**: Draw the main INT8 quantized-inference axis from top to bottom (or left to right). Each node is a rectangle or rounded rectangle with light-blue (CMYK 20/5/0/0) fill and dark-blue solid border. Connect nodes with solid black arrows; label each arrow with the bit-width (e.g., "INT8", "INT32 PSUM").

**Main-axis nodes** (in order):
1. "Input activation (INT8)" with subtitle "from prev layer"
2. "MAC array  INT8 × INT8 → INT16 → INT32 PSUM" — slightly larger, centered
3. "+ bias" — diamond or rounded rectangle
4. "Activation (ReLU)"
5. "Requantize  right-shift + saturate"
6. "Output activation (INT8)" with subtitle "to next layer"

**Side-path inputs** (light-yellow CMYK 0/10/40/0 fill):
- "Weights (INT8) loaded from DDR" connects to MAC-array node from above
- "Bias (INT32) loaded from DDR" connects to +bias node from the right

**Bit-width annotations**: Place a bit-width label next to each arrow, using Times New Roman 9 pt monospaced. Add a footnote at lower-right: "Note: INT8 multiplier ≈ 1/10 area of FP32 multiplier", Times New Roman 8 pt.

**Typography**: English in Times New Roman 10 pt; Chinese (if any) in Source Han Sans 10 pt. **Strictly forbidden**: handwritten fonts, cartoon elements, neon colors, 3D shadows, decorative icons. Maintain professional engineering-paper typesetting throughout.
