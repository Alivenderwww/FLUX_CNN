# 图 4.6 Ky 折叠等价变换示意图
# Figure 4.6 Schematic diagram of the Ky-fold equivalent transform

## 在论文中的角色
- 首次引入：§4.3 段 "第一类编译器侧变换是 Ky 折叠（Ky-fold），适用于 Cin < 16 且 K > 1 的卷积层..." [依赖: Fig.4.6]
- 论证作用：本论文 narrative A 主轴最关键的图之一。直观展示 "Ky 维 + Cin 维等价合并到 cin_fake = groups_y · Cin" 的数学事实，让读者一眼看懂"硬件视角下与 stride=1 标准卷积同构"。对应贡献 C2.1，是创新点 1。

## 图类型
对照图：左半"原 Cin=4, Ky=3 / 16 PE 行 baseline"（仅 4 行 PE 被有效使用，12 行空转），右半"Ky-fold 后 cin_fake=12 / kyper=1"（12 行 PE 被有效使用，仅 4 行空转）。

## 设计要素

### 必含元素
- **左半 baseline**：
  - 上部：原始权重张量 W (Cout, Cin=4, Ky=3, Kx=1) — 用 4×3×1 小立方体表示
  - 中部：16×16 PE 阵列，仅前 4 行（cin=0..3）有色填充，后 12 行（cin=4..15）灰色"空转"
  - 下部：标 "PE 行利用率 = Cin / 16 = 4/16 = 25%"
- **右半 Ky-fold 后**：
  - 上部：变换后权重 W' (Cout, cin_fake=12, kyper=1, Kx=1)，用 12×1×1 小立方体表示
  - 中部：同 16×16 PE 阵列，前 12 行（cin_fake=0..11）有色填充，仅后 4 行空转
  - 下部：标 "PE 行利用率 = cin_fake / 16 = 12/16 = 75%"
- **中央分隔**：双向箭头 "Ky-fold 等价变换" + 数学注释：
  - `groups_y = HW_PE / Cin = 16 / 4 = 4`（取 ⌊⌋）
  - 实际 Ky=3 < 4，所以 `groups_y = 3 → kyper = 1, cin_fake = 3 · 4 = 12`
  - `pad_ky = groups_y · kyper - K = 3 · 1 - 3 = 0`（无 pad 浪费）
  - `I'[y_virt, x, g · Cin + c] = I_padded[y_virt + g · kyper, x, c]`
  - `W'[ky_local, kx, co, g · Cin + c] = W[g · kyper + ky_local, kx, co, c]`
- **底部小字**："Ky-fold 适用条件：Cin < 16 AND K > 1。硬件无感（仍按 stride=1 标准卷积跑），代价是 IFB 占用 × groups_y。"

### 标注要求
- 用红色高亮空转 PE 行，绿色高亮有效 PE 行
- 中央数学注释用框包围，标 "compute_fold_params(K, Cin, HW_PE)"
- 上部权重张量使用 3D 立方体小图，明确 Cin / Ky 维的"重排合并"关系
- 在原权重 → 变换权重之间画一组 ky 颜色分组（ky=0 红、ky=1 绿、ky=2 蓝），变换后这 3 个 ky 沿 Cin 方向拼接成 cin_fake=12

### 视觉层次
- 主角：左右两个 16×16 PE 阵列利用率对比
- 配角：上部权重张量小图
- 背景：中央数学注释

## ASCII 示意稿

```
   ──────────────  baseline (Cin=4, Ky=3)  ─────  Ky-fold 后 (cin_fake=12, kyper=1)
                                          │
   原权重 W                                │   变换权重 W'
   (Cout, Cin=4, Ky=3, Kx=1)              │   (Cout, cin_fake=12, kyper=1, Kx=1)
   ┌─────┬─────┬─────┐                    │   ┌─────┐  cin_fake = 12
   │ ky=0│ ky=1│ ky=2│                    │   │ "" │  按 g · Cin + c 拼接：
   │ R R R│ G G G│ B B B│                  │   │ R0  │  g=0(R),c=0..3 → 0..3
   │ R R R│ G G G│ B B B│   →── Ky-fold ──┼──→│ R1  │  g=1(G),c=0..3 → 4..7
   │ R R R│ G G G│ B B B│   compute_fold  │   │ R2  │  g=2(B),c=0..3 → 8..11
   │ R R R│ G G G│ B B B│   _params       │   │ R3  │
   └─────┴─────┴─────┘                    │   │ G0  │
   Cin=4, Ky=3                            │   │ G1  │
                                          │   │ G2  │
                                          │   │ G3  │
                                          │   │ B0  │
                                          │   │ B1  │
                                          │   │ B2  │
                                          │   │ B3  │
                                          │   └─────┘
                                          │
   16×16 PE 阵列利用率：                  │   16×16 PE 阵列利用率：
   ┌─────────────────────┐                │   ┌─────────────────────┐
   │ row 0 ▓▓▓▓▓▓▓▓▓ (R) │                │   │ row 0 ▓▓▓▓▓ (R0)    │
   │ row 1 ▓▓▓▓▓▓▓▓▓ (R) │ 有效 4 行      │   │ row 1 ▓▓▓▓▓ (R1)    │
   │ row 2 ▓▓▓▓▓▓▓▓▓ (R) │                │   │ row 2 ▓▓▓▓▓ (R2)    │
   │ row 3 ▓▓▓▓▓▓▓▓▓ (R) │                │   │ row 3 ▓▓▓▓▓ (R3)    │
   │ row 4 ░░░░░░░░░ 空  │                │   │ row 4 ▓▓▓▓▓ (G0)    │ 有效 12 行
   │ row 5 ░░░░░░░░░ 空  │                │   │ row 5 ▓▓▓▓▓ (G1)    │
   │ row 6 ░░░░░░░░░ 空  │ 空转 12 行     │   │ row 6 ▓▓▓▓▓ (G2)    │
   │ ...                 │                │   │ row 7 ▓▓▓▓▓ (G3)    │
   │ row 15 ░░░░░░░░ 空  │                │   │ row 8 ▓▓▓▓▓ (B0)    │
   └─────────────────────┘                │   │ row 9 ▓▓▓▓▓ (B1)    │
                                          │   │ row 10 ▓▓▓▓ (B2)    │
   PE 行利用率 = 4/16 = 25%                │   │ row 11 ▓▓▓▓ (B3)    │
                                          │   │ row 12 ░░░░ 空      │  空转 4 行
                                          │   │ ...                 │
                                          │   │ row 15 ░░░ 空       │
                                          │   └─────────────────────┘
                                          │   PE 行利用率 = 12/16 = 75%

   compute_fold_params(K=3, Cin=4, HW_PE=16):
     groups_y = min(K, HW_PE // Cin) = min(3, 4) = 3
     kyper    = ⌈K / groups_y⌉ = ⌈3/3⌉ = 1
     cin_fake = groups_y · Cin = 3 · 4 = 12
     pad_ky   = groups_y · kyper - K = 3·1 - 3 = 0  (无 pad 浪费)

   I'[y_virt, x, g·Cin + c] = I_padded[y_virt + g·kyper, x, c]
   W'[ky_local, kx, co, g·Cin + c] = W[g·kyper + ky_local, kx, co, c]

   ★ 硬件视角下：与 stride=1 / K=kyper / Cin=cin_fake 的标准卷积同构 ★
   ★ 硬件零改动；代价是 IFB 占用 × groups_y（编译器 y 偏移复制）★
```

## 数据来源
- paper.md §4.3 段 "第一类编译器侧变换是 Ky 折叠..."
- docs/pe-fold.md §1
- contributions.md C2.1
- toolchain/hw_files.py: compute_fold_params / fold_input / fold_weights

## 与正文一致性检查
- [x] 适用条件 Cin < 16 AND K > 1 — 与 §4.3 / docs/pe-fold.md 一致
- [x] 参数派生公式（groups_y / kyper / cin_fake / pad_ky）— 与 §4.3 / docs/pe-fold.md 一致
- [x] 虚拟激活 / 虚拟权重定义式 — 与 §4.3 一致
- [x] 硬件零改动、IFB × groups_y 代价 — 与 §4.3 / docs/pe-fold.md 一致

## 不确定项
- 为方便绘制选用 Cin=4 / K=3 → cin_fake=12 示例。如用户希望用 ResNet-11 实际中间层（如 Cin=8 / K=3）作为示例，可调换数字 [TBD]。
