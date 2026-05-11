# 图 4.7 空间到深度（S2D）等价变换示意图
# Figure 4.7 Schematic diagram of the space-to-depth (S2D) transform

## 在论文中的角色
- 首次引入：§4.3 段 "第二类编译器侧变换是空间到深度（S2D）..." [依赖: Fig.4.7]
- 论证作用：本论文 narrative A 主轴最关键的图之二（与图 4.6 并列）。直观展示"stride² 个相位折叠到 cin_new = stride² · Cin"的等价变换 + 在 ResNet-11 Patch 层（K=4 / stride=4 / Cin=3）的具体效果。重点突出"DDR 友好重排无激活复制"——与 Ky-fold 的 IFB × groups_y 代价对照。对应贡献 C2.2，是创新点 2。

## 图类型
对照图：左半"原 K=4 / stride=4 / Cin=3 baseline"（PE 行利用率仅 3/16 ≈ 19%，stride² 相位浪费），右半"S2D 后 K_new=1 / stride=1 / cin_new=48"（PE 行利用率拉满到 16/16 + 阵列继续按 cin slice 跑，无空转）。

## 设计要素

### 必含元素
- **左半 baseline (Patch 层)**：
  - 输入：H×W RGB 图，Cin=3，stride=4 滑窗；标 "stride=4 → 16 个相位 (a=0..3, b=0..3)"
  - 16×16 PE 阵列：仅前 3 行被使用，后 13 行空转 + 还有 stride² 相位浪费
  - 单层 wall cycles：**654,404 cy**（来自 STATUS.md §2.8）
- **右半 S2D 后**：
  - 输入：H/4 × W/4 重排后特征图，cin_new = stride² · Cin = 16 · 3 = 48
  - 16×16 PE 阵列：cin_new=48 在 cin slice 维度上跑 3 个 slice（48/16 = 3）每 slice 都填满 16 行
  - 单层 wall cycles：**129,594 cy**（来自 STATUS.md §2.8）
  - **加速 5.05×**
- **中央分隔**：双向箭头 "S2D 等价变换"
- **数学注释框**：
  - `按 ky = stride·ky' + a, kx = stride·kx' + b, p = a·stride + b 重写`
  - `I'[Y, X, p·Cin + c] = I_padded[Y·stride + a, X·stride + b, c]`
  - `W'[ky', kx', co, p·Cin + c] = W[ky'·stride + a, kx'·stride + b, co, c]`
  - `K_new = ⌈K / stride⌉ = ⌈4/4⌉ = 1, Cin_new = stride² · Cin = 48, stride_new = 1`
  - `pad_waste = (K_new² · stride² - K²) / (K_new² · stride²) = (1·16 - 16)/16 = 0` (K=4, s=4 完美整除)
- **底部小字**：
  - "S2D 适用条件：stride ≥ 2 AND K ≥ stride"
  - "DDR / IFB 代价为零（仅相位重排，无激活复制）"
  - "stride 变 1 后可启用 ARF reuse_en=1（约束 stride==1 && K>1 && tile_w ≤ 33-K）"
  - "K 不被 stride 整除时有 pad_waste 量化损失"

### 标注要求
- 颜色分组：用 16 种相位颜色（4 种主色 × 4 种深浅）填充输入图，每个 stride² 相位对应一种颜色
- 中央等价关系框用紫色高亮
- 加速比 5.05× 用大字加粗显示在右半下方
- 在 cin_new = 48 处加一个 "=16·3" 的小数学说明（因 48 不被 16 整除，需要 cin slice 跑 3 次每次 16）

### 视觉层次
- 主角：左右两组 wall cycles 对比 + 加速比 5.05×
- 配角：输入特征图相位重排示意
- 背景：等价关系数学框

## ASCII 示意稿

```
   ────  baseline (K=4, stride=4, Cin=3) ────  ─── S2D 后 (K_new=1, stride=1, cin_new=48) ──

   输入：H × W RGB                              输入：H/4 × W/4，cin_new=48
   ┌─────────────────────┐                      ┌─────────┐
   │ stride=4 滑窗       │                      │ phase 重│  cin_offset 0..2  ← phase (0,0)
   │ 共 16 个相位        │  ──S2D──→            │ 排后    │  cin_offset 3..5  ← phase (0,1)
   │ (a=0..3, b=0..3)    │  (相位折到 cin)      │         │  cin_offset 6..8  ← phase (0,2)
   │                     │                      │ 沿     │  cin_offset 9..11 ← phase (0,3)
   │ Cin=3 (RGB)         │                      │ Cin     │  ...
   │                     │                      │ 维堆叠 │  cin_offset 45..47← phase (3,3)
   └─────────────────────┘                      └─────────┘
   
   16×16 PE 阵列 baseline：                       16×16 PE 阵列 S2D 后：
   ┌─────────────────────┐                       ┌─────────────────────┐
   │ row 0  ▓▓ R         │                       │ row 0  ▓▓▓▓▓▓▓▓     │
   │ row 1  ▓▓ G         │ 有效 3 行             │ row 1  ▓▓▓▓▓▓▓▓     │ 满载
   │ row 2  ▓▓ B         │                       │ ...                 │ 16 行
   │ row 3  ░░ 空        │                       │ row 15 ▓▓▓▓▓▓▓▓     │
   │ ...                 │ 空转 13 行            │                     │
   │ row 15 ░░ 空        │                       │ cin slice 跑 3 次    │
   └─────────────────────┘                       │ (cin_new=48 / 16=3)  │
                                                 └─────────────────────┘
   PE 行利用率 = 3/16 = 19%                       PE 行利用率 = 16/16 = 100%
   + stride²=16 个相位浪费                       + stride² 浪费消除
   wall cycles = 654,404 cy                       wall cycles = 129,594 cy
                                                     单层加速 5.05× ★

   ┌──────── 等价变换数学（紫色框）─────────────────────────────┐
   │                                                            │
   │ 按 ky = stride·ky' + a, kx = stride·kx' + b, p = a·stride+b │
   │ 重写卷积：                                                  │
   │   I'[Y, X, p·Cin + c]                                       │
   │     = I_padded[Y·stride + a, X·stride + b, c]               │
   │   W'[ky', kx', co, p·Cin + c]                               │
   │     = W[ky'·stride + a, kx'·stride + b, co, c]              │
   │                                                            │
   │ 派生参数：                                                  │
   │   K_new = ⌈K / stride⌉ = ⌈4/4⌉ = 1                          │
   │   Cin_new = stride² · Cin = 16 · 3 = 48                     │
   │   stride_new = 1                                            │
   │   pad_waste = (1·16 - 16)/16 = 0  (K=4, s=4 完美整除)       │
   │                                                            │
   │ ★ 硬件视角下：与 stride=1 / K=K_new / Cin=Cin_new 同构 ★    │
   │ ★ DDR / IFB 代价为零（仅相位重排，无激活复制）★            │
   │ ★ stride=1 后可启用 ARF reuse_en=1 复用                    │
   └────────────────────────────────────────────────────────────┘

   适用：stride ≥ 2 AND K ≥ stride
   注意：K 不被 stride 整除时 pad_waste = (K_new²·s² - K²)/(K_new²·s²) > 0
        例：K=3 / s=2 → K_new=2, pad_waste = (4·4 - 9)/16 = 7/16 ≈ 43.75%
```

## 数据来源
- paper.md §4.3 段 "第二类编译器侧变换是空间到深度（S2D）..."
- docs/pe-fold.md §2
- contributions.md C2.2
- STATUS.md §2.8（Patch 层 654,404 → 129,594 cy）
- toolchain/hw_files.py: compute_s2d_params / s2d_input

## 与正文一致性检查
- [x] 适用条件 stride ≥ 2 AND K ≥ stride — 与 §4.3 / docs/pe-fold.md 一致
- [x] 参数派生公式（K_new / Cin_new / pad_waste） — 与 §4.3 / docs/pe-fold.md 一致
- [x] 虚拟激活 / 虚拟权重定义式 — 与 §4.3 一致
- [x] Patch 层 5.05× 加速 / 654,404 → 129,594 cy — 与 STATUS.md §2.8 / contributions.md C2.2 一致
- [x] 引用谱系（Pixel-Shuffle 来源已在图 2.5 标注）— 与 §1.2.4 / §6.2 创新点 2 诚实声明一致

## 不确定项
无。Patch 层 5.05× 数字已实测。
