# 图 2.2 ResNet 残差短接 identity / projection 形态对比
# Figure 2.2 Identity vs projection short-cut in ResNet residual block

## 在论文中的角色
- 首次引入：§2.2 段 "ResNet 系列残差结构在两条分支的 short-cut 设计上分为 identity 与 projection 两种..." 末 [依赖: Fig.2.2]
- 论证作用：直观对比 identity 与 projection 两种残差形态在通道维 / 空间形状 / 加法时机上的差异，为 §4.2.5 SDP 残差融合的硬件实现细节铺垫。

## 图类型
并排对比图：左半 identity short-cut，右半 projection short-cut；底部用统一图例标注共用元素。

## 设计要素

### 必含元素
- **左半 identity**：
  - 主分支：输入 X （Cin = C, H, W）→ Conv1 (3×3, C→C) → BN/ReLU → Conv2 (3×3, C→C) → 加法 ⊕ → ReLU → 输出 Y
  - 短接分支：从 X 直接到 ⊕（无运算），标注 "identity（直接相加）"
  - 加法节点 ⊕ 旁标注 "通道与空间形状一致"
- **右半 projection**：
  - 主分支：输入 X (Cin = C, H, W) → Conv1 (3×3, C→C', stride=2) → BN/ReLU → Conv2 (3×3, C'→C') → 加法 ⊕ → ReLU → 输出 Y
  - 短接分支：从 X 经 Conv 1×1 (C→C', stride=2) → BN，到 ⊕，标注 "projection（1×1 卷积调形）"
  - 加法节点 ⊕ 旁标注 "通道 / 空间形状对齐后相加"

### 标注要求
- 在两个 ⊕ 节点下方各加一行小字：左 "IFB region 共享（生产层与消费层共占同一 region）"，右 "IFB region 独立（projection 分支单独 region，主分支完成后参与 SDP 加）"——这是 §4.2.5 在硬件侧需要的细节
- 主分支用粗箭头，短接分支用细箭头，颜色区分（主红、短接蓝）
- 卷积块用矩形框，标注 kernel / stride / Cin→Cout

### 视觉层次
- 主角：两个 ⊕ 加法节点（最大、加粗）
- 配角：卷积块矩形
- 背景：箭头与维度标签

## ASCII 示意稿

```
┌─── identity short-cut ────────┐    ┌─── projection short-cut ──────────┐
│                               │    │                                   │
│         X (C,H,W)             │    │         X (C,H,W)                 │
│           │                   │    │           │                       │
│      ┌────┴────┐              │    │      ┌────┴────┐                  │
│      │         │              │    │      │         │                  │
│      ▼         │ identity     │    │      ▼         ▼                  │
│  Conv 3x3      │              │    │  Conv 3x3   Conv 1x1              │
│   C→C          │              │    │  C→C'/s=2   C→C'/s=2              │
│   BN+ReLU      │              │    │  BN+ReLU    BN                    │
│      │         │              │    │      │         │                  │
│      ▼         │              │    │      ▼         │                  │
│  Conv 3x3      │              │    │  Conv 3x3      │                  │
│   C→C          │              │    │  C'→C'         │                  │
│      │         │              │    │      │         │                  │
│      ▼         ▼              │    │      ▼         ▼                  │
│      ⊕ ◄───────┘              │    │      ⊕ ◄───────┘                  │
│      │  通道/空间一致         │    │      │  1x1 调形后相加            │
│      ▼                        │    │      ▼                            │
│    ReLU                       │    │    ReLU                           │
│      │                        │    │      │                            │
│      ▼                        │    │      ▼                            │
│      Y (C,H,W)                │    │      Y (C',H/2,W/2)               │
│                               │    │                                   │
│ IFB region 共享               │    │ projection 分支独占 region        │
└───────────────────────────────┘    └───────────────────────────────────┘
```

## 数据来源
- ResNet 原始论文（He et al. CVPR'16）— literature.md ResNet 条目
- 加速器侧 IFB region 划分：paper.md §2.2 段、§4.2.5 SDP

## 与正文一致性检查
- [x] identity 要求"通道数与空间形状一致"——与 §2.2 表述一致
- [x] projection 要求 "1×1 卷积调整 short-cut 分支"——与 §2.2 表述一致
- [x] IFB region 共享 / 独占措辞与 §4.2.5 一致

## 不确定项
无。
