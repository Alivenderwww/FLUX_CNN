# 表 3.2 目标算法 PE 利用率塌陷场景定义
# Table 3.2 Definition of PE-utilisation collapse scenarios on the target algorithm set

## 在论文中的角色
- 首次引入：§3.2 段 "在该算法约束下需要明确指出，Cin < 16 与 stride ≥ 2 是固定 16×16 阵列上 PE 利用率塌陷的两类典型场景..." [依赖: Tab.3.2]
- 论证作用：把"PE 利用率塌陷"在 ResNet-11 上的具体层量化展示，让读者在 §3.2 即理解后续 §4.3 编译器侧 Ky-fold / S2D 的攻击面。

## 列定义
| 列名 | 含义 | 数据来源 |
|------|------|---------|
| 层名 | ResNet-11 中的层标识 | model_analysis.md |
| 形状 | (Cin, Cout, K, stride, pad, H, W) | model_analysis.md |
| 塌陷类型 | Cin < 16 / stride ≥ 2 / 两者并存 / 无 | 推导 |
| baseline PE 行利用率 | min(Cin, 16)/16 | 计算 |
| baseline PE 列利用率 | min(Cout, 16)/16 | 计算 |
| 编译器优化 | Ky-fold / S2D / S2D + Ky-fold / 无需 | §4.3.5 决策表 |

## 行定义（部分关键层，全表见 model_analysis.md）
- Patch 层（K=4, stride=4, Cin=3）→ S2D
- ResBlock1 conv 1（Cin=8, K=3, stride=1）→ Ky-fold
- ResBlock1 conv 2（Cin=16, K=3, stride=1）→ 无
- ResBlock2 conv 1（Cin=16, K=3, stride=2）→ S2D
- ...

## 表初稿

| 层名 | 形状 (Cin, Cout, K, S, pad) | 塌陷类型 | baseline 行利用率 | baseline 列利用率 | 编译器优化 |
|------|----------------------------|---------|------------------|------------------|----------|
| Patch | (3, 16, 4, 4, 2) | **Cin<16 + stride≥2** | 3/16=19% | 16/16=100% | **S2D** → cin_eff=48, K_eff=1 |
| ResBlock1.conv1 | (8, 16, 3, 1, 1) | Cin<16 | 8/16=50% | 16/16=100% | **Ky-fold** → cin_eff=24, kyper=1 [CHECK] |
| ResBlock1.conv2 | (16, 16, 3, 1, 1) | 无 | 16/16=100% | 16/16=100% | 无需 |
| ResBlock2.conv1 | (16, 32, 3, 2, 1) | stride≥2 | 16/16=100% | 16/16=100%（Cout slice 跑 2 次） | **S2D** → cin_eff=64, K_eff=2 [CHECK] |
| ResBlock2.conv2 | (32, 32, 3, 1, 1) | 无 | 16/16=100% | 16/16=100% | 无需 |
| ResBlock3.conv1 | (32, 64, 3, 2, 1) | stride≥2 | 16/16=100% | 16/16=100% | **S2D** [CHECK] |
| ... | ... | ... | ... | ... | ... |
| FC | (?, 10, 1, 1, 0) | Cout<16 | 16/16 | 10/16=62% [CHECK] | 无（Cout<16 硬件不复用，§4.3 诚实陈述代价） |

[CHECK: 全表行均需用 model_analysis.md 复核 Cin/Cout/K/stride 与是否触发 S2D / Ky-fold]

## 与正文一致性检查
- [x] Cin < 16 / stride ≥ 2 / 联合 三种塌陷类型 — 与 §3.2 / §4.3.1 一致
- [x] Patch 层 (3, 16, 4, 4, 2) 是 S2D 的 prime example — 与 §4.3.4 一致
- [x] FC 层 Cout < 16 列空转 — 与 §4.3 末段诚实陈述一致

## 不确定项
- 大部分行 [CHECK]，需要 model_analysis.md 详细 ResNet-11 层表回填具体形状与决策 — 由 paper-project-analyst 提供（如需）
- baseline 列利用率：Cout slice 跑多次时如何标注？建议在表脚注说明"Cout slice 跑 ⌈Cout/16⌉ 次，每次列利用率 = min(Cout_slice, 16)/16"
