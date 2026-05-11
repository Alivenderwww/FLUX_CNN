# 表 5.6 PE 利用率三模式（baseline / Ky-fold / S2D）与 wall cycles 对比
# Table 5.6 PE utilisation and wall cycles in three compiler modes (baseline / Ky-fold / S2D)

## 在论文中的角色
- 首次引入：§5.5 段 "三模式具体数字与归因详见 §5.5.2 与表 5-6 [CHECK: 三模式整网 PE 利用率与 wall cycles 数字]"
- 论证作用：narrative A 主轴最关键的数据兑现表——直接证明编译器侧 Ky-fold / S2D 在 PE 利用率与 wall cycles 上的具体贡献。对应贡献 C4.7 / C2.1 / C2.2。

## 列定义
| 列名 | 含义 | 数据来源 |
|------|------|---------|
| 层 / 网络 | 哪个层 / 整网 | model_analysis.md / STATUS.md |
| baseline | 不启用编译器优化 | 22-case 回归 baseline 模式 |
| Ky-fold | 仅启用 Ky-fold | 22-case 回归 fold 模式 |
| S2D | 启用 S2D（含 fold 自动联合）| 22-case 回归 fold + s2d 模式 |
| 加速比 | S2D / baseline | 计算 |

## 行定义
- 入口 Patch 层（K=4 / stride=4 / Cin=3，本工作最显著的塌陷场景）
- 一组中间小通道层（Cin < 16）
- 一组中间下采样层（stride=2）
- 一组中间正常层（无塌陷，作对照）
- ResNet-11 整网

## 列指标（每行三组数字）
- PE 行利用率 (%)
- PE 列利用率 (%)
- wall cycles

## 表初稿

| 层 / 网络 | 形状 | baseline 行利 / 列利 / cycles | Ky-fold 行利 / 列利 / cycles | S2D 行利 / 列利 / cycles | S2D 加速比 |
|-----------|------|------------------------------|------------------------------|-----------------------------|----------|
| **Patch** | (Cin=3, Cout=16, K=4, S=4) | 19% / 100% / **654,404** | 不适用（stride=4 不触发 fold 优先级） | **100% / 100% / 129,594** | **5.05×** |
| **ResBlock1.conv1** | (Cin=8, K=3, S=1) | 50% / 100% / [CHECK] | **75%~100% / 100% / [CHECK]** | 不触发 S2D（stride=1） | [CHECK] |
| **ResBlock2.conv1** | (Cin=16, K=3, S=2) | 100% / 100% / [CHECK] | 不触发 fold（Cin≥16） | **100% / 100% / [CHECK]** | [CHECK] |
| **ResBlock3.conv2** | (Cin=64, K=3, S=1) | 100% / 100% / [CHECK] | 不触发 | 不触发 | 1.0× |
| **ResNet-11 整网 N=1** | — | [CHECK: ~1,115K cycles] | [CHECK] | **596,088 cy / 313 fps @ target** | **~1.87×** |

[CHECK: 上表大部分中间层数字来自 22-case 回归报告，需要 paper-project-analyst 抽取整理 model_analysis.md 与 STATUS.md §2.8 的具体数值后回填]

## 数据来源
- paper.md §5.5 / §4.3
- contributions.md C2.1 / C2.2 / C4.7
- STATUS.md §2.8
- 22-case 回归报告（baseline / fold / fold+s2d 三组）
- model_analysis.md ResNet-11 各层 PE 利用率分析

## 与正文一致性检查
- [x] Patch 层 654,404 → 129,594 cycles / 5.05× — 与 STATUS.md §2.8 / §5.5 一致
- [x] ResNet-11 N=1 596,088 cycles / 313 fps — 与 §5.5 一致
- [x] Cin < 16 → Ky-fold；stride ≥ 2 → S2D 决策 — 与 §4.3.5 一致

## 不确定项
- 大部分中间层数字 [CHECK]，需要从 22-case 回归报告 + model_analysis.md 抽取 — 由 paper-project-analyst 复核
- Ky-fold 模式下"行利用率 75%~100%"的具体值取决于 (Cin, K) 组合，需要按 compute_fold_params 公式逐层算 [CHECK]
- ResNet-11 整网 baseline ~1,115K cycles 数字 [CHECK]：是否有"全 baseline" 整网回归数据，待 STATUS.md 复核
