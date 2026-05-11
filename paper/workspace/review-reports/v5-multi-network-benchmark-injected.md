# v5 多网络基准 — 注入 paper.md 摘要

- **方案**：方案 A（分两块插入）
- **工具调用预算**：6 / 12 (Read 4 + Edit 2 + Grep 2 + Write 1)

## 写入位置与新增表号

1. **§5.5 末新增 §5.5.6 多网络ASIC工艺折算性能**（在原 §5.5.5 末与 §5.6 章首之间）
   - 引言段：直接给出 analytical model 口径与 ASIC 850 MHz 频率折算依据
   - 表 5.13 — 7 个代表性 CNN 网络（ResNet11/AlexNet/VGG-16/ResNet-18/ResNet-50/MobileNet-V1/YOLOv2-tiny）@ ASIC 850 MHz 主表
   - 表 5.14 — 同 7 个网络在 FPGA 148.5 MHz 下的对照附表
   - 5.72× 频率比定性段
   - **数据口径与局限性段**（4 点：analytical 偏保守 18% / ASIC 频率未做后端 PnR / 大核 K>8 round chunking / ResNet11 仿真校准值 4,471 fps 优先）

2. **§5.6 内 §5.6 末追加横向对比扩展**（在 "143.8 MHz 仍低于... 列入后续工作" 后、§5.7 章首前）
   - 表 5.15 — vs CPU FP32 单线程加速比（5 个网络，6.7×–16.5×）
   - 表 5.16 — vs Eyeriss / Eyeriss v2 / NVDLA / VTA / Gemmini / Peng / OPU 在 AlexNet/VGG-16/ResNet-50 上的网络级 fps 对比
   - 关键结论："FLUX_CNN ASIC 在 ResNet-50 上达 113 fps，与同 16 nm 工艺 NVDLA 量级相当，且优于 NVDLA 估算约 1.88 倍"
   - **表 5.16 局限性段**（3 点：50% util 估算 / 工艺频率差异大 / 实际投片 Fmax 区间）

## 数据局限性标注位置

- §5.5.6 末段（紧跟表 5.14 后），4 条
- §5.6 表 5.16 后段，3 条

## 红线自检结果（grep 全文 0 命中）

- `详见\s*§` / `见第\s*\d+章` / `本工作` / `本节给出` / `本节针对` / `让读者` / `为后续\s*§` / `下文先在` / `本章后续` / `commit\s+[a-f0-9]{7}` / `Round [A-Z]` / `FLUX_MAC_SIMD` / `IDEAL_SMC` / `工程陷阱` / `工程教训` / `sign correction` / `USE_DSP` / `多轮迭代` / `4 轮迭代` 全部 0 命中
- 引言段无"首先/接着/然后/最后"序列
- 表号 5.13–5.16 续接 §5.5.5 表 5.12 无冲突
