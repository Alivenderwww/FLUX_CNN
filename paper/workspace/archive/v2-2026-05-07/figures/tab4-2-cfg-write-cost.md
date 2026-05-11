# 表 4.2 CFG_WRITE descriptor 与裸 AXI-Lite 配置写次数对比
# Table 4.2 Per-layer configuration write counts of CFG_WRITE descriptors versus raw AXI-Lite writes

## 在论文中的角色
- 首次引入：§4.4 段 "host 每层只需写 4 次 AXI-Lite（指针 / 长度 / 启动 / 状态轮询），等效写次数从约 50 / 层降到 4 / 层" [依赖: Fig.4.9, Tab.4.2]
- 论证作用：把 host 配置开销 ~50→4 的优化以单表精确化展示，与图 4.9 时序图配套。

## 列定义
| 列名 | 含义 | 数据来源 |
|------|------|---------|
| 方案 | 朴素 AXI-Lite / CFG_WRITE descriptor | 设计 |
| host 写 / 层 | 单层切换时 host 通过 AXI-Lite 发出的写次数 | contributions.md C3.2 |
| 写内容 | 写哪些字段 / 寄存器 | RTL/cfg_regs.sv 字段表 |
| host 时序压力 | 对 host CPU 的开销定性描述 | 论文表述 |

## 行定义
- 朴素 AXI-Lite 方案（每字段一次写）
- CFG_WRITE descriptor 方案（DFE 自动批量更新）

## 表初稿

| 方案 | host 写 / 层 | 写内容 | host 时序压力 |
|------|------------|-------|------------|
| 朴素 AXI-Lite | ~50 | 逐字段写：cin / cout / K / stride / pad / H / W / mode / scale / shift / zp / strip_rows / pe_fold / s2d_phase / ... 共 ~50 字段 | 高（每层切换 ~50 × t_axi_lite_write，host CPU 时序压力大） |
| **CFG_WRITE descriptor** | **4** | (1) `DESC_PTR_REG` ← descriptor 起始地址；(2) `DESC_LEN_REG` ← descriptor 数量；(3) `START_REG` ← 1（触发 DFE 拉取）；(4) `STATUS_REG` 读 ← 轮询 done sticky | **低（DFE 自动从 DDR 拉 descriptor 批量更新 cfg_regs，host 仅触发与轮询）** |

**整体对比：~50 / 4 ≈ 12.5×** — 多层切换时延对 host 的负担显著降低。

## 与正文一致性检查
- [x] ~50 / 层 vs 4 / 层 — 与 §4.4 / contributions.md C3.2 一致
- [x] 4 次写内容（ptr / len / start / status）— 与 §4.4 / DFE 设计一致
- [x] DFE 双口 cfg_regs 仲裁 — 与图 4.9 / contributions.md C3.3 一致

## 不确定项
- "~50 字段" 数字 [CHECK: cfg_regs 字段总数最新统计]，可由 RTL/cfg_regs.sv 当前版本字段表回查精确数字 — 由 paper-project-analyst 复核（不影响表的元层论断 ~50/4 的量级）
