# 图 4.5 SDP 后处理量化融合组合链
# Figure 4.5 Fused SDP post-processing combinational chain

## 在论文中的角色
- 首次引入：§4.2 段 "第五层骨架是 SDP 后处理融合..." [依赖: Fig.4.5]
- 论证作用：展示 SDP 后处理 (Single Data Processor) 的融合组合链 `pipe_psum_reg → mult → shift → +zp → ReLU → clip → trunc`，重点是这条链全部在单一组合路径完成（无中间寄存器），是当前 critical path 的诚实陈述根据（§5.4 Fmax 仅 68 MHz）。对应贡献 C1.5，并为 §6.3 future work（SDP 流水切分）提供图示。

## 图类型
组合数据通路图：左侧 32-bit PSUM 输入 → 串联多步 → 右侧 8-bit OFB 输出。同时画出可选的残差短接加法分支（在 clip 之前融合）。

## 设计要素

### 必含元素
- **左侧输入**：`pipe_psum_reg`（32-bit PSUM，唯一寄存器）
- **中间组合链**（无寄存器，纯组合）：
  1. `mult` (× scale) — 与量化 scale 因子相乘
  2. `shift` (>> shift_n) — 右移
  3. `+ zp` — 加 zero-point（INT8 量化偏移）
  4. (可选) `+ short-cut` — 残差加（仅 ResNet 残差层启用）
  5. `ReLU` — 激活函数（max(0, x)）
  6. `clip` — 截断到 INT8 范围 [-128, 127]
  7. `trunc` — 取低 8-bit
- **右侧输出**：`OFB INT8 (8-bit)` → ofb_writer ring
- **底部红色虚线框**：包围整条组合链，标 "critical path：68.4 MHz @ XC7K325T（Vivado 2023.1 OOC）"
- **顶部蓝色虚线**：标 "future work（§6.3）：在此处切 1-2 段流水线 → Fmax 拉到 100+ MHz，仅延迟 +1-2 周期"

### 标注要求
- 在每步算子之间画箭头但**无寄存器小框**——明确区分"全组合"与"流水"
- pipe_psum_reg 用方形寄存器标记（D flip-flop 符号）
- 残差短接 `+ short-cut` 用虚线分支，标 "仅 ResNet 残差层；short-cut 来自独立 IFB region (图 2.2 projection 形态)"
- 在 mult 节点旁标 "scale 来自 cfg_regs（每层独立）"，shift 节点旁标 "shift_n 来自 cfg_regs"

### 视觉层次
- 主角：6-7 步组合算子（横向串联）
- 配角：残差短接虚线分支
- 背景：critical path 红色框 + future work 蓝色虚线

## ASCII 示意稿

```
   future work (§6.3)：在此组合链上切 1-2 段流水线 → Fmax 100+ MHz
   ┌─────────────────────────────────────────────────────────────┐
   ╎      <插入流水寄存器>           <插入流水寄存器>            ╎
   ╎                                                             ╎
   └─────────────────────────────────────────────────────────────┘
                              ┊
                              ▼
   ┌──── critical path：组合 (Fmax = 68.4 MHz @ XC7K325T) ────────┐
   │                                                              │
   │  ┌──────────┐   ×scale    >>shift_n    +zp                   │
   │  │  pipe_   │      │         │          │                    │
   │  │  psum_   │──→ mult ──→ shift ──→ + zp ──┐                 │
   │  │  reg     │ 32b       32b      32b     │  +sc (短接 opt)   │
   │  │ (32-bit) │                            ▼                   │
   │  └──────────┘                          ⊕  ←── short-cut       │
   │  ★ 唯一                                  │   (来自 IFB        │
   │   寄存器 ★                                │    region B)       │
   │                                          ▼                   │
   │                                       ReLU → clip → trunc    │
   │                                                    │         │
   │                                                    ▼         │
   │                                           OFB INT8 (8-bit)   │
   │                                                    │         │
   └────────────────────────────────────────────────────│─────────┘
                                                        ▼
                                                ofb_writer ring → odma

   说明：
   · pipe_psum_reg 是该组合链上唯一的寄存器
   · mult / shift / +zp / +short-cut / ReLU / clip / trunc 全部组合实现
   · 这一长组合路径是当前 critical path → §5.4 Fmax 68 MHz 的根因
   · 修复：在 mult 与 shift 之间、shift 与 +zp 之间各切一段流水
        预计 Fmax → 100+ MHz；功能不变，仅延迟 +1-2 周期 (§6.3)
   · scale / shift_n / zp 由 cfg_regs 每层独立提供
   · short-cut 加法仅 ResNet 残差层启用（图 2.2 projection 形态）
```

## 数据来源
- paper.md §4.2 段 5 / §5.4.3 / §6.3
- contributions.md C1.5
- STATUS.md §1 单核已知问题（critical path 与 Fmax 68.4 MHz）
- docs/modules/ofb_writer.md

## 与正文一致性检查
- [x] 组合链顺序 `pipe_psum_reg → mult → shift → +zp → ReLU → clip → trunc` — 与 §5.4 critical path 描述完全一致
- [x] short-cut 加法在 clip 之前融合 — 与 §2.2 / §4.2.5 一致
- [x] Fmax 68.4 MHz、未达 100 MHz target — 与 §5.4.3 / STATUS.md 一致
- [x] future work 切 1-2 段流水至 100+ MHz — 与 §6.3 一致

## 不确定项
无。组合链长度与 critical path 由 STATUS.md 单核综合表确认。
