# 图 3.1 加速器系统总体架构图
# Figure 3.1 Overall system architecture of the accelerator

## 在论文中的角色
- 首次引入：§3.3 总体架构
- 引用位置：paper.md 第 226、228、230 行（§3.3 开篇）
- 论证作用：奠定整篇硬件章节的"控制核 + 加速核 + 多核扩展 + DDR + AXI 互连"的全局视图。读者看到此图后应理解 FLUX_CNN 对外接口（1× AXI4 主 + 1× AXI-Lite 从）与对内层级。

## 图类型
系统级架构框图。

## 设计要素

### 必含元素
1. **控制核（Host CPU）**：右上方块，标注 "ARM / RISC-V"。
2. **片外 DDR**：右下方块，标注 "Off-chip DDR3"。
3. **AXI 互连**：中央，标注 "AXI Interconnect"。
4. **多核扩展层（multicore_top）**：左侧大框，内含 *N* 个加速核（*N* = 1/2/4 标注）。
5. **加速核（Core[i]）**：multicore_top 内重复 *N* 次的小框，每个标 "Core 0 / Core 1 / ..."。
6. **2 个外部接口**：
   - AXI4 完整主接口（粗箭头：multicore_top → AXI 互连 → DDR）
   - AXI-Lite 从接口（细箭头：控制核 → AXI 互连 → multicore_top.csr）
7. **跨核 SRAM 直送通道**（虚线 / dash）：核间 IFB 互通，可标注 "cross-core SRAM push (N≥2)"。

### 标注要求
- 在 AXI4 主线上标注 "BUS_DATA_W=128"
- 在 AXI-Lite 从线上标注 "CSR_DATA_W=32"
- 每核标注 "16×16 INT8 MAC"
- 多核地址映射：Core[i] IFB `0x8000_0000 + i × 0x1000_0000`（可放图注）

### 视觉层次
- 主角：multicore_top 大框（含 *N* 个加速核）
- 配角：DDR、控制核、AXI 互连
- 背景：跨核 SRAM 直送虚线（仅 *N*≥2 时存在）

## 数据来源
- contributions.md C1 系列（架构）
- CLAUDE.md "顶层结构两层" 段
- STATUS.md §1 / §2

## ASCII 示意稿

```
                              AXI-Lite (32b)
   ┌──────────────┐  ──────────────────────────▶  ┌─────────────────────────┐
   │  Host CPU    │                               │     AXI Interconnect    │
   │ (ARM/RISC-V) │  ◀──── done IRQ ───────       └──┬────────────────┬─────┘
   └──────────────┘                                  │ AXI4 (128b)    │ AXI4
                                                     │                │
                                                     ▼                ▼
                                          ┌──────────────────────────────────┐
                                          │   multicore_top (N=1/2/4)        │
                                          │  ┌──────┐ ┌──────┐ ┌──────┐      │
                                          │  │Core0 │ │Core1 │ │ ...  │      │
                                          │  │16×16 │ │16×16 │ │      │      │
                                          │  │INT8  │ │INT8  │ │      │      │
                                          │  │MAC   │ │MAC   │ │      │      │
                                          │  └──┬───┘ └──┬───┘ └──┬───┘      │
                                          │     └─ x-core SRAM push ─┘ (dash)│
                                          └────────────┬─────────────────────┘
                                                       │ AXI4 (128b)
                                                       ▼
                                                ┌────────────┐
                                                │ Off-chip   │
                                                │ DDR3       │
                                                └────────────┘
```

## 与正文的一致性检查
- [x] §3.3 "加速器对外仅暴露两个接口：一个 AXI4 完整主接口与一个 AXI-Lite 从接口" — 图中两根接口线
- [x] §3.3 "多核扩展层包含 *N* 个加速核（*N*=1/2/4 已综合通过）" — 图中标注 *N*=1/2/4
- [x] 跨核 SRAM 直送在 §3.3 段末 + §4.11 详述 — 图中以虚线提示

## 不确定项
- [TBD: 是否在主图中显示控制核以外的外设（如 USB / Camera Sensor）] — 倾向不显示，保持架构图纯净
