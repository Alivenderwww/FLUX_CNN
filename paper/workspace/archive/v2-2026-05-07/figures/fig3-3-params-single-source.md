# 图 3.3 单源参数 params.py 双向消费流程
# Figure 3.3 Single-source parameter flow of params.py consumed by RTL and Python toolchain

## 在论文中的角色
- 首次引入：§3.5 段 "第一条原则是单源参数 params.py..." [依赖: Fig.3.3]
- 论证作用：用流程图展示"一份 Python 常量 → 自动生成 RTL svh + Python 直接 import"的双向消费机制，把 §3.5 的"单源参数"方法论原则可视化。是 §4.5 多核扩展中"改 NUM_CORE 即可重新派生"工程实践的图示根据。

## 图类型
DAG 流程图：源中心 + 双向箭头 + 两条消费路径。

## 设计要素

### 必含元素
- **中心节点**：`params.py` 圆角框（含 `NUM_COL=NUM_PE=16 / WRF=ARF=PARF=32 / DATA=8 / PSUM=32 / IFB=8192 / WB=1024 / OFB=2048 / BUS_DATA_W=128 / CSR_DATA_W=32 / NUM_CORE=4`）
- **左侧消费路径（RTL）**：`params.py` ──`python params.py`──▶ `RTL/flux_cnn_params.svh` （含 `\`define FLUX_NUM_PE 16` 等宏） ──`\`include`──▶ RTL 模块（line_buffer.sv / mac_array.sv / parf_accum.sv / ofb_writer.sv / wgt_buffer.sv / cfg_regs.sv / multicore_top.sv 等）
- **右侧消费路径（Python toolchain）**：`params.py` ──`from params import *`──▶ Python toolchain（gen_isa_test.py / hw_files.py / models/run_model.py / scheduler.py / run_regression.py）
- **两侧底部汇总框**：分别标 "RTL 端 `\`FLUX_*` 宏引用" 与 "Python 端 `from params import *`"
- **底部一句话**："改 params.py 一处 → RTL 与 Python 同步消费 → 不存在双向解释偏移"

### 标注要求
- 中心节点 params.py 用紫色高亮
- 左侧 RTL 路径用蓝色，右侧 Python 路径用绿色
- 自动生成步骤的命令 "python params.py" 用等宽字体单独标注
- 在两侧消费节点下方分别加小注：左 "改硬件常量自动同步到 RTL"，右 "改硬件常量自动同步到编译器"

### 视觉层次
- 主角：params.py 中心节点（最大、最深色）
- 配角：左右两条路径
- 背景：底部一句话总结

## ASCII 示意稿

```
                          ┌───────────────────────────────────┐
                          │           params.py               │
                          │  (项目根，唯一参数源)             │
                          │                                   │
                          │  NUM_COL = NUM_PE = 16            │
                          │  WRF = ARF = PARF = 32            │
                          │  DATA = 8, PSUM = 32              │
                          │  IFB = 8192, WB = 1024            │
                          │  OFB = 2048                       │
                          │  BUS_DATA_W = 128                 │
                          │  CSR_DATA_W = 32                  │
                          │  NUM_CORE = 4                     │
                          └────────┬───────────────┬──────────┘
                                   │               │
                  python params.py │               │ from params import *
                       (代码生成)  ▼               ▼
                                                  
                  ┌─────────────────────┐    ┌─────────────────────┐
                  │ RTL/flux_cnn_       │    │ Python toolchain    │
                  │ params.svh          │    │                     │
                  │                     │    │ - gen_isa_test.py   │
                  │ `define FLUX_NUM_PE │    │ - hw_files.py       │
                  │         16          │    │ - scheduler.py      │
                  │ `define FLUX_IFB    │    │ - run_regression.py │
                  │         8192        │    │ - models/           │
                  │ ...                 │    │   run_model.py      │
                  └─────────┬───────────┘    └─────────┬───────────┘
                            │ `include                 │ 直接引用
                            ▼                          ▼
                  ┌─────────────────────┐    ┌─────────────────────┐
                  │ RTL 模块            │    │ 编译器 / 仿真驱动   │
                  │                     │    │                     │
                  │ line_buffer.sv      │    │ derive_layer_cfg()  │
                  │ mac_array.sv        │    │ compute_fold_params │
                  │ parf_accum.sv       │    │ compute_s2d_params  │
                  │ ofb_writer.sv       │    │ build_step_cfg_dict │
                  │ wgt_buffer.sv       │    │ ...                 │
                  │ cfg_regs.sv         │    │                     │
                  │ multicore_top.sv    │    │                     │
                  └─────────────────────┘    └─────────────────────┘

   改 params.py 一处 → 重跑 `python params.py` → RTL svh 与 Python 同步消费
   → 不存在"忘了改另一侧"导致的双向解释偏移
```

## 数据来源
- CLAUDE.md "配置 & 代码约定" 段
- paper.md §3.5 / §4.5
- contributions.md C3.5
- params.py 实际内容

## 与正文一致性检查
- [x] 参数列表（NUM_COL/NUM_PE/WRF/ARF/PARF/DATA/PSUM/IFB/WB/OFB/BUS_DATA_W/CSR_DATA_W）与 CLAUDE.md / paper.md §3.5 一致
- [x] "改硬件常量只需改一处"——与 §3.5 表述一致
- [x] 两条消费路径（RTL 端 `\`include`、Python 端 `from params import *`）与 CLAUDE.md 表述一致

## 不确定项
无。
