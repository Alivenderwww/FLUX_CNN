# 图 4.8 PyTorch→ISA 端到端编译流
# Figure 4.8 End-to-end PyTorch-to-ISA compilation flow

## 在论文中的角色
- 首次引入：§4.4 段 "PyTorch→ISA 端到端编译流由 toolchain/models/run_model.py 主入口..." [依赖: Fig.4.8]
- 论证作用：展示"PyTorch nn.Module + 样本图像 → 加速器可执行 descriptor list"的端到端编译流，是 narrative B（系统集成）的关键示意。重点是各编译阶段的关键模块名（run_model.py / hw_files.py / scheduler.py / gen_isa_test.py）+ Ky-fold/S2D 联合触发的位置。对应贡献 C2.4。

## 图类型
DAG 流程图：自上而下 5-6 个阶段，每个阶段画方框 + 阶段产物（中间表示）。

## 设计要素

### 必含元素
- **阶段 0（输入）**：`PyTorch nn.Module` + `样本图像（用于校准量化 scale/zp）`
- **阶段 1（前端解析）**：`models/run_model.py` 主入口 → 提取每层 (Cin, Cout, K, stride, pad, H, W) → 输出 layer_list (Python list)
- **阶段 2（INT8 量化校准）**：每层做 PyTorch 后量化（INT8 对称量化）→ 输出每层 (scale, shift_n, zp, bias)
- **阶段 3（PE 利用率优化决策）**：`scheduler.Layer.force_s2d() / force_fold()` → 按 (Cin, K, stride) 阈值规则做 S2D / Ky-fold 自动决策 → 输出每层 (mode ∈ {baseline, ky-fold, s2d, s2d+fold}, cin_eff, ky_eff, kyper)
- **阶段 4（cfg 派生 single source）**：`hw_files.derive_layer_cfg(cin_eff, ky_eff, ...)` → 输出每层 cfg_dict（字段名与 RTL cfg_regs 一一对应）
- **阶段 5（权重 / 激活打包 + descriptor 生成）**：
  - `fold_weights()` / `s2d_weights()` 按 16×16 PE 阵列 OS+列广播次序打包
  - `build_step_cfg_dict()` + `descriptor_writer` 生成 CFG_WRITE descriptor list
  - 输出：`descriptor list (.bin)` + `打包权重 (.bin)` + `输入激活 (.bin)`
- **阶段 6（部署 / 仿真）**：`run_regression.py` / 板级 host 加载到 DDR + AXI-Lite 启动 → RTL 跑 → 与 PyTorch reference bit-exact 对比

### 标注要求
- 每个阶段框标"主入口模块名 + 关键产物"
- 阶段 3（决策）用红色高亮：标 "narrative A 核心决策点：S2D / Ky-fold / 联合触发"
- 阶段 4（cfg 派生）用紫色高亮：标 "single source — `derive_layer_cfg()` 唯一函数，编译器与 RTL 共享"
- 右侧加一栏"中间表示流"——layer_list → 量化参数 → mode 决策 → cfg_dict → descriptor list（每个产物画一个小立方体）
- 左下角加一句："`python models/run_model.py --model xxx --image-dir xxx --limit 10` 一行命令跑通完整模型部署"

### 视觉层次
- 主角：6 阶段流程主链
- 配角：右侧中间表示流
- 背景：底部命令行示例

## ASCII 示意稿

```
   ┌─────────────────────────────────────────────────────────┐
   │ 阶段 0：输入                                            │
   │  · PyTorch nn.Module (model.pth)                        │
   │  · 校准样本图像（用于量化）                             │
   └────────────────────┬────────────────────────────────────┘
                        ▼
   ┌─────────────────────────────────────────────────────────┐
   │ 阶段 1：前端解析                                        │
   │  models/run_model.py 主入口                              │
   │  → 提取每层 (Cin, Cout, K, stride, pad, H, W)            │
   │  ────产物：layer_list (Python list)──→                  │
   └────────────────────┬────────────────────────────────────┘
                        ▼
   ┌─────────────────────────────────────────────────────────┐
   │ 阶段 2：INT8 量化校准                                    │
   │  PyTorch 后量化 (对称 INT8)                              │
   │  ────产物：每层 (scale, shift_n, zp, bias)──→           │
   └────────────────────┬────────────────────────────────────┘
                        ▼
   ┌──── 阶段 3：PE 利用率优化决策 (★ narrative A 核心 ★) ──┐
   │  scheduler.Layer.force_s2d() / force_fold()             │
   │  · S2D 触发：stride ≥ 2 AND K ≥ stride                  │
   │  · Ky-fold 触发：K > 1 AND Cin' < 16 (S2D 后 Cin')       │
   │  · 联合：先 S2D，再判 Ky-fold                            │
   │  ────产物：mode ∈ {baseline, ky-fold, s2d, s2d+fold}──→ │
   │            (cin_eff, ky_eff, kyper, ...)                │
   └────────────────────┬────────────────────────────────────┘
                        ▼
   ┌──── 阶段 4：cfg 派生 single source (★ §3.5 原则 ★) ────┐
   │  hw_files.derive_layer_cfg(cin_eff, ky_eff, ...)        │
   │  唯一函数，编译器与 RTL testbench 共享调用              │
   │  ────产物：每层 cfg_dict (与 cfg_regs 字段一一对应)──→  │
   └────────────────────┬────────────────────────────────────┘
                        ▼
   ┌─────────────────────────────────────────────────────────┐
   │ 阶段 5：权重打包 + descriptor 生成                       │
   │  · fold_weights() / s2d_weights() 按 OS+列广播次序打包  │
   │  · build_step_cfg_dict() + descriptor_writer            │
   │  ────产物：descriptor list (.bin)                       │
   │            打包权重 (.bin)                              │
   │            输入激活 (.bin)                              │
   └────────────────────┬────────────────────────────────────┘
                        ▼
   ┌─────────────────────────────────────────────────────────┐
   │ 阶段 6：部署 / 仿真                                      │
   │  · run_regression.py (仿真) → bit-exact vs PyTorch ref  │
   │  · 板级：host 加载 .bin 到 DDR + AXI-Lite 启动 → RTL    │
   └─────────────────────────────────────────────────────────┘

   一行命令跑通完整模型部署：
     python models/run_model.py --model mnist_allconv \
                                 --image-dir models/images/mnist_test \
                                 --limit 10
```

## 数据来源
- paper.md §4.4 段 "PyTorch→ISA 端到端编译流..."
- contributions.md C2.4
- CLAUDE.md "常用命令" 段
- toolchain/models/run_model.py / hw_files.py / scheduler.py / gen_isa_test.py

## 与正文一致性检查
- [x] 主入口 models/run_model.py — 与 §4.4 / CLAUDE.md 一致
- [x] cfg 派生 single source = `hw_files.derive_layer_cfg()` — 与 §3.5 第二条原则 / §4.4 一致
- [x] 决策函数 `scheduler.Layer.force_s2d() / force_fold()` — 与 §4.3 / contributions.md C2.3 一致
- [x] 部署命令 `python models/run_model.py --model ...` — 与 CLAUDE.md 完全一致

## 不确定项
无。
