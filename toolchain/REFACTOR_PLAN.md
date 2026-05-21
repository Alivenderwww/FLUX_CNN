# Toolchain 重构计划（v2）

把当前 monolithic toolchain 重构成清晰前/中/后端三段编译链。

## 1. 全局视图

```
┌─────────────────────────────────────────────────────────────┐
│ Stage 0: Board Bring-up (一次性 setup, 不属编译链)           │
│   hardware.json + RTL  → Vivado synth → bitstream.pdi      │
│   echo.c (RPC server)  → A72 ELF                           │
│   bootgen + JTAG 烧    → 板常驻服务                          │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ Stage 1.0: 量化 (offline, per-model 一次)                    │
│   model.pt + calib_dataset/  → quantized.yaml (+ int8.npy)  │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ Stage 1.1: 编译链 (per-case)                                 │
│                                                              │
│   hardware.json + quantized.yaml + [opt input.png]          │
│        ↓ Frontend (PyTorch / YAML → Chain DSL)              │
│   ChainDSL IR  (Layer[] + scale + weights ref)              │
│        ↓ Midend (scheduler + SMC allocator)                 │
│   PlannedSchedule IR  (per_core_plan + dynamic SMC layout)  │
│        ↓ Backend (cfg + desc + SG cmd + data + manifest)    │
│   case/  (manifest.json + chain_data/ + coreX/*.hex)        │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ Stage 2: Runtime (per-inference)                            │
│   case/ + 板 IP + input.png  → deploy                       │
│   → 推理结果 + bit-exact verify                              │
└─────────────────────────────────────────────────────────────┘
```

## 2. 设计决策

| # | 决策 | 选择 | 理由 |
|---|------|------|------|
| 1 | **JSON vs params.py** | JSON 替代（params.py 作为生成 cache） | 单源避免双 source 同步；老 import 0 改动 |
| 2 | **JSON schema** | 两层：`arch` (RTL 常量) + `deployment` (board-specific) | 同 RTL 多部署；语义清晰 |
| 3 | **编译链架构** | Python orchestrator + class 边界（option B） | 改动量小；支持 `--dump-ir` 可选导出 |
| 4 | **前端输入 unify** | 所有源 → Chain DSL → 单 frontend；DSL 用 YAML 序列化 | DSL 已成熟支持 residual；未来加 ONNX 只需 `xxx_to_dsl.py` |
| 5 | **中端职责** | 含调度 + SMC 动态 allocator；hw cfg derive 归后端 | 解决 patch1 类 IFM/OFM 重叠 bug 的根本 |
| 6 | **case 目录格式** | 渐进：保留 .hex/.txt（sim 兼容）+ 加 manifest.json | sim 0 改动；host 升级；manifest 含 hw cfg |
| 7 | **目录重构** | 大重构：`frontend/midend/backend/ir/hardware/` 子目录 | 散件堆放转为分层；旧脚本保留 wrapper |
| 8 | **CLI 入口** | 量化独立；compile 支持 .pt/.yaml 双输入；deploy 独立 CLI | 量化稳态后 case 跟 image 解耦 |

## 3. 目标目录结构

```
toolchain/                         ← 编译链 (Stage 1)
├── frontend/
│   ├── pytorch_to_dsl.py          ← models/compile_model.py 重构
│   ├── dsl_loader.py              ← YAML / Python Chain DSL 加载
│   └── quantize.py                ← per-tensor int8 量化（独立 stage）
├── midend/
│   ├── scheduler.py               ← schedule() + Stage + Step
│   └── smc_allocator.py           ← 动态 SMC layout 分配
├── backend/
│   ├── hw_cfg_derive.py           ← derive_layer_cfg (现 hw_files.py)
│   ├── desc_emit.py               ← desc list .hex
│   ├── sg_cmd_emit.py             ← SG cmd (现 mesh_cmd.py)
│   ├── data_emit.py               ← ifb/wb/rdma 数据 (现 compile_layer.py)
│   └── manifest_emit.py           ← manifest.json
├── ir/
│   ├── chain_dsl.py               ← Layer dataclass (前端 IR)
│   ├── planned_schedule.py        ← PlannedSchedule (中端 IR)
│   └── manifest.py                ← Manifest (case 元数据)
├── hardware/
│   ├── hardware_cfg.py            ← loader class
│   ├── json_to_params.py          ← JSON → params.py cache
│   ├── json_to_svh.py             ← JSON → flux_cnn_params.svh
│   ├── vd100.json                 ← VD100 部署 config
│   └── k325t.json                 ← K325T 部署 config
├── compile.py                     ← CLI 入口：unified front/mid/back
├── quantize.py                    ← CLI 入口：独立量化 stage
├── run_multicore_chain.py         ← deprecated thin wrapper (兼容)
├── run_regression.py              ← 仍可用
└── REFACTOR_PLAN.md               ← 本文档
```

## 4. 实施 Roadmap（按 A→F 顺序，最稳推进）

每阶段独立可验证 / 可回滚，gate PASS 才进下阶段。

### Phase A — 硬件 JSON 准备（不破坏现状，1-2 天）

1. 写 `hardware/vd100.json`（复制 params.py + run_multicore_chain.py 顶部 SMC 常量）
2. 写 `hardware/hardware_cfg.py` (loader class) + `json_to_params.py` + `json_to_svh.py`
3. 跑一次 → 生成 `params.py` + `flux_cnn_params.svh` 跟现状**字节一致**
4. **Gate**：sim 全回归 + board sweep_board_cases PASS（验证 0 破坏）

### Phase B — IR 抽取 + Manifest（2-3 天）

1. 新增 `ir/chain_dsl.py`（从 scheduler.Layer 包装）+ YAML loader/dumper
2. 新增 `ir/manifest.py` + `backend/manifest_emit.py` → case 目录加 `manifest.json`
3. manifest.json 含：schema_version、hw cfg、per-layer SMC layout、derive 后硬件 cfg
4. host `deploy_smc_case.py` 加 `try manifest.json else fallback multicore_meta.txt`
5. **Gate**：sim 不动通过，host sweep 用 manifest 跟用 meta 结果 bit-exact 一致

### Phase C — 大重构目录（3-4 天）

1. 创建 `frontend/midend/backend/ir/hardware/` 子目录
2. 移动文件 + rename：
   - `scheduler.py → midend/scheduler.py`
   - `hw_files.py → backend/hw_cfg_derive.py`
   - `mesh_cmd.py → backend/sg_cmd_emit.py`
   - `compile_layer.py → backend/data_emit.py`
   - `models/compile_model.py → frontend/pytorch_to_dsl.py`
3. 旧脚本（`run_multicore_chain.py`、`run_regression.py`、`run_model.py`）改成 thin wrapper 调新模块
4. 新 CLI 入口 `toolchain/compile.py`
5. **Gate**：旧 CLI + 新 CLI 双跑同一 case，**二进制一致**

### Phase D — SMC 动态 allocator（2-3 天，根治 patch1 类 bug）

1. `midend/smc_allocator.py`：按 layer 实际 size 动态算地址，不再 hardcode 1MB 偏移
2. backend 写 manifest 用 allocator 输出
3. host deploy 100% 从 manifest 读地址（删 SMC_INPUT_BASE 等 hardcode 常量）
4. **Gate**：手工构造 IFM/OFM 重叠场景（如 patch1），验证 allocator 自动规避；sweep_board_cases 全 PASS

### Phase E — 量化 stage 分离（2-3 天）

1. 抽 `frontend/quantize.py`（calibration + scale 计算 + int8 weights 量化）
2. CLI: `python -m toolchain.quantize --model X.pt --calib-dir Y → X.quant.yaml`
3. `compile.py` 接受 `.quant.yaml` 或 `.pt`（向后兼容自动 calib）
4. **Gate**：mnist_allconv 量化 → 编译 → board 推理 PASS；同 model 不同 input image 复用同 case

### Phase F — 旧 CLI deprecate（0.5 天）

1. wrapper 标 deprecated warning
2. 一两个版本后删除

## 5. IR / Schema 设计

### hardware.json schema

```json
{
  "arch": {
    "num_pe": 16, "num_col": 16, "data_width": 8, "psum_width": 32,
    "ifb_depth": 1024, "wb_depth": 640, "ofb_depth": 1024,
    "wrf_depth": 32, "arf_depth": 32, "parf_depth": 32,
    "bus_data_width": 128, "csr_data_width": 32, "csr_addr_width": 12
  },
  "deployment": {
    "target": "vd100",
    "num_cores": 1,
    "ddr_base": "0x10000000",
    "ddr_size_mb": 2048,
    "mem_stride": "0x01000000",
    "regions": {
      "ifm_ofm_base":      "0x00000000",
      "wb_base":           "0x00800000",
      "rdma_base":         "0x00900000",
      "desc_base":         "0x00A00000",
      "idma_cmd_base":     "0x00B00000",
      "odma_cmd_base":     "0x00C00000",
      "input_base":        "0x00D00000",
      "final_ofm_base":    "0x00F00000",
      "layer_data_offset": "0x00080000"
    },
    "csr_axil": {"0": "0xA4000000", "1": "0xA4001000"}
  }
}
```

### Chain DSL YAML（前端 IR）

```yaml
schema_version: "1.0"
model_name: resnet11
quant:
  s_in: 0.012345
  s_out: 0.0089
layers:
  - name: L0
    k: 4
    c_in: 4
    c_out: 16
    h_in: 960
    w_in: 540
    stride: 4
    pad: 0
    sdp_shift: 2
    weights_npy: weights/L0_weight.npy
    bias_npy: weights/L0_bias.npy
  - name: L1
    k: 3
    # ...
  - name: L_res
    input_src: L0
    shortcut_mult: 128
    shortcut_shift: 7
```

### PlannedSchedule（中端 IR，可选 dump）

```yaml
n_cores: 1
stages:
  - layers: [L0, L1]
    assignments:
      - layer: L0
        mode: A
        cores: [0]
        cycles_estimate: 281208
      - layer: L1
        mode: C_w_slice
        cores: [0, 1]
        sub_w: 68
smc_layout:
  L0:
    "core_0":
      ifb_base: "0x10D00000"
      ofm_base: "0x10F00000"
      wb_base:  "0x10800000"
      # ...
```

### Manifest.json（后端 case 元数据）

```json
{
  "schema_version": "1.0",
  "toolchain_commit": "<git sha>",
  "hardware_cfg": "vd100.json",
  "n_cores": 1,
  "layers": [
    {
      "idx": 0, "name": "L0", "k": 3, "c_in": 16, "...": "...",
      "hw_cfg": {
        "IFB_STRIP_ROWS": 6, "IFB_RING_WORDS": 450, "KK": 9
      },
      "smc_layout": {
        "core_0": {"ifb_base": "0x10D00000", "ofm_base": "0x10F00000"}
      }
    }
  ],
  "deploy": {
    "desc_list_base_per_core": ["0x10A00000"],
    "desc_count_per_core": [63],
    "ofm_final_base": "0x10F00000"
  }
}
```

## 6. 风险 + 回滚策略

| 风险 | 缓解 |
|------|------|
| 重构后 sim 回归挂 | Phase A/B Gate 强制全回归；sim 文件格式 0 改动 |
| 动态 SMC allocator 跟 host hardcode 不同步 | Phase D 之前先 Phase B（host 已迁 manifest），再启用 allocator |
| 旧 case 跟新 toolchain 不兼容 | manifest 含 schema_version，host 兼容多版本读取 |
| 大重构破坏老 CLI | 旧脚本保留 wrapper 一两个版本，渐进 deprecate |

## 7. 何时启动

**不在 board bring-up 关键期启动**。当前 N=2 上板 + ResNet11 完整端到端 验证完成后开始 Phase A。

预计窗口：N=2 sweep PASS / ResNet11 全 layer PASS 后约 1-2 周。

每个 Phase 独立 PR，独立可 review / 回滚。
