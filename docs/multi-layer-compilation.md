# Phase G — PyTorch 多层编译

把 `nn.Module` （当前支持 Conv2d + ReLU 串联链）端到端编译到硬件：逐层
量化 → 规划 DDR → 生成 per-layer 硬件文件 → TB 驱动硬件逐层跑 → 终层
比对金标准。硬件零改动（`wgt_buffer / line_buffer / ofb_writer / DMA`
已在 F-2-fix 之后支持干净的跨 layer 重入）。

## 关键洞察

1. **OFB DDR 格式 ≡ IFB DDR 格式**。两者都是 NHWC `(y, x, channel_slice)`
   布局，每 word = 16 个 channel × int8，`AXI_BYTES=16` 对齐一 pixel。
   layer k 硬件把输出直接写到 DDR 后，**无需转置/搬移**即可作为 layer k+1
   的 IFB 读。
2. **硬件支持跨 layer 干净重启**。每次 layer_done 后重新 AXI-Lite 写 cfg
   + CTRL.start_layer 即可。不需要新的 "layer chain" 硬件概念。
3. **跨层 bit-exact 要求用硬件输出做下层输入**。若用 PyTorch float forward
   的 `quantize_symmetric(acts[k+1], s_y_k)` 作为 layer k+1 的 ifb，会和
   layer k 的硬件定点路径有 off-by-1 舍入差异。解决：编译 layer k+1 时
   的整数输入 = layer k 软件模拟的 bit-exact 输出（`ofm_expected_q`）。

## 目录 / 文件

```
toolchain/
├── compile_layer.py      # 单层 Conv2d → 硬件文件（已有；加 DDR base / ifm_int_override）
├── compile_model.py      # Phase G 新增：Module 扫链 + calibration + DDR 规划
├── hw_files.py           # 共享 I/O 层；cfg_to_dict 加 _META_DDR_*_BASE
└── run_regression.py     # CASES 加 ("mnist2", "model", 2) 类入口

sim/tb_core_dma/tb_core_dma.sv
  # preload_ddr 接受 per-layer base；加 skip_ifb_preload / skip_ofb_clear
  # 中间层 OFB 不 clear，且不读 expected_ofm.txt / 不比对

sim/tb_core_dma/cases/
├── caseNN/               # flat 展开的 case 目录（含 single + model 子层）
└── <model_name>_plan/
    └── model_plan.json   # model 元数据（供 debug / 调度追溯）
```

## DDR layout

`compile_model.py` 的 `_plan_ddr()` 把 16 MB DDR 分三段：

| 区域     | 起点      | 止点      | 用途                        |
|----------|-----------|-----------|-----------------------------|
| FM       | `0x00_0000` | `0x80_0000` | Feature map 共享链：FM_0..FM_L |
| WB       | `0x80_0000` | `0xF0_0000` | 每层独立权重区              |
| DESC     | `0xF0_0000` | `0x100_0000`| 每层独立 descriptor 区      |

- FM 共享：`fm_bases[k+1] = fm_bases[k] + align_4K(layer_k.ofb_bytes)`；
  layer k 的 OFB base = layer k+1 的 IFB base
- 每层独立 WB base，按 `cfg['wb_words'] * 256B` 推进
- 每层预留 32 KB descriptor 区（远超 2048 desc）
- 所有 base **4 KB 对齐**
- 超过 region limit 会 raise

## config.txt `_META_*` 字段（Phase G 新增）

```
_META_DDR_IFB_BASE   # byte 地址；TB 用于 preload + 写 IDMA_SRC_BASE
_META_DDR_WB_BASE    # byte 地址；TB 用于 preload + 写 WDMA_SRC_BASE
_META_DDR_OFB_BASE   # byte 地址；TB 用于写 ODMA_DST_BASE + 读 OFB 比对
_META_DDR_DESC_BASE  # byte 地址；TB 用于 preload + 写 DESC_LIST_BASE
_META_SKIP_IFB_PRELOAD  # 非首层=1；TB 跳过 $readmemh ifb.txt（数据来自上层 OFB）
_META_SKIP_OFB_CLEAR    # 非终层=1；TB 不清 OFB（它是下层 IFB），也跳过比对
```

字段缺失时 TB fallback 到 hard-coded localparam（`DDR_IFB_BASE` 等），
保证现有单层回归无感知。

## Scale 传递 / Calibration

`_calibrate_activations(model, x)` 用 PyTorch forward hook 采样：
- `acts[0..L-1]` = 每个 Conv2d 前的输入
- `acts[L]` = 整 model 输出
- `scales[k] = max|acts[k]| / 127`（per-tensor symmetric，最简 max-abs）

layer k 的量化参数：`s_x = scales[k]`, `s_y = scales[k+1]`；mult/shift 从
`derive_mult_shift(s_x * s_w / s_y)` 推出（与单层一致）。

### 层间 bit-exact 保证

`compile_and_emit_conv2d` 加 `ifm_int_override` 参数：
```python
# compile_model.py 里
ifm_override = summaries[k-1]['ofm_expected_q'] if k > 0 else None
compile_and_emit_conv2d(..., ifm_int_override=ifm_override)
```
这样 layer k+1 的整数输入 = layer k `hw_files.compute_expected_ofm` 软件模拟
的输出 = layer k 硬件 RTL 输出。端到端 bit-exact。

## TB 主循环（每 case 一层）

流程（`tb_core_dma.sv`）：
1. `load_config(case_dir)` 读 config.txt，把 `_META_DDR_*` 存到 SV 变量；
   AXI-Lite 写所有 `cfg_regs` 字段（K / stride / Cin / Cout / SDP_MULT / ...）
2. `preload_ddr(...)`：
   - 首层：$readmemh ifb.txt → DDR[`_META_DDR_IFB_BASE`]
   - 每层：wb.txt, desc_list.hex 写各自 base
   - 终层：清 DDR[OFB_BASE..ofb_words]；中间层跳过
3. AXI-Lite 写 `IDMA_SRC_BASE / WDMA_SRC_BASE / ODMA_DST_BASE / DESC_LIST_BASE`
   = 本层 `_META_DDR_*_BASE`
4. `CTRL[4]=1` DFE 拉 descriptor
5. `CTRL[5]=1` 启动 layer；等 `STATUS[11]=layer_done`
6. 终层：读 DDR[OFB_BASE..] vs `expected_ofm.txt` 比对；中间层跳过

下一层照常重入（core 内部计数器在 `evt_start` 脉冲被正确重置；见
`F-2-fix` commit `24c56bc`）。

## 使用

```bash
cd toolchain

# 训 MNIST all-conv 模型 (CPU ~2 分钟, 输出 toolchain/models/mnist_allconv.pt)
.venv/Scripts/python.exe train_mnist.py

# 只跑 model 回归（mnist2 2 层 + mnist_allconv 5 层）
python run_regression.py --only model

# 全回归（35 single + 7 model layers）
python run_regression.py

# 单独调试 compile_model
.venv/Scripts/python.exe compile_model.py --model mnist_allconv \
    --sim-dir ../sim/tb_core_dma --start-idx 0

# 回归 PASS 后验证 MNIST argmax (硬件 int8 vs PyTorch float vs true label)
.venv/Scripts/python.exe validate_mnist.py
```

## MNIST all-conv 模型

不用 Pool，用 stride=2 conv 下采样；末层 K=7 pad=0 当 FC 等价：
```
Conv(1→16, K=3, pad=1, stride=1) + ReLU    28×28 → 28×28
Conv(16→16, K=3, pad=1, stride=2) + ReLU   28×28 → 14×14
Conv(16→16, K=3, pad=1, stride=1) + ReLU   14×14 → 14×14
Conv(16→16, K=3, pad=1, stride=2) + ReLU   14×14 → 7×7
Conv(16→10, K=7, pad=0, stride=1)          7×7  → 1×1  (分类 logits, 无 ReLU)
```
- CPU 训 8 epoch test_acc ≈ 98.9%
- 硬件 bit-exact 匹配 `compute_expected_ofm` 的量化流水
- 末层 `relu_en=0, clip=[-128,127]` signed int8 logits
- argmax 和 PyTorch float forward 一致

## 当前范围 / 限制

- **支持**：`Conv2d + ReLU` 任意深度串联；Sequential / ModuleList / 嵌套
  Module 结构都会 walk；streaming 模式（默认）
- **不支持**：
  - `BatchNorm`（需要在量化前 fuse 进 Conv2d 权重）
  - `MaxPool / AvgPool`（需要 pool 算子，roadmap Phase H）
  - Residual / skip connection（需要 SDP 残差通路，roadmap）
  - `Linear`（需要全连接算子）
  - Non-symmetric 量化 / per-channel quant
- **DDR 尺寸限制**：FM 链总和 ≤ 8 MB（限大 feature map 层数）；
  16 MB DDR 够容纳常规 MNIST/CIFAR 尺度 2-5 层模型
- **硬件约束（每层独立）**：当前 streaming 要求 Cin/Cout 各自 ≤ 16；
  单 strip、单 tile 不受限

## 新增 model 怎么加

在 `compile_model.py` 的 `MODELS` dict 加一个 `_build_*`：
```python
def _build_my_model():
    torch = compile_layer._require_torch()
    torch.manual_seed(0)
    model = torch.nn.Sequential(
        torch.nn.Conv2d(3, 16, 3, padding=1, bias=True), torch.nn.ReLU(),
        torch.nn.Conv2d(16, 16, 3, padding=1, bias=True), torch.nn.ReLU(),
        torch.nn.Conv2d(16,  8, 1, padding=0, bias=True), torch.nn.ReLU(),
    )
    x = torch.randn(1, 3, 48, 48)
    return model, x, 3  # (model, calib_x, n_layers)

MODELS['my_model'] = _build_my_model
```

在 `run_regression.py` 的 `CASES` 追加：
```python
CASES += [
    ("my_model", "model", 3),
]
```

跑 `python run_regression.py --only model` 验证。
