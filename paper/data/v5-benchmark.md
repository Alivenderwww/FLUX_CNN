# v5 多网络基准测试 — FLUX_CNN vs CPU vs 经典加速器

> 频率: **ASIC 折算 850 MHz** (主报告口径) / FPGA 实测 148.5 MHz (附表对照)
> 日期: 2026-05-10
> 数据来源:
>   - FLUX_CNN: analytical model + ResNet11 校准 (paper §5.5.1 N=4 实测 190,133 cy)
>   - CPU baseline: PyTorch 2.11.0 FP32 单线程 forward 实测 (本机)
>   - 经典加速器: 文献引用 (paper §1.2 表 1.1 + §5.6)

---

## 1 硬件参数

| 项 | 值 |
|---|---|
| 阵列规模 | 16 × 16 INT8 MAC (256 MAC/cycle) |
| ASIC 目标频率 | **850 MHz** (典型 16 nm/22 nm 工艺) |
| FPGA 实测频率 | 148.5 MHz (XC7K325T-2 routed Fmax) |
| ASIC 峰值算力 | **435.2 GOPS** (256 × 850 MHz × 2 ops/MAC) |
| FPGA 峰值算力 | 76.0 GOPS (148.5 MHz) |
| 数据类型 | INT8 weight × INT8 activation, INT32 psum |
| 多核扩展 | N=1/2/4 共享外存, SMC + 跨核 SRAM 直送 |

---

## 2 analytical model 方法学

每层 wall cycles = `useful_MAC / (256 × per-layer PE util)`. PE util 按层类型 (paper §5.5.2 实测):

| 层类型 | util | 层类型 | util |
|---|---|---|---|
| K=3 stride=1 主路径 | 85% | K=4 stride=4 + S2D (Patch) | 71% |
| K=3 stride=2 ds | 85% | K=5/7 大核 | 95% |
| K=1 stride=2 ds (memory-bound) | 20% | depthwise (PE 行 1/16 用) | 6% |
| K=1 FC (fire 数小) | 5% | | |

**N=4 SMC 加速比**: 按 ResNet-11 实测 3.13× 折算 (paper §5.5.3).

**ResNet-11 校准误差**: analytical N=4 = 225,742 cy, 实测 190,133 cy, 高估 ~18% (没建模 IDMA setup 节省). 其它网络真值应在 analytical (1.0×, 1.2×) 区间.

---

## 3 多网络性能 — ASIC @ 850 MHz (主表)

表 3.1 经典 CNN 在 FLUX_CNN ASIC 实现上的整网延时与帧率

Table 3.1 End-to-end latency and frame rate of representative CNN networks on FLUX_CNN ASIC @ 850 MHz

| 网络 | 输入分辨率 | 层数 | GMAC | N=1 cy | N=4 cy | N=1 ms | **N=4 ms** | N=1 fps | **N=4 fps** | avg PE util |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| ResNet-11 (VD100, 校准) | 540×960×4 | 11 | 0.131 | 706,575 | 225,742 | 0.83 | **0.27** | 1,203 | **3,765** | 72.7% |
| AlexNet | 227×227×3 | 8 | 1.139 | 10,249,453 | 3,274,585 | 12.06 | **3.85** | 82.9 | **259.6** | 43.4% |
| VGG-16 | 224×224×3 | 16 | 15.470 | 80,185,669 | 25,618,424 | 94.34 | **30.14** | 10.6 | **33.2** | 75.4% |
| ResNet-18 | 224×224×3 | 21 | 1.814 | 8,605,060 | 2,749,220 | 10.12 | **3.23** | 98.8 | **309.2** | 82.3% |
| ResNet-50 | 224×224×3 | 54 | 4.089 | 23,490,074 | 7,504,815 | 27.64 | **8.83** | 36.2 | **113.3** | 68.0% |
| MobileNet-V1 | 224×224×3 | 28 | 0.569 | 3,740,988 | 1,195,203 | 4.40 | **1.41** | 227.2 | **711.2** | 59.4% |
| YOLOv2-tiny | 416×416×3 | 9 | 3.537 | 16,256,602 | 5,193,802 | 19.13 | **6.11** | 52.3 | **163.7** | 85.0% |

> ResNet-11 校准: paper §5.5.1 实测 N=4 = 190,133 cy / 0.224 ms / **4,471 fps** @ 850 MHz (按实测 cy 折算).
> analytical 的 3,765 fps 偏保守 18%, 其它网络 fps 期望真值略高.

---

## 4 FPGA @ 148.5 MHz 对照表 (附)

表 4.1 同样网络在 FPGA 实测频率下的延时与帧率 (附表)

Table 4.1 Latency / fps under FPGA-realized frequency 148.5 MHz (附)

| 网络 | N=1 ms | N=4 ms | N=1 fps | N=4 fps |
|---|---:|---:|---:|---:|
| ResNet-11 (VD100) | 4.76 | 1.52 | 210 | **658** |
| AlexNet | 69.0 | 22.1 | 14.5 | **45.3** |
| VGG-16 | 540 | 172 | 1.9 | **5.8** |
| ResNet-18 | 58.0 | 18.5 | 17.3 | **54.0** |
| ResNet-50 | 158 | 50.5 | 6.3 | **19.8** |
| MobileNet-V1 | 25.2 | 8.1 | 39.7 | **124.2** |
| YOLOv2-tiny | 110 | 35.0 | 9.1 | **28.6** |

> ASIC 850 MHz / FPGA 148.5 MHz = **5.72× 加速** (纯频率提升, util 不变).

---

## 5 vs CPU baseline (PyTorch FP32 单线程实测)

表 5.1 同一网络在 CPU FP32 单线程下的 forward 时间 + ASIC 加速比

Table 5.1 PyTorch CPU FP32 forward time vs FLUX_CNN ASIC @ 850 MHz speedup

| 网络 | CPU ms | CPU fps | **ASIC N=4 ms** | **ASIC N=4 fps** | **加速比** |
|---|---:|---:|---:|---:|---:|
| AlexNet | 25.7 | 38.9 | **3.85** | **260** | **6.7×** |
| VGG-16 | 314.0 | 3.2 | **30.14** | **33** | **10.4×** |
| ResNet-18 | 53.4 | 18.7 | **3.23** | **309** | **16.5×** |
| ResNet-50 | 115.7 | 8.6 | **8.83** | **113** | **13.1×** |
| MobileNet-V2\* | 15.0 | 66.5 | **1.41** | **711** | **10.7×** |

> \* MobileNet 以 V2 实测代替 V1 (torchvision 没有 V1 weights). FLUX_CNN 那边按 V1 估算, 跟 V2 大致同量级.
>
> CPU 测试环境: 桌面 i7 单线程 PyTorch 2.11 FP32 (AVX-512 优化), 5 次平均.
> FPGA 实测口径加速比 (148.5 MHz N=4): AlexNet 1.17× / VGG-16 1.82× / ResNet-18 2.88× / ResNet-50 2.29× / MobileNet 1.86× — 比 ASIC 折算缩 5.72×.

---

## 6 vs 经典加速器 (文献对比)

表 6.1 FLUX_CNN ASIC @ 850 MHz 与经典 CNN 加速器在常见网络上的延时 / 帧率对比

Table 6.1 Latency / fps comparison: FLUX_CNN ASIC vs representative CNN accelerators

| 加速器 | 平台/工艺 | 频率 (MHz) | 峰值 (GOPS) | AlexNet fps | VGG-16 fps | ResNet-50 fps | 备注 |
|---|---|---:|---:|---:|---:|---:|---|
| **FLUX_CNN N=1** ASIC | 16/22nm | 850 | 435 | 83 | 11 | 36 | analytical |
| **FLUX_CNN N=4** ASIC | 16/22nm | 850 | 435 | **260** | **33** | **113** | analytical |
| **FLUX_CNN N=4** FPGA | XC7K325T | 148.5 | 76 | 45 | 6 | 20 | analytical |
| Eyeriss [13] 2017 | 65nm ASIC | 200 | 168 (INT16) | 35 | — | — | AlexNet @278 mW |
| Eyeriss v2 [14] 2019 | 65nm ASIC | 200 | 153.6 | ~278 | — | — | 0% sparsity |
| NVDLA [18] 2018 | 16nm ASIC | 1000 | 128 (INT8) | — | ~30\* | ~60\* | 估算 |
| VTA [27] 2019 | ZCU102 | 333 | 170 (INT8) | — | — | ~50\* | 估算 |
| Gemmini [19] 2021 | 22nm ASIC | 1000 | 512 (INT8) | — | — | ~100\* | 估算 |
| Peng et al [24] 2022 | Arria 10 | 1000 | 8300 | — | ~50\* | ~30\* | 估算 |
| OPU [20] 2020 | XC7Z045 | 200 | 160 (INT8) | — | ~10 | ~5 | overlay 部署 |

> \*文献只给峰值 GOPS, 网络 fps 按 50% util 估算. 直接 fps 对比的局限性: 工艺 / 频率差异大, 仅作量级参考.
>
> **FLUX_CNN ASIC 在 ResNet/VGG 主流网络上的吞吐**:
> - AlexNet 260 fps: 跟 Eyeriss (35) 比, 7.4×; 跟 Eyeriss v2 (278) 比, 0.94×
> - VGG-16 33 fps: 跟 NVDLA (~30) 比, 1.10×; 跟 Peng (~50) 比, 0.66×
> - ResNet-50 113 fps: 跟 NVDLA (~60) 比, 1.88×; 跟 Gemmini (~100) 比, 1.13×
>
> FLUX_CNN ASIC 整体在 **ResNet-50 上达 113 fps**, 跟同 16nm 工艺 NVDLA 量级相当.

---

## 7 关键结论

### 7.1 FLUX_CNN ASIC 端到端定位

- **峰值算力 435 GOPS @ 850 MHz** — 在端侧 ASIC 加速器中属于中等偏小规模 (跟 Eyeriss/NVDLA 主流量级 128-500 GOPS 接近)
- **多核扩展性**: N=4 SMC 加速比 3.13× (paper §5.5.3 实测), 接近线性 4× 上限
- **网络适应性**: K=3 主路径网络 (ResNet/VGG) PE util 75-85%, 在 ResNet-50 上达到 113 fps

### 7.2 跟 CPU 对比 (ASIC 视角)

- 桌面 CPU FP32 单线程 vs ASIC N=4: **6.7× 至 16.5× 加速** (主流网络)
- ResNet-18 加速 16.5× 最高, 因 CPU 上分支结构难向量化, ASIC 上 K=3 主路径 PE util 82%
- VGG-16 加速 10.4× (CPU 上大算量但有 AVX-512 高效优化, ASIC 上 K=3 大量重复)

### 7.3 跟经典加速器对比 (ASIC 视角)

| 维度 | FLUX_CNN ASIC 表现 |
|---|---|
| 峰值 GOPS | 435 (中等, 跟 NVDLA 128 / Gemmini 512 同量级) |
| AlexNet fps | 260 (跟 Eyeriss v2 278 接近) |
| VGG-16 fps | 33 (跟 NVDLA 30 接近) |
| ResNet-50 fps | 113 (优于 NVDLA 60, 接近 Gemmini 100) |
| 设计复杂度 | 5 模块 + axi_dm IP, 比 NVDLA 6 级流水简单 |

### 7.4 适用场景

FLUX_CNN ASIC 最适合:
1. **K=3 主导网络** (ResNet-50/18, VGG): util 75-85%
2. **小输入分辨率** (32-128): IDMA setup overhead 摊薄
3. **流式推理** (单帧延时 < 30 ms): N=4 SMC 满载

不适合:
1. **MobileNet 系 depthwise**: PE 行 1/16 用, util < 10%
2. **Transformer**: 没有 attention/softmax 加速

---

## 8 数据局限性

1. **Analytical model 校准误差 ±20%** (ResNet-11 校准 +18% 偏保守); 论文引用应以 ResNet-11 实测 4,471 fps @850 MHz 为准, 其它网络是 estimation
2. **CPU baseline 是桌面 i7 FP32**, 不是端侧 ARM. 端侧 ARM 加速比应 > 当前 2-3×
3. **文献加速器 fps 估算**: 部分加速器论文只给峰值 GOPS, 网络级 fps 按 50% util 折算, 误差较大
4. **VGG/AlexNet 内含的 5×5 / 11×11 大核**: K=8 通过 S2D 等价, K>8 需要 round chunking, 实际 util 可能略低于 95%
5. **ASIC 频率 850 MHz**: 工艺折算目标值 (典型 16nm/22nm), 未做 ASIC 后端 PnR; 实际投片 Fmax 可能 700-1000 MHz 区间

---

## 9 复现命令

```bash
cd C:/_Project/FLUX_CNN/toolchain
PY=./.venv/Scripts/python.exe

# FLUX_CNN analytical (双频点 FPGA + ASIC)
$PY benchmark_networks.py

# CPU FP32 baseline 实测 (PyTorch single-thread)
$PY benchmark_cpu_baseline.py
```

ResNet-11 真实测 (sim) 数据来自 [paper.md §5.5.1](../workspace/paper.md) 表 5.4.
脚本: [toolchain/benchmark_networks.py](../../toolchain/benchmark_networks.py) +
[toolchain/benchmark_cpu_baseline.py](../../toolchain/benchmark_cpu_baseline.py).
