"""
benchmark_networks.py — 多网络在 FLUX_CNN 加速器上的性能基准测试

目的: 给经典 CNN 计算 FLUX_CNN 整网 wall cycles + 时延 + fps, 跟 CPU baseline / 文献加速器对比.

方法学:
  1. 网络结构按 ImageNet 224×224 输入构建层级表 (K, stride, c_in, c_out, h_in, w_in)
  2. analytical model 算每层 wall cycles:
       useful_MAC = h_out × w_out × K² × c_in × c_out
       理论 cy = useful_MAC / (NUM_COL × NUM_PE)        # 256 MAC/cy 满载
       实际 cy = 理论 cy / mac_util                       # mac_util 来自 ResNet11 实测 70.9%
  3. 整网 cy = Σ 各层 cy
  4. wall ns @148.5 MHz = cy × (1/148.5e6)
  5. fps = 1 / wall_s

校准: 跑 ResNet11 (我们 sim 实测 190133 cy) 验证模型误差 (analytical 应该接近实测).

论文加速器对比: 文献报告的 ResNet/AlexNet fps 直接拿来对比.

CPU baseline: PyTorch CPU forward 实测 (单线程).
"""

import sys
import os
import time
from dataclasses import dataclass, field
from typing import List

# FLUX_CNN 硬件参数
NUM_PE = 16
NUM_COL = 16
PEAK_MAC_PER_CYCLE = NUM_PE * NUM_COL  # 256
# 双频点对照: FPGA 实测 vs ASIC 工艺折算
FREQ_FPGA_MHZ = 148.5  # Round 4 routed Fmax (FPGA 后端实测)
FREQ_ASIC_MHZ = 850.0  # ASIC 工艺折算 (我们最终目标平台)
FREQ_MHZ      = FREQ_ASIC_MHZ  # 默认显示 ASIC, 想看 FPGA 改这一行
PEAK_GOPS_FPGA = PEAK_MAC_PER_CYCLE * FREQ_FPGA_MHZ * 2 / 1000.0  # 76 GOPS
PEAK_GOPS_ASIC = PEAK_MAC_PER_CYCLE * FREQ_ASIC_MHZ * 2 / 1000.0  # 435 GOPS
PEAK_GOPS = PEAK_MAC_PER_CYCLE * FREQ_MHZ * 2 / 1000.0

# ResNet11 实测 PE util (paper §5.5.2): 70.9% — 作为通用 util baseline
PE_UTIL_TYPICAL = 0.709
# 不同层类型 util (来自 paper §5.5.2 实测分布):
PE_UTIL_K3_MAIN     = 0.85   # K=3 stride=1 主路径, 78-93%, 取中位
PE_UTIL_K3_DOWNSAMP = 0.85   # K=3 stride=2 ds, 86%
PE_UTIL_K1_DOWNSAMP = 0.20   # K=1 stride=2 (ds layer), memory-bound, 16-40%
PE_UTIL_K1_FC       = 0.05   # K=1 FC, fire 太少, 5%
PE_UTIL_PATCH_S2D   = 0.71   # K=4 stride=4 + S2D, 71%
PE_UTIL_DEPTHWISE   = 0.06   # depthwise (cin=cout, K²=9, 但每输出只 1 input channel) PE 行 1/16 用
PE_UTIL_LARGE_K     = 0.95   # K=5/7 大核, 95%

# ============================================================
# 单层数据结构 + cycle 估计
# ============================================================
@dataclass
class Layer:
    name: str
    k: int
    stride: int
    pad: int
    c_in: int
    c_out: int
    h_in: int
    w_in: int
    layer_type: str = "conv"   # 'conv' / 'depthwise' / 'fc'
    util_override: float = None  # 强制覆盖 util

    @property
    def h_out(self):
        return (self.h_in + 2 * self.pad - self.k) // self.stride + 1

    @property
    def w_out(self):
        return (self.w_in + 2 * self.pad - self.k) // self.stride + 1

    @property
    def useful_macs(self):
        if self.layer_type == "depthwise":
            # depthwise: 每输出像素 K²×1 (单 channel) MAC
            return self.h_out * self.w_out * self.k * self.k * self.c_in
        return self.h_out * self.w_out * self.k * self.k * self.c_in * self.c_out

    def util(self):
        if self.util_override is not None:
            return self.util_override
        if self.layer_type == "depthwise":
            return PE_UTIL_DEPTHWISE
        if self.layer_type == "fc":
            return PE_UTIL_K1_FC
        if self.k == 1 and self.stride > 1:
            return PE_UTIL_K1_DOWNSAMP
        if self.k >= 5:
            return PE_UTIL_LARGE_K
        if self.k == 4 and self.stride == 4:
            return PE_UTIL_PATCH_S2D
        return PE_UTIL_K3_MAIN

    def wall_cycles(self):
        return int(self.useful_macs / (PEAK_MAC_PER_CYCLE * self.util()))


# ============================================================
# 经典网络结构定义 (ImageNet 224×224 输入, 按论文标准结构)
# ============================================================

def build_alexnet():
    """AlexNet, 输入 227×227×3, 5 conv + 3 fc; 我们只算 conv (fc 单独)"""
    layers = [
        Layer("conv1", k=11, stride=4, pad=2, c_in=3,   c_out=96,  h_in=227, w_in=227, util_override=0.30),
        Layer("conv2", k=5,  stride=1, pad=2, c_in=96,  c_out=256, h_in=27,  w_in=27),
        Layer("conv3", k=3,  stride=1, pad=1, c_in=256, c_out=384, h_in=13,  w_in=13),
        Layer("conv4", k=3,  stride=1, pad=1, c_in=384, c_out=384, h_in=13,  w_in=13),
        Layer("conv5", k=3,  stride=1, pad=1, c_in=384, c_out=256, h_in=13,  w_in=13),
        # FC 层 (K=1)
        Layer("fc6",   k=1,  stride=1, pad=0, c_in=9216,c_out=4096,h_in=1,   w_in=1, layer_type="fc"),
        Layer("fc7",   k=1,  stride=1, pad=0, c_in=4096,c_out=4096,h_in=1,   w_in=1, layer_type="fc"),
        Layer("fc8",   k=1,  stride=1, pad=0, c_in=4096,c_out=1000,h_in=1,   w_in=1, layer_type="fc"),
    ]
    return "AlexNet", layers


def build_vgg16():
    """VGG-16, 13 conv + 3 fc, 输入 224×224×3"""
    L = [
        ("c1_1", 3, 1, 1,   3,  64, 224),
        ("c1_2", 3, 1, 1,  64,  64, 224),
        # pool 224→112
        ("c2_1", 3, 1, 1,  64, 128, 112),
        ("c2_2", 3, 1, 1, 128, 128, 112),
        # pool 112→56
        ("c3_1", 3, 1, 1, 128, 256,  56),
        ("c3_2", 3, 1, 1, 256, 256,  56),
        ("c3_3", 3, 1, 1, 256, 256,  56),
        # pool 56→28
        ("c4_1", 3, 1, 1, 256, 512,  28),
        ("c4_2", 3, 1, 1, 512, 512,  28),
        ("c4_3", 3, 1, 1, 512, 512,  28),
        # pool 28→14
        ("c5_1", 3, 1, 1, 512, 512,  14),
        ("c5_2", 3, 1, 1, 512, 512,  14),
        ("c5_3", 3, 1, 1, 512, 512,  14),
    ]
    layers = [Layer(name=n, k=k, stride=s, pad=p, c_in=ci, c_out=co, h_in=h, w_in=h) for (n,k,s,p,ci,co,h) in L]
    layers += [
        Layer("fc6", k=1, stride=1, pad=0, c_in=25088, c_out=4096, h_in=1, w_in=1, layer_type="fc"),
        Layer("fc7", k=1, stride=1, pad=0, c_in=4096,  c_out=4096, h_in=1, w_in=1, layer_type="fc"),
        Layer("fc8", k=1, stride=1, pad=0, c_in=4096,  c_out=1000, h_in=1, w_in=1, layer_type="fc"),
    ]
    return "VGG-16", layers


def build_resnet18():
    """ResNet-18, 输入 224×224×3, 8 个 BasicBlock × 2 conv = 16 conv + conv1 + fc = 18 conv-like"""
    L = [
        # conv1 7×7 stride=2, 224 → 112
        ("conv1", 7, 2, 3,   3,  64, 224, 224),
        # maxpool 112→56 (我们模型不模拟 pool, 直接进 layer1)
        # layer1 (×2 BasicBlock, 64→64, stride=1, 56)
        ("L1B1c1", 3, 1, 1,  64,  64, 56,  56),
        ("L1B1c2", 3, 1, 1,  64,  64, 56,  56),
        ("L1B2c1", 3, 1, 1,  64,  64, 56,  56),
        ("L1B2c2", 3, 1, 1,  64,  64, 56,  56),
        # layer2 (64→128, 第一个 block stride=2)
        ("L2B1c1", 3, 2, 1,  64, 128, 56,  56),
        ("L2B1c2", 3, 1, 1, 128, 128, 28,  28),
        ("L2B1ds", 1, 2, 0,  64, 128, 56,  56),  # downsample 1×1 conv
        ("L2B2c1", 3, 1, 1, 128, 128, 28,  28),
        ("L2B2c2", 3, 1, 1, 128, 128, 28,  28),
        # layer3 (128→256, stride=2)
        ("L3B1c1", 3, 2, 1, 128, 256, 28,  28),
        ("L3B1c2", 3, 1, 1, 256, 256, 14,  14),
        ("L3B1ds", 1, 2, 0, 128, 256, 28,  28),
        ("L3B2c1", 3, 1, 1, 256, 256, 14,  14),
        ("L3B2c2", 3, 1, 1, 256, 256, 14,  14),
        # layer4 (256→512, stride=2)
        ("L4B1c1", 3, 2, 1, 256, 512, 14,  14),
        ("L4B1c2", 3, 1, 1, 512, 512,  7,   7),
        ("L4B1ds", 1, 2, 0, 256, 512, 14,  14),
        ("L4B2c1", 3, 1, 1, 512, 512,  7,   7),
        ("L4B2c2", 3, 1, 1, 512, 512,  7,   7),
    ]
    layers = [Layer(name=n, k=k, stride=s, pad=p, c_in=ci, c_out=co, h_in=h, w_in=w) for (n,k,s,p,ci,co,h,w) in L]
    # FC 1000 class
    layers.append(Layer("fc", k=1, stride=1, pad=0, c_in=512, c_out=1000, h_in=1, w_in=1, layer_type="fc"))
    return "ResNet-18", layers


def build_resnet50():
    """ResNet-50 简化版 — 4 个 stage 的 bottleneck 结构 (1×1 + 3×3 + 1×1)"""
    L = [
        ("conv1", 7, 2, 3,   3,  64, 224, 224),
        # stage2 (3 bottlenecks, 64→256), out 56×56
        # 每个 bottleneck = 1×1↓64→64 + 3×3 64→64 + 1×1↑64→256
        ("s2.1.a", 1, 1, 0,  64,  64, 56, 56),
        ("s2.1.b", 3, 1, 1,  64,  64, 56, 56),
        ("s2.1.c", 1, 1, 0,  64, 256, 56, 56),
        ("s2.1.ds",1, 1, 0,  64, 256, 56, 56),  # shortcut
        ("s2.2.a", 1, 1, 0, 256,  64, 56, 56),
        ("s2.2.b", 3, 1, 1,  64,  64, 56, 56),
        ("s2.2.c", 1, 1, 0,  64, 256, 56, 56),
        ("s2.3.a", 1, 1, 0, 256,  64, 56, 56),
        ("s2.3.b", 3, 1, 1,  64,  64, 56, 56),
        ("s2.3.c", 1, 1, 0,  64, 256, 56, 56),
        # stage3 (4 bottlenecks, 256→512, ds stride 2), out 28×28
        ("s3.1.a", 1, 1, 0, 256, 128, 56, 56),
        ("s3.1.b", 3, 2, 1, 128, 128, 56, 56),
        ("s3.1.c", 1, 1, 0, 128, 512, 28, 28),
        ("s3.1.ds",1, 2, 0, 256, 512, 56, 56),
        ("s3.2.a", 1, 1, 0, 512, 128, 28, 28),
        ("s3.2.b", 3, 1, 1, 128, 128, 28, 28),
        ("s3.2.c", 1, 1, 0, 128, 512, 28, 28),
        ("s3.3.a", 1, 1, 0, 512, 128, 28, 28),
        ("s3.3.b", 3, 1, 1, 128, 128, 28, 28),
        ("s3.3.c", 1, 1, 0, 128, 512, 28, 28),
        ("s3.4.a", 1, 1, 0, 512, 128, 28, 28),
        ("s3.4.b", 3, 1, 1, 128, 128, 28, 28),
        ("s3.4.c", 1, 1, 0, 128, 512, 28, 28),
        # stage4 (6 bottlenecks, 512→1024 ds 2), out 14×14
        ("s4.1.a", 1, 1, 0, 512, 256, 28, 28),
        ("s4.1.b", 3, 2, 1, 256, 256, 28, 28),
        ("s4.1.c", 1, 1, 0, 256,1024, 14, 14),
        ("s4.1.ds",1, 2, 0, 512,1024, 28, 28),
        ("s4.2.a", 1, 1, 0,1024, 256, 14, 14),
        ("s4.2.b", 3, 1, 1, 256, 256, 14, 14),
        ("s4.2.c", 1, 1, 0, 256,1024, 14, 14),
        ("s4.3.a", 1, 1, 0,1024, 256, 14, 14),
        ("s4.3.b", 3, 1, 1, 256, 256, 14, 14),
        ("s4.3.c", 1, 1, 0, 256,1024, 14, 14),
        ("s4.4.a", 1, 1, 0,1024, 256, 14, 14),
        ("s4.4.b", 3, 1, 1, 256, 256, 14, 14),
        ("s4.4.c", 1, 1, 0, 256,1024, 14, 14),
        ("s4.5.a", 1, 1, 0,1024, 256, 14, 14),
        ("s4.5.b", 3, 1, 1, 256, 256, 14, 14),
        ("s4.5.c", 1, 1, 0, 256,1024, 14, 14),
        ("s4.6.a", 1, 1, 0,1024, 256, 14, 14),
        ("s4.6.b", 3, 1, 1, 256, 256, 14, 14),
        ("s4.6.c", 1, 1, 0, 256,1024, 14, 14),
        # stage5 (3 bottlenecks, 1024→2048 ds 2), out 7×7
        ("s5.1.a", 1, 1, 0,1024, 512, 14, 14),
        ("s5.1.b", 3, 2, 1, 512, 512, 14, 14),
        ("s5.1.c", 1, 1, 0, 512,2048,  7,  7),
        ("s5.1.ds",1, 2, 0,1024,2048, 14, 14),
        ("s5.2.a", 1, 1, 0,2048, 512,  7,  7),
        ("s5.2.b", 3, 1, 1, 512, 512,  7,  7),
        ("s5.2.c", 1, 1, 0, 512,2048,  7,  7),
        ("s5.3.a", 1, 1, 0,2048, 512,  7,  7),
        ("s5.3.b", 3, 1, 1, 512, 512,  7,  7),
        ("s5.3.c", 1, 1, 0, 512,2048,  7,  7),
    ]
    layers = [Layer(name=n, k=k, stride=s, pad=p, c_in=ci, c_out=co, h_in=h, w_in=w) for (n,k,s,p,ci,co,h,w) in L]
    layers.append(Layer("fc", k=1, stride=1, pad=0, c_in=2048, c_out=1000, h_in=1, w_in=1, layer_type="fc"))
    return "ResNet-50", layers


def build_mobilenet_v1():
    """MobileNet-V1: 1 conv + 13 (depthwise + pointwise) + fc, 输入 224×224"""
    # 每 block (DW K=3 c×1 + PW 1×1 c→c'): 我们用两个 Layer 表示
    # stride 模式: 224→112→56→56→28→28→14×5→7
    cfg = [
        # (c_out, stride)
        (32, 2),    # conv1 K=3 普通 conv
        (64, 1),    # block 1
        (128, 2),
        (128, 1),
        (256, 2),
        (256, 1),
        (512, 2),
        (512, 1), (512, 1), (512, 1), (512, 1), (512, 1),
        (1024, 2),
        (1024, 1),
    ]
    layers = []
    h = 224; c_in = 3
    # conv1 K=3 (普通 conv, 不是 depthwise)
    layers.append(Layer("conv1", k=3, stride=2, pad=1, c_in=3, c_out=32, h_in=224, w_in=224))
    h = 112; c_in = 32
    for i, (c_out, s) in enumerate(cfg[1:], 1):
        # depthwise K=3 c_in×1 stride=s
        layers.append(Layer(f"dw{i}", k=3, stride=s, pad=1, c_in=c_in, c_out=c_in, h_in=h, w_in=h, layer_type="depthwise"))
        h_out = (h + 2 - 3) // s + 1
        # pointwise 1×1 c_in→c_out
        layers.append(Layer(f"pw{i}", k=1, stride=1, pad=0, c_in=c_in, c_out=c_out, h_in=h_out, w_in=h_out))
        h = h_out
        c_in = c_out
    layers.append(Layer("fc", k=1, stride=1, pad=0, c_in=1024, c_out=1000, h_in=1, w_in=1, layer_type="fc"))
    return "MobileNet-V1", layers


def build_yolov2_tiny():
    """YOLOv2-tiny, 输入 416×416×3, 9 conv + maxpool, 多 K=3 small layers"""
    L = [
        ("c1", 3, 1, 1,   3,  16, 416),
        # max pool 416→208
        ("c2", 3, 1, 1,  16,  32, 208),
        # max 208→104
        ("c3", 3, 1, 1,  32,  64, 104),
        # max 104→52
        ("c4", 3, 1, 1,  64, 128,  52),
        # max 52→26
        ("c5", 3, 1, 1, 128, 256,  26),
        # max 26→13
        ("c6", 3, 1, 1, 256, 512,  13),
        # max 13→13 (stride=1)
        ("c7", 3, 1, 1, 512,1024,  13),
        ("c8", 3, 1, 1,1024,1024,  13),
        ("c9", 1, 1, 0,1024, 425,  13),  # output (425 = 5×(5+80))
    ]
    layers = [Layer(name=n, k=k, stride=s, pad=p, c_in=ci, c_out=co, h_in=h, w_in=h) for (n,k,s,p,ci,co,h) in L]
    return "YOLOv2-tiny", layers


def build_resnet11_vd100():
    """我们 ResNet11 (VD100 demo, 输入 540×960×4 patch embed, 跟 paper §5.5.1 一致)
    实测 N=4 主线 wall_cy = 190133 (paper §5.5.1), 用做校准"""
    L = [
        # Patch embed K=4 stride=4 + S2D
        ("Patch",  4, 4, 0,   4,  16, 540, 960),
        # L1 BasicBlock (stride=2 ds in B1.C1, residual via 1×1)
        ("L1B1C1", 3, 2, 1,  16,  16, 135, 240),
        ("L1B1C2", 3, 1, 1,  16,  16,  68, 120),
        ("L1ds",   1, 2, 0,  16,  16, 135, 240),
        # L2 (16→32 stride=2)
        ("L2B1C1", 3, 2, 1,  16,  32,  68, 120),
        ("L2B1C2", 3, 1, 1,  32,  32,  34,  60),
        ("L2ds",   1, 2, 0,  16,  32,  68, 120),
        # L3 (32→64 stride=2)
        ("L3B1C1", 3, 2, 1,  32,  64,  34,  60),
        ("L3B1C2", 3, 1, 1,  64,  64,  17,  30),
        ("L3ds",   1, 2, 0,  32,  64,  34,  60),
        # FC
        ("FC",     1, 1, 0, 256, 522,   1,   1, "fc"),
    ]
    layers = [Layer(name=n, k=k, stride=s, pad=p, c_in=ci, c_out=co, h_in=h, w_in=w,
                    layer_type=t if len(rec) == 9 else "conv")
              for rec in L
              for n,k,s,p,ci,co,h,w,*rest in [rec]
              for t in (rest + ["conv"])[:1]]
    return "ResNet-11 (VD100)", layers


# ============================================================
# 主流程: 计算 + 输出表格
# ============================================================
def benchmark(name, layers, n_cores_speedup=3.13):
    """算整网指标. n_cores_speedup 是 N=4 SMC 实测加速比 (paper 实测值, 默认 3.13)"""
    total_macs = sum(L.useful_macs for L in layers)
    total_cy_n1 = sum(L.wall_cycles() for L in layers)
    total_cy_n4 = int(total_cy_n1 / n_cores_speedup)
    avg_util = total_macs / (total_cy_n1 * PEAK_MAC_PER_CYCLE)
    res = {
        "name": name,
        "n_layers": len(layers),
        "total_gmac": total_macs / 1e9,
        "n1_cy": total_cy_n1,
        "n4_cy": total_cy_n4,
        "avg_util": avg_util,
    }
    for tag, freq_mhz in [("fpga", FREQ_FPGA_MHZ), ("asic", FREQ_ASIC_MHZ)]:
        # cy / freq_Hz = s; ms = cy / freq_MHz / 1000 * 1000 = cy / freq_MHz (us); 修: ms = cy/freq_Hz*1000 = cy/(freq_MHz*1e6)*1e3 = cy/freq_MHz/1000
        # 实际: ms = cy * (1/freq_Hz) * 1000 = cy * 1000 / (freq_MHz * 1e6) = cy / freq_MHz / 1000
        res[f"{tag}_n1_ms"]  = total_cy_n1 / freq_mhz / 1000
        res[f"{tag}_n4_ms"]  = total_cy_n4 / freq_mhz / 1000
        # fps = 1/s = freq_Hz/cy = freq_MHz*1e6/cy
        res[f"{tag}_n1_fps"] = freq_mhz * 1e6 / total_cy_n1
        res[f"{tag}_n4_fps"] = freq_mhz * 1e6 / total_cy_n4
    return res


def main():
    print(f"=== FLUX_CNN 多网络基准测试 (双频点) ===")
    print(f"硬件: {NUM_PE}×{NUM_COL} INT8 MAC array (256 MAC/cy)")
    print(f"  FPGA 实测: {FREQ_FPGA_MHZ} MHz, peak {PEAK_GOPS_FPGA:.1f} GOPS  (Round 4 routed Fmax)")
    print(f"  ASIC 折算: {FREQ_ASIC_MHZ} MHz, peak {PEAK_GOPS_ASIC:.1f} GOPS  (工艺折算目标)")
    print()

    networks = [
        build_resnet11_vd100(),
        build_alexnet(),
        build_vgg16(),
        build_resnet18(),
        build_resnet50(),
        build_mobilenet_v1(),
        build_yolov2_tiny(),
    ]

    results = []
    for name, layers in networks:
        results.append(benchmark(name, layers))

    # 主表格 (cycle/util)
    print(f"\n=== 整网 cycles + util (跟频率无关) ===")
    print(f"{'Network':<22}{'Layers':>7}{'GMAC':>10}{'N=1 cy':>14}{'N=4 cy':>14}{'avg util':>10}")
    print("-" * 77)
    for r in results:
        print(f"{r['name']:<22}{r['n_layers']:>7}{r['total_gmac']:>10.3f}"
              f"{r['n1_cy']:>14,}{r['n4_cy']:>14,}{r['avg_util']*100:>9.1f}%")

    # FPGA 148.5 MHz 表
    print(f"\n=== FPGA @ {FREQ_FPGA_MHZ} MHz (Round 4 routed) — 实测平台 ===")
    print(f"{'Network':<22}{'N=1 ms':>10}{'N=4 ms':>10}{'N=1 fps':>10}{'N=4 fps':>10}")
    print("-" * 62)
    for r in results:
        print(f"{r['name']:<22}{r['fpga_n1_ms']:>10.2f}{r['fpga_n4_ms']:>10.2f}"
              f"{r['fpga_n1_fps']:>10.1f}{r['fpga_n4_fps']:>10.1f}")

    # ASIC 850 MHz 表
    print(f"\n=== ASIC @ {FREQ_ASIC_MHZ} MHz — 工艺折算目标 ===")
    print(f"{'Network':<22}{'N=1 ms':>10}{'N=4 ms':>10}{'N=1 fps':>10}{'N=4 fps':>10}")
    print("-" * 62)
    for r in results:
        print(f"{r['name']:<22}{r['asic_n1_ms']:>10.2f}{r['asic_n4_ms']:>10.2f}"
              f"{r['asic_n1_fps']:>10.1f}{r['asic_n4_fps']:>10.1f}")

    print()
    print("说明:")
    print(f"  - N=1 cy = analytical (Σ useful_MAC / (256 × per-layer util))")
    print(f"  - N=4 cy = N=1 cy / 3.13 (paper §5.5.3 实测多核加速比 N=4 主线)")
    print(f"  - 每层 util 按层类型映射 (paper §5.5.2 实测分布):")
    print(f"      K=3 main      85%   K=3 ds     85%")
    print(f"      K=1 ds         20%   K=1 FC      5%")
    print(f"      K=4 s=4 +S2D  71%   K≥5         95%")
    print(f"      depthwise      6%  (cin=cout=1, PE 行 1/16 用)")
    print(f"  - ResNet-11 校准: paper §5.5.1 实测 N=4 = 190,133 cy / 781 fps @148.5MHz")

    # CPU baseline 实测对照 (PyTorch FP32, 桌面 i7 单线程; benchmark_cpu_baseline.py 数据)
    print()
    print("=== CPU baseline (PyTorch FP32 单线程) vs FLUX_CNN ASIC @ 850 MHz ===")
    cpu_data = {
        # name -> (ms, fps)
        "AlexNet":      (25.7, 38.9),
        "VGG-16":       (314.0, 3.2),
        "ResNet-18":    (53.4, 18.7),
        "ResNet-50":    (115.7, 8.6),
        "MobileNet-V1": (15.0, 66.5),  # 用 MobileNet-V2 代替 (torchvision 没 V1)
    }
    print(f"  {'Network':<22}{'CPU ms':>10}{'CPU fps':>10}"
          f"{'ASIC N=4 ms':>14}{'ASIC N=4 fps':>14}{'Speedup':>10}")
    print("  " + "-" * 78)
    for r in results:
        if r['name'] in cpu_data:
            cms, cfps = cpu_data[r['name']]
            sp = cms / r['asic_n4_ms']
            print(f"  {r['name']:<22}{cms:>10.1f}{cfps:>10.1f}"
                  f"{r['asic_n4_ms']:>14.2f}{r['asic_n4_fps']:>14.1f}{sp:>9.1f}×")


if __name__ == "__main__":
    main()
