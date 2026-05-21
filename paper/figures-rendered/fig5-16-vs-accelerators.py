"""
图 5.16 渲染：FLUX_CNN 与代表性 CNN 加速器在峰值算力—网络帧率二维空间的定位图
数据来源：paper.md §5.6 表 5.16
样式：单坐标系散点图（log-log）
  - x = 峰值算力（GOPS），log 轴
  - y = 帧率（fps），log 轴
  - 颜色 = 目标网络（AlexNet 蓝 / VGG-16 绿 / ResNet-50 金）
  - 形状 = 方=本工作（FLUX_CNN）/ 圆=对照工作
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import matplotlib.pyplot as plt
import numpy as np
from _style import (setup_style, save_figure, COLOR_ANNO, COLOR_GUIDE,
                    COLOR_BASELINE, COLOR_IMPROVED, COLOR_THIRD)

setup_style()

# 颜色编码：网络（随 palette 变化）
COLOR_ALEX   = COLOR_BASELINE
COLOR_VGG    = COLOR_IMPROVED
COLOR_RES50  = COLOR_THIRD

# 数据：(加速器名, 峰值GOPS, AlexNet fps, VGG-16 fps, ResNet-50 fps, 是否本工作)
# FLUX_CNN N=4 数据按 paper.md §5.7.7 表 5.6 (按各网络层级 PE 利用率加权分析模型) 折算
# - ASIC @ 850 MHz: 1024 PE × 850 MHz × 2 ops/MAC = 1740.8 GOPS 峰值
# - FPGA @ 135.86 MHz: 1024 PE × 135.86 MHz × 2 ops/MAC = 278.2 GOPS 峰值
data = [
    ("FLUX_CNN N=4 ASIC", 1741,  260,   33,  113, True),
    ("FLUX_CNN N=4 FPGA",  278,   41,    5,   18, True),
    ("Eyeriss",            168,   35, None, None, False),
    ("Eyeriss v2",         154,  278, None, None, False),
    ("NVDLA",              128, None,   30,   60, False),
    ("VTA",                170, None, None,   50, False),
    ("Gemmini",            512, None, None,  100, False),
    ("OPU",                160, None,   10,    5, False),
]

NETWORK_INFO = [
    ("AlexNet",   2, COLOR_ALEX),
    ("VGG-16",    3, COLOR_VGG),
    ("ResNet-50", 4, COLOR_RES50),
]

fig, ax = plt.subplots(figsize=(8.8, 5.6))

# 第 1 步：每个加速器，连接其多网络点的细虚线
for row in data:
    name, gops, *fps_vals, is_ours = row[0], row[1], row[2], row[3], row[4], row[5]
    pts = [(gops, fp) for fp in [row[2], row[3], row[4]] if fp is not None]
    if len(pts) >= 2:
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, linestyle=":", color=COLOR_GUIDE, linewidth=0.7,
                alpha=0.6, zorder=2)

# 第 2 步：散点 — 颜色 = 网络，形状 = 是否本工作（方=本工作 / 圆=对照）
for net_name, idx, color in NETWORK_INFO:
    others_x, others_y = [], []
    ours_x, ours_y = [], []
    for row in data:
        gops = row[1]
        fps = row[idx]
        if fps is None:
            continue
        if row[5]:
            ours_x.append(gops); ours_y.append(fps)
        else:
            others_x.append(gops); others_y.append(fps)

    # 对照工作：圆点 s=75
    ax.scatter(others_x, others_y, s=75, marker="o", color=color, alpha=0.75,
               edgecolor="white", linewidth=1.0, zorder=3, label=net_name)
    # 本工作：方块 s=140，黑色加粗边框
    ax.scatter(ours_x, ours_y, s=140, marker="s", color=color,
               edgecolor="black", linewidth=1.8, zorder=5)

# 第 3 步：加速器名标注 — 每个加速器只标一次（在其最高 fps 点旁）
LABEL_OFFSETS = {
    "FLUX_CNN N=4 ASIC":  ( 22,  22),
    "FLUX_CNN N=4 FPGA":  ( 22, -28),
    "Eyeriss":            (-22, -28),
    "Eyeriss v2":         ( 22,  18),
    "NVDLA":              (-68,  18),
    "VTA":                ( 22,  18),
    "Gemmini":            ( 22,  18),
    "OPU":                ( 22, -22),
}
for row in data:
    name, gops, *fps_vals, is_ours = row[0], row[1], row[2], row[3], row[4], row[5]
    pts = [(gops, fp) for fp in [row[2], row[3], row[4]] if fp is not None]
    if not pts:
        continue
    anchor = max(pts, key=lambda p: p[1])
    dx, dy = LABEL_OFFSETS.get(name, (22, 18))
    short = name.replace("FLUX_CNN ", "FLUX_CNN\n").replace(" et al.", "等")
    fontweight = "bold" if is_ours else "normal"
    fontsize = 9 if is_ours else 8.5
    color = "black" if is_ours else COLOR_ANNO
    ha = "left" if dx >= 0 else "right"
    ax.annotate(short, anchor, xytext=(dx, dy), textcoords="offset points",
                fontsize=fontsize, color=color, fontweight=fontweight, ha=ha,
                arrowprops=dict(arrowstyle="-", color=COLOR_ANNO,
                                linewidth=0.6, alpha=0.5))

ax.set_xscale("log")
ax.set_yscale("log")
ax.set_xlabel("峰值算力（GOPS）")
ax.set_ylabel("网络级帧率（fps）")
ax.set_xlim(80, 3000)
ax.set_ylim(3, 600)
ax.grid(True, which="both", linestyle=":", linewidth=0.5, alpha=0.5)

# 图例：网络颜色 + 形状（本工作 / 对照）
from matplotlib.lines import Line2D
legend_elements = [
    Line2D([0], [0], marker="s", color="none", markerfacecolor="#888888",
           markersize=12, markeredgecolor="black", markeredgewidth=1.5,
           label="本工作 FLUX_CNN"),
    Line2D([0], [0], marker="o", color="none", markerfacecolor="#bbbbbb",
           markersize=9, markeredgecolor="white", markeredgewidth=1.0,
           label="对照工作"),
    Line2D([0], [0], marker="o", color="none", markerfacecolor=COLOR_ALEX,
           markersize=9, markeredgecolor="white", label="AlexNet"),
    Line2D([0], [0], marker="o", color="none", markerfacecolor=COLOR_VGG,
           markersize=9, markeredgecolor="white", label="VGG-16"),
    Line2D([0], [0], marker="o", color="none", markerfacecolor=COLOR_RES50,
           markersize=9, markeredgecolor="white", label="ResNet-50"),
]
ax.legend(handles=legend_elements, loc="lower right", framealpha=0.95,
          fontsize=9, ncol=1)

plt.tight_layout()
save_figure(fig, "fig5-16-vs-accelerators", Path(__file__).parent)
