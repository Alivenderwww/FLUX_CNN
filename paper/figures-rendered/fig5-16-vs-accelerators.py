"""
图 5.16 渲染：FLUX_CNN 与代表性 CNN 加速器在峰值算力—网络帧率二维空间的定位图
数据来源：paper.md §5.6 表 5.16
样式：单坐标系散点图（log-log）
  - x = 峰值算力（GOPS），log 轴
  - y = 帧率（fps），log 轴
  - 颜色编码 = 目标网络（AlexNet 蓝 / VGG-16 绿 / ResNet-50 金）
  - 形状编码 = 是否本工作（方 = FLUX_CNN，圆 = 对照工作）
  - 同一加速器的多网络点用细虚线连接
（参考 DepFiN IEEE JSSC'23 Fig. 1 / Fig. 14 单坐标系多维散点风格）
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import matplotlib.pyplot as plt
import numpy as np
from _style import setup_style, save_figure, COLOR_ANNO, COLOR_GUIDE

setup_style()

# 颜色编码：网络
COLOR_ALEX   = "#1f4e79"
COLOR_VGG    = "#2e7d32"
COLOR_RES50  = "#b8860b"

# 数据：(加速器名, 峰值GOPS, AlexNet fps, VGG-16 fps, ResNet-50 fps, 是否本工作)
data = [
    ("FLUX_CNN N=4 ASIC",  435,  260,   33,  113, True),
    ("FLUX_CNN N=4 FPGA",   76,   45,    6,   20, True),
    ("Eyeriss",            168,   35, None, None, False),
    ("Eyeriss v2",         154,  278, None, None, False),
    ("NVDLA",              128, None,   30,   60, False),
    ("VTA",                170, None, None,   50, False),
    ("Gemmini",            512, None, None,  100, False),
    ("Peng et al.",       8300, None,   50,   30, False),
    ("OPU",                160, None,   10,    5, False),
]

NETWORK_INFO = [
    ("AlexNet",   2, COLOR_ALEX),   # idx 2 in tuple
    ("VGG-16",    3, COLOR_VGG),
    ("ResNet-50", 4, COLOR_RES50),
]

fig, ax = plt.subplots(figsize=(8.5, 5.4))

# 第 1 步：每个加速器，连接其多网络点的细虚线
for row in data:
    name, gops, *fps_vals, is_ours = row[0], row[1], row[2], row[3], row[4], row[5]
    pts = [(gops, fp) for fp in [row[2], row[3], row[4]] if fp is not None]
    if len(pts) >= 2:
        # 多网络数据点用虚线连
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        # 连续点之间画虚线（实际同一 x 不同 y，所以是垂直虚线）
        ax.plot(xs, ys, linestyle=":", color=COLOR_GUIDE, linewidth=0.7,
                alpha=0.6, zorder=2)

# 第 2 步：散点 — 颜色 = 网络，形状 = 是否本工作
for net_name, idx, color in NETWORK_INFO:
    others_x, others_y = [], []
    ours_x, ours_y = [], []
    for row in data:
        gops = row[1]
        fps = row[idx]
        if fps is None:
            continue
        if row[5]:  # is_ours
            ours_x.append(gops); ours_y.append(fps)
        else:
            others_x.append(gops); others_y.append(fps)

    # 对照工作：圆点 s=70
    ax.scatter(others_x, others_y, s=70, marker="o", color=color, alpha=0.75,
               edgecolor="white", linewidth=1.0, zorder=3, label=net_name)
    # 本工作：方块 s=110（比圆点略大但不夸张）+ 加粗边框
    ax.scatter(ours_x, ours_y, s=110, marker="s", color=color,
               edgecolor=COLOR_ANNO, linewidth=1.4, zorder=5)

# 第 3 步：加速器名标注 — 每个加速器只标一次（在其最高 fps 点旁）
LABEL_OFFSETS = {  # 手工微调避免重叠
    "FLUX_CNN N=4 ASIC":  ( 8,  6),
    "FLUX_CNN N=4 FPGA":  ( 8, -16),
    "Eyeriss":            ( 8, -10),
    "Eyeriss v2":         ( 8,  6),
    "NVDLA":              (-50,  6),
    "VTA":                ( 8,  6),
    "Gemmini":            ( 8,  6),
    "Peng et al.":        ( 8, -12),
    "OPU":                ( 8, -12),
}
for row in data:
    name, gops, *fps_vals, is_ours = row[0], row[1], row[2], row[3], row[4], row[5]
    pts = [(gops, fp) for fp in [row[2], row[3], row[4]] if fp is not None]
    if not pts:
        continue
    # 用最高 fps 点作为标签锚点
    anchor = max(pts, key=lambda p: p[1])
    dx, dy = LABEL_OFFSETS.get(name, (8, 6))
    short = name.replace("FLUX_CNN ", "FLUX_CNN\n").replace(" et al.", "等")
    fontweight = "bold" if is_ours else "normal"
    fontsize = 8.5 if is_ours else 8
    ax.annotate(short, anchor, xytext=(dx, dy), textcoords="offset points",
                fontsize=fontsize, color=COLOR_ANNO, fontweight=fontweight)

ax.set_xscale("log")
ax.set_yscale("log")
ax.set_xlabel("峰值算力（GOPS）")
ax.set_ylabel("网络级帧率（fps）")
ax.set_xlim(50, 12000)
ax.set_ylim(3, 600)
ax.grid(True, which="both", linestyle=":", linewidth=0.5, alpha=0.5)

# 三层图例：网络颜色 + 形状（本工作 / 对照）
from matplotlib.lines import Line2D
legend_elements = [
    Line2D([0], [0], marker="s", color="none", markerfacecolor=COLOR_ANNO,
           markersize=10, markeredgecolor=COLOR_ANNO, label="本工作"),
    Line2D([0], [0], marker="o", color="none", markerfacecolor="#999999",
           markersize=8, markeredgecolor="white", label="对照工作"),
    Line2D([0], [0], marker="o", color="none", markerfacecolor=COLOR_ALEX,
           markersize=8, markeredgecolor="white", label="AlexNet"),
    Line2D([0], [0], marker="o", color="none", markerfacecolor=COLOR_VGG,
           markersize=8, markeredgecolor="white", label="VGG-16"),
    Line2D([0], [0], marker="o", color="none", markerfacecolor=COLOR_RES50,
           markersize=8, markeredgecolor="white", label="ResNet-50"),
]
ax.legend(handles=legend_elements, loc="lower right", framealpha=0.95,
          fontsize=8.5, ncol=1)

plt.tight_layout()
save_figure(fig, "fig5-16-vs-accelerators", Path(__file__).parent)
