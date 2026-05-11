"""
图 5.4 渲染：ResNet11 整网端到端时延与帧率（演进路径图）
数据来源：paper.md §5.5.1 表 5.4
样式：单坐标系演进图
  - x = 优化阶段（5 配置）
  - y = 整网周期数（log 轴）
  - 每点标 周期数 / 加速比 / 帧率（合并三行）
  - 段间用引导线 + 文字框 标增量贡献机制 + 减幅
（参考 DepFiN Fig. 14 多阶段叙事风格）
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import matplotlib.pyplot as plt
import numpy as np
from _style import setup_style, save_figure, COLOR_BASELINE, COLOR_IMPROVED, COLOR_THIRD, COLOR_ANNO

setup_style()

# 表 5.4 数据
configs = ["$N=1$\n基线",
           "$N=1$\n含 S2D",
           "$N=2$\n+W 切片",
           "$N=4$\n+W 切片",
           "$N=4$\n+SMC 直送\n+调度优化"]
wall_cy = np.array([1115000, 596088, 450469, 354555, 190133])
fps     = np.array([129, 241, 319, 406, 756])
speedup = np.array([1.00, 1.87, 2.48, 3.14, 5.86])

# 段间增量贡献描述
deltas = [
    None,
    "空间到深度（S2D）\nPatch 层等价化",
    "多核 W 切片\n($N=1{\\rightarrow}2$)",
    "多核 W 切片\n($N=2{\\rightarrow}4$)",
    "跨核 SRAM 直送\n+ 命令调度优化",
]

x = np.arange(len(configs))

fig, ax1 = plt.subplots(figsize=(11.0, 5.6))

# 主线：整网周期数（深蓝点 + 实线）
ax1.plot(x, wall_cy, marker="s", color=COLOR_BASELINE, linewidth=2.0,
         markersize=11, zorder=5,
         markerfacecolor="white", markeredgewidth=2.2)

# 数据点上方标 周期数 / 加速比 / 帧率（三合一框）
for xi, cy, sp, fp in zip(x, wall_cy, speedup, fps):
    ax1.annotate(f"{cy/1000:.1f}K 周期\n{sp:.2f}×  /  {fp} fps",
                 (xi, cy), xytext=(0, 18), textcoords="offset points",
                 ha="center", va="bottom", fontsize=8.5,
                 color=COLOR_BASELINE, fontweight="bold",
                 bbox=dict(boxstyle="round,pad=0.22", facecolor="white",
                           edgecolor=COLOR_BASELINE, linewidth=0.8))

# 段间增量贡献（标在数据线下方，用引导线连）
text_y_bottom = 130000  # 文字框 y 位置（log 轴底部区域）
for i in range(1, len(configs)):
    if deltas[i] is None:
        continue
    delta_pct = (1 - wall_cy[i] / wall_cy[i-1]) * 100
    mid_x = (i - 1 + i) / 2
    seg_mid_y = np.sqrt(wall_cy[i-1] * wall_cy[i])  # 段中点（几何均值）
    ax1.annotate(deltas[i] + f"\n−{delta_pct:.0f}%",
                 xy=(mid_x, seg_mid_y),
                 xytext=(mid_x, text_y_bottom),
                 ha="center", va="top", fontsize=8.5, color=COLOR_THIRD,
                 arrowprops=dict(arrowstyle="-", color=COLOR_THIRD,
                                 linewidth=0.6, alpha=0.5),
                 bbox=dict(boxstyle="round,pad=0.28", facecolor="#fff8e1",
                           edgecolor=COLOR_THIRD, linewidth=0.7))

# 累计加速比注记（最右点高亮）
ax1.annotate("累计 5.86× 加速\n756 fps @ 143.8 MHz",
             xy=(4, wall_cy[-1]),
             xytext=(-110, 50), textcoords="offset points",
             fontsize=9.5, color=COLOR_IMPROVED, fontweight="bold",
             arrowprops=dict(arrowstyle="->", color=COLOR_IMPROVED, linewidth=1.2),
             bbox=dict(boxstyle="round,pad=0.3", facecolor="#e8f4ea",
                       edgecolor=COLOR_IMPROVED, linewidth=1.0))

# 轴
ax1.set_xticks(x)
ax1.set_xticklabels(configs, fontsize=9)
ax1.set_xlabel("优化阶段")
ax1.set_ylabel("整网周期数（对数轴）")
ax1.set_yscale("log")
ax1.set_ylim(80000, 3_500_000)
ax1.grid(True, axis="both", which="both", linestyle=":", linewidth=0.5, alpha=0.5)

plt.tight_layout()
save_figure(fig, "fig5-4-end-to-end", Path(__file__).parent)
