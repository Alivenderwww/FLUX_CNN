"""
图 5.4 渲染：ResNet11 整网端到端时延与帧率（演进路径图）
数据来源：paper.md §5.5.1 表 5.4
样式：双 Y 轴演进图
  - 左 Y：整网周期数（log 轴，深蓝实线）
  - 右 Y：帧率 fps（深绿实线）
  - 段间用文字标增量贡献机制 + 减幅
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import matplotlib.pyplot as plt
import numpy as np
from _style import setup_style, save_figure, COLOR_BASELINE, COLOR_IMPROVED, COLOR_THIRD, COLOR_ANNO

setup_style()

# 表 5.2 数据（按 135.86 MHz 推荐工作频率折算）
configs = ["$N=1$\n基线",
           "$N=1$\n含 S2D",
           "$N=2$\n+W 切片",
           "$N=4$\n+W 切片",
           "$N=4$\n+SMC 直送\n+调度优化"]
wall_cy = np.array([1115000, 596088, 450469, 354555, 173432])
fps     = np.array([122, 228, 302, 383, 783])
speedup = np.array([1.00, 1.87, 2.48, 3.14, 6.43])

# 段间增量贡献描述
deltas = [
    None,
    "空间到深度\n（S2D）",
    "多核 W 切片\n($N{=}1{\\rightarrow}2$)",
    "多核 W 切片\n($N{=}2{\\rightarrow}4$)",
    "跨核 SRAM 直送\n+ 命令调度",
]

x = np.arange(len(configs))

fig, ax1 = plt.subplots(figsize=(11.0, 5.6))
ax2 = ax1.twinx()

# 左 Y：整网周期数（深蓝实线 + 方块）
l_cy, = ax1.plot(x, wall_cy, marker="s", color=COLOR_BASELINE, linewidth=2.0,
                 markersize=10, zorder=5,
                 markerfacecolor="white", markeredgewidth=2.0,
                 label="整网周期数")

# 右 Y：帧率（深绿实线 + 圆点）
l_fp, = ax2.plot(x, fps, marker="o", color=COLOR_IMPROVED, linewidth=1.8,
                 markersize=9, zorder=5,
                 markerfacecolor="white", markeredgewidth=1.8,
                 label="帧率（fps）")

# 周期数值标（蓝点上方）
for xi, cy, sp in zip(x, wall_cy, speedup):
    ax1.text(xi, cy * 1.32, f"{cy/1000:.0f}K\n{sp:.2f}×",
             ha="center", va="bottom", fontsize=9,
             color=COLOR_BASELINE, fontweight="bold")

# 帧率数值标（绿点下方）
for xi, fp in zip(x, fps):
    ax2.text(xi, fp - fp * 0.18, f"{fp} fps",
             ha="center", va="top", fontsize=9,
             color=COLOR_IMPROVED, fontweight="bold")

# 段间增量贡献注记（贴近横轴上方，避开折线）
text_y = 100000  # 周期 log 轴底部
for i in range(1, len(configs)):
    if deltas[i] is None:
        continue
    delta_pct = (1 - wall_cy[i] / wall_cy[i-1]) * 100
    mid_x = (i - 1 + i) / 2
    ax1.text(mid_x, text_y, deltas[i] + f"\n−{delta_pct:.0f}%",
             ha="center", va="center", fontsize=8.5,
             color=COLOR_THIRD, fontweight="bold")

# 轴
ax1.set_xticks(x)
ax1.set_xticklabels(configs, fontsize=9)
ax1.set_xlabel("优化阶段")
ax1.set_ylabel("整网周期数（对数轴）", color=COLOR_BASELINE)
ax1.set_yscale("log")
ax1.set_ylim(80000, 3_000_000)
ax1.tick_params(axis="y", labelcolor=COLOR_BASELINE)
ax2.set_ylabel("帧率（fps）", color=COLOR_IMPROVED)
ax2.set_ylim(0, 900)
ax2.tick_params(axis="y", labelcolor=COLOR_IMPROVED)
ax1.grid(True, axis="both", which="both", linestyle=":", linewidth=0.5, alpha=0.5)

# 双图例
ax1.legend(handles=[l_cy, l_fp], loc="upper right", framealpha=0.95, fontsize=9.5)

plt.tight_layout()
save_figure(fig, "fig5-4-end-to-end", Path(__file__).parent)
