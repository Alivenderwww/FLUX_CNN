"""
图 5.9 渲染：降采样层 stride 扫频单核基线（4 配置 × PE 利用率）
数据来源：paper.md §5.5.5.3 表 5.9
样式：竖向柱图，K=1/K=3 两族对比
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import matplotlib.pyplot as plt
import numpy as np
from _style import (setup_style, save_figure, COLOR_BASELINE, COLOR_IMPROVED,
                    COLOR_ANNO, COLOR_BASELINE_LIGHT, COLOR_IMPROVED_LIGHT)

setup_style()

# 表 5.9 数据
labels   = ["K=1\nstride=1", "K=1\nstride=2", "K=3\nstride=1", "K=3\nstride=2"]
util     = np.array([79.6, 20.7, 95.8, 95.2])
wall_cy  = np.array([5148, 4945, 38484, 9682])
# 着色：K=1 蓝色族，K=3 绿色族；stride=2 用浅一点
colors   = [COLOR_BASELINE, COLOR_BASELINE_LIGHT, COLOR_IMPROVED, COLOR_IMPROVED_LIGHT]

x = np.arange(len(labels))
fig, ax1 = plt.subplots(figsize=(7.4, 4.6))

bars = ax1.bar(x, util, 0.55, color=colors, zorder=3)

# 标 PE 利用率值 + 整网周期数
for bar, u, cy in zip(bars, util, wall_cy):
    ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 2.0,
             f"{u:.1f}%", ha="center", va="bottom",
             fontsize=12, fontweight="bold", color=COLOR_ANNO)
    ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height()/2,
             f"{cy:,}\n周期", ha="center", va="center",
             fontsize=10, color="white", fontweight="bold")

# 标"访存受限" 注记到 K=1 stride=2 柱
ax1.annotate("访存受限",
             xy=(1, 33), xytext=(1, 55),
             ha="center", fontsize=12, color=COLOR_ANNO, fontweight="bold",
             arrowprops=dict(arrowstyle="->", color=COLOR_ANNO, linewidth=1.0,
                             shrinkA=2, shrinkB=2))

ax1.set_xticks(x)
ax1.set_xticklabels(labels, fontsize=10)
ax1.set_ylabel("PE 利用率（%）")
ax1.set_ylim(0, 110)
ax1.set_yticks(np.arange(0, 101, 20))
ax1.grid(True, axis="y", which="major", linestyle=":", linewidth=0.5, alpha=0.6)

plt.tight_layout()
save_figure(fig, "fig5-9-stride-sweep", Path(__file__).parent)
