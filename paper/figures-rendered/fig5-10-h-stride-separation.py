"""
图 5.10 渲染：H 步长分离对 ResNet11 N=4 SMC 降采样层周期数的影响
数据来源：paper.md §5.5.5.3 表 5.10
样式：3 层 + 整网，启用前 / 启用后 对比柱图（双柱组）
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import matplotlib.pyplot as plt
import numpy as np
from _style import setup_style, save_figure, COLOR_BASELINE, COLOR_IMPROVED, COLOR_ANNO

setup_style()

# 表 5.10 数据
labels    = ["L3\n(K=1 s=2)", "L6\n(K=1 s=2)", "L9\n(K=1 s=2)", "整网"]
before    = np.array([18369, 11721, 4151, 204240])
after     = np.array([10921,  6726, 2972, 190977])
reduction = np.array([-40.5, -42.6, -28.4, -6.5])  # 减幅 %

x = np.arange(len(labels))
w = 0.36

fig, ax1 = plt.subplots(figsize=(8.0, 4.5))

bars_b = ax1.bar(x - w/2, before, w, color=COLOR_BASELINE, label="启用前", zorder=3)
bars_a = ax1.bar(x + w/2, after,  w, color=COLOR_IMPROVED, label="启用后", zorder=3)

# 柱顶标注：整网周期数
for bar, v in zip(bars_b, before):
    ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() * 1.06,
             f"{v:,}", ha="center", va="bottom", fontsize=8, color=COLOR_BASELINE)
for bar, v in zip(bars_a, after):
    ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() * 1.06,
             f"{v:,}", ha="center", va="bottom", fontsize=8, color=COLOR_IMPROVED)

# 减幅注记 — 标在 启用后 柱中部
for xi, r, av in zip(x, reduction, after):
    ax1.text(xi + w/2, av * 0.45, f"{r:+.1f}%",
             ha="center", va="center",
             fontsize=11, fontweight="bold", color="white")

ax1.set_xticks(x)
ax1.set_xticklabels(labels, fontsize=9)
ax1.set_ylabel("整网周期数")
ax1.set_yscale("log")
ax1.set_ylim(1000, max(before) * 2.0)
ax1.grid(True, axis="y", which="both", linestyle=":", linewidth=0.5, alpha=0.5)
ax1.legend(loc="upper left", framealpha=0.95)

plt.tight_layout()
save_figure(fig, "fig5-10-h-stride-separation", Path(__file__).parent)
