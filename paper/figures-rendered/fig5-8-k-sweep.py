"""
图 5.8 渲染：卷积核尺寸 K 扫频（触发数 + 周期/触发 + PE 利用率）
数据来源：paper.md §5.5.5.2 表 5.8
样式：双 Y 轴（触发数柱图 / PE 利用率折线 + 周期/触发 注记）
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import matplotlib.pyplot as plt
import numpy as np
from _style import setup_style, save_figure, COLOR_BASELINE, COLOR_IMPROVED, COLOR_ANNO

setup_style()

# 表 5.8 数据
K_labels = ["K=1", "K=3", "K=5", "K=7", "K=8\n含S2D"]
fire     = np.array([1024, 9216, 25600, 50176, 784])
cy       = np.array([1627, 10131, 26912, 52175, 1358])
cy_per_fire = np.array([1.59, 1.10, 1.05, 1.04, 1.73])
util     = np.array([62.9, 91.0, 95.1, 96.2, 57.7])

x = np.arange(len(K_labels))
fig, ax1 = plt.subplots(figsize=(7.6, 4.4))
ax2 = ax1.twinx()

bars = ax1.bar(x, fire, 0.55, color=COLOR_BASELINE, alpha=0.85, label="触发数", zorder=3)
ax1.set_ylabel("触发数", color=COLOR_BASELINE)
ax1.set_yscale("log")  # K² 线性 → log 轴更直观
ax1.tick_params(axis="y", labelcolor=COLOR_BASELINE)

# 右轴 PE 利用率折线
ax2.plot(x, util, marker="o", color=COLOR_IMPROVED, label="PE 利用率", linewidth=1.6, zorder=4)
ax2.set_ylabel("PE 利用率（%）", color=COLOR_IMPROVED)
ax2.set_ylim(0, 105)
ax2.tick_params(axis="y", labelcolor=COLOR_IMPROVED)

# 周期/触发 注记
for xi, ratio in zip(x, cy_per_fire):
    ax1.text(xi, fire[xi] * 1.5, f"周期/触发\n{ratio:.2f}", ha="center", va="bottom",
             fontsize=8, color=COLOR_ANNO,
             bbox=dict(boxstyle="round,pad=0.18", facecolor="white", edgecolor=COLOR_ANNO, linewidth=0.5))

ax1.set_xticks(x)
ax1.set_xticklabels(K_labels)
ax1.set_xlabel("卷积核尺寸 $K$")
ax1.grid(True, axis="y", which="both", linestyle=":", linewidth=0.5, alpha=0.5)

# 双图例
h1, l1 = ax1.get_legend_handles_labels()
h2, l2 = ax2.get_legend_handles_labels()
ax1.legend(h1 + h2, l1 + l2, loc="upper left", framealpha=0.95)

plt.tight_layout()
save_figure(fig, "fig5-8-k-sweep", Path(__file__).parent)
