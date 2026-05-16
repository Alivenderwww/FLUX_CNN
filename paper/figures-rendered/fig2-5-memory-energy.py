"""
图 2.5 渲染：存储层级与访存能耗金字塔
数据来源：Horowitz ISSCC'14 [16] Computing's Energy Problem
样式：横向柱图 + axes break（75 pJ 处折断，跳到 600 pJ），让片内存储 / 运算
      的细微差异可见，同时保留 DRAM 远超的视觉冲击
论证作用：直观展示片外 DRAM 访问能耗比片内寄存器高约 640×，
          支撑 §2.6 "尽量把数据留在片内"的设计原则。
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from _style import (
    setup_style, save_figure,
    COLOR_BASELINE, COLOR_IMPROVED, COLOR_THIRD, COLOR_FOURTH,
    COLOR_ANNO, COLOR_GUIDE, COLOR_PALETTE_6,
)

setup_style()

# ----------------------------------------------------------------------------
# Horowitz ISSCC'14 数据 (45 nm 工艺, 单次访问能耗, 单位 pJ)
# 引用: M. Horowitz, "1.1 Computing's energy problem (and what we can do about it),"
#       2014 IEEE ISSCC, doi:10.1109/ISSCC.2014.6757323
# ----------------------------------------------------------------------------
tiers = [
    ("DRAM",       640.0, "片外"),
    ("SRAM 1 MB",         50.0, "片内"),
    ("SRAM 8 KB",     8.0, "片内"),
    ("32-bit Mult",              3.7, "运算"),
    ("Register File 32-bit",   1.0, "片内"),
    ("32-bit Add",               0.4, "运算"),
]

labels  = [t[0] for t in tiers]
energy  = np.array([t[1] for t in tiers])
groups  = [t[2] for t in tiers]

# 自上而下: DRAM 在最上
y_pos = np.arange(len(tiers))[::-1]

# 配色: 片外深色 / 片内中色 / 运算浅色
def color_for(g):
    if g == "片外": return COLOR_BASELINE
    if g == "片内": return COLOR_IMPROVED
    return COLOR_THIRD

colors = [color_for(g) for g in groups]

# ----------------------------------------------------------------------------
# axes break: 左 ax1 显示 0-75 pJ (细节区), 右 ax2 显示 600-680 pJ (DRAM 区)
# width_ratios=[2, 1] 让左边宽以容纳 5 个密集 bar, 右边窄突出 DRAM
# ----------------------------------------------------------------------------
fig, (ax1, ax2) = plt.subplots(
    1, 2, sharey=True, figsize=(9.5, 5.0),
    gridspec_kw={"width_ratios": [2, 1], "wspace": 0.06},
)

# 两侧都画相同的 6 个 bar; xlim 控制各自可见
for ax in (ax1, ax2):
    ax.barh(y_pos, energy, color=colors, edgecolor="black", linewidth=0.6,
            height=0.62)

ax1.set_xlim(0, 75)
ax2.set_xlim(600, 680)

# 数值标注: 每个 bar 在自己可见的 ax 上标
for i, (e, g) in enumerate(zip(energy, groups)):
    if e <= 75:
        ax1.text(e + 1.2, y_pos[i], f"{e:>5.1f} pJ",
                 va="center", ha="left", fontsize=10,
                 color=COLOR_ANNO, fontweight="bold")
    else:
        ax2.text(e + 2, y_pos[i], f"{e:.1f} pJ",
                 va="center", ha="left", fontsize=10,
                 color=COLOR_ANNO, fontweight="bold")

# 类别色块图例 (放右 ax2 内右下)
legend_handles = [
    mpatches.Patch(color=COLOR_BASELINE, label="片外存储"),
    mpatches.Patch(color=COLOR_IMPROVED, label="片内存储"),
    mpatches.Patch(color=COLOR_THIRD,    label="基础运算"),
]
ax2.legend(handles=legend_handles, loc="lower right",
           framealpha=0.95, fontsize=9.5)

# 隐藏 ax1 右边框 / ax2 左边框 (制造断裂感)
ax1.spines["right"].set_visible(False)
ax2.spines["left"].set_visible(False)
ax2.tick_params(left=False, labelleft=False)

# 断点斜线 (diagonal 标记): 两 ax 各画两个小斜线
d = 0.012
kwargs = dict(transform=ax1.transAxes, color=COLOR_ANNO, clip_on=False,
              linewidth=1.0)
ax1.plot((1 - d, 1 + d), (-d, +d), **kwargs)
ax1.plot((1 - d, 1 + d), (1 - d, 1 + d), **kwargs)
kwargs.update(transform=ax2.transAxes)
ax2.plot((-d, +d), (1 - d, 1 + d), **kwargs)
ax2.plot((-d, +d), (-d, +d), **kwargs)

# 跨断点的 640× 差距标注 (放 ax2 中部空白处, 与 DRAM 数字标签错开)
ax2.text(640, 4.3, "640×",
         ha="center", va="top", fontsize=10.5,
         color=COLOR_ANNO, fontweight="bold")

# y 轴标签 (只在 ax1 上)
ax1.set_yticks(y_pos)
ax1.set_yticklabels(labels, fontsize=10)

# 共享 x 轴标签 (用 fig.supxlabel)
fig.supxlabel("单次访问 / 运算能耗（pJ）", fontsize=11, y=0.02)

# 网格
for ax in (ax1, ax2):
    ax.grid(True, axis="x", which="major", linestyle=":", linewidth=0.5,
            alpha=0.5, color=COLOR_GUIDE)
    ax.set_axisbelow(True)

# tight_layout 边界调整
plt.subplots_adjust(left=0.22, right=0.96, top=0.93, bottom=0.12, wspace=0.06)
save_figure(fig, "fig2-5-memory-energy", Path(__file__).parent)
