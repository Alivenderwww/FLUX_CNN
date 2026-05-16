"""
图 5.7 渲染：不同输入通道下 Y 维折叠对 PE 利用率与整网周期数的影响（双 Y 轴升级版）
数据来源：paper.md §5.5.5.1 表 5.7
样式：双 Y 轴
  - 左 Y：PE 利用率（折线 + 标记）— 关闭折叠 vs 开启折叠
  - 右 Y：整网周期数（柱图，半透明）— 关闭折叠 vs 开启折叠
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import matplotlib.pyplot as plt
import numpy as np
from _style import setup_style, save_figure, COLOR_BASELINE, COLOR_IMPROVED, COLOR_ANNO, COLOR_GUIDE

setup_style()

# 表 5.7 数据
cin_values = np.array([1, 2, 4, 8, 16, 32, 64])
cy_off   = np.array([10131, 10131, 10131, 10131, 10131, 19779, 39396])
cy_on    = np.array([ 4104,  4104,  4104, np.nan, 10131, 19779, 39396])
util_off = np.array([5.7, 11.4, 22.7, 45.5, 91.0, 93.2, 93.6])
util_on  = np.array([14.0, 28.1, 56.1, np.nan, 91.0, 93.2, 93.6])

fig, ax1 = plt.subplots(figsize=(8.5, 4.8))
ax2 = ax1.twinx()

# x 用线性均匀位置，但显示为 Cin 数值
x = np.arange(len(cin_values))
w = 0.35  # 柱宽

# 右 Y：整网周期数柱图（半透明柱）
bars_off_cy = ax2.bar(x - w/2, cy_off, w, color=COLOR_BASELINE, alpha=0.18,
                      zorder=1, label="关闭折叠 周期数")
bars_on_cy  = ax2.bar(x + w/2,
                      np.nan_to_num(cy_on, nan=0), w,
                      color=COLOR_IMPROVED, alpha=0.18, zorder=1, label="开启折叠 周期数")

# 左 Y：PE 利用率折线
l1, = ax1.plot(x, util_off, marker="o", color=COLOR_BASELINE,
               label="关闭折叠 PE 利用率", zorder=4, linewidth=1.6)
l2, = ax1.plot(x, util_on, marker="s", color=COLOR_IMPROVED,
               label="开启折叠 PE 利用率", zorder=4, linewidth=1.6)

# 注记 Cin=8 边界缺陷
ax1.annotate(r"$C_{\mathrm{in}}=8$ 边界缺陷",
             xy=(3, 45.5), xytext=(1.6, 70),
             fontsize=10, color=COLOR_ANNO,
             arrowprops=dict(arrowstyle="->", color=COLOR_ANNO, linewidth=0.8),
             ha="left")

# 拐点 Cin=16 — 标在拐点上方但偏离折线节点（移到左下方更空旷处）
ax1.axvline(x=4, color=COLOR_GUIDE, linestyle="--", linewidth=0.7, alpha=0.7)
ax1.text(3.6, 102, "拐点 $C_{\\mathrm{in}}=16$", fontsize=10, color=COLOR_ANNO,
         ha="right", va="top")

# 加速比注记 — 竖直箭头从关闭折叠节点指向开启折叠节点，2.47× 标在箭头侧
for xi, u_on, u_off in zip([0, 1, 2], util_on[:3], util_off[:3]):
    ax1.annotate("", xy=(xi, u_on), xytext=(xi, u_off),
                 arrowprops=dict(arrowstyle="->", color=COLOR_ANNO,
                                 linewidth=1.4, shrinkA=4, shrinkB=4))
    ax1.text(xi + 0.08, (u_on + u_off) / 2, "2.47×",
             fontsize=10, color=COLOR_ANNO, fontweight="bold",
             ha="left", va="center")

# 周期数比例注记（折叠收益）— Cin=1,2,4 柱顶值上移避开折线
for xi, c_off, c_on in zip([0, 1, 2], cy_off[:3], cy_on[:3]):
    # 关闭折叠柱顶（蓝色）— 利用率折线在低位（5.7%-22.7%），柱顶在 10K 高度
    # 把数值标在柱顶之外（往上偏），用 ax2 数据坐标
    ax2.text(xi - w/2, c_off + 2500, f"{c_off/1000:.1f}K",
             ha="center", va="bottom", fontsize=8, color=COLOR_BASELINE, alpha=0.9,
             fontweight="bold")
    ax2.text(xi + w/2, c_on + 2500, f"{c_on/1000:.1f}K",
             ha="center", va="bottom", fontsize=8, color=COLOR_IMPROVED, alpha=0.95,
             fontweight="bold")

# Cin=8 周期数注记
ax2.text(3 - w/2, cy_off[3] + 2500, f"{cy_off[3]/1000:.1f}K",
         ha="center", va="bottom", fontsize=8, color=COLOR_BASELINE, alpha=0.9,
         fontweight="bold")
# Cin=16/32/64 周期数（仅标关闭折叠）
for xi, c_off in zip([4, 5, 6], cy_off[4:]):
    ax2.text(xi - w/2, c_off + 2500, f"{c_off/1000:.1f}K",
             ha="center", va="bottom", fontsize=8, color=COLOR_BASELINE, alpha=0.9,
             fontweight="bold")

# 轴
ax1.set_xticks(x)
ax1.set_xticklabels([str(c) for c in cin_values])
ax1.set_xlabel(r"输入通道数 $C_{\mathrm{in}}$")
ax1.set_ylabel("PE 利用率（%）")
ax1.set_ylim(0, 110)
ax1.set_yticks(np.arange(0, 101, 20))
ax2.set_ylabel("整网周期数")
ax2.set_ylim(0, max(cy_off) * 1.22)
ax1.grid(True, axis="both", which="major", linestyle=":", linewidth=0.5, alpha=0.5, zorder=0)

# 双图例
h1, l1_lbl = ax1.get_legend_handles_labels()
h2, l2_lbl = ax2.get_legend_handles_labels()
ax1.legend(h1 + h2, l1_lbl + l2_lbl, loc="upper left", framealpha=0.95,
           fontsize=9)

plt.tight_layout()
save_figure(fig, "fig5-7-cin-kyfold", Path(__file__).parent)
