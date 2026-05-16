"""
图 5.11 渲染：核数 N=4 与图像宽度 W 扫频加速比（理论 vs 实际）
数据来源：paper.md §5.5.5.4 表 5.11
样式：单坐标系多曲线
  - 4× 线性上限（虚线参考）
  - halo 理论上限：4 / (1 + 2/sub_W)，反映边界冗余列对加速比的几何上限
  - 实际加速比（实测）
  - N=4 PE 利用率（次坐标轴）
（参考 DepFiN Fig. 14 多曲线风格）
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import matplotlib.pyplot as plt
import numpy as np
from _style import setup_style, save_figure, COLOR_BASELINE, COLOR_IMPROVED, COLOR_THIRD, COLOR_ANNO, COLOR_GUIDE

setup_style()

# 表 5.11 数据
W_values   = np.array([16, 32, 64, 128])
sub_W      = W_values // 4              # 每核子图像宽度
halo       = 2                          # K=3 stride=1 的 halo 列宽
speedup_real = np.array([1.20, 2.43, 3.71, 3.81])
util_n4      = np.array([28.0, 58.4, 90.8, 93.0])  # %

# 理论加速比上限：halo 几何模型 4 / (1 + halo / sub_W)
speedup_halo = 4.0 / (1.0 + halo / sub_W)
halo_pct     = halo / sub_W * 100  # halo 占子图像比例 %

fig, ax1 = plt.subplots(figsize=(8.6, 5.2))
ax2 = ax1.twinx()

# 4× 线性上限参考线
ax1.axhline(y=4.0, color=COLOR_GUIDE, linestyle="--", linewidth=0.8, alpha=0.7)
ax1.text(W_values[0], 4.05, "4× 线性上限",
         fontsize=9, color=COLOR_ANNO, ha="left", va="bottom")

# halo 理论上限（虚线）
l_halo, = ax1.plot(W_values, speedup_halo, marker="^", color=COLOR_THIRD,
                   linestyle="--", label="边界冗余列理论上限",
                   linewidth=1.4, markersize=7, alpha=0.85, zorder=4)

# 实际加速比（实线）
l_real, = ax1.plot(W_values, speedup_real, marker="s", color=COLOR_BASELINE,
                   label="实测加速比",
                   linewidth=1.8, markersize=8, zorder=5)

# 右 Y：N=4 PE 利用率
l_util, = ax2.plot(W_values, util_n4, marker="o", color=COLOR_IMPROVED,
                   label="$N=4$ PE 利用率",
                   linewidth=1.4, markersize=6, alpha=0.85, zorder=4)

# 数据点标注
for w, sp, sh, u, hp in zip(W_values, speedup_real, speedup_halo, util_n4, halo_pct):
    ax1.text(w, sp + 0.18, f"{sp:.2f}×", ha="center", va="bottom",
             fontsize=9, color=COLOR_BASELINE, fontweight="bold")
    ax1.text(w, sh - 0.22, f"{sh:.2f}×", ha="center", va="top",
             fontsize=8, color=COLOR_THIRD)

# 关键观察注记
ax1.annotate("PE 列覆盖下限\n(子图像宽度 = 4)",
             xy=(16, 1.20), xytext=(20, 0.35),
             fontsize=9.5, color=COLOR_ANNO,
             arrowprops=dict(arrowstyle="->", color=COLOR_ANNO, linewidth=0.8),
             ha="left", va="bottom")

ax1.annotate("超过理论上限：\n边界冗余可被 SDP 复用",
             xy=(128, 3.81), xytext=(56, 2.6),
             fontsize=9.5, color=COLOR_ANNO,
             arrowprops=dict(arrowstyle="->", color=COLOR_ANNO, linewidth=0.8),
             ha="left")

# 轴
ax1.set_xscale("log", base=2)
ax1.set_xticks(W_values)
# x 轴标签合并 W 值 + 边界冗余占比（避免独立 text 注记触发 layout 异常）
ax1.set_xticklabels([f"{w}\n(边界占{hp:.1f}%)" for w, hp in zip(W_values, halo_pct)],
                    fontsize=8.5)
ax1.set_xlabel("图像宽度 $W$")
ax1.set_ylabel("加速比（相对 $N=1$）", color=COLOR_BASELINE)
ax1.set_ylim(0, 4.5)
ax1.tick_params(axis="y", labelcolor=COLOR_BASELINE)
ax2.set_ylabel("PE 利用率（%）", color=COLOR_IMPROVED)
# 关键：右 Y 范围扩到 113，使 PE 100% 屏幕高度对齐到左 Y 的 4× 上限
ax2.set_ylim(0, 113)
ax2.set_yticks([0, 20, 40, 60, 80, 100])
ax2.tick_params(axis="y", labelcolor=COLOR_IMPROVED)
ax1.grid(True, axis="both", which="major", linestyle=":", linewidth=0.5, alpha=0.5)

# 三图例
ax1.legend(handles=[l_real, l_halo, l_util], loc="lower right",
           framealpha=0.95, fontsize=9)

plt.tight_layout()
save_figure(fig, "fig5-11-n-w-speedup", Path(__file__).parent)
