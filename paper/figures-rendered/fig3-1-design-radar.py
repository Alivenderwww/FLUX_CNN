"""
图 3.1 渲染：FLUX_CNN 设计目标雷达图
数据来源：§3.2 设计目标与约束（端侧推理 vs 数据中心推理两类典型目标）
样式：5 维雷达图，两条曲线 (端侧 vs 数据中心) + FLUX_CNN 实测点
论证作用：直观展示 FLUX_CNN 在 5 个设计维度上的折衷点 ——
          能效、灵活性、可演进性贴近端侧目标，峰值算力低于数据中心目标。
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import matplotlib.pyplot as plt
import numpy as np
from _style import (
    setup_style, save_figure,
    COLOR_BASELINE, COLOR_IMPROVED, COLOR_THIRD, COLOR_ANNO, COLOR_GRID,
)

setup_style()

# ----------------------------------------------------------------------------
# 5 维设计目标（顺序按论证逻辑：吞吐→能效→灵活性→资源→可演进）
# 评分尺度: 0 (不重要 / 不达标) ~ 100 (核心目标 / 已达标)
# ----------------------------------------------------------------------------
axes = ["峰值算力",
        "能效",
        "调度灵活性",
        "资源占用",
        "可演进性"]

# 三组曲线：
#  - 端侧推理目标:    重能效/灵活/可演进, 不要求超高峰值算力
#  - 数据中心推理目标: 重峰值算力/资源利用率, 能效次要
#  - FLUX_CNN 实测:    贴近端侧曲线
endpoint = np.array([60, 90, 90, 30, 70])    # 端侧目标
dcenter  = np.array([100, 60, 30, 100, 30])  # 数据中心目标
flux     = np.array([55, 85, 92, 70, 95])    # FLUX_CNN 实测 + 设计预期

# 闭合多边形 (首尾相连)
N = len(axes)
angles = np.linspace(0, 2 * np.pi, N, endpoint=False).tolist()
angles += angles[:1]
endpoint = np.concatenate([endpoint, endpoint[:1]])
dcenter  = np.concatenate([dcenter,  dcenter[:1]])
flux     = np.concatenate([flux,     flux[:1]])

# ----------------------------------------------------------------------------
# 绘图
# ----------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(7.5, 7.0), subplot_kw=dict(polar=True))

# 端侧目标 (虚线参考)
ax.plot(angles, endpoint, color=COLOR_IMPROVED, linewidth=1.8,
        linestyle="--", marker="o", markersize=6, label="端侧推理目标")
ax.fill(angles, endpoint, color=COLOR_IMPROVED, alpha=0.10)

# 数据中心目标 (虚线参考)
ax.plot(angles, dcenter, color=COLOR_THIRD, linewidth=1.8,
        linestyle="--", marker="s", markersize=6, label="数据中心推理目标")
ax.fill(angles, dcenter, color=COLOR_THIRD, alpha=0.08)

# FLUX_CNN 实测 (实线主角)
ax.plot(angles, flux, color=COLOR_BASELINE, linewidth=2.6,
        marker="D", markersize=8, label="FLUX_CNN ($N{=}4$ DSP)")
ax.fill(angles, flux, color=COLOR_BASELINE, alpha=0.22)

# 轴样式
ax.set_xticks(angles[:-1])
ax.set_xticklabels(axes, fontsize=11)
ax.set_ylim(0, 105)
ax.set_yticks([20, 40, 60, 80, 100])
ax.set_yticklabels([])  # 去掉刻度数字（用户反馈：含义不明、位置突兀）
ax.grid(True, color=COLOR_GRID, linestyle=":", linewidth=0.6)
ax.spines["polar"].set_color(COLOR_ANNO)
ax.spines["polar"].set_linewidth(0.8)

# 图例
ax.legend(loc="upper right", bbox_to_anchor=(1.30, 1.10),
          framealpha=0.95, fontsize=10)

plt.tight_layout()
save_figure(fig, "fig3-1-design-radar", Path(__file__).parent)
