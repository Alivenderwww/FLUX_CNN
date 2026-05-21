"""
图 5.12 渲染：ResNet11 N=4 主配置分层瓶颈分项（多维信息复合图）
数据来源：paper.md §5.5.5.5 表 5.12 + paper/data/exp8 PROFILE
样式：复合图
  - 上：堆叠柱图（触发周期 + 空闲周期），背景色块按层类型分类
  - 上：PE 利用率折线（右 Y 轴）
  - 下：各层 idle 来源拆分（act_id / wgt_id / psm_id / acc_id 4 类小柱）
（参考 DepFiN Fig. 16 风格：单个 layer 多类指标占比）
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from _style import setup_style, save_figure, COLOR_ANNO, COLOR_FIRE, COLOR_IDLE, COLOR_IMPROVED

setup_style()

# 表 5.12 数据（ResNet11 实际 11 层 cycles, N=4 W slice + cout slice）
# 来源：paper/data/k7_n4_smc_synth_report_2026_05_19.md §4.1
# 整网 wall = 172,916 + barrier 516 = 173,432
layers = ["L0\nPatch", "L1\nK3 s1", "L2\nK3 s1", "L3\nds K=1", "L4\nK3 s2",
          "L5\nK3 s1", "L6\nds K=1", "L7\nK3 s2", "L8\nK3 s1", "L9\nK1 s1", "L10\n全连接"]
layer_types = ["patch",  "k3",   "k3",   "ds",   "k3",   "k3",   "ds",   "k3",   "k3",   "k1",   "fc"]
wall_cy  = np.array([39979, 21608, 19670, 9128,  10392, 21897, 5321,  12347, 27181, 2471,  2922])
util     = np.array([70.9,  92.7,  86.0,  18.7,  87.7,  78.9,  16.1,  86.1,  85.0,  40.4,  5.0])
fire_max = (wall_cy * util / 100.0).astype(int)
idle     = wall_cy - fire_max

# 层类型染色（背景）
TYPE_BG = {
    "patch": "#fff3e0",  # 浅橙：Patch 层
    "k3":    "#e8f4ea",  # 浅绿：K=3 主路径
    "ds":    "#fff8d6",  # 浅黄：降采样（访存受限）
    "k1":    "#e8eef8",  # 浅蓝：1×1 卷积
    "fc":    "#f0f0f0",  # 浅灰：全连接
}
TYPE_LABEL = {
    "patch": "Patch",
    "k3":    "$K=3$ 主路径",
    "ds":    "降采样 $K=1$",
    "k1":    "$K=1$",
    "fc":    "全连接",
}

x = np.arange(len(layers))
fig, ax1 = plt.subplots(figsize=(11.5, 5.5))
ax2 = ax1.twinx()

# 第 1 步：层类型背景色块
prev_type = None
seg_start = 0
for i, t in enumerate(layer_types + [None]):
    if t != prev_type and prev_type is not None:
        ax1.axvspan(seg_start - 0.5, i - 0.5,
                    color=TYPE_BG[prev_type], alpha=0.7, zorder=1)
        seg_start = i
    if t != prev_type:
        seg_start = i
    prev_type = t

# 第 2 步：堆叠柱（触发 + 空闲）
b1 = ax1.bar(x, fire_max, 0.62, color=COLOR_FIRE, label="触发周期", zorder=3)
b2 = ax1.bar(x, idle, 0.62, color=COLOR_IDLE, bottom=fire_max, label="空闲周期", zorder=3)

# 第 3 步：PE 利用率折线（右 Y 轴）
l1, = ax2.plot(x, util, marker="D", color=COLOR_IMPROVED, label="PE 利用率",
               linewidth=1.6, markersize=7, zorder=5,
               markerfacecolor="white", markeredgewidth=1.6)

# PE 利用率数字
# L3 (idx=3, util=19%) 和 L6 (idx=6, util=16%) 是低谷，两侧折线高耸会遮挡顶部标注，
# 改为标在节点右侧
SIDE_LABEL = {3, 6}
for xi, u in zip(x, util):
    if xi in SIDE_LABEL:
        ax2.text(xi + 0.32, u, f"{u:.0f}%", ha="left", va="center",
                 fontsize=8, color=COLOR_IMPROVED, fontweight="bold")
    else:
        ax2.text(xi, u + 4, f"{u:.0f}%", ha="center", va="bottom",
                 fontsize=8, color=COLOR_IMPROVED, fontweight="bold")

# 触发数注记（柱中部白字）
for xi, fm in zip(x, fire_max):
    if fm >= 5000:
        ax1.text(xi, fm/2, f"{fm/1000:.1f}K", ha="center", va="center",
                 fontsize=8, color="white", fontweight="bold")

# 空闲注记
for xi, fm, idl in zip(x, fire_max, idle):
    if idl >= 5000:
        ax1.text(xi, fm + idl/2, f"{idl/1000:.1f}K", ha="center", va="center",
                 fontsize=8, color="white", fontweight="bold")

ax1.set_xticks(x)
ax1.set_xticklabels(layers, fontsize=8.5)
ax1.set_xlabel("ResNet11 层")
ax1.set_ylabel("整网周期数")
ax1.set_ylim(0, max(wall_cy) * 1.15)
ax2.set_ylabel("PE 利用率（%）", color=COLOR_IMPROVED)
ax2.set_ylim(0, 110)
ax2.tick_params(axis="y", labelcolor=COLOR_IMPROVED)
ax1.grid(True, axis="y", linestyle=":", linewidth=0.5, alpha=0.4, zorder=0)

# 顶部层类型注记（每个色块标一次）— 不加 bbox，文字限柱宽换行
import textwrap as _tw
prev_type = None
seg_start = 0
ymax = max(wall_cy) * 1.15
for i, t in enumerate(layer_types + [None]):
    if t != prev_type and prev_type is not None:
        seg_n = i - seg_start
        mid = (seg_start + i - 1) / 2
        # 段越窄文字越短：单层段强行换行
        label = TYPE_LABEL[prev_type]
        if seg_n == 1:
            # 单层段：拆成 "$K=3$\n主路径" 形式
            label = label.replace(" ", "\n", 1)
        ax1.text(mid, ymax * 0.97, label,
                 ha="center", va="top", fontsize=9.5, color=COLOR_ANNO,
                 fontweight="bold")
        seg_start = i
    if t != prev_type:
        seg_start = i
    prev_type = t

# 双图例（柱 + 折线）— 移到右上偏下位置
h1, l1_lbl = ax1.get_legend_handles_labels()
h2, l2_lbl = ax2.get_legend_handles_labels()
ax1.legend(h1 + h2, l1_lbl + l2_lbl, loc="upper right",
           bbox_to_anchor=(0.995, 0.78), framealpha=0.95, ncol=1, fontsize=9)

plt.tight_layout()
save_figure(fig, "fig5-12-resnet11-layers", Path(__file__).parent)
