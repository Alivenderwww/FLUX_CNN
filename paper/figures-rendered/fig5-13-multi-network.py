"""
图 5.13 渲染：FLUX_CNN 在多个代表性 CNN 网络上的整网延时—帧率定位图
数据来源：paper.md §5.5.6 表 5.13 + 表 5.14
样式：散点图（log-log）
  - x = 单帧计算量 GMAC，log 轴
  - y = 帧率 fps，log 轴
  - 颜色 = PE 利用率（colormap 渐变）
  - 形状 = 频点（方 = ASIC 850 MHz / 圆 = FPGA 148.5 MHz）
  - 大小 = 网络层数（层数越多越大）
  - 同一网络的 ASIC / FPGA 双频点用细虚线连
（参考 DepFiN Fig. 14 / Fig. 1 多维散点风格）
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import Normalize
import numpy as np
from _style import setup_style, save_figure, COLOR_ANNO, COLOR_GUIDE

setup_style()

# 表 5.13 + 5.14 N=4 数据
# (网络名, GMAC, 层数, ASIC fps @850MHz, FPGA fps @148.5MHz, 平均PE利用率%)
data = [
    ("ResNet11",     0.131, 11, 3765,  658,  72.7),
    ("AlexNet",      1.139,  8,  260,   45,  43.4),
    ("VGG-16",      15.470, 16,   33,    6,  75.4),
    ("ResNet-18",    1.814, 21,  309,   54,  82.3),
    ("ResNet-50",    4.089, 54,  113,   20,  68.0),
    ("MobileNet-V1", 0.569, 28,  711,  124,  59.4),
    ("YOLOv2-tiny",  3.537,  9,  164,   29,  85.0),
]

fig, ax = plt.subplots(figsize=(8.6, 5.6))

# 颜色映射 = PE 利用率（绿色族）
norm = Normalize(vmin=40, vmax=90)
cmap = cm.get_cmap("YlGn")

# 大小映射 = 层数：层数 8-54 映射到 80-260
def size_of(layers):
    return 80 + (layers - 8) * 4.0

# 第 1 步：每个网络，ASIC 与 FPGA 两点用虚线连
for name, gmac, layers, fps_a, fps_f, util in data:
    ax.plot([gmac, gmac], [fps_a, fps_f], linestyle=":",
            color=COLOR_GUIDE, linewidth=0.8, alpha=0.6, zorder=2)

# 第 2 步：散点
asic_x, asic_y, asic_c, asic_s = [], [], [], []
fpga_x, fpga_y, fpga_c, fpga_s = [], [], [], []
for name, gmac, layers, fps_a, fps_f, util in data:
    asic_x.append(gmac); asic_y.append(fps_a); asic_c.append(util); asic_s.append(size_of(layers))
    fpga_x.append(gmac); fpga_y.append(fps_f); fpga_c.append(util); fpga_s.append(size_of(layers))

# ASIC 方块（突出主报告口径）
sc1 = ax.scatter(asic_x, asic_y, s=asic_s, c=asic_c, cmap=cmap, norm=norm,
                 marker="s", edgecolor=COLOR_ANNO, linewidth=1.2, zorder=5)
# FPGA 圆点
sc2 = ax.scatter(fpga_x, fpga_y, s=fpga_s, c=fpga_c, cmap=cmap, norm=norm,
                 marker="o", edgecolor=COLOR_ANNO, linewidth=0.8, zorder=4, alpha=0.85)

# 第 3 步：网络名标在 ASIC 点旁
LABEL_OFFSETS = {
    "ResNet11":     (10,  6),
    "AlexNet":      (10,  6),
    "VGG-16":       (10,  6),
    "ResNet-18":    (-58,  6),
    "ResNet-50":    (10, -14),
    "MobileNet-V1": (10,  6),
    "YOLOv2-tiny":  (10, -14),
}
for name, gmac, layers, fps_a, fps_f, util in data:
    dx, dy = LABEL_OFFSETS.get(name, (10, 6))
    ax.annotate(name, (gmac, fps_a), xytext=(dx, dy), textcoords="offset points",
                fontsize=8.5, color=COLOR_ANNO, fontweight="bold")
    # PE 利用率小字标在 FPGA 点（避免与网络名重叠）
    ax.annotate(f"{util:.0f}%", (gmac, fps_f), xytext=(8, -3), textcoords="offset points",
                fontsize=7.5, color=COLOR_ANNO, alpha=0.7)

ax.set_xscale("log")
ax.set_yscale("log")
ax.set_xlabel("单帧计算量（GMAC）")
ax.set_ylabel("帧率（fps，$N=4$ 配置）")
ax.set_xlim(0.08, 30)
ax.set_ylim(3, 8000)
ax.grid(True, which="both", linestyle=":", linewidth=0.5, alpha=0.5)

# 颜色条 = PE 利用率
cbar = fig.colorbar(sc1, ax=ax, pad=0.02, fraction=0.04)
cbar.set_label("平均 PE 利用率（%）", fontsize=9)
cbar.ax.tick_params(labelsize=8)

# 形状图例
from matplotlib.lines import Line2D
legend_elements = [
    Line2D([0], [0], marker="s", color="none", markerfacecolor="#999999",
           markersize=10, markeredgecolor=COLOR_ANNO, markeredgewidth=1.2,
           label="ASIC @ 850 MHz"),
    Line2D([0], [0], marker="o", color="none", markerfacecolor="#999999",
           markersize=9, markeredgecolor=COLOR_ANNO, markeredgewidth=0.8,
           label="FPGA @ 148.5 MHz"),
]
# 大小图例（层数）
for layers, label in [(8, "8 层"), (28, "28 层"), (54, "54 层")]:
    legend_elements.append(
        Line2D([0], [0], marker="o", color="none", markerfacecolor="#dddddd",
               markersize=np.sqrt(size_of(layers)) * 0.85,
               markeredgecolor=COLOR_ANNO, markeredgewidth=0.6,
               label=label)
    )

ax.legend(handles=legend_elements, loc="lower left", framealpha=0.95,
          fontsize=8, labelspacing=0.9, handletextpad=0.8)

plt.tight_layout()
save_figure(fig, "fig5-13-multi-network", Path(__file__).parent)
