"""
图 5.13 渲染：FLUX_CNN 在多个代表性 CNN 网络上的整网延时—帧率定位图
数据来源：paper.md §5.5.6 表 5.13 + 表 5.14
样式：散点图（log-log）
  - x = 单帧计算量 GMAC，log 轴
  - y = 帧率 fps，log 轴
  - 颜色 = 频点（深蓝 ASIC 850 MHz / 深金 FPGA 148.5 MHz）
  - 形状 = 方块（统一；本图所有数据均为本工作，与图 5.16 的"方块=本工作"语义一致）
  - 大小 = 网络层数
  - 同一网络的双频点用细虚线连
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import matplotlib.pyplot as plt
import numpy as np
from _style import setup_style, save_figure, COLOR_BASELINE, COLOR_THIRD, COLOR_ANNO, COLOR_GUIDE

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

fig, ax = plt.subplots(figsize=(9.0, 5.6))

# 大小映射 = 层数：层数 8-54 → 100-300
def size_of(layers):
    return 100 + (layers - 8) * 4.5

# 第 1 步：每个网络，ASIC 与 FPGA 两点用虚线连
for name, gmac, layers, fps_a, fps_f, util in data:
    ax.plot([gmac, gmac], [fps_a, fps_f], linestyle="--",
            color=COLOR_GUIDE, linewidth=0.9, alpha=0.6, zorder=2)

# 第 2 步：散点 — 全部方块；颜色区分频点；大小区分层数
asic_x, asic_y, asic_s = [], [], []
fpga_x, fpga_y, fpga_s = [], [], []
for name, gmac, layers, fps_a, fps_f, util in data:
    asic_x.append(gmac); asic_y.append(fps_a); asic_s.append(size_of(layers))
    fpga_x.append(gmac); fpga_y.append(fps_f); fpga_s.append(size_of(layers))

# ASIC 深蓝方块（突出主报告）
ax.scatter(asic_x, asic_y, s=asic_s, color=COLOR_BASELINE,
           marker="s", edgecolor="white", linewidth=1.6, zorder=5,
           label="ASIC @ 850 MHz")
# FPGA 深金方块（次报告）
ax.scatter(fpga_x, fpga_y, s=fpga_s, color=COLOR_THIRD,
           marker="s", edgecolor="white", linewidth=1.2, zorder=4, alpha=0.95,
           label="FPGA @ 148.5 MHz")

# 第 3 步：网络名标注 — 统一标在 ASIC 点右上方且偏移大，配引导线避免遮挡
LABEL_OFFSETS = {  # 全部朝右上偏移
    "ResNet11":     ( 14,  18),
    "AlexNet":      ( 14,  18),
    "VGG-16":       (-86,   8),
    "ResNet-18":    ( 14,  18),
    "ResNet-50":    ( 14,  18),
    "MobileNet-V1": ( 14,  18),
    "YOLOv2-tiny":  ( 60,  46),
}
for name, gmac, layers, fps_a, fps_f, util in data:
    dx, dy = LABEL_OFFSETS.get(name, (14, 18))
    label_text = f"{name}\nPE {util:.0f}%"
    ax.annotate(label_text, (gmac, fps_a),
                xytext=(dx, dy), textcoords="offset points",
                fontsize=9, color=COLOR_ANNO, fontweight="bold",
                ha="left" if dx >= 0 else "left",
                arrowprops=dict(arrowstyle="-", color=COLOR_ANNO,
                                linewidth=0.6, alpha=0.5))

# fps 数值标在点旁（小字）— 偏移加大避开方块本身
for name, gmac, layers, fps_a, fps_f, util in data:
    ax.annotate(f"{fps_a}", (gmac, fps_a), xytext=(-12, -14),
                textcoords="offset points",
                fontsize=8, color=COLOR_BASELINE, ha="right", va="top")
    ax.annotate(f"{fps_f}", (gmac, fps_f), xytext=(-12, 0),
                textcoords="offset points",
                fontsize=8, color=COLOR_THIRD, ha="right", va="center")

ax.set_xscale("log")
ax.set_yscale("log")
ax.set_xlabel("单帧计算量（GMAC）")
ax.set_ylabel("帧率（fps，$N=4$ 配置）")
ax.set_xlim(0.08, 30)
ax.set_ylim(3, 12000)
ax.grid(True, which="both", linestyle=":", linewidth=0.5, alpha=0.5)

# 形状/大小图例
from matplotlib.lines import Line2D
legend_elements = [
    Line2D([0], [0], marker="s", color="none", markerfacecolor=COLOR_BASELINE,
           markersize=11, markeredgecolor="white", markeredgewidth=1.5,
           label="ASIC @ 850 MHz"),
    Line2D([0], [0], marker="s", color="none", markerfacecolor=COLOR_THIRD,
           markersize=11, markeredgecolor="white", markeredgewidth=1.2,
           label="FPGA @ 148.5 MHz"),
]
# 大小图例（层数）
for layers, label in [(8, "8 层"), (28, "28 层"), (54, "54 层")]:
    legend_elements.append(
        Line2D([0], [0], marker="s", color="none", markerfacecolor="#bbbbbb",
               markersize=np.sqrt(size_of(layers)) * 0.85,
               markeredgecolor=COLOR_ANNO, markeredgewidth=0.5,
               label=label)
    )

ax.legend(handles=legend_elements, loc="lower left", framealpha=0.95,
          fontsize=9, labelspacing=0.9, handletextpad=0.9)

plt.tight_layout()
save_figure(fig, "fig5-13-multi-network", Path(__file__).parent)
