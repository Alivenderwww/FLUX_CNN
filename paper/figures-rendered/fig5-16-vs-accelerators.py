"""
图 5.16 渲染：FLUX_CNN 与代表性 CNN 加速器在峰值算力—功耗二维空间的能效定位图
数据来源：paper.md §1.2 表 1.2（峰值算力 / 功耗 / 能效）+ 本工作实测/估算
样式：单坐标系散点图（log-log）
  - x = 峰值算力（GOPS），log 轴
  - y = 功耗（W），log 轴
  - 等能效斜线（45°）= 恒定 TOPS/W 参考线，越靠右下能效越高
  - 形状 = 方=本工作（FLUX_CNN）/ 圆=对照 ASIC / 三角=对照 FPGA
数据口径说明：
  - Eyeriss 峰值已修正为 67 GOPS（168 PE × 200 MHz × 2 ops；原表 168 是 PE 数，非 GOPS）
  - NVDLA 仍按原表 nv_small 配置 128 GOPS（功耗 ~300 mW 为估算）；满配 nv_large=4096 GOPS 待核
  - VTA 无公开功耗数据，本图省略
  - FLUX ASIC 功耗 0.54 W 为 850 MHz / 15.4 mm² 工艺折算估算值（未做后端 PnR）
  - UNPU（65nm，KAIST JSSC'19，真实流片）按 8-bit 权重口径 5.32 TOPS/W：
      峰值 ≈691 GOPS（16-bit 345.6 GOPS × 2，bit-serial 逐位串行），功耗 ≈130 mW（按 5.32 反推）。
      注：UNPU 8-bit 为 8b 权重 × 16b 激活、bit-serial 变精度，与本工作全 INT8、位并行口径不同，仅作同节点量级参照。
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import matplotlib.pyplot as plt
import numpy as np
from _style import (setup_style, save_figure, COLOR_ANNO, COLOR_GUIDE,
                    COLOR_BASELINE, COLOR_IMPROVED, COLOR_THIRD)

setup_style()

# 数据：(加速器名, 峰值GOPS, 功耗W, 是否本工作, 平台, 是否估算)
data = [
    ("FLUX_CNN N=4 ASIC",  1740.0, 0.540, True,  "ASIC", True),   # 估算（工艺折算）
    ("FLUX_CNN N=4 FPGA",   278.2, 1.925, True,  "FPGA", False),  # Vivado 实测估
    ("DianNao",             452.0, 0.485, False, "ASIC", False),
    ("TPU v1",            92000.0, 40.0,  False, "ASIC", False),
    ("Eyeriss",              67.0, 0.278, False, "ASIC", False),  # 修正：168→67 GOPS
    ("NVDLA",               128.0, 0.300, False, "ASIC", True),   # nv_small；功耗估算
    ("Eyeriss v2",          153.6, 0.606, False, "ASIC", False),
    ("Simba",            128000.0, 65.0,  False, "ASIC", False),
    ("OPU",                 160.0, 8.700, False, "FPGA", False),
    ("Gemmini",             512.0, 0.380, False, "ASIC", False),
    ("UNPU",                691.0, 0.130, False, "ASIC", False),  # 65nm 真实流片，8-bit 5.32 TOPS/W
    # VTA：无公开功耗数据，省略
]

X_MIN, X_MAX = 40, 300000
Y_MIN, Y_MAX = 0.08, 150

fig, ax = plt.subplots(figsize=(8.8, 5.8))

# 第 1 步：等能效斜线（恒定 TOPS/W）。功耗 = GOPS / (E·1000)
ISO_LINES = [
    (10.0,  "10 TOPS/W",   55000, 5.5),
    (1.0,   "1 TOPS/W",    55000, 55),
    (0.1,   "0.1 TOPS/W",   7000, 70),
    (0.01,  "0.01 TOPS/W",   800, 80),
]
xs_iso = np.array([X_MIN, X_MAX])
for eff, label, x_lab, y_lab in ISO_LINES:
    ys_iso = xs_iso / (eff * 1000.0)
    ax.plot(xs_iso, ys_iso, linestyle="--", color=COLOR_ANNO, linewidth=0.8,
            alpha=0.65, zorder=1)
    ax.text(x_lab, y_lab, label, fontsize=8, color=COLOR_ANNO, alpha=1.0,
            style="italic", rotation=29, rotation_mode="anchor",
            ha="center", va="bottom", zorder=1)

# FLUX_CNN 自身的两条等能效线（浅蓝灰），便于与竞品横向对比
FLUX_ISO_COLOR = "#7d9bb5"
FLUX_ISO = [
    (3.22, "3.22 TOPS/W", 52000, 17.0),   # N=4 ASIC
    (0.14, "0.14 TOPS/W", 13000, 92.0),   # N=4 FPGA
]
for eff, label, x_lab, y_lab in FLUX_ISO:
    ys_iso = xs_iso / (eff * 1000.0)
    ax.plot(xs_iso, ys_iso, linestyle="-", color=FLUX_ISO_COLOR, linewidth=1.1,
            alpha=0.9, zorder=2)
    ax.text(x_lab, y_lab, label, fontsize=8, color=FLUX_ISO_COLOR,
            fontweight="bold", style="italic", rotation=29,
            rotation_mode="anchor", ha="center", va="bottom", zorder=2)

# 第 2 步：散点。形状=本工作/对照平台；颜色=本工作 vs 对照
def style_for(is_ours, platform):
    if is_ours:
        return dict(marker="s", s=150, color=COLOR_IMPROVED,
                    edgecolor="black", linewidth=1.8, zorder=6)
    if platform == "FPGA":
        return dict(marker="^", s=85, color=COLOR_THIRD, alpha=0.85,
                    edgecolor="white", linewidth=1.0, zorder=4)
    return dict(marker="o", s=80, color=COLOR_BASELINE, alpha=0.80,
                edgecolor="white", linewidth=1.0, zorder=4)

for name, gops, power, is_ours, platform, est in data:
    ax.scatter([gops], [power], **style_for(is_ours, platform))

# 第 3 步：加速器名标注 + 能效数值
LABEL_OFFSETS = {
    "FLUX_CNN N=4 ASIC":  ( 14,  16),
    "FLUX_CNN N=4 FPGA":  ( 16, -26),
    "DianNao":            (  6,  16),
    "TPU v1":             ( -8, -30),
    "Eyeriss":            ( 14,  26),
    "NVDLA":              ( 16,   8),
    "Eyeriss v2":         ( 12,  16),
    "Simba":              ( -8,  20),
    "OPU":                ( 14,  12),
    "Gemmini":            ( 14, -22),
    "UNPU":               ( 18, -16),
}
for name, gops, power, is_ours, platform, est in data:
    eff_tops = gops / power / 1000.0  # TOPS/W
    short = name.replace("FLUX_CNN ", "FLUX_CNN\n")
    tag = "（估算）" if est else ""
    txt = f"{short}{tag}\n{eff_tops:.2f} TOPS/W"
    dx, dy = LABEL_OFFSETS.get(name, (12, 14))
    fontweight = "bold" if is_ours else "normal"
    fontsize = 8.5 if is_ours else 7.8
    color = "black" if is_ours else COLOR_ANNO
    ha = "left" if dx >= 0 else "right"
    ax.annotate(txt, (gops, power), xytext=(dx, dy), textcoords="offset points",
                fontsize=fontsize, color=color, fontweight=fontweight, ha=ha,
                linespacing=1.05,
                arrowprops=dict(arrowstyle="-", color=COLOR_ANNO,
                                linewidth=0.6, alpha=0.5))

ax.set_xscale("log")
ax.set_yscale("log")
ax.set_xlabel("峰值算力（GOPS）        越大越好 →")
ax.set_ylabel("← 越小越好        功耗（W）")
ax.set_xlim(X_MIN, X_MAX)
ax.set_ylim(Y_MIN, Y_MAX)
ax.grid(True, which="major", linestyle=":", linewidth=0.5, alpha=0.45)

# 图例：形状语义
from matplotlib.lines import Line2D
legend_elements = [
    Line2D([0], [0], marker="s", color="none", markerfacecolor=COLOR_IMPROVED,
           markersize=12, markeredgecolor="black", markeredgewidth=1.5,
           label="本工作 FLUX_CNN"),
    Line2D([0], [0], marker="o", color="none", markerfacecolor=COLOR_BASELINE,
           markersize=9, markeredgecolor="white", label="对照 ASIC"),
    Line2D([0], [0], marker="^", color="none", markerfacecolor=COLOR_THIRD,
           markersize=9, markeredgecolor="white", label="对照 FPGA"),
    Line2D([0], [0], linestyle="--", color=COLOR_GUIDE, linewidth=0.8,
           label="等能效参考线"),
]
ax.legend(handles=legend_elements, loc="lower right", framealpha=0.95,
          fontsize=9, ncol=1)

plt.tight_layout()
save_figure(fig, "fig5-16-vs-accelerators", Path(__file__).parent)
