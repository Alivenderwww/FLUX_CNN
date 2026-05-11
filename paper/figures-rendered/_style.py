"""
公共样式：IEEE 期刊学术风格
- 中文用 Microsoft YaHei，数学公式用 Computer Modern
- 不在图内写 "图 X.Y" 标题（论文外部由 figure 注释承担）
- 注记色 #404040 学术深灰
- 多系列对比：基线深蓝，提升深绿
"""

import matplotlib as mpl
import matplotlib.pyplot as plt

# 配色（学术论文常用）
COLOR_BASELINE = "#1f4e79"  # 深蓝（基线 / 关闭 / N=1 / 启用前）
COLOR_IMPROVED = "#2e7d32"  # 深绿（提升 / 开启 / N=4 / 启用后）
COLOR_THIRD    = "#b8860b"  # 深金（第三系列）
COLOR_FOURTH   = "#6a4c93"  # 深紫（第四系列）
COLOR_ANNO     = "#404040"  # 注记深灰
COLOR_GUIDE    = "#808080"  # 辅助线灰
COLOR_GRID     = "#cccccc"

# 瓶颈类型染色（用于堆叠柱）
COLOR_FIRE     = "#1f4e79"  # 触发（蓝）
COLOR_IDLE     = "#d49a4d"  # 空闲（橙黄）

# 加速器横向对比 6 色（避免红色）
COLOR_PALETTE_6 = ["#1f4e79", "#2e7d32", "#b8860b", "#6a4c93", "#5d6d7e", "#7b8b8e"]


def setup_style():
    mpl.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Microsoft YaHei", "Source Han Sans CN", "SimHei", "Arial", "DejaVu Sans"],
        "font.size": 10,
        "axes.labelsize": 11,
        "axes.titlesize": 11,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.fontsize": 9,
        "axes.linewidth": 0.8,
        "axes.edgecolor": "black",
        "axes.spines.top": True,
        "axes.spines.right": True,
        "lines.linewidth": 1.5,
        "lines.markersize": 6,
        "savefig.bbox": "tight",
        "savefig.dpi": 300,
        "axes.unicode_minus": False,
        "mathtext.fontset": "cm",
    })


def save_figure(fig, name_stem, out_dir):
    """保存 PNG (300 DPI) + SVG"""
    from pathlib import Path
    out_dir = Path(out_dir)
    png = out_dir / f"{name_stem}.png"
    svg = out_dir / f"{name_stem}.svg"
    fig.savefig(png, dpi=300)
    fig.savefig(svg)
    print(f"[OK] saved: {png.name} + {svg.name}")
