"""
公共样式：IEEE 期刊学术风格
- 英文 / 数字 / 数学公式统一用 Times New Roman（serif，与正文一致）
- 中文用 Microsoft YaHei 作为 CJK 字形回退（保持原配置，本次只调整英文）
- 不在图内写 "图 X.Y" 标题（论文外部由 figure 注释承担）
- 多系列对比：基线 / 提升 / 第三 / 第四 系列各一色，注记单独一色

配色切换：改下方 PALETTE 变量为以下任一名字即可：
  - "default"      原配色：深蓝 #1f4e79 + 深绿 #2e7d32 + 深金 #b8860b
  - "morandi"      莫兰迪低饱和：雾蓝 + 苔绿 + 陶土
  - "colorbrewer"  ColorBrewer Dark2：绿松 + 暖橙 + 雾紫
  - "tufte"        Tufte 灰阶+点色：深灰主调 + 钴蓝/暖橙强调
  - "oxford"       牛津+砖色经典：牛津深蓝 + 砖橙 + 橄榄绿
"""

import matplotlib as mpl
import matplotlib.pyplot as plt

# ────────────────────────────────────────────────────────────────
# 配色切换：改下行即可
# ────────────────────────────────────────────────────────────────
PALETTE = "tufte"

PALETTES = {
    "default": {
        # 原始配色：深蓝 + 深绿 + 深金
        "BASELINE": "#1f4e79",  # 深蓝（基线 / 关闭 / N=1 / 启用前）
        "IMPROVED": "#2e7d32",  # 深绿（提升 / 开启 / N=4 / 启用后）
        "THIRD":    "#b8860b",  # 深金
        "FOURTH":   "#6a4c93",  # 深紫
        "FIFTH":    "#5d6d7e",
        "SIXTH":    "#7b8b8e",
        "ANNO":     "#404040",  # 注记
        "GUIDE":    "#808080",  # 辅助线
        "GRID":     "#cccccc",
        "FIRE":     "#1f4e79",
        "IDLE":     "#d49a4d",
    },
    "morandi": {
        # 莫兰迪低饱和：水彩报告风
        "BASELINE": "#5c7a99",  # 雾蓝
        "IMPROVED": "#8a9a5b",  # 苔绿
        "THIRD":    "#b8956a",  # 陶土
        "FOURTH":   "#8a6e8e",  # 灰紫
        "FIFTH":    "#7a8b8e",
        "SIXTH":    "#a09a82",
        "ANNO":     "#3a3a3a",
        "GUIDE":    "#999999",
        "GRID":     "#cccccc",
        "FIRE":     "#5c7a99",
        "IDLE":     "#c4a875",
    },
    "colorbrewer": {
        # ColorBrewer Dark2：经典数据可视化配色
        "BASELINE": "#1b9e77",  # 绿松
        "IMPROVED": "#d95f02",  # 暖橙
        "THIRD":    "#7570b3",  # 雾紫
        "FOURTH":   "#66a61e",  # 草绿
        "FIFTH":    "#a6761d",  # 棕黄
        "SIXTH":    "#666666",
        "ANNO":     "#404040",
        "GUIDE":    "#808080",
        "GRID":     "#cccccc",
        "FIRE":     "#1b9e77",
        "IDLE":     "#e6ab02",
    },
    "tufte": {
        # Tufte 灰阶 + 点色：主线灰、强调用彩
        "BASELINE": "#4a4a4a",  # 深灰
        "IMPROVED": "#4682b4",  # 钴蓝
        "THIRD":    "#d97757",  # 暖橙
        "FOURTH":   "#6e6e6e",  # 中灰
        "FIFTH":    "#8e7b6b",
        "SIXTH":    "#9aa19a",
        "ANNO":     "#2a2a2a",
        "GUIDE":    "#888888",
        "GRID":     "#d0d0d0",
        "FIRE":     "#4a4a4a",
        "IDLE":     "#b8956a",
    },
    "oxford": {
        # 牛津 + 砖色经典：浓郁老牌期刊
        "BASELINE": "#002147",  # 牛津深蓝
        "IMPROVED": "#b8581d",  # 砖橙
        "THIRD":    "#6b7c32",  # 橄榄绿
        "FOURTH":   "#7a4e3a",  # 栗棕
        "FIFTH":    "#4a4e6a",
        "SIXTH":    "#8a8174",
        "ANNO":     "#2a2a2a",
        "GUIDE":    "#7a7a7a",
        "GRID":     "#cccccc",
        "FIRE":     "#002147",
        "IDLE":     "#c4825f",
    },
}

_p = PALETTES[PALETTE]


def _lighten(hex_c, amount=0.45):
    """把颜色向白色插值，amount 0=原色，1=纯白"""
    from matplotlib.colors import to_rgb, to_hex
    r, g, b = to_rgb(hex_c)
    return to_hex((r + (1 - r) * amount,
                   g + (1 - g) * amount,
                   b + (1 - b) * amount))


COLOR_BASELINE = _p["BASELINE"]
COLOR_IMPROVED = _p["IMPROVED"]
COLOR_THIRD    = _p["THIRD"]
COLOR_FOURTH   = _p["FOURTH"]
COLOR_ANNO     = _p["ANNO"]
COLOR_GUIDE    = _p["GUIDE"]
COLOR_GRID     = _p["GRID"]
COLOR_FIRE     = _p["FIRE"]
COLOR_IDLE     = _p["IDLE"]
# 派生浅色版（用于柱图同色族对比，如 stride=1/stride=2）
COLOR_BASELINE_LIGHT = _lighten(COLOR_BASELINE)
COLOR_IMPROVED_LIGHT = _lighten(COLOR_IMPROVED)
COLOR_THIRD_LIGHT    = _lighten(COLOR_THIRD)
COLOR_PALETTE_6 = [_p["BASELINE"], _p["IMPROVED"], _p["THIRD"],
                   _p["FOURTH"],   _p["FIFTH"],    _p["SIXTH"]]


def setup_style():
    # 字体策略：英文/数字/公式用 Times New Roman（serif），中文用 Microsoft YaHei 作 CJK 回退
    # 关键：font.family 用 list 形式才触发 matplotlib per-glyph fallback（TNR 缺字形时回退 YaHei）
    mpl.rcParams.update({
        "font.family": ["Times New Roman", "Microsoft YaHei", "DejaVu Serif"],
        "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
        "font.sans-serif": ["Microsoft YaHei", "SimHei", "Arial", "DejaVu Sans"],
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
        # mathtext 用 STIX (Times 同族 serif), 与英文字体一致
        "mathtext.fontset": "stix",
        "mathtext.default": "regular",
    })


def save_figure(fig, name_stem, out_dir):
    """保存 PNG (300 DPI) → png/ 子目录；SVG → svg/ 子目录。
    切换 PALETTE 时输出会覆盖前一份，方便比对。"""
    from pathlib import Path
    out_dir = Path(out_dir)
    png_dir = out_dir / "png"
    svg_dir = out_dir / "svg"
    png_dir.mkdir(exist_ok=True)
    svg_dir.mkdir(exist_ok=True)
    png = png_dir / f"{name_stem}.png"
    svg = svg_dir / f"{name_stem}.svg"
    fig.savefig(png, dpi=300)
    fig.savefig(svg)
    print(f"[OK] {name_stem}.png + {name_stem}.svg  (palette={PALETTE})")
