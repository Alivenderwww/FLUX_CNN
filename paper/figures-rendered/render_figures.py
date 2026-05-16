"""
一键渲染本目录下所有图：TikZ (.tex) + matplotlib (.py) → PNG + SVG + EMF。

设计理念:
  - figX-Y-NAME.tex : TikZ 写的概念示意图 / 架构图 / 流程图
  - algoX-Y-NAME.tex: algorithm2e 写的伪代码算法图
  - figX-Y-NAME.py  : matplotlib 写的数据分析图 (能效柱图 / 雷达 / 曲线扫描)
  - _tikz-style.tex : 共享样式包 (颜色 / 字体 / 节点样式宏), 不单独编译
  - _style.py       : matplotlib 共享样式 (调色板 / 字体 / save_figure), 不单独执行
  - _*.tex / _*.py  : 任何下划线开头的文件都跳过 (内部 include)

依赖:
  - MiKTeX (xelatex) - Windows: C:\\Program Files\\MiKTeX\\miktex\\bin\\x64\\xelatex.exe
  - PyMuPDF (fitz)   - 已装在 toolchain/.venv
  - matplotlib       - 已装在 toolchain/.venv
  - Inkscape         - 可选, 用于 SVG → EMF 转换 (Word 嵌入矢量首选)
                       Windows: C:\\Program Files\\Inkscape\\bin\\inkscape.exe
                       未装时仅跳过 EMF, 不阻塞 PNG/SVG

用法:
  C:\\_Project\\FLUX_CNN\\toolchain\\.venv\\Scripts\\python.exe render_figures.py
  ... fig2-4        # 只编译文件名含 fig2-4 的源
  ... algo          # 只编译算法图
  ... fig2 fig3     # 多个子串过滤
  ... --keep-aux    # 保留中间文件 (aux/log/out) 用于调试
  ... --no-emf      # 跳过 EMF 输出 (加速; PNG/SVG 仍生成)
  ... --tex-only    # 只渲染 .tex 跳过 .py
  ... --py-only     # 只渲染 .py 跳过 .tex

输出目录布局:
  pdf/figX-Y.pdf    # TikZ xelatex 产出 (仅 .tex 来源)
  png/figX-Y.png    # 300 DPI 栅格 (.tex 与 .py 均产出)
  svg/figX-Y.svg    # 矢量 (.tex 与 .py 均产出)
  emf/figX-Y.emf    # Windows EMF, Word 嵌入矢量首选 (.tex 与 .py 均产出)

EMF 适用场景:
  Word 文档插入 EMF, 导出 PDF 后仍是矢量可放大无锯齿。
  比 SVG 更稳 (Word 内部按 EMF 渲染矢量, SVG 走 ViewBox 解析容易跑版)。
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

DPI = 300
PDF_DIR_NAME = "pdf"
PNG_DIR_NAME = "png"
SVG_DIR_NAME = "svg"
EMF_DIR_NAME = "emf"

# MiKTeX 常见安装路径（按可能性排序）
MIKTEX_CANDIDATES = [
    r"C:\Program Files\MiKTeX\miktex\bin\x64",
    r"C:\Program Files (x86)\MiKTeX\miktex\bin\x64",
    os.path.expandvars(r"%LOCALAPPDATA%\Programs\MiKTeX\miktex\bin\x64"),
    os.path.expandvars(r"%LOCALAPPDATA%\Programs\MiKTeX 2.9\miktex\bin\x64"),
]

# Inkscape 常见安装路径 (用于 SVG -> EMF, Word 矢量嵌入首选)
INKSCAPE_CANDIDATES = [
    r"C:\Program Files\Inkscape\bin",
    r"C:\Program Files (x86)\Inkscape\bin",
    os.path.expandvars(r"%LOCALAPPDATA%\Programs\Inkscape\bin"),
]


def find_xelatex() -> str:
    p = shutil.which("xelatex")
    if p:
        return p
    for base in MIKTEX_CANDIDATES:
        exe = Path(base) / "xelatex.exe"
        if exe.exists():
            return str(exe)
    print("[ERROR] xelatex 未找到。请装 MiKTeX 或确认它在 PATH 中。", file=sys.stderr)
    sys.exit(1)


def find_inkscape() -> str | None:
    """寻找 Inkscape; 找不到返回 None (EMF 输出跳过, 不阻塞 PNG/SVG)。"""
    p = shutil.which("inkscape")
    if p:
        return p
    for base in INKSCAPE_CANDIDATES:
        exe = Path(base) / "inkscape.exe"
        if exe.exists():
            return str(exe)
    return None


def check_fitz():
    try:
        import fitz
        return fitz
    except ImportError:
        print("[ERROR] PyMuPDF (fitz) 未安装。", file=sys.stderr)
        print("  pip install pymupdf", file=sys.stderr)
        sys.exit(1)


def pdf_to_png(pdf_path: Path, png_path: Path, fitz, dpi: int = DPI) -> bool:
    """PyMuPDF 把单页 PDF 渲染为 PNG。"""
    try:
        doc = fitz.open(str(pdf_path))
        if doc.page_count < 1:
            print(f"[FAIL] PDF 无页面 ({pdf_path.name})")
            return False
        page = doc.load_page(0)
        zoom = dpi / 72.0
        pix = page.get_pixmap(matrix=fitz.Matrix(zoom, zoom), alpha=False)
        png_path.parent.mkdir(parents=True, exist_ok=True)
        pix.save(str(png_path))
        doc.close()
        return True
    except Exception as e:
        print(f"[FAIL] PyMuPDF 转 PNG 失败 ({pdf_path.name}): {e}")
        return False


def pdf_to_svg(pdf_path: Path, svg_path: Path, fitz) -> bool:
    """PyMuPDF 单页 PDF → SVG（矢量，论文 Word 缩放无锯齿）。

    text_as_path=True: 把 PDF 中所有文字转为矢量路径而非 <text> 引用。
    xelatex 数学公式使用 Computer Modern Math 字体, 文本引用模式下 SVG 渲染
    会因找不到 CMR 字体回退到默认无衬线字体, 导致数学公式字形丢失;
    转路径后 SVG 完全与字体文件解耦, 下游 PNG / EMF 都保留原 PDF 字形。
    代价: SVG 略大、文本不可编辑 (论文图表场景无影响)。
    """
    try:
        doc = fitz.open(str(pdf_path))
        if doc.page_count < 1:
            return False
        page = doc.load_page(0)
        svg_text = page.get_svg_image(text_as_path=True)
        svg_path.parent.mkdir(parents=True, exist_ok=True)
        svg_path.write_text(svg_text, encoding="utf-8")
        doc.close()
        return True
    except Exception as e:
        print(f"[WARN] PyMuPDF 转 SVG 失败 ({pdf_path.name}): {e}")
        return False


def _blend_hex_to_white(hex_color: str, alpha: float) -> str:
    """把 #RRGGBB + alpha 预合成到白色背景, 返回 #RRGGBB。

    EMF 不支持 alpha channel, Inkscape 转 EMF 时会直接丢弃 opacity,
    使得 alpha=0.1 的淡填充变成饱和实色。预合成到白色背景再输出实色,
    可在 EMF 中视觉等效原半透明效果。
    """
    h = hex_color.lstrip("#")
    if len(h) == 3:  # 短形式 #RGB
        h = "".join(c * 2 for c in h)
    r = int(h[0:2], 16)
    g = int(h[2:4], 16)
    b = int(h[4:6], 16)
    r2 = round(r * alpha + 255 * (1 - alpha))
    g2 = round(g * alpha + 255 * (1 - alpha))
    b2 = round(b * alpha + 255 * (1 - alpha))
    return f"#{r2:02x}{g2:02x}{b2:02x}"


def flatten_svg_opacity(svg_text: str) -> str:
    """把 SVG 里的 opacity / fill-opacity / stroke-opacity 预合成到白色背景。

    覆盖两种 SVG 输出风格:
    1. matplotlib 风格 `style="opacity:0.X; fill:#RRGGBB; stroke:#RRGGBB"`
       (inline CSS, 整体 opacity 影响 fill+stroke)
    2. PyMuPDF / PDF-to-SVG 风格 `fill="#RRGGBB" fill-opacity="0.X"` (SVG 属性)

    EMF 中保留预合成后实色, 视觉等效原半透明效果 (背景白)。
    """
    # --- 风格 1: matplotlib inline CSS style ---
    def fix_inline_style(m: re.Match) -> str:
        style = m.group(1)
        op_match = re.search(r"opacity:\s*([0-9.]+)\s*;?", style)
        if not op_match:
            return m.group(0)
        alpha = float(op_match.group(1))
        if alpha >= 0.99:
            return m.group(0)
        if alpha <= 0.01:
            # 完全透明: 删掉 fill/stroke 保持真透明 (不要预合成为白色实色)
            style = re.sub(r"\b(?:fill|stroke):\s*#[0-9a-fA-F]{3,6}\s*;?\s*",
                           "", style)
            style = re.sub(r"opacity:\s*[0-9.]+\s*;?\s*", "", style)
            return f'style="{style.strip()}"'
        # 替换 style 内 fill/stroke 颜色为预合成色
        def color_sub(cm: re.Match) -> str:
            prop = cm.group(1)
            color = cm.group(2)
            return f"{prop}: {_blend_hex_to_white(color, alpha)}"
        style = re.sub(r"\b(fill|stroke):\s*(#[0-9a-fA-F]{3,6})\b",
                       color_sub, style)
        # 删除 opacity 声明
        style = re.sub(r"opacity:\s*[0-9.]+\s*;?\s*", "", style)
        return f'style="{style.strip()}"'

    svg_text = re.sub(r'style="([^"]+)"', fix_inline_style, svg_text)

    # --- 风格 2: SVG 属性 fill-opacity / stroke-opacity ---
    # 处理 <... fill="#RRGGBB" ... fill-opacity="0.X" ...>
    # 与 stroke 同理。注意属性顺序不固定, 用先找 *-opacity 再回溯定位 *.
    def fix_attr_opacity(m: re.Match) -> str:
        tag = m.group(0)
        for kind in ("fill", "stroke"):
            op_m = re.search(rf'{kind}-opacity="([0-9.]+)"', tag)
            if not op_m:
                continue
            alpha = float(op_m.group(1))
            if alpha >= 0.99:
                # 删 alpha=1 的多余声明
                tag = re.sub(rf'\s*{kind}-opacity="[0-9.]+"', "", tag)
                continue
            if alpha <= 0.01:
                # 完全透明: 改为 none 保持真透明 (不要预合成为白色实色)
                tag = re.sub(rf'{kind}="#[0-9a-fA-F]{{3,6}}"',
                             f'{kind}="none"', tag, count=1)
                tag = re.sub(rf'\s*{kind}-opacity="[0-9.]+"', "", tag)
                continue
            color_m = re.search(rf'{kind}="(#[0-9a-fA-F]{{3,6}})"', tag)
            if color_m:
                new_color = _blend_hex_to_white(color_m.group(1), alpha)
                tag = re.sub(rf'{kind}="#[0-9a-fA-F]{{3,6}}"',
                             f'{kind}="{new_color}"', tag, count=1)
            # 删 opacity 属性
            tag = re.sub(rf'\s*{kind}-opacity="[0-9.]+"', "", tag)
        return tag

    # 只对 <path .../> / <rect .../> / <circle ../> 这类自闭合 / 单标签处理,
    # 避免吞掉嵌套结构
    svg_text = re.sub(
        r"<(?:path|rect|circle|ellipse|polygon|polyline|line)\b[^>]*/?>",
        fix_attr_opacity, svg_text,
    )
    return svg_text


def svg_to_emf(svg_path: Path, emf_path: Path, inkscape: str,
               flatten_alpha: bool = True) -> bool:
    """Inkscape 把 SVG 转为 EMF。

    走 SVG 中转而非 PDF 直转, 因为 Inkscape 1.4 在 Windows 上解析复杂
    TikZ-PDF 时会触发 ImageMagick Access Violation, 而 SVG 解析路径稳定。

    flatten_alpha=True 时先把 SVG 里的 opacity / fill-opacity / stroke-opacity
    预合成到白色背景, 避免 EMF (不支持 alpha) 把淡色填充渲染成饱和实色。
    """
    if not svg_path.exists():
        print(f"[WARN] EMF 转换跳过, SVG 未生成 ({svg_path.name})")
        return False
    # 预合成 alpha 到临时 SVG (不动原 SVG 输出)
    src_svg = svg_path
    tmp_svg: Path | None = None
    if flatten_alpha:
        try:
            text = svg_path.read_text(encoding="utf-8")
            new_text = flatten_svg_opacity(text)
            if new_text != text:
                fd, tmp_name = tempfile.mkstemp(suffix=".svg",
                                                 prefix=f"{svg_path.stem}_flat_")
                os.close(fd)
                tmp_svg = Path(tmp_name)
                tmp_svg.write_text(new_text, encoding="utf-8")
                src_svg = tmp_svg
        except Exception as e:
            print(f"[WARN] SVG opacity 预合成失败, 用原 SVG: {e}")
    try:
        emf_path.parent.mkdir(parents=True, exist_ok=True)
        cmd = [
            inkscape, str(src_svg),
            "--export-type=emf",
            "--export-text-to-path",   # 文本转矢量路径, 避免 EMF 字体回退 (Times → 非衬线)
            f"--export-filename={emf_path}",
        ]
        r = subprocess.run(cmd, capture_output=True, encoding="utf-8",
                           errors="replace", timeout=60)
        if not emf_path.exists():
            print(f"[WARN] Inkscape EMF 输出未生成 ({svg_path.name}); "
                  f"stderr: {(r.stderr or '')[-200:]}")
            return False
        return True
    except subprocess.TimeoutExpired:
        print(f"[WARN] Inkscape EMF 转换超时 ({svg_path.name})")
        return False
    except Exception as e:
        print(f"[WARN] Inkscape EMF 转换失败 ({svg_path.name}): {e}")
        return False
    finally:
        if tmp_svg is not None:
            try:
                tmp_svg.unlink()
            except OSError:
                pass


def cleanup_aux(pdf_dir: Path, stem: str) -> None:
    """清理 xelatex 留下的中间文件（保留 .pdf 不动）。"""
    for ext in (".aux", ".log", ".out", ".fls", ".fdb_latexmk", ".xdv", ".synctex.gz"):
        f = pdf_dir / f"{stem}{ext}"
        if f.exists():
            try:
                f.unlink()
            except OSError:
                pass


# ============================================================================
# .tex 路径: xelatex → PDF (pdf/) → PNG + SVG + EMF
# ============================================================================
def render_tex_one(tex_path: Path, xelatex: str, fitz,
                   inkscape: str | None = None,
                   keep_aux: bool = False) -> bool:
    """编译单个 .tex → pdf/ + png/ + svg/ + emf/ (EMF 需要 Inkscape, 否则跳过)。"""
    stem = tex_path.stem
    here = tex_path.parent
    pdf_dir = here / PDF_DIR_NAME
    pdf_dir.mkdir(exist_ok=True)
    pdf_path = pdf_dir / f"{stem}.pdf"
    png_path = here / PNG_DIR_NAME / f"{stem}.png"
    svg_path = here / SVG_DIR_NAME / f"{stem}.svg"
    emf_path = here / EMF_DIR_NAME / f"{stem}.emf"

    has_emf = inkscape is not None
    total_steps = 4 if has_emf else 3

    print(f"\n=== [TEX] {stem} ===")

    # Step 1: xelatex 编译 → pdf/
    print(f"[1/{total_steps}] xelatex {tex_path.name} → pdf/")
    cmd = [
        xelatex,
        "-interaction=nonstopmode",
        "-halt-on-error",
        "-output-directory", str(pdf_dir),
        str(tex_path),
    ]
    r1 = subprocess.run(cmd, cwd=here, capture_output=True,
                        encoding="utf-8", errors="replace")
    pdf_ok = pdf_path.exists() and "Output written on" in (r1.stdout or "")
    if not pdf_ok:
        print(f"[FAIL] xelatex 编译失败 ({stem}); 末尾日志:")
        tail = (r1.stdout or "").splitlines()[-40:]
        for line in tail:
            print(f"  {line}")
        if r1.stderr:
            print(f"  stderr: {r1.stderr[-500:]}")
        return False

    # Step 2: PDF → PNG (300 DPI)
    print(f"[2/{total_steps}] PNG @ {DPI} DPI → png/{stem}.png")
    if not pdf_to_png(pdf_path, png_path, fitz):
        return False

    # Step 3: PDF → SVG (矢量)
    print(f"[3/{total_steps}] SVG (vector) → svg/{stem}.svg")
    pdf_to_svg(pdf_path, svg_path, fitz)  # SVG 失败不阻塞 (PNG 已生成)

    # Step 4: SVG → EMF (Word 嵌入矢量首选; 仅当 Inkscape 可用时执行)
    emf_ok = False
    if has_emf:
        print(f"[4/{total_steps}] EMF (vector, Word) → emf/{stem}.emf")
        emf_ok = svg_to_emf(svg_path, emf_path, inkscape)

    if not keep_aux:
        cleanup_aux(pdf_dir, stem)

    suffix = "+ .emf" if emf_ok else ""
    print(f"[OK] pdf/{stem}.pdf + png + svg {suffix}".rstrip())
    return True


# ============================================================================
# .py 路径: matplotlib 子进程 → PNG + SVG (脚本输出) → EMF (后处理)
# ============================================================================
def render_py_one(py_path: Path,
                  inkscape: str | None = None) -> bool:
    """运行单个 .py 脚本 (matplotlib) → png/ + svg/ + emf/。

    脚本自行调用 _style.save_figure() 输出 PNG + SVG;
    本函数追加 SVG → EMF 转换 (如果 Inkscape 可用)。
    """
    stem = py_path.stem
    here = py_path.parent
    png_path = here / PNG_DIR_NAME / f"{stem}.png"
    svg_path = here / SVG_DIR_NAME / f"{stem}.svg"
    emf_path = here / EMF_DIR_NAME / f"{stem}.emf"

    has_emf = inkscape is not None
    total_steps = 2 if has_emf else 1

    print(f"\n=== [PY ] {stem} ===")

    # Step 1: 运行 python 子进程 (matplotlib 渲染)
    print(f"[1/{total_steps}] python {py_path.name}")
    r1 = subprocess.run([sys.executable, str(py_path)],
                        cwd=here, capture_output=True,
                        encoding="utf-8", errors="replace")
    if r1.returncode != 0:
        print(f"[FAIL] matplotlib 渲染失败 ({stem}); 末尾日志:")
        tail = (r1.stdout or "").splitlines()[-20:]
        for line in tail:
            print(f"  {line}")
        if r1.stderr:
            print(f"  stderr: {r1.stderr[-500:]}")
        return False
    # 确认脚本输出了 png 和 svg
    if not png_path.exists():
        print(f"[FAIL] {stem} 未输出 png (脚本应调用 _style.save_figure)")
        return False

    # Step 2: SVG → EMF
    emf_ok = False
    if has_emf:
        print(f"[2/{total_steps}] EMF (vector, Word) → emf/{stem}.emf")
        emf_ok = svg_to_emf(svg_path, emf_path, inkscape)

    suffix = "+ .emf" if emf_ok else ""
    print(f"[OK] png + svg {suffix}".rstrip())
    return True


# ============================================================================
# 主流程
# ============================================================================
def collect_candidates(here: Path, filters: list[str],
                       want_tex: bool, want_py: bool) -> tuple[list, list]:
    """收集 .tex / .py 候选, 应用子串过滤; 排除 _*.tex/_*.py 与 render_figures.py。"""
    tex_list, py_list = [], []
    if want_tex:
        tex_list = sorted(list(here.glob("fig*.tex")) + list(here.glob("algo*.tex")))
        tex_list = [s for s in tex_list if not s.name.startswith("_")]
    if want_py:
        py_list = sorted(here.glob("fig*.py"))
        py_list = [s for s in py_list
                   if not s.name.startswith("_") and s.name != "render_figures.py"]
    if filters:
        tex_list = [s for s in tex_list if any(f in s.name for f in filters)]
        py_list  = [s for s in py_list  if any(f in s.name for f in filters)]
    return tex_list, py_list


def main(argv) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("filters", nargs="*",
                        help="子串过滤 (任一匹配即编译); 留空编译全部 fig*/algo*")
    parser.add_argument("--keep-aux", action="store_true",
                        help="保留 xelatex 中间文件 (aux/log/out) 用于调试")
    parser.add_argument("--no-emf", action="store_true",
                        help="跳过 SVG → EMF 转换 (默认: 找到 Inkscape 就执行)")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--tex-only", action="store_true",
                       help="只渲染 .tex (跳过 matplotlib .py)")
    group.add_argument("--py-only", action="store_true",
                       help="只渲染 .py (跳过 .tex)")
    args = parser.parse_args(argv)

    here = Path(__file__).parent
    want_tex = not args.py_only
    want_py  = not args.tex_only
    tex_list, py_list = collect_candidates(here, args.filters, want_tex, want_py)
    total = len(tex_list) + len(py_list)

    if total == 0:
        print("[WARN] 没有匹配到任何源文件")
        return 1

    xelatex  = find_xelatex() if tex_list else None
    fitz     = check_fitz()    if tex_list else None
    inkscape = None if args.no_emf else find_inkscape()

    if xelatex:
        print(f"[INFO] xelatex={xelatex}")
    if fitz:
        print(f"[INFO] PyMuPDF={fitz.__version__}")
    print(f"[INFO] python={sys.executable}")
    if inkscape:
        print(f"[INFO] Inkscape={inkscape} (将生成 EMF)")
    elif args.no_emf:
        print(f"[INFO] 已 --no-emf 跳过 EMF 输出")
    else:
        print(f"[INFO] Inkscape 未找到, EMF 输出跳过 "
              f"(下载: https://inkscape.org/release/)")
    print(f"[INFO] 准备渲染 {total} 张图: "
          f"{len(tex_list)} TikZ + {len(py_list)} matplotlib")

    fail = []
    t0 = time.time()
    for s in tex_list:
        if not render_tex_one(s, xelatex, fitz,
                              inkscape=inkscape, keep_aux=args.keep_aux):
            fail.append(s.stem)
    for s in py_list:
        if not render_py_one(s, inkscape=inkscape):
            fail.append(s.stem)
    dt = time.time() - t0

    print(f"\n[DONE] {total - len(fail)}/{total} 张渲染成功，耗时 {dt:.1f}s")
    if fail:
        print("[FAIL] " + ", ".join(fail))
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
