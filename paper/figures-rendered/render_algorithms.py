"""
一键编译本目录下所有 algo*.tex（XeLaTeX / MiKTeX）→ 转 PNG（PyMuPDF）。

依赖（一次性）：
  - MiKTeX (Windows): 提供 xelatex.exe，默认装到
      C:\\Program Files\\MiKTeX\\miktex\\bin\\x64\\
      首次编译会按需自动下载 xeCJK / algorithm2e 等包。
  - PyMuPDF (fitz): 项目 toolchain/.venv 已装。

用法（用项目 toolchain venv 跑）：
  C:\\_Project\\FLUX_CNN\\toolchain\\.venv\\Scripts\\python.exe render_algorithms.py
  ... algo3-1     # 只编译文件名含 algo3-1 的源
  ... 4-1 4-2    # 任意子串过滤

输出：
  algo3-1-fsm.png       算法 3.1 核内 6 级嵌套自循环 FSM
  algo4-1-kyfold.png    算法 4.1 Y 维折叠编译器变换
  algo4-2-s2d.png       算法 4.2 空间到深度重排
  （每张 300 DPI 单页 PNG）
"""

import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

DPI = 300

# MiKTeX 常见安装路径（按可能性排序）
MIKTEX_CANDIDATES = [
    r"C:\Program Files\MiKTeX\miktex\bin\x64",
    r"C:\Program Files (x86)\MiKTeX\miktex\bin\x64",
    os.path.expandvars(r"%LOCALAPPDATA%\Programs\MiKTeX\miktex\bin\x64"),
    os.path.expandvars(r"%LOCALAPPDATA%\Programs\MiKTeX 2.9\miktex\bin\x64"),
]


def find_xelatex() -> str:
    """先查 PATH，再扫 MiKTeX 常见路径。"""
    p = shutil.which("xelatex")
    if p:
        return p
    for base in MIKTEX_CANDIDATES:
        exe = Path(base) / "xelatex.exe"
        if exe.exists():
            return str(exe)
    print("[ERROR] xelatex 未找到。", file=sys.stderr)
    print("  请确认 MiKTeX 已装到 C:\\Program Files\\MiKTeX\\ 或 %LOCALAPPDATA%\\Programs\\MiKTeX\\",
          file=sys.stderr)
    print("  或重启 shell 让 PATH 生效。", file=sys.stderr)
    sys.exit(1)


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
        pix.save(str(png_path))
        doc.close()
        return True
    except Exception as e:
        print(f"[FAIL] PyMuPDF 转 PNG 失败 ({pdf_path.name}): {e}")
        return False


def cleanup_aux(here: Path, stem: str) -> None:
    """清理 xelatex 留下的中间文件（保留 .pdf 和 .png）。"""
    for ext in (".aux", ".log", ".out", ".fls", ".fdb_latexmk", ".xdv", ".synctex.gz"):
        f = here / f"{stem}{ext}"
        if f.exists():
            try:
                f.unlink()
            except OSError:
                pass


def render_one(tex_path: Path, xelatex: str, fitz) -> bool:
    """xelatex 编译 + PyMuPDF 转 PNG。"""
    stem = tex_path.stem
    here = tex_path.parent
    pdf_path = here / f"{stem}.pdf"
    png_path = here / f"{stem}.png"

    print(f"\n=== {stem} ===")

    # Step 1: xelatex 编译 → PDF
    print(f"[1/2] xelatex {tex_path.name}")
    cmd = [
        xelatex,
        "-interaction=nonstopmode",
        "-halt-on-error",
        "-output-directory", str(here),
        str(tex_path),
    ]
    # encoding='utf-8' errors='replace' 避免 Windows GBK 解码报错
    # 注: xelatex 在 MiKTeX 全局安装下会输出 "no MiKTeX administrator has
    # checked for updates" 警告并以 returncode != 0 退出，但 PDF 实际生成成功。
    # 故按 PDF 是否存在 + 是否含 "Output written" 判定成功，而非依赖 returncode。
    r1 = subprocess.run(cmd, cwd=here, capture_output=True,
                        encoding="utf-8", errors="replace")
    pdf_ok = pdf_path.exists() and "Output written on" in (r1.stdout or "")
    if not pdf_ok:
        print(f"[FAIL] xelatex 编译失败 ({stem}); 末尾日志:")
        tail = (r1.stdout or "").splitlines()[-40:]
        for line in tail:
            print(f"  {line}")
        if r1.stderr:
            print(f"  stderr: {r1.stderr[-1000:]}")
        return False

    # Step 2: PyMuPDF → PNG
    print(f"[2/2] fitz render {pdf_path.name} → {png_path.name} @ {DPI} DPI")
    ok = pdf_to_png(pdf_path, png_path, fitz)
    cleanup_aux(here, stem)
    if not ok or not png_path.exists():
        return False
    print(f"[OK] {png_path.name}")
    return True


def main(argv) -> int:
    here = Path(__file__).parent
    sources = sorted(here.glob("algo*.tex"))
    if argv:
        sources = [s for s in sources if any(a in s.name for a in argv)]

    if not sources:
        print("[WARN] 没有匹配到任何 algo*.tex")
        return 1

    xelatex = find_xelatex()
    fitz    = check_fitz()

    print(f"[INFO] 工具就绪: xelatex={xelatex}")
    print(f"[INFO]            PyMuPDF={fitz.__version__}")
    print(f"[INFO] 准备编译 {len(sources)} 段算法图: "
          + ", ".join(s.stem for s in sources))
    print("[NOTE] 首次编译 MiKTeX 会按需自动下载 xeCJK / algorithm2e 包，可能耗时数分钟。")

    fail = []
    t0 = time.time()
    for s in sources:
        if not render_one(s, xelatex, fitz):
            fail.append(s.stem)
    dt = time.time() - t0

    print(f"\n[DONE] {len(sources) - len(fail)}/{len(sources)} 段算法图渲染成功，"
          f"耗时 {dt:.1f}s")
    if fail:
        print("[FAIL] " + ", ".join(fail))
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
