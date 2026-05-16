"""
一键渲染：依次执行本目录下所有 fig*.py。
PNG 输出到 png/，SVG 输出到 svg/。

用法：
  python render_all.py            # 渲染全部
  python render_all.py fig5-7     # 只渲染文件名含 fig5-7 的脚本
  python render_all.py 4 7 11     # 任意子串过滤，可叠加
"""

import runpy
import sys
import time
from pathlib import Path


def main(argv):
    here = Path(__file__).parent
    scripts = sorted(here.glob("fig*.py"))
    if argv:
        scripts = [s for s in scripts if any(a in s.name for a in argv)]

    if not scripts:
        print("[WARN] 没有匹配到任何 fig*.py")
        return 1

    print(f"[INFO] 准备渲染 {len(scripts)} 张图")
    fail = []
    t0 = time.time()
    for s in scripts:
        print(f"\n=== {s.name} ===")
        try:
            runpy.run_path(str(s), run_name="__main__")
        except Exception as e:
            print(f"[FAIL] {s.name}: {e}")
            fail.append(s.name)
    dt = time.time() - t0

    print(f"\n[DONE] {len(scripts) - len(fail)}/{len(scripts)} 张成功，耗时 {dt:.1f}s")
    if fail:
        print("[FAIL] " + ", ".join(fail))
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
