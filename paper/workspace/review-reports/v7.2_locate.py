# -*- coding: utf-8 -*-
import re

with open(r"c:\_Project\FLUX_CNN\paper\workspace\paper.md", "r", encoding="utf-8") as f:
    lines = f.readlines()

print("== formula-CJK / CJK-formula residuals ==")
for i, ln in enumerate(lines[551:], start=552):
    if re.search(r"\$[^$]*\$\s+[一-鿿]", ln) or re.search(r"[一-鿿]\s+\$[^$]*\$", ln):
        print(f"{i}: {ln.rstrip()}")

print("\n== alpha-CJK / CJK-alpha residuals (multi-line newline excluded) ==")
for i, ln in enumerate(lines[551:], start=552):
    # 单行内的中英空格残留
    m = re.search(r"([一-鿿])([ \t]+)([A-Za-z])", ln)
    if m:
        print(f"{i}: {ln.rstrip()}")
        continue
    m = re.search(r"([A-Za-z])([ \t]+)([一-鿿])", ln)
    if m:
        print(f"{i}: {ln.rstrip()}")
