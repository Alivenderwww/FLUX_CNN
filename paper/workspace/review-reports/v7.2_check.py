# -*- coding: utf-8 -*-
import re

with open(r"c:\_Project\FLUX_CNN\paper\workspace\paper.md", "r", encoding="utf-8") as f:
    lines = f.readlines()

content = "".join(lines[551:])  # 552 起

patterns = {
    "CJK_space_alpha": r"[一-鿿]\s+[A-Za-z]",
    "alpha_space_CJK": r"[A-Za-z]\s+[一-鿿]",
    "CJK_space_digit": r"[一-鿿]\s+[0-9]",
    "digit_space_CJK": r"[0-9]\s+[一-鿿]",
    "formula_space_CJK": r"\$[^$]*\$\s+[一-鿿]",
    "CJK_space_formula": r"[一-鿿]\s+\$[^$]*\$",
    "digit_space_pct": r"[0-9]\s+%",
}
for name, pat in patterns.items():
    matches = re.findall(pat, content)
    print(f"{name}: {len(matches)} matches")
    for m in matches[:15]:
        print(f"  {m!r}")
