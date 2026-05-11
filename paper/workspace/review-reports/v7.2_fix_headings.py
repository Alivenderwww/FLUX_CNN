# -*- coding: utf-8 -*-
"""恢复章节标题中数字编号与中文标题之间的单空格"""
import re

PATH = r"c:\_Project\FLUX_CNN\paper\workspace\paper.md"

with open(PATH, "r", encoding="utf-8") as f:
    lines = f.readlines()

changed = 0
# 期望: ## N(.M(.K)*)? 中文标题
# 当前可能: ## N.M中文 -> ## N.M 中文
heading_re = re.compile(r"^(\s*#{1,6}\s+)([\d.]+)([一-鿿])")

for i in range(551, len(lines)):  # 552 起 (0-indexed 551)
    ln = lines[i]
    m = heading_re.match(ln)
    if m:
        new = ln[:m.end(2)] + " " + ln[m.end(2):]
        lines[i] = new
        changed += 1

with open(PATH, "w", encoding="utf-8") as f:
    f.writelines(lines)

print(f"headings fixed: {changed}")
