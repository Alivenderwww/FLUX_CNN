# -*- coding: utf-8 -*-
"""二次清理 paper.md 中 pandoc 残留的转义符号."""
import re
from pathlib import Path

p = Path('paper.md')
t = p.read_text(encoding='utf-8')

# 表格 cell 中的 \[...\] -> [...]
t = t.replace(r'\[', '[').replace(r'\]', ']')
# \- (孤立短横) -> -
t = t.replace(r'\-', '-')
# 残留的 *\* / **\**  / 单行 \  (docx 高亮/段落残留)
t = re.sub(r'^\s*\*\\\*\s*$', '', t, flags=re.M)
t = re.sub(r'^\s*\*\*\\\*\*\s*$', '', t, flags=re.M)
t = re.sub(r'^\s*\\\s*$', '', t, flags=re.M)
# *\* 内联残留
t = t.replace(r'*\*', '')

# 多空行压缩
t = re.sub(r'\n{4,}', '\n\n\n', t)

p.write_text(t, encoding='utf-8')
print(f'OK, {len(t.splitlines())} lines')
