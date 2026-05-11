# -*- coding: utf-8 -*-
"""v7.2 中英文之间空格删除（spec §11.3 字面规则）

范围：paper.md 行 552 起到文末。
方向：删除中文与英文/数字之间的空格；保留数字+单位、引导句、表格分隔、注释、代码、公式内、章节标题。
"""
import re
import sys

PATH = r"c:\_Project\FLUX_CNN\paper\workspace\paper.md"

with open(PATH, "r", encoding="utf-8") as f:
    lines = f.readlines()

CJK = r"[一-鿿]"

# 单位白名单：数字+空格+这些保留（视为连续英数块）
UNIT_TOKENS = [
    "MHz", "GHz", "kHz", "Hz",
    "GOPS", "TOPS", "GMAC", "MAC", "GFLOPS", "MFLOPS",
    "ms", "ns", "us", "s",
    "fps", "FPS",
    "GB", "MB", "KB", "kB", "B",
    "Gb", "Mb", "Kb",
    "W", "mW", "kW",
    "V", "mV",
    "nm", "um",
    "word", "words",
    "ops", "MAC/s",
    "MOps",
    "RAMB18", "RAMB36",
    "DSP48E1", "DSP",
    "BRAM36", "BRAM18", "BRAM",
    "LUT", "FF",
    "INT8", "INT4", "INT16", "FP32", "FP16",
]

# 数字+空格+单位：保留模式（构造一个用于 protect 的检测器）
def is_number_space_unit(text, pos_after_number):
    """检测 text[pos_after_number:] 开头是否是单位 token"""
    rest = text[pos_after_number:]
    for u in UNIT_TOKENS:
        # 单位后必须是非字母数字（边界）
        if rest.startswith(u):
            end = len(u)
            if end >= len(rest) or not rest[end].isalnum():
                return True, end
    return False, 0

def protect_segments_v2(line, is_heading=False):
    """保护：HTML 注释、行内代码、LaTeX 公式。标题行保留 #### 前缀但对正文应用规则。"""
    placeholders = []
    def stash(m):
        placeholders.append(m.group(0))
        return f"\x00{len(placeholders)-1}\x00"

    out = line
    # 标题前缀 (## 含尾空格) 整体保护
    if is_heading:
        m = re.match(r"^(\s*#{1,6}\s+)(.*)$", out, flags=re.DOTALL)
        if m:
            placeholders.append(m.group(1))
            out = f"\x00{len(placeholders)-1}\x00" + m.group(2)

    out = re.sub(r"<!--.*?-->", stash, out, flags=re.DOTALL)
    out = re.sub(r"`[^`]+`", stash, out)
    out = re.sub(r"\$[^$]+\$", stash, out)
    return out, placeholders

def restore_segments(line, placeholders):
    def fill(m):
        return placeholders[int(m.group(1))]
    return re.sub(r"\x00(\d+)\x00", fill, line)

# 引导句保留模式（表 X.Y / 图 X.Y / 公式 (X) / 第 N 章 等）
GUIDE_PATTERNS = [
    re.compile(r"(表)\s+(\d)"),
    re.compile(r"(图)\s+(\d)"),
    re.compile(r"(公式)\s+(\()"),
    re.compile(r"(第)\s+(\d)"),
    re.compile(r"(章)\s+(\d)"),
    re.compile(r"(节)\s+(\d)"),
]
def _guide_repl(m):
    return m.group(1) + "\x01" + m.group(2)
RESTORE_GUIDE = re.compile(r"\x01")

def process(line):
    # 表格分隔行不动
    if re.match(r"^\s*\|.*---", line):
        return line
    # 标题行: 保留 #### 前缀, 对内容应用同样规则
    is_heading = bool(re.match(r"^\s*#{1,6}\s", line))

    out, ph = protect_segments_v2(line, is_heading)

    # 保护引导句中的空格（表 X / 图 X / 公式 (X) / 第 N）
    for pat in GUIDE_PATTERNS:
        out = pat.sub(_guide_repl, out)

    # === 处理数字+空格+单位：先转一个特殊标记保留 ===
    # 数字+空格+UNIT
    def protect_num_unit(m):
        return m.group(1) + "\x02" + m.group(2)
    unit_re = re.compile(
        r"(\d)\s+(" + "|".join(re.escape(u) for u in UNIT_TOKENS) + r")(?![A-Za-z0-9])"
    )
    out = unit_re.sub(protect_num_unit, out)

    # === 真正的删空格 ===
    # 中→英字母
    out = re.sub(r"([一-鿿])\s+([A-Za-z])", r"\1\2", out)
    # 英字母→中
    out = re.sub(r"([A-Za-z])\s+([一-鿿])", r"\1\2", out)
    # 中→数字
    out = re.sub(r"([一-鿿])\s+(\d)", r"\1\2", out)
    # 数字→中
    out = re.sub(r"(\d)\s+([一-鿿])", r"\1\2", out)
    # 数字+% → 数字%
    out = re.sub(r"(\d)\s+(%)", r"\1\2", out)
    # 中→公式占位符（占位符以 \x00 开头）
    out = re.sub(r"([一-鿿])\s+(\x00\d+\x00)", r"\1\2", out)
    # 公式占位符→中
    out = re.sub(r"(\x00\d+\x00)\s+([一-鿿])", r"\1\2", out)
    # 公式占位符→数字
    out = re.sub(r"(\x00\d+\x00)\s+(\d)", r"\1\2", out)
    # 数字→公式占位符
    out = re.sub(r"(\d)\s+(\x00\d+\x00)", r"\1\2", out)
    # 公式占位符→英字母
    out = re.sub(r"(\x00\d+\x00)\s+([A-Za-z])", r"\1\2", out)
    # 英字母→公式占位符
    out = re.sub(r"([A-Za-z])\s+(\x00\d+\x00)", r"\1\2", out)

    # 还原数字+单位
    out = out.replace("\x02", " ")
    # 还原引导句空格
    out = RESTORE_GUIDE.sub(" ", out)

    # 还原保护段
    out = restore_segments(out, ph)
    return out

start = 552  # 1-based
end = len(lines)  # 包括末尾

changed = 0
for i in range(start - 1, end):
    new = process(lines[i])
    if new != lines[i]:
        changed += 1
        lines[i] = new

with open(PATH, "w", encoding="utf-8") as f:
    f.writelines(lines)

print(f"changed lines: {changed} (range {start}-{end})")
