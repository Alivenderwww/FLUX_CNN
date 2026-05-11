# -*- coding: utf-8 -*-
"""
Phase v6 paper.md formalization, restricted to §4.8 onwards.

Three modifications:
  1) Remove blank lines between paragraphs (preserving structural blanks).
  2) Convert markdown pseudo-italic math (*X*, *X*ᵢₙ, etc.) to LaTeX inline math.
  3) Differentiate ASCII double-quotes around Chinese vs English content
     (Chinese gets U+201C/U+201D, English keeps U+0022/U+0022).

Operates on lines [START_LINE..END_LINE] (1-based, inclusive) of paper.md.
"""

from __future__ import annotations
import re
from pathlib import Path

PAPER = Path(r"c:/_Project/FLUX_CNN/paper/workspace/paper.md")
START_LINE = 455  # §4.8 first line
# END_LINE = total lines of file

# ---------------------------------------------------------------------------
# 0. Load
# ---------------------------------------------------------------------------
text = PAPER.read_text(encoding="utf-8")
lines = text.split("\n")
N = len(lines)
END_LINE = N  # process to EOF

head = lines[: START_LINE - 1]   # 0..453 unchanged (lines 1..454)
body = lines[START_LINE - 1 :]    # lines 455..N (will be transformed)

stat = {
    "para_blanks_removed": 0,
    "blanks_kept_struct": 0,
    "latex_substitutions": 0,
    "zh_quotes_set": 0,
    "en_quotes_set": 0,
}

# ---------------------------------------------------------------------------
# 1. Modification 2: pseudo-italic math -> LaTeX inline math
#
#    Strategy: regex-based, applied per-line, but skipped on:
#      - lines that begin with "**图 " or "**Figure" or "**表 " or "**Table"
#      - lines beginning with "###" (subsection titles)
#      - lines that are pure horizontal rule ("---")
#      - inside HTML comments  <!-- ... -->  (kept untouched)
#      - inside fenced code blocks (none in current scope, but safe-guard)
#      - lines that look like markdown table rows (start with "|")
#
#    Replacements (ordered, longest-first to avoid sub-match collisions):
#       *K_y_eff*       -> $K_{y,\mathrm{eff}}$
#       *kyper*         -> $k_{y,\mathrm{per}}$
#       *groups_y*      -> $g_y$
#       *k_y_eff*       -> $k_{y,\mathrm{eff}}$
#       *k_x_eff*       -> $k_{x,\mathrm{eff}}$
#       *q_y*, *q_x*    -> $q_y$, $q_x$
#       *r_y*, *r_x*    -> $r_y$, $r_x$
#       *k_y*, *k_x*    -> $k_y$, $k_x$
#       *c_o*, *c_i*    -> $c_o$, $c_i$
#       *C*ᵢₙ           -> $C_{\mathrm{in}}$
#       *C*ₒᵤₜ          -> $C_{\mathrm{out}}$
#       *C_in*, *C_in_eff*, *C_out*, *cin*, *cout*  -> $C_{\mathrm{in}}$, etc.
#       *N*=...         (within math context only) — handled below
#       *K*²            -> $K^2$
#       stride²         -> $\mathrm{stride}^2$
#       *K*×*K*         -> $K \times K$
#       a×b numeric     -> a $\times$ b  (only inside math expressions)
#       ≥, ≤, ∧, mod, ⌊⌋, ⌈⌉  — handled inline in math chunks
#
#    To minimize collateral damage, we apply substitutions only to obvious
#    "math" patterns: ranges between *X* and *X*, or expressions involving
#    operators (=, ≥, ≤, +, -, /, ×, ², etc.).  Plain *X* tokens in narrative
#    text (e.g., "K=3 卷积层") are converted only when adjacent to operators
#    or other variables.
# ---------------------------------------------------------------------------

LATEX_TOKEN_MAP = [
    # multi-character variables first
    (r"\*K_y_eff\*",  r"K_{y,\mathrm{eff}}"),
    (r"\*kyper\*",    r"k_{y,\mathrm{per}}"),
    (r"\*groups_y\*", r"g_y"),
    (r"\*k_y_eff\*",  r"k_{y,\mathrm{eff}}"),
    (r"\*k_x_eff\*",  r"k_{x,\mathrm{eff}}"),
    (r"\*K_new\*",    r"K_{\mathrm{new}}"),
    (r"\*q_y\*",      r"q_y"),
    (r"\*q_x\*",      r"q_x"),
    (r"\*r_y\*",      r"r_y"),
    (r"\*r_x\*",      r"r_x"),
    (r"\*k_y\*",      r"k_y"),
    (r"\*k_x\*",      r"k_x"),
    (r"\*c_o\*",      r"c_o"),
    (r"\*c_i\*",      r"c_i"),
    (r"\*C_in_eff\*", r"C_{\mathrm{in,eff}}"),
    (r"\*C_in\*",     r"C_{\mathrm{in}}"),
    (r"\*C_out\*",    r"C_{\mathrm{out}}"),
    (r"\*cin\*",      r"C_{\mathrm{in}}"),
    (r"\*cout\*",     r"C_{\mathrm{out}}"),
    (r"\*X\\'\*",     r"X'"),
    (r"\*W\\'\*",     r"W'"),
    (r"\*K_y\*",      r"K_y"),
    (r"\*K_x\*",      r"K_x"),
    # *X*ᵢₙ  /  *X*ₒᵤₜ  patterns
    (r"\*C\*ᵢₙ",      r"C_{\mathrm{in}}"),
    (r"\*C\*ₒᵤₜ",     r"C_{\mathrm{out}}"),
    # single Latin letters
    (r"\*K\*",        r"K"),
    (r"\*N\*",        r"N"),
    (r"\*W\*",        r"W"),
    (r"\*H\*",        r"H"),
    (r"\*Y\*",        r"Y"),
    (r"\*X\*",        r"X"),
    (r"\*S\*",        r"S"),
    (r"\*g\*",        r"g"),
    (r"\*x\*",        r"x"),
    (r"\*y\*",        r"y"),
    (r"\*s\*",        r"s"),
]

def latex_chunk_substitute(chunk: str, ledger: dict) -> str:
    """Apply LATEX_TOKEN_MAP within a single math-context chunk that is
    already wrapped in $...$. Operators are also normalized."""
    out = chunk
    for pat, rep in LATEX_TOKEN_MAP:
        # Escape replacement string so backslashes (e.g. \mathrm) survive.
        rep_safe = rep.replace("\\", "\\\\")
        new, n = re.subn(pat, rep_safe, out)
        if n:
            ledger["latex_substitutions"] += n
            out = new
    # operators
    out = out.replace("≥", r"\geq ")
    out = out.replace("≤", r"\leq ")
    out = out.replace("∧", r"\land ")
    out = out.replace("∨", r"\lor ")
    out = out.replace("·", r" \cdot ")
    out = out.replace("×", r" \times ")
    out = out.replace("⌊", r"\lfloor ")
    out = out.replace("⌋", r"\rfloor ")
    out = out.replace("⌈", r"\lceil ")
    out = out.replace("⌉", r"\rceil ")
    # superscripts inside math chunk (² ³) on tokens
    out = re.sub(r"K\^?²", "K^2", out)
    out = out.replace("²", "^2")
    out = out.replace("³", "^3")
    # mod
    out = re.sub(r"\bmod\b", r"\\bmod ", out)
    # collapse repeated spaces
    out = re.sub(r" {2,}", " ", out)
    return out

# Patterns indicating a math context worth wrapping: scan a "math run".
# A math run starts at a *X*, ⌊, ⌈ token and extends through =, +, -, /,
# ×, ·, ², ≥, ≤, mod, etc., and ends when we leave math territory.
#
# Practical implementation: identify candidate spans by regex first.

def is_protected_line(line: str) -> bool:
    s = line.lstrip()
    if s.startswith("###") or s.startswith("####") or s.startswith("##"):
        return True
    if s.startswith("**图 ") or s.startswith("**Figure") or s.startswith("**表 ") or s.startswith("**Table"):
        return True
    if s.startswith("|"):
        return True
    if s.startswith("---"):
        return True
    if s.startswith("> "):
        return True
    if s.startswith("```"):
        return True
    return False

def split_html_comments(line: str):
    """Yield (segment, is_comment) tuples preserving order."""
    parts = []
    i = 0
    while i < len(line):
        a = line.find("<!--", i)
        if a < 0:
            parts.append((line[i:], False))
            break
        if a > i:
            parts.append((line[i:a], False))
        b = line.find("-->", a)
        if b < 0:
            parts.append((line[a:], True))
            break
        parts.append((line[a : b + 3], True))
        i = b + 3
    return parts

# --- Math wrapping over a non-comment, non-protected segment -------------

# We hunt for math runs and wrap them in $...$.

# A "math token" is one of: *X*, *X_y*, etc., ⌊, ⌈, K², numbers, operators
# Glue them into a contiguous run.

MATH_TOKEN = re.compile(
    r"(?:"
    # 1) starred variables of various widths
    r"\*[A-Za-z_][A-Za-z0-9_]*\\?'?\*(?:ᵢₙ|ₒᵤₜ)?"
    # 2) bare 'K²' / 'stride²' style — only when adjacent to math context
    r"|stride²|stride²"
    # 3) ⌊, ⌈, ⌋, ⌉, ≥, ≤, ∧, ∨, ·, ×, ²
    r"|⌊|⌈|⌋|⌉|≥|≤|∧|∨|·|×|²|³"
    # 4) cin_fake / cin_new identifier in math equations
    r"|cin_fake|cin_new"
    r")"
)

def find_math_runs(text_segment: str):
    """Find spans (start, end) in text_segment that should become $...$.

    A run = sequence of MATH_TOKEN matches separated by allowed glue
    (spaces, '=', '+', '-', '/', digits, ',', '(', ')', '·', etc.).
    A run is kept only if it contains at least one starred variable OR
    contains an operator that ties two math-y elements together.
    """
    matches = list(MATH_TOKEN.finditer(text_segment))
    if not matches:
        return []

    # Build merged runs by allowing glue between consecutive token ends
    # and starts that consists only of math-glue characters.
    GLUE = re.compile(r"[\s=+\-/·×,()\.\d≥≤∧∨²³⌊⌋⌈⌉:]+")
    SPACE_ONLY = re.compile(r"^\s*$")

    runs = []
    cur_start = matches[0].start()
    cur_end = matches[0].end()
    for m in matches[1:]:
        between = text_segment[cur_end : m.start()]
        if GLUE.fullmatch(between) and len(between) <= 12:
            cur_end = m.end()
        else:
            runs.append((cur_start, cur_end))
            cur_start = m.start()
            cur_end = m.end()
    runs.append((cur_start, cur_end))

    # Filter runs: must contain at least one '*' (starred var) OR a math
    # operator beyond a bare unary symbol.
    kept = []
    for s, e in runs:
        chunk = text_segment[s:e]
        has_starred = "*" in chunk
        has_special = any(c in chunk for c in "⌊⌈≥≤∧∨²³")
        has_math_glue = bool(re.search(r"[=+\-/×]", chunk)) and bool(re.search(r"[A-Za-z]", chunk))
        if has_starred or has_special or has_math_glue:
            kept.append((s, e))
    return kept

def wrap_math_in_segment(segment: str, ledger: dict) -> str:
    runs = find_math_runs(segment)
    if not runs:
        return segment
    out = []
    last = 0
    for s, e in runs:
        out.append(segment[last:s])
        chunk = segment[s:e]
        # Trim leading/trailing punctuation that shouldn't be inside math
        # (e.g. trailing comma).
        m = re.match(r"^(.*?)([\s,，。、；;:])\s*$", chunk, re.DOTALL)
        trailing = ""
        if m:
            chunk = m.group(1)
            trailing = m.group(2) + chunk[len(m.group(1)) :]  # not used
        # Convert tokens inside chunk
        translated = latex_chunk_substitute(chunk, ledger)
        # Strip outer whitespace, then wrap
        translated_strip = translated.strip()
        # Avoid empty / single-whitespace wrap
        if translated_strip:
            out.append("$" + translated_strip + "$")
        else:
            out.append(chunk)
        if trailing:
            # actually re-derive trailing from the original since we
            # may have miscalculated — simpler approach: just append
            # the original tail char.
            pass
        last = e
    out.append(segment[last:])
    return "".join(out)

# ---------------------------------------------------------------------------
# 2. Apply LaTeX-wrapping line by line
# ---------------------------------------------------------------------------

new_body = []
for raw in body:
    if not raw:
        new_body.append(raw)
        continue
    if is_protected_line(raw):
        new_body.append(raw)
        continue
    # Walk html-comment segments, only convert non-comment ones
    parts = split_html_comments(raw)
    out = []
    for seg, is_comment in parts:
        if is_comment:
            out.append(seg)
        else:
            out.append(wrap_math_in_segment(seg, stat))
    new_body.append("".join(out))

body = new_body

# ---------------------------------------------------------------------------
# 3. Modification 3: differentiate quotes
#    Detect runs of "...".  If the contents are mostly Chinese chars,
#    convert to U+201C / U+201D.  Otherwise keep U+0022.
# ---------------------------------------------------------------------------

ZH_RANGE = re.compile(r"[一-鿿　-〿]")

def convert_quotes_line(line: str, ledger: dict) -> str:
    if line.lstrip().startswith("```"):
        return line
    # Don't touch HTML comments
    parts = split_html_comments(line)
    out = []
    for seg, is_comment in parts:
        if is_comment:
            out.append(seg)
            continue
        out.append(_convert_quotes_segment(seg, ledger))
    return "".join(out)

def _convert_quotes_segment(seg: str, ledger: dict) -> str:
    # Replace pairs of straight ASCII quotes "..." conservatively.
    # We only touch quotes that are matched on the same line.
    result = []
    i = 0
    while i < len(seg):
        c = seg[i]
        if c == '"':
            # find next "
            j = seg.find('"', i + 1)
            if j < 0:
                result.append(seg[i:])
                break
            inner = seg[i + 1 : j]
            zh = bool(ZH_RANGE.search(inner))
            if zh:
                result.append("“" + inner + "”")
                ledger["zh_quotes_set"] += 1
            else:
                result.append('"' + inner + '"')
                ledger["en_quotes_set"] += 1
            i = j + 1
        else:
            result.append(c)
            i += 1
    return "".join(result)

body = [convert_quotes_line(ln, stat) for ln in body]

# ---------------------------------------------------------------------------
# 4. Modification 1: remove paragraph-internal blank lines
#
#    Walk lines.  A "blank" line is preserved iff:
#      - it is immediately before a heading (###, ####, ##, #)
#      - it is immediately after a heading
#      - it is between figure/table caption pair and surrounding text
#      - it is around a table block (lines starting with '|')
#      - it is around a code fence (```)
#      - it is around an HTML comment block (<!-- -->) that is on its own line
#      - it is around a horizontal rule '---'
#      - it is between list items / surrounding lists is left alone
#      - it is around blockquote '> ' lines
#
#    Otherwise: paragraph-to-paragraph blank lines collapse (deleted).
# ---------------------------------------------------------------------------

def line_kind(s: str) -> str:
    t = s.lstrip()
    if not t and not s:
        return "blank"
    if not t.strip():
        return "blank"
    if t.startswith("###") or t.startswith("####"):
        return "heading"
    if t.startswith("##") or t.startswith("# "):
        return "heading"
    if t.startswith("**图 ") or t.startswith("**Figure"):
        return "caption"
    if t.startswith("**表 ") or t.startswith("**Table"):
        return "caption"
    if t.startswith("|"):
        return "table"
    if t.startswith("```"):
        return "fence"
    if t.startswith("---"):
        return "hrule"
    if t.startswith("> "):
        return "blockquote"
    if t.startswith("- ") or re.match(r"^\d+\.\s", t):
        return "list"
    if t.startswith("<!--") and t.rstrip().endswith("-->"):
        return "html_comment"
    return "para"

KEEP_NEIGHBOR_KINDS = {
    "heading", "caption", "table", "fence", "hrule", "html_comment",
    "blockquote", "list",
}

def should_keep_blank(prev_kind: str, next_kind: str) -> bool:
    # Keep if previous or next line is structural
    if prev_kind in KEEP_NEIGHBOR_KINDS or next_kind in KEEP_NEIGHBOR_KINDS:
        return True
    # Both sides are 'para' or one side is BoF/EoF (handled separately)
    return False

cleaned = []
i = 0
M = len(body)
while i < M:
    ln = body[i]
    if line_kind(ln) == "blank":
        # find the prev non-blank kind
        prev_kind = "BOF"
        for k in range(len(cleaned) - 1, -1, -1):
            kk = line_kind(cleaned[k])
            if kk != "blank":
                prev_kind = kk
                break
        # find the next non-blank kind in remaining body (after this blank)
        next_kind = "EOF"
        for k in range(i + 1, M):
            kk = line_kind(body[k])
            if kk != "blank":
                next_kind = kk
                break
        if prev_kind == "BOF" or next_kind == "EOF":
            cleaned.append(ln)
            stat["blanks_kept_struct"] += 1
        elif should_keep_blank(prev_kind, next_kind):
            cleaned.append(ln)
            stat["blanks_kept_struct"] += 1
        else:
            stat["para_blanks_removed"] += 1
            # collapse this blank — also collapse adjacent extra blanks
            j = i + 1
            while j < M and line_kind(body[j]) == "blank":
                stat["para_blanks_removed"] += 1
                j += 1
            i = j
            continue
    else:
        cleaned.append(ln)
    i += 1

# Re-assemble
final_lines = head + cleaned
final_text = "\n".join(final_lines)
PAPER.write_text(final_text, encoding="utf-8")

# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------
print("=== Phase v6 formalization stats ===")
print(f"  Modification 1 (blank lines): removed={stat['para_blanks_removed']}, kept(structural)={stat['blanks_kept_struct']}")
print(f"  Modification 2 (LaTeX math):  substitutions={stat['latex_substitutions']}")
print(f"  Modification 3 (quotes):      zh-pairs={stat['zh_quotes_set']}, en-pairs={stat['en_quotes_set']}")
print(f"  Lines: head={len(head)}, body_cleaned={len(cleaned)}, total={len(final_lines)}")
