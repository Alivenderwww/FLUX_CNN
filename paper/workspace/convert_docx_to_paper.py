# -*- coding: utf-8 -*-
"""
把 paper-from-docx-latest.md (pandoc 输出) 重新格式化为 paper.md.
保留文字内容, 调整章节编号和 markdown 风格.

输入: paper-from-docx-latest.md
输出: paper.md (会覆盖)
"""
import re
from pathlib import Path

SRC = Path(r"C:\_Project\FLUX_CNN\paper\workspace\paper-from-docx-latest.md")
DST = Path(r"C:\_Project\FLUX_CNN\paper\workspace\paper.md")

# ─────────────────────────────────────────────────────────────
# 章节标题映射 (docx pandoc 输出无编号 -> paper.md 带编号)
# ─────────────────────────────────────────────────────────────
H1_MAP = {
    "# 摘要":            "## 摘要",
    "# Abstract":        "## Abstract",
    "# 文献综述":        "## 1 绪论",
    "# 理论基础":        "## 2 理论基础",
    "# 总体方案":        "## 3 总体方案",
    "# 加速器具体实现":  "## 4 加速器具体实现",
    "# 验证与性能分析":  "## 5 验证与性能分析",
    "# 附录B 实验测试用例配置表": "### 附录B 实验测试用例配置表",
}

# h2 -> h3 (1.1, 1.2, etc.) 与 chapter index 关联
# 我们维持 docx 自身的标题顺序, 但加章节号前缀
H2_NUMBERING = {
    # 章 1
    "## 研究背景与意义":            "### 1.1 研究背景与意义",
    "## 国内外相关研究工作进展":    "### 1.2 国内外相关研究工作进展",
    "## 本论文的主要研究内容":      "### 1.3 本论文的主要研究内容",
    "## 论文组织结构":              "### 1.4 论文组织结构",
    # 章 2
    "## 引言":                       None,  # 多处用, 后面单独处理
    "## 卷积神经网络基础":           "### 2.2 卷积神经网络基础",
    "## 数据流分类与硬件复用策略":   "### 2.3 数据流分类与硬件复用策略",
    "## 硬件加速基础技术":           "### 2.4 硬件加速基础技术",
    "## 非一致性内存访问架构":       "### 2.5 非一致性内存访问架构",
    "## 存储层级与访存功耗差异":     "### 2.6 存储层级与访存功耗差异",
    "## 模型量化基础":               "### 2.7 模型量化基础",
    # 章 3
    "## 设计目标与约束":             "### 3.2 设计目标与约束",
    "## 总体架构":                   "### 3.3 总体架构",
    "## 调度方案":                   "### 3.4 调度方案",
    "## 数据流选择":                 "### 3.5 数据流选择",
    # 章 4
    "## 行缓存模块":                 "### 4.2 行缓存模块",
    "## 乘加阵列模块MAC":            "### 4.3 乘加阵列模块 MAC",
    "## 部分和累加模块":             "### 4.4 部分和累加模块",
    "## 后处理模块SDP":              "### 4.5 后处理模块 SDP",
    "## 权重缓存模块":               "### 4.6 权重缓存模块",
    "## 配置寄存器与6层嵌套自循环FSM": "### 4.7 配置寄存器与 6 层嵌套自循环 FSM",
    "## 编译器优化：Y维折叠 Ky-fold": "### 4.8 编译器优化：Y 维折叠 Ky-fold",
    "## 编译器优化：空间到深度 S2D": "### 4.9 编译器优化：空间到深度 S2D",
    "## 子系统DMA与AXI接口集成":     "### 4.10 子系统 DMA 与 AXI 接口集成",
    "## 多核W切片扩展":              "### 4.11 多核 W 切片扩展",
    "## DSP块跨列复用映射方案":      "### 4.12 DSP 块跨列复用映射方案",
    # 章 5
    "## 验证环境与方法":             "### 5.2 验证环境与方法",
    "## 板级实现验证":               "### 5.4 板级实现验证",
    "## 性能分析":                   "### 5.5 性能分析",
    "## 与已有工作对比":             "### 5.6 与已有工作对比",
}

# 本章小结按章节累加  (## 本章小结 出现 4 次, 在 2/3/4/5 章)
BENZHANGXIAOJIE_ORDER = ["### 2.8 本章小结", "### 3.6 本章小结",
                          "### 4.13 本章小结", "### 5.7 本章小结"]

# ## 引言 出现多次 (在 2/3/4/5 章); ## <span class="mark">功能仿真验证</span> 在 5.3
YINYAN_ORDER = ["### 2.1 引言", "### 3.1 引言", "### 4.1 引言", "### 5.1 引言"]

# 子小节 h3 ### -> h4 #### (例如 2.2.1 卷积层)
H3_TO_H4_MAP = {
    "### 卷积层":                                    "#### 2.2.1 卷积层",
    "### 池化层":                                    "#### 2.2.2 池化层",
    "### 全连接层":                                  "#### 2.2.3 全连接层",
    "### 权重静止数据流":                            "#### 2.3.1 权重静止数据流",
    "### 输出静止数据流":                            "#### 2.3.2 输出静止数据流",
    "### 行静止数据流":                              "#### 2.3.3 行静止数据流",
    "### 数据流选择的工程权衡":                      "#### 2.3.4 数据流选择的工程权衡",
    "### 流水线设计":                                "#### 2.4.1 流水线设计",
    "### 行缓存与窗口生成":                          "#### 2.4.2 行缓存与窗口生成",
    "### 握手协议":                                  "#### 2.4.3 握手协议",
    "### 总线协议AXI":                               "#### 2.4.4 总线协议 AXI",
    "### 端侧卷积神经网络硬件加速器架构研究进展":     "#### 1.2.1 端侧卷积神经网络硬件加速器架构研究进展",
    "### 软硬件协同调度与流式计算加速研究进展":       "#### 1.2.2 软硬件协同调度与流式计算加速研究进展",
    "### Shortcut Bank残差通路":                     "#### 4.5.1 Shortcut Bank 残差通路",
    "### 验证环境":                                  "#### 5.2.1 验证环境",
    "### 验证方法":                                  "#### 5.2.2 验证方法",
    "### 单核ResNet回归测试":                        "#### 5.3.1 单核 ResNet 回归测试",
    "### 单核形状鲁棒测试集":                        "#### 5.3.2 单核形状鲁棒测试集",
    "### 多核W切片测试集":                           "#### 5.3.3 多核 W 切片测试集",
    "### 残差网络ResNet11验证":                      "#### 5.3.4 残差网络 ResNet11 验证",
    "### 资源占用":                                  "#### 5.4.1 资源占用",
    "### 时序分析":                                  "#### 5.4.2 时序分析",
    "### 功耗分析":                                  "#### 5.4.3 功耗分析",
    "### 端到端时延与帧率":                          "#### 5.5.1 端到端时延与帧率",
    "### 乘加阵列单元利用率":                        "#### 5.5.2 乘加阵列单元利用率",
    "### 多核扩展加速比":                            "#### 5.5.3 多核扩展加速比",
    "### 调度策略量化分析":                          "#### 5.5.4 调度策略量化分析",
    "### 参数敏感度扫频分析":                        "#### 5.5.5 参数敏感度扫频分析",
    "### 多网络ASIC工艺折算性能":                    "#### 5.5.6 多网络 ASIC 工艺折算性能",
}


def process(text: str) -> str:
    lines = text.splitlines()
    out = []

    # 状态计数
    yinyan_idx = 0
    benzhangxiaojie_idx = 0

    i = 0
    while i < len(lines):
        ln = lines[i]
        stripped = ln.strip()

        # ─── 删除封面 / 声明 / 目录 / 修改记录 / 致谢说明 ─────────
        if i < 40:
            # 前 40 行是封面 + 声明, 直接跳
            i += 1
            continue

        # 目录 (line 63 起的 [N章 [page](#anchor)](#anchor) 项): 整段跳到 # 文献综述
        if stripped == "目录":
            # 跳到下一个 # 一级标题
            i += 1
            while i < len(lines) and not lines[i].startswith("# "):
                i += 1
            continue

        # ─── 一级标题映射 ───────────────────────────────────
        if stripped in H1_MAP:
            out.append(H1_MAP[stripped])
            i += 1
            continue

        # ─── 二级标题映射 ───────────────────────────────────
        if stripped == "## 引言":
            if yinyan_idx < len(YINYAN_ORDER):
                out.append(YINYAN_ORDER[yinyan_idx])
                yinyan_idx += 1
            i += 1
            continue
        if stripped == "## 本章小结":
            if benzhangxiaojie_idx < len(BENZHANGXIAOJIE_ORDER):
                out.append(BENZHANGXIAOJIE_ORDER[benzhangxiaojie_idx])
                benzhangxiaojie_idx += 1
            i += 1
            continue
        if stripped == "## <span class=\"mark\">功能仿真验证</span>":
            out.append("### 5.3 功能仿真验证")
            i += 1
            continue
        if stripped in H2_NUMBERING and H2_NUMBERING[stripped]:
            out.append(H2_NUMBERING[stripped])
            i += 1
            continue

        # ─── 三级标题映射 ───────────────────────────────────
        if stripped in H3_TO_H4_MAP:
            out.append(H3_TO_H4_MAP[stripped])
            i += 1
            continue

        # ─── 结论 (无 # 标题, 单独识别) ───────────────────────
        if stripped == "结论" and i + 1 < len(lines) and lines[i + 1].strip() == "====":
            out.append("## 6 结论")
            i += 2
            continue
        if stripped == "附录A FLUX_CNN各模块配置寄存器映射表" \
           and i + 1 < len(lines) and "====" in lines[i + 1]:
            out.append("## 附录 A FLUX_CNN 各模块配置寄存器映射表")
            i += 2
            continue
        if stripped == "<span id=\"_Toc216894849\" class=\"anchor\"></span>参考文献":
            out.append("## 参考文献")
            i += 1
            continue
        if stripped == "致谢" and i + 1 < len(lines) and lines[i + 1].strip() == "====":
            out.append("## 致谢")
            i += 2
            continue

        # 跳过模板填充文字
        if stripped.startswith("修改记录正文选用") \
           or stripped.startswith("致谢正文选用") \
           or stripped.startswith("毕业论文（设计）致谢中") \
           or stripped.startswith("对其他在本研究工作") \
           or stripped.startswith("这部分内容不可省略") \
           or stripped.startswith("书写格式说明") \
           or stripped.startswith("标题“致谢”") \
           or stripped.startswith("记录人（签字）") \
           or stripped.startswith("指导教师（签字）") \
           or stripped.startswith("修改是论文写作过程中") \
           or stripped.startswith("以下内容要求填写到毕业论文") \
           or stripped.startswith("一、毕业论文") \
           or stripped.startswith("二、指导教师变更") \
           or stripped.startswith("三、校外毕业论文") \
           or stripped.startswith("四、毕业论文（设计）内容") \
           or stripped.startswith("五、毕业论文（设计）外文") \
           or stripped.startswith("六、毕业论文（设计）最后") \
           or stripped.startswith("本人于****年*月申请") \
           or stripped.startswith("校内指导教师为") \
           or stripped.startswith("包括：指导教师要求的重大") \
           or stripped.startswith("根据实际情况记录论文写作过程中") \
           or stripped.startswith("阅后删除此文本框") \
           or stripped in {"修改记录", "原题目：", "修稿后题目：", "原指导教师：",
                            "变更后指导教师：", "=", "<u>以下内容要求填写到毕业论文（设计）修改记录中：</u>"}:
            i += 1
            continue

        out.append(ln)
        i += 1

    text = "\n".join(out) + "\n"

    # ─────────────────────────────────────────────────────────
    # 全文级正则替换
    # ─────────────────────────────────────────────────────────

    # 1) figure HTML -> markdown image
    # <figure>...<img src="..." style="..." />...<figcaption><p><span...>图N caption</p></figcaption></figure>
    def figure_replace(m):
        block = m.group(0)
        src_m = re.search(r'<img\s+[^>]*src="([^"]+)"', block)
        cap_m = re.search(r'<figcaption>\s*<p>(?:<span[^>]*></span>)?(.*?)</p>\s*</figcaption>',
                          block, re.DOTALL)
        src = src_m.group(1) if src_m else ""
        cap = (cap_m.group(1).strip() if cap_m else "").replace("\n", " ")
        # 替换本地 .docx-media 路径为相对 figures-rendered 路径占位
        src = src.replace(r"C:\_Project\FLUX_CNN\paper\workspace\.docx-media/media/",
                          "figures-rendered/png/").replace("\\", "/")
        return f"\n![{cap}]({src})\n\n*{cap}*\n"

    text = re.sub(r'<figure>.*?</figure>', figure_replace, text, flags=re.DOTALL)

    # 2) 行内单独的 anchor span -> 删除
    text = re.sub(r'<span id="[^"]+" class="anchor"></span>', '', text)

    # 3) <sup>\[N\]</sup> -> [N]
    text = re.sub(r'<sup>\\\[([\d, \-]+)\\\]</sup>', r'[\1]', text)

    # 4) <span class="mark">...</span> -> ... (保留文本, 去高亮)
    text = re.sub(r'<span class="mark">(.*?)</span>',
                  lambda m: m.group(1), text, flags=re.DOTALL)

    # 5) [图N.M](#anchor) -> 图N.M; [表N.M](#anchor) -> 表N.M
    text = re.sub(r'\[(图\s*\d+(?:\.\d+)?)\]\(#[^)]+\)', r'\1', text)
    text = re.sub(r'\[(表\s*\d+(?:\.\d+)?[^\]]*)\]\(#[^)]+\)', r'\1', text)
    text = re.sub(r'\[(附录[^\]]+)\]\(#[^)]+\)', r'\1', text)

    # 6) ``` math 块外加换行
    # 已经是 pandoc 的 math fence, 保留, 但简化里面的 \#(STYLEREF ... ) 编号
    text = re.sub(r'\\#\(\\?\s*STYLEREF[^)]+\)', '', text)

    # 7) 公式中 $`...`$ 这种 pandoc 特殊语法 -> $...$
    text = re.sub(r'\$`([^`]+?)`\$', r'$\1$', text)

    # 8) 行内 \$ -> $
    text = text.replace(r'\$', '$')

    # 9) 公式中 LaTeX 空格 \  /  \,  /  \; 简化为普通空格
    #    例: $\ 65\ nm$  -> $65~nm$ -> 65 nm (公式纯数字+单位的话直接拆出来)
    def _clean_math_inline(m):
        body = m.group(1)
        # 去 LaTeX 间距
        body = re.sub(r'\\[,;:!\s]', ' ', body)
        body = re.sub(r'\s+', ' ', body).strip()
        # 纯数字+单位 (如 65 nm / 980 MHz / 92 TOPS / 0.2 pJ) 解开 $...$
        if re.fullmatch(r"[\d.,~×x*\-\+\s]+\s?(nm|MHz|GHz|GOPS|TOPS|TOPS/W|MAC|mW|W|pJ|KB|MB|GB|bit|fps|FPS|ms|μs|us|ns|cycles|ops)?", body):
            return body
        # 含 \times / \cdot / \sim 等 LaTeX 命令则保留 $..$
        if re.search(r'\\[A-Za-z]+', body):
            return f"${body}$"
        # 单纯几个变量名 (C_in 之类) 保留
        return f"${body}$"

    text = re.sub(r'\$([^$\n]+?)\$', _clean_math_inline, text)

    # 10) HTML 表格 -> markdown 表格 (简化版, 处理 pandoc 输出的 <table>)
    def html_table_to_md(m):
        tbl = m.group(0)
        # caption
        cap_m = re.search(r'<caption>\s*<p>(.*?)</p>\s*</caption>', tbl, re.DOTALL)
        caption = re.sub(r'<[^>]+>', '', cap_m.group(1)).strip() if cap_m else ""

        # 提取所有行
        rows = re.findall(r'<tr>(.*?)</tr>', tbl, re.DOTALL)
        md_rows = []
        for row in rows:
            cells = re.findall(r'<t[hd][^>]*>(.*?)</t[hd]>', row, re.DOTALL)
            md_cells = []
            for c in cells:
                # 删 <p> <strong> <sup> 等 tag, 保留文字
                c = re.sub(r'<strong>(.*?)</strong>', r'**\1**', c, flags=re.DOTALL)
                c = re.sub(r'<sup>(.*?)</sup>', r'\1', c, flags=re.DOTALL)
                c = re.sub(r'</p>\s*<p>', ' ', c, flags=re.DOTALL)
                c = re.sub(r'<[^>]+>', '', c)
                c = re.sub(r'\s+', ' ', c).strip()
                # 反义 \[N\] -> [N]
                c = re.sub(r'\\\[([\d, \-]+)\\\]', r'[\1]', c)
                md_cells.append(c)
            if md_cells:
                md_rows.append("| " + " | ".join(md_cells) + " |")

        if not md_rows:
            return ""

        # 第一行做表头, 第二行加分隔
        n_cols = md_rows[0].count("|") - 1
        sep = "|" + "|".join([" --- "] * n_cols) + "|"
        result = "\n" + md_rows[0] + "\n" + sep + "\n" + "\n".join(md_rows[1:]) + "\n"
        if caption:
            result = result + f"\n*{caption}*\n"
        return result

    text = re.sub(r'<table[^>]*>.*?</table>', html_table_to_md, text, flags=re.DOTALL)

    # 11) 表 1.1 这种 cell 内含多行 ``` math 块的 pandoc-markdown 表 -> 简化
    #     这种表格里 pandoc 把单元格内的 $`1`$ 数学符号转成了 fenced math block
    #     破坏了 markdown 表格结构. 整段重写比较稳妥. 仅做基础清理:
    #     连续 cell 行有 "``` math" 字符串则把这整段表格降格成普通文本块, 提示人工修复.
    def simplify_broken_table(m):
        block = m.group(0)
        if "``` math" not in block:
            return block
        # 标记为需要人工修复, 但内容仍保留
        return f"\n<!-- [TODO] 上方表格因含内嵌 math block 由 pandoc 输出畸形, 需手工重整 -->\n{block}\n"

    text = re.sub(r'(\|[^\n]+\|\n)+', simplify_broken_table, text)

    # 12) Abstract 占位文字段移除 (pandoc 把模板说明也搬进来了)
    abstract_placeholder = re.compile(
        r'## Abstract\n\n.*?(?=## 1 绪论)', re.DOTALL)
    text = abstract_placeholder.sub(
        "## Abstract\n\n*\\[TODO: 英文摘要待用户撰写, 内容与中文摘要对应\\]*\n\n"
        "**Key Words:** *\\[TODO: 3-5 keywords\\]*\n\n",
        text)

    # 13) 多空行压缩
    text = re.sub(r'\n{4,}', '\n\n\n', text)

    return text


def main():
    src_text = SRC.read_text(encoding='utf-8')
    new_text = process(src_text)

    # 头部插入论文标题与符号表 (沿用之前的)
    header = """# 面向端侧流式计算场景的卷积加速器设计

**Design of a Convolutional Accelerator for Edge-Side Streaming Computing Scenarios**

学院：集成电路学院
专业：集成电路设计与集成系统
学生姓名：丁佩一
学号：20222261050
大连理工大学

"""
    DST.write_text(header + new_text, encoding='utf-8')
    print(f"[OK] wrote {DST}  ({len(new_text.splitlines())} lines)")


if __name__ == '__main__':
    main()
