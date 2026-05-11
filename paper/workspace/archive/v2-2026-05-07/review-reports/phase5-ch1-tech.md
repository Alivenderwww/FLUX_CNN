# 技术评审报告 Phase 5 §1 绪论

## 第 1 次评审

### 判定
PASS

### 评审范围（仅管灵魂）

- 评审对象：`paper/workspace/paper.md` 第 1 章绪论（§1.1—§1.4，第 17—115 行，约 35 段）
- 仅评内容正确 / 数据可追溯 / [CHECK]/[TBD] 合规 / 三威胁差异化 / contribution 引用对齐
- **不评格式**（中英空格 / 字母正斜体 / 残留英文术语 / 引用句式 — 留 Phase 8 polisher）
- 未读 §2—§6（仍为 [TBD] 占位，本次不在范围）

### 段落骨架对齐

实际段数与 `paragraph-skeleton.md` §1（35 段）整体匹配，分布略有调整但主题句覆盖完整：

| 节 | 骨架段数 | paper.md 段数 | 备注 |
|----|---------|--------------|------|
| §1.1 | 6 | 6 | 完全对齐 |
| §1.2 章首 | — | 1 | 新增 1 段引言（非回归，是合理过渡） |
| §1.2.1 | 4 | 4 | 完全对齐 |
| §1.2.2 | 4 | 4 | 完全对齐 |
| §1.2.3 | 5 | 5 | 完全对齐（关键差异化章） |
| §1.2.4 | 4 | 3 | 合并 1 段（transition 并入 claim 段，主题未丢） |
| §1.2.5 | 4 | 3 | 合并 1 段（同上） |
| §1.3 | 5 | 5（含贡献列表块） | 完全对齐 |
| §1.4 | 3 | 2 | 合并 1 段（小节安排合并到段 2） |

未发现遗漏关键主题句，未引入骨架未提的"新主张"。

### 三威胁差异化（§1.2.3，最关键）

三处近邻 prior art 差异化论证经得起审稿挑战，且与 `literature.md` §C 完全对齐：

1. **Alwani Fused-layer (MICRO'16)**
   - paper.md：VGGNet-E 前 5 层 77 MB → 3.6 MB / 95% 节省 ✅ 与 literature.md line 304 完全一致
   - 差异点：FLUX 行环只缓 strip_rows × W_IN（contributions.md C1.3 验证），单层硬件可处理任意 H×W；与 Alwani 多层 fused 整图驻留形成"缓冲粒度代差"——论证清晰。

2. **Kang AoCStream (arXiv'22 / Sensors'23)**
   - paper.md venue 写法与 `literature.md` 第 5 次启动 venue 修正后一致（不是骨架里的 FCCM'23）。✅ Writer 对齐了最新 literature.md，非回归。
   - 差异点：FLUX 用 row-credit 反压协议解耦输入分辨率与片上容量，VGA 480×640 单图 4.9 MB 仅用约 10 KB ring（contributions.md C1.3 数据 + literature.md line 537 对照一致）。

3. **Liu Full-Stack (TNNLS'21)**
   - paper.md 作者格式按 literature.md F6 修正后的 "Liu et al." 写法，venue 用 TNNLS'21（不是骨架里的 FPL'23）；Intel Arria 10、layer fusion + multi-level parallelism 描述与 literature.md line 319-325 一致。✅
   - 差异点：FLUX 单核 layer-serial + 多核 W 切片 vs Liu 多核跨层 pipelined，对应 contributions.md C3.4 / C4.4 数据（596K/450K/354K cycles）作为兑现承诺。

三处都给出了"具体数字 + 差异点 + §3.4.3 / §5.7 兑现承诺"的完整模板，**审稿人威胁应对充分**。

### 数据真实性抽样

抽样 6 处具体数字 / claim → 全部可追溯：

1. §1.1 line 23 "BRAM36 仅 445 块" → ✅ literature.md line 525 + STATUS §1（容量常识级，无问题）
2. §1.1 line 27 "Patch 层 654,404 → 129,594 cycles，5.05×" → ✅ contributions.md C2.2 完全一致
3. §1.2.3 line 61 "Alwani VGGNet-E 77 MB→3.6 MB / 95%" → ✅ literature.md line 304
4. §1.2.3 line 65 "ResNet-11 N=1/2/4 596K/450K/354K wall cycles" → ✅ contributions.md C4.4（596,088 / 450,469 / 354,555）
5. §1.3 line 100 贡献 2 "S2D 联合触发 by force_s2d/force_fold" → ✅ contributions.md C2.3 函数名直接对照
6. §1.3 line 102 贡献 4 "N=4 较 N=1 实现 1.68× wall cycles 加速" → ✅ contributions.md C3.4 / C4.4

**未发现编造数字、张冠李戴或与代码不符的实现描述。**

### claim 强度

- §1.2.4 line 75：使用 "已检索文献中尚未见到把 S2D 作为编译器侧 PE 利用率优化路径的明确 prior art" + "据已知文献..." 措辞 ✅ 保守
- §1.2.4 line 75：明确点出"该等价关系并非本工作的发明"——诚实标注谱系来自超分辨率 ✅
- §1.3 五条贡献全部使用功能性描述，**未出现 "first to..." / "novel" / "best" 等过强 claim** ✅
- §1.1 line 25：未夸大 PE 利用率塌陷的负面影响，措辞克制 ✅
- 与 contributions.md "诚实自评强度" 字段完全对齐：C2.1/C2.2 在 paper.md 中作为核心 novelty 候选，但无 over-claim

### [CHECK] / [TBD] 合规

paper.md §1 内容标记（不计文档元信息 line 2-13）：

| 位置 | 标记 | 合规性 |
|------|------|--------|
| line 27 | `[CHECK-4.3.1]` Patch 利用率塌陷数字 | ✅ 合理（contributions.md C2.1 也标 CHECK） |
| line 73 | `[CHECK-1.2.4]` S2D 谱系措辞待复查 | ✅ 合理（literature.md line 548 同标 CHECK） |
| line 75 | `[CHECK-1.2.4]` 重复保险 | ✅ 合理 |
| line 105 | `[TBD-1.3.3]` 贡献是否压到 4 条 | ✅ 合理（contributions.md / outline 取舍点） |

Writer 自报"2 [CHECK] + 3 [TBD]"与实测略有出入（实际 3 [CHECK] + 1 [TBD]），属字数统计偏差，不影响判定。**未发现"该标却未标"或"误标"的情况**。

### 回归性

逐项检查是否引入 paragraph-skeleton 未提的新数据 / 新引用 / 新 claim：

- ✅ 所有引用均来自 `literature.md`（含 §A/§B/§C/§D/§G）；无幻觉新引用
- ✅ §1.1 line 27 提前给出 S2D 单层加速数字（骨架 §1.1 段 4 标 [CHECK]，paper.md 用了 contributions.md C2.2 真实数字）—— 不算回归，是对 [CHECK] 的合法兑现
- ✅ §1.2.5 line 85 提到 `s2d_eff()` / `build_step_cfg_dict` 具体函数名 —— 来自 contributions.md C2.3 直接定位，不算新事实
- ✅ §1.3 line 95 列举 16 条贡献映射 —— 与 outline §4.1 段 2 一致
- ✅ Kang venue 用 "arXiv'22 / Sensors'23" 而非骨架 "FCCM'23"，Liu 用 "TNNLS'21" 而非骨架 "FPL'23" —— 对齐 literature.md v5 修正，**不是回归而是必要修正**
- ✅ §1.2.3 line 65 引入 596K/450K/354K wall cycles 具体数字 —— 来自 contributions.md C4.4，可追溯

**未发现新事实 / 新引用 / 新 claim 引入。**

### 通过

- 段落骨架 35 段对齐
- 三威胁差异化论证完整、数据可追溯、与 literature.md 完全一致
- 6 处抽样数字全部能在 contributions.md 找到来源
- claim 强度保守，使用 "据已知文献..." / "已检索文献中尚未见到..." 措辞
- [CHECK]/[TBD] 标记位置合理
- 无回归性新事实
- 无编造、无幻觉文献、无张冠李戴

### 失败

无。

### 修订建议（非阻塞）

留给后续 polisher / Writer 注意的次要事项（不影响 PASS 判定）：

1. Writer 自报标记数 "2 [CHECK] + 3 [TBD]"与实测 "3 [CHECK] + 1 [TBD]" 不符，属字数统计偏差，建议下次扩写后核对。
2. §1.2.4 / §1.2.5 段数比骨架少 1 段（合并 transition 入 claim），未丢主题句但可在后续 polish 时考虑是否拆回独立 transition 段以提升节奏感。
3. §1.4 段 2 把 "第 2-6 章简介" 全部塞入 1 段（line 113-115），密度偏高，polisher 可考虑拆分以利阅读。
4. Writer 自报字数 5310 < 9000 目标 —— 已确认这是 polisher 阶段补足的范围，不是技术评审的关切。
