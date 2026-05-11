# §5 章节性能对比图渲染（v10 / v10.1 / v11 信息密度升级）

paper.md §5 共 16 张表，本目录提供 9 张可视化图渲染脚本与生成结果。

## 渲染清单

| 图 | 对应表 | 类型 | 论证要点 / 维度 |
|---|---|---|---|
| [fig5-4](fig5-4-end-to-end.png) | 表 5.4 | 演进路径图（log Y） | 5 阶段 × 周期数 / 加速比 / 帧率 / 增量机制 |
| [fig5-7](fig5-7-cin-kyfold.png) | 表 5.7 | 双 Y 轴 折线+柱 | $C_{\mathrm{in}}$ × {折叠开关} × {利用率, 周期数} |
| [fig5-8](fig5-8-k-sweep.png) | 表 5.8 | 双 Y 轴 柱+折线 | $K$ 扫频 × {触发数, PE 利用率, 周期/触发} |
| [fig5-9](fig5-9-stride-sweep.png) | 表 5.9 | 4 配置柱图 | $K=1$ stride=2 访存受限 20.7% |
| [fig5-10](fig5-10-h-stride-separation.png) | 表 5.10 | before/after 柱图（log Y） | 3 降采样层减幅 28-43% |
| [fig5-11](fig5-11-n-w-speedup.png) | 表 5.11 | 多曲线（实测 / halo 理论上限 / 4× 上限） | $N=4$ × $W$ × {加速比, PE 利用率, 边界冗余占比} |
| [fig5-12](fig5-12-resnet11-layers.png) | 表 5.12 | 堆叠柱+折线+层类型背景 | ResNet11 11 层 × {触发, 空闲, PE 利用率, 层类型} |
| [fig5-13](fig5-13-multi-network.png) | 表 5.13/5.14 | 散点图（log-log） | 7 网络 × {GMAC, fps, PE 利用率(色), 层数(大小), 频点(形状)} |
| [fig5-16](fig5-16-vs-accelerators.png) | 表 5.16 | 散点图（log-log） | 9 加速器 × {峰值 GOPS, fps, 网络(色), 是否本工作(形状)} |

## v11 信息密度升级（参考 DepFiN IEEE JSSC'23）

| 图 | v10.1 | v11 升级 |
|---|---|---|
| fig5-4 | 双 Y 轴分组柱（2 维） | 演进图 + 段间增量机制注记（4 维：周期/加速比/帧率/优化机制） |
| fig5-7 | 单系列折线（2 维） | 双 Y 轴 折线+柱（4 维：利用率/周期数 × 折叠开关） |
| fig5-11 | 双折线 + 4× 上限（2 维） | 三曲线 + 边界冗余占比（5 维：实测/halo 理论/4× 上限/PE 利用率/halo 占比） |
| fig5-12 | 堆叠柱+折线（3 维） | + 层类型背景色块 + 柱内数字（5 维：触发/空闲/利用率/层类型/触发量） |
| fig5-13 | 分组柱图（3 维） | 散点图（5 维：GMAC/fps/PE 利用率/层数/频点） |
| fig5-16 | 3 子图分组柱（3 维） | 单坐标系散点图（4 维：算力/fps/网络/是否本工作） |

## 设计原则（DepFiN 风格学习要点）

1. **跨数量级 log-log 散点**：让差距 1000× 的数据可读
2. **ROI 高亮**：方块 + 加粗边框区分本工作 vs 对照
3. **多曲线分族同色**：理论上限虚线 + 实测实线 + 利用率 alpha 0.85
4. **图内嵌入数值**：除轴值外直接标关键数据点的 fps / 减幅
5. **复合维度**：颜色 / 形状 / 大小再各加一维（时间、网络、配置）
6. **段间引导线**：演进图箭头连接 + 增量机制描述

## 公共样式（_style.py）

- 字体：Microsoft YaHei + Computer Modern math
- 配色：基线深蓝 #1f4e79 / 提升深绿 #2e7d32 / 第三系列深金 #b8860b
- 注记色 #404040 学术深灰
- 不写"图 X.Y"图内标题
- 输出 300 DPI PNG + SVG 矢量

## 复现命令

```bash
cd C:/_Project/FLUX_CNN/paper/figures-rendered
PY=../../toolchain/.venv/Scripts/python.exe
$PY fig5-4-end-to-end.py
$PY fig5-7-cin-kyfold.py
$PY fig5-8-k-sweep.py
$PY fig5-9-stride-sweep.py
$PY fig5-10-h-stride-separation.py
$PY fig5-11-n-w-speedup.py
$PY fig5-12-resnet11-layers.py
$PY fig5-13-multi-network.py
$PY fig5-16-vs-accelerators.py
```

## 已知微调项（可选）

- **fig5-13** "ResNet-18" 与 "AlexNet" 标签靠近（GMAC ≈1.1-1.8 区间数据点位置近）
- **fig5-4** 最右数据点上方框（190.1K / 756 fps）与"累计 5.86×"右上框靠近
- **fig5-8** "周期/触发"标签框与 PE 利用率折线点轻微重叠（K=5/K=7）

如需进一步调整，告诉我具体哪张图哪个元素位置 / 颜色 / 字号。

## 参考

- DepFiN: K. Goetschalckx et al., "DepFiN: A 12-nm Depth-First, High-Resolution
  CNN Processor for IO-Efficient Inference," *IEEE Journal of Solid-State Circuits*,
  vol. 58, no. 5, May 2023. — Fig. 1 / Fig. 14 / Fig. 16 散点图风格
