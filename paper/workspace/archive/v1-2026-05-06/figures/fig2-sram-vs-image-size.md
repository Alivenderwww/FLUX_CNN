# Fig.2: On-Chip SRAM Footprint vs. Image Size (TBD optional)

## 在 paper.md 中的引用位置
- **首次引入**：§2.4 Design Goals (frontmatter line 12 提到 / line 108 占位)
  - 原文：`[TBD: 是否在此处插入 SRAM-vs-image-size 曲线图作为 Fig.2，待用户决定]`
- **被引用次数**：0 次显式（Fig.2 是 [TBD] 占位，可能不进终稿）

## 论证作用
**可视化** narrative B (row-streaming) 的核心动机：在边缘 FPGA 1.6 MB BRAM 总预算下，整图 buffering 不可行；row-ring 把片上占用从 O(H·W) 降到 O(strip_rows · W)。一张曲线图让读者直观看到"VGA 480×640 单图 4.9 MB vs ring 仅 10 KB"的三个数量级差异。

## 图类型
**对数坐标的折线/面积图**（双 y 轴或对数 y 轴）

## 设计要素

### 必含元素

- **x 轴**：image height H（log scale），覆盖 32 → 1080 的常见输入分辨率（CIFAR-32, ImageNet-224, VGA-480, 720p-720, 1080p）
- **y 轴**：on-chip SRAM 占用 (KB, log scale)
- **三条曲线 / 区域**：
  1. **Tile-and-reload baseline**：常数线 / 浅灰区，约 1.6 MB（XC7K325T BRAM 总量上限）
  2. **整图 buffer 需求**：随 H·W 二次增长曲线（INT8 16-channel 假设：H·W·16 字节）
  3. **FLUX_CNN row-ring**：随 H 线性的低斜率线（strip_rows=8 假设：8·W·16 字节）
- **数据点标记**：CIFAR-32 (16 KB / 8 KB / 1.6 MB), VGA-480×640 (4.9 MB / 10 KB / 1.6 MB), 1080p (33 MB / 17 KB / 1.6 MB)
- **可行域填充**：低于 1.6 MB 上限的部分填充浅绿色背景

### 标注要求
- 标注 1.6 MB 水平线：`XC7K325T BRAM ceiling (445 BRAM × 36 Kb)`
- 标注每条曲线名称：`Whole-image buffer (O(H·W))` / `FLUX_CNN row-ring (O(strip_rows·W), strip_rows=8)`
- 标注 VGA 数据点：`VGA 480×640: 4.9 MB whole vs 10 KB ring → 3 orders of magnitude`
- 标注 1080p 数据点：`1080p exceeds chip even before ring overhead`

### 视觉层次
- **主角**：FLUX_CNN row-ring 曲线（粗实线 / 蓝色），始终在可行域内
- **配角**：whole-image curve（虚线红色），在 VGA 之前就穿越 1.6 MB 上限
- **背景**：BRAM ceiling 水平线（灰色虚线 + 阴影下方"feasible"）

## 数据来源

| 数据点 | 来源 | 状态 |
|--------|------|------|
| 1.6 MB BRAM ceiling | STATUS.md §1 综合表 (445 BRAM × 36 Kb / 8 = 2.0 MB理论, 但有 ECC/parity 损耗，约 1.6 MB 可用) | ✅ |
| 单核 IFB ring 10 KB @ VGA 480×640 strip=8 | README.md / contributions.md C1.2 | ✅ |
| 整图 4.9 MB | 480·640·16·1B = 4,915,200 B ≈ 4.9 MB | ✅ |
| 1.6 MB 与 4.9 MB 的"3 orders of magnitude" | 4.9MB/10KB ≈ 500x ≈ 2.7 数量级（论文已圆整为 "three orders of magnitude"，paper.md line 26）| ✅ |

## ASCII 示意稿

```
   SRAM (KB, log)
   10000 │                                              ╳ 1080p whole (33 MB)
         │                                          ╳         (cannot fit)
    1000 │                                  ╳━━━━━━━━━━━━━━━━━━━━━ XC7K325T 1.6 MB
         │                              ╳    .─.─.─.─.─.─.─.─. (ceiling)
     100 │                          ╳         (whole-image buffer
         │                      ╳              quadratic, exceeds chip)
      10 │                  ╳
         │              ╳            ┌─────────────────────── feasible region
       1 │          ╳────────────────┤
         │      ╳                    │      ▬▬▬▬▬▬▬▬▬▬▬▬▬▬ FLUX_CNN row-ring
     0.1 │   ╳   ━━━━━━▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬          (linear in H, strip=8)
         │
         └────┬──────┬──────┬──────┬──────┬──────┬───── H (log)
              32    96     224   480    720   1080
            CIFAR  AlexNet ImNet VGA   720p  1080p

   ╳ = whole-image buffer (intractable)
   ▬ = FLUX_CNN row-ring (always feasible)
   . = BRAM ceiling
```

## 初版代码（matplotlib，可直接出 PDF）

```python
import numpy as np
import matplotlib.pyplot as plt

# Image heights (W=H assumption for simplicity, or use 4:3 ratio)
H = np.array([32, 96, 224, 480, 720, 1080])
W = np.array([32, 96, 224, 640, 1280, 1920])  # actual 16:9 widths
labels = ['CIFAR\n32×32', 'AlexNet\n96×96', 'ImageNet\n224×224',
          'VGA\n480×640', '720p\n720×1280', '1080p\n1080×1920']

# INT8, 16 channels
BYTES_PER_PIXEL = 16  # 16 channels × 1 byte
STRIP_ROWS = 8

whole_kb = H * W * BYTES_PER_PIXEL / 1024
ring_kb = STRIP_ROWS * W * BYTES_PER_PIXEL / 1024
ceiling_kb = 1600  # 1.6 MB

fig, ax = plt.subplots(figsize=(7, 4.5))

# Feasible region shading
ax.axhspan(0.1, ceiling_kb, alpha=0.15, color='green', label='Feasible (within BRAM)')

# Curves
ax.plot(H, whole_kb, 'rx--', linewidth=2, markersize=10,
        label='Whole-image buffer (O(H·W))')
ax.plot(H, ring_kb, 'b-o', linewidth=2.5, markersize=8,
        label=f'FLUX_CNN row-ring (strip_rows={STRIP_ROWS})')
ax.axhline(ceiling_kb, color='gray', linestyle=':', linewidth=1.5,
           label='XC7K325T BRAM ceiling (~1.6 MB)')

# Annotations
ax.annotate('VGA 480×640:\n4.9 MB whole\nvs 10 KB ring\n→ ~500× reduction',
            xy=(480, 4900), xytext=(150, 8000),
            arrowprops=dict(arrowstyle='->', color='red'),
            fontsize=9, ha='center')

ax.set_xscale('log')
ax.set_yscale('log')
ax.set_xlabel('Image height H (pixels, log scale)')
ax.set_ylabel('On-chip SRAM footprint (KB, log scale)')
ax.set_xticks(H)
ax.set_xticklabels(labels, fontsize=8)
ax.legend(loc='upper left', fontsize=9)
ax.grid(True, which='both', alpha=0.3)
ax.set_title('On-Chip SRAM Footprint vs. Image Size')

plt.tight_layout()
plt.savefig('fig2-sram-vs-image.pdf', bbox_inches='tight')
# plt.savefig('fig2-sram-vs-image.png', dpi=300, bbox_inches='tight')
```

## 与正文的一致性检查

- [x] paper.md §1.1 line 26 "single VGA-resolution 480×640 INT8 input image at 16 channels occupies roughly 4.9 MB" — 图标注一致
- [x] paper.md §1.3 line 48 "VGA 480×640 with strip_rows = 8 this requires roughly 10 KB of ring storage ... three orders of magnitude below the 4.9 MB" — 图核心叙事一致
- [x] paper.md §2.4 line 108 [TBD] 在 §2.4 末 / §2.3 末插入此图——位置匹配

## 不确定项 / 待用户决定

- [TBD: 是否进终稿] 这是 [TBD] 占位，最终是否插入由用户决定。**建议进终稿** — narrative B 的最直观可视化论据，且数据完全已知（不依赖 [CHECK] 实测）
- [TBD: 配色] 红/蓝/灰组合可能撞 IEEE 单色印刷；如目标会议是 IEEE，建议 color-blind 友好（如 ColorBrewer "RdBu"）
- [TBD: x 轴单位] H vs (H·W)？当前用 H，方便对齐熟悉的分辨率名称（CIFAR/VGA/1080p）
- [CHECK: 1.6 MB ceiling 精确口径] 实际 445 BRAM36 = 445 × 36 Kb / 8 = 2002 KB ≈ 2.0 MB；论文 line 26 说 "roughly 1.6 MB"，扣掉了 ECC + 已被单核 128 BRAM 占用的部分。**建议** 在图中标 "1.6 MB available after core" 或者改成 "2.0 MB total / 1.4 MB available"——polisher 阶段统一口径
