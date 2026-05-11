# 图 5.3 Pareto 前沿散点图（GOPS/DSP vs 整网 PE 利用率）
# Figure 5.3 Pareto frontier scatter plot of GOPS-per-DSP versus end-to-end PE utilisation

## 在论文中的角色
- 首次引入：§5.7 段 "具体 Pareto 散点图（GOPS / DSP vs 整网 PE 利用率）见图 5-3 [CHECK: 图 5-3 散点数据来源]"
- 论证作用：把本工作放到与 fpgaConvNet / Snowflake / Angel-Eye / Liu Full-Stack / Aydonat / Lu Winograd / Ma OPU 等同器件 / 同思路 streaming CNN 加速器的二维 Pareto 前沿坐标系中，可视化"中型器件 + 编译器侧 PE 利用率优化" 位置。

## 图类型
散点图：横轴 = 整网 PE 利用率（%），纵轴 = GOPS / DSP（每 DSP 算力效率）；每个点 = 一个文献工作；标注本工作（红色五角星）的 Pareto 位置。

## 设计要素

### 必含元素
- **横轴**：整网 PE 利用率（%），范围 [0, 100]
- **纵轴**：GOPS / DSP，范围 [0, 1.5] 或类似（具体根据数字范围调）
- **点 A：本工作 (单核)** — XC7K325T，PE 利用率 86.6% [CHECK]、GOPS=51.2 / DSP=82 → **0.624 GOPS/DSP**
- **点 B：本工作 (N=4 SMC)** — XC7K325T，PE 利用率 [CHECK]、N=4 GOPS [CHECK] / DSP=320 → [CHECK]
- **点 C：fpgaConvNet [Venieris]** — Zynq XC7Z045，[CHECK] 数据
- **点 D：Snowflake [Bottleson ISCAS'17]** — Zynq XC7Z045，PE 利用率 91% [CHECK]、128 GOPS [CHECK] / DSP_count [CHECK]
- **点 E：Angel-Eye [TCAD'18]** — Zynq，[CHECK]
- **点 F：Liu Full-Stack [TNNLS'21]** — Arria 10 GX1150，PE 利用率 97%、>1.3 TOPS [CHECK] / 1518 DSP → [CHECK] GOPS/DSP
- **点 G：Aydonat Intel DLA** — Arria 10，1382 GFLOPS [CHECK]（FP16，与 INT8 不直接可比）
- **点 H：Lu Winograd ZCU102** — ZCU102，854.6 GOPS [CHECK]（Winograd 路径）
- **点 I：Ma OPU** — [CHECK 数据]
- **本工作 Pareto 标注**：在点 A / B 旁画粗体红色五角星 + 标 "本工作"
- **Pareto 前沿线**：根据可比点画一条折线（凸壳上沿）

### 标注要求
- 不同器件量级用不同形状：方形 = 7-series（XC7Z045 / XC7K325T）、圆形 = UltraScale+（ZCU102）、三角 = Arria 10
- 不同工作路线用不同颜色：streaming = 蓝、Winograd = 黄、OPU = 紫、本工作 = 红
- 在每个点旁标短标签（"FpgaCN" / "Snow" / "Liu" / "DLA" / "Lu Wino" / "Ma OPU" / "本工作 N=1" / "本工作 N=4"）
- 顶部注："数据完整度 [CHECK]——表 5.9 baseline 部分仅 fpgaConvNet 2.94× 是 performance density 不是 fps，Aydonat 走 FP16 路线，与 INT8 不直接可比；本图仅展示 PE 利用率 + GOPS/DSP 两维度可对齐数据"
- 加 "口径差异：整网 PE 利用率统计口径不完全等同（含 / 不含 IDMA stall），见表 5.9 脚注"

### 视觉层次
- 主角：本工作 A / B 两个红色五角星
- 配角：其他 baseline 散点
- 背景：Pareto 前沿折线 + 坐标轴

## ASCII 示意稿

```
   GOPS / DSP
   ▲
1.0│    
   │             ▲ Liu Full-Stack [CHECK]
0.8│             (Arria 10, PE% 97%)
   │
0.6│ ★ 本工作 N=1
   │   (XC7K325T, PE% 86.6% [CHECK], 0.624 GOPS/DSP @ target)
   │ ★ 本工作 N=4 SMC [CHECK]
0.4│
   │  ■ Snowflake [CHECK]
   │  (XC7Z045, PE% 91%)
0.2│
   │   ▲ Aydonat DLA [CHECK]
   │   (Arria 10, FP16)            ● Lu Winograd [CHECK]
0.0│ ■ FpgaCN ■ Angel-Eye          (ZCU102)         ● Ma OPU
   │  [CHECK]   [CHECK]
   └──────────┬──────────┬──────────┬──────────┬──→ 整网 PE 利用率 (%)
              25%        50%        75%        100%

   形状：■ 7-series FPGA   ▲ Arria 10   ● UltraScale+
   颜色：蓝 = streaming   黄 = Winograd   紫 = OPU   红 = 本工作

   注释：
   · 仅展示数据完整度允许的 baseline；fpgaConvNet/Aydonat/Lu/Ma 数据
     完整度 [CHECK]，口径差异显著
   · 整网 PE 利用率统计口径不完全等同（含/不含 IDMA stall），表 5.9 脚注详
   · 本工作 N=1 / N=4 SMC 落在"中型器件 + 编译器侧 PE 利用率优化"位置
   · 与 Liu Full-Stack 形成同思路、不同器件规模的可比对照
```

## 数据来源
- paper.md §5.7 表 5-9 / §5.7 末段 Pareto 散点图
- literature.md §A-G（各 baseline 条目）
- 本工作数据：STATUS.md §1 单核综合 / §2.8 / §2.12 / contributions.md C4.x

## 初版 Python 代码（用户最终绘制时参考）

```python
import matplotlib.pyplot as plt

# [CHECK: 各 baseline 数据完整度待 literature.md 复核]
data = {
    '本工作 N=1':       {'pe_util': 86.6, 'gops_per_dsp': 0.624, 'shape': '*', 'color': 'red'},  # [CHECK: 86.6%]
    '本工作 N=4 SMC':   {'pe_util': 0,    'gops_per_dsp': 0,     'shape': '*', 'color': 'red'},  # [CHECK]
    'fpgaConvNet':      {'pe_util': 0,    'gops_per_dsp': 0,     'shape': 's', 'color': 'blue'}, # [CHECK]
    'Snowflake':        {'pe_util': 91,   'gops_per_dsp': 0,     'shape': 's', 'color': 'blue'}, # [CHECK]
    'Angel-Eye':        {'pe_util': 0,    'gops_per_dsp': 0,     'shape': 's', 'color': 'blue'}, # [CHECK]
    'Liu Full-Stack':   {'pe_util': 97,   'gops_per_dsp': 0,     'shape': '^', 'color': 'blue'}, # [CHECK]
    'Aydonat DLA':      {'pe_util': 0,    'gops_per_dsp': 0,     'shape': '^', 'color': 'orange'},# [CHECK FP16/INT8 不可比]
    'Lu Winograd':      {'pe_util': 0,    'gops_per_dsp': 0,     'shape': 'o', 'color': 'yellow'},# [CHECK]
    'Ma OPU':           {'pe_util': 0,    'gops_per_dsp': 0,     'shape': 's', 'color': 'purple'},# [CHECK]
}

fig, ax = plt.subplots(figsize=(8, 6))
for name, d in data.items():
    if d['pe_util'] == 0 or d['gops_per_dsp'] == 0:
        continue  # [CHECK] 数据缺失跳过
    ax.scatter(d['pe_util'], d['gops_per_dsp'],
               marker=d['shape'], color=d['color'], s=200 if '本工作' in name else 100,
               label=name)
    ax.annotate(name, (d['pe_util'], d['gops_per_dsp']), fontsize=9, ha='left', va='bottom')

ax.set_xlabel('整网 PE 利用率 (%)')
ax.set_ylabel('GOPS / DSP (peak @ target Fmax)')
ax.set_title('Pareto 前沿：FPGA streaming CNN 加速器')
ax.set_xlim(0, 100)
ax.set_ylim(0, 1.0)
ax.grid(True, alpha=0.3)
ax.legend(loc='best', fontsize=8)
plt.tight_layout()
plt.savefig('figures/fig5-3-pareto.pdf')
```

## 与正文一致性检查
- [x] 图轴定义（PE 利用率 vs GOPS/DSP）— 与 §5.7 末段表述一致
- [x] 与表 5-9 同器件 streaming 横向数据呼应
- [x] Pareto 前沿措辞 — 与 §5.7 末段 "Pareto 前沿定位" 一致

## 不确定项
- 大量 baseline 数据 [CHECK]，等用户从 literature.md 详查后回填具体数字
- 本工作 N=4 SMC GOPS/DSP 数字 [CHECK]，需要 STATUS.md 与 §5.5 主表口径决策（W 切片 354K 还是 SMC 220K）后才能算
- 整网 PE 利用率 86.6% [CHECK]，需要与 model_analysis.md 交叉核对
