# 图 2.5 子像素重排（Pixel-Shuffle）等价关系示意图
# Figure 2.5 Pixel-shuffle equivalence between strided convolution and unit-stride convolution

## 在论文中的角色
- 首次引入：§2.5 段 "步幅卷积可以通过子像素重排转换为 stride=1 卷积加上对输入特征图的相位重排..." [依赖: Fig.2.5]
- 论证作用：用 stride=2 的具体小例子（4 个相位）建立直观，为 §4.3.4 / §4.3.5 的 S2D 加速器编译器侧数学推导铺垫。本图重点是"等价关系本身"——属于超分辨率领域已有概念，本工作只在加速器侧应用。

## 图类型
分相重排示意图：左侧原 stride=2 卷积输入（一张 H×W 特征图，标 4 个相位的位置），中间箭头为 "split into 4 phases"，右侧 4 张 (H/2 × W/2) 子图（每张对应一个相位 (a,b)，沿通道维堆叠成 cin_new = 4·Cin）。

## 设计要素

### 必含元素
- **左侧**：原输入特征图 H×W（取 H=W=8 示意），用 4 种颜色标记 (h,w) 位置 mod 2 的 4 个相位 (0,0)/(0,1)/(1,0)/(1,1)
- **中间**：标 "Pixel-Shuffle Split"（=Space-to-Depth），含 4 个箭头分别指向 4 个子图
- **右侧**：4 张 (H/2 × W/2) 子图，每张取自原图同相位的像素；4 张子图沿通道维堆叠示意成 cin_new = 4·Cin 的复合特征图
- **底部小字**：等价关系等式：`Conv(stride=2, input H×W, Cin) ≡ Conv(stride=1, input H/2 × W/2, cin_new = 4·Cin) + 相位重排（无激活复制）`

### 标注要求
- 4 种相位用 4 种纯色（红 / 蓝 / 绿 / 黄）填充对应像素
- 4 张子图也用对应纯色边框，建立"颜色 = 相位"的对应
- 箭头标注 "phase (0,0) → cin offset 0~Cin-1" / "phase (0,1) → cin offset Cin~2Cin-1" / 以此类推
- 图下加注："数学等价由超分辨率领域 Pixel-Shuffle / Sub-pixel Convolution（Shi CVPR'16）建立；本工作 §4.3 把它迁移到加速器编译器侧作 PE 利用率优化"

### 视觉层次
- 主角：左 / 右两组特征图与中间箭头
- 配角：4 个相位标签
- 背景：等价关系数学公式

## ASCII 示意稿

```
   原输入 (H×W=8×8, Cin)            4 个相位子图 (H/2 × W/2 = 4×4)
   ┌────────────────────┐           ┌────────┐
   │ R B R B R B R B    │           │ R R R R│  phase (0,0)
   │ G Y G Y G Y G Y    │           │ R R R R│  cin_offset = 0~Cin-1
   │ R B R B R B R B    │           │ R R R R│
   │ G Y G Y G Y G Y    │ ──split── │ R R R R│
   │ R B R B R B R B    │  ──→──    └────────┘
   │ G Y G Y G Y G Y    │           ┌────────┐
   │ R B R B R B R B    │           │ B B B B│  phase (0,1)
   │ G Y G Y G Y G Y    │           │ B B B B│  cin_offset = Cin~2Cin-1
   └────────────────────┘           │ B B B B│
                                    │ B B B B│
   R=phase(0,0)                     └────────┘
   B=phase(0,1)                     ┌────────┐
   G=phase(1,0)                     │ G G G G│  phase (1,0)
   Y=phase(1,1)                     │ G G G G│  cin_offset = 2Cin~3Cin-1
                                    │ G G G G│
                                    │ G G G G│
                                    └────────┘
                                    ┌────────┐
                                    │ Y Y Y Y│  phase (1,1)
                                    │ Y Y Y Y│  cin_offset = 3Cin~4Cin-1
                                    │ Y Y Y Y│
                                    │ Y Y Y Y│
                                    └────────┘
                                    
                    沿通道维堆叠 → cin_new = 4·Cin
   
   等价关系：Conv(stride=2, H×W, Cin) ≡ Conv(stride=1, H/2 × W/2, 4·Cin) + 相位重排
   引用谱系：Pixel-Shuffle / Sub-pixel Convolution (Shi et al. CVPR'16)
```

## 数据来源
- Shi et al. "Real-Time Single Image and Video Super-Resolution Using an Efficient Sub-Pixel Convolutional Neural Network" CVPR 2016
- paper.md §2.5 / §4.3.4
- docs/pe-fold.md §2

## 与正文一致性检查
- [x] "stride=s 的卷积 ≡ stride=1 卷积 on s² 个相位 + 通道维拼接"——与 §2.5 数学事实一致
- [x] 引用谱系明确标注（超分辨率领域 Sub-pixel）——与 §1.2.4 / §2.5 / §6.2 创新点 2 的诚实声明一致
- [x] 与 §4.3.4 数学推导用同一记号（stride=s, p = a·s + b）

## 不确定项
无。本图为已发表数学等价关系的可视化。
