# 图 1.1 端侧 CNN 推理应用场景全景
# Figure 1.1 Application landscape of edge-side CNN inference

## 在论文中的角色
- 首次引入：§1.1 研究背景与意义（paper.md L93 / L97）
- 引用位置：paper.md L93 / L97
- 论证作用：在绪论开篇用一张"四象限场景图"建立读者对端侧 CNN 推理的共同图景，强调四类典型场景共享"高带宽数据流—边缘处理—低时延响应"特征。读者看到此图后应理解 FLUX_CNN 的目标部署形态，并对"端侧"与"云端"的本质差异（功耗、时延、带宽预算）形成直观印象。

## 图类型
四象限全景示意图（中央芯片节点 + 四角应用场景）。

## 设计要素

### 必含元素
1. **中央节点**：FLUX_CNN 加速器芯片符号（圆角矩形 + 内部 16×16 网格点示意 MAC 阵列），标注"FLUX_CNN 端侧加速器"。
2. **四象限应用场景**（围绕中央节点四角放置）：
   - **左上 — 智慧城市视频监控**：摄像头 + 1080p 视频流图标，标"目标检测 / 车牌识别"，延时预算"< 50 ms / 帧"。
   - **右上 — 工业视觉缺陷检测**：传送带 + 工件 + 相机示意，标"像素级缺陷分割"，延时预算"零帧丢失 / 实时分割"。
   - **左下 — 消费电子手势识别**：手部轮廓 + 麦克风符号，标"手势 / 关键词唤醒"，功耗预算"< 1 W 常开"。
   - **右下 — 自动驾驶辅助感知**：车辆 + 路面分割示意，标"前向感知 / 车道线"，延时预算"端到端 < 20 ms"。
3. **四条连接线**：中央 FLUX_CNN 节点向四象限各画一条带箭头的引线（双向：原始数据流入、推理结果回流）。
4. **共同特征标语**：图底部居中一行"高带宽数据流 — 边缘处理 — 低时延响应"。

### 标注要求
- 每象限上方一行场景中文名 + 英文名（小字）
- 每象限内中部一行典型算子（如"YOLOv5n / ResNet"）
- 每象限下方一行延时 / 功耗预算
- 中央 FLUX_CNN 节点下方一行"16×16 INT8 MAC, 1×AXI4 + 1×AXI-Lite"

### 视觉层次
- 主角：中央 FLUX_CNN 节点 + 四象限场景框
- 配角：四条引线 + 共同特征标语
- 背景：无背景纹理，纯白底

## 数据来源
- paper.md §1.1 L85-L93（四类典型场景列举与延时预算描述）
- 文献 [1][2][3][4] — 监控 / 工业视觉 / 消费电子 / 自动驾驶
- contributions.md §一句话定位（端侧定位陈述）

## ASCII 示意稿

```
   ┌────────────────────┐                ┌────────────────────┐
   │ 智慧城市视频监控    │                │ 工业视觉缺陷检测   │
   │ (Smart-city video) │                │ (Industrial vision)│
   │ ▓▓▓ 1080p stream   │                │ ▓ 传送带 + 相机    │
   │ YOLOv5n / ResNet   │                │ U-Net 像素分割     │
   │ < 50 ms / frame    │                │ 零帧丢失 / 实时    │
   └─────────┬──────────┘                └─────────┬──────────┘
             │                                     │
             │       ┌───────────────────────┐     │
             └──────▶│   FLUX_CNN 加速器     │◀────┘
                     │ ┌───────────────────┐ │
                     │ │ 16×16 INT8 MAC    │ │
                     │ │ • • • • • • • • • │ │
                     │ │ • • • • • • • • • │ │
                     │ └───────────────────┘ │
                     │ 1×AXI4 + 1×AXI-Lite   │
                     └─────────┬───────────┬─┘
             ┌─────────────────┘           └────────────────┐
             ▼                                              ▼
   ┌────────────────────┐                ┌────────────────────┐
   │ 消费电子手势识别    │                │ 自动驾驶辅助感知   │
   │ (Consumer gesture) │                │ (ADAS perception)  │
   │ ✋ 手势 + 麦克风   │                │ 🚗 前向感知 / 车道  │
   │ MobileNet KWS      │                │ ResNet / YOLO      │
   │ 功耗 < 1 W 常开    │                │ 端到端 < 20 ms     │
   └────────────────────┘                └────────────────────┘

        共同特征：高带宽数据流 — 边缘处理 — 低时延响应
```

## 与正文的一致性检查
- [x] §1.1 "智慧城市的视频监控、工业视觉缺陷检测、消费电子中的手势识别与关键词唤醒、自动驾驶辅助感知" — 四象限对齐四类
- [x] §1.1 "单台设备的推理时延普遍要求控制在 50 ms 以内" — 左上象限延时标注
- [x] §1.1 "高带宽数据流—边缘处理—低时延响应" — 底部共同特征标语
- [x] §1.1 "电池供电的可穿戴设备甚至要求亚瓦级常驻功耗" — 左下象限功耗标注

## 不确定项
- [TBD: 是否在每象限中加上典型 FPS 数字（如视频监控 30 fps）] — 倾向不加，避免数据准确性争议；延时预算更通用
- [TBD: 中央 FLUX_CNN 节点是否画成芯片俯视图（die photo 风格）还是抽象矩形] — 倾向抽象矩形，与后续 §3 架构图风格统一

## image 生成提示词

### 中文版

科研论文配图，**端侧 CNN 推理应用场景全景图**，IEEE 期刊配色风格，白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1:1（正方形）或 1:1.2（高略大于宽），避免整体过于竖向（高:宽 > 1.5:1）或过于横向（宽:高 > 1.5:1），以适合 A4 单栏插图。**

**版式（四象限围绕中央节点）**：
- **中央节点**：画面正中绘制一个圆角矩形（淡黄色 CMYK 0/10/30/0 填充、深灰色 #404040 实线边框），尺寸约占画面 25%×25%。内部上半区域绘制一个 16×16 的小点阵（深蓝色 #1f4e79 小圆点示意 MAC 阵列），下半区域写两行文字：第一行"FLUX_CNN 端侧加速器"（思源黑体 10 pt 加粗），第二行"16×16 INT8 MAC, 1×AXI4 + 1×AXI-Lite"（Times New Roman 8 pt）。
- **四象限场景框**：在画面左上、右上、左下、右下四个角各放一个圆角矩形（淡灰色 CMYK 0/0/0/10 填充、深灰色边框），尺寸约占画面 30%×25%。每框分三行：第一行场景中文名（思源黑体 10 pt 加粗）+ 英文名（Times New Roman 8 pt 斜体）；第二行典型算子（如"YOLOv5n / ResNet"）；第三行延时或功耗预算（小字斜体）。
  - 左上："智慧城市视频监控 (Smart-city video surveillance) / YOLOv5n / ResNet / < 50 ms per frame"
  - 右上："工业视觉缺陷检测 (Industrial defect inspection) / U-Net / zero frame drop, real-time"
  - 左下："消费电子手势识别 (Consumer gesture & KWS) / MobileNet / always-on < 1 W"
  - 右下："自动驾驶辅助感知 (ADAS perception) / ResNet / YOLO / end-to-end < 20 ms"
- **连接线**：从中央节点到四象限各画一条深灰色 #404040 细实线（带双向小箭头），代表数据流入与结果回流。
- **底部标语**：画面下方居中一行小字（Times New Roman 9 pt 斜体）"Common pattern: high-bandwidth stream — edge processing — low-latency response"，中文版本上方再加一行"共同特征：高带宽数据流 — 边缘处理 — 低时延响应"（思源黑体 9 pt）。

**字体**：所有英文 Times New Roman 10 pt，标题加粗；中文思源黑体 10 pt。**严肃学术风格，IEEE 期刊配色，禁止卡通**：禁止手写体、彩色渐变、阴影、3D 立体、写实图标（如真实摄像头照片、卡通车辆）、emoji。可使用极简线条勾勒的几何符号代替图标。整体保持工科论文严谨风格。

### English version

Scientific paper figure, **Application landscape of edge-side CNN inference**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **The overall figure aspect ratio should be approximately 1:1 (square) or 1:1.2 (slightly taller than wide). Avoid overly tall layouts (height:width > 1.5:1) or overly wide layouts (width:height > 1.5:1), to fit a single-column A4 figure.**

**Layout (four quadrants around a central node)**:
- **Central node**: Place a rounded rectangle (light-yellow CMYK 0/10/30/0 fill, dark-gray #404040 solid border) at the center, sized roughly 25%×25% of the canvas. Inside the upper half draw a 16×16 dot-grid (deep-blue #1f4e79 small circles representing the MAC array). The lower half holds two text lines: line 1 "FLUX_CNN edge accelerator" (Source Han Sans 10 pt bold), line 2 "16×16 INT8 MAC, 1×AXI4 + 1×AXI-Lite" (Times New Roman 8 pt).
- **Four quadrant scene boxes**: At the top-left, top-right, bottom-left, and bottom-right corners, place rounded rectangles (light-gray CMYK 0/0/0/10 fill, dark-gray border), each about 30%×25% in size. Each box contains three lines: scene name in Chinese (Source Han Sans 10 pt bold) + English (Times New Roman 8 pt italic); typical operator; latency / power budget (small italic).
  - Top-left: "Smart-city video surveillance / YOLOv5n, ResNet / < 50 ms per frame"
  - Top-right: "Industrial defect inspection / U-Net / zero frame drop, real-time"
  - Bottom-left: "Consumer gesture & KWS / MobileNet / always-on < 1 W"
  - Bottom-right: "ADAS perception / ResNet, YOLO / end-to-end < 20 ms"
- **Connecting lines**: From the central node to each quadrant draw thin dark-gray #404040 solid lines with bidirectional small arrowheads, representing raw data flowing in and inference results flowing out.
- **Bottom caption**: A single italic line at the bottom (Times New Roman 9 pt) "Common pattern: high-bandwidth stream — edge processing — low-latency response", with a Chinese counterpart above it (Source Han Sans 9 pt).

**Typography**: All English in Times New Roman 10 pt with bold titles; Chinese in Source Han Sans 10 pt. **Serious academic style, IEEE-journal palette, no cartoon**: strictly forbid handwritten fonts, color gradients, drop shadows, 3D effects, photorealistic icons (e.g., real camera or vehicle photographs), or emoji. Use minimal-line geometric symbols when a glyph is needed. Maintain rigorous engineering-paper aesthetics throughout.
