# 图 4.1 行缓存模块结构图
# Figure 4.1 Line-buffer module structure

## 在论文中的角色
- 首次引入：§4.2 行缓存模块
- 引用位置：paper.md 第 282、284 行
- 论证作用：展示"片外像素流→片上 *K*×*K* 窗口"转换的具体硬件结构。读者看完图后应理解 IFB ring buffer + 行写入 / 窗口读取 / 行释放三步流程 + row credit 反压机制。

## 图类型
模块结构图（含数据通路 + 反压通路）。

## 设计要素

### 必含元素
1. **IFB SRAM**（中央）：标注容量 8192 word；按行组织为环形缓冲，画 *strip_rows* 行的 ring 结构（典型 4-8 行示意）。
2. **行写指针 wptr**：环形 ring 上一个箭头沿圆周走动。
3. **行读指针 / 窗口生成器**：从 ring 中按 (*y_out*, *x_out*) 计数读出 *K*×*K* 窗口。
4. **ARF（Activation Register File）=32**：在窗口生成器右侧的小寄存器堆，缓存当前窗口供 PE 重复读取。
5. **本地计数器**：(*y_out*, *x_out*, *k_x*, *k_y*) 4 级，画在模块左下角。
6. **2 路 valid-ready 接口**：上游 idma_ctrl 写口，下游 mac_array 窗口口。
7. **row credit 反向控制流**：从 line_buffer → idma_ctrl 的虚线箭头，标 "row_credit"。

### 标注要求
- 模块上方：cfg 输入（*H_in*, *W_in*, *K*, stride, pad, strip_rows, cin_slice）
- ring 的容量约束：*strip_rows* × *W_in* ≤ IFB
- forward-pressure 发射条件：rows_available ≥ *y_out* × stride + *K_y*

### 视觉层次
- 主角：IFB ring + 窗口生成器
- 配角：ARF、本地计数器
- 背景：row credit 反压线

## 数据来源
- paper.md §4.2
- docs/modules/line_buffer.md
- contributions.md C1.3 "任意 H×W 输入" 创新点

## ASCII 示意稿

```
   from idma_ctrl                                    cfg_regs
     │ INT8 row                                          │
     │ v/r                                               │ H_in, W_in,
     ▼                                                   │ K, stride, pad
  ┌────────────────────────────────────────────┐         │
  │ line_buffer                                │ ◀───────┘
  │                                            │
  │   ┌───────────── IFB ring ────────────┐    │
  │   │  row 0 ──┐                        │    │
  │   │  row 1   │  ↻ wptr (write)        │    │
  │   │  row 2   │                        │    │
  │   │   ...    │  ↻ rptr (read)         │    │
  │   │ row K-1 ─┘                        │    │
  │   │ (8192 word total, strip_rows × W) │    │
  │   └────────┬───────────────────────────┘   │
  │            │                                │
  │            │ K×K window slice               │
  │            ▼                                │
  │     ┌─────────────┐                         │
  │     │  ARF (32)   │                         │
  │     │  hold cur.  │                         │
  │     │  K×K window │                         │
  │     └──────┬──────┘                         │
  │            │ 16-elem activation vec (cin)   │
  │            │ v/r                            │
  │   ┌────────┴────────┐                       │
  │   │ local counters  │  (y_out, x_out,       │
  │   │  4-level        │   k_x, k_y)           │
  │   └─────────────────┘                       │
  │                                             │
  │   row_credit  ─── back-pressure ────┐       │
  └─────────────────────────────────────┼───────┘
                                        │
                                        ▼
                                  to idma_ctrl (release row)

                                        │
                                        │ act vector to mac_array
                                        ▼
```

## 与正文的一致性检查
- [x] §4.2 三步流程（行写入 / 窗口读取 / 行释放）与图一致
- [x] forward-pressure 发射条件以图注形式给出
- [x] 与 §3.3 图 3.2 中 line_buffer 块对齐

## 不确定项
- [TBD: 是否在 ring 上明确标 K=3 / K=7 两种典型 strip_rows] — 倾向不标，保持图通用，仅在图注里说明 "strip_rows 由编译器派生"
