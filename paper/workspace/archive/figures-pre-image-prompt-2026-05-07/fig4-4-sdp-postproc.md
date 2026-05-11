# 图 4.4 SDP 后处理模块结构图
# Figure 4.4 SDP post-processing module structure

## 在论文中的角色
- 首次引入：§4.5 SDP 后处理模块
- 引用位置：paper.md 第 324、326 行
- 论证作用：展示 ofb_writer 内部 5 步处理链 "PSUM → bias add → shift → saturate → residual fusion → saturate → OFB write"，强调"INT8 量化推理后处理一站式完成"，无需主机介入。

## 图类型
数据流图（线性 5 阶段流水）。

## 设计要素

### 必含元素
1. **输入 PSUM 32-bit**（左）：从 parf_accum 来。
2. **bias_rf 寄存器堆**：上方侧路，标 "bias_rf (32-bit per cout)"，由 RDMA 预加载。
3. **bias add**：第 1 阶段加法器。
4. **shift（量化移位）**：第 2 阶段算术右移，移位量来自 cfg_regs。
5. **saturate to INT8**：第 3 阶段饱和裁剪到 \[−128, 127\]。
6. **Shortcut Bank 残差缓冲**：上方另一侧路，标容量与 §5.4.1 缩减后一致（2048 word / N=4 配置），由上一层 OFM 直接写入。
7. **residual scale + add**：第 4 阶段，按 shortcut_mult / shortcut_shift 缩放后相加。
8. **saturate again**：第 5 阶段第二次饱和。
9. **OFB ring**：右侧环形缓冲（2048 word），按 (*y_out*, *x_out*, *c_out*) 写入。
10. **row credit 反向控制流**：OFB → ODMA → ofb_writer 的虚线。

### 标注要求
- 各阶段中间数据位宽：bias 32b → shift 后 INT8、residual 后再饱和 INT8
- "all stages currently combinational in single stage" — 标 "single-stage combinational" + "future: insert pipeline reg → see §5.6"
- OFB ring 容量 2048 word
- bias_rf 由 RDMA 拉入；Shortcut Bank 由上一层 OFM push

### 视觉层次
- 主角：5 阶段链条
- 配角：bias_rf、Shortcut Bank、OFB ring
- 背景：cfg 输入（shift_amt、sat_min/sat_max、shortcut_mult/shift）

## 数据来源
- paper.md §4.5
- docs/modules/ofb_writer.md
- contributions.md C1.x SDP 后处理
- STATUS.md §1（SDP 量化链 WNS=-4.618 ns，关键路径过长）

## ASCII 示意稿

```
   from parf_accum                  bias_rf               Shortcut Bank
   16× PSUM (32b)                 (32b/cout)              (INT8/spatial)
        │                              │                       │
        │                              │ bias                  │ residual
        ▼                              ▼                       ▼
   ┌─────────────────────────────────────────────────────────────────┐
   │ ofb_writer (SDP — Scalar Data Path)                             │
   │                                                                 │
   │   ┌──────┐    ┌─────────┐    ┌──────────┐    ┌──────────┐       │
   │   │ +bias│ ─▶ │  shift  │ ─▶ │ saturate │ ─▶ │ +residual│       │
   │   │ (32b)│    │ (>>)    │    │ → INT8   │    │ (scaled) │       │
   │   └──────┘    └─────────┘    └──────────┘    └─────┬────┘       │
   │      [stage 1]   [stage 2]    [stage 3]            │            │
   │                                                    ▼            │
   │                                            ┌──────────────┐     │
   │                                            │ saturate ag. │     │
   │                                            │ → INT8       │     │
   │                                            └──────┬───────┘     │
   │                                                   │ [stage 5]   │
   │   (currently single-stage combinational ─                       │
   │    relates to F_max bottleneck per §5.4.2)                      │
   │                                                   │             │
   │                                                   ▼             │
   │                                          ┌────────────────┐     │
   │                                          │ OFB ring       │     │
   │                                          │ (2048 word)    │     │
   │                                          └────────┬───────┘     │
   │                                                   │             │
   │   row_credit  ◀───── from ODMA (back-pressure)    │             │
   └───────────────────────────────────────────────────┼─────────────┘
                                                       │ INT8 row
                                                       ▼
                                                    to ODMA
```

## 与正文的一致性检查
- [x] §4.5 五步流程顺序与图一致："偏置加 → 移位 → 饱和 → 残差 → 饱和 → OFB"
- [x] §4.5 "目前在单 stage 内组合实现，未做 stage 切分" — 图注 "currently single-stage combinational"
- [x] 与 §5.4.2 Fmax 限制因素一致（SDP 量化链关键路径）

## 不确定项
- [TBD: 残差融合在某些层不启用 — 是否在图中画 mux bypass] — 倾向画一条 bypass 虚线，由 cfg 选通
