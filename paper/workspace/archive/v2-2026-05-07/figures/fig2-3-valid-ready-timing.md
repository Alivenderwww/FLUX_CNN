# 图 2.3 valid-ready 握手协议时序示意
# Figure 2.3 Timing diagram of the valid-ready handshake protocol

## 在论文中的角色
- 首次引入：§2.4 段 "该协议天然支持上下游模块的弹性 join，无需中心 FSM..." [依赖: Fig.2.3]
- 论证作用：用波形图直观展示 valid 与 ready 同时为高才完成数据移交的语义，并展示三种典型情况（消费方 ready 拉低导致 stall / 生产方 valid 拉低导致气泡 / 双方握手成功），为 §4.2 的 5 模块去中心化握手流水铺垫。

## 图类型
四信号波形图（CLK / VALID / READY / DATA），含 4 ~ 5 拍时序，并在 DATA 行上标注每拍的有效 token。

## 设计要素

### 必含元素
- 信号：CLK（方波）、VALID、READY、DATA[7:0]（或 DATA 标抽象记号 D0/D1/D2）
- 至少 5 拍，包含三种情况：
  - 拍 T1：VALID=1, READY=1 → 握手成功，DATA=D0 被消费
  - 拍 T2：VALID=1, READY=0 → 生产方等待，DATA=D1 hold（消费方未 ready，stall）
  - 拍 T3：VALID=1, READY=1 → D1 终于被消费
  - 拍 T4：VALID=0, READY=1 → 生产方无数据，气泡，DATA 无效（X 或 don't care）
  - 拍 T5：VALID=1, READY=1 → D2 被消费
- 每个握手成功的拍下方加一个绿色对勾 "✓ accepted"
- stall / 气泡拍下方加注释 "stall (consumer not ready)" / "bubble (no data)"

### 标注要求
- 图下方加一句规则："握手协议规则：仅当 VALID 与 READY 同时为高时一拍数据被实际消费；其余情况下 DATA hold（生产方）或被忽略（消费方）"
- 用红色虚线竖线标记每一拍 CLK 上升沿
- DATA 行的 hold 状态用阴影框，新数据用实色框

### 视觉层次
- 主角：VALID / READY 信号（粗线）
- 配角：CLK / DATA
- 背景：注释标签

## ASCII 示意稿

```
        T1      T2      T3      T4      T5
        │       │       │       │       │
CLK   __│ ┌──┐  │ ┌──┐  │ ┌──┐  │ ┌──┐  │ ┌──┐
        │ │  │  │ │  │  │ │  │  │ │  │  │ │  │
______│ └──┘___│ └──┘___│ └──┘___│ └──┘___│ └──┘___

VALID ─────────────────────────────┐ ┌───────────────
                                   └─┘
                          (T4: 生产方暂无数据 → 气泡)

READY ──────┐ ┌────────────────────────────────────
            └─┘
            (T2: 消费方还没准备好 → stall)

DATA  ──< D0 >──< D1 hold ── D1 >──<  X  >──< D2 >──
         ✓        stall      ✓     bubble    ✓
       accepted             accepted        accepted

握手规则：VALID ∧ READY → 该拍 DATA 被消费；
         否则 DATA 在生产方 hold，消费方忽略。
```

## 数据来源
- AXI4 valid-ready 协议规范（ARM AMBA AXI4 Specification）
- paper.md §2.4 段、§4.2.1 段（5 模块去中心化流水）
- contributions.md C1.2

## 与正文一致性检查
- [x] "仅当 VALID 与 READY 同时为高时一拍数据被实际消费"——与 §2.4 表述一致
- [x] "弹性 join、气泡传递"——T2 stall 与 T4 bubble 均图示
- [x] 与 §4.2.1 5 模块去中心化握手流水所依赖的协议一致

## 不确定项
无。
