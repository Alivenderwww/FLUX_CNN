# Mesh NUMA 协议设计 (Phase 6.5 重构)

## 1. 背景

**Phase 6 PoC 限制**：当前 mesh 实现是"显式路由 push 模型"——
- ConvCore 出 OFM 用 `cfg_ofm_tdest` 显式指定送哪个 mem
- IFB 由 mem 主动推送（cfg_skip_idma=1），mem 端有 desc engine
- 每个 mem 物理上"独占整图坐标空间"（broadcast 或本地段）

这导致 W slice 跨层时 halo 列在邻居 mem 中取不到（`mesh_wslice5` FAIL 768 个错），
是"模型错"不是"实现 bug"——push 模型把"路由决策"压在 mem 上，违背了 mem 应该
是"哑存储"的设计。

## 2. NUMA 模型

4 个 mem 物理独立，但**逻辑上联合编址成一块大 RAM**：

```
全局共享地址空间 (4 mem 联合, 总 64 MB):
┌─────────────────────────────────────────────────────────┐
│ awaddr/araddr 32-bit:                                   │
│   [25:24] = mem_id (路由解码)                           │
│   [23:0]  = mem 内本地偏移 (16MB / mem)                 │
└─────────────────────────────────────────────────────────┘

mem ID 0 → mem[0].ddr_mem 物理位置
mem ID 1 → mem[1].ddr_mem 物理位置
mem ID 2 → mem[2].ddr_mem 物理位置
mem ID 3 → mem[3].ddr_mem 物理位置
```

**所有 ConvCore 看到的是同一个全局地址空间**——读/写时不需要知道数据物理在哪个 mem，
mesh 路由器根据 `addr[25:24]` 自动路由 packet 到对应 mem。

## 3. Packet 协议

### 3.1 Opcode 编码 (4-bit)

| opcode | 名称 | 方向 | 说明 |
|---|---|---|---|
| 0x0 | WRITE | ConvCore → mem | 写突发 (header + N data flit) |
| 0x1 | READ_REQ | ConvCore → mem | 读请求 (header only, 0 data flit) |
| 0x2 | READ_RESP | mem → ConvCore | 读响应 (header + N data flit) |
| 其他 | 保留 | - | - |

### 3.2 Packet header flit (128-bit)

**WRITE / READ_REQ packet header**：
```
bit [127:124]  opcode      (4-bit)
bit [123:104]  addr20      (20-bit, awaddr/araddr 高 24 位中的 [23:4], mem 内 word 偏移)
bit [103:88]   burst_len   (16-bit, 数据 flit 数 = AXI awlen+1 / arlen+1)
bit [87:80]    return_id   (8-bit, READ_REQ 用: 响应送回的 mesh tdest = 发起者节点;
                             WRITE 不用, 设 0)
bit [79:0]     reserved    (80-bit, 0)
```

**READ_RESP packet header**：
```
bit [127:124]  opcode = 0x2
bit [123:104]  addr20      (复制 REQ 的 addr20, 给请求者 IDMA 对齐用)
bit [103:88]   burst_len   (复制 REQ)
bit [87:80]    req_id      (8-bit, 复制 REQ 的 return_id, 仅调试用)
bit [79:0]     reserved
```

### 3.3 Packet 时序

```
WRITE packet (axi_writer_to_axis 出):
  flit 0: header (tdest = mem ID 解码, opcode=0x0, addr20, burst_len)
  flit 1..N: data (= AXI wdata)

READ_REQ packet (axi_reader_to_axis 出):
  flit 0: header only (tdest = mem ID, opcode=0x1, addr20, burst_len, return_id=自己节点)
  (无 data flit)

READ_RESP packet (mem → ConvCore, 路由回 return_id 节点):
  flit 0: header (tdest = req.return_id, opcode=0x2, addr20, burst_len)
  flit 1..N: data (= mem.ddr_mem[addr20+i])
```

## 4. Tdest 编码 (8-bit)

跟 Phase 6 一致：
- `[7:4] = dst_y` (0 = mem 行, 1 = ConvCore 行)
- `[3:0] = dst_x` (0..3 = 列)

mem[c] @ tdest = `0x00 | c`
ConvCore[c] @ tdest = `0x10 | c`

**地址→tdest 解码** (WRITE / READ_REQ packet 由发起方算):
```
mem_id = awaddr[25:24] (or araddr[25:24])
tdest = {4'd0, mem_id[3:0]}  // dst_y=0 (mem 行), dst_x=mem_id
```

**Return tdest** (READ_RESP packet, 由 mem 算):
- mem 收 READ_REQ 时记录 header 里的 return_id
- mem 发 READ_RESP 时 packet header tdest = return_id

ConvCore[c] 发 READ_REQ 时填 return_id = `0x10 | c` (= ConvCore[c] 的 mesh 节点)。

## 5. 假设与限制

### 5.1 burst 不跨 mem 边界

ConvCore 内部 axi master (axi_dm IP) 的 burst max length = 16 word = 256 byte，
mem 边界粒度 16MB = `1<<24` byte。**任何 burst 长度不可能跨 mem 边界**——这是个
便利的物理约束。

driver 安排 IFB / OFM 段时确保**单 row 起点对齐到 mem 边界**——比如 W slice
4 核切时让每核 W 段恰好对应一个 mem，单核单行 burst 全在一个 mem 内。

halo 列读取因为 axi burst 拆分天然处理：ConvCore[1] 起 IDMA 读 W=[7..16]，axi_dm
内部把 burst 拆成 W=7 (1 word) + W=[8..16] (9 word) 两个 burst (因为 W=7 在 mem[0],
W=[8..16] 在 mem[1])，**两个 burst 各自对应一个 packet**，自然路由到对应 mem。

### 5.2 READ 响应顺序

- 同一个 burst 的所有 data flit 顺序到达 (mesh wormhole 保证 packet 内顺序)
- 不同 burst (不同 ARID) 的 response 可能乱序回来 (mesh 节点排队顺序不定)
- ConvCore 端 axi master 已经支持 ARID 区分 (axi_dm IP 处理), OK

### 5.3 反压

- READ_REQ 发出去后 mem 收到, 处理时间不固定 (mem 排队 + 内部读 ddr_mem)
- ConvCore 端 axi master read fifo (axi_dm 内部) 有一定深度 (默认 16 entries)
- 如果未完成 burst 数超过 fifo 深度, axi master 自动 stall

## 6. RTL 模块清单

### 6.1 新建模块

**`axi_reader_to_axis.sv`** (ConvCore 端发起读):
- 输入: AXI master read req 通道 (s_axi_arid/araddr/arlen/arvalid/arready)
- 输出: AXIS master (m_axis_*, READ_REQ packet, 1 flit)
- 行为: AR 握手 → 解码 araddr[25:24] → 发 1 flit READ_REQ packet → 重置等下次 AR
- cfg_return_id 输入 (= 自己的 mesh 节点 tdest)

**`axis_to_axi_read_resp.sv`** (ConvCore 端接收 READ_RESP):
- 输入: AXIS slave (s_axis_*, READ_RESP packet)
- 输出: AXI master read response 通道 (m_axi_rid/rdata/rresp/rlast/rvalid/rready)
- 行为: 收 packet header (opcode=0x2, addr20, burst_len) → 后续 N 个 data flit 按
  AXI rvalid 推到 axi master → 最后 flit 拉 rlast

**`axis_packet_read.sv`** (mem 端被读处理):
- 输入: AXIS slave (收 READ_REQ)
- 输入: mem.ddr_mem 读端口 (sram_re/raddr/rdata)
- 输出: AXIS master (发 READ_RESP)
- 行为: 收 REQ header → 解析 addr20/burst_len/return_id → 顺序读 ddr_mem
  N word → 组 RESP header + N data flit 发出

### 6.2 修改模块

**`axi_writer_to_axis.sv`**:
- 加 awaddr → tdest 解码 (`tdest = {4'd0, awaddr[25:24]}`), 取代当前的 `cfg_tdest` 单值
- packet header opcode = 0x0 (WRITE)
- 拆 burst 跨 mem (PoC 阶段假设单 burst 不跨, 不实现)

**`mem_core_stub.sv`**:
- 加 axis slave 输入端口 (收 READ_REQ packet)
- 加 axis master 输出端口 (发 READ_RESP packet) — 跟现有 m_axis 复用 (但现有
  m_axis 是 desc engine 推 IFB 用, NUMA 模式下删 desc engine 让出 m_axis)
- 删: desc engine FSM (NUMA 不需要 mem 主动推, 全部被动响应)
- 删: CSR (DESC_LIST_ADDR / DESC_COUNT / CTRL) — mem 不需要 host 配
- ddr_mem 读端口扩成双口 (axis_packet_rx 写 + axis_packet_read 读)

**`mesh_core_wrapper.sv`**:
- 当前: 只有 axi master write → axis 桥 (axi_writer_to_axis)
- 加: axi master read → axis 桥 (axi_reader_to_axis + axis_to_axi_read_resp)
- ConvCore.bus_ar/r 不再外接 (走 mesh)
- 当前 wrapper 接 mesh 1 个 LOCAL 端口, NUMA 后还是 1 个 (双向 packet 复用同一端口)

**`multicore_top_mesh.sv`**:
- 删: 4-to-1 axi master read 仲裁器 (read 走 mesh, 不再聚合到顶层 DDR)
- 删: 顶层 `bus_ar*` / `bus_r*` 输出端口 (mesh 内部完整闭环)
- 删: `mem_csr_*` 端口 + axi_lite_1to4 (mem 不需要 CSR)

**`cfg_regs.sv`**:
- 删: `r_ofm_tdest` / `r_ofm_opcode` 寄存器 (NUMA 不需要)
- 删: `r_ifb_strip_rows_host` / vld 旁路 (desc 接管)
- `cfg_skip_idma` 默认 0 (NUMA 模式 IDMA 启用)

### 6.3 driver 改动

**`run_multicore_chain.py` mesh 模式**:
- 不再强制 `skip_idma=True` (NUMA 用 IDMA 拉)
- DDR layout: 用全局编址 (按 W slice 段切到 mem ID)
- 删 mem cmd list 生成 (mem 不需要 cmd)
- 删 mesh_n_cores meta + W_OUT_START / MY_W_OUT meta (回归正常单 DDR 模式 layout)

**`mesh_cmd.py`**:
- 整个文件可以删 (NUMA 不需要 mem cmd)

### 6.4 TB 改动

**`tb_mesh_chain.sv`**:
- 删: mem desc preload + mem_engine_start / wait
- 改成跟 `tb_multicore_chain.sv` 几乎一样: host 配 ConvCore desc → start_layer →
  ConvCore 自己 IDMA 拉 IFB / ODMA 写 OFM (走 mesh)
- 删: broadcast preload IFB 到 4 mem
- 改成: 按全局编址 preload (ResNet11 layer 0 IFB 整图按 W 段散布到 4 mem
  各自 ddr_mem 的物理位置, 跟 NUMA 地址映射对齐)

## 7. 验证目标

1. `mesh_simple3` (n=1, 3 layer): NUMA 模式 PASS
2. `mesh_block1` (n=1, 3 layer + residual): PASS
3. `mesh_wslice1` (n=4, 1 layer W slice): PASS
4. `mesh_wslice5` (n=4, **5 层 W slice chain**): **PASS** ← 当前 push 模型 FAIL,
   NUMA 模式应自然解决 cross-W-slice halo
5. `mesh_resnet11_n1`: PASS
6. `mesh_resnet11_n4`: PASS, 加速比应跟 push 模型 + force_multicore (2.96×) 接近
   (NUMA 协议 overhead 比 push 略多 1 拍 packet header 但少了 mem cmd push 串行)
