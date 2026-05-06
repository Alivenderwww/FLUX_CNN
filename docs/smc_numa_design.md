# SMC + NUMA 真分布式架构设计 (Phase 7)

## 1. 背景

Phase 6 mesh PoC + 几轮设计反思后，确定**真正可工程化、可综合、可量产**的 NUMA
架构方向：

- 4 mem 物理独立、统一全局地址空间
- halo 列**物理只一份**（无重复存储）
- ConvCore 端 axi master 起 burst, **AXI Crossbar / SmartConnect IP 自动按地址路由**
- 多 burst 路径：driver 编译期算 cmd list, ConvCore 内 axi_dm IP **SG mode** 自动跑
- 所有路由智能在 IP 内, 自写 RTL 最少
- 跟 single-DDR 模式完全统一编程模型, 只是 DDR 拆成 4 个独立 mem

## 2. 设计哲学 (来自用户多轮 align)

> "从任何一个 Core 的视角来看, 根本就不存在什么其他 Core, 它看到的只有一大块 RAM
>  地址, 甚至包括自己的 RAM 地址也在内. 对于 halo 问题, W slice 切片成 n 块, 无论
>  是输入图像放在哪个 RAM 上, 还是输出图像存到哪个 RAM 上, 宏观上的数据就是按照
>  事先规定好的地址存在那里, **没有重叠, 没有广播, halo 区域也没有重复存放**.
>  Conv Core 拿数据的时候, 就是单纯的从地址拿数, 对于它来说不存在 halo 的问题,
>  甚至连 W slice 都可以不存在, 它算的就是一张 W/n 的图片."

> "DMA 操控完全交由软件控制. 现在 DMA 是由 cmd 控制的吧, 那就 cmd 完全交给软件
>  生成, 然后 Core 只负责把 cmd 拉到自己的核, 然后装载 cmd 到 DMA, 并推进计数就行,
>  不用管地址是什么."

> "禁止重复存储! 如果 B 不行, 那就多 burst 吧, 这也是 AXI 协议的规定."

> "能尽量用 vivado IP 就尽量用."

## 3. 架构

```
┌──────────┐
│ConvCore[0]│─ axi master (axi_dm SG mode, 自动按 cmd list 多 burst) ─┐
├──────────┤                                                          │
│ConvCore[1]│─ axi master ─────────────────────────────────────────────┤
├──────────┤                                                          ├─ AXI SmartConnect IP
│ConvCore[2]│─ axi master ─────────────────────────────────────────────┤  (4M ↔ 4S, 全连接)
├──────────┤                                                          │  按 base/high address 路由
│ConvCore[3]│─ axi master ─────────────────────────────────────────────┘
└──────────┘                                                           │
                                                                       ├─ mem[0] (axi slave)  全局 [0x0000_0000, 0x00FF_FFFF]  (16 MB)
                                                                       ├─ mem[1] (axi slave)  全局 [0x0100_0000, 0x01FF_FFFF]
                                                                       ├─ mem[2] (axi slave)  全局 [0x0200_0000, 0x02FF_FFFF]
                                                                       └─ mem[3] (axi slave)  全局 [0x0300_0000, 0x03FF_FFFF]


物理布局 (driver 编译期决定):
  整图 IFM 按 W 段散布:
    段 0 (W[0..7])   存 mem[0] 全局 [0x0000_0000+], 单份不重复
    段 1 (W[8..15])  存 mem[1] 全局 [0x0100_0000+], 单份不重复
    段 2 (W[16..23]) 存 mem[2] 全局 [0x0200_0000+], 单份不重复
    段 3 (W[24..31]) 存 mem[3] 全局 [0x0300_0000+], 单份不重复
  (halo 列物理上只有一份, e.g. W=8 只在 mem[1])


ConvCore[1] 跑 W[8..15] 段卷积时, 需要 IFM W[7..16]:
  driver 编译期生成 SG cmd list (写到 mem 某段, e.g. mem[1] 的 cmd 区):
    cmd 1: src = 0x00FF_FFE0 (mem[0] 内 W=7 位置), len = 1×cin×16 byte, dst = IFB SRAM[0]
    cmd 2: src = 0x0100_0000 (mem[1] 内 W=8 起点), len = 8×cin×16 byte, dst = IFB SRAM[1]
    cmd 3: src = 0x0200_0000 (mem[2] 内 W=16 位置), len = 1×cin×16 byte, dst = IFB SRAM[9]
  
  H 行 IFM × 3 段 = 3H 条 cmd / layer
  
  host 写 ConvCore[1].cfg_regs:
    IDMA_CMD_LIST_PTR = <cmd list 在 mem 某段的地址>
    IDMA_CMD_COUNT    = 3H
    start IDMA pulse
  
  ConvCore[1] 内 axi_dm IP (SG mode):
    自动从 IDMA_CMD_LIST_PTR 拉 cmd, 顺序起 axi master read burst
    每 burst awaddr = cmd.src, len = cmd.len
  
  AXI SmartConnect 收每个 burst:
    按 awaddr 解码到 mem[0] / mem[1] / mem[2] 各自路由
    mem 端 axi slave 服务, 数据回流 ConvCore[1] axi_dm read fifo
    ConvCore[1] axi_dm 把数据按 cmd dst 写入 IFB SRAM
  
ConvCore[1] 视角:
  "host 启 IDMA, 拉了 3H 条 cmd, 数据填满了 IFB SRAM"
  ConvCore 不知道 halo / W slice / mem 拓扑, 它看到的就是一张 sub_W × H 的 IFM


ODMA 同理:
  driver 算 OFM SG cmd list (每核 OFM 段在自己 mem):
    cmd: dst = mem[c] 内本核 OFM 段位置, len = sub_W_out × cout × 16 / 行
  ConvCore.axi_dm SG mode 自动 burst write
  SmartConnect 按 awaddr 路由 mem[c]

halo 不需要存第二份: ConvCore[1] 写 OFM 只写本核段 mem[1] 内. 下层 ConvCore[1]
的 IDMA cmd list 包含 "拉本段 + 拉邻居 mem 内的左/右 halo 列". 物理 halo 列只一份.
```

## 4. 关键 IP 升级 (Vivado 端)

### 4.1 axi_dm IP: Simple → Scatter-Gather mode

当前 `Syn/gen_axi_datamover.tcl` 配置 axi_dm IP 是 Simple mode (单 cmd 单 transfer).
需要重生成 IP 改成 SG mode:

```tcl
# Syn/gen_axi_datamover_sg.tcl
create_ip -name axi_datamover ...
set_property -dict {
    CONFIG.c_enable_mm2s {1}
    CONFIG.c_enable_s2mm {1}
    CONFIG.c_m_axi_mm2s_data_width {128}
    CONFIG.c_m_axi_s2mm_data_width {128}
    CONFIG.c_addr_width {32}
    
    # SG mode 关键参数
    CONFIG.c_mm2s_btt_used {23}                # btt 字段位宽 (cmd 内描述 transfer 长度)
    CONFIG.c_s2mm_btt_used {23}
    CONFIG.c_include_mm2s_dre {0}              # 不需要 data realignment
    CONFIG.c_include_s2mm_dre {0}
    
    # SG mode (重要)
    CONFIG.c_include_mm2s {Full}               # 跟 simple mode 一样
    CONFIG.c_include_s2mm {Full}
    # 但 cmd 接口由 driver 通过 SG descriptor list 提供, 不再是 host 一条条发
    # 实际 axi_dm 的 SG 是通过 axi cdma + axi_sg 配套 IP 实现, 而不是 axi_dm 自身一个参数
}
```

**实际上 Xilinx axi_dm 没有"SG mode"开关** — 它是 simple mode (cmd 接口由 user
逻辑驱动). SG 能力由一个独立的 IP `axi_sg` 提供, 它管理 cmd list (在 DDR 里),
按列表把 cmd 一条条喂给 axi_dm.

**升级路径**:
- 选项 A: 用 Xilinx `axi_dma` IP (替代 axi_datamover) — 它自带 SG 支持, 内部
  含 SG engine + datamover, 一个 IP 搞定
- 选项 B: 保留 axi_datamover, 加一个 cmd dispatcher unit (自写, ~200 行 SV) +
  axi master read 逻辑拉 cmd list

**选 A 更简洁** (用 Xilinx 现成 IP):
```tcl
create_ip -name axi_dma -vendor xilinx.com -library ip -version 7.1 -module_name axi_dma_sg
set_property -dict {
    CONFIG.c_include_sg {1}                 # 启 SG 模式
    CONFIG.c_sg_length_width {23}            # SG cmd 内 length 字段位宽
    CONFIG.c_sg_include_stscntrl_strm {0}    # 不需要 status/control stream
    CONFIG.c_m_axi_mm2s_data_width {128}
    CONFIG.c_m_axi_s2mm_data_width {128}
    CONFIG.c_mm2s_burst_size {16}            # max burst 16 word (256 byte)
    CONFIG.c_s2mm_burst_size {16}
    CONFIG.c_include_mm2s {1}
    CONFIG.c_include_s2mm {1}
} [get_ips axi_dma_sg]
```

### 4.2 AXI SmartConnect 4M ↔ 4S

```tcl
# Syn/gen_axi_smc_4to4.tcl
create_ip -name smartconnect ...
set_property -dict {
    CONFIG.NUM_SI {4}                        # 4 master input
    CONFIG.NUM_MI {4}                        # 4 slave output
    CONFIG.NUM_CLKS {1}                      # 单时钟域
    
    # Slave (mem) 地址区间 (跟 driver layout 一致):
    CONFIG.M00_AXI_BASEADDR {0x00000000}
    CONFIG.M00_AXI_HIGHADDR {0x00FFFFFF}     # mem[0]: 16 MB
    CONFIG.M01_AXI_BASEADDR {0x01000000}
    CONFIG.M01_AXI_HIGHADDR {0x01FFFFFF}     # mem[1]
    CONFIG.M02_AXI_BASEADDR {0x02000000}
    CONFIG.M02_AXI_HIGHADDR {0x02FFFFFF}     # mem[2]
    CONFIG.M03_AXI_BASEADDR {0x03000000}
    CONFIG.M03_AXI_HIGHADDR {0x03FFFFFF}     # mem[3]
    
    CONFIG.HAS_ARESETN {1}
    CONFIG.PROTOCOL {AXI4}
} [get_ips axi_smc_4to4]
```

## 5. SG cmd 二进制格式 (跟 axi_dma SG descriptor 对齐)

Xilinx axi_dma SG descriptor 格式 (64 byte / cmd, 在 DDR 内):
```
offset 0x00: NXTDESC      [31:0]   下一条 cmd 在 DDR 的地址 (链表)
offset 0x04: NXTDESC_MSB  [31:0]   高 32 bit (64-bit address support)
offset 0x08: BUFFER_ADDR  [31:0]   transfer 起点 byte addr
offset 0x0C: BUFFER_ADDR_MSB [31:0]
offset 0x10: reserved
offset 0x14: reserved
offset 0x18: CONTROL      [31:0]   bit[25:0] = transfer length (byte)
                                    bit[26]   = end of frame (eof)
                                    bit[27]   = start of frame (sof)
offset 0x1C: STATUS       [31:0]   写回 cmd 完成状态
offset 0x20-0x3F: APP fields (可选)
```

driver 编译期生成这种格式的 cmd list, 写到 DDR 某段, 给 ConvCore.axi_dma 起头指针.

## 6. ConvCore 内部接口改造

### 6.1 idma_ctrl.sv 改造

当前 `idma_ctrl.sv` 接 axi_datamover 的 cmd 接口 (Simple mode). 升级到 axi_dma
SG mode 需要改造:

```verilog
// 旧 (Simple mode):
module idma_ctrl (
    input  logic        cfg_idma_start,    // host 启
    input  logic [31:0] cfg_idma_src_base,  // 单 cmd 起点
    input  logic [23:0] cfg_idma_byte_len,  // 单 cmd 长度
    output ...                               // axi_dm cmd 接口
);

// 新 (SG mode):
module idma_ctrl_sg (
    input  logic        cfg_idma_start,
    input  logic [31:0] cfg_idma_cmd_list_ptr,   // SG cmd list 在 DDR 起点
    input  logic        cfg_idma_done,            // 全部 cmd 跑完
    // axi_dma SG 接口 (跟 axi_dma IP 直接对接, 不需要 user 逻辑管 cmd 拆分)
    output logic        sg_start,
    output logic [31:0] sg_curdesc_ptr,
    input  logic        sg_idle
);
```

实际上 axi_dma IP 自带 SG engine, ConvCore 端 idma_ctrl 改成 "host 配 cmd_list_ptr +
启动" 就够了, axi_dma 自动跑完.

### 6.2 cfg_regs 改造

旧:
```
IDMA_SRC_BASE  : 32-bit
IDMA_BYTE_LEN  : 24-bit
```

新:
```
IDMA_CMD_LIST_PTR  : 32-bit  (SG cmd list 起点)
IDMA_CMD_COUNT     : 16-bit  (cmd 条数, 仅作 sanity check, axi_dma 内部按 cmd.NXTDESC 链表跑)
```

ODMA / WDMA / RDMA 同理改造.

## 7. driver 改造

### 7.1 全局地址 layout

`run_multicore_chain.py` 加 `--smc` flag:

```python
SMC_MEM_BASE = [0x0000_0000, 0x0100_0000, 0x0200_0000, 0x0300_0000]
SMC_MEM_SIZE = 0x0100_0000     # 16 MB / mem

def smc_global_addr(mem_id, mem_offset):
    """全局地址 = mem ID 区间 base + mem 内偏移"""
    return SMC_MEM_BASE[mem_id] + mem_offset
```

### 7.2 W slice 段散布

driver 算每核负责的 W 段, 物理存到对应 mem (无重复):

```python
# layer 0 IFM (整图按 W 4 段散布到 4 mem)
for c in range(4):
    geom_c = compute_w_slice_geom(...)
    seg_lo = geom_c['w_in_start']        # 段起点列
    seg_hi = geom_c['w_in_start'] + geom_c['sub_W']  # 段末列 (注意此处不含 halo)
    
    # 该核段在 mem[c] 内 byte offset 0 起 (本核段紧凑存, 无 halo)
    mem_id = c
    mem_offset = 0
    
    # 不重复存储: 段在物理 mem[c] 只一份
```

### 7.3 SG cmd list 生成

per-core per-layer SG cmd list (含 halo 列拉邻居 mem):

```python
def gen_idma_sg_cmd_list(core_id, layer, ...):
    """
    生成 ConvCore[core_id] 的 IDMA SG cmd list.
    每行 IFM (含 halo) 拆成多个 cmd, 每条 cmd 在一个 mem 内的 transfer.
    """
    cmds = []
    for r in range(layer.h_in):
        # 该核该行需要的 W 列范围 (含 halo)
        w_lo = my_w_in_start - my_halo_l
        w_hi = my_w_in_end + my_halo_r
        
        # 按 W 列遍历, 找跨 mem 边界, 拆 cmd
        cur_w = w_lo
        while cur_w < w_hi:
            # 当前列在哪个 mem?
            mem_id = which_mem(cur_w)
            # 该 mem 内本行段终点 (该 mem 段的右边界)
            mem_end_w = min(w_hi, mem_seg_end(mem_id))
            
            cmds.append(SgCmd(
                src_addr = smc_global_addr(mem_id, mem_internal_offset(r, cur_w)),
                length   = (mem_end_w - cur_w) * cin_slices * 16,
                dst_sram = ifb_sram_offset(r, cur_w - w_lo),
            ))
            cur_w = mem_end_w
    
    return cmds
```

### 7.4 SG cmd list 物理布局

cmd list 写在 mem[0] 的某专用区 (driver 编译期 reserve):
```
mem[0] layout:
  [0x0000_0000, 0x0080_0000)  # 8 MB: layer 数据 (IFM/OFM 段)
  [0x0080_0000, 0x0090_0000)  # 1 MB: SG cmd list 区 (per core × per layer)
  [0x0090_0000, 0x00A0_0000)  # 1 MB: ConvCore desc list 区 (CFG_WRITE 等)
  ...
```

ConvCore[c] 的 IDMA 起点 ptr = mem[0] 的 cmd list 区某 offset, 通过 SmartConnect
路由到 mem[0] 拉 cmd.

## 8. TB 改造

`tb_smc_chain.sv` 替代 `tb_mesh_chain.sv`:

```sv
multicore_top_smc u_dut (
    .clk(clk), .rst_n(rst_n),
    .csr_*  (...),                 // host CSR (4-fanout 给 4 ConvCore)
    .done_per_core(done_per_core)
);

// 4 个 axi_slave_mem 模拟 4 mem, 由 SmartConnect 路由
// (multicore_top_smc 内部已实例化 4 mem, TB 不需要再加)

// host preload 数据到 mem (按 SMC layout):
//   layer 0 IFB 按 W 4 段切, preload 到 4 mem 不同区间
//   SG cmd list preload 到 mem[0] cmd list 区
//   ConvCore desc list preload 到 mem[0] desc list 区
```

## 9. 实施分阶段

| 阶段 | 任务 | 估时 |
|---|---|---|
| **A. IP 升级** | axi_datamover → axi_dma SG / 重生 axi_smc | 0.5-1 天 |
| **B. ConvCore 端改造** | idma_ctrl / odma_ctrl / wdma_ctrl / rdma_ctrl 接 axi_dma SG; cfg_regs 改字段 | 2-3 天 |
| **C. multicore_top_smc.sv** | 顶层骨架已有, 等 IP 实例化填充 | 0.5 天 |
| **D. driver 改造** | SG cmd list 生成 (W slice 段 + halo 跨 mem 拆 cmd); meta 输出 | 2-3 天 |
| **E. TB 改造** | tb_smc_chain.sv (跟 tb_multicore_chain 几乎一致, 改 DUT + preload layout) | 1 天 |
| **F. sim 调试** | wslice5 / ResNet11 N=4 真 PASS 验证 | 2-3 天 |
| **总计** | | **1.5-2 周** |

## 10. 验证目标

- `mesh_simple3` SMC 模式 PASS
- `mesh_block1` SMC 模式 PASS
- `mesh_wslice1` SMC 模式 PASS
- **`mesh_wslice5` SMC 模式 PASS** ← 当前 mesh push 模型 FAIL, SMC 应该真 PASS
- `mesh_resnet11_n1` SMC 模式 PASS
- **`mesh_resnet11_n4` SMC 模式 PASS, 加速比测算** ← 之前是 trivial PASS

## 11. 兼容路径

- **保留** `multicore_top.sv` (single-DDR 4-core mode), 跟 SMC 解耦
- **deprecate** `multicore_top_mesh.sv` (push 模型多核 W slice multi-layer chain
  本来就 broken, 不再继续投入)
- **新顶层** `multicore_top_smc.sv` 替代 mesh, 真分布式 NUMA 架构

## 12. NUMA / mesh 还有用吗

mesh + AXIS NoC 协议 (router_node, axis_packet_*, axi_writer_to_axis 等) 仍然作为
"未来扩展到几十核 NoC" 的硬件预留. 4-8 核场景 SmartConnect crossbar 更简洁高效;
扩展到 16-64 核时 crossbar 性价比下降, 切到 mesh 拓扑.

当前 RTL 库内的 mesh 文件保留为后续可选硬件路径, 不删除.
