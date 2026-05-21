"""
mesh_cmd.py  --  SMC SG cmd list 生成器 (历史命名 mesh_cmd, 现仅 SMC + NUMA 用)

设计目标 (Phase 7 SMC + NUMA, 2026-03):
  - 4 ConvCore + 4 mem + axi_smc_4to4 (axi_crossbar IP) + 全局地址 mem_id 路由
  - driver 编译期生成每核每层 IDMA / ODMA SG cmd list, 跨 mem 边界 axi_crossbar IP 路由
  - cmd list .hex 文件 $readmemh 兼容, mem 加载到 SMC_*_CMD_BASE 区, dispatcher 拉

提供:
  - SgCmd / OdmaSgCmd dataclass + to_hex_lines() 编码
  - gen_idma_sg_cmd_list_w_slice / gen_odma_sg_cmd_list_w_slice 高层 cmd 生成器
  - compute_smc_w_segments W slice 4 等分公平切片
  - write_sg_cmd_list / write_odma_sg_cmd_list .hex 文件 writer

历史: 早期 mesh / 4-DDR PoC 也住在这个文件里, 已删除.
"""

from __future__ import annotations
from dataclasses import dataclass


@dataclass
class SgCmd:
    """[Phase 7 SMC + NUMA] IDMA SG cmd, 描述一次 axi master read burst.

    跟 idma_sg_dispatcher.sv 内部解码格式对齐 (32 byte = 2 beats × 128-bit):
      word 0:  src_addr     [31:0]   transfer 起点 byte (全局地址)
      word 1:  btt          [22:0]   transfer 长度 byte
               last_cmd     [23]     1 = list 末尾
               reserved     [31:24]  0
      word 2:  sram_offset  [31:0]   IFB SRAM 写起点 (word offset)
      word 3:  reserved
    """
    src_addr:    int        # 全局地址 byte (跨 mem 边界由 axi_smc_4to4 IP 路由)
    btt:         int        # transfer 长度 byte (单 burst, 不跨 axi slave)
    sram_offset: int        # IFB SRAM 内 word offset
    last_cmd:    bool = False
    name:        str = ""

    def to_hex_lines(self) -> list:
        """编码成 2 行 hex (每行 16 byte = 32 hex char), 跟 $readmemh 兼容."""
        word0 = self.src_addr & 0xFFFFFFFF
        word1 = (self.btt & 0x7FFFFF) | ((1 << 23) if self.last_cmd else 0)
        word2 = self.sram_offset & 0xFFFFFFFF
        word3 = 0
        beat0 = (word0) | (word1 << 32) | (word2 << 64) | (word3 << 96)
        beat1 = 0    # reserved 16 byte
        return [f"{beat0:032x}", f"{beat1:032x}"]


def gen_idma_sg_cmd_list_w_slice(
    *,
    target_core_id:    int,
    h_in:              int,
    cin_slices:        int,
    w_in_lo:           int,
    w_in_hi:           int,
    seg_w_starts:      list,
    seg_mem_bases:     list,
    seg_widths:        list,
    ifb_ring_words:    int = 0,
    h_compress_stride: int = 1,
    w_compress_stride: int = 1,
    local_seg_first:   bool = False,
) -> list:
    """生成 ConvCore[c] 的 IDMA SG cmd list (W slice 紧凑 + 跨 mem halo 拉取).

    每行 IFM 拆成多条 cmd: 每条在一个 mem 内的连续 burst.
    跨 mem 边界时多条 cmd. 物理 halo 列只一份 (driver 编译期确保 layout 无重复).

    seg_* 数组描述整图 W 段散布: 段 i 在整图 W 列 [seg_w_starts[i], seg_w_starts[i]+seg_widths[i]),
    物理在 mem[i].ddr_mem 内全局地址 seg_mem_bases[i] 起.

    Round I 优化: h_compress_stride > 1 时只发 stride 行 cmd (跳过 K=1 stride>1 layer
    用不到的奇数行). 此时 cmd 数 = h_in / h_compress_stride. 调用方需保证 h_in
    跟 stride 整除 (= layer 实际拉的行数, 通常 = H_out).
    src_addr 内 r 直接用 cmd index, 但每 cmd 跨度 = h_compress_stride × row_stride.

    Round J 探针: w_compress_stride > 1 时每 cmd 只读 1 像素 + cur_w 跳 w_compress_stride.
    cmd 数 H 维不变, W 维从 chunk_cols → ceil(chunk_cols / w_compress_stride). 调用方
    保证 w_in_lo 是 W 压缩后的"采样起点". axi_dm 不支持 strided burst, 所以单像素
    cmd 是唯一拿到数据量减半的方式 (代价: cmd 数膨胀 sub_W_out 倍, 测 ROI 用).

    Round K: local_seg_first=True 时每行内 cmd 重排 — 本地 mem 段 (seg_id==target_core_id)
    cmd 排前, 远端 (跨 mem halo) cmd 排后. sram_offset 仍按 col 顺序 (不影响 IFB layout).
    意图: 让 4 核同时 hammer 4 不同 mem (并行), 远端 halo 后发避开 mem 0 head-of-line.
    dispatcher r_rows_pushed 按 cmds_per_row 计 cmd 完成数, 跟 cmd issue 顺序无关.
    """
    n_segs = len(seg_w_starts)
    assert len(seg_mem_bases) == n_segs and len(seg_widths) == n_segs

    cmds = []
    sram_offset = 0
    for r in range(h_in):
        # 第一遍按 col 顺序生成本行 cmd (含 seg_id), sram_offset 按 col 顺序累加
        row_cmds = []
        cur_w = w_in_lo
        while cur_w <= w_in_hi:
            seg_id = -1
            for s in range(n_segs):
                if seg_w_starts[s] <= cur_w < seg_w_starts[s] + seg_widths[s]:
                    seg_id = s
                    break
            if seg_id < 0:
                raise ValueError(f"col w={cur_w} not in any segment")

            seg_w_end = seg_w_starts[seg_id] + seg_widths[seg_id] - 1
            chunk_lo = cur_w
            if w_compress_stride > 1:
                # Round J 探针: 单像素 cmd (chunk_cols=1), cur_w 跳 w_compress_stride
                chunk_hi = chunk_lo
                chunk_cols = 1
            else:
                chunk_hi = min(w_in_hi, seg_w_end)
                chunk_cols = chunk_hi - chunk_lo + 1

            seg_w = seg_widths[seg_id]
            w_local_lo = chunk_lo - seg_w_starts[seg_id]
            # Round I: h_compress_stride > 1 时, 每 cmd src_addr 跳 stride 行
            byte_addr = (seg_mem_bases[seg_id]
                         + r * h_compress_stride * seg_w * cin_slices * 16
                         + w_local_lo * cin_slices * 16)
            chunk_btt = chunk_cols * cin_slices * 16

            sram_offset_eff = (sram_offset % ifb_ring_words) if ifb_ring_words > 0 else sram_offset
            row_cmds.append((seg_id, SgCmd(
                src_addr    = byte_addr,
                btt         = chunk_btt,
                sram_offset = sram_offset_eff,
                last_cmd    = False,
                name        = f"r{r}_w[{chunk_lo}:{chunk_hi+1}]_seg{seg_id}",
            )))
            sram_offset += chunk_cols * cin_slices
            if w_compress_stride > 1:
                cur_w = chunk_hi + w_compress_stride
            else:
                cur_w = chunk_hi + 1

        # Round K: 行内 cmd 重排 (本地 mem 段优先). sram_offset 已固定 (按 col 顺序),
        # 只改 issue 顺序. dispatcher 按 cmds_per_row 计完成数, 跟 issue 顺序无关.
        if local_seg_first and len(row_cmds) > 1:
            row_cmds.sort(key=lambda x: 0 if x[0] == target_core_id else 1)
        for _, c in row_cmds:
            cmds.append(c)

    if cmds:
        cmds[-1].last_cmd = True
    return cmds


def write_sg_cmd_list(out_path: str, cmds: list, header_lines: list = None) -> int:
    """写 IDMA SG cmd list 到 .hex 文件 (每条 cmd 2 行, $readmemh 兼容)."""
    with open(out_path, 'w') as f:
        f.write("// IDMA SG cmd list ($readmemh 32-byte/cmd)\n")
        if header_lines:
            for line in header_lines:
                f.write(f"// {line}\n")
        f.write(f"// n_cmds = {len(cmds)}\n")
        for c in cmds:
            for hex_line in c.to_hex_lines():
                f.write(hex_line)
                if c.name and hex_line == c.to_hex_lines()[0]:
                    f.write(f"  // {c.name}{' [LAST]' if c.last_cmd else ''}")
                f.write("\n")
    return len(cmds)


@dataclass
class OdmaSgCmd:
    """[Phase 7 SMC + NUMA] ODMA SG cmd, 跟 odma_sg_dispatcher.sv 解码格式对齐
    (32 byte = 2 beats × 128-bit, 仅第 1 beat 有效, 16 byte):
      word 0:  dst_addr     [31:0]   目标全局地址 byte
      word 1:  btt          [22:0]   transfer 长度 byte (= sub_W × cout_slices × 16)
               last_cmd     [23]
               reserved     [31:24]
      word 2:  ofb_w_start  [15:0]   该段在 OFB SRAM 的起始 W 列 (NHWC gather 用)
               reserved     [31:16]
      word 3:  reserved
    """
    dst_addr:    int
    btt:         int
    ofb_w_start: int
    last_cmd:    bool = False
    name:        str = ""

    def to_hex_lines(self) -> list:
        word0 = self.dst_addr & 0xFFFFFFFF
        word1 = (self.btt & 0x7FFFFF) | ((1 << 23) if self.last_cmd else 0)
        word2 = self.ofb_w_start & 0xFFFF
        word3 = 0
        beat0 = (word0) | (word1 << 32) | (word2 << 64) | (word3 << 96)
        beat1 = 0
        return [f"{beat0:032x}", f"{beat1:032x}"]


def gen_odma_sg_cmd_list_w_slice(
    *,
    h_out:             int,
    cout_slices:       int,
    w_out_lo:          int,
    w_out_hi:          int,
    seg_w_starts:      list,
    seg_mem_bases:     list,
    seg_widths:        list,
    ofb_w_offset_in_core: int = 0,
) -> list:
    """生成 ConvCore[c] 的 ODMA SG cmd list (W slice 紧凑写到目标 mem 段).

    本核 OFM W 列范围 [w_out_lo, w_out_hi]. 跨整图 W 段时拆多条 cmd,
    每条在一个 mem 内的连续 burst write.
    """
    n_segs = len(seg_w_starts)
    assert len(seg_mem_bases) == n_segs and len(seg_widths) == n_segs

    cmds = []
    for r in range(h_out):
        cur_w = w_out_lo
        while cur_w <= w_out_hi:
            seg_id = -1
            for s in range(n_segs):
                if seg_w_starts[s] <= cur_w < seg_w_starts[s] + seg_widths[s]:
                    seg_id = s
                    break
            if seg_id < 0:
                raise ValueError(f"col w={cur_w} not in any OFM segment")

            seg_w_end = seg_w_starts[seg_id] + seg_widths[seg_id] - 1
            chunk_lo = cur_w
            chunk_hi = min(w_out_hi, seg_w_end)
            chunk_cols = chunk_hi - chunk_lo + 1

            seg_w = seg_widths[seg_id]
            w_local_lo = chunk_lo - seg_w_starts[seg_id]
            byte_addr = (seg_mem_bases[seg_id]
                         + r * seg_w * cout_slices * 16
                         + w_local_lo * cout_slices * 16)
            chunk_btt = chunk_cols * cout_slices * 16

            ofb_w_start_in_core = ofb_w_offset_in_core + (chunk_lo - w_out_lo)

            cmds.append(OdmaSgCmd(
                dst_addr    = byte_addr,
                btt         = chunk_btt,
                ofb_w_start = ofb_w_start_in_core,
                last_cmd    = False,
                name        = f"r{r}_w[{chunk_lo}:{chunk_hi+1}]_seg{seg_id}",
            ))
            cur_w = chunk_hi + 1

    if cmds:
        cmds[-1].last_cmd = True
    return cmds


def write_odma_sg_cmd_list(out_path: str, cmds: list, header_lines: list = None) -> int:
    """写 ODMA SG cmd list 到 .hex 文件."""
    with open(out_path, 'w') as f:
        f.write("// ODMA SG cmd list ($readmemh 32-byte/cmd)\n")
        if header_lines:
            for line in header_lines:
                f.write(f"// {line}\n")
        f.write(f"// n_cmds = {len(cmds)}\n")
        for c in cmds:
            for hex_line in c.to_hex_lines():
                f.write(hex_line)
                if c.name and hex_line == c.to_hex_lines()[0]:
                    f.write(f"  // {c.name}{' [LAST]' if c.last_cmd else ''}")
                f.write("\n")
    return len(cmds)


# ============================================================================
# Cout slice 切片 (D 路径退化版): 每核负责 cout 段, 共享整图 IFM
# ============================================================================
#
# 适用场景: H_OUT × W_OUT 很小 + cout >= n_cores × NUM_COL, 且上层 IFM 是 mode A
#   集中存放 (e.g., L10 FC W=H=1, host preload 整图到 mem[0]).
#
# 数据布局:
#   - IFM 整图集中在 mem[k] (k = ifm_mem_core), 4 核 IDMA 都从 mem[k] 拉相同数据
#   - WB 按 cout 段切, 每核装自己段 weight 到自己 mem (preload 时各 mem 各装一段)
#   - OFM 每核写自己 cout 段到自己 mem (host 比对时从 4 mem gather)
#
# 跟 W slice 的区别:
#   - W slice: 每核 W 段不同, IFM 切, 同步 cout
#   - cout slice: IFM 整图共享, cout 段切
# ============================================================================
def compute_cout_segments(cout_full: int, n_split: int, NUM_COL: int = 16) -> tuple:
    """整 cout 维切 n_split 段, 段宽是 NUM_COL 的整数倍 (除最后段含尾巴).

    Round-robin: 把 cout_slices 个 NUM_COL-block 平均分给 n 核, 余给前 rem 核 +1 个 cs.
    最后一段 cout 范围含尾巴 (cout_full % NUM_COL ≠ 0 时).

    返回:
      seg_cout_starts : 每核段起点 (cout idx, NUM_COL 倍数)
      seg_widths_cout : 每核段实际 cout 通道数 (最后一段可能 < NUM_COL × cs)
      seg_cs          : 每核 cout_slices 数 (PE col 维 NUM_COL block 数)

    例: cout=528 NUM_COL=16 n=4 → cs_full=33
        cs/core = [9, 8, 8, 8]
        starts  = [0, 144, 272, 400]
        widths  = [144, 128, 128, 128]   (sum=528)
    """
    cs_full = (cout_full + NUM_COL - 1) // NUM_COL
    base_cs = cs_full // n_split
    rem_cs  = cs_full % n_split
    seg_cs = [(base_cs + 1) if i < rem_cs else base_cs for i in range(n_split)]
    seg_cout_starts = [0] * n_split
    cur = 0
    for i in range(n_split):
        seg_cout_starts[i] = cur * NUM_COL
        cur += seg_cs[i]
    seg_widths_cout = [
        seg_cs[i] * NUM_COL if i < n_split - 1
        else cout_full - seg_cout_starts[i]
        for i in range(n_split)
    ]
    return seg_cout_starts, seg_widths_cout, seg_cs


def gen_idma_sg_cmd_list_cout_slice(
    *,
    h_in:           int,
    w_in:           int,
    cin_slices:     int,
    ifb_mem_base:   int,
    ifb_ring_words: int = 0,
) -> list:
    """生成 cout slice 模式 IDMA SG cmd list (每核拉相同整图 IFM).

    cout 切下 IFM 全核共享, 整图集中存放在 ifb_mem_base 起 (一般是 mem[0] mode A).
    每行 IFM 一条 cmd, btt = W_in × cin_slices × 16.
    """
    cmds = []
    sram_offset = 0
    row_words = w_in * cin_slices
    for r in range(h_in):
        sram_offset_eff = (sram_offset % ifb_ring_words) if ifb_ring_words > 0 else sram_offset
        cmds.append(SgCmd(
            src_addr    = ifb_mem_base + r * row_words * 16,
            btt         = row_words * 16,
            sram_offset = sram_offset_eff,
            last_cmd    = (r == h_in - 1),
            name        = f"r{r}_coutSlice",
        ))
        sram_offset += row_words
    return cmds


def gen_odma_sg_cmd_list_cout_slice(
    *,
    h_out:                int,
    w_out:                int,
    cout_slices_per_core: int,
    ofm_mem_base:         int,
) -> list:
    """生成 cout slice 模式 ODMA SG cmd list (每核写自己 cout 段紧凑 layout).

    每核 ODMA 写自己 cout 段到自己 mem 紧凑 layout. 每行 OFM 一条 cmd,
    btt = W_out × cout_slices_per_core × 16. (注意: cout_slices_per_core 是
    本核 PE col block 数, 不是整图 cout_slices.)
    """
    cmds = []
    row_words = w_out * cout_slices_per_core
    for r in range(h_out):
        cmds.append(OdmaSgCmd(
            dst_addr    = ofm_mem_base + r * row_words * 16,
            btt         = row_words * 16,
            ofb_w_start = 0,
            last_cmd    = (r == h_out - 1),
            name        = f"r{r}_coutSlice",
        ))
    return cmds


# ============================================================================
# 整图 W 4 等分散布到 4 mem (driver layout 决定)
# ============================================================================
def compute_smc_w_segments(W_full: int, n_mem: int) -> tuple:
    """整图 W 切 n_mem 段 (核间负载均衡: 余数分散给前 rem 个核, 每个 +1 列).

    Round-A optimization (2026-05-07): 旧版"余数全给末核"让 W_full=135 切 4 段 = [33,33,33,36],
    末核多 9% cells, layer barrier 让前 3 核闲等末核, util 损失 5~8%.
    新版"余数分散"让 W=135 = [34,34,34,33], 段宽差距 ≤ 1 col (3% vs 9%), 整网更均衡.

    跟 hw_files.compute_w_slice_geom 内 OFM 切法保持一致 (同公式):
      base = W_full // n; rem = W_full % n
      core[i] width = base+1 if i<rem else base
      core[i] start = i*(base+1) if i<rem else rem*(base+1) + (i-rem)*base

    例: W=135, n=4 → widths=[34,34,34,33], starts=[0,34,68,102]
        W=33,  n=4 → widths=[9,8,8,8],     starts=[0,9,17,25]
        W=32,  n=4 → widths=[8,8,8,8],     starts=[0,8,16,24]
    """
    base = W_full // n_mem
    rem  = W_full %  n_mem
    seg_widths  = [(base + 1) if i < rem else base for i in range(n_mem)]
    seg_w_starts = [0] * n_mem
    cur = 0
    for i in range(n_mem):
        seg_w_starts[i] = cur
        cur += seg_widths[i]
    return seg_w_starts, seg_widths
