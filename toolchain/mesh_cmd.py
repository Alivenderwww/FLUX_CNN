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
) -> list:
    """生成 ConvCore[c] 的 IDMA SG cmd list (W slice 紧凑 + 跨 mem halo 拉取).

    每行 IFM 拆成多条 cmd: 每条在一个 mem 内的连续 burst.
    跨 mem 边界时多条 cmd. 物理 halo 列只一份 (driver 编译期确保 layout 无重复).

    seg_* 数组描述整图 W 段散布: 段 i 在整图 W 列 [seg_w_starts[i], seg_w_starts[i]+seg_widths[i]),
    物理在 mem[i].ddr_mem 内全局地址 seg_mem_bases[i] 起.
    """
    n_segs = len(seg_w_starts)
    assert len(seg_mem_bases) == n_segs and len(seg_widths) == n_segs

    cmds = []
    sram_offset = 0
    for r in range(h_in):
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
            chunk_hi = min(w_in_hi, seg_w_end)
            chunk_cols = chunk_hi - chunk_lo + 1

            seg_w = seg_widths[seg_id]
            w_local_lo = chunk_lo - seg_w_starts[seg_id]
            byte_addr = (seg_mem_bases[seg_id]
                         + r * seg_w * cin_slices * 16
                         + w_local_lo * cin_slices * 16)
            chunk_btt = chunk_cols * cin_slices * 16

            sram_offset_eff = (sram_offset % ifb_ring_words) if ifb_ring_words > 0 else sram_offset
            cmds.append(SgCmd(
                src_addr    = byte_addr,
                btt         = chunk_btt,
                sram_offset = sram_offset_eff,
                last_cmd    = False,
                name        = f"r{r}_w[{chunk_lo}:{chunk_hi+1}]_seg{seg_id}",
            ))
            sram_offset += chunk_cols * cin_slices
            cur_w = chunk_hi + 1

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
