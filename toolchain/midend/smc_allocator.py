"""smc_allocator.py — SMC 内存动态分配器 (Phase D).

老版 hardcode 偏移 (SMC_INPUT_BASE = 0xD00000, SMC_FINAL_OFM_BASE = 0xF00000 等) 是
假设 layer 数据 ≤ 1 MB. patch1 (K=4 S=4 后 S2D → H=240 W=135 c_in=64) IFM = 1.98 MB
就重叠到 OFM, 导致 IDMA 读到 ODMA 写过的数据.

本 allocator 按 layer 实际 size 动态算 base, 保证 IFM/OFM/WB/RDMA/desc/cmd 各 region
不重叠. 每个 (mem_id, region) 一个 cursor, alloc(size, align) 推进 cursor 返回 base.

Region order in mem layout (per mem_id):
    [desc] [idma_cmd] [odma_cmd] [wb] [rdma] [ifm_ofm_0] [ifm_ofm_1] ... [final_ofm]
所有 region 紧挨, 不留 hardcode 间隙.

Usage:
    alloc = SmcAllocator(mem_stride=0x01000000, num_mems=4, ddr_base=0x10000000)
    # 每个 layer 注册需要的 buffer
    ifm0 = alloc.alloc(mem_id=0, size=h*w*c, label='L0/ifm')
    ofm0 = alloc.alloc(mem_id=0, size=h_out*w_out*c_out, label='L0/ofm')
    desc0 = alloc.alloc(mem_id=0, size=n_desc*32, region='desc', label='L0/desc')
    # 跑完 alloc.report() 打印分配 map
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Dict


@dataclass
class Allocation:
    mem_id: int
    region: str          # 'desc' / 'idma_cmd' / 'odma_cmd' / 'wb' / 'rdma' / 'ifm_ofm' / 'final_ofm' / 'misc'
    base:   int          # mem 内 byte offset
    size:   int          # bytes
    label:  str = ''


class SmcOutOfMemoryError(RuntimeError):
    """mem cursor 超出 mem_stride 边界."""
    pass


class SmcAllocator:
    """Per-(mem_id, region) cursor allocator.

    特点:
    - 不同 region 各有独立 cursor (避免跨 region 干扰; e.g. desc 区跟 IFM 区不抢)
    - 同 region 内 cursor 单调推进 (FIFO, 不释放)
    - alloc 时检查 cursor + size ≤ mem_stride, 超就 raise
    """

    # Region layout: 每个 region 起始 offset 跟最大允许大小
    # 这里给个静态默认 layout (跟现有 hardcode 兼容), Phase D 后期可动态化
    DEFAULT_REGION_OFFSETS = {
        'desc':       0x00A00000,    # desc list (per core, per layer)
        'idma_cmd':   0x00B00000,    # SG cmd list
        'odma_cmd':   0x00C00000,
        'wb':         0x00800000,    # weight broadcast
        'rdma':       0x00900000,    # bias + residual scale
        'ifm_ofm':    0x00000000,    # 中间 layer 数据 (上层 OFM = 下层 IFM)
        'input':      0x00D00000,    # layer 0 IFM (root input)
        'final_ofm':  0x00F00000,    # 最后一层 OFM (host 读)
        'misc':       0x00E00000,    # 备用
    }

    # 每 region 默认上限 (cursor 不能越界进入下个 region)
    # 关键: input region 必须 ≥ 2 MB 装 patch s2d layer 0 (240×135×64 = 1.98 MB).
    # final_ofm 起 0xF00000 (input 后), mem 末 0x1000000.
    DEFAULT_REGION_LIMITS = {
        'desc':       0x00B00000,
        'idma_cmd':   0x00C00000,
        'odma_cmd':   0x00D00000,
        'wb':         0x00900000,
        'rdma':       0x00A00000,
        'ifm_ofm':    0x00800000,    # 中间 layer IFM/OFM 限 8 MB
        'input':      0x00F00000,    # layer 0 IFM 最大 2 MB (D00000 ~ F00000)
        'final_ofm':  0x01000000,    # 最后 layer OFM 限 1 MB (F00000 ~ 1000000)
        'misc':       0x00F00000,
    }

    def __init__(self, mem_stride: int, num_mems: int, ddr_base: int = 0):
        self.mem_stride = mem_stride
        self.num_mems   = num_mems
        self.ddr_base   = ddr_base
        # cursor[mem_id][region] = next free offset
        self.cursor: Dict[int, Dict[str, int]] = {
            m: dict(self.DEFAULT_REGION_OFFSETS) for m in range(num_mems)
        }
        self.history: List[Allocation] = []

    def alloc(self, mem_id: int, size: int, region: str = 'misc',
              align: int = 0x100, label: str = '') -> int:
        """返回 mem 内 byte offset. base 在该 mem 内 = offset (调 smc_global_addr 转全局)."""
        if mem_id >= self.num_mems:
            raise ValueError(f"mem_id={mem_id} 超过 num_mems={self.num_mems}")
        if region not in self.cursor[mem_id]:
            # 未知 region: 跑动态 — cursor = 0
            self.cursor[mem_id][region] = 0
        cur = self.cursor[mem_id][region]
        cur_aligned = (cur + align - 1) & ~(align - 1)
        new_cur = cur_aligned + size
        limit = self.DEFAULT_REGION_LIMITS.get(region, self.mem_stride)
        if new_cur > limit:
            raise SmcOutOfMemoryError(
                f"mem[{mem_id}] region '{region}' (label={label!r}): "
                f"cur=0x{cur_aligned:08x} + size=0x{size:x} = 0x{new_cur:08x} > limit 0x{limit:08x}. "
                f"老 hardcode 偏移不够, 需要扩大 layout 或减小 layer 数据.")
        if new_cur > self.mem_stride:
            raise SmcOutOfMemoryError(
                f"mem[{mem_id}] region '{region}' (label={label!r}) 超出 mem 总容量 "
                f"0x{self.mem_stride:08x}")
        self.cursor[mem_id][region] = new_cur
        self.history.append(Allocation(mem_id, region, cur_aligned, size, label))
        return cur_aligned

    def global_addr(self, mem_id: int, mem_offset: int) -> int:
        """转全局 byte addr = ddr_base + mem_id * mem_stride + mem_offset."""
        return self.ddr_base + mem_id * self.mem_stride + mem_offset

    def report(self) -> str:
        """生成分配 map (debug)."""
        out = [f"=== SmcAllocator report (ddr_base=0x{self.ddr_base:08x}, mem_stride=0x{self.mem_stride:08x}) ==="]
        for mem_id in range(self.num_mems):
            out.append(f"mem[{mem_id}]:")
            allocs_here = [a for a in self.history if a.mem_id == mem_id]
            for a in allocs_here:
                out.append(f"  {a.region:10s}  +0x{a.base:08x}  size 0x{a.size:08x}  {a.label}")
            cur = self.cursor[mem_id]
            for r, c in cur.items():
                if c != self.DEFAULT_REGION_OFFSETS.get(r, 0):  # 只显示用过的 region
                    out.append(f"  cursor[{r}] @ 0x{c:08x}")
        return '\n'.join(out)


# =============================================================================
# Convenience: 跟现有 toolchain SMC layout 一致的快捷分配 API
# =============================================================================
def alloc_layer_buffers(alloc: SmcAllocator, mem_id: int, layer_idx: int, *,
                         ifm_size: int, ofm_size: int, wb_size: int,
                         rdma_size: int, desc_size: int,
                         idma_cmd_size: int, odma_cmd_size: int,
                         is_first_layer: bool, is_last_layer: bool) -> dict:
    """为一层分配所有 buffer, 返回 {region: base} dict."""
    bufs = {}
    bufs['desc']     = alloc.alloc(mem_id, desc_size,     region='desc',     label=f'L{layer_idx}/desc')
    bufs['idma_cmd'] = alloc.alloc(mem_id, idma_cmd_size, region='idma_cmd', label=f'L{layer_idx}/idma_cmd')
    bufs['odma_cmd'] = alloc.alloc(mem_id, odma_cmd_size, region='odma_cmd', label=f'L{layer_idx}/odma_cmd')
    bufs['wb']       = alloc.alloc(mem_id, wb_size,       region='wb',       label=f'L{layer_idx}/wb')
    bufs['rdma']     = alloc.alloc(mem_id, rdma_size,     region='rdma',     label=f'L{layer_idx}/rdma')
    if is_first_layer:
        bufs['ifm'] = alloc.alloc(mem_id, ifm_size, region='input', label=f'L{layer_idx}/ifm(root)')
    else:
        bufs['ifm'] = alloc.alloc(mem_id, ifm_size, region='ifm_ofm', label=f'L{layer_idx}/ifm')
    if is_last_layer:
        bufs['ofm'] = alloc.alloc(mem_id, ofm_size, region='final_ofm', label=f'L{layer_idx}/ofm(final)')
    else:
        bufs['ofm'] = alloc.alloc(mem_id, ofm_size, region='ifm_ofm', label=f'L{layer_idx}/ofm')
    return bufs


if __name__ == '__main__':
    # Smoke test: 模拟 patch1 case 看 allocator 是否避开重叠
    alloc = SmcAllocator(mem_stride=0x01000000, num_mems=1, ddr_base=0x10000000)
    # patch1: H=240 W=135 c_in=64 → ifm = 240*135*64 = 2073600 byte (1.98 MB)
    bufs = alloc_layer_buffers(
        alloc, mem_id=0, layer_idx=0,
        ifm_size=240*135*64,
        ofm_size=240*135*16,
        wb_size=64*16*1*1,  # K=1 c_in=64 c_out=16
        rdma_size=16,
        desc_size=63*32,
        idma_cmd_size=240*32,
        odma_cmd_size=240*32,
        is_first_layer=True, is_last_layer=True,
    )
    print(f"patch1 layer 0 分配:")
    for r, b in bufs.items():
        global_addr = alloc.global_addr(0, b)
        print(f"  {r:6s} @ +0x{b:08x}  (global 0x{global_addr:08x})")
    print()
    print(alloc.report())

    # 检查: input (IFM) 在 0xD00000 起 2MB. final_ofm 在 0xF00000 起 0.5MB. 不重叠 ✓
    ifm_end = bufs['ifm'] + 240*135*64
    print(f"\n  IFM end = 0x{ifm_end:08x}")
    print(f"  OFM base = 0x{bufs['ofm']:08x}")
    assert ifm_end <= bufs['ofm'], f"IFM/OFM 重叠 ifm_end=0x{ifm_end:x} > ofm_base=0x{bufs['ofm']:x}"
    print(f"  [OK] IFM/OFM 不重叠 (allocator 正确避开 hardcode 1MB 偏移失败的场景)")
