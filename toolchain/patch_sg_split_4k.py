"""Patch idma_sg.hex: 把跨 4KB 边界的 cmd 拆成 2 sub-cmd, 让 axi_dm 不卡.

每 cmd 32 byte (2 hex line × 16 byte):
- line 1 (低 128 bit big-endian hex):
  bit[127:96] reserved
  bit[95:64]  sram_offset
  bit[63:32]  {reserved[7:0], last_cmd[1], btt[22:0]}
  bit[31:0]   src_addr
- line 2 (高 128 bit, all 0)
"""
import sys
import os
import shutil

PAGE = 0x1000  # 4 KB AXI boundary

def parse_line(s):
    """16 byte big-endian hex string → src, btt, last_cmd, sram_offset."""
    v = int(s, 16)
    src = v & 0xFFFFFFFF
    w1 = (v >> 32) & 0xFFFFFFFF
    btt = w1 & 0x7FFFFF
    last_cmd = (w1 >> 23) & 1
    sram_offset = (v >> 64) & 0xFFFFFFFF
    return src, btt, last_cmd, sram_offset

def encode_line(src, btt, last_cmd, sram_offset):
    """src+btt+last+sram_offset → 16 byte hex line (big-endian)."""
    w0 = src & 0xFFFFFFFF
    w1 = (btt & 0x7FFFFF) | ((last_cmd & 1) << 23)
    w2 = sram_offset & 0xFFFFFFFF
    v = (w2 << 64) | (w1 << 32) | w0
    return f'{v:032x}'

def patch_file(path):
    with open(path) as f:
        lines = f.readlines()

    out = []
    cmds = []
    for ln in lines:
        s = ln.strip()
        if not s or s.startswith('//') or s.startswith('#'):
            out.append(ln)
            continue
        # 去掉 inline // comment
        hex_part = s.split('//')[0].strip().split()[0]
        cmds.append((ln, hex_part))

    # cmds 每 2 行 1 cmd
    assert len(cmds) % 2 == 0, f'cmd hex 行数 {len(cmds)} 不是 2 倍'

    n_orig = len(cmds) // 2
    n_split = 0
    new_cmd_lines = []
    for i in range(n_orig):
        line1 = cmds[i*2][1]
        line2 = cmds[i*2+1][1]
        src, btt, last_cmd, sram_offset = parse_line(line1)

        end = src + btt
        page_end = (src + PAGE) & ~(PAGE - 1)
        if end > page_end:
            # 跨 4KB, 拆 2 sub-cmd
            btt1 = page_end - src
            btt2 = btt - btt1
            sram_offset2 = sram_offset + btt1 // 16
            # sub-cmd 1: src .. page_end, last_cmd=0
            new_cmd_lines.append(encode_line(src, btt1, 0, sram_offset) + '\n')
            new_cmd_lines.append(line2 + '\n')
            # sub-cmd 2: page_end .. end, last_cmd 继承
            new_cmd_lines.append(encode_line(page_end, btt2, last_cmd, sram_offset2) + '\n')
            new_cmd_lines.append(line2 + '\n')
            n_split += 1
        else:
            new_cmd_lines.append(line1 + '\n')
            new_cmd_lines.append(line2 + '\n')

    out.extend(new_cmd_lines)

    # backup
    bak = path + '.orig'
    if not os.path.exists(bak):
        shutil.copy(path, bak)
    with open(path, 'w') as f:
        f.writelines(out)

    return n_orig, n_split

if __name__ == '__main__':
    case_dir = sys.argv[1] if len(sys.argv) > 1 else '../sim/tb_smc/cases/sim_n4_v20'
    files = []
    for c in range(4):
        for l in range(11):
            p = os.path.join(case_dir, f'core{c}', f'layer{l:02d}_idma_sg.hex')
            if os.path.exists(p):
                files.append(p)

    total_split = 0
    total_orig = 0
    for p in files:
        n_orig, n_split = patch_file(p)
        print(f'{p}: {n_orig} cmd, {n_split} split (cross 4KB)')
        total_orig += n_orig
        total_split += n_split

    print(f'\nTotal: {total_orig} cmd, {total_split} split, also need cmd_count update in meta')
