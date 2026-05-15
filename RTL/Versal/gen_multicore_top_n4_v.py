#!/usr/bin/env python3
"""Generate multicore_top_n4_v.v wrapper for N=4 (BD-friendly .v wrapper).

Each AXI port (csr_axil + m_axi) needs full X_INTERFACE_INFO attributes
per signal. N=4 doubles the boilerplate to ~600 lines. Generate programmatically.

Output: ../RTL/Versal/multicore_top_n4_v.v
"""
import os

N = 4

CSR_SIGNALS = [
    # (axi_name, port_suffix, direction, width_str)
    ('AWADDR',  'awaddr',  'input',  '[11:0]'),
    ('AWVALID', 'awvalid', 'input',  ''),
    ('AWREADY', 'awready', 'output', ''),
    ('WDATA',   'wdata',   'input',  '[31:0]'),
    ('WSTRB',   'wstrb',   'input',  '[3:0]'),
    ('WVALID',  'wvalid',  'input',  ''),
    ('WREADY',  'wready',  'output', ''),
    ('BRESP',   'bresp',   'output', '[1:0]'),
    ('BVALID',  'bvalid',  'output', ''),
    ('BREADY',  'bready',  'input',  ''),
    ('ARADDR',  'araddr',  'input',  '[11:0]'),
    ('ARVALID', 'arvalid', 'input',  ''),
    ('ARREADY', 'arready', 'output', ''),
    ('RDATA',   'rdata',   'output', '[31:0]'),
    ('RRESP',   'rresp',   'output', '[1:0]'),
    ('RVALID',  'rvalid',  'output', ''),
    ('RREADY',  'rready',  'input',  ''),
]

MAXI_SIGNALS = [
    ('AWID',    'awid',    'output', '[3:0]'),
    ('AWADDR',  'awaddr',  'output', '[31:0]'),
    ('AWLEN',   'awlen',   'output', '[7:0]'),
    ('AWSIZE',  'awsize',  'output', '[2:0]'),
    ('AWBURST', 'awburst', 'output', '[1:0]'),
    ('AWLOCK',  'awlock',  'output', ''),
    ('AWCACHE', 'awcache', 'output', '[3:0]'),
    ('AWPROT',  'awprot',  'output', '[2:0]'),
    ('AWQOS',   'awqos',   'output', '[3:0]'),
    ('AWVALID', 'awvalid', 'output', ''),
    ('AWREADY', 'awready', 'input',  ''),
    ('WDATA',   'wdata',   'output', '[127:0]'),
    ('WSTRB',   'wstrb',   'output', '[15:0]'),
    ('WLAST',   'wlast',   'output', ''),
    ('WVALID',  'wvalid',  'output', ''),
    ('WREADY',  'wready',  'input',  ''),
    ('BID',     'bid',     'input',  '[3:0]'),
    ('BRESP',   'bresp',   'input',  '[1:0]'),
    ('BVALID',  'bvalid',  'input',  ''),
    ('BREADY',  'bready',  'output', ''),
    ('ARID',    'arid',    'output', '[3:0]'),
    ('ARADDR',  'araddr',  'output', '[31:0]'),
    ('ARLEN',   'arlen',   'output', '[7:0]'),
    ('ARSIZE',  'arsize',  'output', '[2:0]'),
    ('ARBURST', 'arburst', 'output', '[1:0]'),
    ('ARLOCK',  'arlock',  'output', ''),
    ('ARCACHE', 'arcache', 'output', '[3:0]'),
    ('ARPROT',  'arprot',  'output', '[2:0]'),
    ('ARQOS',   'arqos',   'output', '[3:0]'),
    ('ARVALID', 'arvalid', 'output', ''),
    ('ARREADY', 'arready', 'input',  ''),
    ('RID',     'rid',     'input',  '[3:0]'),
    ('RDATA',   'rdata',   'input',  '[127:0]'),
    ('RRESP',   'rresp',   'input',  '[1:0]'),
    ('RLAST',   'rlast',   'input',  ''),
    ('RVALID',  'rvalid',  'input',  ''),
    ('RREADY',  'rready',  'output', ''),
]

CSR_AXIL_PARAM = ('"MODE Slave, ADDR_WIDTH 12, DATA_WIDTH 32, PROTOCOL AXI4LITE, '
                  'FREQ_HZ 100000000, ID_WIDTH 0, HAS_BURST 0, HAS_LOCK 0, HAS_PROT 0, '
                  'HAS_CACHE 0, HAS_QOS 0, HAS_REGION 0, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 1"')

MAXI_PARAM = ('"MODE Master, PROTOCOL AXI4, ADDR_WIDTH 32, DATA_WIDTH 128, ID_WIDTH 4, '
              'FREQ_HZ 100000000, NUM_WRITE_OUTSTANDING 1, NUM_WRITE_THREADS 1, '
              'NUM_READ_OUTSTANDING 1, NUM_READ_THREADS 1, READ_WRITE_MODE READ_WRITE, '
              'HAS_BURST 1, HAS_LOCK 1, HAS_PROT 1, HAS_CACHE 1, HAS_QOS 1, HAS_REGION 0, '
              'HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 1, SUPPORTS_NARROW_BURST 1, '
              'MAX_BURST_LENGTH 256, AWUSER_WIDTH 0, ARUSER_WIDTH 0, WUSER_WIDTH 0, '
              'RUSER_WIDTH 0, BUSER_WIDTH 0"')


def gen_csr_axil(idx):
    lines = []
    lines.append(f'    // ---- csr_axil_{idx} (per-core CSR slave, 12-bit addr) ----')
    lines.append(f'    (* X_INTERFACE_PARAMETER = {CSR_AXIL_PARAM} *)')
    for (sig, suffix, direction, width) in CSR_SIGNALS:
        attr = f'(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 csr_axil_{idx} {sig}" *)'
        port_name = f'csr_axil_{idx}_{suffix}'
        width_pad = width if width else ''
        lines.append(f'    {attr:<90} {direction:<6} {width_pad:<8} {port_name},')
    return '\n'.join(lines)


def gen_maxi(idx):
    lines = []
    lines.append(f'    // ---- m_axi_{idx} (per-core AXI4 master, 128-bit data) ----')
    lines.append(f'    (* X_INTERFACE_PARAMETER = {MAXI_PARAM} *)')
    for (sig, suffix, direction, width) in MAXI_SIGNALS:
        attr = f'(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi_{idx} {sig}" *)'
        port_name = f'm_axi_{idx}_{suffix}'
        width_pad = width if width else ''
        lines.append(f'    {attr:<90} {direction:<6} {width_pad:<8} {port_name},')
    return '\n'.join(lines)


def gen_inst_connect():
    """生成内部 multicore_top_n4 实例化的端口连接 (concat per-core ports into packed buses)."""
    lines = []

    # CSR slave 端口 (按 multicore_top_n4 内部 packed bus 顺序连)
    csr_groups = {
        'csr_axil_awaddr':  (12,  'awaddr'),
        'csr_axil_awvalid': (1,   'awvalid'),
        'csr_axil_awready': (1,   'awready'),
        'csr_axil_wdata':   (32,  'wdata'),
        'csr_axil_wstrb':   (4,   'wstrb'),
        'csr_axil_wvalid':  (1,   'wvalid'),
        'csr_axil_wready':  (1,   'wready'),
        'csr_axil_bresp':   (2,   'bresp'),
        'csr_axil_bvalid':  (1,   'bvalid'),
        'csr_axil_bready':  (1,   'bready'),
        'csr_axil_araddr':  (12,  'araddr'),
        'csr_axil_arvalid': (1,   'arvalid'),
        'csr_axil_arready': (1,   'arready'),
        'csr_axil_rdata':   (32,  'rdata'),
        'csr_axil_rresp':   (2,   'rresp'),
        'csr_axil_rvalid':  (1,   'rvalid'),
        'csr_axil_rready':  (1,   'rready'),
    }
    for sv_port, (width, suffix) in csr_groups.items():
        # concat order: core[N-1] ... core[0] (MSB-first)
        concat = ', '.join([f'csr_axil_{i}_{suffix}' for i in range(N-1, -1, -1)])
        lines.append(f'        .{sv_port:<22}({{{concat}}}),')

    maxi_groups = {
        'm_axi_awid':     (4,   'awid'),
        'm_axi_awaddr':   (32,  'awaddr'),
        'm_axi_awlen':    (8,   'awlen'),
        'm_axi_awsize':   (3,   'awsize'),
        'm_axi_awburst':  (2,   'awburst'),
        'm_axi_awlock':   (1,   'awlock'),
        'm_axi_awcache':  (4,   'awcache'),
        'm_axi_awprot':   (3,   'awprot'),
        'm_axi_awqos':    (4,   'awqos'),
        'm_axi_awvalid':  (1,   'awvalid'),
        'm_axi_awready':  (1,   'awready'),
        'm_axi_wdata':    (128, 'wdata'),
        'm_axi_wstrb':    (16,  'wstrb'),
        'm_axi_wlast':    (1,   'wlast'),
        'm_axi_wvalid':   (1,   'wvalid'),
        'm_axi_wready':   (1,   'wready'),
        'm_axi_bid':      (4,   'bid'),
        'm_axi_bresp':    (2,   'bresp'),
        'm_axi_bvalid':   (1,   'bvalid'),
        'm_axi_bready':   (1,   'bready'),
        'm_axi_arid':     (4,   'arid'),
        'm_axi_araddr':   (32,  'araddr'),
        'm_axi_arlen':    (8,   'arlen'),
        'm_axi_arsize':   (3,   'arsize'),
        'm_axi_arburst':  (2,   'arburst'),
        'm_axi_arlock':   (1,   'arlock'),
        'm_axi_arcache':  (4,   'arcache'),
        'm_axi_arprot':   (3,   'arprot'),
        'm_axi_arqos':    (4,   'arqos'),
        'm_axi_arvalid':  (1,   'arvalid'),
        'm_axi_arready':  (1,   'arready'),
        'm_axi_rid':      (4,   'rid'),
        'm_axi_rdata':    (128, 'rdata'),
        'm_axi_rresp':    (2,   'rresp'),
        'm_axi_rlast':    (1,   'rlast'),
        'm_axi_rvalid':   (1,   'rvalid'),
        'm_axi_rready':   (1,   'rready'),
    }
    for sv_port, (width, suffix) in maxi_groups.items():
        concat = ', '.join([f'm_axi_{i}_{suffix}' for i in range(N-1, -1, -1)])
        lines.append(f'        .{sv_port:<22}({{{concat}}}),')

    return '\n'.join(lines)


# Generate full file
out_lines = []
out_lines.append('`timescale 1ns/1ps')
out_lines.append('')
out_lines.append('// =============================================================================')
out_lines.append('// multicore_top_n4_v.v  --  N=4 多核 .v wrapper (auto-generated by gen_multicore_top_n4_v.py)')
out_lines.append('//')
out_lines.append('// 暴露 4×csr_axil (per-core 12-bit) + 4×m_axi (per-core 128-bit data) 给 BD.')
out_lines.append('// BD 内 SmartConnect 处理 CIPS.M_AXI_FPD → csr_axil_N + 4×m_axi → 4×axi_bram_ctrl(URAM 1MB).')
out_lines.append('//')
out_lines.append('// 内部实例化 multicore_top_n4 SV 模块 (generate-for 实例化 4 个 core_top).')
out_lines.append('// =============================================================================')
out_lines.append('')
out_lines.append('(* CORE_GENERATION_INFO = "multicore_top_n4_v,multicore_top_n4,{}" *)')
out_lines.append('module multicore_top_n4_v (')
out_lines.append('    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *)')
out_lines.append('    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF csr_axil_0:csr_axil_1:csr_axil_2:csr_axil_3:m_axi_0:m_axi_1:m_axi_2:m_axi_3, ASSOCIATED_RESET rst_n" *)')
out_lines.append('    input                clk,')
out_lines.append('    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 rst_n RST" *)')
out_lines.append('    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)')
out_lines.append('    input                rst_n,')
out_lines.append('')

for i in range(N):
    out_lines.append(gen_csr_axil(i))
    out_lines.append('')
for i in range(N):
    out_lines.append(gen_maxi(i))
    out_lines.append('')

out_lines.append('    // ---- IRQ (任一核 done sticky 拉高) ----')
out_lines.append('    output               irq_done')
out_lines.append(');')
out_lines.append('')
out_lines.append('    // 内部实例化 multicore_top_n4 SV 模块, per-core ports concat 成 packed bus')
out_lines.append('    multicore_top_n4 u_mc_n4 (')
out_lines.append('        .clk            (clk),')
out_lines.append('        .rst_n          (rst_n),')
out_lines.append(gen_inst_connect())
out_lines.append('        .irq_done       (irq_done)')
out_lines.append('    );')
out_lines.append('')
out_lines.append('endmodule')

content = '\n'.join(out_lines)
out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'multicore_top_n4_v.v')
with open(out_path, 'w') as f:
    f.write(content)
print(f'Generated: {out_path}  ({len(out_lines)} lines)')
