"""Dump IDMA SG cmd[0..2] + ODMA SG cmd[0..2] + check OFM DDR has data."""
import os, sys
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, '.')
from vd100_rpc import Vd100Rpc

with Vd100Rpc('169.254.111.10') as rpc:
    rpc.ping()
    print('=== c0 IDMA SG cmd list @ 0x10b00000 (cmd[0..2], 32 byte each) ===')
    data = rpc.read_ddr(0x10b00000, 3 * 32)
    for i in range(3):
        d = data[i*32:(i+1)*32]
        print(f'  cmd[{i}] {d.hex()}')

    print()
    print('=== c0 ODMA SG cmd list @ 0x10c00000 (cmd[0..2]) ===')
    data = rpc.read_ddr(0x10c00000, 3 * 32)
    for i in range(3):
        d = data[i*32:(i+1)*32]
        print(f'  cmd[{i}] {d.hex()}')

    print()
    print('=== c0 ODMA DST area @ 0x10d80000 (first 96 byte — IFB[10] aka next layer in) ===')
    data = rpc.read_ddr(0x10d80000, 96)
    print(f'  {data.hex()}')
    nonzero = sum(1 for b in data if b != 0)
    print(f'  nonzero bytes: {nonzero}/96')
