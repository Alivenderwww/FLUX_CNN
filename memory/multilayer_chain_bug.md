---
name: 多层 chain TB Layer 2+ OFM 不匹配 bug
description: tb_multicore_chain 跑 ≥3 层 chain 时, Layer 2 起每层 OFM 对不上 (单层 wslice1 OK), 待修
type: project
---

# 多层 chain TB Layer 2+ OFM 不匹配 bug

> P1 阶段, 2026-04-30. wslice1 单层 N=2 验证通过, 多层 chain 有 bug.

## 现象

`tb_multicore_chain.sv` + `run_multicore_chain.py` 跑多层 chain (host stage barrier 模式):
- N=1 simple3 (3 层): Layer 0 / 1 OFM PASS, **Layer 2 OFM 1003/1024 mismatches**
- N=2 simple3 (3 层 W slice): Layer 0 / 1 OFM PASS, **Layer 2 (== final) OFM 1003/1024 mismatches**
- N=2 wslice4 (4 层): Layer 0 / 1 OFM PASS, Layer 2 1003/1024, **Layer 3 (final) 1024/1024 mismatches**
- N=2 wslice5 (5 层): 类似 pattern

**Layer 2 mismatch 数 (1003/1024) 跟 N 无关**, 说明跟 W 切片几何无关, 是 multi-layer chain 路径的 bug.

单层 case 全 PASS:
- wslice1 N=2: PASS @ 7567 cycles (含 host stage barrier 开销)
- run_regression 单核 26-case 全 PASS

## 反直觉

mismatch pattern 看 first OFB[0]:
- expected = `00006c0000810000a5ff00345dff8e00`
- got      = `00006c00000100000000000000008e00`

byte-level 比对 (16 channels NHWC):
- channel 0, 1, 2, 13, 14, 15: 全部一致 (含 0x6c, 0x8e 等 mid-range 值)
- channel 3..12: 大部分 `got` 是 0, `expected` 是非 0

像 PE 列 0/1/2/13/14/15 算对了, PE 列 3..12 没算 (输出 0).

## 已排除原因

1. ✗ W slice 数学错误 — 单层 wslice1 PASS, 而且 N=1 多层也复现 bug
2. ✗ Layer 2 IFM 数据错误 — `Layer 1 intermediate OFM @ 0x200000` 验证 0/1024 mismatches, 即 Layer 2 IDMA 读源数据正确
3. ✗ DDR address 计算错误 — cfg dump 显示 Layer 2 IDMA_SRC_BASE=0x200000, ODMA_DST=0x300000 (intermediate) / 0x1900000 (final), 都对
4. ✗ Cfg 寄存器写入 — Layer 2 desc list 完整, 全部 56 个 CFG_WRITE descriptors 跟 single-layer 一致
5. ✗ DataMover write 落 DDR 的延迟 — 即使 wait 2000 cycles 后 POST 检查也是 mismatch

## 怀疑方向

1. **WB SRAM stale**: Layer 1 的 weight 没被 Layer 2 WDMA 完全覆盖? 如果只有一部分 weight 更新, mac_array 部分 PE 读 Layer 1 weight, 部分读 Layer 2.
2. **wgt_buffer 内部状态残留**: 比如 round counter / kk counter 没在 layer 边界复位
3. **bias_rf 残留**: Layer 1 的 bias 没被 Layer 2 RDMA 覆盖? 但 rdma_data 都是 0, 不会有残留 ≠ 0 的偏差
4. **ARF / PARF SRAM 残留**: 在 layer 边界没复位
5. **start_dfe / start_layer pulse 时序**: 也许 Layer 2 的 start_layer 和 Layer 1 的 sequencer 收尾有 race

## 复现命令

```bash
cd toolchain
python run_multicore_chain.py --case_name multicore_demo --demo simple3 --n_cores 1
cd ../sim/tb_multicore
vsim -c -do "do run_chain.tcl"
```

观察输出 `POST-LAYER 2 OFM check @ 0x...: mismatches=1003/1024`.

## 注意: 之前看过的 "false PASS"

`tb_multicore_chain.sv` 有一段 `check_final_ofb` 任务**没被调用** (主流程漏了 call), 导致 mismatches 默认 0,
打出 "RESULT: PASS". 已修. 之前所有 ≥2 layer chain 的 PASS 报告都是假的, 重新跑全失败 Layer 2.

## 调试建议

1. dump WB SRAM 内容 (Layer 2 计算前) vs Layer 2 wb.txt 的字节级对比
2. dump 第一个 mac_psum 的输入 act/wgt 跟 Python 算出的预期对比
3. 加一个 Layer 1 → Layer 2 之间 reset 信号 (短暂拉 rst_n) 看是否能恢复
4. 对比 `run_regression` 11-layer chain (相同 host 多 case 启动模式) 跟 multicore_chain 的差异

## 当前状态 (2026-04-30)

- RTL idma_ctrl + core_top wiring 已就位 (P1 RTL 部分完成, 单核 26-case 回归 0 退化)
- `derive_w_slice_cfg` + `compute_w_slice_geom` 已就位 (P1 编译器部分完成)
- `run_multicore_chain.py` per-layer desc lists 已就位
- `tb_multicore_chain.sv` host stage barrier loop + check_final_ofb 已修
- **wslice1 单层 N=2 PASS**: 验证 P1 W slice 核心数学 + RTL row_stride 正确
- **多层 chain bug 待修**: 后续工作
