# FSM 与流水线

## 1. 6 层嵌套循环

```
for cs    in 0..cout_slices-1          (cs_cnt)
  [packed: LOAD_WGT 一次性灌 total_wrf 权重]
  for yout in 0..h_out-1               (yout_cnt)
    for tile in 0..num_tiles-1         (tile_cnt)
      for cins in 0..cin_slices-1      (cins_cnt)
        for round in 0..rounds_per_cins-1  (round_cnt)
          [chunked: LOAD_WGT 灌 cur_round_len 权重]
          for pos in 0..cur_round_len-1   (pos_in_round ↔ ky/kx 联动)
            SUBOP (LDnMAC: cnt=0 prefetch, cnt=1..mac_len MAC)
      ST_OFM (valid_w cycles)
```

整个循环由硬件 FSM 推进，软件只写一次配置寄存器就等 `done` 即可。

---

## 2. FSM 状态

```
            ┌──────┐
            │ IDLE │◄──── rst_n
            └──┬───┘
               │ start=1
               ▼
           ┌─────────────┐
           │ S_SHIFT_WR  │   一拍，把 cfg_sdp_shift 写入 SDP
           └──────┬──────┘
                  ▼
           ┌─────────────┐   LOAD_WGT length:
           │ S_LOAD_WGT  │     packed:  cfg_total_wrf
           └──────┬──────┘     chunked: cur_round_len
                  │ sub_cnt == len-1
                  ▼
       ┌─► ┌─────────────┐
       │   │   S_SUBOP   │   内部 cnt: 0..mac_len
       │   └──────┬──────┘
       │          │
       │          ├── subop_end && round_end_only  (chunked) ──► S_LOAD_WGT
       │          ├── subop_end && cins_end &&   ──────────────► S_LOAD_WGT
       │          │     !cins_last && chunked
       │          ├── subop_end && !last_subop_in_tile ────────► S_SUBOP
       │          └── subop_end && last_subop_in_tile
       │                                                       ▼
       │                                               ┌─────────────┐
       │                                               │  S_ST_OFM   │   valid_w cycles
       │                                               └──────┬──────┘
       │                                                      │
       │          ┌──── !done, packed & (tile,yout)_last ─────┤
       └──────────┤     → S_LOAD_WGT (下一 cs 重装)            │
                  │                                            │
                  ├──── !done, chunked ─────► S_LOAD_WGT        │
                  │     （每新 tile 都要重装）                  │
                  │                                            │
                  ├──── !done, packed, 非 tile_last|yout_last ─┤
                  │     → S_SUBOP                              │
                  │                                            │
                  └──── tile_last && yout_last && cs_last      │
                                                               ▼
                                                        ┌──────────┐
                                                        │  S_DONE  │
                                                        └──────────┘
```

### 关键触发条件

- `subop_last_cycle = (state == S_SUBOP) && (sub_cnt == mac_len)`
- `pos_last = (pos_in_round == cur_round_len - 1)`
- `round_last = (round_cnt == cfg_rounds_per_cins - 1)`
- `cins_end = subop_last_cycle && ky_last && kx_last`
- `round_end_only = subop_last_cycle && pos_last && !round_last`
- `last_subop_in_tile = cins_last && ky_last && kx_last`

---

## 3. SUBOP 内部时序

`SUBOP` 对应原 ISA 的 `LDnMAC` 指令（ld_len + mac_len 拍）：

```
sub_cnt:   0        1        2        3    ...    mac_len
           │        │        │        │           │
cnt=0      ▼        │        │        │           │
IFB 读请求 [发出 ifb_re=1, ifb_raddr=base+0]
arf_we_d1  [<= 1]   │        │        │           │

cnt=1               ▼        │        │           │
IFB 读请求 (若 ld>1)[发出 ifb_re=1, ifb_raddr=base+1]
arf_we_d1           [<= 1]   │        │           │
arf_we       [从 d1 流出]    │        │           │
arf_waddr=0  [写 arf[0] w/ ifb_rdata(cnt=0 的读)]
arf_read=0   [读 arf[0]；bypass MUX → act_to_mac = ifb_rdata]
compute_en=1 [MAC 开始]      │        │           │

cnt=2                        ▼        │           │
...  同理，滑窗 arf[1]                 │           │
prod_out reg [MAC cnt=1 的乘积出结果]  │           │
adder_tree   [组合求和]                │           │

cnt=3                                  ▼           │
adder_tree_reg [寄存了 cnt=2 的和]                 │
parf_we_reg    [d2 从 cnt=1 的信号传递到位]
parf 写入      [累加到 PARF[0]]
```

**关键不变量**：
- `arf_we` 延迟 1 拍（`arf_we_d1`）匹配 IFB 1 拍读延迟
- `wrf_we` 延迟 1 拍（`wrf_we_d1`）匹配 WB 1 拍读延迟
- `ofb_we` 延迟 2 拍（`ofb_we_d2`）匹配 PARF 内部 `parf_addr→parf_addr_d1→parf_addr_reg` 流水 + SDP 组合

---

## 4. 推进逻辑（SUBOP 末拍）

每个 `sub_op` 结束（`sub_cnt == mac_len`），并发做下列推进：

### 4.1 ky / kx 推进（与 round 无关）

```
if (kx_last && ky_last) {
    kx=0; ky=0;   // cins 边界
} else if (kx_last) {
    kx=0; ky++;   // 换行
    r_ifb_ky += cfg_w_in;
} else {
    kx++;
}
```

### 4.2 pos_in_round / round_cnt 推进

```
if (pos_last) {
    pos_in_round = 0;
    if (round_last) {
        round_cnt = 0;
        // 这也是 cins 边界
        if (!cins_last) {
            cins++;
            r_ifb_cins += cfg_ifb_cin_step;
            r_wb_cins  += cfg_wb_cin_step;       // chunked
            if (packed) r_wrf_base += cfg_kk;
        } else {
            cins = 0;  // tile 尾，让 ST_OFM 后推进 tile
            reset IFB/WB 指针 到下一 tile 开头
        }
    } else {
        round_cnt++;   // 同 cins 下一轮（chunked only）
    }
} else {
    pos_in_round++;
}
```

### 4.3 ST_OFM 末拍推进 tile/yout/cs

```
if (!tile_last) {
    tile_cnt++;
    x_tile_out_r += TILE_W;
    x_tile_in_r  += TILE_IN_STEP;
} else {
    tile_cnt = 0;
    if (!yout_last) {
        yout++;
        r_ifb_yout += IFB_ROW_STEP;
        r_ofb_yout += W_OUT;
    } else {
        yout = 0;
        if (!cs_last) {
            cs++;
            r_wb_cs    += WB_COUT_STEP;
            r_ofb_cs   += OFB_COUT_STEP;
            r_wrf_base = 0;     // packed: 下一 cs 重新开始累加 cins*kk
        }
        // cs_last: 去 S_DONE
    }
}
```

---

## 5. 流水线气泡分析

### 每 sub_op 有 1 拍预取气泡

`SUBOP` 总共 `mac_len + 1` 拍，第 0 拍是 IFB 预取（无 MAC）。所以 MAC 有效率 = `mac_len / (mac_len + 1)`：

- TILE_W=32 满 tile → 32/33 ≈ 97%
- partial tile valid_w=22 → 22/23 ≈ 96%

### sub_op 之间无气泡

FSM 直接 `sub_op → sub_op` 切换，不需要 fetch/decode 拍（这是相对 ISA 方案的主要改进）。

### ST_OFM 气泡

`ST_OFM` 串行读 PARF、走 SDP、写 OFB，期间不跑 MAC。每 tile 末尾产生 `valid_w` 拍气泡（= 32 或 `last_valid_w`）。

### 轮次 LOAD_WGT 气泡

每个 chunked LOAD_WGT 占 `cur_round_len` 拍（≤32）。对 K=7、Cin ≤ 16、1 cin_slice：每 tile 2 次 LOAD_WGT，共 32+17=49 拍，相对整 tile 的 MAC 拍数（2 rounds × ≤32 positions × 32 mac_len ≈ 2048）占比 ~2.4%。

### 典型 MAC 利用率（回归实测）

| 配置 | MAC 利用率 | 理论上限 |
|------|-----------|----------|
| K=3 66×118（3 满 tile + 1 partial） | 87.32% | ~88% |
| K=3 cin_slices=2 | 91.78% | - |
| K=7 62×114 | 91.69% | - |
| K=5 30×56 s=2 | 93.01% | - |

---

## 6. 下一步优化方向（未实现）

1. **相邻 sub_op 的预取融合**：把 `sub_op → sub_op` 切换处的 IFB 预取与前一 sub_op 末尾的 MAC 重叠，消除每个 sub_op 的 1 拍气泡。理论上限 ~97% → ~99%。
2. **ST_OFM 与下一 tile 的 MAC 重叠**：需要 PARF 双 bank（读写分离），面积代价较大但能消除每 tile 的 32 拍气泡。
3. **加法树流水**：16-PE 求和当前是全组合（8-bit * 8-bit 累加 → 32-bit，4 级加法），在 16×16 时成为关键路径。插入 1-2 级流水寄存器可提升时钟频率到 300-500 MHz。
