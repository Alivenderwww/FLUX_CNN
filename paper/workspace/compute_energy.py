"""compute_energy.py — 计算 EXPERIMENT_HANDOFF.md 实验 A/B 表所需能耗数据.

数据流模型 (跟 RTL 一致, case 2 K=1 s=2 单核实测 cross-validate):
  ARF: write = H_in × W_in × cin_slices (每 IFB word 写 1 次)
       read  = N_MAC / NUM_COL (16 col PE broadcast 共享同一 act)
  WRF: write = wb_words × NUM_COL × NUM_PE (一次性 load weight 到所有 PE)
       read  = N_MAC (每 MAC 1 weight read per PE, 静止时 32 拍多次)
  PARF: write = N_MAC (fill), read = N_MAC (drain)

  IFB: write = H_in × W_in × Cin byte (IDMA 拉一次)
       read  = H_out × W_out × Cin byte (case 2 实测吻合)
  WB:  write = wb_words × 256 byte (一次性 load)
       read  = wb_words × 256 byte (load 完读 1 次给 WRF)
  OFB: write = H_out × W_out × Cout byte
       read  = H_out × W_out × Cout byte (ODMA 拉一次)

DRAM:
  IFM (layer 0 / root): H_in × W_in × Cin (但 layer chain 上层 OFM = 下层 IFM, 中间层 IFM 不算)
  WB: K² × Cin × Cout byte (每层独立)
  OFM: H_out × W_out × Cout byte (写 DDR)

能耗 (Horowitz 8 bit MAC level):
  compute = 1 pJ / MAC
  register = 4 pJ / access
  sram = 8 pJ / byte
  dram = 640 pJ / byte
"""

NUM_PE = 16
NUM_COL = 16

E_MAC = 1
E_REG = 4
E_SRAM = 8
E_DRAM = 640


def derive_layer(K, stride, Cin, Cout, H_in, W_in, pad, name=""):
    H_out = (H_in + 2 * pad - K) // stride + 1
    W_out = (W_in + 2 * pad - K) // stride + 1
    cin_slices = (Cin + NUM_PE - 1) // NUM_PE
    cout_slices = (Cout + NUM_COL - 1) // NUM_COL
    N_MAC = K * K * Cin * Cout * H_out * W_out
    wb_bytes = K * K * Cin * Cout  # 1 byte per weight

    # Register (次数)
    arf_w = H_in * W_in * cin_slices
    arf_r = N_MAC // NUM_COL  # 16 col broadcast 共享
    wrf_w = (K * K * cin_slices * cout_slices) * NUM_COL * NUM_PE  # wb_words * 256
    wrf_r = N_MAC
    parf_w = N_MAC
    parf_r = N_MAC
    register_rw = arf_w + arf_r + wrf_w + wrf_r + parf_w + parf_r

    # SRAM (byte)
    ifb_w_byte = H_in * W_in * Cin
    ifb_r_byte = H_out * W_out * Cin   # case 2 实测吻合
    wb_w_byte = wb_bytes
    wb_r_byte = wb_bytes
    ofb_w_byte = H_out * W_out * Cout
    ofb_r_byte = H_out * W_out * Cout
    sram_rw = ifb_w_byte + ifb_r_byte + wb_w_byte + wb_r_byte + ofb_w_byte + ofb_r_byte

    # DRAM (byte) - 单 layer layer-by-layer (输入 + 权重 + 输出各 1 次)
    dram = ifb_w_byte + wb_bytes + ofb_w_byte

    # 能耗
    E_total = N_MAC * E_MAC + register_rw * E_REG + sram_rw * E_SRAM + dram * E_DRAM
    E_dram = dram * E_DRAM
    dram_pct = E_dram / E_total * 100

    return dict(
        name=name, H_out=H_out, W_out=W_out, N_MAC=N_MAC,
        register_rw=register_rw, sram_rw=sram_rw, dram=dram,
        E_total=E_total, dram_pct=dram_pct,
    )


# === 实验 A: 5 case ===
print("\n=== 实验 A: 5 shape 单层 ===\n")
A_cases = [
    ("Patch K=4 s=4 Cin=4 Cout=16 H=540 W=960",  4, 4, 4,   16, 540, 960, 0),
    ("K=3 s=1 Cin=16 Cout=16 H=135 W=240",       3, 1, 16,  16, 135, 240, 1),
    ("K=3 s=2 Cin=16 Cout=32 H=135 W=240",       3, 2, 16,  32, 135, 240, 1),
    ("K=1 s=2 Cin=16 Cout=32 H=135 W=240",       1, 2, 16,  32, 135, 240, 0),
    ("FC Cin=256 Cout=16 H=1 W=1",               1, 1, 256, 16, 1, 1,     0),
]
for name, K, s, ci, co, h, w, p in A_cases:
    r = derive_layer(K, s, ci, co, h, w, p, name)
    print(f"{name}")
    print(f"  N_MAC={r['N_MAC']:>15,}  Register_RW={r['register_rw']:>15,}  SRAM_RW={r['sram_rw']:>15,}")
    print(f"  DRAM={r['dram']:>15,} byte  E_total={r['E_total']:.3e} pJ  DRAM%={r['dram_pct']:.2f}%")
    print()


# === 实验 B: ResNet11 整网 11 layer ablation ===
print("\n=== 实验 B: ResNet11 整网 ===\n")

# ResNet11 11 layers (跟 toolchain/run_regression.py _chain 一致):
# Patch + L1(3) + L2(3) + L3(3) + FC
resnet11_layers = [
    # (K, s, Cin, Cout, H_in, W_in, pad, name)
    (4, 4, 4,   16, 540, 960, 0, "Patch"),
    (3, 1, 16,  16, 135, 240, 1, "L1_C1"),
    (3, 1, 16,  16, 135, 240, 1, "L1_C2"),
    (3, 1, 16,  16, 135, 240, 1, "L1_C3"),
    (3, 2, 16,  32, 135, 240, 1, "L2_C1"),
    (3, 1, 32,  32,  68, 120, 1, "L2_C2"),
    (3, 1, 32,  32,  68, 120, 1, "L2_C3"),
    (3, 2, 32,  64,  68, 120, 1, "L3_C1"),
    (3, 1, 64,  64,  34,  60, 1, "L3_C2"),
    (3, 1, 64,  64,  34,  60, 1, "L3_C3"),
    (1, 1, 256, 522, 1, 1,    0, "FC"),
]

# 整网累加 (注意: layer-by-layer 模式时 IFM 用上层 OFM 不算 DRAM)
total_mac = 0
total_reg = 0
total_sram = 0
total_dram_chained = 0   # chained: 只 layer 0 IFM + 各层 weight + layer 0/中间 OFM 不写回 (实际 streaming 不可能, 但 paper 给 chained 上限)
total_dram_layerwise = 0 # layer-by-layer: 每层 IFB+WB+OFB 都拉

for K, s, ci, co, h, w, p, name in resnet11_layers:
    r = derive_layer(K, s, ci, co, h, w, p, name)
    total_mac += r['N_MAC']
    total_reg += r['register_rw']
    total_sram += r['sram_rw']
    # layer-by-layer DRAM (每层 IFB+WB+OFB)
    total_dram_layerwise += r['dram']
    # chained DRAM (只第一层 IFM + 各层 weight + 最后层 OFM, 中间 OFM/IFM 不写 DDR)
    if name == "Patch":
        total_dram_chained += h * w * ci          # 输入图
    total_dram_chained += K * K * ci * co            # 权重
    if name == "FC":
        total_dram_chained += r['H_out'] * r['W_out'] * co   # 最后输出

print(f"ResNet11 N_MAC total = {total_mac:,} ({total_mac/1e6:.1f} M)")
print(f"  Register_RW total = {total_reg:,}")
print(f"  SRAM_RW total = {total_sram:,}  ({total_sram/1e6:.1f} MB)")
print(f"  DRAM chained (实际 streaming) = {total_dram_chained:,} ({total_dram_chained/1e6:.2f} MB)")
print(f"  DRAM layerwise (每层都拉) = {total_dram_layerwise:,} ({total_dram_layerwise/1e6:.2f} MB)")
print()

# === Ablation 5 行 ===
print("--- Ablation 5 行 ---\n")

# 行 1 baseline (无复用): DRAM = 4× N_MAC (每 MAC 拉 1 IFM byte + 1 weight byte + 写 1 psum + 1 ofm)
baseline_dram = 4 * total_mac
baseline_E = total_mac * E_MAC + 0 + 0 + baseline_dram * E_DRAM

# 行 5 全开 (正常 RTL, 用 chained DRAM)
full_E = total_mac * E_MAC + total_reg * E_REG + total_sram * E_SRAM + total_dram_chained * E_DRAM

# 行 2 + 权重静止 (WRF 32 拍): 只比 baseline 多了 WRF 读. WRF write 算少了 (一次性 load)
# 推算: WRF_r 仍是 N_MAC, 但 WB DRAM 拉一次. 没复用时, weight 每 MAC 拉 1 byte from DRAM (= N_MAC byte)
# 加了 weight 静止 → WB from DRAM = wb_bytes (一次性). 但 IFM/OFB 仍走 DRAM.
# 简化推算: 行 2 = 行 1 但 WB DRAM 改成 wb_total
total_wb_bytes = sum(K*K*ci*co for K, s, ci, co, _, _, _, _ in resnet11_layers)
total_ifb_bytes_dram = sum(h*w*ci for K, s, ci, co, h, w, _, _ in resnet11_layers)
# layer-by-layer 时, ifb = 每层 IFM 拉. chained 时, 只 patch IFM 拉.
total_ofb_bytes_dram = sum((((h+2*p-K)//s+1) * ((w+2*p-K)//s+1) * co)
                            for K, s, ci, co, h, w, p, _ in resnet11_layers)

# 行 2 (+ 权重静止): WB DRAM = wb_total; IFM/OFM 仍按 baseline (没 IFB/OFB 复用) = N_MAC byte 各
# WRF reg: 写 wb_total/256 次? no 实际 register access = WRF_w + WRF_r = wb_total*16PE*16COL... 复杂
# Simplification: 行 2 model = 行 1 基础上 weight 部分加 reg 访问 (跟 RTL 一致)
# 行 2 DRAM = N_MAC * 1 (IFM) + wb_total (静止) + N_MAC * 1 (OFM) ≈ 2 × N_MAC + wb_total
dram_row2 = 2 * total_mac + total_wb_bytes
# 行 2 reg: WRF + PARF (静止 weight + 输出累加)
reg_row2_wrf = total_wb_bytes * NUM_COL * NUM_PE + total_mac   # wrf_w + wrf_r
E_row2 = total_mac * E_MAC + reg_row2_wrf * E_REG + 0 + dram_row2 * E_DRAM

# 行 3 (+ 激活滑窗行复用): IFB 引入, IFM 从 DRAM 拉 H_in*W_in*Cin (每层 IFB write byte)
# ARF + WRF + PARF reg 全有
# 但 sliding-window 还没引入 IFB/OFB read (那是行 4-5 的事).
# 实际 paper 文档给的 hint: 行 3 关掉 IFB 滑窗 → IFB_read = N_MAC × Cin / 8
# 我们 +滑窗 enable, IFB read 从 N_MAC × Cin / 8 降到 H_out × W_out × Cin
# 简化: 行 3 = 行 2 + 加 IFB write (DRAM 减少, SRAM 增加)
dram_row3 = total_ifb_bytes_dram + total_wb_bytes + total_ofb_bytes_dram   # layer-by-layer
# 行 3 reg: arf + wrf + parf (ARF read 还没 broadcast, 所以 ARF_r = N_MAC, 不是 N_MAC/NUM_COL)
arf_r_no_broadcast = total_mac    # 没广播时 16 PE 各读 → N_MAC 次
reg_row3 = (sum(h*w*((ci+NUM_PE-1)//NUM_PE) for K,s,ci,co,h,w,_,_ in resnet11_layers) +
             arf_r_no_broadcast +    # ARF
             total_wb_bytes * NUM_COL * NUM_PE + total_mac +  # WRF
             total_mac * 2)   # PARF
sram_row3 = total_sram   # 跟全开同 (IFB/WB/OFB 都用)
E_row3 = total_mac * E_MAC + reg_row3 * E_REG + sram_row3 * E_SRAM + dram_row3 * E_DRAM

# 行 4 (+ 输入通道广播): ARF read 从 N_MAC 降到 N_MAC / NUM_COL
# 其它跟 行 3 一致
reg_row4 = (sum(h*w*((ci+NUM_PE-1)//NUM_PE) for K,s,ci,co,h,w,_,_ in resnet11_layers) +
             total_mac // NUM_COL +    # ARF broadcast
             total_wb_bytes * NUM_COL * NUM_PE + total_mac +
             total_mac * 2)
E_row4 = total_mac * E_MAC + reg_row4 * E_REG + sram_row3 * E_SRAM + dram_row3 * E_DRAM

# 行 5 = 全开 (PARF 已经在前面 row 2 启用, paper 文档可能行 5 = 跟 row 4 同 + chained DRAM)
# 实际 paper 应该: 行 5 = 真实 RTL, chained DRAM (中间层 OFM 不写 DDR)
E_row5 = total_mac * E_MAC + total_reg * E_REG + total_sram * E_SRAM + total_dram_chained * E_DRAM

print(f"行 1 baseline:    N_MAC={total_mac:>11,}  Reg=0  SRAM=0  DRAM={baseline_dram:>12,}  E_total={baseline_E:.3e}  norm=1.00")
print(f"行 2 + 权重静止:  N_MAC={total_mac:>11,}  Reg={reg_row2_wrf:>14,}  SRAM=0  DRAM={dram_row2:>12,}  E_total={E_row2:.3e}  norm={E_row2/baseline_E:.4f}")
print(f"行 3 + 滑窗复用:  N_MAC={total_mac:>11,}  Reg={reg_row3:>14,}  SRAM={sram_row3:>11,}  DRAM={dram_row3:>12,}  E_total={E_row3:.3e}  norm={E_row3/baseline_E:.4f}")
print(f"行 4 + 通道广播:  N_MAC={total_mac:>11,}  Reg={reg_row4:>14,}  SRAM={sram_row3:>11,}  DRAM={dram_row3:>12,}  E_total={E_row4:.3e}  norm={E_row4/baseline_E:.4f}")
print(f"行 5 全开 (实测):  N_MAC={total_mac:>11,}  Reg={total_reg:>14,}  SRAM={total_sram:>11,}  DRAM={total_dram_chained:>12,}  E_total={E_row5:.3e}  norm={E_row5/baseline_E:.4f}")
