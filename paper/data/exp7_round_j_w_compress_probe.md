# 实验 7 / Round J 探针: W 维 stride 软件压缩 — axi_dm cmd 颗粒度反噬

**日期**: 2026-05-08
**结论**: ❌ **不可行** — ResNet11 N=4 SMC: 190977 → **261627** cy (**+37%**, +70650 cy)
**含义**: 软件层 W 压缩走 axi_dm IP 必然性失败, 硬件 2D 寻址才是出路 (论文素材: "为什么必须改硬件")

## 动机

Round I (paper/data/exp6) H 维 stride 分离拿了 -5.9% (ds layer L3/L6/L9 cy 减 28-43%).
留了 W 维同样问题: ds layer K=1 stride>1 W 维仍 dense 拉数据, 1/stride 的列被白拉.

**理论收益**: ds layer 数据量再减半, 整网 ~ -2~3%.

**实施限制**: axi_dm IP 不支持 strided burst — 每 cmd 是连续地址 burst. 要 W 维跳读
只能"每个保留像素 1 cmd". 拆 cmd 实现 W 压缩, 测 axi_dm cmd setup overhead 是否
吃掉数据量减半收益 → ROI 拐点验证.

## 实施 (纯 driver 改动, RTL 0 行)

```
toolchain/mesh_cmd.py:        gen_idma_sg_cmd_list_w_slice 加 w_compress_stride 参数
toolchain/run_multicore_chain.py: ds layer 检测 + FLUX_W_COMPRESS=1 env 启用
toolchain/hw_files.py:        cfg_to_dict 加 _STRIDE_W / _IFB_ROW_DENSE_W
                              影响 STRIDE / IFB_ISS_STEP / TILE_PIX_STEP / IFB_ROW_STEP
```

**实现要点**:
- W 压缩时 mesh_cmd 每 cmd 1 像素 (`chunk_cols=1`), `cur_w` 跳 stride
- cfg.STRIDE = 1 让 line_buffer iss_pos_s 不再 ×stride (IFB 内 W 维 dense)
- cfg.IFB_ROW_STEP 用 dense W (= sub_W_in / stride) 而非原 sub_W_in
- 默认 FLUX_W_COMPRESS=0 OFF, 不影响 Round I baseline

## 实测 (commit `<待 commit>`)

### ResNet11 N=4 SMC 整网 cy

| 阶段 | Total cy | Δ vs Round I |
|---|---:|---|
| Round I L9 fix (commit `e8ca7b0`) | 190977 | – |
| **Round J W 压缩 ON (探针)** | **261627** | **+70650 (+37%)** ❌ |

### Per-layer (Round I baseline → Round J W 压缩)

| L | 描述 | Round I cy | Round J cy | Δ | 倍数 |
|---|---|---:|---:|---:|---:|
| L0 Patch | s2d K=1 cs=4 | 46019 | 46019 | 0 | – |
| L1 K=3 s=2 | | 27709 | 27709 | 0 | – |
| L2 K=3 s=1 | | 19800 | 19800 | 0 | – |
| **L3 ds K=1 s=2** | cs=1 | 10921 | **66640** | **+55719** | **×6.1** ❌ |
| L4 K=3 s=2 | | 11305 | 11305 | 0 | – |
| L5 K=3 s=1 | | 22176 | 22176 | 0 | – |
| **L6 ds K=1 s=2** | cs=1 | 6726 | **18606** | **+11880** | **×2.8** ❌ |
| L7 K=3 s=2 | | 12539 | 12539 | 0 | – |
| L8 K=3 s=1 | | 27364 | 27364 | 0 | – |
| **L9 ds K=1 s=2** | cs=2 | 2972 | **6023** | **+3051** | **×2.0** ❌ |
| L10 FC | | 2927 | 2927 | 0 | – |
| **Total** | | **190977** | **261627** | **+70650** | **+37%** |

**OFM bit-exact PASS** — 数学正确, 性能反向.

### IDMA dispatcher state cy 分解 (Core 0, ResNet11 整 chain 累计)

| state | baseline (W OFF) | W 压缩 ON | Δ |
|---|---:|---:|---:|
| fetch (cmd 串行获取) | 28662 | **66390** | **+37728 (×2.3)** |
| ring_wait (line_buffer 反压) | 31098 | 33618 | +2520 |
| issue (cmd issue 发往 axi_dm) | 1471 | 3991 | ×2.7 |
| data (axi_dm 实际搬运) | 77717 | **108087** | **+30370 (+39%)** |
| **sum** | **138959** | **212097** | **+73138** |

跟 ResNet11 整网 cy +70650 数量级吻合 (dispatcher 占 ConvCore 关键路径).

### IDMA cmd 数膨胀 (per core, ds layers)

| L | 描述 | baseline cmd | W 压缩 cmd | 倍数 |
|---|---|---:|---:|---:|
| L3 (sub_W=33, stride=2) | per_row 1 → 17 | 120 | 2040 | **×17** |
| L6 (sub_W=17, stride=2) | per_row 1 → 8-9 | 60 | 480-540 | ×8-9 |
| L9 (sub_W=8-9, stride=2) | per_row 1 → 4-5 | 30 | 120-150 | ×4-5 |

每 cmd btt 从 chunk_cols × cs × 16 byte (≥ 1 burst, AXI4 8-beat × 16B) 减到 16 byte (单 beat).

## 失败根因

### 1. cmd setup overhead 主导

axi_dm IP 每 cmd ~10-15 cy descriptor fetch + parse + AXI cmd issue overhead. 
单像素 cmd 时这部分占 fetch=66390 cy 主导 (× ~17 cmd 数膨胀).

### 2. data channel 利用率崩塌

axi_dm AXI 通道理论支持 burst 长度 1-256 beat. baseline 每 cmd 跨 ~33 像素 = 多 beat
连续 burst, 通道近满. W 压缩后每 cmd 1 beat (16B), beat 间 turnaround 让 data 时间
反而 +30K cy (本应数据量减半).

### 3. 量纲对比

| 收益 (理论) | 成本 (实测) |
|---|---|
| ds layer 数据量减半 ~5K cy/L3 (理论) | cmd setup 增加 ~50K cy/L3 |
| 净: -5K | 净: +56K |
| **ROI: -1× (反向)** | |

## 论文意义

### 1. 实证 axi_dm IP 颗粒度限制

不是"理论上不可行" — 是"在已知 cmd 颗粒度 / cmd setup overhead 的 IP 上必然失败".

```
single-cmd fetch overhead × cmd 数膨胀 倍数 > 数据量减半的 data 时间收益
```

阈值: 当 cmd 数 / 像素数 > k (axi_dm 上 k ≈ 1-2 像素/cmd 是阈值, 此处单像素远低于阈值).

### 2. 推动硬件 2D 寻址必要性

软件层 trade-off 都试过 (Round I H 压缩 vs Round J W 压缩), 结论是:
- H 压缩: cmd 数不变 (每 cmd 还是连续 burst, 只是 base 跳 stride 行) → ✓ -5.9%
- W 压缩: cmd 数爆炸 → ❌ +37%

**硬件 2D 寻址**(用户 idea) 是把"跳读 pattern"塞到 cmd 编码里, 让 mem 端 SRAM 接口
解码生成寻址序列. 跳过 axi_dm 单 burst 颗粒度限制, 一个 cmd 解决一行 W 维跳读.

ROI 估算: 投入 1-2 周 (改 IDMA/ODMA 协议 + MemCore 自定义解码 + axi_smc USER 透传)
换 ds layer ~50% 数据量减半 ≈ 整网 -2~3%. 比"投半天拿 -37%"靠谱很多.

### 3. 控制变量法验证流程

不预判 "axi_dm cmd 颗粒度多大才能盈亏平衡" — 直接拆 cmd 测 cy. 半天数据出来,
ROI 阴性, 立刻闭环硬件 2D 寻址讨论. **paper 章节: "为什么硬件改造比软件优化更值得"**.

## 数据来源

- Baseline Round I L9 fix: commit `e8ca7b0` (190977 cy)
- Round J 探针 commit: `<待 commit>` (driver only, RTL 0 行, FLUX_W_COMPRESS=1 触发)
- 实验跑 sim: `FLUX_W_COMPRESS=1 python toolchain/run_multicore_chain.py --smc --demo resnet11 --case_name smc_resnet11 --n_cores 4`
  接 `cd sim/tb_smc && vsim -c -do run.tcl` (case 由 active_case.txt 自动确定)
- 还原 baseline: `FLUX_W_COMPRESS=0 ...`

## 下一步选项

| 选项 | 投入 | 预期收益 | ROI |
|---|---|---|---|
| ❌ 软件层 W 压缩 (本探针) | 0.5 天 | -37% (反向) | 强阴性 |
| ⚠️ 算法层 K=1 ds + K=3 conv 合并 (RepVGG) | 训练侧 1 周 | ~16% 整网 | 中 |
| ✓ 硬件 2D 寻址 mem core | 1-2 周 | ~2-3% 整网 | 中 |
| ✓ 算法 + 硬件 2D 联合 | 训练 + 硬件 各一周 | ~16-20% | 高 (但跨域) |

实施推荐: 当前论文阶段不投硬件 2D 寻址 (架构稳定性优先), 把本探针作为
"为什么需要硬件 2D 寻址" 的实证素材写入论文 future work / discussion.
