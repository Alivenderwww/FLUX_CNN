# VD100 板级 demo Host Runtime

VD100 (Versal AI Edge VE2302) 板上跑 ResNet11 推理 demo 的 host runtime 代码。

## 架构

```
Windows PC                                  ALINX VD100 板 (VE2302)
┌──────────────────┐                     ┌────────────────────────┐
│ resnet11_client  │  TCP socket         │ resnet11_server (PS    │
│   (Python)       │ ◄──────千兆网线────►│   A72 Linux baremetal) │
│                  │                     │                        │
│ - PIL 加载真图   │                     │ - mmap /dev/mem CSR    │
│ - resize 960x540 │                     │ - mmap /dev/mem DDR4   │
│ - INT8 量化      │                     │ - 启动 PL 3 ConvCore   │
│ - 发 IFM         │                     │ - 等 done IRQ          │
│ - 收 OFM         │                     │ - 取 OFM 回 PC         │
│ - top-5 logit    │                     │                        │
└──────────────────┘                     └────────────────────────┘
                                                   │
                                                   ▼
                                       PL multicore_top_vd100
                                       (3 ConvCore + 1to3 demux)
                                                   │
                                                   ▼ Versal NoC
                                            DDR4 4GB (board)
```

## 文件

| 路径 | 说明 |
|---|---|
| `vd100_pc/resnet11_client.py` | Windows 上位机 Python client (PIL + socket) |
| `vd100_ps/resnet11_server.c` | VD100 板 PS A72 Linux server (mmap + socket) |

## 协议 (TCP, port 5000)

```
Request:                          Response:
  magic   uint32 'FXNN'             magic   uint32 'FXNN'
  cmd     uint32 = 1 (INFER)        status  uint32 (0=OK)
  ifm_len uint32 = 2,073,600        cy      uint32 (HW cycles)
  ifm     bytes (int8 [-128,127])   ofm_len uint32 = 522
                                    ofm     bytes (int8 logit)
```

## 用法

### 上位机 (Windows PC)

```cmd
cd C:\_Project\FLUX_CNN\host\vd100_pc
:: 装依赖 (一次)
pip install Pillow

:: 推理
python resnet11_client.py ^
    --image C:\_Project\FLUX_CNN\toolchain\models\images\resnet11_test\gradient.png ^
    --vd100-ip 192.168.1.100 ^
    --port 5000 ^
    --repeat 10
```

### VD100 板 (PS A72 Linux)

```bash
# 上位机 cross-compile (用 Petalinux SDK)
cd C:\_Project\FLUX_CNN\host\vd100_ps
aarch64-linux-gnu-gcc -O2 -o resnet11_server resnet11_server.c

# 部署到板上
scp resnet11_server root@192.168.1.100:/home/root/

# ssh 上板启动
ssh root@192.168.1.100
./resnet11_server 5000
# [server] listen on 0.0.0.0:5000
```

## 性能预期

| 阶段 | 时间 | 说明 |
|---|---|---|
| PC PIL 加载 + resize | ~50 ms | 一次性 |
| TCP IFM 上传 (2 MB / 1 Gbps) | ~25 ms | 千兆网理论上限 |
| PL 3 核 ResNet11 推理 (sim 估) | ~2.3 ms | 230K cy @ 100 MHz |
| TCP OFM 下载 (522 byte) | <1 ms | 极小 |
| **端到端 RTT** | **~80 ms** | **= ~12 FPS demo** |

如果连续 inference (复用 IFM upload), 板内推理~2.3ms = **~430 FPS** (HW 计算上限)。

## 已知 TODO

`resnet11_server.c` 是 demo 骨架, 板上跑通需要:

1. **CSR 地址校准**: `CSR_BASE_PHYS = 0xA0000000` 是 BD 设的 PS GP aperture, 跟
   `Syn/vd100_bd/pl_config.tcl` 里 `assign_bd_address` 一致. 板级 P&R 后可能要
   re-check.
2. **DDR layout 匹配 driver**: `DDR_DESC_OFFSET / DDR_WEIGHT_OFFSET / DDR_OFM_OFFSET`
   需要跟 toolchain/run_multicore_chain.py 的 DDR planner 输出对齐. 当前是占位值.
3. **desc_list / weight 预加载**: server 启动时一次性把 ResNet11 desc list +
   weight 从 SD 卡 / EMMC 加载到 PS DDR (跟 driver 输出的 hex 文件对齐). 这部分
   server 还没实现, 需要加 init 阶段.
4. **PS UIO IRQ 替代 polling**: 当前 server 用 busy poll status[0]=DONE 等 PL,
   实际生产用 epoll /dev/uio0 等中断, 节省 CPU.
5. **千兆网 PHY 延时配置**: ALINX 文档 3_06 说 ETH2 接 PL EMIO 需要 PHY 延时
   调整 (mdiorw 工具). 我们用 ETH0 (PS 直连, 标准) 不需要.

## 跟 main 分支 K325T 的差异

| 维度 | main (K325T) | vd100-demo (VE2302) |
|---|---|---|
| 板 | 没板, 只 sim+P&R 数据 | ALINX VD100 (实际板) |
| Host 接口 | (无) | 千兆以太网 + Windows PC |
| host runtime | (无) | C (PS Linux) + Python (PC) |
| paper 数据 | sim cy + P&R LUT/DSP/Fmax | sim cy + OOC LUT/DSP/Fmax + 真实 demo (待板级 bring-up) |
