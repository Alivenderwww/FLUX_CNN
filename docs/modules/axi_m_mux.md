# axi_m_mux

N→1 AXI4 Master 聚合器（同时钟域）。把 4 个 AXI Master（IDMA / WDMA / ODMA / DFE）聚合成一条对外 AXI4 M 接口。本模块自身无寄存器，全组合 mux + 内部 axi_master_arbiter 仲裁器。

## 参数

| 参数 | 含义 | 默认 |
| --- | --- | ---: |
| `M_WIDTH` | log2(masters)，masters = 2^M_WIDTH | 2（→ 4 master） |
| `M_ID` | per-master ID 位宽 | 2 |
| `ADDR_W / DATA_W` | AXI 参数 | 32 / 128 |

## 接口

### N masters 侧（packed 2D 打包）

每个通道是 `[2^M_WIDTH][per-master-bits]` 的 packed 2D 数组：`M_AWID[3:0][1:0]`、`M_AWADDR[3:0][31:0]`、…。N=4 各 master 的 5 个 AXI 通道全部打包在一起。

### 1 bus 侧（对外 AXI4 M）

标准 AXI4 master 信号：`B_AWID[M_ID+M_WIDTH-1:0]` = 4 bit（M_ID=2 + M_WIDTH=2），其余按标准位宽。

## ID 编解码

下行 AW / AR 通道：
```
B_AWID = {master_idx, M_AWID[master_idx]}
B_ARID = {rd_addr_master_sel, M_ARID[rd_addr_master_sel]}
```
master_idx 拼在高位，per-master ID 在低位。

上行 B / R 通道：master_idx 由 arbiter 根据 `B_BID[M_ID +: M_WIDTH]` 译出，per-master ID 取低位 `B_BID[M_ID-1:0]` 写回 `M_BID[master_idx]`。

## 仲裁

例化 `axi_master_arbiter`，给出 5 条通道各自的 master 选择信号：
- `wr_addr_master_sel` + `wr_addr_master_lock`
- `wr_data_master_sel` + `wr_data_master_lock`
- `wr_resp_master_sel`
- `rd_addr_master_sel` + `rd_addr_master_lock`
- `rd_data_master_sel`

仲裁规则：低 index 优先（IDMA[0] > WDMA[1] > ODMA[2] > DFE[3]）。lock 信号在 burst 期间保持 sel 不变。

## 通道 mux

每条通道用 `wr_*_master_sel / rd_*_master_sel` 做 packed array 索引。

AW（举例）：
```
B_AWID    = {wr_addr_master_sel, M_AWID   [wr_addr_master_sel]}
B_AWADDR  =                      M_AWADDR [wr_addr_master_sel]
B_AWLEN   =                      M_AWLEN  [wr_addr_master_sel]
B_AWBURST =                      M_AWBURST[wr_addr_master_sel]
B_AWVALID =                      M_AWVALID[wr_addr_master_sel]

M_AWREADY = '0
M_AWREADY[wr_addr_master_sel] = B_AWREADY
```

W、AR 类似（前者用 wr_data sel、后者用 rd_addr sel）。

B / R 反向：
```
M_BID / M_BRESP / M_BVALID = '0
M_BID[wr_resp_master_sel]   = B_BID[M_ID-1:0]
M_BRESP[wr_resp_master_sel] = B_BRESP
M_BVALID[wr_resp_master_sel]= B_BVALID
B_BREADY = M_BREADY[wr_resp_master_sel]
```

R 同理用 rd_data sel。

## 时序

整个 mux 是组合的，无延迟。AW/AR sel 由 arbiter 在请求拉高时给出，burst 期间 lock 保持。B/R 的 sel 由 arbiter 解码当拍 BUS_*_ID 给出（让 response 路由回正确的 master）。

## 在 core_top 中的位置

实例 `u_axi_mux`：
- 4 个 master 端：`u_idma`(0)、`u_wdma`(1)、`u_odma`(2)、`u_dfe`(3)
- 1 个对外 bus 端：core_top 的 AXI4 M 接口（接 DDR / TB 的 axi_slave）
