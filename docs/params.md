# 参数表 (Single Source of Truth)

> 项目根目录的 `params.py` 是 RTL 跟 Python toolchain **唯一**参数定义来源.
> 改完 `params.py` **必须** 跑 `python params.py` 重生成 RTL 头文件.

## 文件关系

```
┌─────────────────────────────┐
│  params.py (根目录)          │   ← Single Source of Truth
│  - NUM_PE / NUM_COL / 等     │
│  - SRAM 容量                 │
│  - AXI 宽度                  │
│  - 全局地址映射 (多核)       │
│  - CSR_ADDR_MAP (cfg_regs)   │
│  - codegen 函数 gen_svh()    │
└───────────┬─────────────────┘
            │
   ┌────────┴───────┐
   │                │
   ▼                ▼
┌──────────────┐  ┌──────────────────────┐
│ Python 端     │  │ RTL 端                │
│ from params   │  │ `include "flux_cnn_   │
│ import *      │  │   params.svh"        │
│              │  │  (跑 gen_svh() 生成) │
│ - hw_files.py │  │                      │
│ - scheduler   │  │ - cfg_regs.sv        │
│ - gen_isa_test│  │ - core_top.sv        │
│              │  │ - multicore_top.sv    │
└──────────────┘  └──────────────────────┘
```

## 改参数流程

```bash
# 1. 改 params.py 中的某个常量
vim params.py            # 例如把 NUM_PE=16 改成 NUM_PE=32

# 2. 重生成 RTL svh
python params.py
# Output: Generated C:/_Project/FLUX_CNN/RTL/flux_cnn_params.svh
#         - Datapath: NUM_PE=32, NUM_COL=16, ...

# 3. 跑回归确认 RTL 跟 Python 同步
cd toolchain && python run_regression.py

# 4. commit
git add params.py RTL/flux_cnn_params.svh
git commit -m "Refactor: 改 NUM_PE 16→32"
```

## 已统一的参数清单

| 类别 | 参数 | RTL 引用 | Python 引用 |
|---|---|---|---|
| Datapath | NUM_PE, NUM_COL, DATA_WIDTH, PSUM_WIDTH, WRF/ARF/PARF_DEPTH | `\`FLUX_NUM_PE` 等 | `from params import NUM_PE` |
| SRAM | IFB_DEPTH, WB_DEPTH, OFB_DEPTH, SHORTCUT_DEPTH | `\`FLUX_IFB_DEPTH` 等 | `from params import IFB_DEPTH` |
| AXI/CSR | BUS_ADDR_W, BUS_DATA_W, AXI_M_ID, ... | `\`FLUX_BUS_DATA_W` 等 | `from params import BUS_DATA_W` |
| 多核地址映射 | CORE_IFB_BASE, CORE_IFB_STRIDE, ... | `\`FLUX_CORE_IFB_BASE` | `from params import core_ifb_axi_base` |
| CSR addr map | 58 个 reg addr (CTRL/STATUS/H_OUT/...) | `\`FLUX_ADDR_CTRL` 等 | `from params import CSR_ADDR_MAP` |

## 例子

### Python 端用法

```python
# toolchain/scheduler.py
from params import NUM_PE, NUM_COL, IFB_DEPTH, OFB_DEPTH
# ...

# toolchain/hw_files.py
from params import CSR_ADDR_MAP, BOOT_REG_ADDRS
addr = CSR_ADDR_MAP['SDP_SHIFT']   # = 0x160
```

### RTL 端用法

```systemverilog
// RTL/cfg_regs.sv
`include "flux_cnn_params.svh"

module cfg_regs #(...) (...);
    localparam [11:0] ADDR_SDP_SHIFT = `FLUX_ADDR_SDP_SHIFT;  // = 12'h160
endmodule

// RTL/core_top.sv
`include "flux_cnn_params.svh"

module core_top #(
    parameter int NUM_PE = `FLUX_NUM_PE,    // 默认 16, 可被实例化时 override
    parameter int IFB_DEPTH = `FLUX_IFB_DEPTH
)(...);
endmodule
```

## 例外: 不在 params.py 的常量

以下硬编码暂时保留 (后续按需迁过来):
- TB-only 默认 DDR layout (在 `tb_*.sv` 内 localparam)
- gen_isa_test.py 内 random seed 等数值
- AXI ID 编码 / SDP 内部位宽分配等次要参数

如果某次需要在 RTL/Python 都改, 优先迁进 `params.py`.

## 验证

`params.py` 改完跑一次:
```bash
python params.py
cd toolchain
python run_regression.py     # 单核 26-case
cd ../sim/tb_multicore
vsim -c -do run.tcl          # M1.5 双核 DDR mode
vsim -c -do run_xcore.tcl    # M2 跨核 chain
```

全 PASS 表示 RTL/Python 同步.
