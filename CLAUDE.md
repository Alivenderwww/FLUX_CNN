# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FLUX_CNN is a CNN accelerator in SystemVerilog. A 16×16 int8 MAC array (256 MACs) is orchestrated by a **configuration-register driven FSM** — software writes a flat register map (`H_OUT`, `W_OUT`, `K`, `STRIDE`, tile/cin/cout/round step values, SRAM base addresses, …) and pulses `start`; hardware autonomously walks the 6-level loop `cs → yout → tile → cins → round → (ky,kx)` to completion. No fetch, no decode, no instruction stream.

An earlier Macro-ISA variant (with `INST_SRAM`, `OP_LD32MAC`, etc.) existed up through commit `7a4d616` and is gone — the `docs/isa-legacy.md` file records it for historical context. Treat the cfg-driven flow as canonical. A Python "compiler" (`gen_isa_test.py` — kept the filename, now a **config generator**) writes `config.txt` + test vectors; the RTL is exercised in ModelSim; synthesis targets Xilinx UltraScale+ (`xcku060`).

The core architectural commitment: **2D image iteration, arbitrary K×K kernels, stride, channel tiling, and WRF round chunking are all scheduled in hardware (the FSM) from pre-computed step sizes**. Software provides no runtime sequencing — every address is produced by `running_pointer += STEP` (pure addition, zero hardware multipliers on the critical path; one small shift `round_cnt*32` and one ≤3-bit `sub_cnt * eff_stride` are the only exceptions).

## Common Commands

All simulation commands **must be run from `sim/tb_core_isa/`** (the TCL and `.f` files use relative paths).

```bash
cd sim/tb_core_isa

# 1. Generate test vectors + config + golden outputs
python gen_isa_test.py --k 3 --h_in 68 --w_in 120 --stride 1 --num_cin 8 --num_cout 8 --shift 0
# Flags: --k --h_in --w_in --stride --num_cin --num_cout --tile_w (keep 32) --shift --seed
# Produces: ifb.txt  wb.txt  config.txt  expected_ofm.txt  sim_params.f

# 2. Run simulation (batch, no waveform)
vsim -c -do tb_core_isa.tcl
# Drop -c (or use do.bat) to open the GUI.

# 3. Run the full regression suite
python run_regression.py --label cfg-driven --out regression_cfg_driven.txt
```

Synthesis (from `Syn/`, requires Vivado in PATH):

```bash
vivado -mode tcl -source run_syn.tcl -log syn_run.log -nojournal
```

There is **no build system, lint config, or unit-test harness** beyond the ModelSim TB — verification is directed tests + golden OFM comparison written by `gen_isa_test.py`.

## Architecture

### Data Path (top = `core_top`)

```
cfg_regs (poked by TB)  ──►  core_ctrl (6-loop FSM) ──► drives everything below
IFB (64b × SRAM_DEPTH)  →  ARF (32-deep act RF)  →  mac_array (16 col × 16 PE)
                                                    │  each PE has WRF (std_rf, 32-deep ×8b)
                                                    │  each col has PARF (32-deep × 32b)
                                                    ▼
WB (2048b × SRAM_DEPTH) → WRF fan-out (broadcast)
PARF → SDP (shift + ReLU + clip) → OFB (128b × SRAM_DEPTH)
```

Key modules (in `RTL/`):

- **`core_ctrl.sv`** — Configuration registers + self-driving FSM. States: `IDLE → SHIFT_WR → LOAD_WGT → SUBOP → ST_OFM → DONE`. Inside `SUBOP`, an internal `sub_cnt` counter runs 0..`mac_len` to drive a single LDnMAC-equivalent beat (cnt=0 IFB prefetch, cnt=1..mac_len overlapping IFB-load + MAC + PARF-accum). Outer loop counters `cs_cnt / yout_cnt / tile_cnt / cins_cnt / round_cnt / pos_in_round / ky_cnt / kx_cnt` advance on `sub_cnt == mac_len`. **TB writes cfg registers directly via hierarchical reference (`u_core_top.u_ctrl.cfg_*`) — no external bus.**
- **`core_top.sv`** — Module wiring + LD32MAC-style **bypass MUX** that forwards `ifb_rdata` around ARF when `arf_waddr == arf_read_addr` (kills the 1-cycle read-after-write hazard).
- **`mac_array` / `mac_col` / `mac_pe`** — 16×16 int8 MACs (parameters `NUM_COL=NUM_PE=16`). Activations broadcast across columns; each column accumulates one output channel into its PARF.
- **`std_rf.sv`** — generic synchronous RF (combinational read) used as WRF / ARF / PARF.
- **`sdp.sv`** — `shift_amt` configurable; dequant right-shift → ReLU → saturate to uint8.
- **`sram_model.sv`** — behavioral 1-cycle-read SRAM. All three buffers (IFB/WB/OFB) are instances. Synthesis maps to RAMB36 (the dominant area).

### Configuration register map (see `docs/config-registers.md`)

Fields fall into five groups; **all derived values are pre-computed by `gen_isa_test.py`** so the FSM only ever adds:

| Group | Fields |
|-|-|
| Loop bounds | `H_OUT`, `W_OUT`, `W_IN`, `K`, `STRIDE`, `CIN_SLICES`, `COUT_SLICES`, `TILE_W`, `NUM_TILES`, `LAST_VALID_W` |
| Weight scheduling | `TOTAL_WRF`, `WRF_PACKED`, `KK`, `ROUNDS_PER_CINS`, `ROUND_LEN_LAST` |
| Address bases (20-bit) | `IFB_BASE`, `WB_BASE`, `OFB_BASE` |
| Address steps (20-bit) | `IFB_CIN_STEP = H_IN·W_IN`, `IFB_ROW_STEP = stride·W_IN`, `WB_CIN_STEP = K²`, `WB_COUT_STEP = K²·cin_slices`, `OFB_COUT_STEP = H_OUT·W_OUT`, `TILE_IN_STEP = TILE_W·stride` |
| SDP | `SDP_SHIFT`, `SDP_RELU_EN` |

Scalar-register `r0/r1` conventions from the ISA era are **gone** — addresses come from `IFB_BASE / OFB_BASE + running_pointer`.

### Three data-reuse mechanisms (don't break these when editing ctrl/array)

1. **Weight Stationary** — for `K²·cin_slices ≤ 32` (packed), WRF is loaded once per `cs` and reused across the whole `yout × tile × cins × ky × kx` sweep. Otherwise (chunked), WRF reloads per `(cins, round)`.
2. **ARF sliding window (stride=1 + full tile only)** — `kx=0` loads `TILE_W` pixels via 32 IFB reads; `kx=1..K-1` each load one new pixel, MAC reads a slid window from ARF. The 5-bit ARF wrap (ARF_DEPTH=32) is load-bearing — do not change ARF_DEPTH without rethinking `sub_arf_rd_base`/`sub_arf_wr_base`. Partial tiles (`valid_w < TILE_W`) and `stride > 1` fall back to full reloads.
3. **Output-channel broadcast** — one 64-bit `act_to_mac` drives all 16 columns simultaneously; each column accumulates one output channel.

### Round chunking (K² > WRF_DEPTH)

For K≥7 (K²=49 or larger), the K² kernel taps of one `cins` can't fit in the 32-deep WRF, so `ROUNDS_PER_CINS > 1`:

- Within a `cins`, kernel positions are chunked 32 at a time. Each round = one `LOAD_WGT` (32 weights except possibly the last round) + 32 `SUBOP`s.
- `round_cnt` and `pos_in_round` track the current round and position within it; `ky_cnt / kx_cnt` advance across round boundaries **without reset** (kernel coordinates are global within a cins).
- PARF is **not** cleared between rounds — accumulation continues.
- `wrf_raddr = r_wrf_base + pos_in_round` (packed: `r_wrf_base = cins·K²`; chunked: `r_wrf_base = 0`, restarts per LOAD_WGT).

### Pipeline-delay discipline (critical when editing `core_ctrl`)

Control signals to downstream stages are explicitly delayed to match datapath latency:

- `wrf_we_d1` (1 cycle, matches WB SRAM read)
- `arf_we_d1` (1 cycle, matches IFB SRAM read)
- `ofb_we_d2` (2 cycles, matches `parf_addr` → `parf_addr_d1` → `parf_addr_reg` pipeline inside `mac_col` + SDP combinational)

`SUBOP` runs for `mac_len + 1` cycles (`mac_len == valid_w`). All counter advances fire on `sub_cnt == mac_len`.

## Project Conventions

- Language: **SystemVerilog (IEEE 1800)** — packed structs, enums, no packages (the ISA package was deleted). Keep `+incdir+../../RTL` and add new files to both `sim/tb_core_isa/sim_file_list.f` and `Syn/run_syn.tcl` when introducing modules.
- Parameter defaults: `NUM_COL=NUM_PE=16`, `WRF_DEPTH=ARF_DEPTH=PARF_DEPTH=32`, `DATA_WIDTH=8`, `PSUM_WIDTH=32`, `SRAM_DEPTH=8192` (TB overrides at elab time via `-gSRAM_DEPTH=...` from `sim_params.f`).
- Internal address width: `ADDR_W=20` (1M words reach) — lifted from 16-bit when enabling 224×224 single-channel full-feature-map storage. Hard ceiling: `PYTHON_SRAM_LIMIT=65536` in `gen_isa_test.py`.
- `gen_isa_test.py` is the source of truth for derived-value arithmetic (step sizes, round lengths, OFM dims). When changing cfg-register semantics, update the Python generator in lockstep.
- Regression baselines live as checked-in `regression_*.txt` files. `regression_cfg_driven.txt` is the current reference; `regression_baseline.txt / 16x16.txt / cslice.txt / pipeline.txt / subchan.txt` are ISA-era snapshots kept for historical comparison.
- Commits use Chinese prefixes (`Feat:`, `Docs:`, etc.). Match the existing style.

## Further Reading

- `README.md` — top-level narrative + TOC
- `docs/architecture.md` — module hierarchy, datapath diagrams, reuse strategies
- `docs/config-registers.md` — full register map with bit widths and software pre-computation rules
- `docs/fsm-pipeline.md` — FSM states, loop nesting, pipeline-delay timing diagrams
- `docs/simulation.md` — how to run single tests and regressions, TB cfg-poke mechanism
- `docs/synthesis.md` — Vivado OOC flow, resource/timing/power numbers
- `docs/isa-legacy.md` — the decommissioned Macro-ISA flow (64-bit instruction format, `INST_SRAM`, `OP_LDnMAC`) for historical reference
- `docs/roadmap.md` — phase 2+ plans (padding, residual, multi-core scheduling)
- `model_analysis.md` — target-model PE utilization analysis
- `history.md` — design-evolution conversation log (contains the ISA→cfg-driven transition discussion)
