# paboulink-rtl

Playground for developing OFDM-oriented RTL blocks with a Python-driven simulation flow using cocotb and Verilator.

## Prerequisites
- Python 3.12 (managed by [uv](https://docs.astral.sh/uv/latest/index/))
- [Verilator](https://verilator.org/) available on your PATH (`brew install verilator`, `apt install verilator`, ...)

## Setup
```bash
# create or activate a uv-managed environment
uv venv
source .venv/bin/activate

# install python dependencies (runtime + dev extras)
uv pip install -e ".[dev]"
```

## Running the simulations
```bash
# execute the cocotb-powered pytest suite
uv run pytest
```
The pytest flow drives Verilator through cocotb-test. Generated simulation builds live under `tests/sim_build/`, while diagnostic plots are written to `tests/plots/` for quick inspection.

## Project layout
- `rtl/complex_conjugate.sv` – complex conjugate helper for IQ streams.
- `tests/test_complex_conjugate.py` – randomized cocotb regression for the conjugate block.
- `rtl/nco_cfo_compensator.sv` – dual-channel NCO that removes carrier frequency offset with AXI4-Stream + AXI4-Lite interfaces.
- `tests/test_nco_cfo.py` – cocotb scenarios that cover both steady-state and on-the-fly CFO updates, streaming IQ vectors and persisting comparison plots under `tests/plots/`.

Use this scaffold as a starting point for additional OFDM-oriented RTL blocks and verification infrastructure.
