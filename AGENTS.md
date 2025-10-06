# Agent Setup Guide

This project targets a Python-driven RTL verification flow using cocotb and Verilator. Follow the steps below to prep your local environment and run the regression suite that drives the simulations.

## 1. Prerequisites
- Python 3.12 (we recommend managing it with [uv](https://docs.astral.sh/uv/latest/))
- Verilator available on your `PATH` (e.g. `brew install verilator`, `apt install verilator`, …)

## 2. Environment bootstrap
```bash
# create the virtual environment (stored in .venv/)
uv venv

# activate the environment for your shell session
source .venv/bin/activate

# install runtime + dev dependencies in editable mode
uv pip install -e ".[dev]"
```
The editable install keeps Python modules in sync with the working tree, while the `[dev]` extra pulls in the pytest helpers used by the regression suite.

## 3. Running the tests
```bash
# execute the cocotb-backed pytest suite
uv run pytest
```
The test flow invokes Verilator through `cocotb-test`, generating build artifacts under `tests/sim_build/` and diagnostic plots under `tests/plots/`. Remove those directories when you need a clean slate.

## 4. Useful variations
- Parallelize the pytest run when you have many simulations: `uv run pytest -n auto`
- Rebuild Verilator outputs from scratch: delete `tests/sim_build/` before running pytest again
- Capture verbose cocotb logs: `uv run pytest -s`

With the environment prepared, you can iterate on RTL modules in `rtl/` and drive new verification scenarios from `tests/`.
