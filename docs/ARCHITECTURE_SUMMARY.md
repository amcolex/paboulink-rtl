# Minn Preamble Detector - Architecture Summary

## Overview

This document summarizes the architectural planning for the OFDM Minn preamble detector implementation.

## Design Confirmed Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| NFFT | 2048 | FFT size |
| Q | 512 | Quarter length (NFFT/4) |
| CP | 512 | Cyclic prefix length |
| Active Subcarriers | 1200 | Every 4th subcarrier |
| W_IN | 12 bits | Input I/Q bit width |
| THRESHOLD | 0x1800 | Q1.15 format (~0.09375) |
| IIR_SHIFT | 3 | Smoother shift amount |
| HYSTERESIS | 2 samples | Gate close hysteresis |
| TIMING_OFFSET | 0 | Flag alignment offset |
| OUTPUT_DELAY | 2048 | Output FIFO delay (= NFFT) |

## Module Structure

```
minn_preamble_detector (top-level)
├── minn_delay_line (×6 instances - delays and FIFOs)
├── minn_running_sum (×4 instances - 2 per antenna for R and E)
├── minn_antenna_datapath (×2 instances - one per antenna)
├── minn_gate_peak_fsm (×1 instance - detection logic)
└── minn_output_buffer (×1 instance - delayed output with flags)
```

## Files to Create

### RTL Files
1. [`rtl/minn_delay_line.sv`](rtl/minn_delay_line.sv) - Parameterized shift register
2. [`rtl/minn_running_sum.sv`](rtl/minn_running_sum.sv) - Sliding window accumulator
3. [`rtl/minn_antenna_datapath.sv`](rtl/minn_antenna_datapath.sv) - Per-antenna processing
4. [`rtl/minn_gate_peak_fsm.sv`](rtl/minn_gate_peak_fsm.sv) - Detection state machine
5. [`rtl/minn_output_buffer.sv`](rtl/minn_output_buffer.sv) - Circular buffer with flag back-patch
6. [`rtl/minn_preamble_detector.sv`](rtl/minn_preamble_detector.sv) - Top-level integration

### Python Test Files
1. [`tests/ofdm_utils.py`](tests/ofdm_utils.py) - OFDM symbol generation utilities
2. [`tests/minn_preamble.py`](tests/minn_preamble.py) - Minn preamble generator
3. [`tests/test_minn_detector.py`](tests/test_minn_detector.py) - cocotb testbench with plots

## Key Design Decisions

1. **Modular Architecture**: Separate files for each major functional block for better testability and code reuse
2. **Synchronous Design**: All logic uses synchronous reset with `initial` block initialization per verilog_guide.md
3. **Fixed-Point Arithmetic**: All widths explicitly calculated and documented as localparams
4. **BRAM Inference**: Large FIFOs and output buffer designed to infer block RAM
5. **Pipeline Stages**: Natural register boundaries to keep timing clean

## Resource Estimates

- **Memory**: ~280 Kbits (primarily delay lines and output buffer)
- **DSP Slices**: 9 (8 for correlations/power, 1 for threshold multiply)
- **LUTs**: ~800 (adder trees, control logic)
- **Flip-Flops**: ~700 (pipeline registers, FSM)

**Target**: Should fit comfortably on Artix-7 35T or equivalent

## Critical Bit Widths

```
W_IN = 12            // Input I/Q
W_w = 25             // Power (I² + Q²)
W_p = 25             // Quarter-lag product
W_E = 34             // Running sum of power (per antenna)
W_R = 34             // Running sum of product (per antenna)
W_E_tot = 37         // Total energy (3 quarters, 2 antennas)
W_C = 36             // Total correlation (2 quarters, 2 antennas)
```

## Test Strategy

### Verification Levels
1. **Unit Tests**: Each sub-module tested independently
2. **Integration Tests**: Complete detector with various scenarios
3. **Performance Tests**: SNR sweeps, CFO sensitivity

### Test Scenarios
- Perfect preamble (clean detection)
- Noisy preamble (AWGN, SNR sweep)
- CFO test (frequency offset)
- False alarm test (noise only)
- Timing accuracy verification

### Plots Generated
- Input I/Q time series
- Metric vs time (C_smooth)
- Energy vs time (E_tot)
- Threshold comparison
- Gate state timeline
- Frame flag markers

## Implementation Sequence

**Phase 1**: Basic building blocks
- `minn_delay_line.sv`
- `minn_running_sum.sv`

**Phase 2**: Complex datapaths
- `minn_antenna_datapath.sv`

**Phase 3**: Control logic
- `minn_gate_peak_fsm.sv`

**Phase 4**: Output management
- `minn_output_buffer.sv`

**Phase 5**: Integration
- `minn_preamble_detector.sv`

**Phase 6**: Python test infrastructure
- `ofdm_utils.py`
- `minn_preamble.py`
- `test_minn_detector.py`

## Documentation Created

1. ✅ [`docs/minn_design.md`](docs/minn_design.md) - Original specification (provided)
2. ✅ [`docs/verilog_guide.md`](docs/verilog_guide.md) - Coding standards (provided)
3. ✅ [`docs/minn_detector_architecture.md`](docs/minn_detector_architecture.md) - Detailed architecture
4. ✅ [`docs/minn_implementation_plan.md`](docs/minn_implementation_plan.md) - Implementation strategy
5. ✅ [`docs/ARCHITECTURE_SUMMARY.md`](docs/ARCHITECTURE_SUMMARY.md) - This document

## Coding Standards Checklist

All code will follow [`docs/verilog_guide.md`](docs/verilog_guide.md):

- ✅ `` `default_nettype none`` at start of each file
- ✅ Use `logic` instead of `reg` (SystemVerilog)
- ✅ All parameters have default values
- ✅ Initialize all registers via `initial` blocks
- ✅ Use localparam for all derived constants
- ✅ Parameterize all bit widths (no hardcoded integers)
- ✅ Blocking `=` in combinational, non-blocking `<=` in sequential
- ✅ Ternary operators instead of if/else where possible
- ✅ Boolean expressions as explicit comparisons
- ✅ No `assign` statements except module output wiring
- ✅ Signed arithmetic throughout with explicit types

## Ready for Implementation

All architectural planning is complete. The design is fully specified with:
- ✅ Confirmed parameters
- ✅ Complete module hierarchy
- ✅ Calculated bit widths
- ✅ Memory and resource estimates
- ✅ Detailed implementation plan
- ✅ Test strategy
- ✅ Coding standards

**Next Step**: Switch to Code mode to implement the SystemVerilog modules and Python test infrastructure.

---

**Architecture Phase Completed**: 2025-10-29
**Ready for Implementation**: Yes
**Estimated Implementation Time**: 6-8 modules + 3 test files