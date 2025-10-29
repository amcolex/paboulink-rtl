# Minn Preamble Detector Implementation Plan

## File Structure

### RTL Files (in `rtl/`)
1. `minn_running_sum.sv` - Generic running sum accumulator with FIFO
2. `minn_delay_line.sv` - Parameterized delay line (for Q, 2Q delays)
3. `minn_antenna_datapath.sv` - Complete per-antenna signal processing
4. `minn_gate_peak_fsm.sv` - Gate-and-peak detection state machine
5. `minn_output_buffer.sv` - Dual-port circular buffer with flag back-patch
6. `minn_preamble_detector.sv` - Top-level integration module

### Test Files (in `tests/`)
1. `test_minn_detector.py` - Main cocotb test with plotting
2. `ofdm_utils.py` - OFDM symbol generation utilities
3. `minn_preamble.py` - Minn preamble generation ([A A A A] → [A A -A -A])

### Documentation Files (in `docs/`)
1. ✓ `minn_design.md` - Original design specification (provided)
2. ✓ `minn_detector_architecture.md` - Architecture specification (created)
3. ✓ `minn_implementation_plan.md` - This file

## Implementation Order and Rationale

### Phase 1: Basic Building Blocks
**Goal**: Create and verify fundamental components

1. **`minn_delay_line.sv`** (simplest)
   - Parameterized shift register
   - Can be used for both simple delays and FIFO applications
   - Easy to verify independently
   
2. **`minn_running_sum.sv`** (moderate)
   - Uses delay_line internally
   - Implements "subtract old, add new" sliding window
   - Verify with known input sequences
   
**Verification**: Simple directed tests, waveform inspection

### Phase 2: Complex Datapaths
**Goal**: Implement per-antenna processing pipeline

3. **`minn_antenna_datapath.sv`** (complex)
   - Integrates delay_line and running_sum
   - Implements quarter-lag correlation and power computation
   - Multiple pipeline stages
   - Most complex datapath logic
   
**Verification**: Feed known preamble patterns, verify correlation peaks

### Phase 3: Control Logic
**Goal**: Implement detection decision logic

4. **`minn_gate_peak_fsm.sv`** (moderate)
   - State machine with hysteresis
   - Peak tracking logic
   - Flag address computation
   
**Verification**: Stimulate with synthetic metric streams, verify state transitions

### Phase 4: Output Management
**Goal**: Implement delayed output with frame synchronization

5. **`minn_output_buffer.sv`** (moderate)
   - Dual-port memory with circular addressing
   - Back-patch flag writing
   - Continuous read stream
   
**Verification**: Write pattern, read after delay, verify flag placement

### Phase 5: Integration
**Goal**: Complete system integration and testing

6. **`minn_preamble_detector.sv`** (integration)
   - Instantiate two antenna datapaths
   - Combine metrics
   - Integrate FSM and output buffer
   - Connect all signals
   
**Verification**: End-to-end tests with real OFDM preambles

### Phase 6: Python Test Infrastructure
**Goal**: Create comprehensive verification environment

7. **`ofdm_utils.py`**
   - IFFT/FFT functions
   - Subcarrier mapping
   - Cyclic prefix addition
   - AWGN channel simulation
   
8. **`minn_preamble.py`**
   - Generate QPSK symbols on every 4th subcarrier
   - Create [A A A A] pattern
   - Apply sign inversion for [A A -A -A]
   - Package as complete preamble
   
9. **`test_minn_detector.py`**
   - cocotb testbench
   - Multiple test scenarios (perfect, noisy, CFO, multipath)
   - Plotting of metrics, energy, detections
   - Performance statistics

## Key Design Decisions

### 1. Modular vs Monolithic
**Decision**: Modular design with separate files for each major block

**Rationale**:
- Easier to verify each component independently
- Better code reuse (delay_line, running_sum are generic)
- Clearer design intent and documentation
- Follows FPGA design best practices from verilog_guide.md

### 2. Synchronous vs Asynchronous Reset
**Decision**: Synchronous reset with initial values

**Rationale**:
- Per verilog_guide.md, FPGA flip-flops have asynchronous reset hardware but should be driven synchronously
- Use `initial` blocks for register initialization
- Reduces reset tree size and improves timing
- Matches project coding standard

### 3. Pipeline Register Placement
**Decision**: Pipeline stages at natural boundaries (after adder trees, before FSM)

**Rationale**:
- Keeps critical paths manageable
- Adder trees in combiner are likely critical path
- FSM logic benefits from registered inputs
- Threshold multiply needs isolation

### 4. Fixed-Point Arithmetic
**Decision**: All arithmetic is signed, with explicit bit width tracking

**Rationale**:
- Matches design specification calculations
- Prevents overflow surprises
- Clear documentation via localparam width definitions
- Saturating arithmetic where necessary

### 5. Memory Implementation
**Decision**: Use BRAM inference for large FIFOs and output buffer

**Rationale**:
- Q=512 is too large for SRL efficiently
- BRAM is plentiful on modern FPGAs
- Dual-port BRAM enables concurrent flag write
- Synthesis tools will infer correctly with proper coding

## Coding Standards Summary

### From verilog_guide.md:
1. ✓ Use `logic` instead of `reg` (SystemVerilog)
2. ✓ `` `default_nettype none`` at start of each file
3. ✓ All parameters have default values
4. ✓ Initialize all registers via `initial` blocks or at declaration
5. ✓ Use localparam for derived constants
6. ✓ Parameterize all bit widths
7. ✓ Use blocking `=` in combinational blocks
8. ✓ Use non-blocking `<=` in sequential blocks
9. ✓ Use ternary operators instead of if/else where possible
10. ✓ Define all signals before use (top-to-bottom)
11. ✓ No `assign` statements except for module output wiring
12. ✓ Boolean expressions as explicit comparisons

### Project-Specific:
1. All widths defined as localparams with clear names (W_IN, W_R, W_E, etc.)
2. Use signed arithmetic explicitly throughout
3. Comment all non-obvious bit width calculations
4. Include synthesis directives for BRAM inference where appropriate
5. Add debug outputs for key signals (metric, energy, above_thresh)

## Test Strategy

### Unit Tests (per module)
- **delay_line**: Verify delay amount, initial state
- **running_sum**: Feed known sequences, verify accumulation
- **antenna_datapath**: Feed orthogonal signals, verify correlation = 0
- **fsm**: Synthetic metric input, verify state transitions and hysteresis
- **output_buffer**: Write/read patterns, verify delay and flag placement

### Integration Tests
1. **Perfect Preamble**: Clean signal, verify single clean detection at correct time
2. **Noisy Preamble**: Add AWGN, sweep SNR, measure Pd (detection probability)
3. **CFO Test**: Apply frequency offset (±5% of subcarrier spacing), verify detection degrades gracefully
4. **False Alarm Test**: Random noise only, verify no false detections
5. **Timing Accuracy**: Verify frame_start flag aligns to expected sample

### Performance Metrics
- Detection probability vs SNR
- False alarm rate vs threshold
- Detection delay (latency from preamble start to flag)
- Metric peak value vs signal strength
- Sensitivity to CFO

### Visualization
For each test, generate plots:
1. Input I/Q time series
2. Metric vs time (C_smooth)
3. Energy vs time (E_tot)
4. Threshold vs time (T × E_tot)
5. Gate state vs time
6. Frame flag markers
7. Spectrogram of input (optional)

## Implementation Checklist

### Per File:
- [ ] Add `` `default_nettype none`` header
- [ ] Define all parameters with defaults
- [ ] Calculate all localparams for bit widths
- [ ] Initialize all registers
- [ ] Add comprehensive comments
- [ ] Include synthesis attributes where needed
- [ ] Add debug outputs
- [ ] Write unit test
- [ ] Verify in simulation
- [ ] Review against verilog_guide.md

### Integration:
- [ ] Connect all sub-modules
- [ ] Verify bit width matching at all interfaces
- [ ] Add top-level debug outputs
- [ ] Verify no combinational loops
- [ ] Check for unintentional latches
- [ ] Run full system test with perfect preamble
- [ ] Run noise tests
- [ ] Generate plots
- [ ] Document any deviation from spec

## FPGA Resource Estimation

Based on architecture specification:

### Memory
- Delay lines: ~90 Kbits × 2 antennas = 180 Kbits
- Output buffer: ~100 Kbits
- **Total: ~280 Kbits** (should fit easily in most FPGAs)

### DSP Slices
- 4 multipliers × 2 antennas = 8 DSP
- 1 threshold multiplier = 1 DSP
- **Total: 9 DSP slices**

### LUTs (estimated)
- Adder trees: ~500 LUTs
- FSM: ~100 LUTs
- Control logic: ~200 LUTs
- **Total: ~800 LUTs** (very small)

### Flip-Flops (estimated)
- Pipeline registers: ~500 FFs
- FSM and control: ~200 FFs
- **Total: ~700 FFs**

**Target**: Should fit comfortably on small FPGAs (Artix-7 35T or equivalent)

## Risk Mitigation

### Potential Issues and Solutions

1. **Timing Closure**
   - Risk: Adder tree too deep
   - Mitigation: Add pipeline stage after combiner if needed
   
2. **Memory Inference**
   - Risk: Synthesis doesn't infer BRAM
   - Mitigation: Use specific coding patterns, check synthesis reports
   
3. **Bit Width Overflow**
   - Risk: Accumulator overflow in running sums
   - Mitigation: Use saturating arithmetic, verify with worst-case inputs
   
4. **CFO Sensitivity**
   - Risk: Detection fails with high CFO
   - Mitigation: This is expected per spec; document threshold adjustment needed
   
5. **False Alarms**
   - Risk: Noise triggers false detections
   - Mitigation: Tune threshold, increase hysteresis if needed

## Success Criteria

The implementation is complete when:
1. ✓ All modules synthesize without errors or warnings
2. ✓ Unit tests pass for all sub-modules
3. ✓ Integration test detects perfect preamble at correct time
4. ✓ Detection probability > 95% at SNR = 10 dB
5. ✓ False alarm rate < 1% with noise-only input
6. ✓ Metric plots match expected behavior from design spec
7. ✓ Frame flag timing verified against OFDM symbol structure
8. ✓ Resource usage within estimates
9. ✓ Code follows all guidelines from verilog_guide.md
10. ✓ Documentation complete and accurate

## Next Steps for Implementation

Ready to proceed to Code mode for implementation. The architecture is fully specified, parameters are confirmed, and implementation strategy is clear.

**Recommended sequence**:
1. Start with `minn_delay_line.sv` - simplest building block
2. Move to `minn_running_sum.sv` - uses delay_line
3. Implement `minn_antenna_datapath.sv` - integrates both
4. Create FSM and output buffer in parallel
5. Integrate in top-level module
6. Build Python test infrastructure
7. Run comprehensive verification
8. Generate plots and analyze results

Each step builds on verified previous steps, reducing integration risk.