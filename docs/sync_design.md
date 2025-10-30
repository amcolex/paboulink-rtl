Below is the standalone hardware specification for the **OFDM frame synchronizer / scheduler** that lives immediately downstream of the Minn preamble detector. The block consumes the dual-antenna AXI4-Stream used across the chain, strips cyclic prefixes, assigns symbol indices, and time-schedules future frames once a single start-of-frame event has been observed.

> TL;DR for the RTL engineer:
> • One AXI4-Stream in, one out; packed `{ch1_q, ch1_i, ch0_q, ch0_i}` with 1 sample/clk.  
> • On the **first Minn `sof` pulse**, latch timing; thereafter, **ignore detector flags** and run off fixed counters.  
> • In `COLLECTING`, only forward the **NFFT payload samples** for each symbol; drop every CP locally.  
> • Output `TUSER[6:0] = symbol_index` (0-based) for every payload sample; `TLAST` fires on the final payload sample of a frame.  
> • Between frames, `TRACKING` counts a programmable gap and applies optional ±128-sample STO tweaks before re-entering collection.  
> • Reset returns to `SEARCHING`, clears counters, and re-arms the detector interface.

---

## 1) Placement and context

```
            ┌────────────────────┐   AXIS (dual I/Q, sof flag)
RF front →  │ minn_preamble_det  │ ───────────────────────────► this_block
            └────────────────────┘
                                              │
                                              │ AXIS (dual I/Q, symbol index)
                                              ▼
                                        Downstream FFT / mapper
```

* The Minn detector asserts `s_axis_tuser[0] = 1` on the **first payload sample** of the first OFDM symbol after the preamble (CP already consumed).  
* This synchronizer observes that single pulse, locks timing, and then **free-runs** using fixed frame geometry.  
* Downstream logic receives only payload samples (no CP) plus a 7-bit symbol counter that wraps at the frame boundary.

---

## 2) External interface

| Signal | Dir | Width | Description |
|--------|-----|-------|-------------|
| `clk` | in | 1 | Fabric clock (same domain as Minn detector). |
| `rst` | in | 1 | Active-high synchronous reset; clears state machine, counters, and output pipeline. |
| `s_axis_tdata` | in | `4*INPUT_WIDTH` | Packed `{ch1_q, ch1_i, ch0_q, ch0_i}` samples from Minn detector. |
| `s_axis_tuser` | in | 1 | Bit0 is Minn frame-start pulse; ignored after first lock. |
| `s_axis_tvalid` | in | 1 | AXI4-Stream valid. |
| `s_axis_tready` | out | 1 | Always 1 once reset is released; the block never back-pressures the detector. |
| `s_axis_tlast` | in | 1 | Optional; if asserted, treated as metadata and not required for scheduling. |
| `m_axis_tdata` | out | `4*OUTPUT_WIDTH` | Forwarded dual-channel payload samples; width matches input. |
| `m_axis_tuser` | out | 7 | Symbol index (0..`N_FRAME_SYMBOLS-1`). |
| `m_axis_tvalid` | out | 1 | High on payload samples while the downstream is ready. |
| `m_axis_tready` | in | 1 | AXI4-Stream ready from consumer. |
| `m_axis_tlast` | out | 1 | Pulsed on the final payload sample of each frame. |
| `sto_correction` | in | 8 | Signed sample-time offset (two’s complement) captured in `TRACKING`. |
| `sto_valid` | in | 1 | One-cycle strobe indicating `sto_correction` is valid. |
| `sto_ready` | out | 1 | Block can accept a new STO command (high in `TRACKING`, low otherwise). |

Notes:

* `INPUT_WIDTH` == `OUTPUT_WIDTH`; the block is bit-transparent on the payload path.  
* The output `TUSER` field is exactly 7 bits wide and carries the current symbol index (`symbol_index[6:0]`).  
* `sto_correction` is consumed atomically; multiple requests in one tracking interval accumulate (saturating) before the next frame launch.

---

## 3) Programmable parameters

| Parameter | Purpose | Default | Constraints |
|-----------|---------|---------|-------------|
| `INPUT_WIDTH` | Bits per I or Q sample on each antenna. | 12 | ≥4 |
| `AXIS_TUSER_WIDTH_IN` | Width of incoming `s_axis_tuser` bus. | 1 | ≥1, LSB = Minn flag. |
| `NFFT` | FFT length / payload samples per symbol. | 2048 | >0, divisible by 4 (preamble) but no longer required here. |
| `CP_LEN` | Cyclic prefix length in samples. | 512 | ≥0 |
| `N_FRAME_SYMBOLS` | Number of OFDM payload symbols per frame. | 28 | ≥1, ≤128 (fits in output TUSER). |
| `FRAME_GAP_SAMPLES` | Samples between end of one frame and start of the next. | `N_FRAME_SYMBOLS*(NFFT+CP_LEN)` | ≥0 |
| `SYMBOL_COUNTER_WIDTH` | Output symbol counter width. | 7 | Must cover `N_FRAME_SYMBOLS-1`. |
| `STO_ACC_WIDTH` | Internal accumulator width for STO scheduling. | 12 | ≥ (log2(`FRAME_GAP_SAMPLES`) + 8). |

All parameters are constant at synthesis. Runtime configurability (e.g., via CSRs) can be added later with small wrappers that drive the parameter signals.

---

## 4) Derived timing constants

* `SYMBOL_LEN = NFFT + CP_LEN`  
* `FRAME_PAYLOAD_LEN = NFFT * N_FRAME_SYMBOLS`  
* `FRAME_PERIOD = SYMBOL_LEN * N_FRAME_SYMBOLS + FRAME_GAP_SAMPLES`  
* `STO_MIN/MAX = ±(2^(STO_ACC_WIDTH-1)-1)`  
* `SYMBOL_COUNTER_MAX = N_FRAME_SYMBOLS - 1`

These constants drive the counters used in `COLLECTING` (per-symbol) and in `TRACKING` (inter-frame).

---

## 5) State machine

```
            ┌────────────┐  sof_detect
    reset → │ SEARCHING  │ ─────────────→ │
            └────────────┘                 │
                                            ▼
                                  ┌────────────────┐ payload symbols complete
                                  │  COLLECTING    │ ───────────────┐
                                  └────────────────┘                │
                                          ▲                         │
                                          │                         │
                                      start_next_frame  sto_load    │
                                          │                         │
                                  ┌────────────────┐ gap counter → 0│
                                  │   TRACKING     │ ───────────────┘
                                  └────────────────┘ resets sampling cadence
```

* **SEARCHING**  
  * `s_axis_tready = 1`, `m_axis_tvalid = 0`.  
  * Watch for `s_axis_tvalid && sof_pulse`; when seen, seed counters (`symbol_index = 0`, `sample_in_symbol = 0`) and transition to `COLLECTING`.  
  * Discard any other metadata (TLAST, STO).

* **COLLECTING**  
  * Actively forward payload samples:  
    * For `0 ≤ sample_in_symbol < NFFT`, handshake like a normal AXI register slice (stall if downstream back-pressures).  
    * For `NFFT ≤ sample_in_symbol < SYMBOL_LEN`, consume input samples but do **not** raise `m_axis_tvalid` (CP discard).  
  * Increment `sample_in_symbol` modulo `SYMBOL_LEN`.  
  * When `sample_in_symbol` rolls from `SYMBOL_LEN-1` back to 0, increment `symbol_index` (mod `N_FRAME_SYMBOLS`).  
  * When `symbol_index` wraps, assert `m_axis_tlast` on the final payload sample, clear internal symbol counter, preload `gap_count = FRAME_GAP_SAMPLES`, and move to `TRACKING`.  
  * Downstream `sof` pulses are ignored (masked).

* **TRACKING**  
  * No payload forwarding; `m_axis_tvalid = 0`.  
  * `gap_count` decrements every accepted input sample.  
  * While in this state, accept optional `sto_correction`. Each valid command updates a signed accumulator: `gap_count += sto_value`. Clamp to `[0, FRAME_PERIOD_MAX]` to avoid underrun/overflow.  
  * When `gap_count` reaches zero, immediately seed `symbol_index = 0`, `sample_in_symbol = 0`, and re-enter `COLLECTING` (next input sample treated as symbol 0 sample 0).  
  * If reset occurs, drop back to `SEARCHING`.

---

## 6) Sample and counter tracking

| Counter | Width | Function | Update rules |
|---------|-------|----------|--------------|
| `sample_in_symbol` | `ceil_log2(SYMBOL_LEN)` | Position inside current OFDM symbol. | Increments with every accepted sample; wraps at `SYMBOL_LEN`. |
| `symbol_index` | `SYMBOL_COUNTER_WIDTH` | Which symbol within the frame (0-start). | Increments when `sample_in_symbol` wraps to 0; wraps at `N_FRAME_SYMBOLS`. |
| `gap_count` | `ceil_log2(FRAME_GAP_SAMPLES + 1) + 8` | Remaining samples before next frame launch. | Loaded on transition to `TRACKING`, decremented per sample, plus STO adjustments. |
| `sto_accum` | `STO_ACC_WIDTH` | Running sum of STO corrections applied to the pending launch. | Updated by `sto_correction`; folded into `gap_count`. |

The design keeps two one-hot enables: `payload_phase = (state==COLLECTING) && (sample_in_symbol < NFFT)` and `cp_phase = (state==COLLECTING) && !payload_phase`. These gates drive the handshake mux that either forwards or drops samples.

---

## 7) Data-path overview

```
                               ┌──────────────────────────────────┐
                               │  AXIS register slice / skid buf  │
                               └──────────────┬───────────────────┘
                                              │
                                    sample_in_symbol / symbol_index
                                              │
                                    ┌─────────▼─────────┐
                                    │ Payload gate mux  │
                                    │  (pass or drop)   │
                                    └─────────┬─────────┘
                                              │
                                      ┌───────▼────────┐
                                      │ TUSER packer   │
                                      │ (symbol index) │
                                      └───────┬────────┘
                                              │
                                     AXI4-Stream out (payload only)
```

* The skid buffer ensures payload samples are not lost if `m_axis_tready` de-asserts during `COLLECTING`.  
* CP samples bypass the skid buffer’s valid path; only their `tready` path participates, keeping the stream aligned without storing the discarded data.  
* `s_axis_tlast` is sampled alongside payload data; it does **not** drive state. The block re-asserts `m_axis_tlast` deterministically on the frame boundary, regardless of the incoming TLAST pulse.

---

## 8) `TUSER` encoding (output)

| Bit(s) | Name | Description |
|--------|------|-------------|
| `[6:0]` | `symbol_index` | Current OFDM symbol number (0-based) when this payload sample was emitted. |

During CP discard or gaps, `m_axis_tvalid = 0`, so `TUSER` is undefined externally (internally held at zero).

---

## 9) Start-of-frame handling

* A single detector pulse (any cycle where `s_axis_tvalid && sof_pulse`) arms the machine.  
* Additional pulses (spurious or due to overlapping detection) are ignored while in `COLLECTING` or `TRACKING`.  
* When the machine returns to `SEARCHING` (only via reset), the next detector pulse re-locks timing.  
* Optionally, the Minn pulse can be recorded into a sticky status bit for diagnostics; doing so does not affect the datapath.

---

## 10) STO correction protocol

* `sto_valid` may assert only in `TRACKING`. The block returns `sto_ready = 1` in that state, `0` otherwise.  
* Each accepted command updates `sto_accum += sto_correction`. Clamp results to `[-255, +255]` (8-bit range) to match the interface.  
* Right before launching the next frame (`gap_count` underflow), apply `sto_accum` to adjust the launch timer: `gap_count = max(0, gap_count + sto_accum)`; then clear `sto_accum`.  
* Corrections received too late (after `gap_count` hits zero) are deferred to the *following* frame by leaving `sto_accum` uncleared until the next `TRACKING` entry.

This protocol allows software to nudge the launch point based on fine timing estimates computed downstream.

---

## 11) Reset and error behavior

* `rst = 1` synchronously forces `state = SEARCHING`, clears all counters, empties the skid buffer, and deasserts `m_axis_tvalid`.  
* No attempt is made to drain partially forwarded frames; downstream modules see a discontinuity.  
* The block can optionally expose status counters (lock active, frames seen, overflow of STO) for software, but these are outside the core scope.

---

## 12) Resource considerations

* **Counters:** three medium-width adders (symbol, sample, gap).  
* **Storage:** single-entry skid buffer (two registers) for payload flow control; no large FIFOs.  
* **Comparators:** `sample_in_symbol == NFFT`, `sample_in_symbol == SYMBOL_LEN-1`, `symbol_index == N_FRAME_SYMBOLS-1`, `gap_count == 0`.  
* **STO arithmetic:** one signed adder sized to `STO_ACC_WIDTH`.  
* All logic is synchronous; no inferred latches, no gated clocks, reset synchronous and active-high.  
* `s_axis_tready` tied high post-reset ensures the Minn detector never stalls, which simplifies upstream buffering.

---

## 13) Verification strategy

1. **Unit-level cocotb**  
   * Drive synthetic frames with known geometry; assert that only payload samples emerge and symbol indices increment correctly.  
   * Randomize `m_axis_tready` to stress the skid buffer and AXI compliance.  
   * Insert CP-only intervals to confirm they are suppressed.  
   * Apply STO commands and verify launch timing adjusts by the requested sample count.
2. **Integration with Minn detector**  
   * Replay captured RF traces through Minn + synchronizer; confirm first Minn pulse is honored, subsequent pulses ignored.  
   * Assert that payload timing matches software reference (e.g., numpy pipeline) within ±1 sample after STO adjustments.  
   * Confirm `m_axis_tlast` pulses once per frame at the expected spot.
3. **Corner cases**  
   * `FRAME_GAP_SAMPLES = 0` ensures back-to-back frames work.  
   * `CP_LEN = 0` degenerates to pure payload streaming.  
   * Reset during `COLLECTING` and `TRACKING` returns to `SEARCHING` cleanly.

---

## 14) Behavioral pseudocode

```text
state      = SEARCHING
symbol_idx = 0
sample_cnt = 0
gap_cnt    = 0
sto_accum  = 0

while rising_edge(clk):
    if rst:
        state      = SEARCHING
        symbol_idx = 0
        sample_cnt = 0
        gap_cnt    = 0
        sto_accum  = 0
        drop_output()
        continue

    accept = s_axis_tvalid  // tready is hard-tied high except reset

    case state of
      SEARCHING:
         if accept && sof_pulse:
             state      = COLLECTING
             symbol_idx = 0
             sample_cnt = 0
         drop_output()

      COLLECTING:
         if accept:
             if sample_cnt < NFFT:
                 emit_payload(sample = s_axis_tdata,
                              user   = symbol_idx,
                              last   = (symbol_idx == N_FRAME_SYMBOLS-1) &&
                                       (sample_cnt == NFFT-1))
             else:
                 drop_output()

             if sample_cnt == SYMBOL_LEN-1:
                 sample_cnt = 0
                 if symbol_idx == N_FRAME_SYMBOLS-1:
                     symbol_idx = 0
                     gap_cnt    = FRAME_GAP_SAMPLES
                     state      = TRACKING
                 else:
                     symbol_idx += 1
             else:
                 sample_cnt += 1

      TRACKING:
         drop_output()
         if sto_valid && sto_ready:
             sto_accum = clamp(sto_accum + sto_correction)
         if accept:
             if gap_cnt > 0:
                 gap_cnt -= 1
             if gap_cnt == 0:
                 adjust = clamp(sto_accum)
                 if adjust >= 0:
                     gap_cnt = adjust
                 else:
                     // Negative adjust: consume extra samples immediately
                     consume_extra(abs(adjust))
                 if gap_cnt == 0:
                     state      = COLLECTING
                     symbol_idx = 0
                     sample_cnt = 0
                     sto_accum  = 0
```

`consume_extra()` is shorthand for subtracting additional samples if a negative STO requests an earlier frame start; implementation performs the subtraction by saturating `gap_cnt` at zero after applying the signed adjustment.

---

## 15) Integration notes

* Instantiate this block right after `minn_preamble_detector`, reusing its AXIS channel.  
* Configure downstream FFT (or mapper) to expect `TUSER[6:0]` as the symbol index.  
* The Minn detector’s `OUTPUT_MARGIN` must be large enough to cover at least one symbol so that the synchronizer has time to drop CP without underflowing the Minn output FIFO.  
* For software control, expose `FRAME_GAP_SAMPLES`, `N_FRAME_SYMBOLS`, and STO controls through existing CSR infrastructure (if available) or tie them to synthesis-time parameters for the first revision.

This document should contain everything required to implement the synchronizer in synthesizable Verilog/SystemVerilog that complies with `docs/verilog_guidelines.md`.
