Below is the standalone hardware specification for the **dual-antenna streaming FFT wrapper** that converts the shared OFDM AXI4-Stream into two independent SPIRAL iterative FFT instances and returns a matched AXI4-Stream of frequency-domain bins. The wrapper preserves per-symbol metadata, hides the SPIRAL load/compute gaps behind ping-pong buffering, and delivers continuous complex spectra at the sample clock.

> TL;DR for the RTL engineer:
> - AXI4-Stream in/out; input packs `{ant1_q, ant1_i, ant0_q, ant0_i}` (12 bits each) plus `TUSER[6:0] = symbol_index`.
> - Two identical SPIRAL 2048-point iterative FFT cores run in parallel, one per antenna, clocked from `clk_fft >= 6 * clk_axis`.
> - Per-antenna ping-pong BRAMs absorb the 11,287-cycle core gap; burst feeders pack pairs of time samples into the core `{X0..X3}` ports.
> - Output capture FIFOs re-serialize FFT bins back to one sample per clock at `clk_axis`, mirror the symbol index, and gate `TLAST` on the final bin.
> - All buffering is back-pressure aware; `s_axis_tready` drops only if both ping banks are busy (should not happen under the clocking budget).
> - All operating parameters are compile-time constants; the wrapper only exposes sticky status outputs for observability.

---

## 1) Placement and context

```
         +--------------------+   dual AXIS (time samples)
RF -> ...| frame_sync         | ---------------------------+
         +--------------------+                           |
                                                         v
                                            +----------------------------------+
                                            | fft_stream_wrapper               |
                                            |  - ingress & ping-pong buffers   |
                                            |  - FFT schedulers (2x SPIRAL)    |
                                            |  - egress formatter              |
                                            +----------------------------------+
                                                         |
                                                         v
         +--------------------+   dual AXIS (FFT bins, TUSER)
         | equalizer / MIMO   | <---------------------------+
         +--------------------+
```

* Upstream guarantees that cyclic prefixes are already stripped and that samples are symbol-aligned; each `clk_axis` cycle contains both antennas' complex samples.
* Downstream expects time-aligned spectra where `TUSER[6:0]` carries the symbol number (wraps modulo 128). `TLAST` is asserted on each symbol's final FFT bin.
* The wrapper is clock-domain bridged: `clk_axis` about 30.72 MHz (sample domain), `clk_fft` at or above 200 MHz (FFT domain, derived in Section 4).

---

## 2) Functional scope

* Consume a continuous dual-antenna complex stream and perform two 2048-point FFTs per OFDM symbol, one per antenna.
* Preserve the per-sample ordering: FFT bin `k` of antenna 0 and antenna 1 share a cycle at the output.
* Provide full AXI4-Stream backpressure handling on ingress and egress; never drop samples silently.
* Retain and propagate the 7-bit symbol index for every bin; assert `TLAST` for the last bin of the symbol.
* Document compile-time parameters for FFT length, optional scaling shifts, and future bin decimation settings selected at synthesis.
* Supply visibility hooks: buffer occupancy, overrun detection, FFT busy, symbol latency counters.

---

## 3) External interfaces

| Port | Dir | Clock | Width | Notes |
|------|-----|-------|-------|-------|
| `clk_axis` | in | - | 1 | Sample domain clock (about 30.72 MHz). |
| `rst_axis_n` | in | `clk_axis` | 1 | Active-low synchronous reset for ingress/egress. |
| `clk_fft` | in | - | 1 | High-speed FFT domain clock (target 200-245.76 MHz). |
| `rst_fft_n` | in | `clk_fft` | 1 | Active-low synchronous reset for FFT domain. |
| `s_axis_tdata` | in | `clk_axis` | 48 | `{ant1_q[11:0], ant1_i[11:0], ant0_q[11:0], ant0_i[11:0]}`. |
| `s_axis_tuser` | in | `clk_axis` | 7 | Symbol index holding for all samples of a symbol. |
| `s_axis_tvalid` | in | `clk_axis` | 1 | AXI4-Stream valid. |
| `s_axis_tready` | out | `clk_axis` | 1 | Deasserted only when both ingress buffers are full. |
| `s_axis_tlast` | in | `clk_axis` | 1 | Optional; ignored (metadata only). |
| `m_axis_tdata` | out | `clk_axis` | 48 | FFT output packed `{ant1_q, ant1_i, ant0_q, ant0_i}` (post scaling). |
| `m_axis_tuser` | out | `clk_axis` | 7 | Symbol index aligned to FFT bins. |
| `m_axis_tvalid` | out | `clk_axis` | 1 | Valid when FFT bins available. |
| `m_axis_tready` | in | `clk_axis` | 1 | Backpressure from downstream. |
| `m_axis_tlast` | out | `clk_axis` | 1 | High on last FFT bin of the symbol. |
| `status_flags` | out | `clk_axis` | 6 | Sticky status bits: `{busy_any, axis_backpressure, ingress_overrun, fft_gap_violation, egress_underflow, token_fifo_overflow}`. |
| `latency_debug` | out | `clk_axis` | 16 | Optional sampled latency metric (cycles from ingest to first bin), updates once per symbol. |

Notes:

* All data fields are signed two's complement; align with project RTL guidelines.
* Status outputs are held until reset; system software can sample them through fabric GPIO if desired.

---

## 4) Throughput and clocking requirements

The SPIRAL core (`dft_top`) expects two complex words per `clk_fft` cycle and mandates a gap of 11,287 cycles between `next` pulses. With an FFT size of 2048 samples per antenna:

* Load time: 2048 samples divided by 2 samples per cycle equals 1024 `clk_fft` cycles.
* Total cycle budget: 11,287 cycles per transform, or 11,287 / `clk_fft` seconds.
* To sustain continuous streaming at 30.72 Msps per antenna, the FFT must finish within one symbol period:  
  `T_symbol = 2048 / 30.72e6` seconds, roughly 66.666 microseconds.  
  Requirement: `11,287 / clk_fft <= T_symbol`, which implies `clk_fft >= 168.9 MHz`.

Design margin:

* Target `clk_fft = 245.76 MHz` (8x 30.72 MHz); this yields 45.9 microseconds per transform, about 30 percent headroom.
* If `clk_fft` drops below 180 MHz, ingress buffers will eventually overflow; this asserts `status_flags[2]`.
* The ingress and egress logic stay on `clk_axis` to keep AXI deterministic and avoid retiming upstream blocks.

---

## 5) Top-level data flow

```
clk_axis domain:                               clk_fft domain:

        +-----------+   tokens +----------+   +------------+   +------------+
samples | ingress & |---------->| async    |-->| per-ant FFT|-->| output cap |--> AXIS out
  AXIS  | ping-pong |          | scheduler|   | engines    |   | ping FIFOs |
        +-----------+          +----------+   +------------+   +------------+
```

1. **Ingress/ping-pong (clk_axis)**  
   - Demultiplex the 48-bit word into two complex samples, sign-extend to 14 bits (prevents overflow on optional scaling).  
   - Write each antenna into a dual-bank BRAM (`depth = NFFT`, `width = 2 x 14 bits`).  
   - Track `symbol_index`, `write_count`, and latch metadata when a bank becomes full.
2. **Async scheduler (two-clock FIFO)**  
   - Push a token `{bank_id, symbol_index}` into a CDC FIFO once both antennas finish the same symbol.  
   - FFT domain pops tokens and asserts `next` into both cores when idle, referencing the same `symbol_index`.
3. **FFT burst feeder (clk_fft)**  
   - Interleave addresses to read two consecutive time samples per cycle and drive the SPIRAL core ports `{X0, X1, X2, X3}`.  
   - Fixed pipeline ensures `next` high one cycle before first data (per SPIRAL spec).  
   - Apply any compile-time gain adjustment (`INPUT_SHIFT`, default zero) before driving the core; normally this stage is transparent because the SPIRAL generator embeds scaling.
4. **Output capture (clk_fft)**  
   - Collect `{Y0..Y3}` for 1024 cycles after `next_out` is seen; store per antenna into output FIFOs (depth at least 2048).  
   - Track bin index to generate `last_bin` flag and to align with symbol metadata.
5. **AXIS egress (clk_axis)**  
   - Drain FIFOs at `clk_axis`, packaging the two complex outputs back into `{ant1, ant0}` order.  
   - Reissue `symbol_index` stored in the token; manage `tvalid/tready`.  
   - Assert `tlast` on bin `NFFT-1`.

---

## 6) Ingress buffering and symbol handling

* Per-antenna ping-pong: two physical banks per antenna (`bank0`, `bank1`). Each bank is a simple dual-port RAM (write in `clk_axis`, read in `clk_fft`).  
  - `addr` increments every `clk_axis` when `s_axis_tvalid && s_axis_tready`.  
  - When `addr == NFFT-1`, raise `bank_full`.  
  - Store `symbol_index` alongside `bank_full` flag (one register per bank).  
  - Switch banks immediately after full; if the next bank is still busy (not released by FFT domain), deassert `s_axis_tready`.
* Symbol alignment: assume both antennas share identical symbol cadence. Ingress asserts a combined `symbol_ready` only when both antennas fill the same bank index.  
* Metadata queue: push `{symbol_index, ping_idx}` into a small FIFO (depth 4) feeding the CDC stage. This ensures order even under temporary FFT stalls.  
* Data width: extend 12-bit inputs to 14 bits before storage to absorb one guard bit for optional pre-FFT scaling (configured shifts up to 2 bits). All subsequent math retains sign extension.

---

## 7) FFT domain control and feeding

Each antenna owns an identical controller; both launch together with the same symbol token.

* Token FIFO: dual-clock FIFO (`clk_axis` write, `clk_fft` read) with payload `{symbol_index[6:0], ping_idx}`. Empty stalls the FFT domain; full asserts backpressure upstream through `bank_busy`.
* Start sequence: when a token pops and both FFT cores are idle:
  1. Load `read_base = ping_idx ? bank1_addr : bank0_addr`.
  2. Assert `core_next` for one `clk_fft` cycle.
  3. On the following cycle, begin streaming reads. Controller issues `read_addr` from 0 to `NFFT-1` and packs two successive samples per cycle:  
     - Cycle `n`: send sample `2n` to `{X0, X1}` and sample `2n+1` to `{X2, X3}`.
* Gap enforcement: counters track `load_cycles` and `cooldown_cycles`; controller refuses new token until 11,287 cycles have elapsed since the previous `next`. This matches SPIRAL's gap rule even if `clk_fft` is faster than minimum.
* Scaling strategy:  
  - Dynamic scaling is handled inside the SPIRAL core.  
  - Wrapper-level arithmetic shifts are fixed at synthesis through parameters `INPUT_SHIFT` and `OUTPUT_SHIFT` (both default zero) in case additional headroom is needed. Adjusting them requires re-synthesis.
* Output capture:  
  - Wait for `core_next_out`.  
  - Start capturing 1024 cycles of `{Y0..Y3}`.  
  - For antenna 0: use `{Y0, Y1}`; for antenna 1: use `{Y2, Y3}`.  
  - Write into per-antenna asynchronous FIFOs that bridge back to `clk_axis`.

---

## 8) AXIS egress formatting

* Two asynchronous FIFOs (`clk_fft` write, `clk_axis` read) hold antenna spectra. Depth recommendation: 4096 words (safety margin for downstream stalls). Word format: `{imag[17:0], real[17:0]}` after scaling (allow up to 18 bits with guard).  
* `egress` state machine in the `clk_axis` domain pops both FIFOs simultaneously when `m_axis_tready` is high.  
* `tvalid` is asserted as long as both FIFOs report non-empty; if downstream backpressures, FIFOs naturally absorb up to their depth.  
* `tuser` is fetched from a metadata FIFO (mirrors token FIFO) and held constant for the entire symbol.  
* `tlast` pulses when `bin_idx == NFFT-1`; `bin_idx` increments with every cycle where data is emitted.  
* If either FIFO underflows (should not happen provided `clk_fft` meets the requirement), assert `status_flags[4]` and insert zeros until the symbol completes to maintain framing.

---

## 9) Static parameters and status signals

All operating parameters are fixed at synthesis time. Suggested module generics:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `NFFT` | 2048 | FFT length (must match SPIRAL core). |
| `SAMPLES_PER_CYCLE` | 2 | Must equal the SPIRAL core throughput. |
| `GAP_CYCLES` | 11287 | Cooldown between `next` pulses (from core metadata). |
| `INPUT_SHIFT` | 0 | Optional arithmetic right shift applied before feeding the core. |
| `OUTPUT_SHIFT` | 0 | Optional right shift applied after capture. |
| `FIFO_DEPTH` | 4096 | Output FIFO depth per antenna (power of two). |

The wrapper exposes simple observability ports:

* `status_flags[5:0]`: sticky bits `{busy_any, axis_backpressure, ingress_overrun, fft_gap_violation, egress_underflow, token_fifo_overflow}` cleared only by reset.
* `latency_debug[15:0]`: sampled latency in `clk_axis` cycles from final ingress write to first egress bin of the same symbol; for lab visibility only.

---

## 10) Error handling and observability

* Ingress overrun: occurs if both ping banks are still owned by the FFT domain when new samples arrive; asserts `status_flags[2]` and forces `s_axis_tready = 0`. Recovery: wait for the FFT domain to release a bank.
* FFT gap violation: protective assertion that ensures the controller never issues `next` before the 11,287-cycle cooldown; flagged on `status_flags[3]`.
* Egress underflow: if downstream throttles longer than FIFO depth, zeros are inserted and `status_flags[4]` is set.
* Optional sample capture: debug mux can loop one antenna path back to output without FFT; enablement is a compile-time switch (`ENABLE_LOOPBACK_TEST` parameter).
* Provide per-domain debug buses exposing state machines, bank indices, and FFT cycle counters for on-chip logic analyzers.

---

## 11) Reset sequence

1. Drive both resets low (`rst_axis_n = 0`, `rst_fft_n = 0`) for at least four cycles of their respective clocks.
2. Release `rst_fft_n`; wait at least 16 cycles; internal controllers clear to idle, `ping_busy = 0`, output FIFOs flushed.
3. Release `rst_axis_n`; the block asserts `s_axis_tready = 1` once both banks idle.
4. Release the upstream source; ingest begins with the first valid AXI sample (no run-time programming required).

Resets do not cross domains; rely on handshake logic to flush residual tokens.

---

## 12) Verification plan

1. Unit lint and CDC:  
   - Leverage the project's CDC lint suite to verify dual-clock FIFOs and synchronizers.  
   - Confirm no combinational paths between `clk_axis` and `clk_fft`.
2. Cocotb system tests (build on `tests/test_spiral_dft.py`):  
   - Single-symbol sanity: drive one symbol per antenna with known sine waves; verify FFT magnitude versus NumPy.  
   - Continuous streaming: feed 16 symbols back-to-back at 30.72 Msps, ensure no `tready` stalls and outputs match the software model.  
   - Backpressure: hold `m_axis_tready = 0` for random bursts; ensure FIFOs absorb data and flags remain clear.  
   - Clock ratio sweep: run with `clk_fft` barely over requirement and observe warnings; with forced slow clock expect ingress backpressure and flags.  
   - Error injection: build a variant with `ENABLE_LOOPBACK_TEST = 1` to bypass the FFT; confirm output mirrors input (helps isolate core issues).
3. Hardware bring-up:  
   - Use ILAs on both clocks to capture bank indices, `next`, `next_out`, token flow.  
   - Record end-to-end latency and compare to the `latency_debug` output.

---

## 13) Implementation notes

* Reuse the bundled SPIRAL core (`rtl/spiral_dft_iterative_2048pt.v`) untouched; wrap it in a thin module exposing clean ports. Instantiate two copies named `fft_ant0` and `fft_ant1`.  
* Place ingress ping-pong memories in BRAM (true dual-port) to ease timing; depth parameterizes with `NFFT`.  
* Prefer gray-coded write/read pointers for async FIFOs; ensure depth is a power of two.  
* Keep per-clock state machines simple:  
  - `clk_axis`: three states (`IDLE`, `WRITE`, `STALL`).  
  - `clk_fft`: four states (`INIT_GAP`, `LOAD`, `COMPUTE_WAIT`, `DRAIN`).  
* Pipeline the `{Y*}` capture path: register outputs once before FIFO write to meet 245 MHz timing.  
* Follow project RTL guidelines (`docs/verilog_guidelines.md`) for naming, reset polarity, and AXIS semantics.

---

This specification defines the complete behavior, timing, and integration contract of the streaming FFT wrapper so an RTL engineer can implement it with predictable resource usage and without revisiting the SPIRAL core internals. Consult the verification plan prior to merge to ensure cocotb coverage matches the scenarios outlined above.
