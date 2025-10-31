Below is a **standalone design document** for the requested wrapper. It assumes the attached SPIRAL‑generated iterative FFT core is the one at `/spiral_dft_iterative_2048pt.v` with top module `dft_top`. The document is structured so an RTL engineer can implement directly without guesswork.

---

# 2x2 OFDM FFT Wrapper (AXI‑Stream ⇄ SPIRAL Iterative FFT)

**Version:** 1.0
**Date:** 2025‑10‑31
**Owner:** Baseband / PHY team

---

## 1. Purpose & Scope

Design an efficient, streaming‑style FFT wrapper for a **2‑antenna (2×)** OFDM RX (or TX) datapath.

* **Input:** single AXI‑Stream, **48‑bit** `TDATA` = `{ant1.Q[11:0], ant1.I[11:0], ant0.Q[11:0], ant0.I[11:0]}`; `TUSER[6:0] = symbol_number`; **one complex sample per antenna per beat**; `TLAST` marks end of symbol.
* **Output:** same AXI‑Stream format and timing semantics as input, after FFT.
* **Core:** SPIRAL iterative **2048‑pt FFT**, 12‑bit fixed‑point I/Q, interface `dft_top`.
* **Channels:** 2 independent FFT instances (one per antenna).
* **Throughput target:** sustain **30.72 Msps per antenna** (LTE‑class), i.e., **15 k FFT/s per antenna** (2048 samples / symbol).
* **Clocking:** decouple sample clock and FFT core clock; meet core throughput using dual‑clock ping‑pong buffers.

The wrapper guarantees continuous symbol‑rate operation without stalling the upstream AXI source, provided the FFT core clock meets the requirement in §4.2.

**Non‑goals:** CP removal/insertion, windowing, DC offset, and channel re‑mapping. Do those upstream/downstream. If you try to smuggle CP handling into this wrapper, you’ll just create timing grief.

---

## 2. Referenced FFT Core (`dft_top`)

Extracted facts from the SPIRAL file:

* **Top:** `module dft_top (clk, reset, next, next_out, X0..X3, Y0..Y3);`
* **Word widths:** all data ports are **12 bits**.
* **Data packing:** **2 complex words per core cycle**:

  * Inputs: `(X0 = Re0, X1 = Im0)` and `(X2 = Re1, X3 = Im1)`
  * Outputs likewise: `(Y0,Y1)` and `(Y2,Y3)`.
* **Handshake:**

  * Pulse `next=1` for 1 cycle → *next* cycle is the first input cycle.
  * After internal computation, `next_out=1` pulses → *next* cycle is the first output cycle.
* **Frame lengths:** **2048 complex** samples in; **1024 core cycles** of input (because 2 complex/cycle). Same on output.
* **Timing (from the file):**

  * **Gap:** **11 287** core cycles between *beginnings* of consecutive transforms.
  * **Latency:** **12 267** core cycles from `next` to `next_out`.

These are contract values; design everything around them.

---

## 3. External Interfaces

### 3.1 Clocks & Reset

* `s_axis_aclk` : sample/stream clock (30.72 MHz nominal).
* `core_aclk`   : FFT core clock (≥ §4.2 minimum; recommend 184.32 MHz or 200 MHz).
* `aresetn`     : active‑low async reset, internally synchronized to both domains.

### 3.2 AXI‑Stream Input (slave)

* `s_axis_tdata[47:0]` = `{ant1_q, ant1_i, ant0_q, ant0_i}`, little endian within each 12‑bit lane if platform prefers; pick a mapping and **document it** in the top file.
* `s_axis_tvalid`, `s_axis_tready`
* `s_axis_tlast` : asserted on **sample 2047** of a symbol (per antenna).
* `s_axis_tuser[6:0]` : **symbol_number** (mod 128). Latched on the **first** beat of each symbol.

**Contract:** input provides exactly **2048 beats per symbol**, CP already removed. If you feed 2048+CP here, you’re wrong—strip CP upstream.

### 3.3 AXI‑Stream Output (master)

* `m_axis_tdata[47:0]` = `{ant1_q, ant1_i, ant0_q, ant0_i}` after FFT.
* `m_axis_tvalid`, `m_axis_tready`
* `m_axis_tlast` : asserted on **bin 2047**.
* `m_axis_tuser[6:0]` : propagated symbol_number of the corresponding input symbol.

**Default behavior:** output runs continuously at one beat per bin (nominally 30.72 MHz). If downstream de‑asserts `m_axis_tready` mid‑frame and we’re in “thin mode” (§6.3), backpressure is **not** propagated to the FFT core; an overflow flag will latch. Use “safe mode” for bulletproof operation.

### 3.4 Optional AXI‑Lite (recommended)

Minimal status/config:

* `STATUS`: sticky flags (underflow/overflow/mismatch), frame counters.
* `CFG`: mode bits (output buffering mode), optional output right‑shift.
* `MIN_FCLK`: read‑only value for computed minimum `core_aclk` (for sanity checking).

---

## 4. Performance & Clocking

### 4.1 Symbol and transform rates

* Symbol rate per antenna: ( R_{sym} = \frac{30.72\text{ Msps}}{2048} = 15{,}000\ \text{symbols/s} ).
* Required transforms per second per core: **15 k**.

### 4.2 FFT core clock requirement

Transform throughput = ( \frac{F_{core}}{\text{gap}} ).

* Minimum ( F_{core} = 15{,}000 \times 11{,}287 = \mathbf{169.305\ MHz} ).

**Recommendations:**

* **Option A (ratio‑friendly):** **184.32 MHz** (= 30.72 MHz × 6). Throughput ≈ 16 325 t/s → **8.8%** headroom.

  * Latency = 12 267 / 184.32 MHz ≈ **66.55 µs**.
  * Gap time = 11 287 / 184.32 MHz ≈ **61.22 µs**.
* **Option B (extra margin):** **200 MHz**. Throughput ≈ 17 720 t/s → **18%** headroom.

  * Latency ≈ **61.34 µs**. Gap ≈ **56.44 µs**.

**Pick B** if your device/constraints are tight; **A** if you prefer clean integer clock ratios (simplifies CDC timing math).

---

## 5. Top‑Level Block Diagram (conceptual)

```
                 ┌─────────────────────────────────────────────────────────┐
s_axis_* ───────►│  Ingress: AXIS RX + Symbol Framer + Deinterleaver      │
                 │   - latch TUSER, count 2048 beats, check TLAST         │
                 └───────┬───────────────────────────┬────────────────────┘
                         │                           │
                         │                           │
            Ant0 Domain  ▼                           ▼  Ant1 Domain
                 ┌──────────────────────┐     ┌──────────────────────┐
                 │ Ping/Pong IN BRAM    │     │ Ping/Pong IN BRAM    │
                 │ 2048 x 24b x 2 banks │     │ 2048 x 24b x 2 banks │
                 └───────┬──────────────┘     └───────┬──────────────┘
                         │ pair 2 samples/cycle        │ pair 2 samples/cycle
 core_aclk domain        ▼                              ▼
                 ┌──────────────────────┐     ┌──────────────────────┐
                 │ 2‑lane Packer        │     │ 2‑lane Packer        │
                 ├──────────────────────┤     ├──────────────────────┤
                 │  dft_top  (FFT #0)   │     │  dft_top  (FFT #1)   │
                 └───────┬──────────────┘     └───────┬──────────────┘
                         │ 2 complex/cycle            │ 2 complex/cycle
                         │                              │
                 ┌───────▼──────────────┐     ┌───────▼──────────────┐
                 │ Output Buffer (opt)  │     │ Output Buffer (opt)  │
                 │ BRAM ping/pong or    │     │ BRAM ping/pong or    │
                 │ shallow async FIFO   │     │ shallow async FIFO   │
                 └─────────┬────────────┘     └─────────┬────────────┘
                           │          merge bins (same index from both cores)
                           ▼
                 ┌─────────────────────────────────────────────────────────┐
                 │  Egress: AXIS TX + Bin Merger + TLAST/TUSER management │
                 └─────────────────────────────────────────────────────────┘
```

---

## 6. Detailed Design

### 6.1 Data format & packing

**Input beat (48 bits):**

```
[47:36] ant1_Q, [35:24] ant1_I, [23:12] ant0_Q, [11:0] ant0_I
```

(You may swap I/Q ordering if your system standard prefers `{I,Q}`; keep it consistent across in/out and document.)

**Packer → FFT mapping (per antenna):**
For address `s = 0..2047` in the IN BRAM bank,

* For core cycle `k = s >> 1`:

  * `X0 = I[s & ~1]`, `X1 = Q[s & ~1]`
  * `X2 = I[s | 1 ]`, `X3 = Q[s | 1 ]`

Thus 1024 core cycles deliver 2048 complex samples.

**FFT output capture (per antenna):**

* On `next_out`, first output cycle arrives **next core cycle**.
* Each core cycle produces two complex outputs:

  * `(Y0,Y1)` → bin `2k`
  * `(Y2,Y3)` → bin `2k+1`

### 6.2 Ingress (AXIS RX & symbol framer)

* **Counters:** `sym_sample_cnt` (0..2047).

* **TUSER capture:** latch `sym_id` when `sym_sample_cnt==0` and `tvalid && tready`.

* **Write path:** deinterleave into **two dual‑port BRAM ping‑pong banks**, one per antenna:

  * Depth: **2048**
  * Width: **24 bits** (I[11:0] concatenated with Q[11:0])
  * Ports: write @ `s_axis_aclk`, read @ `core_aclk`.

* **Banking:** `in_bank_sel[ant]` toggles every symbol. Producer writes to the “fill” bank while consumer (FFT packer) reads from the other.

* **Backpressure:** assert `s_axis_tready=1` when **both antennas** have their current fill bank available; otherwise `0`. You can’t accept only half a word; the two antennas are delivered together. If you don’t like that, split the bus earlier in the chain.

* **Integrity checks (sticky flags):**

  * `TLAST` expected at count 2047; flag `E_TLAST_POS` otherwise.
  * If `tlast` arrives early/late: still close the frame, but flag error and discard extra beats until next `tlast`.

### 6.3 Core scheduler & packers (per antenna)

* **FSM (per antenna in `core_aclk` domain):**

  1. `IDLE`: wait for `in_bank_ready`.
  2. `ARM`: pulse `next=1` **for one core clock**; load `read_idx=0`.
  3. `FEED`: 1024 cycles; each cycle read two samples (`addr=2k` and `2k+1`) and drive `X0..X3`.
  4. `WAIT_OUT`: wait for `next_out=1`.
  5. `DRAIN`: 1024 cycles; capture outputs either:

     * **Thin mode:** directly into a **small async FIFO** (depth ≥ 32 beats) to bridge to egress (low BRAM cost). Requires downstream not to stall for long; overflow flag if it does.
     * **Safe mode (default):** write **Output Ping‑Pong BRAM** (2048×24b) which decouples completely from egress. Zero data loss regardless of downstream stalls.
  6. Mark `out_bank_ready`, toggle `in_bank_sel`, go to `IDLE`.

* **Start synchronization:** The two channel FSMs are **gated** by a small arbiter so both `next` pulses can be aligned when both `in_bank_ready` are true. This keeps both cores in lockstep, simplifying the merger. If one antenna is late (rare with a common input), start them independently and buffer at egress; but the default is **lockstep**.

### 6.4 Egress (bin merger & AXIS TX)

* **Bin merger:** for **bin index `b=0..2047`**, read ant0 bin `b` and ant1 bin `b` (either directly from the small FIFOs in thin mode, or from the out BRAMs in safe mode) and pack into a 48‑bit word per the format in §6.1.

* **Symbol tagging:** propagate `sym_id` captured at ingress to egress; in lockstep mode it’s identical for both antennas. In async recovery mode, pick `ant0`’s tag and flag `E_TUSER_MISMATCH` if `ant1` differs.

* **TLAST:** assert on `b==2047`.

* **Backpressure:** `m_axis_tvalid` can be held high across cycles; `tready` stalls the **merger**.

  * **Thin mode:** merger stalling does **not** stall cores; if FIFOs fill, latch `E_TX_OVERFLOW` and drop trailing bins of the current symbol (still drive TLAST on the 2047th bin you actually delivered). This is a trade‑off mode to save BRAM.
  * **Safe mode (recommended):** no drops; entire symbol is buffered.

### 6.5 Reset & CDC

* **Reset:** `aresetn` → two synchronizers (one per domain).

* **Bank/flag reset:** clear bank select, counters, sticky flags, and FIFOs/BRAM read/write pointers.

* **CDC primitives:**

  * Dual‑port BRAMs provide the primary clock crossing for payload (write in `s_axis_aclk`, read in `core_aclk`).
  * For control strobes (`*_bank_ready`), use **toggle synchronizers** with gray counters or handshakes (two‑flip synchronizers per direction + ack). Don’t wing this; metastability is not a vibe.

---

## 7. Parameterization

Top module generics/parameters:

| Name          | Default  | Description                                                                       |
| ------------- | -------- | --------------------------------------------------------------------------------- |
| `N_FFT`       | 2048     | FFT size. **Fixed** by the provided SPIRAL core. Keep as a constant for now.      |
| `DATA_W`      | 12       | I/Q width. Must be 12 to match core.                                              |
| `OUT_SHIFT`   | 0        | Optional arithmetic right shift on output bins (0..2) for EVM/overflow trade‑off. |
| `SAFE_MODE`   | 1        | 1 = output BRAM ping‑pong; 0 = thin mode with shallow FIFOs.                      |
| `AXIS_ENDIAN` | `"QIQI"` | Defines `[47:0]` packing order (`QIQI` as specified; alternative `IQQI`).         |
| `SYNC_START`  | 1        | Start both cores in lockstep per symbol (recommended).                            |

If you deviate from 12‑bit or 2048‑pt, regenerate the SPIRAL core and revisit memory sizing, gap/latency, and clock numbers.

---

## 8. Resource Estimate (ballpark, per 2‑antenna system)

* **Core internal (per the SPIRAL file, per core):**

  * 4 × RAM (2048 × 24) + 2 × ROM (2048 × 12) → ≈ **5.5 × 36 kb BRAM** per core → **~11 BRAMs** total for two cores (device dependent; ROMs may map to BRAM or LUTs).

* **Wrapper buffers:**

  * **Ingress ping‑pong:** 2 antennas × 2 banks × (2048 × 24) ≈ **192 kb** → **~6 × 36 kb BRAM**.
  * **Egress safe mode:** same again → **~6 × 36 kb BRAM**.
  * **Total BRAM (safe mode):** ~11 (cores) + 12 (wrapper) ≈ **~23 × 36 kb BRAM**.
  * **Thin mode:** save ~6 BRAM (egress) at the cost of overflow risk on `tready` stalls.

* **Logic/DSP:** negligible compared to cores; a few small FSMs, adders for counters, muxes; no extra multipliers.

---

## 9. Latency Budget

For a given symbol:

1. **Ingress buffering:** receive 2048 beats @ 30.72 MHz → **66.67 µs**.
2. **Core latency:** `12 267 / Fcore` → **61.34–66.55 µs** depending on clock.
3. **Egress streaming:** 2048 beats @ 30.72 MHz → **66.67 µs** (if you require fully serialized AXIS).

Pipelining overlaps these stages across different symbols; **throughput** is governed by the **gap**, not latency, and is satisfied if §4.2 is met.

---

## 10. Error Handling (sticky flags)

* `E_TLAST_POS` : TLAST did not occur at sample 2047.
* `E_TUSER_MISMATCH` : per‑antenna TUSER mismatch for a symbol (shouldn’t happen with shared input).
* `E_RX_OVERRUN` : ingress bank not available when a new symbol arrived (backpressure asserted, just record it).
* `E_TX_OVERFLOW` : egress thin FIFO overflowed due to `tready` stalls.
* `E_CORE_UNDERRUN` : packer failed to deliver 1024 cycles after `next` (should never happen if banks are ready).

All flags readable/clearable via AXI‑Lite.

---

## 11. Implementation Notes & State Machines

### 11.1 Ingress (s_axis_aclk)

```verilog
// Pseudocode
if (!aresetn) reset();
else if (s_axis_tvalid && s_axis_tready) begin
  if (sym_sample_cnt == 0) sym_id <= s_axis_tuser;
  write_bram(ant0.fill_bank, sym_sample_cnt, {Q0,I0});
  write_bram(ant1.fill_bank, sym_sample_cnt, {Q1,I1});
  if (sym_sample_cnt == 2047) begin
    assert(s_axis_tlast);
    mark_in_bank_ready(ant0);
    mark_in_bank_ready(ant1);
    toggle_fill_banks();
    sym_sample_cnt <= 0;
  end else sym_sample_cnt <= sym_sample_cnt + 1;
end

s_axis_tready <= banks_available_for_both_antennas;
```

### 11.2 Core scheduler & packer (per antenna, core_aclk)

```verilog
case (state)
  IDLE: if (in_bank_ready && (SYNC_START ? peer_ready : 1)) begin
          next <= 1; state <= ARM;
        end
  ARM:  begin next <= 0; rd <= 0; state <= FEED; end
  FEED: begin
          {X0,X1} <= bram[rd*2];
          {X2,X3} <= bram[rd*2+1];
          rd <= rd + 1;
          if (rd == 1023) state <= WAIT_OUT;
        end
  WAIT_OUT: if (next_out) begin wr <= 0; state <= DRAIN; end
  DRAIN: begin
           capture (Y0,Y1) as bin 2*wr;
           capture (Y2,Y3) as bin 2*wr+1;
           wr <= wr + 1;
           if (wr == 1023) begin
             set_out_bank_ready();
             toggle_consume_bank();
             state <= IDLE;
           end
         end
endcase
```

### 11.3 Egress merger (s_axis_aclk)

```verilog
if (out_both_ready) begin
  for (b = 0..2047) stream:
    data <= {ant1_Q[b], ant1_I[b], ant0_Q[b], ant0_I[b]};
    tuser <= sym_id;
    tlast <= (b==2047);
    wait m_axis_tready;
end
```

---

## 12. Ordering & Scaling

* **Bin order:** SPIRAL usually emits **natural order** for the transform this generator targets. Verify in simulation against a golden model (e.g., Python `numpy.fft.fft`) for a few impulses and sinusoids; if it’s bit‑reversed, insert a compile‑time **bit‑reversal read address** for the egress buffer (cheap).
* **Fixed‑point scaling:** the provided core keeps I/O at 12 bits; internal scaling/rounding is handled inside the core. If your constellation/EVM budget needs margin, enable `OUT_SHIFT` (1–2 LSB arshift) before egress to tame peaks; yes, it’s crude but often sufficient.

---

## 13. Verification Plan

1. **Unit sim (per core):**

   * Drive 2048‑pt complex impulse at index 0 and N/4; confirm output spectra positions and magnitudes.
   * Validate `gap` and `latency` counters versus the stated numbers.

2. **Wrapper sim (2× cores):**

   * Drive two independent complex sinusoids per antenna at known bins; verify both channels.
   * Verify lockstep start: `next` pulses aligned; first bins of both antennas coincide.
   * AXIS protocol checks: TLAST at 2047; TUSER propagation; no beats lost.

3. **Stress:**

   * De‑assert `m_axis_tready` in thin mode → expect `E_TX_OVERFLOW` and symbol truncation.
   * Early/late TLAST → `E_TLAST_POS` latch; next symbol still processed.

4. **Throughput:**

   * Run 1000 consecutive symbols at 30.72 Msps; ensure no `s_axis_tready` deassertions after initial pipeline fill (with `core_aclk` per §4.2).

5. **Corner:**

   * Symbol_number wrap at 127 → 0; propagate modulo 128.

**Golden model:** Python/Matlab with fixed‑point quantization to 12b; allow ±1 LSB on magnitude and ±1 bin on phase wrap for edge bins.

---

## 14. Integration Notes

* **Don’t hide CP here.** Upstream must deliver CP‑stripped symbols of exactly 2048 samples.
* **Clocking:** derive 184.32 MHz from the same PLL chain as 30.72 MHz for clean ratios, or go 200 MHz if place/route is easier. Validate the BRAM true dual‑port timing at chosen Fmax.
* **Reset sequencing:** release `core_aclk` domain first, then `s_axis_aclk`, or gate schedulers until both report “ready”.
* **Synthesis attributes:** keep BRAMs inferred; set `ram_style = "block"`. For small FIFOs in thin mode, use vendor async FIFO primitives to save time.

---

## 15. Deliverables

* **RTL:**

  * `fft2x_wrapper.sv` (top)
  * `axis_ingress.sv`, `packer_core_if.sv`, `core_scheduler.sv` (×2), `egress_merger.sv`
  * `dualclk_bram.v` (or vendor primitive wrapper), `async_fifo.v`
  * `regs_axil.sv` (optional but recommended)

* **Testbench:**

  * `tb_fft2x_wrapper.sv` with AXIS BFMs, clock gen (30.72 MHz/184.32 MHz), and Python/Matlab vectors.

* **Constraints:** SDC/XDC for both clocks, false‑path/async CDC constraints for toggle synchronizers.

---

## 16. Strong guidance (so we don’t debug avoidable pain)

* **Use input and output ping‑pong BRAMs unless you’re absolutely constrained.** Thin mode is a BRAM optimization with real risk during backpressure.
* **Start both cores in lockstep.** It makes the egress merger trivial and avoids symbol skew.
* **Meet the clock math.** Anything below **169.305 MHz** on `core_aclk` will eventually fall behind. If you think you can “almost” meet timing, you can’t; raise the clock or add another core.
* **Validate bit ordering and bin ordering once, early.** Document in the top comment and lock it.

---

## 17. Appendix — `dft_top` port summary (for stubbing)

```verilog
module dft_top (
  input              clk,
  input              reset,     // active high
  input              next,      // 1-cycle pulse: inputs start next cycle
  output             next_out,  // 1-cycle pulse: outputs start next cycle
  input       [11:0] X0, X1, X2, X3,   // inputs: 2 complex per cycle
  output      [11:0] Y0, Y1, Y2, Y3    // outputs: 2 complex per cycle
);
```

* One transform every **11 287** core cycles (inputs included).
* Latency **12 267** core cycles to first output.

---

### Final word

This design is lean, deterministic, and clock‑math honest. The only bad choices here are (a) trying to avoid ping‑pong on the input, or (b) under‑clocking the cores. Do either and you’ll be papering over stalls forever. The spec above gives you clean separation of concerns: AXIS framing, buffering, 2‑lane packing to the core, and merged egress—each with clear timing contracts. If you need this generalized to more antennas or different N, replicate the per‑antenna slice and regenerate the SPIRAL core; the wrapper architecture stays the same.
