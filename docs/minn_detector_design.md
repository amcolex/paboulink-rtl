Below is a complete, *hardware‑first* design document for your OFDM frame‑detector (Minn) block. I’m going to be blunt: the naïve “brute‑force sliding window with 4 complex multiplies + 4 power terms per antenna per sample” will blow your DSP budget for no real benefit. The right approach is a product‑stream architecture that collapses Minn’s math to **one complex multiply per antenna per input sample** plus three light sliding sums. This is exactly the kind of trick your 6× faster fclk was born to enable.

I tie every spec back to a compact C reference so your RTL and sim benches stay lock‑step. Where I deviate (removing CFO estimation, adaptive normalization, dynamic thresholds, and timing offsets), I say so explicitly and keep the math honest. The references below map directly to your code: detector state and parameters in `minn_detector.h` , the implementation in `minn_detector.c` (metric definition, sliding updates, gating logic, and the “pending” delay queue idea) , and LTE‑like defaults (30.72 Msps, FFT sizes, gate defaults) in `modem_config.c` .

---

## 0) Scope and assumptions

* **Function**: detect the OFDM preamble using the Minn metric on two complex input channels, then output delayed I/Q with a one‑cycle **detection flag asserted only on the metric peak** (no timing offset; the flag rides on the peak’s sample).
* **Inputs**:

  * Two complex channels (call them ch0, ch1), **12‑bit signed I/Q** each, 30.72 Msps.
* **Clocking**: one faster fabric clock **fclk = 184.32 MHz (6×)** or higher. New input samples arrive every 6 fclk cycles.
* **OFDM parameters**: use LTE‑like defaults unless overridden: sample‑rate 30.72 Msps at 20 MHz BW implies **FFT size N = 2048**, **Q = N/4 = 512** (these are derived in your config) .
* **Simplifications** (hard requirements you gave):

  1. **No CFO estimation** (ignore angle of the correlation; the C version computes it, we don’t) .
  2. **Fixed threshold** (no adaptive normalization, i.e., `enable_adaptive_norm = false`, and we’ll gate on a fixed constant—defaults show `gate_threshold ≈ 0.2`) .
  3. **No timing offset** (flag lands **at the metric peak index**; the C code supported a `timing_offset`, we set it to 0 and implement that behavior in hardware) .

---

## 1) Minn metric math (hardware‑friendly form)

Your C reference computes per sliding window (length N = 4Q) two quarter‑lag correlations and three quarter energies, sums across antennas, then the metric (clipped to non‑negative real part), optionally smoothed and optionally normalized. Core lines (with small re‑labeling) that we will implement in RTL:

* **Quarter correlations (per antenna)** over a window starting at index `s` (one new `s` per input sample):

  * ( C_1(s) = \sum_{n=0}^{Q-1} x[n+s] \cdot \overline{x[n+s+Q]} )
  * ( C_2(s) = \sum_{n=0}^{Q-1} x[n+s+2Q] \cdot \overline{x[n+s+3Q]} )
  * **Total correlation across both antennas**: ( C(s) = \sum_{\text{ant}\in{0,1}} (C_1^{(\text{ant})}(s) + C_2^{(\text{ant})}(s)) ).
    This matches the reference’s `corr_c1`/`corr_c2` rolling updates and inter‑antenna sum. 

* **Quarter energies** (per antenna):
  ( E_1 = \sum |x[s+Q]|^2,\quad E_2 = \sum |x[s+2Q]|^2,\quad E_3 = \sum |x[s+3Q]|^2 ).
  **Total energy**: ( E(s) = \sum_{\text{ant}}(E_1 + E_2 + E_3) ). That is exactly the reference’s `energy_q1/q2/q3` path. 

* **Minn metric (simplified)**:
  Let ( R(s) = \max(0, \Re{C(s)}) ).
  [
  \text{metric}(s) = \frac{R(s)^2}{E(s)^2}.
  ]
  (This is the reference behavior when adaptive normalization is off; we’ll optionally drop smoothing as well.) 

* **Gate/peak logic** (we keep the logic, drop normalization and CFO): whenever the metric rises above a **fixed threshold**, open a gate; while open, track the **peak value and its index**; when it falls back below the threshold, **finalize** detection at the **peak index** and assert one flag there. This mirrors the C “gate_active/peak tracking” but with `timing_offset = 0`, `cfo_valid = false`, and `normalize_enabled = false`. 

### Key hardware trick: collapse to a single product stream

Define a per‑antenna delayed product
[
p[n] \triangleq x[n]\cdot \overline{x[n+Q]}.
]
Observe:

* ( C_1(s) = \sum_{n=s}^{s+Q-1} p[n] ).
* ( C_2(s) = \sum_{n=s+2Q}^{s+3Q-1} p[n] ) (same (p[\cdot]), just offset by (2Q)).
  So we can compute **one complex product per antenna per sample**—just a Q‑delayed self‑product—and form both (C_1) and (C_2) via two **boxcar (Q‑tap) moving sums** on the *same* (p[\cdot]) stream at two different time offsets. Addition is cheap; multiplies are not. We will exploit that.

For energies, let ( e[n] = |x[n]|^2 ) **summed across antennas**. Then, as your C code implicitly does via three quarter sums, the total energy denominator is simply a **3Q‑tap moving sum**:
[
E(s) = \sum_{n=s+Q}^{s+4Q-1} e[n] = \sum_{n=t-3Q+1}^{t} e[n],
]
where ( t ) is the time index when window start (s) becomes current (see timing in §4). 

**Result**: one CMUL/antenna/sample + three moving sums (two complex Q‑tap for (C_1, C_2) from (p), and one real 3Q‑tap for (E)). That’s the whole Minn engine.

---

## 2) Fixed‑point plan (safe, not stingy)

Assume **12‑bit signed** I and Q at the input (two’s complement).

**Per sample:**

* ( |x|^2 = I^2 + Q^2 ): each square is 12×12 → 24b; sum → **25b unsigned** (use 26b to be safe).
* ( p[n] = x[n]\cdot \overline{x[n+Q]} ): real part (=\ I_1I_2 + Q_1Q_2), imag part (=\ Q_1I_2 - I_1Q_2). Each product 12×12 → 24b; sum/diff → **25b signed** (use 26–27b).

**Moving sums:**

* Complex **Q‑tap** sum (Q ≤ 512) adds ≈ 9 bits headroom → **Re/Im ~ 36b signed**.
* Complex **(C(s) = C_1+C_2)**: add one more bit margin → **37b signed** on Re (Imag unused for metric, but keep parity).
* Real **3Q‑tap** energy sum adds ≈ 11 bits → **~36–37b unsigned** (use **38b** margin).

**Threshold compare without division (recommended)**
You want to gate on metric ≥ T (T in [0,1)). Avoid divides/squares in hardware:
[
\frac{R}{E} \ge K \quad\text{with}\quad K \triangleq \sqrt{T}.
]
Compute **RHS = (E\cdot K)** with K in **Q1.15** (or Q1.16). Compare **(R << 15) ≥ E·K**. One constant multiply and a shift; no divider, no 72‑bit square. The reference uses `gate_threshold` ~0.2 by default; picking (K=\sqrt{0.2}\approx0.4472) in Q1.15 is perfectly fine .

**Saturation/rounding:**

* Saturate accumulators on overflow (never wrap on a detector).
* Round when down‑scaling for optional debug outputs; keep full internal precision for the compare.

---

## 3) Top‑level I/O and parameters (SystemVerilog stance)

**Parameters** (static at synthesis or loaded via CSR):

* `N_FFT` (e.g., 2048), `Q = N_FFT/4` (compile‑time assert divisible by 4), **checked as in your header** .
* `K_Q15` (sqrt(threshold) in Q1.15).
* `SMOOTH_LEN` (optional, set to 1 to disable).

**Clocks & handshakes**

* `clk_fabric` at 184.32 MHz (or higher).
* `in_valid` pulses **once every 6 cycles** when a new sample pair is present; or use AXI‑Stream with `tvalid` and a 6‑cycle pacing.
* Outputs run at the same paced rate (one output sample per input sample after pipeline delay).

**Ports**

* Inputs: `ch0_i[11:0], ch0_q[11:0], ch1_i[11:0], ch1_q[11:0], in_valid`.
* Outputs: delayed streams `out_ch0_i[11:0], out_ch0_q[11:0], out_ch1_i[11:0], out_ch1_q[11:0], out_valid`, and `detect_flag` (1 on the single peak sample, else 0). Optional `metric_dbg` (e.g., 16b).

Default values (FFT sizes, sample‑rate) match your `modem_config.c` (e.g., for 20 MHz BW: sample‑rate = 30.72 Msps, FFT = 2048), and `minn` defaults show `gate_threshold=0.2`, `smooth_window=16`—we’ll keep smoothing optional and default to “off” in RTL to honor your simplifications. 

---

## 4) Micro‑architecture

### 4.1 Dataflow (per sample pair)

**Stage A — Front‑end & delays (per antenna):**

* **Q‑delay line** for `x[n]` (complex).
* On each `in_valid`:

  * Compute ( e_{\text{ant}}[n] = I^2 + Q^2 ) (store to a 3Q energy FIFO after antenna sum).
  * Compute ( p_{\text{ant}}[n-Q] = x[n-Q]\cdot \overline{x[n]} ) using the Q‑delayed copy and current sample.

**Stage B — Antenna sum:**

* ( p_{\text{sum}}[n-Q] = p_0[n-Q] + p_1[n-Q] ).
* ( e_{\text{sum}}[n] = e_0[n] + e_1[n] ).

**Stage C — Moving sums (single‑stream, cheap):**

* **SumB (Q‑tap on p)**: window ([t-2Q+1,,t-Q]). Update as `sumB = sumB + p[n-Q] − p[n-2Q]`.
* **SumA (Q‑tap on p delayed by 2Q)**: feed `p` through a **2Q delay**, then `sumA = sumA + p[n-3Q] − p[n-4Q]`.
* **Energy (3Q‑tap on e)**: `E = E + e[n] − e[n-3Q]`.

**Stage D — Metric & gate:**

* ( C = \text{sumA} + \text{sumB} ).
* ( R = \max(0, \Re{C}) ).
* Compare **(R << 15) ≥ E * K_Q15** → `above_threshold`.
* Optional simple smoothing: boxcar or IIR over the **ratio** (costly) or—preferable—over **R** and **E** separately with the same gain and compare ( \frac{\langle R\rangle}{\langle E\rangle} ) with the same (K). Given your “keep it simple,” ship with `SMOOTH_LEN=1` (off). This aligns to the “raw smoothed metric” idea in the C code but without the adaptive normalization path we disabled. 

**Stage E — Peak logic and *pending* queue:**

* **Gate FSM** (exactly like the C logic but simpler):

  * If `above_threshold` and gate is **closed**, **open** and initialize peak=(R, idx).
  * While open, update peak if `R` increases.
  * When `above_threshold` de‑asserts, **finalize**:

    * **Detection index** = `peak_idx` (no timing offset; we deliberately diverge from the C offset option) .
    * **Back‑annotate** a `1` into a **pending flag FIFO** at the slot corresponding to `peak_idx`. That’s the same idea as the reference’s `pending` ring and `gate_peak_queue_pos`; you’re writing a future flag into a not‑yet‑emitted entry. We carry no CFO value. 
* **Output delay alignment**:

  * The metric computed at time (t) corresponds to a window starting at ( s = t-4Q+1). To **output the sample at index (s) when you compute/decide the metric**, delay the raw sample stream by **(D_\text{out} = 4Q-1)** samples (plus pipeline latency).
  * Practically: implement a **sample FIFO** deep enough to hold (4Q + \text{gate_length_max} + \text{pipeline}). During an open gate you either (A) stall popping like the C version does (simplest), or (B) keep popping and size the FIFO so the peak slot is still in the buffer when you finalize. The C code stalls popping while the gate is active; that’s fine here too and makes the flag write trivial. 

**What we deliberately remove vs. the C code**

* **Adaptive normalization** and its state machine: gone (set `normalize_enabled=false`), consistent with your simplification and the defaults in config. 
* **CFO estimation** (angle of correlation across Q): gone. The C reference computes it as (-\angle(C)/ (2\pi Q) \cdot f_s); we don’t implement it. 
* **Timing offset**: set to **0**; your flag is asserted on the peak itself, not shifted. The header exposes that parameter, but we lock it to zero in RTL. 

---

## 5) Time‑multiplexing plan @ 6 cycles per sample

You have **6 fclk cycles** per input sample. Use them ruthlessly.

**Minimum compute required per input sample** (for both antennas total):

* **2 complex products** to form (p_0[n-Q]) and (p_1[n-Q]).
* **4 real squares** for (e_0, e_1) (I² and Q² per antenna).
* Several additions/subtractions for moving sums (can ride in parallel with multiply pipelines).

**Pragmatic schedule**

* Instantiate **one complex‑multiply (CMUL) unit**, time‑share across the two antennas over the 6‑cycle slot. A CMUL built from **two real multipliers per cycle** over **3 pipelined cycles** (Gauss or classical) gives plenty of headroom.
* Instantiate **one real multiplier** time‑shared for I²/Q² across both antennas (4 squares in 4 cycles), or **two** if your target DSPs are cheap—either option fits 6 cycles.
* Move‑sum updates (adds/subtracts & FIFO pointers) are single‑cycle and can sit in the interstitial cycles.

**Why not the obvious direct sliding update?** Because that needs **8 complex multiplies + 8 |·|² per sample** across two antennas (as the C sliding update shows when you subtract/add four pairs per antenna), which wildly exceeds your per‑sample cycle budget. The product‑stream trick eliminates **6/8 of the CMULs** on paper and lets one CMUL do the job comfortably. The C version’s rolling sums validate that those terms are exactly the ones we synthesize with boxcars. 

---

## 6) Storage plan (BRAM/FIFO)

For **N = 2048, Q = 512** (20 MHz profile) :

* **Q‑delay line** per antenna for complex samples: depth 512 × (I,Q 12b each) → ~12 kb per antenna (use SRL or small BRAM).
* **p‑stream delay**:

  * One **Q‑depth complex FIFO** for SumB subtraction (store last Q values of (p)): 512 × (2×~27b) ≈ 27 kb.
  * One **2Q delay** (for SumA’s input offset): 1024 × (2×~27b) ≈ 54 kb.
  * One **Q‑depth complex FIFO** for SumA subtraction: another ~27 kb.
    You can merge these as a single **3Q deep complex circular buffer** with two read taps (at delays Q and 3Q) and two write‑back pointers; that’s the most compact.
* **Energy FIFO**: **3Q** real values for E’s subtraction: 1536 × ~26b ≈ 40 kb.
* **Pending output FIFO**: store samples and a 1‑bit flag for at least (4Q + \text{margin}). The C code uses `output_delay = N` and stalls popping while the gate is active, which reduces margin needs and is very easy to implement; copy that. 

All numbers above are conservative—actual bit‑widths after synthesis/STA may let you down‑size some FIFOs.

---

## 7) Control FSMs

1. **Sample scheduler** (6‑cycle tick): asserts a micro‑counter 0..5 for time‑sharing the multiplies and orchestrating FIFO reads/writes.

2. **Gate/peak FSM**:

   * `IDLE` → `GATE_OPEN` on first `above_threshold`.
   * Track (peak_value, peak_qpos) **inside the pending queue** while open.
   * `GATE_OPEN` → `DETECT` when `above_threshold` drops; write flag `1` into `pending[peak_qpos]`.
   * Resume output popping if it was paused. (That mirrors the reference’s `pending_can_pop` gating) .

3. **Output arbiter**: if you adopt **stall‑while‑gate**, simply hold `out_valid=0` while gate is open, otherwise keep popping. The C logic pauses popping during a gate; same here for the simplest correct behavior. 

---

## 8) Latency and alignment

* **Algorithmic group delay**: (D_\text{alg} = 4Q-1) samples from the first time the window is fully valid (the “−1” aligns the start‐index arithmetic; you can choose +/−1 convention and be consistent).
* **Pipeline delay**: CMUL + adder trees + FIFO read latencies (usually a handful of fabric cycles).
* **Gate finalization delay**: variable; if you stall outputs while the gate is open (simple mode), the **net external delay** becomes fixed again (you resume as soon as you flag the peak). This matches the reference’s “don’t pop while gate_active” design. 

---

## 9) Verification strategy (mirrors the C reference)

* **Golden model**: compile your C detector with `normalize_enabled=false`, `timing_offset=0`, and a chosen `gate_threshold`. Drive it with the same sample streams as the RTL and compare:

  * `detect_flag` pulse index,
  * delayed I/Q outputs at that index,
  * (optional) raw/smoothed metric traces.
    The public API and params are declared in `minn_detector.h`; defaults and FFT sizing come from `modem_config.c`.  
* **Corner cases**: no detection in a frame (flag must never pulse, outputs stream as pure delay); two close detections (ensure gate closes before the next opens; if not, widen the “drop‑out” condition by one sample of hysteresis—still a **single** threshold, just require strictly `< T` to close).

---

## 10) SystemVerilog implementation roadmap

**Milestone 1: Bit‑exact math kernel**

1. Parameter block and assertions: `N_FFT`, derive `Q=N_FFT/4`. **Error if N not divisible by 4** (as your C init does)  .
2. Front‑end delays and **p‑stream** generation (Q‑delay + CMUL) on both antennas; antenna sum to a single complex stream.
3. Implement the three moving sums (SumA, SumB, Energy) with circular buffers; verify against C for a few symbols.

**Milestone 2: Thresholding and gating**
4. Implement **(R << 15) ≥ E·K_Q15** compare; verify threshold crossings vs. C with normalization disabled.
5. Gate/peak FSM exactly as above; add **pending** FIFO for flags/samples. This mirrors the C `pending` queue approach and its “pause popping while gate_active” guard. 

**Milestone 3: Interfaces & timing polish**
6. AXI‑Stream or ready/valid wrappers, 6‑cycle scheduler.
7. Parameterize thresholds via CSR; optional export of `metric_dbg`.
8. Synthesis‑time guards for bit‑widths and FIFO depths; run post‑synth sim to confirm no overflow on worst‑case vectors.

**Milestone 4: (Optional) niceties**
9. Add **light smoothing** by equal‑gain boxcar on (R) and (E) before compare (keeps the no‑divide compare). Keep default OFF (1‑tap).
10. Expose a CSR to choose stall‑while‑gate vs. deep‑queue non‑stall mode.

---

## 11) Resource and timing notes

* **DSPs**: 1 CMUL (time‑shared across antennas) + 1–2 real multipliers for I²/Q² is usually under a dozen DSP blocks total.
* **BRAM**: dominated by the **3Q energy FIFO** and the **3Q complex p buffer**. For Q=512, expect ~120–130 kb before packing optimizations.
* **Fmax**: 184.32 MHz is comfortable if you pipeline CMUL to ≥3 stages and register every BRAM/FIFO read path.

You *could* do it with two CMULs to make scheduling trivial, but I’d start with one; the product‑stream architecture is what saves you.

---

## 12) Hard opinions & pitfalls

* **Don’t** implement the textbook sliding update that recomputes four CMULs per antenna per sample; your 6‑cycle budget will cry. The product‑stream approach is *exactly* equivalent and vastly cheaper.
* **Don’t** normalize adaptively in RTL unless field data proves you need it; a fixed threshold plus a sane boxcar optional smoother is plenty. The C code’s normalization logic is clever but adds state you’ve explicitly said you don’t want. 
* **Be precise** about which sample the flag lines up with. In this design, it’s the **window start index** (the metric’s `s`), exactly as your reference does when `timing_offset=0`. If you later want a small negative bias (common to tuck the FFT start inside CP), that’s just a CSR addend to the back‑annotation pointer—no math changes.  

---

## 13) Quick “block map” you can hand to the RTL engineer

1. **Input capture & 6‑cycle scheduler**
2. **Per‑antenna Q‑delay** → **CMUL with conj** → (p_0, p_1)
3. **Antenna sum**: (p = p_0 + p_1), (e = |x_0|^2 + |x_1|^2)
4. **p buffer (3Q deep)** with two taps:

   * **SumB (Q‑tap)** on `p[n-Q]`
   * **2Q delay → SumA (Q‑tap)** on `p[n-3Q]`
5. **Energy FIFO (3Q)** → **E = sum(e) over 3Q**
6. (C = \text{SumA} + \text{SumB}), (R=\max(0,\Re C))
7. **Compare** `(R<<15) ≥ E*K_Q15` → **gate FSM** w/ peak track
8. **Pending FIFO** for samples + retro flag write at `peak_qpos`
9. **Output** delayed samples + `detect_flag` single‑pulse

---

## 14) What maps to what in your C

* `quarter_len = fft_size/4` → `Q = N/4` (compile‑time assert), directly from your header. 
* Sliding window math and the “gate/peak” semantics are from `minn_detector_process_one` (we mirror the same peak tracking and the **pause popping while gate_active** behavior so the back‑annotation is safe). 
* No adaptive normalization: set `normalize_enabled=false` like in your default args; we don’t carry `norm_noise_estimate`. 
* No CFO: we drop the angle‑based estimate in the C code entirely. 

---

## 15) Final calibration tips

* Start thresholds around **T = 0.2** (so (K \approx 0.447)) as in your defaults. Verify on injected preambles at modest SNR; adjust per radio front‑end scaling. 
* If you see chattering at the edges of the gate, require **strictly below** threshold to close (no second threshold—just make the inequality strict on close).
* If field CFO is non‑negligible and you later want a coarse estimator, you can re‑introduce the C expression ( \text{CFO} = -\angle(C)/(2\pi Q)\cdot f_s ) with **no effect on the detection path**—export it on a sideband. 

---
