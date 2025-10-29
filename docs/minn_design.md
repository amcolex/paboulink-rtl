Below is a standalone, FPGA‑ready specification for a **streaming OFDM preamble detector** based on the Minn timing metric family, rewritten to be practical in RTL. It takes **two complex input channels** (I/Q for each), processes **one complex sample per clock** (parallel or time‑multiplexed), and produces a **continuous, delayed stream** of I/Q samples with a **frame‑start flag** aligned to the detected preamble. We explicitly **drop adaptive/dynamic thresholding** and **drop CFO estimation**, per your constraints.

> TL;DR for the FPGA engineer:
> • No divides, no CORDIC, no angle math.
> • One quarter‑lag complex dot‑product stream + one power stream per antenna.
> • A single Q‑length running sum feeds both “quarter pairs”; energies are a Q‑length sum with two delayed taps.
> • Decision is a constant‑threshold compare using **cross‑multiplication**, not division.
> • Gate-then‑peak logic sets a one‑cycle frame flag into a delayed output FIFO.
> • Default output delay = **N = FFT size** (safe, simple).

---

## 1) Context and signal model

* **OFDM training layout**: assume an OFDM preamble/training symbol with **four quarters** of length (Q = N/4) samples (N is the FFT size, divisible by 4). The training field is designed so that **adjacent quarters are strongly correlated**—that’s the handle we grab.
* **Two antennas / two channels**: you have **ch0** and **ch1**, both complex baseband streams, **time‑aligned** at the receiver front end.

The supplied C reference uses two sliding quarter‑lag correlators per antenna (between quarter 0↔1 and 2↔3), three quarter‑energies, a smoothed/normalized metric, and a gate‑and‑peak detector; it also estimates CFO from the correlation phase and delays output by one training‑symbol (N) to stick the flag on the right sample. We mirror the correlation/energy core but **remove** both adaptive normalization / dynamic thresholding and CFO, and we keep the **N‑sample output delay** behavior.  

---

## 2) Math (cleaned up for streaming hardware)

We build everything from **two simple streams** per antenna:

* Quarter‑lag complex product (real part only):
  [
  p_a[n] ;=; \Re{,x_a[n-Q]\cdot x_a^*[n],}
  \quad\text{(antenna } a \in {0,1}\text{)}
  ]
* Instantaneous power:
  [
  w_a[n] ;=; |x_a[n]|^2 ;=; I_a[n]^2 + Q_a[n]^2
  ]

Now define **Q‑length running sums** of those streams:

* Running sum of the quarter‑lag product (one per antenna):
  [
  R_a[n] ;=; \sum_{k=n-Q}^{n-1} p_a[k]
  ]
* Running sum of power (one per antenna):
  [
  E_a[n] ;=; \sum_{k=n-Q}^{n-1} w_a[k]
  ]

> Key trick: with just **one** Q‑length running sum per stream, we get the “two quarter pairs” and “three quarter energies” **for free** via **delayed taps**:

* For the Minn‑style two correlators (quarter 0↔1 and 2↔3) at time (n):

  * (r_{2,a}[n] = R_a[n]) (correlation of the **most recent** two quarters)
  * (r_{1,a}[n] = R_a[n-Q]) (correlation of the **previous** two quarters)
* For the three quarter‑energies:

  * (e_{3,a}[n] = E_a[n])
  * (e_{2,a}[n] = E_a[n-Q])
  * (e_{1,a}[n] = E_a[n-2Q])

Total correlation (real) and total energy across **both antennas**:
[
C[n] ;=; \sum_{a\in{0,1}} \Big(r_{1,a}[n] + r_{2,a}[n]\Big)
\quad , \quad
\mathcal{E}[n] ;=; \sum_{a\in{0,1}} \Big(e_{1,a}[n]+e_{2,a}[n]+e_{3,a}[n]\Big)
]

Minn/plateau‑free-ish timing uses a **positive‑real detector** (CFO reduces (\Re{\cdot}) but the 2‑quarter trick mitigates some rotation):

[
C_+ [n] ;=; \max\big(C[n], 0\big)
]

### Thresholding without division (FPGA‑friendly)

The classic metric is (M[n] = \dfrac{C_+[n]}{\mathcal{E}[n]}) or squared variants. Division is ugly in RTL, so we **cross‑multiply**:

[
C_+[n] ;\ge; T \cdot \mathcal{E}[n]
]

* (T) is a **fixed, static threshold** in **Q0.F** fixed‑point (e.g., Q1.15).
* Implementation: multiply (\mathcal{E}[n]) by constant (T) (one DSP) and compare to (C_+[n]).
* If your dynamic range is large, you can scale both sides by a shared power‑of‑two to keep bit widths neat—no change in logic.

> In the C reference, the raw Minn metric is ((\max(\Re{c_1+c_2},0))^2 / (\text{energy sum})^2) with optional smoothing/normalization/gating; we keep the **same discriminator** but use the unsquared form and **constant threshold**, which is equivalent after re‑scaling and is cheaper to implement. 

### Optional smoothing, still cheap

For jitter reduction, apply a **single‑pole IIR** to (C_+[n]) only:
[
\tilde{C}[n] = \tilde{C}[n-1] + \frac{C_+[n]-\tilde{C}[n-1]}{2^S}
]
Choose shift (S\in[2,5]). Compare (\tilde{C}[n]) to (T\cdot\mathcal{E}[n]). This costs **one subtract, one add, one shift**. (The C reference does a boxcar smoother; an IIR with power‑of‑two gain is more FPGA‑friendly. If you prefer exact parity, you can implement a length‑L boxcar with another running sum and an L‑deep FIFO.) 

---

## 3) Gate‑and‑peak decision logic (no CFO, static threshold)

We keep the standard **gate‑then‑peak** strategy:

1. **Gate open** when (\tilde{C}[n] \ge T\cdot\mathcal{E}[n]).
2. While gate is open, **track the peak**: keep the max (\tilde{C}[n]) and its sample index (n_\text{peak}).
3. **Gate close** when the inequality is false for **H hysteresis** samples (H=0 or small integer to avoid chatter).
4. On gate close, **declare detection** and set the **frame flag** for the sample at
   [
   n_\text{flag} = n_\text{peak} + \texttt{timing_offset}
   ]
   The **timing_offset** aligns flag to your desired downstream boundary (start of the first long‑FFT symbol, cyclic‑prefix boundary, etc.). The C code does the same offsetting. We **do not** compute or use CFO. 

This exactly mirrors the reference design’s gating/peaking and “write‑back a flag into a delayed queue,” minus the adaptive normalization and CFO angle/Hz computation. The original sets the output delay to **N** and back‑patches a flag at the proper queue position; we’ll do the same in RTL with a circular BRAM. 

---

## 4) Streaming reformulation (one‑sample/clock)

Everything updates with **O(1)** work per sample using FIFOs and running sums.

### Per‑antenna datapath

For each antenna (a):

1. **Quarter delay**: a Q‑deep FIFO gives (x_a[n-Q]).
2. **Quarter‑lag product** (real part only):
   [
   p_a[n] = I_a[n-Q]\cdot I_a[n] + Q_a[n-Q]\cdot Q_a[n]
   ]
   (Only **two real multipliers** and one adder — conjugation is free: flip sign on Q when needed; for the real part it cancels naturally.)
3. **Running sum of products** (Q window):
   Maintain FIFO (P_a) of length Q and accumulator (R_a):
   [
   R_a \leftarrow R_a + p_a[n] - P_a.\text{pop_oldest}();\quad P_a.\text{push}(p_a[n])
   ]
4. **Running sum of power** (Q window):
   Compute (w_a[n]=I_a[n]^2 + Q_a[n]^2) (two real multipliers + one adder), keep FIFO (W_a) and accumulator (E_a):
   [
   E_a \leftarrow E_a + w_a[n] - W_a.\text{pop_oldest}();\quad W_a.\text{push}(w_a[n])
   ]
5. **Delayed taps**:

   * Use a simple **Q‑deep delay** on (R_a) to get (R_a[n-Q]) for (r_{1,a}).
   * Use two **Q‑deep delays** on (E_a) to get (E_a[n-Q]) and (E_a[n-2Q]) for (e_{2,a}) and (e_{1,a}).

### Cross‑antenna combine (per sample)

* (C[n] = \sum_a \big(R_a[n] + R_a[n-Q]\big))
* (\mathcal{E}[n]=\sum_a \big(E_a[n] + E_a[n-Q] + E_a[n-2Q]\big))
* (C_+[n] = \max(C[n],0))
* Optional smoothing (\tilde{C}[n]).
* Compare (\tilde{C}[n]) vs (T\cdot \mathcal{E}[n]).
* Gate/peak and **flag back‑patch** as below.

This is precisely the same math the C code implements with a ring buffer and “subtract old, add new” slides; we’ve expressed it in a pure streaming form with **one** Q‑sum and **delayed taps** instead of separately recomputing two disjoint Q‑sums. 

---

## 5) Output delay, flagging, and FIFO organization

* Maintain an **output circular buffer** (dual‑port BRAM) that stores, per input sample:

  * ch0 sample (I/Q), ch1 sample (I/Q)
  * a **1‑bit frame flag** (defaults to 0)
  * (optional) **metric tap** for debug (truncate (\tilde C) if desired)

* Let the buffer depth be (\texttt{DEPTH} \ge N + \text{margin}) (margin ≥ max{smoothing lag, hysteresis H}). The C code simply uses **DEPTH = N** and it works in practice; if you run extremely long gates or large smoothing, add a small cushion. 

* Push each incoming pair of ch0/ch1 samples into the circular RAM (write pointer ++). On **detection**, compute the **physical index** of the flagged sample:
  [
  \text{flag_addr} = (\text{write_ptr_at_peak} + \texttt{timing_offset}) \bmod \texttt{DEPTH}
  ]
  and set its **flag bit = 1** by a second write on the **other RAM port** (no stalls). This back‑patch mirrors the pending‑queue write‑back used in software. 

* On every clock after an initial latency, **read out** the buffer at a fixed **read pointer** that trails the write pointer by **OUTPUT_DELAY = N** samples. That produces a **continuous, delayed stream** with a correctly aligned single‑cycle **frame_start flag**.

---

## 6) Time‑multiplexed vs. parallel channels

Two equivalent implementations:

* **Parallel** (simplest): duplicate per‑antenna datapath (items 1–5 above) and combine at the end. One sample/clock per antenna lane.
* **Time‑multiplexed** (if IO delivers ch0 then ch1 on alternate cycles): share the multipliers/adders at **2× clock** or operate on alternating cycles at the **same clock**. Keep **separate state** (FIFOs, accumulators, delays) per antenna; multiplex inputs and demultiplex internal states by a channel bit. The combine step runs whenever a **pair** (ch0,ch1) for the same time index is available; use a small aligner if streams are staggered.

---

## 7) Fixed‑point plan (concrete widths)

Let input I/Q be signed **W_IN** bits (typical: 12–14). Define:

* **Power**: (w = I^2 + Q^2) → width (W_w = 2\cdot W_{IN}+1).

* **E (Q‑sum of power)**: add (\lceil \log_2 Q\rceil) → (W_E = W_w + \lceil \log_2 Q\rceil).

  * Three‑quarter sum per antenna adds (\lceil \log_2 3\rceil=2) → (W_{E3} = W_E + 2).
  * Sum across two antennas adds 1 → **(W_{\mathcal{E}} = W_E + 3)**.

* **Quarter‑lag product (real)**: (p = I_{-Q}I + Q_{-Q}Q)

  * Each product (I_{-Q}I) is (2\cdot W_{IN}) bits; the sum adds 1: (W_p = 2\cdot W_{IN}+1).

* **R (Q‑sum of p)**: (W_R = W_p + \lceil \log_2 Q\rceil).

  * Two‑quarter sum per antenna (r1+r2) adds 1 → (W_{r\Sigma a} = W_R + 1).
  * Sum across antennas adds 1 → **(W_C = W_R + 2)**.

* **Threshold multiply**: (T) as **Q1.F** (e.g., Q1.15). Scale (\mathcal{E}) by (T) using one DSP.
  **Compare**: (C_+[n]) (truncate/round to match (T\cdot\mathcal{E}) scale).

> Example (N=256 ⇒ Q=64, W_IN=12):
> (W_w=25), (W_E=31), (W_{\mathcal{E}}=34);
> (W_p=25), (W_R=31), (W_C=33).
> These are friendly to 18×25 / 27×18 DSP slices. Use **saturating adders** anywhere sums might overflow.

---

## 8) Control and corner cases

* **Reset/flush**: clear FIFOs, accumulators, delays, gate state, and set output RAM flag bits to 0. Warm‑up takes (2Q) for R delays and (3Q) for E delays; until then, force the comparator low (no gate).
* **Timing offset**: signed, small integer (e.g., –32…+32). If the computed back‑patch address falls outside the buffered window (shouldn’t, with DEPTH≥N), clip to the nearest valid entry or ignore (config choice). The reference applies this offset on the latched peak index. 
* **Threshold tuning**: CFO reduces (\Re{\cdot}) by (\cos(\phi)). Because we **don’t correct CFO**, set (T) with headroom for the worst expected CFO across a quarter. Start low (e.g., 0.08–0.15 in Q1.15 for many systems) and adjust with lab data.
* **Hysteresis** (H): if noise flickers near threshold, require **H consecutive lows** to close the gate (H=1–4 is typical). This is just a tiny down‑counter.

---

## 9) Interfaces

**Inputs** (per clock):

* `ch0_i, ch0_q, ch1_i, ch1_q` (or one channel per clock with `ch_sel`)
* Optional `in_valid`/`in_ready` handshake (AXI‑Stream friendly).

**Outputs** (per clock, continuous):

* `out_ch0_i, out_ch0_q, out_ch1_i, out_ch1_q` (delayed by N + small margin)
* `frame_start` (1 when the delayed sample equals (n_\text{flag}), else 0)
* Optional `metric_dbg` (truncated (\tilde C)).

**Latency**: dominated by the output buffer (**≈ N** clocks), plus a few pipeline stages.

The C reference uses an **N‑sample output delay** and consolidates the decision/flag into that delayed queue; mirroring this makes system integration painless. 

---

## 10) Micro‑architecture blocks (RTL sketch)

1. **Per‑antenna input pipe**

   * Q‑deep complex delay for (x[n-Q]) (BRAM or SRL).
   * Two multipliers + adder for (p[n]).
   * (P) FIFO (Q deep) + accumulator (R) (add‑sub in one DSP‑adder chain).
   * Two multipliers + adder for (w[n]).
   * (W) FIFO (Q deep) + accumulator (E).
   * Q‑deep delays on (R) and on (E) to generate taps (R[n-Q], E[n-Q], E[n-2Q]).

2. **Combiner**

   * Add (R_0 + R_0^{(-Q)} + R_1 + R_1^{(-Q)}) ⇒ (C).
   * Add (E_0 + E_0^{(-Q)} + E_0^{(-2Q)} + E_1 + E_1^{(-Q)} + E_1^{(-2Q)}) ⇒ (\mathcal{E}).
   * Clamp negative (C) to zero ⇒ (C_+).
   * Optional IIR smoother ⇒ (\tilde C).
   * Constant multiply: (T \cdot \mathcal{E}).
   * Comparator ⇒ `above_thresh`.

3. **Gate & peak tracker**

   * FSM with states: IDLE → GATE_OPEN → PEAK_HELD (on close) → IDLE.
   * Track `peak_val`, `peak_wptr` (write pointer snapshot).
   * On close, compute `flag_addr = (peak_wptr + timing_offset) mod DEPTH`.

4. **Output buffer**

   * Dual‑port circular BRAM with structs: `{ch0_i, ch0_q, ch1_i, ch1_q, flag}`.
   * Port A: sequential writes of current samples.
   * Port B: **random flag write** at `flag_addr`.
   * Read pointer = write pointer delayed by `OUTPUT_DELAY = N`.
   * Stream out one entry per clock.

5. **Time‑multiplex option** (if needed)

   * Add a `ch_sel` toggle, maintain **two copies** of (P,W,R,E) state, and process alternating samples.
   * Combine only on the “phase” where both channels for the same time index have been seen.

Data hazards: none, provided the back‑patch write uses the second BRAM port and the read port is independent.

---

## 11) Resource notes (per antenna lane)

* **Multipliers**: 4 DSPs (2 for (p), 2 for (w)).
* **Accumulators / adders**:

  * Two running sums (R and E): each needs one add and one sub per clock; map to DSP adder with pre‑adder or LUT adders.
  * A few adders for combining taps and antennas.
* **BRAM / SRLs**:

  * Q‑deep delay for (x[n-Q]) (complex) — typically SRLs for small Q, BRAM for large Q.
  * Two Q‑deep FIFOs (p and w) per antenna — depth = Q, width ≈ 25–31 bits per entry.
  * Q and 2Q delays for R and E taps — can be SRL32 chains or BRAM.
  * Output circular buffer — depth (≥ N); width = 4×W_IN + flag (+ optional metric).

If you’re multiplier‑tight, you can:

* **Share** the power multipliers across antennas in a time‑multiplexed design.
* Implement **threshold scale** as a shift‑add constant if T is chosen as a power‑of‑two fraction.

---

## 12) Behavioral parity vs. the C reference

* **Same core math**: two quarter‑lag sums + three quarter energies per antenna, summed across antennas; positive‑real projection; gate + peak + timing offset; **output delay N**; delayed flag back‑patch in a pending/circular queue. 
* **Dropped**: adaptive smoothing/normalization and **dynamic threshold**; **CFO estimate** from the correlation angle. (The C code computes CFO from the phase of the peak correlation and stores it; we do not.) 
* **Interface parity**: your RTL outputs a continuous delayed stream with a frame flag, analogous to the software path that pushes samples to a pending queue and emits them with a delay and a flag when ready. The C header embodies this delayed‑stream contract. 

---

## 13) Practical tuning guidance

* **Threshold (T)**: start conservative (e.g., Q1.15 value 0x1400–0x2000). Sweep with recorded air‑captures. You’re balancing false alarms (too low) vs. missed detections under CFO/fading (too high).
* **IIR shift (S)**: (S=3) is a sweet spot; bigger (S) smooths more but delays the gate’s edges.
* **Hysteresis (H)**: 1–4 samples kills chatter.
* **Timing offset**: small negative values can land you neatly on the start of the **long‑training field** (depends on your preamble design).

---

## 14) Verification plan (succinct)

1. **Unit tests**:

   * Perfect preamble, no noise: single crisp detection; flag at expected offset.
   * Add AWGN; sweep (E_b/N_0), verify Pd/Pfa vs. (T).
   * Add CFO (e.g., ±1% subcarrier spacing): verify Pd with fixed T; adjust (T) if needed.
   * Multipath with mild ISI: verify plateau narrowing still yields a unique peak.

2. **Bit‑true vs. software**:

   * Run the software metric path **with adaptive normalization disabled** and CFO ignored; match fixed‑point RTL by quantizing the C path. The C code already exposes raw smoothed metrics and the gating path; treat that as your numerical oracle for bit‑true tests. 

---

## 15) Pseudocode (1 sample/clk, parallel antennas)

```text
on each clock with valid sample x0, x1:

// === antenna a in {0,1} ===
x_a_del = delay_Q(x_a)
p_a     = dot_real(x_a_del, x_a)              // I*I + Q*Q  (2 mul + add)
R_a    += p_a - fifo_P_a.pop_push(p_a)        // running sum over Q
w_a     = I*I + Q*Q                            // 2 mul + add
E_a    += w_a - fifo_W_a.pop_push(w_a)        // running sum over Q
R_a_d1  = delay_Q(R_a)
E_a_d1  = delay_Q(E_a)
E_a_d2  = delay_Q(E_a_d1)

// === combine both antennas ===
C       = (R_0 + R_0_d1) + (R_1 + R_1_d1)
E_tot   = (E_0 + E_0_d1 + E_0_d2) + (E_1 + E_1_d1 + E_1_d2)
C_pos   = max(C, 0)
C_smooth= C_smooth + (C_pos - C_smooth) >>> S  // optional

above   = (C_smooth >= T * E_tot)

// gate/peak FSM
if (!gate && above) { gate = 1; peak_val = C_smooth; peak_wptr = write_ptr; }
if (gate) {
   if (C_smooth > peak_val) { peak_val = C_smooth; peak_wptr = write_ptr; }
   if (!above for H samples) {
       gate = 0;
       flag_addr = peak_wptr + timing_offset mod DEPTH;
       ram[flag_addr].flag = 1;
   }
}

// output circular buffer
ram[write_ptr] = {x0, x1, flag=0}
write_ptr++
read_ptr = write_ptr - OUTPUT_DELAY (mod DEPTH)
emit ram[read_ptr]
```

---

## 16) What changed vs. the drop‑in C code (for the implementer)

* We **replaced** the in‑place ring‑buffer sliding (subtract old/add new) with **streaming running sums** and **delayed taps**—same arithmetic, less random access, perfect for RTL. The C’s internal variables `corr_c1/c2` and `energy_q1/q2/q3` map directly to ((R[n-Q], R[n])) and ((E[n-2Q], E[n-Q], E[n])) here. 
* We **kept** the pending/output queue semantics and **N‑sample delay**, but we implement it as a circular BRAM with dual‑port back‑patch of the flag. This mirrors the software’s `pending` queue and `output_delay = window_len = N`. 
* We **removed** adaptive normalization and dynamic thresholding (`normalize_enabled`, `norm_*` fields) and CFO (`last_cfo_estimate_hz` path). Your RTL comparator uses a **fixed, programmable threshold** register instead.  

---

### Final notes

* If someone proposes “let’s still divide to get a pretty metric,” veto it. The **cross‑multiply** compare is mathematically equivalent for thresholding and saves you a divider.
* If CFO turns out to be rough in your channel, don’t resurrect angle math in hardware. Either **lower (T)** a bit or add a **small FIR smoother** on (C_+). If that’s still not enough, a later, **post‑FFT CFO** corrector is a better place to spend gates.
* This detector is embarrassingly parallel. If you later add more antennas, just replicate the per‑antenna lane and widen the adders.

If you want, I can translate this into a module breakdown with AXI‑Stream ports, parameter generics (N, Q, T, S, H, timing_offset), and a small SystemVerilog testbench scaffold to start simulation.
