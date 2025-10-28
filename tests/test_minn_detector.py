
import math
import os
import shutil
from pathlib import Path

import numpy as np
import pytest
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge
from cocotb_test.simulator import run

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

VERILATOR = shutil.which("verilator")

WIDTH = 12
N_FFT = 2048
Q = N_FFT // 4
# Gate threshold for the Minn metric; increased to avoid small side-peak triggers.
THRESHOLD = 0.25
K_Q15 = int(round(math.sqrt(THRESHOLD) * (1 << 15)))
CP_LEN = N_FFT // 16
OFDM_SYMBOL_LEN = N_FFT + CP_LEN
NUM_DATA_SYMBOLS = 2
NUM_FRAMES = 2
FRAME_GAP_SYMBOLS = 3
FLUSH_LEN = 4 * Q
WINDOW_DELAY = 4 * Q - 1
PLOT_DIR = Path(__file__).parent / "plots"
PLOT_DIR.mkdir(parents=True, exist_ok=True)


def _build_baseband_sequence(seed: int = 0x5A17) -> np.ndarray:
    rng = np.random.default_rng(seed)
    prefix = _complex_noise(rng, OFDM_SYMBOL_LEN, scale=0.05)
    frames = []
    for frame_idx in range(NUM_FRAMES):
        frames.append(_build_frame_samples(rng))
        if frame_idx != NUM_FRAMES - 1:
            guard_len = FRAME_GAP_SYMBOLS * OFDM_SYMBOL_LEN
            frames.append(_complex_noise(rng, guard_len, scale=0.05))
    suffix = _complex_noise(rng, OFDM_SYMBOL_LEN, scale=0.05)
    return np.concatenate([prefix, *frames, suffix], axis=0)


def _complex_noise(rng: np.random.Generator, length: int, scale: float) -> np.ndarray:
    return (
        rng.normal(scale=scale, size=(length, 2)) +
        1j * rng.normal(scale=scale, size=(length, 2))
    )


def _create_preamble_symbol(rng: np.random.Generator) -> np.ndarray:
    seg_a = rng.normal(scale=0.7, size=Q) + 1j * rng.normal(scale=0.7, size=Q)
    seg_b = rng.normal(scale=0.65, size=Q) + 1j * rng.normal(scale=0.65, size=Q)
    return np.concatenate([seg_a, seg_a, seg_b, seg_b], axis=0)


def _generate_qpsk_symbols(rng: np.random.Generator, count: int) -> np.ndarray:
    constellation = np.array([1 + 1j, 1 - 1j, -1 + 1j, -1 - 1j]) / math.sqrt(2.0)
    symbols = constellation[rng.integers(0, 4, size=(count, N_FFT))]
    time_domain = np.fft.ifft(symbols, axis=1) * math.sqrt(N_FFT)
    return time_domain


def _add_cyclic_prefix(symbol: np.ndarray) -> np.ndarray:
    cp = symbol[-CP_LEN:]
    return np.concatenate([cp, symbol], axis=0)


def _build_frame_samples(rng: np.random.Generator) -> np.ndarray:
    preamble_td = _create_preamble_symbol(rng)
    data_symbols_td = _generate_qpsk_symbols(rng, NUM_DATA_SYMBOLS)
    symbols_with_cp = []
    phase_shifts = np.exp(1j * rng.uniform(0.0, 2.0 * math.pi, size=2))
    for symbol_td in [preamble_td, *data_symbols_td]:
        symbol_with_cp = _add_cyclic_prefix(symbol_td)
        channel_samples = np.column_stack([
            symbol_with_cp * phase_shifts[0],
            symbol_with_cp * phase_shifts[1],
        ])
        symbols_with_cp.append(channel_samples)
    return np.concatenate(symbols_with_cp, axis=0)


def _quantize_samples(samples: np.ndarray) -> np.ndarray:
    scale = (1 << (WIDTH - 1)) - 1
    max_mag = np.max(np.abs(samples))
    # If the floating-point stimulus exceeds +/-1.0, normalize so we use the full 12-bit range
    # without hard clipping. Otherwise leave it untouched.
    norm_denom = max(max_mag, 1.0)
    scaled_real = np.clip(samples.real / norm_denom, -0.999, 0.999)
    scaled_imag = np.clip(samples.imag / norm_denom, -0.999, 0.999)
    i_vals = np.round(scaled_real * scale).astype(int)
    q_vals = np.round(scaled_imag * scale).astype(int)
    return np.stack([i_vals, q_vals], axis=-1)


def _to_complex(int_samples: np.ndarray) -> np.ndarray:
    return int_samples[..., 0].astype(np.float64) + 1j * int_samples[..., 1].astype(np.float64)


def _compute_minn_metric(int_samples: np.ndarray) -> np.ndarray:
    r_vals, energy_vals = _compute_minn_components(int_samples)
    metrics = np.zeros_like(r_vals, dtype=np.float64)
    valid_len = r_vals.shape[0] - (4 * Q - 1)
    valid_len = max(valid_len, 0)
    if valid_len > 0:
        energy_slice = energy_vals[:valid_len]
        ratio = np.divide(r_vals[:valid_len], energy_slice, out=np.zeros_like(energy_slice), where=energy_slice != 0)
        metrics[:valid_len] = ratio ** 2
    return metrics


def _compute_minn_components(int_samples: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    complex_samples = _to_complex(int_samples)
    total_len = complex_samples.shape[0]
    r_vals = np.zeros(total_len, dtype=np.float64)
    energy_vals = np.zeros(total_len, dtype=np.float64)
    for start in range(0, total_len - 4 * Q + 1):
        seg0 = complex_samples[start:start + Q]
        seg1 = complex_samples[start + Q:start + 2 * Q]
        seg2 = complex_samples[start + 2 * Q:start + 3 * Q]
        seg3 = complex_samples[start + 3 * Q:start + 4 * Q]
        c1 = np.sum(seg0 * np.conj(seg1))
        c2 = np.sum(seg2 * np.conj(seg3))
        corr = c1 + c2
        r_val = max(0.0, corr.real)
        energy = np.sum(np.abs(seg1) ** 2 + np.abs(seg2) ** 2 + np.abs(seg3) ** 2)
        r_vals[start] = r_val
        energy_vals[start] = energy
    return r_vals, energy_vals


def _find_gate_peaks(metrics: np.ndarray, threshold: float) -> list[int]:
    gate = False
    peak_value = -1.0
    peak_idx = -1
    peaks: list[int] = []
    holdoff_counter = 0
    detection_armed = True
    for idx, value in enumerate(metrics):
        if holdoff_counter > 0:
            holdoff_counter -= 1
            if holdoff_counter == 0:
                detection_armed = True
        if value >= threshold and detection_armed:
            if not gate:
                gate = True
                peak_value = value
                peak_idx = idx
            elif value >= peak_value:
                peak_value = value
                peak_idx = idx
        else:
            if gate:
                peaks.append(peak_idx)
                gate = False
                peak_value = -1.0
                peak_idx = -1
                detection_armed = False
                holdoff_counter = N_FFT
    if gate:
        peaks.append(peak_idx)
    return peaks


def _prepare_stimulus() -> dict:
    payload_complex = _build_baseband_sequence()
    payload_ints = _quantize_samples(payload_complex)
    metrics = _compute_minn_metric(payload_ints)
    peak_indices = _find_gate_peaks(metrics, THRESHOLD)
    if not peak_indices:
        raise RuntimeError("Minn metric never crossed the threshold in the prepared stimulus")
    flush = np.zeros((FLUSH_LEN, 2, 2), dtype=int)
    stimulus_ints = np.concatenate([payload_ints, flush], axis=0)
    return {
        "payload_complex": payload_complex,
        "payload_ints": payload_ints,
        "stimulus_ints": stimulus_ints,
        "metrics": metrics,
        "peak_indices": peak_indices,
    }


@cocotb.test()
async def minn_detector_finds_preamble(dut):
    data = _prepare_stimulus()
    payload_ints = data["payload_ints"]
    stimulus_ints = data["stimulus_ints"]
    metrics = data["metrics"]
    expected_peaks = data["peak_indices"]
    payload_len = payload_ints.shape[0]
    py_r_vals, py_energy_vals = _compute_minn_components(payload_ints)

    clock = Clock(dut.clk, 1, units="ns")
    cocotb.start_soon(clock.start())

    dut.rst_n.value = 0
    dut.in_valid.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    out_index = 0
    observed_flags = []
    output_flags = []
    out_ch0_i_vals = []
    out_ch0_q_vals = []
    out_ch1_i_vals = []
    out_ch1_q_vals = []
    rtl_metric_r_vals = []
    rtl_metric_energy_vals = []

    def _consume_output():
        nonlocal out_index
        if dut.out_valid.value:
            out_i0 = dut.out_ch0_i.value.signed_integer
            out_q0 = dut.out_ch0_q.value.signed_integer
            out_i1 = dut.out_ch1_i.value.signed_integer
            out_q1 = dut.out_ch1_q.value.signed_integer
            out_ch0_i_vals.append(out_i0)
            out_ch0_q_vals.append(out_q0)
            out_ch1_i_vals.append(out_i1)
            out_ch1_q_vals.append(out_q1)
            if out_index < payload_len:
                exp = payload_ints[out_index]
                assert out_i0 == int(exp[0, 0])
                assert out_q0 == int(exp[0, 1])
                assert out_i1 == int(exp[1, 0])
                assert out_q1 == int(exp[1, 1])
            flag_val = int(dut.detect_flag.value)
            output_flags.append(flag_val)
            if flag_val:
                observed_flags.append(out_index)
            out_index += 1
        if dut.dbg_metric_valid.value:
            rtl_metric_r_vals.append(int(dut.dbg_metric_r.value))
            rtl_metric_energy_vals.append(int(dut.dbg_metric_energy.value))

    for sample in stimulus_ints:
        ch0_i, ch0_q = int(sample[0, 0]), int(sample[0, 1])
        ch1_i, ch1_q = int(sample[1, 0]), int(sample[1, 1])

        dut.ch0_i.value = ch0_i
        dut.ch0_q.value = ch0_q
        dut.ch1_i.value = ch1_i
        dut.ch1_q.value = ch1_q
        dut.in_valid.value = 1

        await RisingEdge(dut.clk)

        _consume_output()

        dut.in_valid.value = 0

    drain_cycles = 0
    max_drain_cycles = FLUSH_LEN + WINDOW_DELAY
    while out_index < payload_len and drain_cycles < max_drain_cycles:
        await RisingEdge(dut.clk)
        _consume_output()
        drain_cycles += 1

    dut._log.info("Minn detector produced %d samples, expected %d", out_index, payload_len)
    dut._log.info("Detected flag indices: %s (expected %s)", observed_flags, expected_peaks)

    assert out_index >= payload_len, "Output stream did not cover the entire payload"

    assert len(observed_flags) == len(expected_peaks), (
        f"Detector fired {len(observed_flags)} times, expected {len(expected_peaks)}"
    )
    for peak_expected, peak_observed in zip(expected_peaks, observed_flags):
        assert peak_observed == peak_expected, (
            f"Detector flagged index {peak_observed}, expected {peak_expected}"
        )

    assert len(rtl_metric_r_vals) == len(rtl_metric_energy_vals), "RTL metric lists length mismatch"
    valid_metric_span = max(0, payload_len - 4 * Q + 1)
    assert len(rtl_metric_r_vals) >= valid_metric_span, "RTL metric stream shorter than expected span"
    rtl_metric_r_array = np.array(rtl_metric_r_vals, dtype=np.float64)
    rtl_metric_energy_array = np.array(rtl_metric_energy_vals, dtype=np.float64)
    py_r_series = py_r_vals[:payload_len]
    py_energy_series = py_energy_vals[:payload_len]
    np.testing.assert_allclose(
        rtl_metric_r_array[:valid_metric_span],
        py_r_series[:valid_metric_span],
        rtol=1e-6,
        atol=1e-6,
    )
    np.testing.assert_allclose(
        rtl_metric_energy_array[:valid_metric_span],
        py_energy_series[:valid_metric_span],
        rtol=1e-6,
        atol=1e-6,
    )
    rtl_metric_series = np.zeros(payload_len, dtype=np.float64)
    rtl_ratio_series = np.zeros(payload_len, dtype=np.float64)
    py_ratio_series = np.zeros(payload_len, dtype=np.float64)
    if valid_metric_span > 0:
        energy_slice_py = py_energy_series[:valid_metric_span]
        energy_slice_rtl = rtl_metric_energy_array[:valid_metric_span]
        ratio_py = np.divide(
            py_r_series[:valid_metric_span],
            energy_slice_py,
            out=np.zeros_like(energy_slice_py),
            where=energy_slice_py != 0,
        )
        ratio_rtl = np.divide(
            rtl_metric_r_array[:valid_metric_span],
            energy_slice_rtl,
            out=np.zeros_like(energy_slice_rtl),
            where=energy_slice_rtl != 0,
        )
        py_ratio_series[:valid_metric_span] = ratio_py
        rtl_ratio_series[:valid_metric_span] = ratio_rtl
        rtl_metric_series[:valid_metric_span] = ratio_rtl ** 2
    np.testing.assert_allclose(
        rtl_metric_series,
        metrics[:payload_len],
        rtol=1e-6,
        atol=1e-6,
    )

    # Generate diagnostic plot comparing metric, threshold, and detected flag trace.
    indices = np.arange(payload_len)
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    payload_complex = data["payload_complex"]
    axes[0].plot(indices, np.abs(payload_complex[:payload_len, 0]), label="|ch0|")
    axes[0].plot(indices, np.abs(payload_complex[:payload_len, 1]), label="|ch1|")
    axes[0].set_ylabel("Magnitude")
    axes[0].legend(loc="upper right")
    axes[0].set_title("Minn detector input magnitudes")

    axes[1].plot(indices, metrics[:payload_len], label="Minn metric (Python)")
    axes[1].plot(indices, rtl_metric_series, linestyle="--", label="Minn metric (RTL)")
    axes[1].axhline(THRESHOLD, color="red", linestyle="--", label="Threshold")
    for idx_peak, expected_peak in enumerate(expected_peaks):
        label = "Expected peak" if idx_peak == 0 else None
        axes[1].axvline(expected_peak, color="green", linestyle=":", label=label)
    for idx_flag, observed_peak in enumerate(observed_flags):
        label = "RTL peak" if idx_flag == 0 else None
        axes[1].axvline(observed_peak, color="black", linestyle="-.", label=label)
    axes[1].set_ylabel("Metric")
    axes[1].legend(loc="upper right")
    axes[1].set_title("Minn metric comparison (Python vs RTL)")

    flag_trace = np.zeros(payload_len)
    count = min(len(output_flags), payload_len)
    flag_trace[:count] = output_flags[:count]
    axes[2].step(indices, flag_trace, where="post", label="detect_flag")
    axes[2].set_ylabel("Flag")
    axes[2].set_xlabel("Sample index")
    axes[2].legend(loc="upper right")

    plt.tight_layout()
    plot_path = PLOT_DIR / "minn_detector_metric.png"
    fig.savefig(plot_path)
    plt.close(fig)

    if valid_metric_span > 0:
        component_indices = np.arange(valid_metric_span)
        fig_cmp, axes_cmp = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

        axes_cmp[0].plot(component_indices, py_r_series[:valid_metric_span], label="R numerator (Python)")
        axes_cmp[0].plot(
            component_indices,
            rtl_metric_r_array[:valid_metric_span],
            linestyle="--",
            label="R numerator (RTL)",
        )
        axes_cmp[0].set_ylabel("R value")
        axes_cmp[0].legend(loc="upper right")
        axes_cmp[0].set_title("Minn metric numerator comparison")

        axes_cmp[1].plot(component_indices, py_energy_series[:valid_metric_span], label="Energy (Python)")
        axes_cmp[1].plot(
            component_indices,
            rtl_metric_energy_array[:valid_metric_span],
            linestyle="--",
            label="Energy (RTL)",
        )
        axes_cmp[1].set_ylabel("Energy")
        axes_cmp[1].legend(loc="upper right")
        axes_cmp[1].set_title("Minn metric energy window comparison")

        axes_cmp[2].plot(component_indices, py_ratio_series[:valid_metric_span], label="R/E (Python)")
        axes_cmp[2].plot(
            component_indices,
            rtl_ratio_series[:valid_metric_span],
            linestyle="--",
            label="R/E (RTL)",
        )
        axes_cmp[2].axhline(math.sqrt(THRESHOLD), color="red", linestyle="--", label="sqrt(threshold)")
        axes_cmp[2].set_ylabel("R/E")
        axes_cmp[2].set_xlabel("Metric sample index")
        axes_cmp[2].legend(loc="upper right")
        axes_cmp[2].set_title("Minn metric ratio comparison")

        plt.tight_layout()
        plot_path_components = PLOT_DIR / "minn_detector_metric_components.png"
        fig_cmp.savefig(plot_path_components)
        plt.close(fig_cmp)

    # Plot the delayed detector outputs with the detection flag overlay.
    out_sample_count = len(out_ch0_i_vals)
    if out_sample_count:
        indices_out = np.arange(out_sample_count)
        flag_trace_out = np.zeros(out_sample_count)
        flag_trace_out[:len(output_flags)] = output_flags
        metrics_out = metrics[:out_sample_count]
        rtl_metrics_out = rtl_metric_series[:out_sample_count]

        fig_out, axes_out = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

        axes_out[0].plot(indices_out, out_ch0_i_vals, label="ch0 I")
        axes_out[0].plot(indices_out, out_ch0_q_vals, label="ch0 Q")
        for idx_flag, flag_idx in enumerate(observed_flags):
            if flag_idx < out_sample_count:
                label = "detect_flag pulse" if idx_flag == 0 else None
                axes_out[0].axvline(flag_idx, color="black", linestyle="-.", label=label)
        axes_out[0].set_ylabel("Channel 0")
        axes_out[0].legend(loc="upper right")
        axes_out[0].set_title("RTL Minn detector output samples")

        axes_out[1].plot(indices_out, out_ch1_i_vals, label="ch1 I")
        axes_out[1].plot(indices_out, out_ch1_q_vals, label="ch1 Q")
        for idx_flag, flag_idx in enumerate(observed_flags):
            if flag_idx < out_sample_count:
                label = "detect_flag pulse" if idx_flag == 0 else None
                axes_out[1].axvline(flag_idx, color="black", linestyle="-.", label=label)
        axes_out[1].set_ylabel("Channel 1")
        axes_out[1].legend(loc="upper right")

        axes_out[2].plot(indices_out, metrics_out, label="Minn metric (Python)")
        axes_out[2].plot(indices_out, rtl_metrics_out, linestyle="--", label="Minn metric (RTL)")
        axes_out[2].axhline(THRESHOLD, color="red", linestyle="--", label="Threshold")
        for idx_peak, expected_peak in enumerate(expected_peaks):
            if expected_peak < out_sample_count:
                label = "Expected peak" if idx_peak == 0 else None
                axes_out[2].axvline(expected_peak, color="green", linestyle=":", label=label)
        for idx_flag, observed_peak in enumerate(observed_flags):
            if observed_peak < out_sample_count:
                label = "RTL peak" if idx_flag == 0 else None
                axes_out[2].axvline(observed_peak, color="black", linestyle="-.", label=label)
        axes_out[2].set_ylabel("Metric")
        axes_out[2].legend(loc="upper right")
        axes_out[2].set_title("Minn metric comparison (RTL vs Python)")

        axes_out[3].step(indices_out, flag_trace_out, where="post", label="detect_flag", alpha=0.8)
        axes_out[3].set_ylabel("Flag")
        axes_out[3].set_xlabel("Output sample index")
        axes_out[3].legend(loc="upper right")

        plt.tight_layout()
        plot_path_out = PLOT_DIR / "minn_detector_output_stream.png"
        fig_out.savefig(plot_path_out)
        plt.close(fig_out)


@pytest.mark.skipif(VERILATOR is None, reason="Verilator executable not found; install Verilator to run this test.")
def test_minn_detector():
    rtl_dir = Path(__file__).resolve().parent.parent / "rtl"
    rtl_file = rtl_dir / "minn_frame_detector.sv"
    build_dir = Path("tests") / "sim_build" / "minn_detector"

    run(
        verilog_sources=[str(rtl_file)],
        toplevel="minn_frame_detector",
        toplevel_lang="verilog",
        module=os.path.splitext(os.path.basename(__file__))[0],
        parameters={
            "WIDTH": WIDTH,
            "N_FFT": N_FFT,
            "K_Q15": K_Q15,
        },
        sim_build=str(build_dir),
        simulator="verilator",
        verilog_compile_args=["-sv"],
        extra_env={
            "COCOTB_RESULTS_FILE": str(build_dir / "results.xml"),
        },
    )
