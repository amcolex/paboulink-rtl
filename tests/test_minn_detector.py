
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
THRESHOLD = 0.2
K_Q15 = int(round(math.sqrt(THRESHOLD) * (1 << 15)))
FLUSH_LEN = 4 * Q
WINDOW_DELAY = 4 * Q - 1
PLOT_DIR = Path(__file__).parent / "plots"
PLOT_DIR.mkdir(parents=True, exist_ok=True)


def _build_baseband_sequence(seed: int = 0x5A17) -> np.ndarray:
    rng = np.random.default_rng(seed)
    prefix_len = 320
    suffix_len = 320
    prefix = (rng.normal(scale=0.05, size=(prefix_len, 2)) +
              1j * rng.normal(scale=0.05, size=(prefix_len, 2)))
    seg_a = (rng.normal(scale=0.7, size=(Q, 2)) +
             1j * rng.normal(scale=0.7, size=(Q, 2)))
    seg_b = (rng.normal(scale=0.65, size=(Q, 2)) +
             1j * rng.normal(scale=0.65, size=(Q, 2)))
    preamble = np.concatenate([seg_a, seg_a, seg_b, seg_b], axis=0)
    suffix = (rng.normal(scale=0.05, size=(suffix_len, 2)) +
              1j * rng.normal(scale=0.05, size=(suffix_len, 2)))
    return np.concatenate([prefix, preamble, suffix], axis=0)


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
    complex_samples = _to_complex(int_samples)
    total_len = complex_samples.shape[0]
    metrics = np.zeros(total_len, dtype=np.float64)
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
        if energy > 0.0:
            metric = (r_val / energy) ** 2
        else:
            metric = 0.0
        metrics[start] = metric
    return metrics


def _find_gate_peak(metrics: np.ndarray, threshold: float) -> int:
    gate = False
    peak_value = -1.0
    peak_idx = -1
    for idx, value in enumerate(metrics):
        if value >= threshold:
            if not gate:
                gate = True
                peak_value = value
                peak_idx = idx
            elif value >= peak_value:
                peak_value = value
                peak_idx = idx
        else:
            if gate:
                break
    return peak_idx


def _prepare_stimulus() -> dict:
    payload_complex = _build_baseband_sequence()
    payload_ints = _quantize_samples(payload_complex)
    metrics = _compute_minn_metric(payload_ints)
    peak_idx = _find_gate_peak(metrics, THRESHOLD)
    if peak_idx < 0:
        raise RuntimeError("Minn metric never crossed the threshold in the prepared stimulus")
    flush = np.zeros((FLUSH_LEN, 2, 2), dtype=int)
    stimulus_ints = np.concatenate([payload_ints, flush], axis=0)
    return {
        "payload_complex": payload_complex,
        "payload_ints": payload_ints,
        "stimulus_ints": stimulus_ints,
        "metrics": metrics,
        "peak_idx": peak_idx,
    }


@cocotb.test()
async def minn_detector_finds_preamble(dut):
    data = _prepare_stimulus()
    payload_ints = data["payload_ints"]
    stimulus_ints = data["stimulus_ints"]
    metrics = data["metrics"]
    expected_peak_idx = data["peak_idx"]
    payload_len = payload_ints.shape[0]

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
    dut._log.info("Detected flag indices: %s", observed_flags)

    assert out_index >= payload_len, "Output stream did not cover the entire payload"

    assert len(observed_flags) == 1, "Detector should fire exactly once"
    assert observed_flags[0] == expected_peak_idx, (
        f"Detector flagged index {observed_flags[0]}, expected {expected_peak_idx}"
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

    axes[1].plot(indices, metrics[:payload_len], label="Minn metric")
    axes[1].axhline(THRESHOLD, color="red", linestyle="--", label="Threshold")
    axes[1].axvline(expected_peak_idx, color="green", linestyle=":", label="Expected peak")
    axes[1].axvline(observed_flags[0], color="black", linestyle="-.", label="RTL peak")
    axes[1].set_ylabel("Metric")
    axes[1].legend(loc="upper right")

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

    # Plot the delayed detector outputs with the detection flag overlay.
    out_sample_count = len(out_ch0_i_vals)
    if out_sample_count:
        indices_out = np.arange(out_sample_count)
        flag_trace_out = np.zeros(out_sample_count)
        flag_trace_out[:len(output_flags)] = output_flags

        fig_out, axes_out = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

        axes_out[0].plot(indices_out, out_ch0_i_vals, label="ch0 I")
        axes_out[0].plot(indices_out, out_ch0_q_vals, label="ch0 Q")
        if observed_flags:
            axes_out[0].axvline(observed_flags[0], color="black", linestyle="-.", label="detect_flag pulse")
        axes_out[0].set_ylabel("Channel 0")
        axes_out[0].legend(loc="upper right")
        axes_out[0].set_title("Minn detector output samples")

        axes_out[1].plot(indices_out, out_ch1_i_vals, label="ch1 I")
        axes_out[1].plot(indices_out, out_ch1_q_vals, label="ch1 Q")
        if observed_flags:
            axes_out[1].axvline(observed_flags[0], color="black", linestyle="-.", label="detect_flag pulse")
        axes_out[1].set_ylabel("Channel 1")
        axes_out[1].legend(loc="upper right")

        axes_out[2].step(indices_out, flag_trace_out, where="post", label="detect_flag")
        axes_out[2].set_ylabel("Flag")
        axes_out[2].set_xlabel("Output sample index")
        axes_out[2].legend(loc="upper right")

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
