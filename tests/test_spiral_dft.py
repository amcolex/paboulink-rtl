import math
import os
import sys
from pathlib import Path
import shutil

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, with_timeout
from cocotb_test.simulator import run

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tests.utils.ofdm import generate_quantized_qpsk_symbol

VERILATOR = shutil.which("verilator")

DATA_WIDTH = 12
NUM_COMPLEX_SAMPLES = 2048
SAMPLES_PER_CYCLE = 2
CYCLES_PER_TRANSFORM = NUM_COMPLEX_SAMPLES // SAMPLES_PER_CYCLE


def _plot_suffix() -> str:
    variant = os.getenv("SPIRAL_DFT_VARIANT")
    if not variant:
        return "spiral_dft_iterative_2048pt"
    return "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in variant)


RTL_VARIANTS = [
    ("spiral_dft_iterative_2048pt", "spiral_dft_iterative_2048pt.v"),
    ("spiral_dft_it_2048_unscaled", "spiral_dft_it_2048_unscaled.v"),
]


def _to_unsigned(value: int, width: int) -> int:
    """Convert a signed integer into width-bit two's complement."""
    mask = (1 << width) - 1
    return value & mask


def _from_unsigned(value: int, width: int) -> int:
    """Interpret an unsigned representation as a signed two's complement integer."""
    value = int(value)
    if value & (1 << (width - 1)):
        value -= 1 << width
    return value


def generate_pwm_waveform(
    num_samples: int,
    *,
    period: int = 64,
    duty_cycle: float = 0.25,
    amplitude: int = 1023,
) -> np.ndarray:
    """Return a bipolar PWM waveform sized to num_samples."""
    if not (0.0 < duty_cycle < 1.0):
        raise ValueError("duty_cycle must be between 0 and 1.")
    signed_limit = (1 << (DATA_WIDTH - 1)) - 1
    if amplitude > signed_limit or amplitude <= 0:
        raise ValueError(f"amplitude must be in the range 1..{signed_limit}.")
    period = max(1, int(period))
    high_samples = max(1, int(round(period * duty_cycle)))
    pattern = np.full(period, -amplitude, dtype=np.int16)
    pattern[:high_samples] = amplitude
    repeats = math.ceil(num_samples / period)
    waveform = np.tile(pattern, repeats)[:num_samples]
    return waveform.astype(np.int16, copy=False)


@cocotb.test()
async def spiral_dft_pwm_reference(dut):
    """Feed a PWM stimulus through the SPIRAL DFT and capture its response."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())

    dut.reset.value = 0
    dut.next.value = 0
    dut.X0.value = 0
    dut.X1.value = 0
    dut.X2.value = 0
    dut.X3.value = 0

    await RisingEdge(dut.clk)
    dut.reset.value = 1
    await RisingEdge(dut.clk)
    dut.reset.value = 0

    stimulus = generate_pwm_waveform(
        NUM_COMPLEX_SAMPLES,
        period=64,
        duty_cycle=0.25,
        amplitude=1023,
    )

    dut.next.value = 1
    await RisingEdge(dut.clk)
    dut.next.value = 0

    for cycle in range(CYCLES_PER_TRANSFORM):
        base = cycle * SAMPLES_PER_CYCLE
        sample_a = int(stimulus[base])
        sample_b = int(stimulus[base + 1])
        dut.X0.value = _to_unsigned(sample_a, DATA_WIDTH)
        dut.X1.value = 0
        dut.X2.value = _to_unsigned(sample_b, DATA_WIDTH)
        dut.X3.value = 0
        await RisingEdge(dut.clk)

    dut.X0.value = 0
    dut.X1.value = 0
    dut.X2.value = 0
    dut.X3.value = 0

    await with_timeout(RisingEdge(dut.next_out), timeout_time=200_000, timeout_unit="ns")

    outputs: list[complex] = []
    for _ in range(CYCLES_PER_TRANSFORM):
        await RisingEdge(dut.clk)
        real_0 = _from_unsigned(dut.Y0.value.integer, DATA_WIDTH)
        imag_0 = _from_unsigned(dut.Y1.value.integer, DATA_WIDTH)
        real_1 = _from_unsigned(dut.Y2.value.integer, DATA_WIDTH)
        imag_1 = _from_unsigned(dut.Y3.value.integer, DATA_WIDTH)
        outputs.append(complex(real_0, imag_0))
        outputs.append(complex(real_1, imag_1))

    assert len(outputs) == NUM_COMPLEX_SAMPLES, "Did not capture the full transform."
    magnitudes = np.abs(np.asarray(outputs, dtype=np.complex128))
    assert np.any(magnitudes > 0), "FFT outputs were all zero."

    reference = np.fft.fft(stimulus.astype(np.float64))
    reference_mag = np.abs(reference)

    reference_peak = float(np.max(reference_mag))

    reference_norm = reference_mag / reference_peak if reference_peak > 0 else reference_mag
    hardware_mag = magnitudes

    plots_dir = Path(__file__).parent / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)
    plot_path = plots_dir / f"spiral_dft_pwm_{_plot_suffix()}.png"

    time_axis = np.arange(NUM_COMPLEX_SAMPLES)
    freq_axis = np.arange(NUM_COMPLEX_SAMPLES // 2)

    fig, (ax_time, ax_freq) = plt.subplots(2, 1, figsize=(9, 6), sharex=False)
    ax_time.set_title("PWM Stimulus (1023/-1023)")
    ax_time.plot(time_axis, stimulus, linewidth=0.8, label="Input (Real)")
    ax_time.set_xlabel("Sample")
    ax_time.set_ylabel("Amplitude (LSBs)")
    ax_time.grid(True, linestyle=":", linewidth=0.5)
    ax_time.legend()

    ax_freq.set_title("FFT Magnitude")
    line_ref, = ax_freq.plot(
        freq_axis,
        reference_norm[: freq_axis.size],
        label="NumPy FFT (Normalized)",
    )
    ax_freq.set_ylabel("Magnitude (Normalized)")

    ax_freq_right = ax_freq.twinx()
    # Plot hardware magnitudes on a dedicated axis so values remain unscaled.
    line_hw, = ax_freq_right.plot(
        freq_axis,
        hardware_mag[: freq_axis.size],
        label="SPIRAL DFT (Raw)",
        linestyle="--",
    )
    ax_freq_right.set_ylabel("SPIRAL Magnitude (LSBs)")
    ax_freq.set_xlabel("FFT Bin")
    ax_freq.grid(True, linestyle=":", linewidth=0.5)

    lines = [line_ref, line_hw]
    labels = [line.get_label() for line in lines]
    ax_freq.legend(lines, labels, loc="best")

    fig.tight_layout()
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)

    dut._log.info(f"Saved FFT plot to {plot_path}")


@cocotb.test()
async def spiral_dft_ofdm_constellation(dut):
    """Drive an OFDM symbol through the SPIRAL DFT and capture time/constellation plots."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())

    dut.reset.value = 0
    dut.next.value = 0
    dut.X0.value = 0
    dut.X1.value = 0
    dut.X2.value = 0
    dut.X3.value = 0

    await RisingEdge(dut.clk)
    dut.reset.value = 1
    await RisingEdge(dut.clk)
    dut.reset.value = 0

    symbol = generate_quantized_qpsk_symbol(
        n_fft=NUM_COMPLEX_SAMPLES,
        data_width=DATA_WIDTH,
        num_subcarriers=256,
        seed=11,
        cp_len=0,
        include_cp=False,
    )

    quant_real = symbol.i_values
    quant_imag = symbol.q_values

    dut.next.value = 1
    await RisingEdge(dut.clk)
    dut.next.value = 0

    for cycle in range(CYCLES_PER_TRANSFORM):
        base = cycle * SAMPLES_PER_CYCLE
        sample_a_real = int(quant_real[base])
        sample_a_imag = int(quant_imag[base])
        sample_b_real = int(quant_real[base + 1])
        sample_b_imag = int(quant_imag[base + 1])

        dut.X0.value = _to_unsigned(sample_a_real, DATA_WIDTH)
        dut.X1.value = _to_unsigned(sample_a_imag, DATA_WIDTH)
        dut.X2.value = _to_unsigned(sample_b_real, DATA_WIDTH)
        dut.X3.value = _to_unsigned(sample_b_imag, DATA_WIDTH)
        await RisingEdge(dut.clk)

    dut.X0.value = 0
    dut.X1.value = 0
    dut.X2.value = 0
    dut.X3.value = 0

    await with_timeout(RisingEdge(dut.next_out), timeout_time=200_000, timeout_unit="ns")

    outputs: list[complex] = []
    for _ in range(CYCLES_PER_TRANSFORM):
        await RisingEdge(dut.clk)
        real_0 = _from_unsigned(dut.Y0.value.integer, DATA_WIDTH)
        imag_0 = _from_unsigned(dut.Y1.value.integer, DATA_WIDTH)
        real_1 = _from_unsigned(dut.Y2.value.integer, DATA_WIDTH)
        imag_1 = _from_unsigned(dut.Y3.value.integer, DATA_WIDTH)
        outputs.append(complex(real_0, imag_0))
        outputs.append(complex(real_1, imag_1))

    rtl_bins = np.asarray(outputs, dtype=np.complex128)
    plots_dir = Path(__file__).parent / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)

    time_axis = np.arange(NUM_COMPLEX_SAMPLES)

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    axes[0].plot(time_axis, quant_real, linewidth=0.8)
    axes[0].set_ylabel("I (LSBs)")
    axes[0].set_title("OFDM Symbol - Time Domain (Input)")
    axes[0].grid(True, linestyle=":", linewidth=0.5)

    axes[1].plot(time_axis, quant_imag, linewidth=0.8, color="tab:orange")
    axes[1].set_ylabel("Q (LSBs)")
    axes[1].set_xlabel("Sample Index")
    axes[1].grid(True, linestyle=":", linewidth=0.5)

    fig.tight_layout()
    time_plot_path = plots_dir / f"spiral_dft_ofdm_timeseries_{_plot_suffix()}.png"
    fig.savefig(time_plot_path, dpi=150)
    plt.close(fig)
    dut._log.info(f"Saved OFDM input time-series plot to {time_plot_path}")

    occupied_bins = symbol.occupied_bins
    reference_fft = symbol.reference_fft
    ref_points = reference_fft[occupied_bins]
    rtl_points = rtl_bins[occupied_bins]

    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    axes[0].scatter(ref_points.real, ref_points.imag, alpha=0.6, s=12, color="tab:blue")
    axes[0].set_title("Reference (NumPy)")
    axes[1].scatter(
        rtl_points.real,
        rtl_points.imag,
        marker="x",
        s=20,
        linewidths=0.8,
        color="tab:orange",
    )
    axes[1].set_title("SPIRAL DFT (RTL)")

    for ax in axes:
        ax.axhline(0, color="black", linewidth=0.5, linestyle=":")
        ax.axvline(0, color="black", linewidth=0.5, linestyle=":")
        ax.grid(True, linestyle=":", linewidth=0.5)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("I")
        ax.set_ylabel("Q")

    fig.suptitle("OFDM Constellation (No Additional Scaling)")
    fig.tight_layout()
    constellation_path = plots_dir / f"spiral_dft_ofdm_constellation_{_plot_suffix()}.png"
    fig.savefig(constellation_path, dpi=150)
    plt.close(fig)
    dut._log.info(f"Saved OFDM constellation plot to {constellation_path}")


@pytest.mark.skipif(VERILATOR is None, reason="Verilator executable not found; install Verilator to run this test.")
@pytest.mark.parametrize(
    ("variant_name", "rtl_filename"),
    RTL_VARIANTS,
    ids=[name for name, _ in RTL_VARIANTS],
)
def test_spiral_dft_pwm(variant_name: str, rtl_filename: str):
    rtl_dir = Path(__file__).resolve().parent.parent / "rtl"
    rtl_file = rtl_dir / rtl_filename
    if not rtl_file.exists() or rtl_file.stat().st_size == 0:
        pytest.fail(f"RTL variant {variant_name} not available at {rtl_file}.")
    build_dir = Path("tests") / "sim_build" / f"spiral_dft_pwm_{variant_name}"

    run(
        verilog_sources=[str(rtl_file)],
        toplevel="dft_top",
        toplevel_lang="verilog",
        module=os.path.splitext(os.path.basename(__file__))[0],
        sim_build=str(build_dir),
        simulator="verilator",
        verilog_compile_args=["--Wno-WIDTHEXPAND"],
        extra_env={
            "COCOTB_RESULTS_FILE": str(build_dir / "results.xml"),
            "SPIRAL_DFT_VARIANT": variant_name,
        },
    )
