import shutil
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge
from cocotb_test.simulator import run

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tests.utils.ofdm import generate_quantized_qpsk_symbol
from tests.utils.plotting import module_plot_dir

VERILATOR = shutil.which("verilator")

NFFT = 2048
DATA_WIDTH = 12
GAP_CYCLES = 11287
SIGNED_LIMIT = (1 << (DATA_WIDTH - 1)) - 1
PLOTS_DIR = module_plot_dir(__file__)


def _to_unsigned(value: int, width: int) -> int:
    mask = (1 << width) - 1
    return value & mask


def _from_unsigned(value: int, width: int) -> int:
    if value & (1 << (width - 1)):
        value -= 1 << width
    return value


def pack_samples(ant0_real: int, ant0_imag: int, ant1_real: int, ant1_imag: int) -> int:
    """Pack two complex samples into the AXI4-Stream word format."""
    mask = (1 << DATA_WIDTH) - 1
    return (
        (_to_unsigned(ant1_imag, DATA_WIDTH) << 36)
        | (_to_unsigned(ant1_real, DATA_WIDTH) << 24)
        | (_to_unsigned(ant0_imag, DATA_WIDTH) << 12)
        | _to_unsigned(ant0_real, DATA_WIDTH)
    )


def generate_pwm_waveform(
    num_samples: int,
    *,
    period: int = 64,
    duty_cycle: float = 0.25,
    amplitude: int = SIGNED_LIMIT,
) -> np.ndarray:
    num_samples = int(num_samples)
    if num_samples <= 0:
        return np.zeros(0, dtype=np.int16)
    period = max(1, int(period))
    high_samples = max(1, int(round(period * duty_cycle)))
    amplitude = max(0, min(int(amplitude), SIGNED_LIMIT))
    pattern = np.full(period, -amplitude, dtype=np.int16)
    pattern[:high_samples] = amplitude
    repeats = int(np.ceil(num_samples / period))
    waveform = np.tile(pattern, repeats)[:num_samples]
    return waveform.astype(np.int16, copy=False)


async def initialize_fft_streaming_wrapper(dut) -> None:
    axis_clock = Clock(dut.clk_axis, 10, units="ns")
    fft_clock = Clock(dut.clk_fft, 4, units="ns")
    cocotb.start_soon(axis_clock.start())
    cocotb.start_soon(fft_clock.start())

    dut.rst_axis_n.value = 0
    dut.rst_fft_n.value = 0
    dut.s_axis_tdata.value = 0
    dut.s_axis_tuser.value = 0
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tlast.value = 0
    dut.m_axis_tready.value = 1

    for _ in range(6):
        await RisingEdge(dut.clk_fft)
    dut.rst_fft_n.value = 1

    for _ in range(6):
        await RisingEdge(dut.clk_axis)
    dut.rst_axis_n.value = 1

    for _ in range(6):
        await RisingEdge(dut.clk_axis)


async def stream_symbol(
    dut,
    ant0_real: np.ndarray,
    *,
    ant0_imag: np.ndarray | None = None,
) -> None:
    ant0_real = np.asarray(ant0_real, dtype=np.int64)
    if ant0_imag is None:
        ant0_imag = np.zeros_like(ant0_real)
    else:
        ant0_imag = np.asarray(ant0_imag, dtype=np.int64)

    for sample_idx in range(ant0_real.size):
        packed = pack_samples(
            int(ant0_real[sample_idx]),
            int(ant0_imag[sample_idx]),
            0,
            0,
        )

        dut.s_axis_tdata.value = packed
        dut.s_axis_tuser.value = 0
        dut.s_axis_tvalid.value = 1

        while True:
            await RisingEdge(dut.clk_axis)
            if dut.s_axis_tready.value:
                break

        dut.s_axis_tvalid.value = 0

    dut.s_axis_tdata.value = 0
    dut.s_axis_tuser.value = 0
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tlast.value = 0


async def capture_fft_bins(dut, expected_bins: int = NFFT) -> np.ndarray:
    outputs: list[complex] = []
    max_cycles = expected_bins + GAP_CYCLES + 4096

    for _ in range(max_cycles):
        await RisingEdge(dut.clk_axis)
        if dut.m_axis_tvalid.value:
            data_word = int(dut.m_axis_tdata.value)
            ant0_real = _from_unsigned(data_word & ((1 << DATA_WIDTH) - 1), DATA_WIDTH)
            ant0_imag = _from_unsigned((data_word >> 12) & ((1 << DATA_WIDTH) - 1), DATA_WIDTH)
            outputs.append(complex(ant0_real, ant0_imag))
            if len(outputs) >= expected_bins:
                break

    if len(outputs) < expected_bins:
        dut._log.warning(
            "Captured %d FFT bins (expected %d)",
            len(outputs),
            expected_bins,
        )

    return np.asarray(outputs, dtype=np.complex128)


def plot_time_and_fft(
    time_real: np.ndarray,
    time_imag: np.ndarray | None,
    fft_bins: np.ndarray,
    *,
    title_prefix: str,
    filename: str,
) -> Path:
    plots_dir = PLOTS_DIR
    time_real = np.asarray(time_real, dtype=np.int64)
    time_axis = np.arange(time_real.size)

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=False)

    axes[0].plot(time_axis, time_real, label="I", linewidth=0.8)
    if time_imag is not None:
        time_imag = np.asarray(time_imag, dtype=np.int64)
        axes[0].plot(time_axis, time_imag, label="Q", linewidth=0.8)
    axes[0].set_title(f"{title_prefix} Time Samples (Raw)")
    axes[0].set_ylabel("Amplitude (LSBs)")
    axes[0].set_xlabel("Sample Index")
    axes[0].grid(True, linestyle=":", linewidth=0.5)
    if time_imag is not None:
        axes[0].legend(loc="best")

    fft_bins = np.asarray(fft_bins, dtype=np.complex128)
    if fft_bins.size:
        freq_axis = np.arange(fft_bins.size)
        axes[1].plot(freq_axis, fft_bins.real, label="Real", linewidth=0.8)
        axes[1].plot(freq_axis, fft_bins.imag, label="Imag", linewidth=0.8)
        axes[1].legend(loc="best")
    else:
        axes[1].text(
            0.5,
            0.5,
            "No FFT data captured",
            ha="center",
            va="center",
            transform=axes[1].transAxes,
        )
    axes[1].set_title(f"{title_prefix} FFT Output (Raw)")
    axes[1].set_xlabel("FFT Bin")
    axes[1].set_ylabel("Amplitude (LSBs)")
    axes[1].grid(True, linestyle=":", linewidth=0.5)

    fig.tight_layout()
    plot_path = plots_dir / filename
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    return plot_path


@cocotb.test()
async def fft_streaming_ofdm_plot(dut):
    await initialize_fft_streaming_wrapper(dut)

    ofdm_symbol = generate_quantized_qpsk_symbol(
        n_fft=NFFT,
        data_width=DATA_WIDTH,
        num_subcarriers=256,
        seed=7,
        cp_len=0,
        include_cp=False,
    )

    await stream_symbol(
        dut,
        ofdm_symbol.i_values,
        ant0_imag=ofdm_symbol.q_values,
    )

    rtl_bins = await capture_fft_bins(dut)

    plot_path = plot_time_and_fft(
        ofdm_symbol.i_values,
        ofdm_symbol.q_values,
        rtl_bins,
        title_prefix="OFDM",
        filename="fft_streaming_wrapper_ofdm.png",
    )
    dut._log.info("Saved OFDM plot to %s", plot_path)


@cocotb.test()
async def fft_streaming_pwm_plot(dut):
    await initialize_fft_streaming_wrapper(dut)

    pwm_waveform = generate_pwm_waveform(
        NFFT,
        period=64,
        duty_cycle=0.25,
        amplitude=1024,
    )

    await stream_symbol(dut, pwm_waveform)

    rtl_bins = await capture_fft_bins(dut)

    plot_path = plot_time_and_fft(
        pwm_waveform,
        None,
        rtl_bins,
        title_prefix="PWM",
        filename="fft_streaming_wrapper_pwm.png",
    )
    dut._log.info("Saved PWM plot to %s", plot_path)


@pytest.mark.skipif(VERILATOR is None, reason="Verilator is required to run this test")
def test_fft_streaming_wrapper(tmp_path: Path):
    tests_dir = Path(__file__).parent
    rtl_dir = tests_dir.parent / "rtl"

    run(
        verilog_sources=[
            str(rtl_dir / "fft_streaming_wrapper.v"),
            str(rtl_dir / "dual_clock_ram.v"),
            str(rtl_dir / "dual_clock_fifo.v"),
            str(rtl_dir / "spiral_dft_it_2048_16bit_scaled.v"),
        ],
        toplevel="fft_streaming_wrapper",
        toplevel_lang="verilog",
        module="test_fft_streaming_wrapper",
        sim_build=str(tmp_path / "sim_build_fft_streaming_wrapper"),
        simulator="verilator",
        compile_args=["-Wno-WIDTHEXPAND"],
        waves=False,
    )
