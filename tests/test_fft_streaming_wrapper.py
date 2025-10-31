import shutil
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

VERILATOR = shutil.which("verilator")

NFFT = 2048
DATA_WIDTH = 12
GAP_CYCLES = 11287


def _to_unsigned(value: int, width: int) -> int:
    mask = (1 << width) - 1
    return value & mask


def _from_unsigned(value: int, width: int) -> int:
    if value & (1 << (width - 1)):
        value -= 1 << width
    return value


def generate_symbol(tone_specs: list[tuple[int, float]]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return quantized I/Q arrays and complex view for the requested tones."""
    n = np.arange(NFFT, dtype=np.float64)
    waveform = np.zeros(NFFT, dtype=np.complex128)
    for bin_idx, amplitude in tone_specs:
        waveform += amplitude * np.exp(2j * np.pi * bin_idx * n / NFFT)

    real = np.clip(np.round(np.real(waveform)), -2048, 2047).astype(np.int16)
    imag = np.clip(np.round(np.imag(waveform)), -2048, 2047).astype(np.int16)
    complex_view = real.astype(np.float64) + 1j * imag.astype(np.float64)
    return real, imag, complex_view


def pack_samples(ant0_real: int, ant0_imag: int, ant1_real: int, ant1_imag: int) -> int:
    """Pack two complex samples into the AXI4-Stream word format."""
    mask = (1 << DATA_WIDTH) - 1
    return (
        (_to_unsigned(ant1_imag, DATA_WIDTH) << 36)
        | (_to_unsigned(ant1_real, DATA_WIDTH) << 24)
        | (_to_unsigned(ant0_imag, DATA_WIDTH) << 12)
        | _to_unsigned(ant0_real, DATA_WIDTH)
    )


@cocotb.test()
async def fft_streaming_functional(dut):
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

    symbol_tones = [
        {
            "ant0": [(32, 900.0)],
            "ant1": [(8, 650.0), (96, 420.0)],
        },
        {
            "ant0": [(128, 780.0), (256, 360.0)],
            "ant1": [(20, 840.0)],
        },
    ]

    stimulus_symbols: list[dict[str, np.ndarray]] = []

    for symbol_idx, tone_spec in enumerate(symbol_tones):
        ant0_real, ant0_imag, ant0_complex = generate_symbol(tone_spec["ant0"])
        ant1_real, ant1_imag, ant1_complex = generate_symbol(tone_spec["ant1"])

        stimulus_symbols.append(
            {
                "ant0": ant0_complex,
                "ant1": ant1_complex,
            }
        )

        for sample_idx in range(NFFT):
            packed = pack_samples(
                int(ant0_real[sample_idx]),
                int(ant0_imag[sample_idx]),
                int(ant1_real[sample_idx]),
                int(ant1_imag[sample_idx]),
            )

            dut.s_axis_tdata.value = packed
            dut.s_axis_tuser.value = symbol_idx & 0x7F
            dut.s_axis_tvalid.value = 1

            while True:
                await RisingEdge(dut.clk_axis)
                if dut.s_axis_tready.value:
                    break

            dut.s_axis_tvalid.value = 0

    dut.s_axis_tdata.value = 0
    dut.s_axis_tuser.value = 0
    dut.s_axis_tvalid.value = 0

    outputs: dict[int, dict[str, list[complex]]] = {
        idx: {"ant0": [], "ant1": []} for idx in range(len(symbol_tones))
    }
    observed_symbols: dict[int, int] = {}
    tlast_counts = {idx: 0 for idx in range(len(symbol_tones))}

    total_expected = len(symbol_tones) * NFFT
    received = 0
    max_cycles = len(symbol_tones) * (NFFT + GAP_CYCLES + 2000)

    for _ in range(max_cycles):
        await RisingEdge(dut.clk_axis)
        if dut.m_axis_tvalid.value:
            data_word = int(dut.m_axis_tdata.value)
            symbol_id = int(dut.m_axis_tuser.value)
            observed_symbols[symbol_id] = observed_symbols.get(symbol_id, 0) + 1
            if symbol_id not in outputs:
                raise AssertionError(f"Unexpected symbol index {symbol_id} on output")

            ant0_real = _from_unsigned(data_word & ((1 << DATA_WIDTH) - 1), DATA_WIDTH)
            ant0_imag = _from_unsigned((data_word >> 12) & ((1 << DATA_WIDTH) - 1), DATA_WIDTH)
            ant1_real = _from_unsigned((data_word >> 24) & ((1 << DATA_WIDTH) - 1), DATA_WIDTH)
            ant1_imag = _from_unsigned((data_word >> 36) & ((1 << DATA_WIDTH) - 1), DATA_WIDTH)

            outputs[symbol_id]["ant0"].append(complex(ant0_real, ant0_imag))
            outputs[symbol_id]["ant1"].append(complex(ant1_real, ant1_imag))
            received += 1

            if dut.m_axis_tlast.value:
                tlast_counts[symbol_id] += 1

        if received >= total_expected:
            break

    else:
        raise AssertionError("Timed out waiting for FFT bins")

    dut = cocotb.top
    dut._log.info(f"Observed symbol counts: {observed_symbols}")

    for symbol_id in range(len(symbol_tones)):
        dut._log.info(
            f"Captured {len(outputs[symbol_id]['ant0'])} bins for symbol {symbol_id} (ant0)"
        )
        dut._log.info(
            f"Captured {len(outputs[symbol_id]['ant1'])} bins for symbol {symbol_id} (ant1)"
        )
        assert len(outputs[symbol_id]["ant0"]) == NFFT
        assert len(outputs[symbol_id]["ant1"]) == NFFT
        assert tlast_counts[symbol_id] == 1, "TLAST did not assert exactly once per symbol"

    for _ in range(512):
        await RisingEdge(dut.clk_axis)
        if int(dut.status_flags.value) & 1 == 0:
            break
    else:
        raise AssertionError("Busy flag remained set after processing completed")

    status_value = int(dut.status_flags.value)
    assert (status_value & 0b111110) == 0, f"Status flags reported an error: {status_value:06b}"

    plots_dir = Path(__file__).parent / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)

    for symbol_id, stimulus in enumerate(stimulus_symbols):
        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        freq_axis = np.arange(NFFT)

        for row, antenna in enumerate(["ant0", "ant1"]):
            rtl_bins = np.asarray(outputs[symbol_id][antenna], dtype=np.complex128)
            ref_bins = np.fft.fft(stimulus[antenna])

            rtl_mag = np.abs(rtl_bins)
            ref_mag = np.abs(ref_bins)

            rtl_norm = rtl_mag / np.max(rtl_mag) if np.max(rtl_mag) > 0 else rtl_mag
            ref_norm = ref_mag / np.max(ref_mag) if np.max(ref_mag) > 0 else ref_mag

            max_error = float(np.max(np.abs(rtl_norm - ref_norm)))
            rtl_peak_bin = int(np.argmax(rtl_mag))
            ref_peak_bin = int(np.argmax(ref_mag))
            dut._log.info(
                f"Symbol {symbol_id} {antenna}: max_error={max_error:.4f}, rtl_peak_bin={rtl_peak_bin}, ref_peak_bin={ref_peak_bin}"
            )
            assert max_error < 0.05, f"Normalized magnitude mismatch on symbol {symbol_id} {antenna}"
            assert int(np.argmax(rtl_mag)) == int(np.argmax(ref_mag)), "FFT peak bin mismatch"

            axes[row].plot(freq_axis, ref_norm, label="NumPy", linewidth=0.8)
            axes[row].plot(freq_axis, rtl_norm, label="RTL", linestyle="--", linewidth=0.8)
            axes[row].set_ylabel("Normalized |X[k]|")
            axes[row].set_title(f"Symbol {symbol_id} - {antenna.upper()}")
            axes[row].grid(True, linestyle=":", linewidth=0.5)
            axes[row].legend()

        axes[-1].set_xlabel("FFT Bin")
        fig.tight_layout()
        plot_path = plots_dir / f"fft_streaming_wrapper_symbol{symbol_id}.png"
        fig.savefig(plot_path, dpi=150)
        plt.close(fig)
        dut._log.info(f"Saved FFT magnitude comparison to {plot_path}")


@pytest.mark.skipif(VERILATOR is None, reason="Verilator is required to run this test")
def test_fft_streaming_wrapper(tmp_path: Path):
    tests_dir = Path(__file__).parent
    rtl_dir = tests_dir.parent / "rtl"

    run(
        verilog_sources=[
            str(rtl_dir / "fft_streaming_wrapper.v"),
            str(rtl_dir / "dual_clock_fifo.v"),
            str(rtl_dir / "spiral_dft_iterative_2048pt.v"),
        ],
        toplevel="fft_streaming_wrapper",
        toplevel_lang="verilog",
        module="test_fft_streaming_wrapper",
        sim_build=str(tmp_path / "sim_build_fft_streaming_wrapper"),
        simulator="verilator",
        compile_args=["-Wno-WIDTHEXPAND"],
        waves=False,
    )
