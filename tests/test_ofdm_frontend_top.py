import os
import shutil
import sys
from pathlib import Path
from typing import List, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge
from cocotb_test.simulator import run
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from tests.utils.ofdm import (
    OFDMParameters,
    centered_subcarrier_indices,
    generate_preamble,
    generate_qpsk_symbol,
)

VERILATOR = shutil.which("verilator")


def _pack_sample(ch0_i: int, ch0_q: int, ch1_i: int, ch1_q: int, width: int) -> int:
    mask = (1 << width) - 1
    word = 0
    word |= (ch0_i & mask)
    word |= (ch0_q & mask) << width
    word |= (ch1_i & mask) << (2 * width)
    word |= (ch1_q & mask) << (3 * width)
    return word


def _unpack_sample(word: int, width: int) -> Tuple[int, int, int, int]:
    mask = (1 << width) - 1
    ch0_i = word & mask
    ch0_q = (word >> width) & mask
    ch1_i = (word >> (2 * width)) & mask
    ch1_q = (word >> (3 * width)) & mask

    def _to_signed(value: int) -> int:
        if value & (1 << (width - 1)):
            value -= 1 << width
        return value

    return tuple(_to_signed(v) for v in (ch0_i, ch0_q, ch1_i, ch1_q))


def _quantize(samples: np.ndarray, width: int) -> Tuple[np.ndarray, np.ndarray, float]:
    min_val = -(1 << (width - 1))
    max_val = (1 << (width - 1)) - 1
    max_mag = np.max(np.abs(samples))
    if max_mag == 0:
        scale = 1.0
    else:
        scale = (max_val - 2) / max_mag
    scaled = samples * scale
    i_vals = np.clip(np.round(scaled.real), min_val, max_val).astype(np.int32)
    q_vals = np.clip(np.round(scaled.imag), min_val, max_val).astype(np.int32)
    return i_vals, q_vals, scale


def _align_symbol(
    captured: np.ndarray, reference: np.ndarray
) -> Tuple[np.ndarray, int, complex, float]:
    fft_captured = np.fft.fft(captured)
    fft_reference = np.fft.fft(reference)
    corr = np.fft.ifft(fft_captured * np.conj(fft_reference))
    shift = int(np.argmax(np.abs(corr)))
    aligned = np.roll(captured, -shift)
    denom = np.vdot(reference, reference)
    gain = np.vdot(reference, aligned) / denom if denom != 0 else 1.0
    error = np.linalg.norm(aligned - gain * reference) / (np.sqrt(denom) + 1e-12)
    if shift > reference.size // 2:
        shift -= reference.size
    return aligned, shift, gain, error


async def _reset_dut(dut) -> None:
    dut.rst.value = 1
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tdata.value = 0
    dut.s_axis_tuser.value = 0
    dut.s_axis_tlast.value = 0
    dut.m_axis_tready.value = 0
    dut.sto_valid.value = 0
    dut.sto_correction.value = 0
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_bready.value = 0
    dut.s_axi_arvalid.value = 0
    dut.s_axi_rready.value = 0
    for _ in range(10):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)


@cocotb.test()
async def frontend_end_to_end(dut):
    clock = Clock(dut.clk, 4, units="ns")
    cocotb.start_soon(clock.start())

    await _reset_dut(dut)

    nfft = int(dut.NFFT.value)
    cp_len = int(dut.CP_LEN.value)
    n_symbols = int(dut.N_FRAME_SYMBOLS.value)
    width = int(dut.INPUT_WIDTH.value)
    frame_gap = int(dut.FRAME_GAP_SAMPLES.value)

    plot_dir = Path(__file__).resolve().parent / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)

    dut.m_axis_tready.value = 1

    params = OFDMParameters(n_fft=nfft, cp_len=cp_len)
    preamble_rng = np.random.default_rng(202)
    payload_rng = np.random.default_rng(303)

    num_frames = 2
    frame_segments: List[np.ndarray] = []
    symbol_ranges: List[Tuple[int, int]] = []
    frame_spans: List[Tuple[int, int]] = []
    gap_spans: List[Tuple[int, int]] = []
    cursor = 0

    for frame_idx in range(num_frames):
        frame_start = cursor
        preamble, _ = generate_preamble(
            params=params,
            include_cp=True,
            normalize=True,
            rng=preamble_rng,
        )
        frame_segments.append(preamble)
        cursor += preamble.size
        for _ in range(n_symbols):
            symbol_td, _ = generate_qpsk_symbol(
                params=params,
                include_cp=True,
                normalize=True,
                rng=payload_rng,
            )
            payload_start = cursor + cp_len
            frame_segments.append(symbol_td)
            symbol_ranges.append((payload_start, nfft))
            cursor += symbol_td.size
        frame_spans.append((frame_start, cursor))
        if frame_idx != num_frames - 1:
            gap = np.zeros(frame_gap, dtype=np.complex128)
            frame_segments.append(gap)
            cursor += gap.size
            gap_spans.append((cursor - gap.size, cursor))

    stream = np.concatenate(frame_segments)

    time_axis_full = np.arange(stream.size)
    frame_color = "tab:blue"
    gap_color = "tab:orange"

    plt.figure(figsize=(10, 4))
    line_i, = plt.plot(time_axis_full, stream.real, label="I", linewidth=0.7)
    line_q, = plt.plot(time_axis_full, stream.imag, label="Q", linewidth=0.7)
    for start, end in frame_spans:
        plt.axvspan(start, end, color=frame_color, alpha=0.08)
    for start, end in gap_spans:
        plt.axvspan(start, end, color=gap_color, alpha=0.12)
    handles, labels = plt.gca().get_legend_handles_labels()
    handles.extend(
        [
            Patch(facecolor=frame_color, alpha=0.2, label="Frame region"),
            Patch(facecolor=gap_color, alpha=0.2, label="Gap region"),
        ]
    )
    labels.extend(["Frame region", "Gap region"])
    plt.legend(handles, labels, loc="upper right")
    plt.title("Full Input Stream (Frames + Gaps)")
    plt.xlabel("Sample")
    plt.ylabel("Amplitude")
    plt.tight_layout()
    plt.savefig(plot_dir / "ofdm_frontend_input_stream.png", dpi=120)
    plt.close()

    first_frame_segments = frame_segments[: 1 + n_symbols]
    first_frame = np.concatenate(first_frame_segments)
    time_axis = np.arange(first_frame.size)
    preamble_len = first_frame_segments[0].size
    symbol_len = nfft + cp_len

    plt.figure(figsize=(9, 4))
    plt.plot(time_axis, first_frame.real, label="I", linewidth=0.8)
    plt.plot(time_axis, first_frame.imag, label="Q", linewidth=0.8)
    for boundary in [preamble_len + i * symbol_len for i in range(n_symbols)]:
        plt.axvline(boundary, color="k", linestyle="--", linewidth=0.6, alpha=0.4)
    plt.title("First Input Frame (Time Domain)")
    plt.xlabel("Sample")
    plt.ylabel("Amplitude")
    plt.legend(loc="upper right")
    plt.tight_layout()
    plt.savefig(plot_dir / "ofdm_frontend_input_frame.png", dpi=120)
    plt.close()

    i_vals, q_vals, scale = _quantize(stream, width)

    ch0_i = i_vals
    ch0_q = q_vals
    ch1_i = i_vals
    ch1_q = q_vals

    total_samples = stream.size
    sample_index = 0

    captured_symbols: List[List[int]] = []
    captured_symbol_users: List[int] = []
    captured_symbol_lasts: List[int] = []
    symbol_buffer: List[int] = []
    symbol_user: int | None = None
    frame_count = 0
    target_frames = 1

    while sample_index < total_samples:
        word = _pack_sample(
            int(ch0_i[sample_index]),
            int(ch0_q[sample_index]),
            int(ch1_i[sample_index]),
            int(ch1_q[sample_index]),
            width,
        )
        dut.s_axis_tdata.value = word
        dut.s_axis_tuser.value = 0
        dut.s_axis_tvalid.value = 1
        dut.s_axis_tlast.value = 0

        await RisingEdge(dut.clk)

        if dut.s_axis_tready.value:
            sample_index += 1

        if dut.m_axis_tvalid.value and frame_count < target_frames:
            word = int(dut.m_axis_tdata.value)
            user = int(dut.m_axis_tuser.value)
            last = int(dut.m_axis_tlast.value)
            if not symbol_buffer:
                symbol_user = user
            else:
                assert symbol_user == user, "Unexpected user change within symbol"
            symbol_buffer.append(word)
            if len(symbol_buffer) == nfft:
                captured_symbols.append(symbol_buffer.copy())
                captured_symbol_users.append(symbol_user if symbol_user is not None else 0)
                captured_symbol_lasts.append(last)
                symbol_buffer.clear()
                symbol_user = None
                if last:
                    frame_count += 1

    dut.s_axis_tvalid.value = 0
    dut.s_axis_tlast.value = 0

    drain_cycles = 0
    drain_limit = 40 * (nfft + cp_len)
    while frame_count < target_frames and drain_cycles < drain_limit:
        await RisingEdge(dut.clk)
        drain_cycles += 1
        if dut.m_axis_tvalid.value and frame_count < target_frames:
            word = int(dut.m_axis_tdata.value)
            user = int(dut.m_axis_tuser.value)
            last = int(dut.m_axis_tlast.value)
            if not symbol_buffer:
                symbol_user = user
            else:
                assert symbol_user == user, "Unexpected user change within symbol"
            symbol_buffer.append(word)
            if len(symbol_buffer) == nfft:
                captured_symbols.append(symbol_buffer.copy())
                captured_symbol_users.append(symbol_user if symbol_user is not None else 0)
                captured_symbol_lasts.append(last)
                symbol_buffer.clear()
                symbol_user = None
                if last:
                    frame_count += 1

    assert frame_count == target_frames
    assert not symbol_buffer, "Partial symbol remaining"
    assert len(captured_symbols) == target_frames * n_symbols
    assert captured_symbol_lasts.count(1) == target_frames

    captured_complex = []
    for word in (sample for symbol in captured_symbols for sample in symbol):
        ch0_i_val, ch0_q_val, _, _ = _unpack_sample(word, width)
        captured_complex.append((ch0_i_val + 1j * ch0_q_val) / scale)

    captured_complex = np.asarray(captured_complex, dtype=np.complex128)
    captured_complex = captured_complex.reshape((-1, nfft))

    symbol_users = captured_symbol_users
    expected_users = list(range(n_symbols)) * target_frames
    assert symbol_users == expected_users

    expected_complex = []
    for start, length in symbol_ranges:
        end = start + length
        symbol_i = ch0_i[start:end].astype(np.float64)
        symbol_q = ch0_q[start:end].astype(np.float64)
        expected_complex.append((symbol_i + 1j * symbol_q) / scale)
    expected_complex = np.asarray(expected_complex, dtype=np.complex128)

    shifts = []
    errors = []
    max_shift = cp_len + (nfft // 4)

    for idx, (observed, reference) in enumerate(zip(captured_complex, expected_complex)):
        aligned, shift, gain, error = _align_symbol(observed, reference)
        ref_max = float(np.max(np.abs(reference)))
        obs_max = float(np.max(np.abs(observed)))
        dut._log.info(
            "symbol %0d shift=%d error=%.4f gain=(%.3f%+.3fj) ref_max=%.3f obs_max=%.3f",
            idx,
            shift,
            float(np.abs(error)),
            float(np.real(gain)),
            float(np.imag(gain)),
            ref_max,
            obs_max,
        )
        shifts.append(shift)
        errors.append(error)
        assert abs(shift) <= max_shift
        assert np.abs(error) < 0.12
        expected_complex[idx] = reference
        if np.abs(gain) > 1e-9:
            captured_complex[idx] = aligned / gain
        else:
            captured_complex[idx] = aligned

    first_symbol_freq = np.fft.fftshift(np.fft.fft(captured_complex[0]))
    carriers = centered_subcarrier_indices(params.num_active)
    dc_index = nfft // 2
    active_bins = (dc_index + carriers) % nfft
    constellation = first_symbol_freq[active_bins]

    plt.figure(figsize=(5, 5))
    plt.scatter(constellation.real, constellation.imag, s=10)
    plt.title("Recovered QPSK Constellation (Frame 0, Symbol 0)")
    plt.xlabel("In-phase")
    plt.ylabel("Quadrature")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(plot_dir / "ofdm_frontend_constellation.png", dpi=120)
    plt.close()

    avg_error = float(np.mean(errors))
    assert avg_error < 0.1


@pytest.mark.skipif(VERILATOR is None, reason="Verilator executable not found")
def test_ofdm_frontend_top():
    rtl_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "rtl"))
    sources = [
        os.path.join(rtl_dir, "complex_conjugate.sv"),
        os.path.join(rtl_dir, "minn_delay_line.sv"),
        os.path.join(rtl_dir, "minn_running_sum.sv"),
        os.path.join(rtl_dir, "minn_antenna_path.sv"),
        os.path.join(rtl_dir, "minn_preamble_detector.sv"),
        os.path.join(rtl_dir, "nco_cfo_compensator.sv"),
        os.path.join(rtl_dir, "ofdm_frame_synchronizer.sv"),
        os.path.join(rtl_dir, "ofdm_frontend_top.sv"),
    ]

    build_dir = os.path.join("tests", "sim_build", "ofdm_frontend_top")

    nfft = 2048
    cp_len = 512
    n_symbols = 4
    frame_gap = 3 * (nfft + cp_len)

    run(
        verilog_sources=sources,
        toplevel="ofdm_frontend_top",
        toplevel_lang="verilog",
        module=os.path.splitext(os.path.basename(__file__))[0],
        parameters={
            "INPUT_WIDTH": 12,
            "NFFT": nfft,
            "CP_LEN": cp_len,
            "N_FRAME_SYMBOLS": n_symbols,
            "FRAME_GAP_SAMPLES": frame_gap,
            "SYMBOL_COUNTER_WIDTH": 7,
            "OUTPUT_MARGIN": cp_len,
            "THRESH_VALUE": 256,
            "THRESH_FRAC_BITS": 15,
            "SMOOTH_SHIFT": 2,
            "HYSTERESIS": 1,
            "TIMING_OFFSET": 0,
        },
        sim_build=build_dir,
        simulator="verilator",
        extra_env={
            "COCOTB_RESULTS_FILE": os.path.join(build_dir, "results.xml"),
        },
    )
