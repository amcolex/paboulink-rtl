import math
import os
import random
import shutil
from pathlib import Path

import numpy as np
import pytest
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, Timer
from cocotb_test.simulator import run

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

VERILATOR = shutil.which("verilator")

WIDTH = 12
CHANNELS = 2
ACC_WIDTH = 32
LUT_ADDR_WIDTH = 8
LUT_DATA_WIDTH = 16
FRACTION_BITS = LUT_DATA_WIDTH - 1
SCALE = (1 << (WIDTH - 1)) - 1
PHASE_MASK = (1 << ACC_WIDTH) - 1
TUSER_WIDTH = 4
TUSER_MASK = (1 << TUSER_WIDTH) - 1


def _quantize(value: float) -> int:
    clipped = max(min(value, 0.999), -0.999)
    return int(round(clipped * SCALE))


def _pack_samples(i_vals, q_vals) -> int:
    value = 0
    for ch in range(CHANNELS):
        base = ch * 2 * WIDTH
        value |= (i_vals[ch] & ((1 << WIDTH) - 1)) << base
        value |= (q_vals[ch] & ((1 << WIDTH) - 1)) << (base + WIDTH)
    return value


def _signed_from_bits(value: int, width: int) -> int:
    if value & (1 << (width - 1)):
        value -= 1 << width
    return value


def _tuser_value(sample_idx: int) -> int:
    return sample_idx & TUSER_MASK


def _generate_luts():
    size = 1 << LUT_ADDR_WIDTH
    amplitude = (1 << (LUT_DATA_WIDTH - 1)) - 1
    cos_lut = [int(round(math.cos(2 * math.pi * idx / size) * amplitude)) for idx in range(size)]
    sin_lut = [int(round(math.sin(2 * math.pi * idx / size) * amplitude)) for idx in range(size)]
    return cos_lut, sin_lut


COS_LUT, SIN_LUT = _generate_luts()


def _phase_to_index(phase: int) -> int:
    return (phase >> (ACC_WIDTH - LUT_ADDR_WIDTH)) & ((1 << LUT_ADDR_WIDTH) - 1)


def _rotate_fixed_point(i_in: int, q_in: int, cos_val: int, sin_val: int) -> complex:
    real_tmp = i_in * cos_val + q_in * sin_val
    imag_tmp = q_in * cos_val - i_in * sin_val
    real_fixed = real_tmp >> FRACTION_BITS
    imag_fixed = imag_tmp >> FRACTION_BITS
    max_val = (1 << (WIDTH - 1)) - 1
    min_val = -(1 << (WIDTH - 1))
    real_fixed = max(min(real_fixed, max_val), min_val)
    imag_fixed = max(min(imag_fixed, max_val), min_val)
    return complex(real_fixed / SCALE, imag_fixed / SCALE)


def _compute_expected_records(iq_records, phase_schedule, phase_start=0):
    assert len(iq_records) == len(phase_schedule)
    expected = []
    phase = phase_start & PHASE_MASK
    for sample_idx, sample in enumerate(iq_records):
        phase_index = _phase_to_index(phase)
        cos_val = COS_LUT[phase_index]
        sin_val = SIN_LUT[phase_index]
        row = [_rotate_fixed_point(i_in, q_in, cos_val, sin_val) for (i_in, q_in) in sample]
        expected.append(row)
        phase = (phase + (phase_schedule[sample_idx] & PHASE_MASK)) & PHASE_MASK
    return expected


async def _reset(dut):
    dut.rst_n.value = 0
    dut.s_axis_tuser.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)


async def _axi_write(dut, addr: int, data: int):
    dut.s_axi_awaddr.value = addr & 0xF
    dut.s_axi_awvalid.value = 1
    dut.s_axi_wdata.value = data & 0xFFFFFFFF
    dut.s_axi_wvalid.value = 1
    dut.s_axi_wstrb.value = 0xF
    dut.s_axi_bready.value = 1

    while True:
        await RisingEdge(dut.clk)
        if dut.s_axi_awready.value and dut.s_axi_wready.value:
            break

    dut.s_axi_awvalid.value = 0
    dut.s_axi_wvalid.value = 0

    while not dut.s_axi_bvalid.value:
        await RisingEdge(dut.clk)

    await RisingEdge(dut.clk)
    dut.s_axi_bready.value = 0


async def _axi_read(dut, addr: int) -> int:
    dut.s_axi_araddr.value = addr & 0xF
    dut.s_axi_arvalid.value = 1
    dut.s_axi_rready.value = 1

    while True:
        await RisingEdge(dut.clk)
        if dut.s_axi_arready.value:
            break

    dut.s_axi_arvalid.value = 0

    while not dut.s_axi_rvalid.value:
        await RisingEdge(dut.clk)

    value = int(dut.s_axi_rdata.value)
    dut.s_axi_rready.value = 0
    await RisingEdge(dut.clk)
    return value


async def _capture_stream(dut, expected_samples: int, ready_generator=None):
    received = 0
    output_records = []
    tuser_trace = []
    if ready_generator is None:
        dut.m_axis_tready.value = 1
    while received < expected_samples:
        await RisingEdge(dut.clk)
        if ready_generator is not None:
            dut.m_axis_tready.value = 1 if next(ready_generator) else 0
        if dut.m_axis_tvalid.value and dut.m_axis_tready.value:
            word = int(dut.m_axis_tdata.value)
            tuser_trace.append(int(dut.m_axis_tuser.value) & TUSER_MASK)
            row = []
            for ch in range(CHANNELS):
                base = ch * 2 * WIDTH
                i_raw = (word >> base) & ((1 << WIDTH) - 1)
                q_raw = (word >> (base + WIDTH)) & ((1 << WIDTH) - 1)
                i_val = _signed_from_bits(i_raw, WIDTH)
                q_val = _signed_from_bits(q_raw, WIDTH)
                row.append(complex(i_val / SCALE, q_val / SCALE))
            output_records.append(row)
            dut._log.debug(
                "Output sample %d: tlast=%d values=%s"
                % (received, int(dut.m_axis_tlast.value), row)
            )
            if dut.m_axis_tlast.value:
                if received != expected_samples - 1:
                    dut._log.error(
                        "Unexpected tlast at sample %d (expected final index %d)"
                        % (received, expected_samples - 1)
                    )
                assert received == expected_samples - 1
            received += 1
    return output_records, tuser_trace


def _maybe_plot(records_in, records_out, title_prefix: str, real_path: Path, imag_path: Path):
    time_axis = np.arange(len(records_in))

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for ch, ax in enumerate(axes):
        ax.plot(time_axis, [val[ch].real for val in records_in], label=f"Channel {ch} Input", alpha=0.6)
        ax.plot(time_axis, [val[ch].real for val in records_out], label=f"Channel {ch} Output", linewidth=2)
        ax.set_ylabel("Real")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("Sample")
    fig.suptitle(f"{title_prefix} (Real Component)")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(real_path, dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for ch, ax in enumerate(axes):
        ax.plot(time_axis, [val[ch].imag for val in records_in], label=f"Channel {ch} Input", alpha=0.6)
        ax.plot(time_axis, [val[ch].imag for val in records_out], label=f"Channel {ch} Output", linewidth=2)
        ax.set_ylabel("Imag")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("Sample")
    fig.suptitle(f"{title_prefix} (Imag Component)")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(imag_path, dpi=150)
    plt.close(fig)


def _plot_segmented(records_in, records_out, title_prefix: str, segment_boundaries, real_path: Path, imag_path: Path):
    time_axis = np.arange(len(records_in))

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for ch, ax in enumerate(axes):
        ax.plot(time_axis, [val[ch].real for val in records_in], label=f"Channel {ch} Input", alpha=0.5)
        ax.plot(time_axis, [val[ch].real for val in records_out], label=f"Channel {ch} Output", linewidth=2)
        for boundary in segment_boundaries:
            ax.axvline(boundary - 0.5, color="k", linestyle="--", alpha=0.35)
        ax.set_ylabel("Real")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("Sample")
    fig.suptitle(f"{title_prefix} (Real Component)")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(real_path, dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for ch, ax in enumerate(axes):
        ax.plot(time_axis, [val[ch].imag for val in records_in], label=f"Channel {ch} Input", alpha=0.5)
        ax.plot(time_axis, [val[ch].imag for val in records_out], label=f"Channel {ch} Output", linewidth=2)
        for boundary in segment_boundaries:
            ax.axvline(boundary - 0.5, color="k", linestyle="--", alpha=0.35)
        ax.set_ylabel("Imag")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("Sample")
    fig.suptitle(f"{title_prefix} (Imag Component)")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(imag_path, dpi=150)
    plt.close(fig)


@cocotb.test()
async def nco_cfo_compensation(dut):
    clock = Clock(dut.clk, 5, units="ns")
    cocotb.start_soon(clock.start())

    dut.m_axis_tready.value = 1
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tuser.value = 0
    dut.s_axis_tlast.value = 0
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_arvalid.value = 0
    dut.s_axi_bready.value = 0
    dut.s_axi_rready.value = 0
    dut.s_axi_wstrb.value = 0

    await _reset(dut)

    freq_norm = 0.0625
    phase_inc = int(freq_norm * (1 << ACC_WIDTH)) & PHASE_MASK
    await _axi_write(dut, 0x0, phase_inc)

    num_samples = 128
    baseband_ch0 = np.exp(1j * 0.05 * np.arange(num_samples)) * 0.7
    baseband_ch1 = np.exp(1j * 0.09 * np.arange(num_samples)) * 0.6
    baseband = np.vstack([baseband_ch0, baseband_ch1])

    quantized_records = []
    input_records = []
    capture_task = cocotb.start_soon(_capture_stream(dut, num_samples))
    for idx in range(num_samples):
        cfo_angle = (phase_inc * idx / float(1 << ACC_WIDTH)) * 2 * math.pi
        rot = complex(math.cos(cfo_angle), math.sin(cfo_angle))
        samples = baseband[:, idx] * rot
        i_vals = [_quantize(samples[ch].real) for ch in range(CHANNELS)]
        q_vals = [_quantize(samples[ch].imag) for ch in range(CHANNELS)]
        quantized_records.append(list(zip(i_vals, q_vals)))
        input_records.append([complex(i_vals[ch] / SCALE, q_vals[ch] / SCALE) for ch in range(CHANNELS)])
        dut.s_axis_tdata.value = _pack_samples(i_vals, q_vals)
        dut.s_axis_tvalid.value = 1
        dut.s_axis_tuser.value = _tuser_value(idx)
        dut.s_axis_tlast.value = 1 if idx == num_samples - 1 else 0
        while True:
            await RisingEdge(dut.clk)
            if dut.s_axis_tready.value and dut.s_axis_tvalid.value:
                break
        dut.s_axis_tvalid.value = 0
        dut.s_axis_tlast.value = 0
        dut.s_axis_tuser.value = 0
        if idx < 3:
            dut._log.debug(
                "Input sample %d delivered: I=%s Q=%s"
                % (idx, [val for val, _ in quantized_records[idx]], [val for _, val in quantized_records[idx]])
            )

    output_records, output_tuser = await capture_task

    expected = _compute_expected_records(quantized_records, [phase_inc] * num_samples)
    tol = 1.5 / SCALE
    for idx in range(num_samples):
        for ch in range(CHANNELS):
            diff_real = abs(expected[idx][ch].real - output_records[idx][ch].real)
            diff_imag = abs(expected[idx][ch].imag - output_records[idx][ch].imag)
            if diff_real > tol or diff_imag > tol:
                dut._log.error(
                    "Mismatch sample %d channel %d: expected=%s got=%s"
                    % (idx, ch, expected[idx][ch], output_records[idx][ch])
                )
            assert diff_real <= tol
            assert diff_imag <= tol

    expected_tuser = [_tuser_value(idx) for idx in range(num_samples)]
    assert output_tuser == expected_tuser

    plot_dir = Path(__file__).resolve().parent / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    _maybe_plot(
        input_records,
        output_records,
        "NCO CFO Compensation",
        plot_dir / "nco_cfo_compensation.png",
        plot_dir / "nco_cfo_compensation_imag.png",
    )


@cocotb.test()
async def nco_cfo_dynamic_update(dut):
    clock = Clock(dut.clk, 5, units="ns")
    cocotb.start_soon(clock.start())

    dut.m_axis_tready.value = 1
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tuser.value = 0
    dut.s_axis_tlast.value = 0
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_arvalid.value = 0
    dut.s_axi_bready.value = 0
    dut.s_axi_rready.value = 0
    dut.s_axi_wstrb.value = 0

    await _reset(dut)

    freq_init = 0.05
    freq_updated = -0.08
    phase_inc_init = int(freq_init * (1 << ACC_WIDTH)) & PHASE_MASK
    phase_inc_updated = int(freq_updated * (1 << ACC_WIDTH)) & PHASE_MASK
    await _axi_write(dut, 0x0, phase_inc_init)

    num_samples = 160
    change_idx = num_samples // 2
    baseband_ch0 = np.full(num_samples, 0.75 + 0.1j, dtype=np.complex128)
    baseband_ch1 = np.full(num_samples, 0.55 - 0.25j, dtype=np.complex128)
    baseband = np.vstack([baseband_ch0, baseband_ch1])

    quantized_records = []
    input_records = []
    phase_schedule = []
    phase_inc = phase_inc_init

    capture_task = cocotb.start_soon(_capture_stream(dut, num_samples))

    for idx in range(num_samples):
        if idx == change_idx:
            await _axi_write(dut, 0x0, phase_inc_updated)
            phase_inc = phase_inc_updated
        phase_schedule.append(phase_inc)

        phase_acc = sum(phase_schedule[:idx]) & PHASE_MASK
        cfo_angle = (phase_acc / float(1 << ACC_WIDTH)) * 2 * math.pi
        rot = complex(math.cos(cfo_angle), math.sin(cfo_angle))
        samples = baseband[:, idx] * rot
        i_vals = [_quantize(samples[ch].real) for ch in range(CHANNELS)]
        q_vals = [_quantize(samples[ch].imag) for ch in range(CHANNELS)]
        quantized_records.append(list(zip(i_vals, q_vals)))
        input_records.append([complex(i_vals[ch] / SCALE, q_vals[ch] / SCALE) for ch in range(CHANNELS)])
        dut.s_axis_tdata.value = _pack_samples(i_vals, q_vals)
        dut.s_axis_tvalid.value = 1
        dut.s_axis_tuser.value = _tuser_value(idx)
        dut.s_axis_tlast.value = 1 if idx == num_samples - 1 else 0
        while True:
            await RisingEdge(dut.clk)
            if dut.s_axis_tready.value and dut.s_axis_tvalid.value:
                break
        dut.s_axis_tvalid.value = 0
        dut.s_axis_tlast.value = 0
        dut.s_axis_tuser.value = 0

    output_records, output_tuser = await capture_task

    expected = _compute_expected_records(quantized_records, phase_schedule)
    tol = 1.5 / SCALE
    for idx in range(num_samples):
        for ch in range(CHANNELS):
            diff_real = abs(expected[idx][ch].real - output_records[idx][ch].real)
            diff_imag = abs(expected[idx][ch].imag - output_records[idx][ch].imag)
            if diff_real > tol or diff_imag > tol:
                dut._log.error(
                    "Mismatch sample %d channel %d: expected=%s got=%s"
                    % (idx, ch, expected[idx][ch], output_records[idx][ch])
                )
            assert diff_real <= tol
            assert diff_imag <= tol

    expected_tuser = [_tuser_value(idx) for idx in range(num_samples)]
    assert output_tuser == expected_tuser

    plot_dir = Path(__file__).resolve().parent / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    _maybe_plot(
        input_records,
        output_records,
        "On-the-fly CFO Update",
        plot_dir / "nco_cfo_dynamic_real.png",
        plot_dir / "nco_cfo_dynamic_imag.png",
    )


@cocotb.test()
async def nco_cfo_multi_cfo_sweep(dut):
    clock = Clock(dut.clk, 5, units="ns")
    cocotb.start_soon(clock.start())

    dut.m_axis_tready.value = 1
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tuser.value = 0
    dut.s_axis_tlast.value = 0
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_arvalid.value = 0
    dut.s_axi_bready.value = 0
    dut.s_axi_rready.value = 0
    dut.s_axi_wstrb.value = 0

    await _reset(dut)

    segment_cfos = [0.0, 0.0375, -0.12, 0.25]
    segment_length = 80
    num_samples = segment_length * len(segment_cfos)

    baseband_ch0 = np.exp(1j * 0.02 * np.arange(segment_length)) * 0.65
    baseband_ch1 = np.exp(1j * 0.04 * np.arange(segment_length)) * 0.45
    baseband_segment = np.vstack([baseband_ch0, baseband_ch1])

    quantized_records = []
    input_records = []
    phase_schedule = []

    capture_task = cocotb.start_soon(_capture_stream(dut, num_samples))

    phase_acc = 0
    for seg_idx, freq in enumerate(segment_cfos):
        phase_inc = int(freq * (1 << ACC_WIDTH)) & PHASE_MASK
        await _axi_write(dut, 0x0, phase_inc)
        for idx in range(segment_length):
            sample_idx = seg_idx * segment_length + idx
            angle = (phase_acc / float(1 << ACC_WIDTH)) * 2 * math.pi
            rot = complex(math.cos(angle), math.sin(angle))
            samples = baseband_segment[:, idx] * rot
            i_vals = [_quantize(samples[ch].real) for ch in range(CHANNELS)]
            q_vals = [_quantize(samples[ch].imag) for ch in range(CHANNELS)]
            quantized_records.append(list(zip(i_vals, q_vals)))
            input_records.append([complex(i_vals[ch] / SCALE, q_vals[ch] / SCALE) for ch in range(CHANNELS)])
            dut.s_axis_tdata.value = _pack_samples(i_vals, q_vals)
            dut.s_axis_tvalid.value = 1
            dut.s_axis_tuser.value = _tuser_value(sample_idx)
            dut.s_axis_tlast.value = 1 if (seg_idx == len(segment_cfos) - 1 and idx == segment_length - 1) else 0
            while True:
                await RisingEdge(dut.clk)
                if dut.s_axis_tready.value and dut.s_axis_tvalid.value:
                    break
            dut.s_axis_tvalid.value = 0
            dut.s_axis_tlast.value = 0
            dut.s_axis_tuser.value = 0
            phase_schedule.append(phase_inc)
            phase_acc = (phase_acc + phase_inc) & PHASE_MASK

    output_records, output_tuser = await capture_task

    expected = _compute_expected_records(quantized_records, phase_schedule)
    tol = 1.5 / SCALE
    for idx in range(num_samples):
        for ch in range(CHANNELS):
            diff_real = abs(expected[idx][ch].real - output_records[idx][ch].real)
            diff_imag = abs(expected[idx][ch].imag - output_records[idx][ch].imag)
            if diff_real > tol or diff_imag > tol:
                dut._log.error(
                    "Mismatch sample %d channel %d: expected=%s got=%s"
                    % (idx, ch, expected[idx][ch], output_records[idx][ch])
                )
            assert diff_real <= tol
            assert diff_imag <= tol

    expected_tuser = [_tuser_value(idx) for idx in range(num_samples)]
    assert output_tuser == expected_tuser

    plot_dir = Path(__file__).resolve().parent / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    segment_boundaries = [segment_length * i for i in range(1, len(segment_cfos))]
    _plot_segmented(
        input_records,
        output_records,
        "Multi-CFO Sweep",
        segment_boundaries,
        plot_dir / "nco_cfo_sweep_real.png",
        plot_dir / "nco_cfo_sweep_imag.png",
    )


@cocotb.test()
async def nco_cfo_constant_input_cfo_sweep(dut):
    clock = Clock(dut.clk, 5, units="ns")
    cocotb.start_soon(clock.start())

    dut.m_axis_tready.value = 1
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tuser.value = 0
    dut.s_axis_tlast.value = 0
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_arvalid.value = 0
    dut.s_axi_bready.value = 0
    dut.s_axi_rready.value = 0
    dut.s_axi_wstrb.value = 0

    await _reset(dut)

    segment_cfos = [0.0, 0.05, -0.07, 0.18]
    segment_length = 96
    num_samples = segment_length * len(segment_cfos)

    t_full = np.arange(num_samples)
    baseband_ch0_full = np.exp(1j * 0.03 * t_full) * 0.62
    baseband_ch1_full = np.exp(1j * 0.055 * t_full) * 0.52
    baseband_full = np.vstack([baseband_ch0_full, baseband_ch1_full])

    quantized_records = []
    input_records = []
    phase_schedule = []

    capture_task = cocotb.start_soon(_capture_stream(dut, num_samples))

    for seg_idx, freq in enumerate(segment_cfos):
        phase_inc = int(freq * (1 << ACC_WIDTH)) & PHASE_MASK
        await _axi_write(dut, 0x0, phase_inc)

        seg_start = seg_idx * segment_length
        seg_end = seg_start + segment_length
        for sample_idx in range(seg_start, seg_end):
            samples = baseband_full[:, sample_idx]
            i_vals = [_quantize(samples[ch].real) for ch in range(CHANNELS)]
            q_vals = [_quantize(samples[ch].imag) for ch in range(CHANNELS)]
            quantized_records.append(list(zip(i_vals, q_vals)))
            input_records.append([complex(i_vals[ch] / SCALE, q_vals[ch] / SCALE) for ch in range(CHANNELS)])
            dut.s_axis_tdata.value = _pack_samples(i_vals, q_vals)
            dut.s_axis_tvalid.value = 1
            dut.s_axis_tuser.value = _tuser_value(sample_idx)
            dut.s_axis_tlast.value = 1 if sample_idx == num_samples - 1 else 0
            while True:
                await RisingEdge(dut.clk)
                if dut.s_axis_tready.value and dut.s_axis_tvalid.value:
                    break
            dut.s_axis_tvalid.value = 0
            dut.s_axis_tlast.value = 0
            dut.s_axis_tuser.value = 0
            phase_schedule.append(phase_inc)

    output_records, output_tuser = await capture_task

    expected = _compute_expected_records(quantized_records, phase_schedule)
    tol = 1.5 / SCALE
    for idx in range(num_samples):
        for ch in range(CHANNELS):
            diff_real = abs(expected[idx][ch].real - output_records[idx][ch].real)
            diff_imag = abs(expected[idx][ch].imag - output_records[idx][ch].imag)
            if diff_real > tol or diff_imag > tol:
                dut._log.error(
                    "Mismatch sample %d channel %d: expected=%s got=%s"
                    % (idx, ch, expected[idx][ch], output_records[idx][ch])
                )
            assert diff_real <= tol
            assert diff_imag <= tol

    expected_tuser = [_tuser_value(idx) for idx in range(num_samples)]
    assert output_tuser == expected_tuser

    plot_dir = Path(__file__).resolve().parent / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    segment_boundaries = [segment_length * i for i in range(1, len(segment_cfos))]
    _plot_segmented(
        input_records,
        output_records,
        "Constant Input, CFO Sweep",
        segment_boundaries,
        plot_dir / "nco_cfo_constant_signal_real.png",
        plot_dir / "nco_cfo_constant_signal_imag.png",
    )


@cocotb.test()
async def nco_cfo_zero_cfo_passthrough(dut):
    clock = Clock(dut.clk, 5, units="ns")
    cocotb.start_soon(clock.start())

    dut.m_axis_tready.value = 1
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tuser.value = 0
    dut.s_axis_tlast.value = 0
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_arvalid.value = 0
    dut.s_axi_bready.value = 0
    dut.s_axi_rready.value = 0
    dut.s_axi_wstrb.value = 0

    await _reset(dut)

    await _axi_write(dut, 0x0, 0)

    num_samples = 64
    rng = random.Random(42)
    quantized_records = []
    input_records = []
    capture_task = cocotb.start_soon(_capture_stream(dut, num_samples))

    for idx in range(num_samples):
        i_vals = [rng.randint(-(1 << (WIDTH - 1)) + 10, (1 << (WIDTH - 1)) - 10) for _ in range(CHANNELS)]
        q_vals = [rng.randint(-(1 << (WIDTH - 1)) + 10, (1 << (WIDTH - 1)) - 10) for _ in range(CHANNELS)]
        quantized_records.append(list(zip(i_vals, q_vals)))
        input_records.append([complex(i_vals[ch] / SCALE, q_vals[ch] / SCALE) for ch in range(CHANNELS)])
        dut.s_axis_tdata.value = _pack_samples(i_vals, q_vals)
        dut.s_axis_tvalid.value = 1
        dut.s_axis_tuser.value = _tuser_value(idx)
        dut.s_axis_tlast.value = 1 if idx == num_samples - 1 else 0
        while True:
            await RisingEdge(dut.clk)
            if dut.s_axis_tready.value and dut.s_axis_tvalid.value:
                break
        dut.s_axis_tvalid.value = 0
        dut.s_axis_tlast.value = 0
        dut.s_axis_tuser.value = 0

    output_records, output_tuser = await capture_task

    expected = _compute_expected_records(quantized_records, [0] * num_samples)
    tol = 1.5 / SCALE
    for idx in range(num_samples):
        for ch in range(CHANNELS):
            diff_real = abs(expected[idx][ch].real - output_records[idx][ch].real)
            diff_imag = abs(expected[idx][ch].imag - output_records[idx][ch].imag)
            if diff_real > tol or diff_imag > tol:
                dut._log.error(
                    "Mismatch sample %d channel %d: expected=%s got=%s"
                    % (idx, ch, expected[idx][ch], output_records[idx][ch])
                )
            assert diff_real <= tol
            assert diff_imag <= tol

    expected_tuser = [_tuser_value(idx) for idx in range(num_samples)]
    assert output_tuser == expected_tuser


@cocotb.test()
async def nco_cfo_backpressure_stress(dut):
    clock = Clock(dut.clk, 5, units="ns")
    cocotb.start_soon(clock.start())

    dut.m_axis_tready.value = 1
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tuser.value = 0
    dut.s_axis_tlast.value = 0
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_arvalid.value = 0
    dut.s_axi_bready.value = 0
    dut.s_axi_rready.value = 0
    dut.s_axi_wstrb.value = 0

    await _reset(dut)

    freq_norm = 0.1875
    phase_inc = int(freq_norm * (1 << ACC_WIDTH)) & PHASE_MASK
    await _axi_write(dut, 0x0, phase_inc)

    num_samples = 96
    rng = random.Random(1337)
    quantized_records = []

    def ready_generator():
        while True:
            for _ in range(3):
                yield 1
            yield 0

    ready_gen = ready_generator()
    capture_task = cocotb.start_soon(_capture_stream(dut, num_samples, ready_gen))

    for idx in range(num_samples):
        if rng.random() < 0.25:
            for _ in range(rng.randint(1, 3)):
                await RisingEdge(dut.clk)
        i_vals = [rng.randint(-(1 << (WIDTH - 1)) + 1, (1 << (WIDTH - 1)) - 1) for _ in range(CHANNELS)]
        q_vals = [rng.randint(-(1 << (WIDTH - 1)) + 1, (1 << (WIDTH - 1)) - 1) for _ in range(CHANNELS)]
        quantized_records.append(list(zip(i_vals, q_vals)))
        dut.s_axis_tdata.value = _pack_samples(i_vals, q_vals)
        dut.s_axis_tvalid.value = 1
        dut.s_axis_tuser.value = _tuser_value(idx)
        dut.s_axis_tlast.value = 1 if idx == num_samples - 1 else 0
        while True:
            await RisingEdge(dut.clk)
            if dut.s_axis_tready.value and dut.s_axis_tvalid.value:
                break
        dut.s_axis_tvalid.value = 0
        dut.s_axis_tlast.value = 0
        dut.s_axis_tuser.value = 0

    output_records, output_tuser = await capture_task
    expected = _compute_expected_records(quantized_records, [phase_inc] * num_samples)
    tol = 1.5 / SCALE
    for idx in range(num_samples):
        for ch in range(CHANNELS):
            diff_real = abs(expected[idx][ch].real - output_records[idx][ch].real)
            diff_imag = abs(expected[idx][ch].imag - output_records[idx][ch].imag)
            if diff_real > tol or diff_imag > tol:
                dut._log.error(
                    "Mismatch sample %d channel %d: expected=%s got=%s"
                    % (idx, ch, expected[idx][ch], output_records[idx][ch])
                )
            assert diff_real <= tol
            assert diff_imag <= tol

    expected_tuser = [_tuser_value(idx) for idx in range(num_samples)]
    assert output_tuser == expected_tuser


@cocotb.test()
async def nco_cfo_saturation_extents(dut):
    clock = Clock(dut.clk, 5, units="ns")
    cocotb.start_soon(clock.start())

    dut.m_axis_tready.value = 1
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tuser.value = 0
    dut.s_axis_tlast.value = 0
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_arvalid.value = 0
    dut.s_axi_bready.value = 0
    dut.s_axi_rready.value = 0
    dut.s_axi_wstrb.value = 0

    await _reset(dut)

    freq_norm = 0.3125
    phase_inc = int(freq_norm * (1 << ACC_WIDTH)) & PHASE_MASK
    await _axi_write(dut, 0x0, phase_inc)

    num_samples = 32
    max_val = (1 << (WIDTH - 1)) - 1
    min_val = -(1 << (WIDTH - 1))
    quantized_records = []
    patterns = [
        (max_val, max_val),
        (min_val, min_val),
        (max_val, min_val),
        (min_val, max_val),
    ]

    capture_task = cocotb.start_soon(_capture_stream(dut, num_samples))

    for idx in range(num_samples):
        pattern = patterns[idx % len(patterns)]
        i_vals = [pattern[0]] * CHANNELS
        q_vals = [pattern[1]] * CHANNELS
        quantized_records.append(list(zip(i_vals, q_vals)))
        dut.s_axis_tdata.value = _pack_samples(i_vals, q_vals)
        dut.s_axis_tvalid.value = 1
        dut.s_axis_tuser.value = _tuser_value(idx)
        dut.s_axis_tlast.value = 1 if idx == num_samples - 1 else 0
        while True:
            await RisingEdge(dut.clk)
            if dut.s_axis_tready.value and dut.s_axis_tvalid.value:
                break
        dut.s_axis_tvalid.value = 0
        dut.s_axis_tlast.value = 0
        dut.s_axis_tuser.value = 0
    output_records, output_tuser = await capture_task

    expected = _compute_expected_records(quantized_records, [phase_inc] * num_samples)
    for idx in range(num_samples):
        for ch in range(CHANNELS):
            exp_val = expected[idx][ch]
            got_val = output_records[idx][ch]
            diff_real = abs(exp_val.real - got_val.real)
            diff_imag = abs(exp_val.imag - got_val.imag)
            if diff_real > (1.5 / SCALE) or diff_imag > (1.5 / SCALE):
                dut._log.error(
                    "Mismatch sample %d channel %d: expected=%s got=%s"
                    % (idx, ch, exp_val, got_val)
                )
            assert diff_real <= (1.5 / SCALE)
            assert diff_imag <= (1.5 / SCALE)
            assert -1.001 <= got_val.real <= 1.001
            assert -1.001 <= got_val.imag <= 1.001

    expected_tuser = [_tuser_value(idx) for idx in range(num_samples)]
    assert output_tuser == expected_tuser


@cocotb.test()
async def nco_cfo_axi_lite_readback(dut):
    clock = Clock(dut.clk, 5, units="ns")
    cocotb.start_soon(clock.start())

    dut.m_axis_tready.value = 1
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tlast.value = 0
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_arvalid.value = 0
    dut.s_axi_bready.value = 0
    dut.s_axi_rready.value = 0
    dut.s_axi_wstrb.value = 0

    await _reset(dut)

    target_incs = [
        int(0.0 * (1 << ACC_WIDTH)) & PHASE_MASK,
        int(0.11 * (1 << ACC_WIDTH)) & PHASE_MASK,
        int(-0.19 * (1 << ACC_WIDTH)) & PHASE_MASK,
        int(0.32 * (1 << ACC_WIDTH)) & PHASE_MASK,
    ]

    for inc in target_incs:
        await _axi_write(dut, 0x0, inc)
        read_val = await _axi_read(dut, 0x0)
        assert (read_val & PHASE_MASK) == inc


@pytest.mark.skipif(VERILATOR is None, reason="Verilator executable not found; install Verilator to run this test.")
def test_nco_cfo_compensator():
    rtl_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "rtl"))
    rtl_file = os.path.join(rtl_dir, "nco_cfo_compensator.sv")
    build_dir = os.path.join("tests", "sim_build", "nco_cfo_compensator")
    vcd_dir = Path(__file__).resolve().parents[1] / "vcd" / "nco_cfo_compensator"
    vcd_dir.mkdir(parents=True, exist_ok=True)
    vcd_path = vcd_dir / "nco_cfo_compensator.vcd"
    vcd_path.unlink(missing_ok=True)

    run(
        verilog_sources=[rtl_file],
        toplevel="nco_cfo_compensator",
        toplevel_lang="verilog",
        module=os.path.splitext(os.path.basename(__file__))[0],
        parameters={
            "AXIS_TUSER_WIDTH": TUSER_WIDTH,
        },
        sim_build=build_dir,
        simulator="verilator",
        verilog_compile_args=["--trace", "--trace-structs"],
        plus_args=["--trace", "--trace-file", str(vcd_path)],
        extra_env={
            "COCOTB_RESULTS_FILE": os.path.join(build_dir, "results.xml"),
        },
    )
