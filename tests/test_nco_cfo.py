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
CHANNELS = 2
ACC_WIDTH = 32
LUT_ADDR_WIDTH = 8
LUT_DATA_WIDTH = 16
FRACTION_BITS = LUT_DATA_WIDTH - 1
SCALE = (1 << (WIDTH - 1)) - 1


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


def _generate_luts():
    size = 1 << LUT_ADDR_WIDTH
    amplitude = (1 << (LUT_DATA_WIDTH - 1)) - 1
    cos_lut = [int(round(math.cos(2 * math.pi * idx / size) * amplitude)) for idx in range(size)]
    sin_lut = [int(round(math.sin(2 * math.pi * idx / size) * amplitude)) for idx in range(size)]
    return cos_lut, sin_lut


COS_LUT, SIN_LUT = _generate_luts()


async def _reset(dut):
    dut.rst_n.value = 0
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


@cocotb.test()
async def nco_cfo_compensation(dut):
    clock = Clock(dut.clk, 5, units="ns")
    cocotb.start_soon(clock.start())

    dut.s_axis_tvalid.value = 0
    dut.s_axis_tlast.value = 0
    dut.m_axis_tready.value = 1
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_arvalid.value = 0
    dut.s_axi_bready.value = 0
    dut.s_axi_rready.value = 0
    dut.s_axi_wstrb.value = 0

    await _reset(dut)

    freq_norm = 0.0625
    phase_inc = int(freq_norm * (1 << ACC_WIDTH)) & ((1 << ACC_WIDTH) - 1)
    await _axi_write(dut, 0x0, phase_inc)

    num_samples = 128
    phase_acc = 0

    baseband_ch0 = np.exp(1j * 0.05 * np.arange(num_samples)) * 0.7
    baseband_ch1 = np.exp(1j * 0.09 * np.arange(num_samples)) * 0.6
    baseband = np.vstack([baseband_ch0, baseband_ch1])

    input_records = []
    expected_records = []
    output_records = []

    async def capture_outputs():
        received = 0
        while received < num_samples:
            await RisingEdge(dut.clk)
            if dut.m_axis_tvalid.value and dut.m_axis_tready.value:
                word = int(dut.m_axis_tdata.value)
                row = []
                for ch in range(CHANNELS):
                    base = ch * 2 * WIDTH
                    i_raw = (word >> base) & ((1 << WIDTH) - 1)
                    q_raw = (word >> (base + WIDTH)) & ((1 << WIDTH) - 1)
                    i_val = _signed_from_bits(i_raw, WIDTH)
                    q_val = _signed_from_bits(q_raw, WIDTH)
                    row.append(complex(i_val / SCALE, q_val / SCALE))
                output_records.append(row)
                if dut.m_axis_tlast.value:
                    assert received == num_samples - 1
                received += 1

    capture_task = cocotb.start_soon(capture_outputs())

    for idx in range(num_samples):
        cfo_angle = (phase_acc / float(1 << ACC_WIDTH)) * 2 * math.pi
        cfo_rot = complex(math.cos(cfo_angle), math.sin(cfo_angle))
        samples = baseband[:, idx] * cfo_rot
        i_vals = []
        q_vals = []
        for ch in range(CHANNELS):
            i_vals.append(_quantize(samples[ch].real))
            q_vals.append(_quantize(samples[ch].imag))
        packed = _pack_samples(i_vals, q_vals)

        dut.s_axis_tdata.value = packed
        dut.s_axis_tvalid.value = 1
        dut.s_axis_tlast.value = 1 if idx == num_samples - 1 else 0

        while True:
            await RisingEdge(dut.clk)
            if dut.s_axis_tready.value and dut.s_axis_tvalid.value:
                break

        dut.s_axis_tvalid.value = 0
        dut.s_axis_tlast.value = 0

        input_records.append([complex(i_vals[ch] / SCALE, q_vals[ch] / SCALE) for ch in range(CHANNELS)])

        lut_index = (phase_acc >> (ACC_WIDTH - LUT_ADDR_WIDTH)) & ((1 << LUT_ADDR_WIDTH) - 1)
        cos_val = COS_LUT[lut_index]
        sin_val = SIN_LUT[lut_index]

        expected_row = []
        for ch in range(CHANNELS):
            i_in = i_vals[ch]
            q_in = q_vals[ch]
            real_tmp = i_in * cos_val + q_in * sin_val
            imag_tmp = q_in * cos_val - i_in * sin_val
            real_fixed = real_tmp >> FRACTION_BITS
            imag_fixed = imag_tmp >> FRACTION_BITS
            max_val = (1 << (WIDTH - 1)) - 1
            min_val = -(1 << (WIDTH - 1))
            real_fixed = max(min(real_fixed, max_val), min_val)
            imag_fixed = max(min(imag_fixed, max_val), min_val)
            expected_row.append(complex(real_fixed / SCALE, imag_fixed / SCALE))
        expected_records.append(expected_row)

        phase_acc = (phase_acc + phase_inc) & ((1 << ACC_WIDTH) - 1)

    await capture_task

    assert len(output_records) == num_samples

    tol = 1.5 / SCALE
    for idx in range(num_samples):
        for ch in range(CHANNELS):
            exp_val = expected_records[idx][ch]
            got_val = output_records[idx][ch]
            assert abs(exp_val.real - got_val.real) <= tol
            assert abs(exp_val.imag - got_val.imag) <= tol

    plot_dir = (Path(__file__).resolve().parent / "plots")
    plot_dir.mkdir(parents=True, exist_ok=True)
    plot_path_real = plot_dir / "nco_cfo_compensation.png"
    plot_path_imag = plot_dir / "nco_cfo_compensation_imag.png"

    time_axis = np.arange(num_samples)

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for ch, ax in enumerate(axes):
        ax.plot(time_axis, [val[ch].real for val in input_records], label=f"Channel {ch} In-Phase", alpha=0.6)
        ax.plot(time_axis, [val[ch].real for val in output_records], label=f"Channel {ch} Out-Phase", linewidth=2)
        ax.set_ylabel("Real")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("Sample")
    fig.suptitle("NCO CFO Compensation (Real Component)")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(plot_path_real, dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for ch, ax in enumerate(axes):
        ax.plot(time_axis, [val[ch].imag for val in input_records], label=f"Channel {ch} In-Quadrature", alpha=0.6)
        ax.plot(time_axis, [val[ch].imag for val in output_records], label=f"Channel {ch} Out-Quadrature", linewidth=2)
        ax.set_ylabel("Imag")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("Sample")
    fig.suptitle("NCO CFO Compensation (Imag Component)")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(plot_path_imag, dpi=150)
    plt.close(fig)


@cocotb.test()
async def nco_cfo_dynamic_update(dut):
    clock = Clock(dut.clk, 5, units="ns")
    cocotb.start_soon(clock.start())

    dut.s_axis_tvalid.value = 0
    dut.s_axis_tlast.value = 0
    dut.m_axis_tready.value = 1
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_arvalid.value = 0
    dut.s_axi_bready.value = 0
    dut.s_axi_rready.value = 0
    dut.s_axi_wstrb.value = 0

    await _reset(dut)

    freq_init = 0.05
    freq_updated = -0.08
    phase_inc_init = int(freq_init * (1 << ACC_WIDTH)) & ((1 << ACC_WIDTH) - 1)
    phase_inc_updated = int(freq_updated * (1 << ACC_WIDTH)) & ((1 << ACC_WIDTH) - 1)

    await _axi_write(dut, 0x0, phase_inc_init)

    num_samples = 160
    change_idx = num_samples // 2
    phase_acc = 0
    current_inc = phase_inc_init

    baseband_ch0 = np.full(num_samples, 0.75 + 0.1j, dtype=np.complex128)
    baseband_ch1 = np.full(num_samples, 0.55 - 0.25j, dtype=np.complex128)
    baseband = np.vstack([baseband_ch0, baseband_ch1])

    input_records = []
    expected_records = []
    output_records = []

    async def capture_outputs():
        received = 0
        while received < num_samples:
            await RisingEdge(dut.clk)
            if dut.m_axis_tvalid.value and dut.m_axis_tready.value:
                word = int(dut.m_axis_tdata.value)
                row = []
                for ch in range(CHANNELS):
                    base = ch * 2 * WIDTH
                    i_raw = (word >> base) & ((1 << WIDTH) - 1)
                    q_raw = (word >> (base + WIDTH)) & ((1 << WIDTH) - 1)
                    i_val = _signed_from_bits(i_raw, WIDTH)
                    q_val = _signed_from_bits(q_raw, WIDTH)
                    row.append(complex(i_val / SCALE, q_val / SCALE))
                output_records.append(row)
                if dut.m_axis_tlast.value:
                    assert received == num_samples - 1
                received += 1

    capture_task = cocotb.start_soon(capture_outputs())

    for idx in range(num_samples):
        if idx == change_idx:
            await _axi_write(dut, 0x0, phase_inc_updated)
            current_inc = phase_inc_updated

        cfo_angle = (phase_acc / float(1 << ACC_WIDTH)) * 2 * math.pi
        cfo_rot = complex(math.cos(cfo_angle), math.sin(cfo_angle))
        samples = baseband[:, idx] * cfo_rot
        i_vals = []
        q_vals = []
        for ch in range(CHANNELS):
            i_vals.append(_quantize(samples[ch].real))
            q_vals.append(_quantize(samples[ch].imag))
        packed = _pack_samples(i_vals, q_vals)

        dut.s_axis_tdata.value = packed
        dut.s_axis_tvalid.value = 1
        dut.s_axis_tlast.value = 1 if idx == num_samples - 1 else 0

        while True:
            await RisingEdge(dut.clk)
            if dut.s_axis_tready.value and dut.s_axis_tvalid.value:
                break

        dut.s_axis_tvalid.value = 0
        dut.s_axis_tlast.value = 0

        input_records.append([complex(i_vals[ch] / SCALE, q_vals[ch] / SCALE) for ch in range(CHANNELS)])

        lut_index = (phase_acc >> (ACC_WIDTH - LUT_ADDR_WIDTH)) & ((1 << LUT_ADDR_WIDTH) - 1)
        cos_val = COS_LUT[lut_index]
        sin_val = SIN_LUT[lut_index]

        expected_row = []
        for ch in range(CHANNELS):
            i_in = i_vals[ch]
            q_in = q_vals[ch]
            real_tmp = i_in * cos_val + q_in * sin_val
            imag_tmp = q_in * cos_val - i_in * sin_val
            real_fixed = real_tmp >> FRACTION_BITS
            imag_fixed = imag_tmp >> FRACTION_BITS
            max_val = (1 << (WIDTH - 1)) - 1
            min_val = -(1 << (WIDTH - 1))
            real_fixed = max(min(real_fixed, max_val), min_val)
            imag_fixed = max(min(imag_fixed, max_val), min_val)
            expected_row.append(complex(real_fixed / SCALE, imag_fixed / SCALE))
        expected_records.append(expected_row)

        phase_acc = (phase_acc + current_inc) & ((1 << ACC_WIDTH) - 1)

    await capture_task

    assert len(output_records) == num_samples

    tol = 1.5 / SCALE
    for idx in range(num_samples):
        for ch in range(CHANNELS):
            exp_val = expected_records[idx][ch]
            got_val = output_records[idx][ch]
            assert abs(exp_val.real - got_val.real) <= tol
            assert abs(exp_val.imag - got_val.imag) <= tol

    plot_dir = (Path(__file__).resolve().parent / "plots")
    plot_dir.mkdir(parents=True, exist_ok=True)
    plot_real = plot_dir / "nco_cfo_dynamic_real.png"
    plot_imag = plot_dir / "nco_cfo_dynamic_imag.png"

    time_axis = np.arange(num_samples)

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for ch, ax in enumerate(axes):
        ax.plot(time_axis, [val[ch].real for val in input_records], label=f"Channel {ch} Input", alpha=0.6)
        ax.plot(time_axis, [val[ch].real for val in output_records], label=f"Channel {ch} Output", linewidth=2)
        ax.axvline(change_idx - 0.5, color="k", linestyle="--", alpha=0.4, label="CFO update" if ch == 0 else None)
        ax.set_ylabel("Real")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("Sample")
    fig.suptitle("On-the-fly CFO Update (Real Component)")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(plot_real, dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for ch, ax in enumerate(axes):
        ax.plot(time_axis, [val[ch].imag for val in input_records], label=f"Channel {ch} Input", alpha=0.6)
        ax.plot(time_axis, [val[ch].imag for val in output_records], label=f"Channel {ch} Output", linewidth=2)
        ax.axvline(change_idx - 0.5, color="k", linestyle="--", alpha=0.4, label="CFO update" if ch == 0 else None)
        ax.set_ylabel("Imag")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("Sample")
    fig.suptitle("On-the-fly CFO Update (Imag Component)")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(plot_imag, dpi=150)
    plt.close(fig)


@pytest.mark.skipif(VERILATOR is None, reason="Verilator executable not found; install Verilator to run this test.")
def test_nco_cfo_compensator():
    rtl_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "rtl"))
    rtl_file = os.path.join(rtl_dir, "nco_cfo_compensator.sv")
    build_dir = os.path.join("tests", "sim_build", "nco_cfo_compensator")

    run(
        verilog_sources=[rtl_file],
        toplevel="nco_cfo_compensator",
        toplevel_lang="verilog",
        module=os.path.splitext(os.path.basename(__file__))[0],
        parameters={},
        sim_build=build_dir,
        simulator="verilator",
        extra_env={
            "COCOTB_RESULTS_FILE": os.path.join(build_dir, "results.xml"),
        },
    )
