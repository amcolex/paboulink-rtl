import os
import shutil
from dataclasses import dataclass
from typing import List

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge
from cocotb_test.simulator import run
import pytest


VERILATOR = shutil.which("verilator")


@dataclass
class AxisSample:
    data: int
    sof: int = 0
    sto_valid: int = 0
    sto_value: int = 0


def _pack_sample(value: int, width: int) -> int:
    mask = (1 << width) - 1
    packed = 0
    for lane in range(4):
        packed |= ((value + lane) & mask) << (lane * width)
    return packed


async def _reset_dut(dut) -> None:
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tuser.value = 0
    dut.s_axis_tlast.value = 0
    dut.sto_valid.value = 0
    dut.sto_correction.value = 0
    dut.m_axis_tready.value = 0
    dut.rst.value = 1
    for _ in range(5):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)


@cocotb.test()
async def synchronizer_basic_frame_forwarding(dut):
    """Verify that payload samples propagate with the expected symbol indices."""

    clock = Clock(dut.clk, 4, units="ns")
    cocotb.start_soon(clock.start())

    await _reset_dut(dut)

    nfft = int(dut.NFFT.value)
    cp_len = int(dut.CP_LEN.value)
    n_symbols = int(dut.N_FRAME_SYMBOLS.value)
    width = int(dut.INPUT_WIDTH.value)

    dut.m_axis_tready.value = 1

    stream: List[AxisSample] = []
    value = 0

    # One dummy cycle prior to the SOF pulse.
    stream.append(AxisSample(data=_pack_sample(value, width)))
    value += 1

    for sym in range(n_symbols):
        for sample in range(nfft):
            sof = 1 if (sym == 0 and sample == 0) else 0
            stream.append(AxisSample(data=_pack_sample(value, width), sof=sof))
            value += 1
        for _ in range(cp_len):
            # Use a sentinel value to ensure CP samples are not forwarded.
            stream.append(AxisSample(data=_pack_sample(1 << (width - 1), width)))

    captured_data: List[int] = []
    captured_users: List[int] = []
    captured_last: List[int] = []

    for sample in stream:
        dut.s_axis_tdata.value = sample.data
        dut.s_axis_tuser.value = sample.sof
        dut.s_axis_tvalid.value = 1
        dut.s_axis_tlast.value = 0

        await RisingEdge(dut.clk)

        if dut.m_axis_tvalid.value:
            captured_data.append(int(dut.m_axis_tdata.value))
            captured_users.append(int(dut.m_axis_tuser.value))
            captured_last.append(int(dut.m_axis_tlast.value))

    dut.s_axis_tvalid.value = 0
    dut.s_axis_tlast.value = 0
    for _ in range(5):
        await RisingEdge(dut.clk)
        if dut.m_axis_tvalid.value:
            captured_data.append(int(dut.m_axis_tdata.value))
            captured_users.append(int(dut.m_axis_tuser.value))
            captured_last.append(int(dut.m_axis_tlast.value))

    assert len(captured_data) == nfft * n_symbols

    expected_words = [_pack_sample(idx, width) for idx in range(1, 1 + nfft * n_symbols)]
    assert captured_data == expected_words

    # Symbol index should remain constant during each payload block and wrap to zero for the new frame.
    expected_users = []
    for sym in range(n_symbols):
        expected_users.extend([sym] * nfft)
    assert captured_users == expected_users

    assert captured_last.count(1) == 1
    assert captured_last[-1] == 1


@cocotb.test()
async def synchronizer_tracking_and_sto(dut):
    """Exercise the tracking gap and STO corrections across multiple frames."""

    clock = Clock(dut.clk, 6, units="ns")
    cocotb.start_soon(clock.start())

    await _reset_dut(dut)

    nfft = int(dut.NFFT.value)
    cp_len = int(dut.CP_LEN.value)
    n_symbols = int(dut.N_FRAME_SYMBOLS.value)
    width = int(dut.INPUT_WIDTH.value)
    frame_gap = int(dut.FRAME_GAP_SAMPLES.value)

    dut.m_axis_tready.value = 1

    frames = 3
    stream: List[AxisSample] = []
    value = 0

    for frame_idx in range(frames):
        for sym in range(n_symbols):
            for sample in range(nfft):
                sof = 1 if (frame_idx == 0 and sym == 0 and sample == 0) else 0
                stream.append(AxisSample(data=_pack_sample(value, width), sof=sof))
                value += 1
            for _ in range(cp_len):
                stream.append(AxisSample(data=_pack_sample(1 << (width - 1), width)))

        if frame_idx == 0:
            gap_extra = 2
            for idx in range(frame_gap + gap_extra):
                sto_valid = 1 if idx == 0 else 0
                sto_value = 2 if idx == 0 else 0
                stream.append(AxisSample(data=_pack_sample(0, width), sto_valid=sto_valid, sto_value=sto_value))
        elif frame_idx == 1:
            gap_shrink = 1
            for idx in range(frame_gap - gap_shrink):
                sto_valid = 1 if idx == 0 else 0
                sto_value = -gap_shrink if idx == 0 else 0
                stream.append(AxisSample(data=_pack_sample(0, width), sto_valid=sto_valid, sto_value=sto_value))

    expected_payload_words = [_pack_sample(idx, width) for idx in range(frames * n_symbols * nfft)]

    captured_data: List[int] = []
    cycle = 0

    for sample in stream:
        dut.s_axis_tdata.value = sample.data
        dut.s_axis_tuser.value = sample.sof
        dut.s_axis_tvalid.value = 1
        dut.sto_valid.value = sample.sto_valid
        dut.sto_correction.value = sample.sto_value & 0xFF
        dut.s_axis_tlast.value = 0

        await RisingEdge(dut.clk)
        cycle += 1

        if sample.sto_valid:
            assert int(dut.sto_ready.value) == 1

        if dut.m_axis_tvalid.value:
            captured_data.append(int(dut.m_axis_tdata.value))

    dut.s_axis_tvalid.value = 0
    dut.sto_valid.value = 0
    dut.s_axis_tlast.value = 0
    for _ in range(10):
        await RisingEdge(dut.clk)
        cycle += 1
        if dut.m_axis_tvalid.value:
            captured_data.append(int(dut.m_axis_tdata.value))

    assert captured_data == expected_payload_words


@cocotb.test()
async def synchronizer_backpressure(dut):
    """Confirm that short bursts of back-pressure are absorbed by the FIFO."""

    clock = Clock(dut.clk, 5, units="ns")
    cocotb.start_soon(clock.start())

    await _reset_dut(dut)

    nfft = int(dut.NFFT.value)
    cp_len = int(dut.CP_LEN.value)
    n_symbols = int(dut.N_FRAME_SYMBOLS.value)
    width = int(dut.INPUT_WIDTH.value)

    total_payload = nfft * n_symbols

    ready_pattern = [1, 0, 1, 1]
    pattern_idx = 0

    captured = []
    expected = []

    value = 0

    for cycle in range(total_payload + cp_len * n_symbols + 20):
        if cycle < total_payload + cp_len * n_symbols:
            sample_idx = cycle % (nfft + cp_len)
            sof = 1 if (cycle == 0) else 0
            data = _pack_sample(value, width)
            if sample_idx < nfft:
                expected.append(data)
            value += 1
            dut.s_axis_tdata.value = data
            dut.s_axis_tuser.value = sof
            dut.s_axis_tvalid.value = 1
            dut.s_axis_tlast.value = 0
        else:
            dut.s_axis_tvalid.value = 0
            dut.s_axis_tlast.value = 0

        dut.m_axis_tready.value = ready_pattern[pattern_idx]
        pattern_idx = (pattern_idx + 1) % len(ready_pattern)

        await RisingEdge(dut.clk)

        if dut.m_axis_tvalid.value and dut.m_axis_tready.value:
            captured.append(int(dut.m_axis_tdata.value))

    assert captured == expected


@pytest.mark.parametrize(
    "parameters",
    [
        {
            "INPUT_WIDTH": 12,
            "NFFT": 8,
            "CP_LEN": 2,
            "N_FRAME_SYMBOLS": 3,
            "FRAME_GAP_SAMPLES": 6,
            "SYMBOL_COUNTER_WIDTH": 3,
            "STO_ACC_WIDTH": 6,
        }
    ],
)
def test_ofdm_frame_synchronizer(parameters):
    if VERILATOR is None:
        pytest.skip("Verilator executable not found; install Verilator to run this test.")

    rtl_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "rtl"))
    rtl_file = os.path.join(rtl_dir, "ofdm_frame_synchronizer.sv")
    build_dir = os.path.join("tests", "sim_build", "ofdm_frame_synchronizer")

    run(
        verilog_sources=[rtl_file],
        toplevel="ofdm_frame_synchronizer",
        toplevel_lang="verilog",
        module=os.path.splitext(os.path.basename(__file__))[0],
        parameters=parameters,
        sim_build=build_dir,
        simulator="verilator",
        extra_env={
            "COCOTB_RESULTS_FILE": os.path.join(build_dir, "results.xml"),
        },
    )
