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


SCENARIOS = {
    "pre_sof_suppression": "Samples prior to the first SOF pulse must be ignored.",
    "payload_forwarding": "Each payload sample within the FFT window propagates to the output.",
    "symbol_indexing": "The symbol counter should advance per symbol and wrap each frame.",
    "frame_last_flag": "m_axis_tlast must only assert on the final payload sample of a frame.",
    "positive_sto_expansion": "Positive STO corrections extend the inter-frame gap when tracking.",
    "negative_sto_shrink": "Negative STO corrections shorten the gap without corrupting payload timing.",
    "sto_ready_gating": "sto_ready is asserted only during the tracking window.",
    "sto_clamp_positive_limit": "Large positive STO commands clamp to the configured maximum.",
    "sto_clamp_zero_limit": "Strong negative STO commands clamp to zero for immediate realignment.",
    "output_fifo_backpressure": "Downstream stalls are absorbed without dropping payload data.",
    "no_output_without_sof": "Without an SOF pulse the synchronizer must remain idle.",
}

SCENARIO_COVERAGE = {
    "synchronizer_basic_frame_forwarding": {
        "pre_sof_suppression",
        "payload_forwarding",
        "symbol_indexing",
        "frame_last_flag",
    },
    "synchronizer_tracking_and_sto": {
        "positive_sto_expansion",
        "negative_sto_shrink",
        "sto_ready_gating",
    },
    "synchronizer_sto_clamping": {
        "sto_clamp_positive_limit",
        "sto_clamp_zero_limit",
        "payload_forwarding",
    },
    "synchronizer_backpressure": {
        "output_fifo_backpressure",
        "payload_forwarding",
    },
    "synchronizer_no_sof_no_output": {
        "no_output_without_sof",
    },
}


def _pack_sample(value: int, width: int) -> int:
    mask = (1 << width) - 1
    packed = 0
    for lane in range(4):
        packed |= ((value + lane) & mask) << (lane * width)
    return packed


def _build_frame_samples(
    *,
    start_value: int,
    nfft: int,
    cp_len: int,
    n_symbols: int,
    width: int,
    sof_on_first: bool,
    first_sample_sto: int | None = None,
) -> tuple[List[AxisSample], int]:
    """Materialize one frame worth of stimulus including the cyclic prefix."""
    samples: List[AxisSample] = []
    value = start_value
    sentinel = _pack_sample(1 << (width - 1), width)
    for sym in range(n_symbols):
        for sample in range(nfft):
            sof = 1 if (sof_on_first and sym == 0 and sample == 0) else 0
            sto_valid = 1 if (first_sample_sto is not None and sym == 0 and sample == 0) else 0
            sto_value = first_sample_sto if sto_valid else 0
            samples.append(
                AxisSample(
                    data=_pack_sample(value, width),
                    sof=sof,
                    sto_valid=sto_valid,
                    sto_value=sto_value,
                )
            )
            value += 1
        for _ in range(cp_len):
            samples.append(AxisSample(data=sentinel))
    return samples, value


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

    outputs_before_sof = 0

    for idx, sample in enumerate(stream):
        dut.s_axis_tdata.value = sample.data
        dut.s_axis_tuser.value = sample.sof
        dut.s_axis_tvalid.value = 1
        dut.s_axis_tlast.value = 0

        await RisingEdge(dut.clk)

        if idx == 0:
            outputs_before_sof += int(dut.m_axis_tvalid.value)

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

    assert outputs_before_sof == 0

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
    gap_extra = 0
    gap_shrink = 0

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

    payload_per_frame = nfft * n_symbols
    frames_total = frames
    expected_payload_words = [_pack_sample(idx, width) for idx in range(frames_total * payload_per_frame)]

    captured_data: List[int] = []
    capture_times: List[int] = []
    sto_ready_history: List[tuple[int, int]] = []
    sto_valid_cycles: List[int] = []
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

        ready_val = int(dut.sto_ready.value)
        sto_ready_history.append((cycle, ready_val))

        if sample.sto_valid:
            sto_valid_cycles.append(cycle)
            assert ready_val == 1

        if dut.m_axis_tvalid.value:
            captured_data.append(int(dut.m_axis_tdata.value))
            capture_times.append(cycle)

    dut.s_axis_tvalid.value = 0
    dut.sto_valid.value = 0
    dut.sto_correction.value = 0
    dut.s_axis_tlast.value = 0
    for _ in range(10):
        await RisingEdge(dut.clk)
        cycle += 1
        ready_val = int(dut.sto_ready.value)
        sto_ready_history.append((cycle, ready_val))
        if dut.m_axis_tvalid.value:
            captured_data.append(int(dut.m_axis_tdata.value))
            capture_times.append(cycle)

    assert len(captured_data) == frames_total * payload_per_frame
    assert captured_data == expected_payload_words

    assert sto_valid_cycles, "Expected at least one STO command during tracking."
    first_sto_cycle = sto_valid_cycles[0]
    assert all(val == 0 for cyc, val in sto_ready_history if cyc < first_sto_cycle)

    frame_gap = int(dut.FRAME_GAP_SAMPLES.value)

    gap_after_frame0 = capture_times[payload_per_frame] - capture_times[payload_per_frame - 1]
    expected_gap0 = cp_len + (frame_gap + gap_extra) + 1
    assert gap_after_frame0 == expected_gap0

    gap_after_frame1 = capture_times[2 * payload_per_frame] - capture_times[2 * payload_per_frame - 1]
    expected_gap1 = cp_len + (frame_gap - gap_shrink) + 1
    assert gap_after_frame1 == expected_gap1

    first_sample_frame1_cycle = capture_times[payload_per_frame]
    assert any(
        val == 0
        for cyc, val in sto_ready_history
        if cyc >= first_sample_frame1_cycle
    ), "sto_ready should drop once frame collection resumes."


@cocotb.test()
async def synchronizer_no_sof_no_output(dut):
    """Verify that the synchronizer stays idle without a start-of-frame pulse."""

    clock = Clock(dut.clk, 5, units="ns")
    cocotb.start_soon(clock.start())

    await _reset_dut(dut)

    width = int(dut.INPUT_WIDTH.value)

    dut.m_axis_tready.value = 1

    outputs_seen = 0

    for value in range(3 * int(dut.NFFT.value)):
        dut.s_axis_tdata.value = _pack_sample(value, width)
        dut.s_axis_tuser.value = 0
        dut.s_axis_tvalid.value = 1
        dut.s_axis_tlast.value = 0

        await RisingEdge(dut.clk)

        outputs_seen += int(dut.m_axis_tvalid.value)

    dut.s_axis_tvalid.value = 0
    for _ in range(5):
        await RisingEdge(dut.clk)
        outputs_seen += int(dut.m_axis_tvalid.value)

    assert outputs_seen == 0
    assert int(dut.sto_ready.value) == 0


@cocotb.test()
async def synchronizer_sto_clamping(dut):
    """Validate STO saturation at both the positive and zero limits."""

    clock = Clock(dut.clk, 4, units="ns")
    cocotb.start_soon(clock.start())

    await _reset_dut(dut)

    nfft = int(dut.NFFT.value)
    cp_len = int(dut.CP_LEN.value)
    n_symbols = int(dut.N_FRAME_SYMBOLS.value)
    width = int(dut.INPUT_WIDTH.value)
    frame_gap = int(dut.FRAME_GAP_SAMPLES.value)
    sto_acc_width = int(dut.STO_ACC_WIDTH.value)
    sto_max = (1 << (sto_acc_width - 1)) - 1

    dut.m_axis_tready.value = 1

    stream: List[AxisSample] = []
    value = 0

    frame0, value = _build_frame_samples(
        start_value=value,
        nfft=nfft,
        cp_len=cp_len,
        n_symbols=n_symbols,
        width=width,
        sof_on_first=True,
    )
    stream.extend(frame0)

    pos_correction = sto_max + 12
    pos_gap_cycles = min(sto_max, frame_gap + pos_correction)
    for idx in range(pos_gap_cycles):
        sto_valid = 1 if idx == 0 else 0
        sto_value = pos_correction if idx == 0 else 0
        stream.append(AxisSample(data=_pack_sample(0, width), sto_valid=sto_valid, sto_value=sto_value))

    frame1, value = _build_frame_samples(
        start_value=value,
        nfft=nfft,
        cp_len=cp_len,
        n_symbols=n_symbols,
        width=width,
        sof_on_first=False,
    )
    stream.extend(frame1)

    neg_correction = -(frame_gap + 12)
    frame2, value = _build_frame_samples(
        start_value=value,
        nfft=nfft,
        cp_len=cp_len,
        n_symbols=n_symbols,
        width=width,
        sof_on_first=False,
        first_sample_sto=neg_correction,
    )
    stream.extend(frame2)

    payload_per_frame = nfft * n_symbols
    frames_total = 3
    expected_payload_words = [_pack_sample(idx, width) for idx in range(frames_total * payload_per_frame)]

    captured_data: List[int] = []
    captured_times: List[int] = []
    sto_ready_history: List[tuple[int, int]] = []
    sto_valid_cycles: List[int] = []
    cycle = 0

    for sample in stream:
        dut.s_axis_tdata.value = sample.data
        dut.s_axis_tuser.value = sample.sof
        dut.s_axis_tvalid.value = 1
        dut.s_axis_tlast.value = 0
        dut.sto_valid.value = sample.sto_valid
        dut.sto_correction.value = sample.sto_value & 0xFF

        await RisingEdge(dut.clk)
        cycle += 1

        ready_val = int(dut.sto_ready.value)
        sto_ready_history.append((cycle, ready_val))

        if sample.sto_valid:
            sto_valid_cycles.append(cycle)
            assert ready_val == 1

        if dut.m_axis_tvalid.value:
            captured_data.append(int(dut.m_axis_tdata.value))
            captured_times.append(cycle)

    dut.s_axis_tvalid.value = 0
    dut.sto_valid.value = 0
    dut.sto_correction.value = 0
    for _ in range(8):
        await RisingEdge(dut.clk)
        cycle += 1
        ready_val = int(dut.sto_ready.value)
        sto_ready_history.append((cycle, ready_val))
        if dut.m_axis_tvalid.value:
            captured_data.append(int(dut.m_axis_tdata.value))
            captured_times.append(cycle)

    assert len(captured_data) == frames_total * payload_per_frame
    assert captured_data == expected_payload_words

    assert sto_valid_cycles, "Expected STO commands to fire in this scenario."
    first_pos_sto = sto_valid_cycles[0]
    assert all(val == 0 for cyc, val in sto_ready_history if cyc < first_pos_sto)

    gap_between_f0_f1 = captured_times[payload_per_frame] - captured_times[payload_per_frame - 1]
    expected_gap_f0_f1 = cp_len + pos_gap_cycles + 1
    assert gap_between_f0_f1 == expected_gap_f0_f1

    gap_between_f1_f2 = captured_times[2 * payload_per_frame] - captured_times[2 * payload_per_frame - 1]
    expected_gap_f1_f2 = cp_len + 1
    assert gap_between_f1_f2 == expected_gap_f1_f2

    # The zero-gap frame should cause sto_ready to drop immediately after the first payload beat.
    first_sample_frame2_cycle = captured_times[2 * payload_per_frame]
    assert any(
        (cyc > first_sample_frame2_cycle and val == 0)
        for cyc, val in sto_ready_history
    )


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


def test_synchronizer_scenario_map_complete():
    covered = set().union(*SCENARIO_COVERAGE.values()) if SCENARIO_COVERAGE else set()
    assert covered == set(SCENARIOS.keys())


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
