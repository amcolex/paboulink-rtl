import os
import random
import shutil

import pytest
from cocotb.triggers import Timer
import cocotb
from cocotb_test.simulator import run

VERILATOR = shutil.which("verilator")


@cocotb.test()
async def conjugate_matches_expected(dut):
    """Feed random IQ samples and check that the conjugate output is correct."""
    width = dut.i_real.value.n_bits
    min_val = -(1 << (width - 1))
    max_val = (1 << (width - 1)) - 1

    rng = random.Random(0)
    for _ in range(20):
        i_real = rng.randint(min_val, max_val)
        i_imag = rng.randint(min_val, max_val)

        dut.i_real.value = i_real
        dut.i_imag.value = i_imag
        await Timer(1, units="ns")

        assert dut.o_real.value.signed_integer == i_real
        assert dut.o_imag.value.signed_integer == -i_imag


@pytest.mark.parametrize("width", [12, 16])
def test_complex_conjugate(width):
    rtl_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "rtl"))
    rtl_file = os.path.join(rtl_dir, "complex_conjugate.sv")
    build_dir = os.path.join("tests", "sim_build", f"complex_conjugate_{width}")

    if VERILATOR is None:
        pytest.skip("Verilator executable not found; install Verilator to run this test.")

    run(
        verilog_sources=[rtl_file],
        toplevel="complex_conjugate",
        toplevel_lang="verilog",
        module=os.path.splitext(os.path.basename(__file__))[0],
        parameters={"WIDTH": width},
        sim_build=build_dir,
        simulator="verilator",
        extra_env={
            "COCOTB_RESULTS_FILE": os.path.join(build_dir, "results.xml"),
        },
    )
