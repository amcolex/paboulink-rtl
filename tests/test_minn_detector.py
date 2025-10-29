"""
cocotb testbench for Minn preamble detector.
Tests detection with OFDM preambles and generates diagnostic plots.
"""

import os
import shutil
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

import pytest
import cocotb
from cocotb.triggers import RisingEdge, Timer
from cocotb.clock import Clock
from cocotb_test.simulator import run

from ofdm_utils import quantize_iq, dequantize_iq, add_awgn
from minn_preamble import generate_dual_antenna_preamble, verify_minn_structure

VERILATOR = shutil.which("verilator")

# Test parameters
NFFT = 2048
CP_LENGTH = 512
NUM_ACTIVE = 1200
SPACING = 4
W_IN = 12
QUANTIZE_SCALE = 2000  # Scale factor for 12-bit quantization


async def reset_dut(dut):
    """Reset the DUT."""
    dut.rst.value = 1
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)


async def send_samples(dut, ch0_i, ch0_q, ch1_i, ch1_q):
    """
    Send quantized samples to the detector.
    
    Args:
        dut: Device under test
        ch0_i, ch0_q: Antenna 0 I/Q samples (integer arrays)
        ch1_i, ch1_q: Antenna 1 I/Q samples (integer arrays)
    """
    num_samples = len(ch0_i)
    
    for i in range(num_samples):
        dut.in_valid.value = 1
        dut.ch0_i.value = int(ch0_i[i])
        dut.ch0_q.value = int(ch0_q[i])
        dut.ch1_i.value = int(ch1_i[i])
        dut.ch1_q.value = int(ch1_q[i])
        await RisingEdge(dut.clk)
    
    # De-assert valid after sending
    dut.in_valid.value = 0


async def collect_outputs(dut, num_samples):
    """
    Collect outputs from the detector.
    
    Returns:
        Dictionary with output arrays: out_i/q, frame_start flags, metric, energy
    """
    results = {
        'out_ch0_i': [],
        'out_ch0_q': [],
        'out_ch1_i': [],
        'out_ch1_q': [],
        'frame_start': [],
        'metric': [],
        'energy': [],
        'above_thresh': [],
        'out_valid': []
    }
    
    for _ in range(num_samples):
        await RisingEdge(dut.clk)
        
        # Read outputs
        results['out_valid'].append(int(dut.out_valid.value))
        
        if int(dut.out_valid.value) == 1:
            results['out_ch0_i'].append(int(dut.out_ch0_i.value.signed_integer))
            results['out_ch0_q'].append(int(dut.out_ch0_q.value.signed_integer))
            results['out_ch1_i'].append(int(dut.out_ch1_i.value.signed_integer))
            results['out_ch1_q'].append(int(dut.out_ch1_q.value.signed_integer))
            results['frame_start'].append(int(dut.frame_start.value))
        else:
            results['out_ch0_i'].append(0)
            results['out_ch0_q'].append(0)
            results['out_ch1_i'].append(0)
            results['out_ch1_q'].append(0)
            results['frame_start'].append(0)
        
        # Read debug signals
        results['metric'].append(int(dut.dbg_metric.value))
        results['energy'].append(int(dut.dbg_energy.value))
        results['above_thresh'].append(int(dut.dbg_above_thresh.value))
    
    # Convert to numpy arrays
    for key in results:
        results[key] = np.array(results[key])
    
    return results


def plot_detector_results(results, preamble_start_idx, test_name="minn_detector"):
    """
    Generate diagnostic plots for the detector results.
    
    Args:
        results: Dictionary of output arrays from collect_outputs()
        preamble_start_idx: Sample index where preamble starts
        test_name: Name for the plot files
    """
    # Create plots directory
    plot_dir = Path("tests/plots")
    plot_dir.mkdir(parents=True, exist_ok=True)
    
    num_samples = len(results['metric'])
    time_axis = np.arange(num_samples)
    
    # Figure 1: Metric and Energy over time
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    # Metric
    axes[0].plot(time_axis, results['metric'], label='C_smooth (Metric)', linewidth=1)
    axes[0].axvline(x=preamble_start_idx, color='r', linestyle='--', label='Preamble Start', alpha=0.5)
    # Mark detections
    detections = np.where(results['frame_start'] == 1)[0]
    if len(detections) > 0:
        axes[0].scatter(detections, results['metric'][detections], 
                       color='green', marker='*', s=200, zorder=5, label='Detection')
    axes[0].set_ylabel('Metric Value')
    axes[0].set_title('Minn Detector Metric (Correlation)')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    
    # Energy
    axes[1].plot(time_axis, results['energy'], label='E_total (Energy)', linewidth=1, color='orange')
    axes[1].axvline(x=preamble_start_idx, color='r', linestyle='--', label='Preamble Start', alpha=0.5)
    axes[1].set_ylabel('Energy Value')
    axes[1].set_title('Total Signal Energy (3 Quarters, 2 Antennas)')
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    
    # Above threshold indicator
    axes[2].plot(time_axis, results['above_thresh'], label='Above Threshold', linewidth=1.5, color='purple')
    axes[2].fill_between(time_axis, 0, results['above_thresh'], alpha=0.3, color='purple')
    axes[2].axvline(x=preamble_start_idx, color='r', linestyle='--', label='Preamble Start', alpha=0.5)
    if len(detections) > 0:
        for det_idx in detections:
            axes[2].axvline(x=det_idx, color='green', linestyle=':', alpha=0.7)
    axes[2].set_ylabel('Gate State')
    axes[2].set_xlabel('Sample Index')
    axes[2].set_title('Threshold Comparison (Gate State)')
    axes[2].grid(True, alpha=0.3)
    axes[2].legend()
    axes[2].set_ylim([-0.1, 1.1])
    
    plt.tight_layout()
    plt.savefig(plot_dir / f"{test_name}_metrics.png", dpi=150)
    plt.close()
    
    # Figure 2: Output I/Q time series
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    # Channel 0
    valid_mask = results['out_valid'] == 1
    valid_indices = np.where(valid_mask)[0]
    
    if len(valid_indices) > 0:
        axes[0].plot(valid_indices, results['out_ch0_i'][valid_mask], 
                    label='I', alpha=0.7, linewidth=0.8)
        axes[0].plot(valid_indices, results['out_ch0_q'][valid_mask], 
                    label='Q', alpha=0.7, linewidth=0.8)
        
        # Mark frame starts
        frame_starts = np.where((results['frame_start'] == 1) & valid_mask)[0]
        if len(frame_starts) > 0:
            for fs in frame_starts:
                axes[0].axvline(x=fs, color='red', linestyle='--', alpha=0.5)
        
        axes[0].set_ylabel('Amplitude')
        axes[0].set_title('Output Channel 0 (I/Q)')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        
        # Channel 1
        axes[1].plot(valid_indices, results['out_ch1_i'][valid_mask], 
                    label='I', alpha=0.7, linewidth=0.8)
        axes[1].plot(valid_indices, results['out_ch1_q'][valid_mask], 
                    label='Q', alpha=0.7, linewidth=0.8)
        
        if len(frame_starts) > 0:
            for fs in frame_starts:
                axes[1].axvline(x=fs, color='red', linestyle='--', alpha=0.5)
        
        axes[1].set_ylabel('Amplitude')
        axes[1].set_xlabel('Sample Index')
        axes[1].set_title('Output Channel 1 (I/Q)')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
    
    plt.tight_layout()
    plt.savefig(plot_dir / f"{test_name}_outputs.png", dpi=150)
    plt.close()
    
    print(f"\nPlots saved to {plot_dir}/")
    print(f"  - {test_name}_metrics.png")
    print(f"  - {test_name}_outputs.png")


@cocotb.test()
async def test_perfect_preamble_detection(dut):
    """Test detection with perfect (noiseless) preamble."""
    
    # Start clock
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    
    # Reset
    await reset_dut(dut)
    
    # Generate preamble for both antennas (use same seed for correlation)
    print("\nGenerating Minn preamble...")
    ch0_preamble, ch1_preamble, _, _ = generate_dual_antenna_preamble(
        nfft=NFFT, 
        cp_length=CP_LENGTH,
        num_active=NUM_ACTIVE,
        spacing=SPACING,
        antenna_seed=42  # Same seed = correlated antennas
    )
    
    # Verify structure
    corr_01, corr_23, corr_02 = verify_minn_structure(ch0_preamble, NFFT, CP_LENGTH)
    print(f"Preamble structure verification:")
    print(f"  Q0-Q1 correlation: {corr_01:.4f}")
    print(f"  Q2-Q3 correlation: {corr_23:.4f}")
    print(f"  Q0-Q2 correlation: {corr_02:.4f} (should be negative)")
    
    # Add some leading zeros for initial conditions
    num_leading = 100
    leading_zeros = np.zeros(num_leading, dtype=complex)
    
    # Concatenate: leading zeros + preamble
    ch0_full = np.concatenate([leading_zeros, ch0_preamble])
    ch1_full = np.concatenate([leading_zeros, ch1_preamble])
    
    # Quantize to fixed-point
    ch0_i_int, ch0_q_int = quantize_iq(ch0_full, W_IN, QUANTIZE_SCALE)
    ch1_i_int, ch1_q_int = quantize_iq(ch1_full, W_IN, QUANTIZE_SCALE)
    
    print(f"\nSending {len(ch0_i_int)} samples to detector...")
    print(f"  Preamble starts at sample {num_leading}")
    
    # Send samples in background
    send_task = cocotb.start_soon(send_samples(dut, ch0_i_int, ch0_q_int, ch1_i_int, ch1_q_int))
    
    # Collect outputs (need extra samples for output delay)
    total_collection_samples = len(ch0_i_int) + 2500
    results = await collect_outputs(dut, total_collection_samples)
    
    # Wait for send to complete
    await send_task
    
    # Analyze results
    detections = np.where(results['frame_start'] == 1)[0]
    print(f"\nDetections found: {len(detections)}")
    if len(detections) > 0:
        print(f"  Detection indices: {detections}")
        print(f"  Expected around: {num_leading + NFFT}")  # After output delay
    
    # Generate plots
    plot_detector_results(results, num_leading, "perfect_preamble")
    
    # Verify detection occurred
    assert len(detections) > 0, "No preamble detected!"
    assert len(detections) == 1, f"Expected 1 detection, got {len(detections)}"
    
    print("\n✓ Perfect preamble detection test passed!")


@pytest.mark.parametrize("nfft,w_in", [(2048, 12)])
def test_minn_detector_verilator(nfft, w_in):
    """Pytest wrapper to run cocotb test with Verilator."""
    
    if VERILATOR is None:
        pytest.skip("Verilator executable not found")
    
    rtl_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "rtl"))
    
    # List all required RTL files
    rtl_files = [
        os.path.join(rtl_dir, "minn_delay_line.sv"),
        os.path.join(rtl_dir, "minn_running_sum.sv"),
        os.path.join(rtl_dir, "minn_antenna_datapath.sv"),
        os.path.join(rtl_dir, "minn_gate_peak_fsm.sv"),
        os.path.join(rtl_dir, "minn_output_buffer.sv"),
        os.path.join(rtl_dir, "minn_preamble_detector.sv"),
    ]
    
    build_dir = os.path.join("tests", "sim_build", f"minn_detector_{nfft}_{w_in}")
    
    # Parameters for the detector
    parameters = {
        "NFFT": nfft,
        "W_IN": w_in,
        "THRESHOLD": 0x1800,  # Q1.15 format
        "IIR_SHIFT": 3,
        "HYSTERESIS": 2,
        "TIMING_OFFSET": 0,
        "OUTPUT_DELAY": nfft,
    }
    
    run(
        verilog_sources=rtl_files,
        toplevel="minn_preamble_detector",
        toplevel_lang="verilog",
        module=os.path.splitext(os.path.basename(__file__))[0],
        parameters=parameters,
        sim_build=build_dir,
        simulator="verilator",
        compile_args=["--trace", "--trace-structs"],
        extra_env={
            "COCOTB_RESULTS_FILE": os.path.join(build_dir, "results.xml"),
        },
    )


if __name__ == "__main__":
    # Run the test directly
    test_minn_detector_verilator(2048, 12)