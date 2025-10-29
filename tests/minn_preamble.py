"""
Minn preamble generation for OFDM timing synchronization.
Implements the classic Minn structure: [A A A A] → [A A -A -A]
"""

import numpy as np
from ofdm_utils import subcarrier_mapping, ifft_with_cp, generate_qpsk_symbols


def generate_minn_preamble(nfft=2048, cp_length=512, num_active=1200, spacing=4):
    """
    Generate Minn preamble with [A A -A -A] structure.
    
    The Minn preamble is designed so that:
    - The FFT symbol has 4 identical quarters in frequency domain
    - In time domain, this creates [A A A A] pattern
    - We then invert the second half to get [A A -A -A]
    - This enables quarter-lag correlation for timing detection
    
    Args:
        nfft: FFT size (must be divisible by 4)
        cp_length: Cyclic prefix length in samples
        num_active: Number of active subcarriers
        spacing: Subcarrier spacing (every Nth subcarrier)
    
    Returns:
        preamble_time: Time domain preamble with CP (complex array)
        preamble_freq: Frequency domain representation
        active_indices: Indices of active subcarriers used
    """
    assert nfft % 4 == 0, "NFFT must be divisible by 4 for Minn preamble"
    
    q = nfft // 4  # Quarter length
    
    # Generate QPSK symbols for the preamble
    # We need num_active symbols total, placed on every 'spacing'th subcarrier
    preamble_symbols = generate_qpsk_symbols(num_active)
    
    # Create indices for active subcarriers
    # Center them around DC with specified spacing
    # For NFFT=2048, num_active=1200, spacing=4:
    # We'll place 600 on positive frequencies and 600 on negative frequencies
    half_active = num_active // 2
    
    # Positive frequencies: [spacing, 2*spacing, 3*spacing, ...]
    pos_indices = np.arange(1, half_active + 1) * spacing
    
    # Negative frequencies: wrap around using NFFT
    neg_indices = nfft - np.arange(1, half_active + 1) * spacing
    
    # Combine indices
    active_indices = np.concatenate([pos_indices, neg_indices])
    
    # Ensure we don't exceed array bounds
    active_indices = active_indices[active_indices < nfft]
    
    # Adjust if we have fewer indices than symbols
    num_indices = len(active_indices)
    if num_indices < num_active:
        preamble_symbols = preamble_symbols[:num_indices]
    elif num_indices > num_active:
        active_indices = active_indices[:num_active]
    
    # Map symbols to frequency domain (this creates [A A A A] pattern in time)
    freq_domain = subcarrier_mapping(preamble_symbols, nfft, active_indices)
    
    # Perform IFFT to get time domain [A A A A] pattern
    time_domain_aaaa = np.fft.ifft(freq_domain)
    
    # Create [A A -A -A] pattern by inverting second half
    time_domain_minn = time_domain_aaaa.copy()
    half_point = nfft // 2
    time_domain_minn[half_point:] = -time_domain_minn[half_point:]
    
    # Add cyclic prefix
    cyclic_prefix = time_domain_minn[-cp_length:]
    preamble_with_cp = np.concatenate([cyclic_prefix, time_domain_minn])
    
    return preamble_with_cp, freq_domain, active_indices


def verify_minn_structure(preamble_time, nfft, cp_length):
    """
    Verify that the preamble has the expected [A A -A -A] quarter structure.
    
    Args:
        preamble_time: Time domain preamble with CP
        nfft: FFT size
        cp_length: CP length
    
    Returns:
        correlation_01: Correlation between quarters 0 and 1
        correlation_23: Correlation between quarters 2 and 3
        correlation_02: Correlation between quarters 0 and 2 (should be negative)
    """
    # Remove CP
    preamble_no_cp = preamble_time[cp_length:cp_length + nfft]
    
    q = nfft // 4
    
    # Extract quarters
    q0 = preamble_no_cp[0:q]
    q1 = preamble_no_cp[q:2*q]
    q2 = preamble_no_cp[2*q:3*q]
    q3 = preamble_no_cp[3*q:4*q]
    
    # Compute correlations (normalized)
    corr_01 = np.abs(np.vdot(q0, q1)) / (np.linalg.norm(q0) * np.linalg.norm(q1))
    corr_23 = np.abs(np.vdot(q2, q3)) / (np.linalg.norm(q2) * np.linalg.norm(q3))
    
    # This should be negative (inverted)
    corr_02_raw = np.vdot(q0, q2) / (np.linalg.norm(q0) * np.linalg.norm(q2))
    corr_02 = np.real(corr_02_raw)  # Should be approximately -1
    
    return corr_01, corr_23, corr_02


def generate_dual_antenna_preamble(nfft=2048, cp_length=512, num_active=1200, 
                                    spacing=4, antenna_seed=None):
    """
    Generate Minn preambles for dual-antenna setup.
    
    Both antennas transmit the same preamble structure, but with potentially
    different random QPSK symbols (or identical if same seed).
    
    Args:
        nfft: FFT size
        cp_length: Cyclic prefix length
        num_active: Number of active subcarriers
        spacing: Subcarrier spacing
        antenna_seed: If provided, use same seed for both antennas (correlated)
                      If None, use different random seeds (uncorrelated)
    
    Returns:
        ch0_preamble: Time domain preamble for antenna 0 with CP
        ch1_preamble: Time domain preamble for antenna 1 with CP
        freq_domain: Frequency domain representation (same structure for both)
        active_indices: Active subcarrier indices
    """
    if antenna_seed is not None:
        # Use same seed for both antennas (correlated transmission)
        np.random.seed(antenna_seed)
        ch0_preamble, freq_domain, active_indices = generate_minn_preamble(
            nfft, cp_length, num_active, spacing
        )
        
        np.random.seed(antenna_seed)
        ch1_preamble, _, _ = generate_minn_preamble(
            nfft, cp_length, num_active, spacing
        )
    else:
        # Different random seeds (uncorrelated transmission)
        ch0_preamble, freq_domain, active_indices = generate_minn_preamble(
            nfft, cp_length, num_active, spacing
        )
        ch1_preamble, _, _ = generate_minn_preamble(
            nfft, cp_length, num_active, spacing
        )
    
    return ch0_preamble, ch1_preamble, freq_domain, active_indices


if __name__ == "__main__":
    # Test the preamble generation
    print("Generating Minn preamble...")
    preamble, freq, indices = generate_minn_preamble()
    
    print(f"Preamble length with CP: {len(preamble)}")
    print(f"Expected: {2048 + 512} = 2560")
    print(f"Number of active subcarriers: {len(indices)}")
    
    # Verify structure
    corr_01, corr_23, corr_02 = verify_minn_structure(preamble, 2048, 512)
    print(f"\nVerification:")
    print(f"  Correlation Q0-Q1: {corr_01:.4f} (should be ~1.0)")
    print(f"  Correlation Q2-Q3: {corr_23:.4f} (should be ~1.0)")
    print(f"  Correlation Q0-Q2: {corr_02:.4f} (should be ~-1.0)")
    
    if corr_01 > 0.95 and corr_23 > 0.95 and corr_02 < -0.95:
        print("\n✓ Minn preamble structure verified!")
    else:
        print("\n✗ Warning: Preamble structure may be incorrect")