"""
OFDM symbol generation utilities for testing the Minn preamble detector.
"""

import numpy as np


def generate_qpsk_symbols(num_symbols):
    """Generate random QPSK symbols (±1±j)/sqrt(2)."""
    # QPSK constellation: [1+1j, 1-1j, -1+1j, -1-1j] / sqrt(2)
    bits = np.random.randint(0, 4, num_symbols)
    qpsk = np.array([1+1j, 1-1j, -1+1j, -1-1j]) / np.sqrt(2)
    return qpsk[bits]


def subcarrier_mapping(symbols, nfft, active_indices):
    """
    Map symbols to subcarriers for OFDM.
    
    Args:
        symbols: Complex symbols to map
        nfft: FFT size
        active_indices: Indices of active subcarriers (DC-centered, wrapped)
    
    Returns:
        freq_domain: Frequency domain OFDM symbol (length nfft)
    """
    freq_domain = np.zeros(nfft, dtype=complex)
    freq_domain[active_indices] = symbols
    return freq_domain


def ifft_with_cp(freq_domain, cp_length):
    """
    Perform IFFT and add cyclic prefix.
    
    Args:
        freq_domain: Frequency domain OFDM symbol
        cp_length: Cyclic prefix length in samples
    
    Returns:
        time_domain: Time domain OFDM symbol with CP prepended
    """
    # IFFT to get time domain
    time_domain_no_cp = np.fft.ifft(freq_domain)
    
    # Add cyclic prefix (copy last cp_length samples to beginning)
    cyclic_prefix = time_domain_no_cp[-cp_length:]
    time_domain = np.concatenate([cyclic_prefix, time_domain_no_cp])
    
    return time_domain


def fft_remove_cp(time_domain, nfft, cp_length):
    """
    Remove cyclic prefix and perform FFT.
    
    Args:
        time_domain: Time domain OFDM symbol with CP
        nfft: FFT size
        cp_length: Cyclic prefix length
    
    Returns:
        freq_domain: Frequency domain OFDM symbol
    """
    # Remove cyclic prefix
    time_domain_no_cp = time_domain[cp_length:cp_length + nfft]
    
    # FFT to get frequency domain
    freq_domain = np.fft.fft(time_domain_no_cp)
    
    return freq_domain


def generate_ofdm_symbol(nfft, cp_length, active_subcarrier_indices, data_symbols=None):
    """
    Generate a complete OFDM symbol in time domain.
    
    Args:
        nfft: FFT size
        cp_length: Cyclic prefix length
        active_subcarrier_indices: Indices of active subcarriers
        data_symbols: Optional pre-generated symbols (else random QPSK)
    
    Returns:
        ofdm_symbol: Time domain OFDM symbol with CP (length nfft + cp_length)
    """
    num_active = len(active_subcarrier_indices)
    
    # Generate symbols if not provided
    if data_symbols is None:
        data_symbols = generate_qpsk_symbols(num_active)
    
    # Map to subcarriers
    freq_domain = subcarrier_mapping(data_symbols, nfft, active_subcarrier_indices)
    
    # IFFT and add CP
    ofdm_symbol = ifft_with_cp(freq_domain, cp_length)
    
    return ofdm_symbol


def add_awgn(signal, snr_db):
    """
    Add AWGN to signal at specified SNR.
    
    Args:
        signal: Complex signal
        snr_db: Desired SNR in dB
    
    Returns:
        noisy_signal: Signal with added noise
    """
    # Calculate signal power
    signal_power = np.mean(np.abs(signal)**2)
    
    # Calculate noise power from SNR
    snr_linear = 10**(snr_db / 10.0)
    noise_power = signal_power / snr_linear
    
    # Generate complex Gaussian noise
    noise_std = np.sqrt(noise_power / 2)  # /2 for I and Q components
    noise = noise_std * (np.random.randn(len(signal)) + 1j * np.random.randn(len(signal)))
    
    return signal + noise


def apply_cfo(signal, freq_offset_hz, sample_rate_hz):
    """
    Apply carrier frequency offset to signal.
    
    Args:
        signal: Complex signal
        freq_offset_hz: Frequency offset in Hz
        sample_rate_hz: Sample rate in Hz
    
    Returns:
        signal_with_cfo: Signal with CFO applied
    """
    n = np.arange(len(signal))
    phase_increment = 2 * np.pi * freq_offset_hz / sample_rate_hz
    cfo_phasor = np.exp(1j * phase_increment * n)
    
    return signal * cfo_phasor


def quantize_iq(signal, bit_width, scale_factor=1.0):
    """
    Quantize complex signal to fixed-point representation.
    
    Args:
        signal: Complex floating-point signal
        bit_width: Number of bits for I and Q (signed)
        scale_factor: Scaling factor before quantization
    
    Returns:
        quantized_i: Integer I values (signed)
        quantized_q: Integer Q values (signed)
    """
    # Scale signal
    scaled = signal * scale_factor
    
    # Separate I and Q
    i_float = np.real(scaled)
    q_float = np.imag(scaled)
    
    # Quantize to signed integers
    max_val = (1 << (bit_width - 1)) - 1
    min_val = -(1 << (bit_width - 1))
    
    quantized_i = np.clip(np.round(i_float), min_val, max_val).astype(int)
    quantized_q = np.clip(np.round(q_float), min_val, max_val).astype(int)
    
    return quantized_i, quantized_q


def dequantize_iq(quantized_i, quantized_q, scale_factor=1.0):
    """
    Convert quantized I/Q back to complex floating-point.
    
    Args:
        quantized_i: Integer I values
        quantized_q: Integer Q values
        scale_factor: Inverse of quantization scale factor
    
    Returns:
        signal: Complex floating-point signal
    """
    i_float = quantized_i / scale_factor
    q_float = quantized_q / scale_factor
    
    return i_float + 1j * q_float