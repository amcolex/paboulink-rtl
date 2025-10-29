from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

# Default system parameters keep the module easy to use out of the box.
N_FFT = 2048
NUM_ACTIVE_SUBCARRIERS = 1200
CYCLIC_PREFIX = 512
SAMPLE_RATE_HZ = 30_720_000.0  # 30.72 MHz default sampling rate


@dataclass(frozen=True)
class OFDMParameters:
    """Container for core OFDM dimensions."""

    n_fft: int = N_FFT
    num_active: int = NUM_ACTIVE_SUBCARRIERS
    cp_len: int = CYCLIC_PREFIX

    def __post_init__(self) -> None:
        if self.n_fft % 2:
            raise ValueError("FFT size must be even.")
        if self.num_active % 2:
            raise ValueError("Active subcarrier count must be even to skip DC.")
        if self.num_active >= self.n_fft:
            raise ValueError("Active subcarrier count must be smaller than FFT size.")
        if self.cp_len < 0:
            raise ValueError("Cyclic prefix length must be non-negative.")


def centered_subcarrier_indices(num_active: int, *, spacing: int = 1) -> np.ndarray:
    """Return symmetric subcarrier indices around DC while skipping the 0th bin."""
    if num_active % 2:
        raise ValueError("num_active must be even so the DC bin can be skipped cleanly.")
    if spacing <= 0:
        raise ValueError("spacing must be a positive integer.")
    half = num_active // 2
    negative = np.arange(-half, 0, dtype=int)
    positive = np.arange(1, half + 1, dtype=int)
    indices = np.concatenate((negative, positive))
    if spacing != 1:
        indices = indices * spacing
    return indices


def allocate_subcarriers(n_fft: int, indices: np.ndarray, values: np.ndarray) -> np.ndarray:
    """Place active subcarrier values into an FFT-sized spectrum."""
    if indices.shape[0] != values.shape[0]:
        raise ValueError("indices and values must be the same length.")
    spectrum = np.zeros(n_fft, dtype=np.complex128)
    dc_index = n_fft // 2
    placement = (dc_index + indices) % n_fft
    spectrum[placement] = values
    return spectrum


def spectrum_to_time_domain(
    spectrum: np.ndarray,
    *,
    normalize: bool = True,
) -> np.ndarray:
    """Convert a centered spectrum into the time domain."""
    time_domain = np.fft.ifft(np.fft.ifftshift(spectrum))
    if normalize:
        power = np.mean(np.abs(time_domain) ** 2)
        if power > 0:
            time_domain = time_domain / np.sqrt(power)
    return time_domain.astype(np.complex128, copy=False)


def add_cyclic_prefix(symbol: np.ndarray, cp_len: int) -> np.ndarray:
    """Prepend the last cp_len samples as the cyclic prefix."""
    if cp_len <= 0:
        return np.asarray(symbol, dtype=np.complex128)
    symbol = np.asarray(symbol, dtype=np.complex128)
    return np.concatenate((symbol[-cp_len:], symbol))


def remove_cyclic_prefix(symbol: np.ndarray, cp_len: int) -> np.ndarray:
    """Drop the cyclic prefix from a symbol."""
    if cp_len <= 0:
        return np.asarray(symbol, dtype=np.complex128)
    return np.asarray(symbol, dtype=np.complex128)[cp_len:]


def generate_ofdm_symbol(
    subcarrier_values: np.ndarray,
    *,
    params: OFDMParameters | None = None,
    include_cp: bool = True,
    normalize: bool = True,
    spacing: int = 1,
) -> np.ndarray:
    """Build a single OFDM symbol from active subcarrier values."""
    params = OFDMParameters() if params is None else params
    values = np.asarray(subcarrier_values, dtype=np.complex128)
    indices = centered_subcarrier_indices(values.size, spacing=spacing)
    if values.size and np.max(np.abs(indices)) >= params.n_fft // 2:
        raise ValueError("Active subcarriers exceed the available FFT bins.")
    spectrum = allocate_subcarriers(params.n_fft, indices, values)
    symbol = spectrum_to_time_domain(spectrum, normalize=normalize)
    if include_cp:
        symbol = add_cyclic_prefix(symbol, params.cp_len)
    return symbol


def random_qpsk_values(
    num_active: int,
    rng: np.random.Generator | None = None,
) -> np.ndarray:
    """Return unit-power QPSK symbols for the requested number of subcarriers."""
    if rng is None:
        rng = np.random.default_rng()
    raw = rng.integers(0, 4, size=num_active)
    real = np.where(raw & 1, 1.0, -1.0)
    imag = np.where(raw & 2, 1.0, -1.0)
    return (real + 1j * imag) / np.sqrt(2.0)


def generate_qpsk_symbol(
    *,
    params: OFDMParameters | None = None,
    rng: np.random.Generator | None = None,
    include_cp: bool = True,
    normalize: bool = True,
    spacing: int = 1,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate a random QPSK OFDM symbol and return (time_domain, subcarrier_values)."""
    params = OFDMParameters() if params is None else params
    if params.num_active % spacing:
        raise ValueError("params.num_active must be divisible by the spacing value.")
    values = random_qpsk_values(params.num_active // spacing, rng=rng)
    symbol = generate_ofdm_symbol(
        values,
        params=params,
        include_cp=include_cp,
        normalize=normalize,
        spacing=spacing,
    )
    return symbol, values


def generate_preamble(
    *,
    params: OFDMParameters | None = None,
    include_cp: bool = True,
    normalize: bool = True,
    subcarrier_values: np.ndarray | None = None,
    subcarrier_value: complex = 1.0 + 0.0j,
) -> tuple[np.ndarray, np.ndarray]:
    """Create a preamble symbol shaped as [A A -A -A] in the time domain.

    The implementation uses every fourth subcarrier (spacing=4), then flips the
    sign of the second half of the time-domain symbol to create the requested
    pattern. Returns (time_domain_symbol, active_subcarrier_values) where the
    time-domain vector already includes the cyclic prefix if `include_cp` is True.
    """
    params = OFDMParameters() if params is None else params
    if params.n_fft % 4:
        raise ValueError("Preamble generation requires an FFT length divisible by 4.")
    quarter_active = params.num_active // 4
    if quarter_active == 0:
        raise ValueError("Not enough active subcarriers to build a quarter-band preamble.")

    if subcarrier_values is None:
        values = np.full(quarter_active, subcarrier_value, dtype=np.complex128)
    else:
        values = np.asarray(subcarrier_values, dtype=np.complex128)
        if values.shape[0] != quarter_active:
            raise ValueError(
                f"Expected {quarter_active} subcarrier values, got {values.shape[0]} instead."
            )

    indices = centered_subcarrier_indices(values.size, spacing=4)
    spectrum = allocate_subcarriers(params.n_fft, indices, values)
    base_symbol = spectrum_to_time_domain(spectrum, normalize=normalize)

    # Flip the sign on the second half of the time-domain symbol.
    preamble = base_symbol.copy()
    preamble[params.n_fft // 2 :] *= -1.0  # results in [A A -A -A]

    if include_cp:
        preamble = add_cyclic_prefix(preamble, params.cp_len)
    return preamble, values


def generate_frame(
    data_subcarriers: Sequence[np.ndarray],
    *,
    params: OFDMParameters | None = None,
    include_cp: bool = True,
    normalize: bool = True,
    add_preamble: bool = True,
    preamble: np.ndarray | None = None,
    spacing: int = 1,
) -> np.ndarray:
    """Assemble a frame from optional preamble plus a list of data symbols."""
    params = OFDMParameters() if params is None else params
    if params.num_active % spacing:
        raise ValueError("params.num_active must be divisible by the chosen spacing.")

    symbols: list[np.ndarray] = []
    if add_preamble:
        if preamble is None:
            preamble, _ = generate_preamble(
                params=params,
                include_cp=include_cp,
                normalize=normalize,
            )
        symbols.append(np.asarray(preamble, dtype=np.complex128))

    for block in data_subcarriers:
        block = np.asarray(block, dtype=np.complex128)
        expected = params.num_active // spacing
        if block.size != expected:
            raise ValueError(f"Each data block must contain {expected} subcarriers.")
        symbol = generate_ofdm_symbol(
            block,
            params=params,
            include_cp=include_cp,
            normalize=normalize,
            spacing=spacing,
        )
        symbols.append(symbol)

    if not symbols:
        return np.array([], dtype=np.complex128)
    return np.concatenate(symbols)
