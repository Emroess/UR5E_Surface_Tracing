"""Simple real-time filters for wrench / signals.

Used to reduce noise on TCP force/torque feedback from RTDE.
"""

import numpy as np
from typing import Union, Optional


class LowPassFilter:
    """First-order exponential low-pass filter.

    y[k] = alpha * x[k] + (1 - alpha) * y[k-1]

    alpha in (0, 1]:
      - small alpha (e.g. 0.05-0.15) = heavy smoothing, good for noisy wrench
      - larger alpha = more responsive, less filtering
    """

    def __init__(self, alpha: float = 0.1, dim: int = 6, initial_value: Optional[np.ndarray] = None):
        self.alpha = float(np.clip(alpha, 1e-6, 1.0))
        self.dim = dim
        if initial_value is None:
            self.state = np.zeros(dim)
        else:
            self.state = np.asarray(initial_value, dtype=float).copy()
            if self.state.shape != (dim,):
                raise ValueError(f"initial_value must be length {dim}")

    def reset(self, value: Optional[np.ndarray] = None):
        if value is None:
            self.state.fill(0.0)
        else:
            self.state = np.asarray(value, dtype=float).copy()

    def update(self, x: Union[list, np.ndarray]) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        if x.shape != (self.dim,):
            raise ValueError(f"Input must be {self.dim}-dimensional, got {x.shape}")
        self.state = self.alpha * x + (1.0 - self.alpha) * self.state
        return self.state.copy()

    @property
    def value(self) -> np.ndarray:
        return self.state.copy()


class Deadband:
    """Simple per-element deadband (useful on wrench to ignore sensor bias/noise)."""

    def __init__(self, threshold: Union[float, np.ndarray], dim: int = 6):
        if np.isscalar(threshold):
            self.threshold = np.full(dim, float(threshold))
        else:
            self.threshold = np.asarray(threshold, dtype=float)
            if self.threshold.shape != (dim,):
                raise ValueError("threshold must be scalar or length-6 array")

    def apply(self, x: Union[list, np.ndarray]) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        out = np.zeros_like(x)
        mask = np.abs(x) > self.threshold
        out[mask] = x[mask] - np.sign(x[mask]) * self.threshold[mask]
        return out
