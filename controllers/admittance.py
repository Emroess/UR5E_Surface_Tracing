"""External Admittance Controller (Python / RTDE).

Admittance: measured wrench -> desired motion (velocity or pose offset).

This is the "force-responsive" behavior commonly used for hand-guiding,
exploration, and compliant interaction when you only have a position/velocity
interface (servoj / speedj / speedL).

The controller is a discrete 6-DOF decoupled mass-damper-spring system:

    M * a + D * v + K * (x - x_ref) = F_ext

Simplified Euler integration is used. When using speedL we primarily
care about v (the admittance velocity). An internal integrated pose
is maintained only when you want a position target.

Recommended for free-space + light interaction development in URSim.
"""

from __future__ import annotations
import numpy as np
from typing import Optional, Tuple
from utils.filters import LowPassFilter, Deadband


class AdmittanceController:
    """6-DOF decoupled admittance controller.

    Args:
        mass: 6-vector or scalar (kg or kg*m^2). Small values = more responsive.
        damping: 6-vector or scalar (Ns/m or Nms/rad). Primary tuning knob.
        stiffness: 6-vector or scalar. Usually small or zero for pure admittance.
                   Non-zero gives a restoring "home" behavior.
        wrench_lowpass_alpha: exponential filter coefficient on raw wrench (0.05-0.25 typical).
        wrench_deadband: per-axis threshold (N or Nm) below which force is treated as 0.
        max_vel: saturation on output velocity command (m/s and rad/s).
        dt: nominal control period (used for integration). Can be overridden per step.
    """

    def __init__(
        self,
        mass: Tuple[float, ...] | float = (2.0, 2.0, 2.0, 0.1, 0.1, 0.1),
        damping: Tuple[float, ...] | float = (80.0, 80.0, 80.0, 4.0, 4.0, 4.0),
        stiffness: Tuple[float, ...] | float = (5.0, 5.0, 5.0, 0.2, 0.2, 0.2),
        wrench_lowpass_alpha: float = 0.12,
        wrench_deadband: Tuple[float, ...] | float = (1.5, 1.5, 1.5, 0.15, 0.15, 0.15),
        max_vel: Tuple[float, ...] | float = (0.25, 0.25, 0.25, 0.8, 0.8, 0.8),
        dt: float = 1.0 / 125.0,
    ):
        self.dim = 6

        def _as6(val):
            if np.isscalar(val) or len(val) == 1:
                return np.full(self.dim, float(val))
            return np.asarray(val, dtype=float)

        self.M = _as6(mass)
        self.D = _as6(damping)
        self.K = _as6(stiffness)
        self.max_vel = _as6(max_vel)

        self.wrench_filter = LowPassFilter(alpha=wrench_lowpass_alpha, dim=self.dim)
        self.deadband = Deadband(threshold=wrench_deadband, dim=self.dim)

        self.dt = float(dt)

        # Internal states
        self.v = np.zeros(self.dim)          # integrated velocity (admittance vel)
        self.x = np.zeros(self.dim)          # integrated "virtual" position (optional use)
        self.x_ref = np.zeros(self.dim)      # reference / home pose for spring term

        self._initialized = False

    def reset(self, x_ref: Optional[np.ndarray] = None):
        """Reset integrators. Optionally set new spring reference (usually current pose)."""
        self.v.fill(0.0)
        self.x.fill(0.0)
        self.wrench_filter.reset()
        if x_ref is not None:
            self.x_ref = np.asarray(x_ref, dtype=float).copy()
        else:
            self.x_ref.fill(0.0)
        self._initialized = True

    def set_reference(self, x_ref: np.ndarray):
        """Update the spring zero-force pose (e.g. current pose when enabling compliance)."""
        self.x_ref = np.asarray(x_ref, dtype=float).copy()

    def step(self, wrench_raw: np.ndarray, dt: Optional[float] = None) -> np.ndarray:
        """Compute one admittance velocity command.

        Returns 6D velocity suitable for speedL (or to be integrated for a servo target).
        """
        if not self._initialized:
            # First call: treat current wrench as zero point
            self.wrench_filter.reset(wrench_raw)
            self.reset(x_ref=np.zeros(6))
            self._initialized = True

        dt = float(dt) if dt is not None else self.dt

        # Filter + deadband wrench
        f = self.wrench_filter.update(wrench_raw)
        f = self.deadband.apply(f)

        # Discrete mass-damper-spring (per axis)
        # a = (F - D*v - K*(x - x_ref)) / M
        # v += a * dt
        # x += v * dt   (we keep x mainly for the spring term and optional pose output)

        for i in range(self.dim):
            m = max(self.M[i], 1e-4)
            restoring = self.K[i] * (self.x[i] - self.x_ref[i])
            a = (f[i] - self.D[i] * self.v[i] - restoring) / m
            self.v[i] += a * dt
            self.x[i] += self.v[i] * dt

        # Saturate
        v_cmd = np.clip(self.v, -self.max_vel, self.max_vel)
        self.v = v_cmd.copy()  # keep saturated state for next integration

        return v_cmd.copy()

    def get_virtual_pose(self) -> np.ndarray:
        """Return the internally integrated virtual pose (x). Useful if you want to servo to it."""
        return self.x.copy()

    def get_velocity(self) -> np.ndarray:
        """Return last computed velocity command."""
        return self.v.copy()
