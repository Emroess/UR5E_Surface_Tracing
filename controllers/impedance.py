"""External Impedance Controller (Python / RTDE).

Impedance: desired pose + gains  -->  "virtual force" from position error.
On a position-controlled robot (via RTDE servo/speed) we achieve impedance-like
behavior by continuously updating the command pose/velocity so that external
forces cause temporary deviations and the robot "pushes back".

Practical implementation used here (very common for UR external control):
- Maintain an explicit desired pose (x_des).
- Measure wrench.
- Compute a compliant offset: delta = lowpass( wrench / K )   [with deadband + rate limits]
- The instantaneous target becomes x_target = x_des + delta
- Additionally apply damping on the rate of change of delta (prevents jerk).
- Output either:
    1. A velocity command (speedL) that drives toward x_target, or
    2. The x_target directly for use with servoJ/servoL.

This produces spring-damper behavior around x_des. Higher K = stiffer.

Use for "soft position holding" that yields nicely to contact or human interaction.
"""

from __future__ import annotations
import numpy as np
from typing import Optional, Tuple
from utils.filters import LowPassFilter, Deadband


class ImpedanceController:
    """6-DOF decoupled impedance controller realized via target pose modulation.

    Args:
        stiffness: 6-vector or scalar. Main "spring" gain. Higher = less deviation under force.
        damping: 6-vector or scalar. Damping on the *compliance offset* (not robot velocity).
                 Helps reduce oscillation of the virtual target.
        wrench_lowpass_alpha: filter strength on measured wrench.
        wrench_deadband: ignore small forces (sensor bias / noise).
        max_delta: maximum allowed compliance offset (m / rad) for safety.
        vel_p_gain: proportional gain when converting (x_target - current) to velocity.
                    Only used by compute_velocity_correction().
        max_vel: saturation for any velocity output.
        dt: nominal timestep.
    """

    def __init__(
        self,
        stiffness: Tuple[float, ...] | float = (800.0, 800.0, 800.0, 30.0, 30.0, 30.0),
        damping: Tuple[float, ...] | float = (40.0, 40.0, 40.0, 1.5, 1.5, 1.5),
        wrench_lowpass_alpha: float = 0.15,
        wrench_deadband: Tuple[float, ...] | float = (2.0, 2.0, 2.0, 0.2, 0.2, 0.2),
        max_delta: Tuple[float, ...] | float = (0.08, 0.08, 0.08, 0.4, 0.4, 0.4),
        vel_p_gain: Tuple[float, ...] | float = (4.0, 4.0, 4.0, 6.0, 6.0, 6.0),
        max_vel: Tuple[float, ...] | float = (0.20, 0.20, 0.20, 0.6, 0.6, 0.6),
        dt: float = 1.0 / 125.0,
    ):
        self.dim = 6

        def _as6(val):
            if np.isscalar(val) or (hasattr(val, "__len__") and len(val) == 1):
                return np.full(self.dim, float(val))
            return np.asarray(val, dtype=float)

        self.K = _as6(stiffness)
        self.D = _as6(damping)
        self.max_delta = _as6(max_delta)
        self.vel_p_gain = _as6(vel_p_gain)
        self.max_vel = _as6(max_vel)

        self.wrench_filter = LowPassFilter(alpha=wrench_lowpass_alpha, dim=self.dim)
        self.deadband = Deadband(threshold=wrench_deadband, dim=self.dim)

        self.dt = float(dt)

        # State
        self.x_des = np.zeros(self.dim)     # user-provided desired pose (base or task frame)
        self.delta = np.zeros(self.dim)     # compliance offset (x_target = x_des + delta)
        self._initialized = False

    def reset(self, x_desired: Optional[np.ndarray] = None):
        """Reset compliance offset. Set (or update) the desired pose."""
        self.delta.fill(0.0)
        self.wrench_filter.reset()
        if x_desired is not None:
            self.x_des = np.asarray(x_desired, dtype=float).copy()
        self._initialized = True

    def set_desired_pose(self, x_desired: np.ndarray):
        """Change the impedance attractor pose (e.g. from a trajectory generator)."""
        self.x_des = np.asarray(x_desired, dtype=float).copy()

    def step(self, wrench_raw: np.ndarray, current_pose: Optional[np.ndarray] = None, dt: Optional[float] = None) -> np.ndarray:
        """Compute the instantaneous compliant target pose (x_des + delta).

        Call this every cycle. Feed the returned pose to a servo command (servoL if available,
        or integrate into velocity commands).

        current_pose is currently unused for the delta calculation but can be used in future
        for more advanced "force + position error" impedance formulations.
        """
        dt = float(dt) if dt is not None else self.dt

        if not self._initialized:
            self.reset(x_desired=current_pose if current_pose is not None else np.zeros(6))
            self.wrench_filter.reset(wrench_raw)
            self._initialized = True

        # Process wrench
        f = self.wrench_filter.update(wrench_raw)
        f = self.deadband.apply(f)

        # Core impedance -> compliance offset
        # delta_dot = (F / K) - (D/K) * delta     (first order lag on the offset)
        # This is equivalent to a virtual spring-damper on the offset itself.
        for i in range(self.dim):
            if abs(self.K[i]) < 1e-6:
                target_delta = 0.0
            else:
                target_delta = f[i] / self.K[i]
            # First-order approach of delta toward target_delta with damping influence
            # We treat D here as additional damping on delta motion
            err = target_delta - self.delta[i]
            d_delta = (err * self.K[i] - self.D[i] * self.delta[i]) / max(self.K[i], 1e-3)
            self.delta[i] += d_delta * dt

        # Enforce max deviation (safety)
        self.delta = np.clip(self.delta, -self.max_delta, self.max_delta)

        x_target = self.x_des + self.delta
        return x_target.copy()

    def compute_velocity_correction(self,
                                    current_pose: np.ndarray,
                                    wrench_raw: Optional[np.ndarray] = None,
                                    dt: Optional[float] = None) -> np.ndarray:
        """Alternative output: a velocity that pulls toward the current compliant target.

        Useful if you prefer to stay in velocity control (speedL) even for impedance.
        """
        dt = float(dt) if dt is not None else self.dt
        if wrench_raw is not None:
            # Make sure internal delta is up to date
            self.step(wrench_raw, current_pose, dt)

        x_target = self.x_des + self.delta
        err = x_target - current_pose

        v = self.vel_p_gain * err
        # crude damping on velocity
        v = np.clip(v, -self.max_vel, self.max_vel)
        return v

    def get_target_pose(self) -> np.ndarray:
        return (self.x_des + self.delta).copy()

    def get_delta(self) -> np.ndarray:
        return self.delta.copy()
