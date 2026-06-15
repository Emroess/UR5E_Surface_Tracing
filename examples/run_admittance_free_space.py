#!/usr/bin/env python3
"""
External Admittance Controller - Free Space Test (URSim / Real)

This script is the primary SIL development tool for admittance behavior.

Features:
- Uses AdmittanceController + RtdeInterface
- Optional **simulated wrench injection** so you can validate the control law,
  directionality, gains, and state machine logic **even when URSim wrench feedback
  is unrealistic or zero** (as documented in the project discussion).
- Clean rate-controlled loop with speedL commands.
- Graceful Ctrl-C handling.

Usage (with URSim running):
    python examples/run_admittance_free_space.py

Inside the script, toggle:
    USE_SIMULATED_WRENCH = True

You will see the TCP move in response to the synthetic forces (the sign/direction
of motion should match the force axis you inject). This proves the Python-side
math, filters, integration, and RTDE command path are working.

When running on real hardware with a real wrench sensor you can set
USE_SIMULATED_WRENCH = False and push the robot to test real compliance.
"""

import sys
import time
import numpy as np

sys.path.insert(0, "..")
sys.path.insert(0, ".")

from utils.rtde_utils import RtdeInterface, ControlRate, wait_for_robot_ready
from controllers.admittance import AdmittanceController


# ====================== TUNING FOR YOUR EXPERIMENTS ======================
CONTROL_HZ = 125.0
DURATION_S = 60.0

# Set True to inject fake forces for logic validation in URSim (highly recommended first)
USE_SIMULATED_WRENCH = True

# Simulated force profile (easy to edit)
def get_simulated_wrench(t: float, state) -> np.ndarray:
    """Return a 6D wrench that changes over time for testing."""
    w = np.zeros(6)
    # Example: slow sine push in +Z (index 2) and small +X oscillation
    w[0] = 8.0 * np.sin(2 * np.pi * 0.3 * t)          # Fx ~ +/- 8N
    w[2] = 12.0 * np.sin(2 * np.pi * 0.15 * t + 1.0)  # Fz ~ +/- 12N
    # Add a light torque pulse occasionally
    if 8.0 < (t % 20) < 10.0:
        w[4] = 1.2  # Ty
    return w


# Admittance gains - START CONSERVATIVE for free space
# (translational first 3, rotational last 3)
ADMITTANCE_MASS = (1.5, 1.5, 1.5, 0.08, 0.08, 0.08)
ADMITTANCE_DAMPING = (120.0, 120.0, 120.0, 6.0, 6.0, 6.0)   # high damping = less "runaway"
ADMITTANCE_STIFFNESS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)      # 0 = pure damper (no spring return)
WRENCH_ALPHA = 0.10
WRENCH_DEADBAND = (1.2, 1.2, 1.2, 0.12, 0.12, 0.12)
MAX_VEL = (0.15, 0.15, 0.15, 0.5, 0.5, 0.5)
# =========================================================================


def main(host: str = "localhost"):
    print("=" * 60)
    print("UR5e EXTERNAL ADMITTANCE CONTROLLER (free-space / SIL test)")
    print("=" * 60)
    print(f"Host: {host}   Rate: {CONTROL_HZ} Hz   Simulated wrench: {USE_SIMULATED_WRENCH}")
    print("Press Ctrl-C to stop cleanly.\n")

    iface = RtdeInterface(host=host, frequency=CONTROL_HZ, verbose=True)
    if not iface.connect():
        print("ERROR: Could not connect to RTDE. Check Docker port mapping for 30004.")
        return 1

    if not wait_for_robot_ready(iface):
        print("ERROR: No valid pose from robot. Is Polyscope in REMOTE control mode?")
        iface.disconnect()
        return 1

    # Initialize controller with current pose as reference (even if K=0 this is harmless)
    state0 = iface.get_state()
    admittance = AdmittanceController(
        mass=ADMITTANCE_MASS,
        damping=ADMITTANCE_DAMPING,
        stiffness=ADMITTANCE_STIFFNESS,
        wrench_lowpass_alpha=WRENCH_ALPHA,
        wrench_deadband=WRENCH_DEADBAND,
        max_vel=MAX_VEL,
        dt=1.0 / CONTROL_HZ,
    )
    admittance.reset(x_ref=state0.tcp_pose)

    print("Starting admittance loop. Robot should stay still or move only when 'pushed' (real or simulated).")
    rate = ControlRate(CONTROL_HZ)
    t0 = time.monotonic()
    cycle = 0

    try:
        while time.monotonic() - t0 < DURATION_S:
            t = time.monotonic() - t0
            state = iface.get_state()

            if USE_SIMULATED_WRENCH:
                wrench = get_simulated_wrench(t, state)
            else:
                wrench = state.tcp_force

            # Core admittance computation -> velocity command
            v_cmd = admittance.step(wrench, dt=rate.dt)

            # Send velocity command (Cartesian tool speed in base frame)
            iface.send_speedL(v_cmd, acceleration=0.4, t=0.002)

            # Occasional status
            if cycle % int(CONTROL_HZ) == 0:
                print(f"t={t:5.1f}s  | wrench={'SIM' if USE_SIMULATED_WRENCH else 'REAL'} {np.round(wrench,1)}")
                print(f"          v_cmd={np.round(v_cmd,3)}")

            # Very basic "state machine" hook: if large force, print alert (extend here)
            if np.max(np.abs(wrench)) > 25.0:
                print("  [!] Large force detected (sim or real)")

            rate.sleep()
            cycle += 1

    except KeyboardInterrupt:
        print("\nCtrl-C received.")
    finally:
        print("Stopping robot...")
        iface.safe_stop()
        iface.disconnect()
        print("Admittance test finished.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
