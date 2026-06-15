#!/usr/bin/env python3
"""
External Impedance Controller - Free Space Test (URSim / Real)

Primary SIL tool for testing impedance (virtual spring-damper around a desired pose).

- Holds (or slowly follows) a desired TCP pose.
- External forces (real or SIMULATED) cause the command target to shift.
- Robot "pushes back" when the force is removed.
- Uses speedL velocity output for simplicity and universality (you can also
  use the get_target_pose() with servoL/servoj if preferred).

Same caveats as the admittance example: in URSim the real wrench is usually
not useful for contact. Use the simulated wrench to validate the response
logic and gain tuning first.
"""

import sys
import time
import numpy as np

sys.path.insert(0, "..")
sys.path.insert(0, ".")

from utils.rtde_utils import RtdeInterface, ControlRate, wait_for_robot_ready
from controllers.impedance import ImpedanceController


# ====================== TUNING ======================
CONTROL_HZ = 125.0
DURATION_S = 60.0

USE_SIMULATED_WRENCH = True

def get_simulated_wrench(t: float, state) -> np.ndarray:
    w = np.zeros(6)
    # Push in -Y for  a while, then release, then X, etc.
    phase = (t % 18.0)
    if 2 < phase < 7:
        w[1] = -10.0 * np.sin((phase-2)/5 * np.pi)   # smooth push in Y
    if 10 < phase < 14:
        w[0] = 7.0
        w[2] = 5.0
    return w


# Impedance gains - moderate for free-space feel
IMPEDANCE_K = (600.0, 600.0, 600.0, 25.0, 25.0, 25.0)   # N/m , Nm/rad
IMPEDANCE_D = (35.0, 35.0, 35.0, 1.8, 1.8, 1.8)        # damping on offset
WRENCH_ALPHA = 0.12
WRENCH_DEADBAND = (1.8, 1.8, 1.8, 0.18, 0.18, 0.18)
MAX_DELTA = (0.06, 0.06, 0.06, 0.35, 0.35, 0.35)
VEL_P = (3.5, 3.5, 3.5, 5.0, 5.0, 5.0)
MAX_VEL = (0.12, 0.12, 0.12, 0.5, 0.5, 0.5)
# ====================================================


def main(host: str = "localhost"):
    print("=" * 60)
    print("UR5e EXTERNAL IMPEDANCE CONTROLLER (free-space / SIL test)")
    print("=" * 60)
    print(f"Host: {host}   Rate: {CONTROL_HZ} Hz   Simulated: {USE_SIMULATED_WRENCH}")
    print("The robot will try to stay near the pose captured at start.")
    print("Press Ctrl-C to exit.\n")

    iface = RtdeInterface(host=host, frequency=CONTROL_HZ, verbose=True)
    if not iface.connect():
        print("ERROR: RTDE connect failed.")
        return 1

    if not wait_for_robot_ready(iface, timeout=8.0):
        print("ERROR: No pose. Check robot is powered and in remote mode.")
        iface.disconnect()
        return 1

    state0 = iface.get_state()
    print(f"Initial TCP pose captured as impedance target:\n  {np.round(state0.tcp_pose, 4)}")

    impedance = ImpedanceController(
        stiffness=IMPEDANCE_K,
        damping=IMPEDANCE_D,
        wrench_lowpass_alpha=WRENCH_ALPHA,
        wrench_deadband=WRENCH_DEADBAND,
        max_delta=MAX_DELTA,
        vel_p_gain=VEL_P,
        max_vel=MAX_VEL,
        dt=1.0 / CONTROL_HZ,
    )
    impedance.reset(x_desired=state0.tcp_pose)

    print("Impedance active. Apply (simulated or real) force to see deviation + return behavior.")
    rate = ControlRate(CONTROL_HZ)
    t0 = time.monotonic()
    cycle = 0

    try:
        while time.monotonic() - t0 < DURATION_S:
            t = time.monotonic() - t0
            state = iface.get_state()

            wrench = get_simulated_wrench(t, state) if USE_SIMULATED_WRENCH else state.tcp_force

            # Update impedance (internal delta) and get a velocity correction toward compliant target
            v_cmd = impedance.compute_velocity_correction(
                current_pose=state.tcp_pose,
                wrench_raw=wrench,
                dt=rate.dt
            )

            iface.send_speedL(v_cmd, acceleration=0.35, t=0.002)

            if cycle % int(CONTROL_HZ) == 0:
                delta = impedance.get_delta()
                target = impedance.get_target_pose()
                print(f"t={t:5.1f}s  wrench={np.round(wrench,1)}  delta={np.round(delta,4)}")
                print(f"          target={np.round(target,3)}")

            rate.sleep()
            cycle += 1

    except KeyboardInterrupt:
        print("\nUser interrupt.")
    finally:
        print("Stopping...")
        iface.safe_stop()
        iface.disconnect()
        print("Impedance test finished.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
