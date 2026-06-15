#!/usr/bin/env python3
"""Minimal RTDE read + control loop example.

Demonstrates:
- Connection via RtdeInterface
- High-rate state reading (pose, wrench, speed)
- Sending a zero velocity command at controlled rate (safe "hold" behavior)
- Clean shutdown on Ctrl-C

Run against URSim (or real robot in remote mode) after:
    pip install -r ../requirements.txt

This is the "hello world" before adding admittance/impedance.
"""

import sys
import time
import numpy as np

# Make it runnable from repo root or examples/
sys.path.insert(0, "..")
sys.path.insert(0, ".")

from utils.rtde_utils import RtdeInterface, ControlRate, wait_for_robot_ready


def main(host: str = "localhost", hz: float = 125.0, duration: float = 30.0):
    print(f"Connecting to RTDE at {host} ...")
    iface = RtdeInterface(host=host, frequency=hz, verbose=True)

    if not iface.connect():
        print("Failed to connect. Is URSim running with RTDE port forwarded (usually -p 30004:30004)?")
        return 1

    if not wait_for_robot_ready(iface):
        print("Robot did not report a valid pose. Check Polyscope is in RUNNING / REMOTE mode.")
        iface.disconnect()
        return 1

    print("Robot ready. Running zero-velocity hold loop (safe). Press Ctrl-C to stop.")
    rate = ControlRate(hz)
    t0 = time.monotonic()
    count = 0

    try:
        while time.monotonic() - t0 < duration:
            state = iface.get_state()

            # Example: print occasionally
            if count % int(hz) == 0:  # once per second
                print(f"t={state.timestamp:.1f}  pose={np.round(state.tcp_pose, 3)}")
                print(f"       wrench={np.round(state.tcp_force, 1)}")

            # Send zero velocity (Cartesian) to hold position under external control
            # speedL with zero vel tells the controller "stop moving" while in external mode
            iface.send_speedL(np.zeros(6), acceleration=0.3, t=0.002)

            rate.sleep()
            count += 1

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        print("Stopping...")
        iface.safe_stop()
        iface.disconnect()
        print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
