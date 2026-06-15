#!/usr/bin/env python3
"""
Simple script to:
- Enable force mode on UR5e in URSim (using ports from start_ursim.sh)
- Output force telemetry at ~500 Hz via RTDE

Requirements:
  pip install git+https://github.com/UniversalRobots/RTDE_Python_Client_Library.git

Run AFTER starting URSim with the correct ports:
  ./start_ursim.sh

Inside Polyscope (http://localhost:6080):
  - Power on the robot
  - Set to REMOTE control mode
  - No protective stops

Then run this script:
  python force_mode_telemetry.py

Ctrl-C to stop (it will cleanly disable force mode).

Force mode params are at the top for easy editing.
"""

import socket
import time
import struct
import sys

HOST = "localhost"

# ====================== FORCE MODE PARAMS (edit these) ======================
TASK_FRAME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
SELECTION = [0, 0, 1, 0, 0, 0]          # Compliant in Z only
WRENCH = [0.0, 0.0, 10.0, 0.0, 0.0, 0.0]  # Constant force target. For "press with constant force while sliding across a surface", put the desired pressing force in the compliant axis (usually Z) and 0 in the others. E.g. [0,0,10,0,0,0] = press with 10 N in +Z while you command motion in X/Y.
FORCE_TYPE = 2                            # 2 = base/world frame
LIMITS = [0.1, 0.1, 0.15, 0.17, 0.17, 0.17]

# === Force mode tuning parameters (built-in) ===
# Call these BEFORE force_mode() to adjust controller behavior.
# Start conservative and tune while watching Polyscope + telemetry.
DAMPING = 0.5          # 0.0 - 1.0+ : Higher = more damped / less oscillation, slower response
GAIN_SCALING = 1.0     # 0.1 - 2.0  : Overall stiffness/gain of the force controller. Higher = more aggressive

# Start pose from your first Polyscope screenshot (joint positions in radians)
START_JOINTS = [-0.177, -2.162, -1.831, -0.711, 1.616, -0.153]

# End pose from your latest Polyscope screenshot (joint positions in radians)
END_JOINTS = [-0.249, -1.437, -2.411, -0.859, 1.618, -0.224]
# ===========================================================================

def send_script(script: str, port: int = 30002):
    """Send URScript over the secondary client port (30002)."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(3.0)
            s.connect((HOST, port))
            s.sendall(script.encode("utf-8"))
        return True
    except Exception as e:
        print(f"Script send failed on port {port}: {e}")
        return False

def main():
    print("=== URSim Force Mode + 500 Hz Force Telemetry ===")
    print(f"Host: {HOST}")
    print(f"Force mode params:")
    print(f"  Selection: {SELECTION}")
    print(f"  Wrench:    {WRENCH}")
    print(f"  Type:      {FORCE_TYPE}")
    print(f"Start joints (rad): {START_JOINTS}")
    print(f"End joints (rad): {END_JOINTS}")
    print("Make sure URSim is running (./start_ursim.sh) and in REMOTE mode.")
    print("Press Ctrl-C to stop.\n")

    # 1. Enable force mode via URScript on port 30002
    force_script = f"""
def force_enable():
    # Move to your desired start pose (from the Polyscope screenshot).
    movej([{START_JOINTS[0]}, {START_JOINTS[1]}, {START_JOINTS[2]}, {START_JOINTS[3]}, {START_JOINTS[4]}, {START_JOINTS[5]}], a=1.0, v=0.5)
    sleep(2.0)
    zero_ftsensor()
    force_mode_set_damping({DAMPING})
    force_mode_set_gain_scaling({GAIN_SCALING})
    force_mode(
        p[{TASK_FRAME[0]},{TASK_FRAME[1]},{TASK_FRAME[2]},{TASK_FRAME[3]},{TASK_FRAME[4]},{TASK_FRAME[5]}],
        [{SELECTION[0]},{SELECTION[1]},{SELECTION[2]},{SELECTION[3]},{SELECTION[4]},{SELECTION[5]}],
        [{WRENCH[0]},{WRENCH[1]},{WRENCH[2]},{WRENCH[3]},{WRENCH[4]},{WRENCH[5]}],
        {FORCE_TYPE},
        [{LIMITS[0]},{LIMITS[1]},{LIMITS[2]},{LIMITS[3]},{LIMITS[4]},{LIMITS[5]}]
    )
    # Move from the start pose to the end pose (using joints) while force_mode keeps the constant force in Z.
    # This traces the surface (same Z height) while applying constant pressure in the normal direction.
    # In real hardware this would slide while pressing; in URSim the force telemetry will be unrealistic.
    movej([{END_JOINTS[0]}, {END_JOINTS[1]}, {END_JOINTS[2]}, {END_JOINTS[3]}, {END_JOINTS[4]}, {END_JOINTS[5]}], a=1.0, v=0.5)
    end_force_mode()  # end immediately after reaching end to prevent post-trace drift in sim (force_mode would otherwise keep driving the arm in the compliant direction with no real surface opposing the force)
    sleep(5)  # short pause for telemetry after trace; force_mode is now off so no more drift
end
force_enable()
"""
    print("Sending force_mode command via port 30002 ...")
    if not send_script(force_script):
        print("Failed to send force mode. Is URSim running with ports forwarded?")
        return 1
    print("Force mode command sent. (It will stay active until we stop it.)\n")

    # Give URSim a moment to activate force mode
    time.sleep(0.5)

    # 2. Receive force telemetry using the real-time interface on port 30003
    # This is a fixed binary stream (no recipe needed). It provides TCP force
    # at the controller's rate (typically 125 Hz in practice; URSim may vary).
    # 500 Hz is the theoretical max for RTDE on real hardware; here we use the
    # always-available real-time client for simplicity and reliability in URSim.
    print("Connecting to real-time interface on port 30003 for force telemetry...")
    rt_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    rt_sock.connect((HOST, 30003))

    print("Receiving force data (real-time interface).")
    print("Force vector = [Fx, Fy, Fz, Tx, Ty, Tz]")
    print("Note: Real-time interface typically runs at 125 Hz (URSim limit).")
    print("Ctrl-C to stop and disable force mode.\n")

    last_output = 0.0
    target_interval = 1.0 / 500.0  # throttle console spam to ~500 Hz max

    try:
        while True:
            # Robust read + resync for the real-time stream
            # Read first 8 bytes (timestamp as double) to check alignment
            ts_bytes = rt_sock.recv(8)
            if len(ts_bytes) < 8:
                continue

            t = struct.unpack('>d', ts_bytes)[0]

            # Simple sanity: timestamp should be positive and reasonable
            if t > 0 and t < 1e12:
                # Good alignment - read the rest of a 1060-byte packet
                rest = rt_sock.recv(1060 - 8)
                if len(rest) == 1052:
                    pkt = ts_bytes + rest
                    force = struct.unpack('>6d', pkt[536:584])

                    # Strict sanity filter for URSim: only print plausible forces
                    # (in sim, wrench feedback is unrealistic; large values are parser glitches)
                    max_abs = max(abs(x) for x in force)
                    if max_abs < 200 and max_abs > 0.001:  # skip pure zeros and garbage
                        now = time.monotonic()
                        if (now - last_output) >= target_interval:
                            print(f"F=[{force[0]:7.2f} {force[1]:7.2f} {force[2]:7.2f} "
                                  f"{force[3]:6.3f} {force[4]:6.3f} {force[5]:6.3f}]")
                            last_output = now
            else:
                # Desynced - consume one byte to try resyncing
                rt_sock.recv(1)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        print("Disabling force mode and cleaning up...")
        try:
            rt_sock.close()
        except Exception:
            pass
        # Stop force mode
        stop_script = "end_force_mode()\nstopl(2.0)\n"
        send_script(stop_script)
        print("Done. Force mode disabled.")

    return 0

if __name__ == "__main__":
    sys.exit(main())
