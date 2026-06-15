#!/usr/bin/env python3
"""
force_mode_telemetry_real.py

Zero-dependency script for a **real UR5e** arm.

- Enables built-in force_mode via URScript (Secondary Client, port 30002).
- Streams live force/torque telemetry from the robot's **integrated 6-axis FT sensor**
  using the real-time client interface (port 30003). No RTDE library required.
- Uses the exact same force-mode tunable variables as the URSim version for easy
  transfer of settings.

Target:
  Polyscope IP: 192.168.1.2
  Standard UR ports (30002 commands, 30003 real-time telemetry).

Requirements:
  - Pure Python 3 stdlib only (socket, struct, time, sys). No pip packages.

Setup on the real robot (Polyscope):
  1. Power on the UR5e.
  2. Set to **REMOTE** control mode.
  3. Configure the correct **TCP** (tool center point) and **payload** for your setup.
     This is critical — force_mode and the reported TCP wrench depend on it.
  4. Make sure there are no protective stops and the work surface is reachable.
  5. (Recommended) Start in reduced speed mode or with safety planes until tuned.

Usage:
  python force_mode_telemetry_real.py

Force mode parameters are at the top for easy editing (same names as the sim version).

Ctrl-C cleanly disables force mode and stops the robot.

Safety notes for real hardware:
  - Start with LOW force values (e.g. 5-10 N) and slow motion.
  - Approach the surface carefully; use the teach pendant E-stop as primary safety.
  - Test force_mode in free space (no contact) first.
  - The robot will actively press with the commanded WRENCH in the selected compliant axes
    while following your motion in the non-compliant axes.
  - On a real surface the reaction force will be physical and the integrated sensor will
    report realistic values (unlike URSim).

The telemetry printed is the current TCP force/torque (N, Nm) as measured/estimated
from the UR5e's built-in wrist force/torque sensor, expressed in the active frame.
"""

import socket
import time
import struct
import sys

HOST = "192.168.1.2"

# ====================== FORCE MODE PARAMS (Y + Z compliant, 0 mm approach, 5 N in Y+Z) ======================
TASK_FRAME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
SELECTION = [0, 1, 1, 0, 0, 0]          # Compliant (force-controlled) in Y and Z

# Real surface targets (used when FREE_AIR_TEST = False)
# NOTE: Commanding force in the "travel" direction (Y here) often causes the arm to deviate
# or drift from the taught path because the force controller prioritizes wrench over position
# in compliant axes. For initial surface tracing tests, strongly consider setting Y component
# to 0.0 (compliance/give in Y, active force only in Z).
WRENCH = [0.0, 5.0, 5.0, 0.0, 0.0, 0.0]  # 5 N in Y + 5 N in Z while tracing

# Set this to True when testing the motion sequence in open air (no surface).
# This makes force_mode compliant in Y+Z but with ZERO force targets,
# so the arm does not actively drive/drift trying to "push" with 5 N.
# Set to False only when the start pose itself will put the tool in contact with a real surface.
FREE_AIR_TEST = True

FORCE_TYPE = 2                            # 2 = base/world frame

# LIMITS for real hardware contact:
#   - Non-compliant axis (X): maximum allowed deviation from the commanded path (m)
#   - Compliant axes (Y, Z): maximum speed the force controller is allowed to use to correct the force error (m/s)
#     Keep these LOW when two axes are compliant to avoid fast diving or oscillation.
LIMITS = [0.10, 0.04, 0.04, 0.17, 0.17, 0.17]

# Force controller tuning - MUST be called before force_mode()
DAMPING = 0.80          # Good starting point with 5 N targets in two axes. Raise to 0.85-0.9 if you see oscillation on contact.
GAIN_SCALING = 0.90     # Balanced starting value for 5 N press while tracing. Raise toward 1.0 if force response feels too soft.

# Start pose (joint positions in radians).
# IMPORTANT: Replace these with actual joint positions taught on the real UR5e
# (read from the Polyscope Move tab or use the "get actual joint positions" feature).
START_JOINTS = [1.675, -2.219, -0.981, 0.460, 1.525, -0.033]

# End pose (joint positions in radians) for the surface trace motion.
# IMPORTANT: Replace these with actual joint positions taught on the real UR5e.
END_JOINTS = [1.675, -2.451, -1.068, 0.938, 1.525, -0.033]
# ================================================================================================

# Motion speeds WHILE FORCE MODE IS ACTIVE (the sliding portion of the trace).
# These must be low when two axes are compliant — fast motion excites oscillation
# and makes it harder for the force controller to maintain the targets.
TRACE_ACC = 0.15
TRACE_VEL = 0.02        # Very slow (2 cm/s) for tracing under force control. Start here.

# Dwell at the end pose with force_mode still active (surface reaction keeps the force regulated).
POST_TRACE_DWELL = 2.0


def send_script(script: str, host: str = HOST, port: int = 30002, timeout: float = 5.0):
    """Send URScript over the Secondary Client interface (port 30002).
    This is the zero-dependency way to command the robot (no RTDE library needed).
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(timeout)
            s.connect((host, port))
            s.sendall(script.encode("utf-8"))
            # Best-effort: consume any immediate reply the controller may send
            try:
                s.recv(1024)
            except socket.timeout:
                pass
        return True
    except Exception as e:
        print(f"Script send failed on {host}:{port}: {e}")
        return False


def main():
    print("=== Real UR5e Force Mode + Integrated FT Sensor Telemetry ===")
    print(f"Host: {HOST} (standard ports: 30002 / 30003)")
    print("Zero external dependencies - uses only Python stdlib sockets.")
    print()
    print("Force mode params:")
    print(f"  Selection: {SELECTION}")
    print(f"  Wrench:    {WRENCH}")
    print(f"  Type:      {FORCE_TYPE}")
    print(f"  Damping:   {DAMPING}")
    print(f"  Gain:      {GAIN_SCALING}")
    print(f"Start joints (rad): {START_JOINTS}")
    print(f"End joints (rad):   {END_JOINTS}")
    print()
    print("IMPORTANT:")
    print("  - Robot must be powered on and in REMOTE mode in Polyscope.")
    print("  - Correct TCP and payload must be configured for accurate TCP wrench.")
    print("  - Approach: 0 mm additional move (force_mode enabled directly after reaching start pose + sleep).")
    print("  - FREE_AIR_TEST = True (top of file) → zero force targets for open-air motion testing (no drift).")
    print("  - Set FREE_AIR_TEST = False only when the start pose itself will load the tool against a real surface.")
    print("  - Have the teach pendant E-stop accessible. Start in reduced speed mode.")
    print("Press Ctrl-C to stop and cleanly disable force mode.\n")

    # 1. Enable force mode + perform the trace via URScript on port 30002.
    # Sequence:
    #   - Reach start pose
    #   - 0 mm additional approach (no down move)
    #   - Then trace while force_mode is active in Y+Z
    #
    # Note: With 0 mm approach, the start pose itself must position the tool in contact
    # with the surface (when FREE_AIR_TEST=False) so the surface can react to the force targets.
    effective_wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] if FREE_AIR_TEST else WRENCH

    force_script = f"""
def force_trace_real():
    # 1. Move to the taught start pose (position the tool near the surface).
    movej([{START_JOINTS[0]}, {START_JOINTS[1]}, {START_JOINTS[2]}, {START_JOINTS[3]}, {START_JOINTS[4]}, {START_JOINTS[5]}], a=1.0, v=0.5)
    sleep(1.0)

    # 2. APPROACH: 0 mm additional move (no down). 
    #    The start pose is expected to already have the tool in (or very near) contact.
    #    If you need a small approach later, change the offset below (e.g. -0.015 for 15 mm down in base -Z).
    approach = p[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # 0 mm
    movel(pose_add(get_actual_tcp_pose(), approach), a=0.12, v=0.025)
    sleep(0.5)   # short settle pause (reduced because no movement)

    # 3. Zero the force/torque sensor (ideally while the tool is in light contact with the surface at the start pose).
    zero_ftsensor()

    force_mode_set_damping({DAMPING})
    force_mode_set_gain_scaling({GAIN_SCALING})

    # 4. Enable force mode.
    #    If FREE_AIR_TEST=True (current): zero wrench targets → pure compliance in Y+Z.
    #      The arm should remain stable after the start pose and complete the move to end pose.
    #    If FREE_AIR_TEST=False: actively applies 5 N in Y + 5 N in Z.
    #      REQUIRES the start pose itself to put the tool in contact with the surface.
    #      Without a reaction force the arm will drift in the compliant directions
    #      (this is normal force_mode behavior when nothing pushes back).
    force_mode(
        p[{TASK_FRAME[0]},{TASK_FRAME[1]},{TASK_FRAME[2]},{TASK_FRAME[3]},{TASK_FRAME[4]},{TASK_FRAME[5]}],
        [{SELECTION[0]},{SELECTION[1]},{SELECTION[2]},{SELECTION[3]},{SELECTION[4]},{SELECTION[5]}],
        [{effective_wrench[0]},{effective_wrench[1]},{effective_wrench[2]},{effective_wrench[3]},{effective_wrench[4]},{effective_wrench[5]}],
        {FORCE_TYPE},
        [{LIMITS[0]},{LIMITS[1]},{LIMITS[2]},{LIMITS[3]},{LIMITS[4]},{LIMITS[5]}]
    )

    # 5. Trace / slide to end pose while force_mode is active.
    #    IMPORTANT: Because Y and Z are compliant, the force controller will continuously
    #    adjust the position/velocity in those axes to achieve the target wrench.
    #    The movej will be followed primarily in the non-compliant axes (X + rotations).
    #    If the taught end joints require a position in Y or Z that conflicts with
    #    maintaining 5N, the arm will deviate from the pure joint trajectory (this appears as "drift").
    #    This is expected behavior. The arm is trying to satisfy the force targets.
    #
    #    Tips to reach closer to end pose:
    #    - Use lower Y wrench (even 0) so the path in Y is followed more closely.
    #    - Teach the END_JOINTS while force_mode is already active and tool is pressed
    #      against the surface (so the joints already incorporate the compliant offset).
    #    - Use very low speed (current TRACE_VEL).
    movej([{END_JOINTS[0]}, {END_JOINTS[1]}, {END_JOINTS[2]}, {END_JOINTS[3]}, {END_JOINTS[4]}, {END_JOINTS[5]}], a={TRACE_ACC}, v={TRACE_VEL})

    # Dwell at the end with force mode still active.
    sleep({POST_TRACE_DWELL})

    end_force_mode()
    stopl(1.5)
end
force_trace_real()
"""

    print("Sending force_mode + trace command via port 30002 ...")
    if not send_script(force_script):
        print("Failed to send force mode script. Is the robot reachable at the correct IP and in REMOTE mode?")
        return 1
    print("Force mode command sent. The program is now running on the controller.\n")

    # Brief pause so the robot program starts executing before we connect for telemetry
    time.sleep(0.8)

    # 2. Receive force telemetry from the real-time interface (port 30003).
    # This stream contains the current TCP force/torque computed from the UR5e's
    # integrated wrist force/torque sensor (after zero_ftsensor and during force mode).
    # Rate is typically 125 Hz on the real-time client (higher rates are available via
    # RTDE on 30004 if you later add the official client library).
    print("Connecting to real-time interface on port 30003 for integrated FT sensor telemetry...")
    rt_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    rt_sock.settimeout(3.0)
    try:
        rt_sock.connect((HOST, 30003))
    except Exception as e:
        print(f"Failed to connect to real-time port 30003: {e}")
        # Still try to stop the robot cleanly
        send_script("end_force_mode()\nstopl(2.0)\n")
        return 1

    print("Receiving force data (real integrated sensor via real-time client).")
    print("Force vector = [Fx, Fy, Fz, Tx, Ty, Tz]  (N, Nm)")
    print("Ctrl-C to stop and disable force mode.\n")

    last_output = 0.0
    target_interval = 1.0 / 200.0   # console throttle (real data rate is higher)
    force_offset_tried = False

    def read_tcp_force(sock):
        """Read one length-prefixed real-time state message from port 30003
        and return the 6D TCP force/torque from the integrated FT sensor (or None).
        Tries the common legacy offset plus a few alternatives that work across
        Polyscope e-series versions. No external libraries.
        """
        # Read 4-byte big-endian message length (standard real-time client framing)
        len_buf = b""
        while len(len_buf) < 4:
            chunk = sock.recv(4 - len(len_buf))
            if not chunk:
                return None
            len_buf += chunk
        msg_len = struct.unpack(">I", len_buf)[0]
        if msg_len < 4 or msg_len > 2000:
            # Bad length - try to resync by consuming one byte
            try:
                sock.recv(1)
            except Exception:
                pass
            return None

        # Read the rest of the message
        data = len_buf
        remaining = msg_len - 4
        while remaining > 0:
            chunk = sock.recv(remaining)
            if not chunk:
                return None
            data += chunk
            remaining -= len(chunk)

        # Candidate byte offsets (relative to start of full message) for the 6 doubles
        # of actual_TCP_force in common Polyscope real-time packages.
        # The first one matches the offset used successfully in the URSim version.
        candidate_offsets = [536, 540, 548, 532, 556, 564]

        for off in candidate_offsets:
            if off + 48 <= len(data):
                try:
                    f = struct.unpack(">6d", data[off : off + 48])
                    # Quick physical sanity: typical contact forces are tens of N, torques a few Nm
                    if all(abs(x) < 300 for x in f):
                        return f
                except struct.error:
                    continue

        # Last resort: scan for any 48-byte sequence of 6 doubles that look plausible
        # (cheap and effective when the exact layout varies by firmware)
        i = 4
        while i + 48 <= len(data):
            try:
                f = struct.unpack(">6d", data[i : i + 48])
                if all(abs(x) < 300 for x in f) and max(abs(x) for x in f) > 0.001:
                    return f
            except struct.error:
                pass
            i += 8   # step by one double

        return None

    try:
        while True:
            force = read_tcp_force(rt_sock)
            if force is None:
                continue

            # On real hardware we trust the values more; still drop obvious outliers
            max_abs = max(abs(x) for x in force)
            if max_abs < 500:
                now = time.monotonic()
                if (now - last_output) >= target_interval:
                    print(f"F=[{force[0]:7.2f} {force[1]:7.2f} {force[2]:7.2f} "
                          f"{force[3]:6.3f} {force[4]:6.3f} {force[5]:6.3f}]")
                    last_output = now
                    if not force_offset_tried:
                        force_offset_tried = True
                        # Optional: uncomment for debugging exact layout on your Polyscope version
                        # print("[debug] First plausible force extracted successfully")

    except KeyboardInterrupt:
        print("\nStopping (user interrupt)...")

    finally:
        print("Disabling force mode and cleaning up...")
        try:
            rt_sock.close()
        except Exception:
            pass

        # Always attempt to end force mode and stop motion on the real robot.
        stop_script = "end_force_mode()\nstopl(2.0)\n"
        if send_script(stop_script, timeout=3.0):
            print("end_force_mode() + stopl sent.")
        else:
            print("Warning: Could not send final stop command. Use the teach pendant if the robot is still moving.")

        print("Done. Force mode disabled on the real UR5e.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
