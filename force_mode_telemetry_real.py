#!/usr/bin/env python3
"""
force_mode_telemetry_real.py

- Enables built-in force_mode via URScript (Secondary Client, port 30002) with a
  Cartesian constant-height slide (movel) while Z force is regulated.
- Streams live force/torque telemetry from the robot's integrated 6-axis FT sensor
  using the real-time client interface (port 30003). No RTDE library required.

Target:
  Polyscope IP: 192.168.1.2
  Standard UR ports (30002 commands, 30003 real-time telemetry).

Requirements:
  - Python 3

Setup on the real robot (Polyscope):
  1. Power on the UR5e.
  2. Set to **REMOTE** control mode.
  3. Configure the correct **TCP** (tool center point) and **payload** for your setup.
     This is critical — force_mode and the reported TCP wrench depend on it.
  4. Make sure there are no protective stops and the work surface is reachable.
  5. (Recommended) Start in reduced speed mode or with safety planes until tuned.

Usage:
  python force_mode_telemetry_real.py

Safety notes for real hardware:
  - Using TOOL frame so EE Z (per Polyscope) = compliant normal to table.
  - Trace in tool Y (back/forward) at constant height.
  - After trace + dwell: end_force_mode() then movej(RETRACT_JOINTS) -- you MUST edit RETRACT_JOINTS at top with a safe raised pose (jog arm up after trace, record joints).
  - Start with LOW force (3-5 N) and slow TRACE_VEL.
  - Test trace direction and force sign with FREE_AIR_TEST=True first.
  - Teach start pose with tool near contact.
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

# ====================== FORCE MODE PARAMS (Z-compliant surface tracing) ======================
TASK_FRAME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
SELECTION = [0, 0, 1, 0, 0, 0]

# SELECTION indexing for force_mode():
# With FORCE_TYPE=1 (tool frame), the indices are in the END-EFFECTOR frame (as shown in Polyscope):
#   [0] = X   (linear)   -- left/right per your description
#   [1] = Y   (linear)   -- back/forward per your description
#   [2] = Z   (linear)   -- perpendicular to table (normal to surface)
#   [3] = Rx  (rotation about X)
#   [4] = Ry  (rotation about Y)
#   [5] = Rz  (rotation about Z)

WRENCH = [0.0, 0.0, 10.0, 0.0, 0.0, 0.0]  

# Set this to True when testing the motion sequence in open air (no surface).
# This makes force_mode compliant in the set axes but with ZERO force targets,
# so the arm does not actively drive/drift trying to "push" with nonzero forces.
# Set to False only when the start pose itself will put the tool in (or very near) contact
# with a real surface so that the force controller can regulate against a real reaction.
FREE_AIR_TEST = False

FORCE_TYPE = 1 # 1 = tool frame (use this because EE Z is the normal to the table)

# LIMITS for real hardware contact:
#   - Non-compliant axis: maximum allowed deviation from the commanded path (m)
#   - Compliant axes: maximum speed the force controller is allowed to use to correct the force error (m/s)
LIMITS = [0.10, 0.04, 0.08 0.17, 0.17, 0.17]

# Force controller tuning - MUST be called before force_mode()
DAMPING = 0.80         
GAIN_SCALING = 0.70    

# Start pose (joint positions in radians)..
START_JOINTS = [1.675, -2.219, -0.981, 0.460, 1.525, -0.033]

# End pose (joint positions in radians) for the surface trace motion.
END_JOINTS = [1.675, -2.451, -1.068, 0.938, 1.525, -0.033]

# Safe retracted pose after tracing (arm raised clear above the table).
# IMPORTANT: Jog the arm UP to a safe retracted height AFTER a trace (with force off), 
# then record the joints here. This must be a raised pose so the arm clears the table.
RETRACT_JOINTS = [1.675, -1.5, -1.2, -0.5, 1.525, -0.033]  # <<< REPLACE WITH YOUR ACTUAL RAISED JOINTS!
# ================================================================================================

# Motion speeds WHILE FORCE MODE IS ACTIVE (the sliding portion of the trace).
TRACE_ACC = 0.01
TRACE_VEL = 0.01

# Lateral trace offset in TOOL / end-effector frame (see Polyscope EE axes).
# X = left/right on table
# Y = back/forward on table   <--- set this (second number) for "back"
# Z must be 0 (to keep commanded motion at constant height; force_mode handles Z)
TRACE_LATERAL_OFFSET = [-0.09, 0.3, 0.0]  # example: 15 cm in tool +Y; use negative for opposite direction

# Dwell at the end pose with force_mode still active.
POST_TRACE_DWELL = 1.0

# Seconds to wait after enabling force_mode (and after zero_ftsensor) for the initial press
# to stabilize before issuing the trace motion command. Prevents the slide from fighting
# the engagement transient.
PRESS_SETTLE_TIME = 1.0


def send_script(script: str, host: str = HOST, port: int = 30002, timeout: float = 5.0):
    """Send URScript over the Secondary Client interface (port 30002).
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
    print(f"  Selection: {SELECTION} (in TOOL frame: Z = normal to table)")
    print(f"  Wrench:    {WRENCH}")
    print(f"  Type:      {FORCE_TYPE} (tool frame)")
    print(f"  Damping:   {DAMPING}")
    print(f"  Gain:      {GAIN_SCALING}")
    print(f"  Trace offset: {TRACE_LATERAL_OFFSET} (in TOOL/EE frame: Y for back/forward on table)")
    print(f"Start joints (rad): {START_JOINTS}")
    print(f"End joints (rad):   {END_JOINTS}")
    print(f"Retract joints (rad): {RETRACT_JOINTS}")
    print()
    print("IMPORTANT:")
    print("  - Robot must be powered on and in REMOTE mode in Polyscope.")
    print("  - Correct TCP and payload must be configured for accurate TCP wrench.")
    print("  - Using TOOL frame (FORCE_TYPE=1): SELECTION[2]=1 means compliance in EE Z (normal to table).")
    print("  - Trace offset in TOOL/EE frame (set Y component for back/forward; Z must stay 0).")
    print("  - After trace + dwell: end_force_mode() then movej(RETRACT_JOINTS) -- set RETRACT_JOINTS (at top) to a safe raised pose to retract the *entire* arm clear of table.")
    print("  - 0 mm approach: force_mode enabled after start pose.")
    print("  - FREE_AIR_TEST = True for testing the trace motion without force.")
    print("  - Have the teach pendant E-stop accessible. Start in reduced speed mode.")
    print("Press Ctrl-C to stop and cleanly disable force mode.\n")

    # 1. Enable force mode + perform the trace via URScript on port 30002.
    # Sequence (executed on the robot):
    #   - Reach start pose (must put tool in light contact for real-surface use)
    #   - 0 mm movel (placeholder)
    #   - zero_ftsensor()
    #   - force_mode in TOOL frame (SELECTION[2]=1 means EE Z = table normal)
    #   - Settle
    #   - speedl in TOOL Y (back/forward) -- straighter tangential motion
    #   - Dwell
    #   - end_force_mode()
    #   - movej(RETRACT_JOINTS) -- retract entire arm to safe raised pose
    #   - stopl
    #
    # Key: compliance (force control) in EE Z (normal to table), commanded velocity in EE Y.
    effective_wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] if FREE_AIR_TEST else WRENCH

    # Precompute for the trace: use TRACE_LATERAL_OFFSET as the desired displacement in TOOL frame
    # (X=left/right, Y=back/forward, Z must be 0)
    offset = TRACE_LATERAL_OFFSET[:3]
    mag = (offset[0]**2 + offset[1]**2 + offset[2]**2) ** 0.5
    if mag < 0.0001:
        mag = 1.0
    unit_x = offset[0] / mag
    unit_y = offset[1] / mag
    unit_z = offset[2] / mag
    trace_speed = TRACE_VEL
    trace_dist = mag
    trace_time = trace_dist / trace_speed if trace_speed > 0 else 1.0

    force_script = f"""
def force_trace_real():
    # 1. Move to the taught start pose (must place tool in light contact for real use).
    movej([{START_JOINTS[0]}, {START_JOINTS[1]}, {START_JOINTS[2]}, {START_JOINTS[3]}, {START_JOINTS[4]}, {START_JOINTS[5]}], a=1.0, v=0.5)
    sleep(1.0)

    # 2. 0 mm approach placeholder (no extra motion).
    approach = p[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    movel(pose_add(get_actual_tcp_pose(), approach), a=0.12, v=0.025)
    sleep(0.5)

    # 3. Zero sensor (while in light contact).
    zero_ftsensor()

    force_mode_set_damping({DAMPING})
    force_mode_set_gain_scaling({GAIN_SCALING})

    # 4. Enable force mode (Z compliant + target wrench).
    textmsg("FORCE_MODE_ENABLED")
    force_mode(
        p[{TASK_FRAME[0]},{TASK_FRAME[1]},{TASK_FRAME[2]},{TASK_FRAME[3]},{TASK_FRAME[4]},{TASK_FRAME[5]}],
        [{SELECTION[0]},{SELECTION[1]},{SELECTION[2]},{SELECTION[3]},{SELECTION[4]},{SELECTION[5]}],
        [{effective_wrench[0]},{effective_wrench[1]},{effective_wrench[2]},{effective_wrench[3]},{effective_wrench[4]},{effective_wrench[5]}],
        {FORCE_TYPE},
        [{LIMITS[0]},{LIMITS[1]},{LIMITS[2]},{LIMITS[3]},{LIMITS[4]},{LIMITS[5]}]
    )

    # Let the initial press build and stabilize before we command lateral motion.
    sleep({PRESS_SETTLE_TIME})
    textmsg("SETTLED_PRESS")

    # 5. Trace with speedl using direction from TRACE_LATERAL_OFFSET in TOOL frame (EE axes).
    #    While force_mode regulates tool Z (normal to surface).
    #    The offset you set (e.g. Y component for back/forward) is transformed to base velocity.
    textmsg("STARTING_TRACE")
    pressed = get_actual_tcp_pose()
    # Direction in TOOL frame from the offset the user set (EE X/Y/Z as per Polyscope)
    dir_tool = p[{unit_x}, {unit_y}, {unit_z}, 0,0,0]
    rot_pose = p[0,0,0, pressed[3],pressed[4],pressed[5]]
    dir_base = pose_trans(rot_pose, dir_tool)
    speed = {trace_speed}
    vx = dir_base[0] * speed
    vy = dir_base[1] * speed
    vz = dir_base[2] * speed   # should be 0 if user set Z=0 in offset
    vel = [vx, vy, vz, 0, 0, 0]
    acc = 0.5
    t = {trace_time}
    speedl(vel, acc, t)
    textmsg("TRACE_COMPLETE")

    # Optional: if you really need the exact END_JOINTS posture *after* tracing under force,
    # you can do the joint move *after* end_force_mode() below instead.

    # Dwell while still under force (good for observing stable regulation).
    sleep({POST_TRACE_DWELL})

    end_force_mode()
    textmsg("RETRACTING")

    # Retract the entire arm up and clear of the table to RETRACT_JOINTS.
    # This is a full joint-space retract (safer than small Cartesian lift after contact/trace).
    # Jog arm UP to clear height AFTER a trace (force off), record joints, and put them in RETRACT_JOINTS at top.
    movej([{RETRACT_JOINTS[0]}, {RETRACT_JOINTS[1]}, {RETRACT_JOINTS[2]}, {RETRACT_JOINTS[3]}, {RETRACT_JOINTS[4]}, {RETRACT_JOINTS[5]}], a=0.5, v=0.2)
    stopl(1.0)
end
force_trace_real()
"""

    print("=== Generated URScript (for debugging - copy if robot errors) ===")
    print(force_script)
    print("=== End of generated URScript ===")

    print("Sending force_mode + trace command via port 30002 ...")
    if not send_script(force_script):
        print("Failed to send force mode script. Is the robot reachable at the correct IP and in REMOTE mode?")
        return 1
    print("Force mode command sent. The program is now running on the controller.\n")

    # Pause so the robot program starts executing before we connect for telemetry
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
