#!/usr/bin/env python3
"""
force_mode_example.py - SIMPLE, ZERO-DEPENDENCY version for URSim testing

Goal: Let you quickly enable built-in forceMode in URSim from Python with
**no extra packages to install**, no cmake, no Boost, no compilation hell.

This directly answers your question: "Why do I need to install rtde on the
client side if the script is just making calls to it over the UrSim port?"

Short answer:
- For a simple "turn force mode on for 15 seconds" test, you DON'T.
  The current script uses the Secondary Client (port 30002) which just
  accepts raw URScript text. Pure socket, no library needed.

- For real external control loops (the actual goal of this project:
  high-rate admittance/impedance controllers that read wrench/pose at
  125-500 Hz and stream speedj/servoj commands in a tight loop), you DO
  need a proper RTDE client library on the host.

Why a library is normally required:
UR robots expose several interfaces. The important one for external
closed-loop control is **RTDE on port 30004**. RTDE is not "send text".
It is a binary protocol where you:
  1. Negotiate "recipes" (exactly which variables you want to receive
     and at what frequency).
  2. Send structured control commands (forceMode, speedL, servoJ, etc.).
  3. Keep a synchronized, high-rate bidirectional stream alive.

The `ur_rtde` package (the one that was painful to build) is a convenient
high-level wrapper around that protocol. It gives you nice calls like
`rtde_c.forceMode(...)` and `rtde_r.getActualTCPForce()` and handles all
the binary packing, threading, and error handling for you.

The heavy C++ dependency exists because that particular package chose to
wrap the official UR Client Library (URCL) for performance.

In this file we cheat for the simple test by using port 30002 (Secondary
Client). That port lets any client send URScript. Inside the script we
just call the URScript function `force_mode(...)`. No protocol negotiation
needed → no library needed.

For the real controllers/ and examples/ in this repo, we want proper
RTDE because:
- We need continuous high-rate feedback while we are moving.
- We want to run a Python control loop that reacts to force every 8 ms.
- Text-script-sending over 30002 is not designed for that use case.

**Direct answer to your question:**

**No. You do not need to "install and compile RTDE".**

RTDE is a protocol that is **already implemented inside the controller**
(the real UR5e or the URSim software running in your Docker container).
Nothing to install or compile on the "server" side.

On your development machine (the *client*), you need software that speaks
the RTDE protocol over the network to that controller.

You have choices for that client software:
- For simple tests: don't use RTDE. The current script uses the Secondary
  Client (port 30002) with plain sockets — zero packages required.
- For real external RTDE control loops: use an RTDE client library on your
  PC. The official one from Universal Robots is pure Python (no compilation
  needed). The `ur_rtde` package we struggled with is just one (very nice
  but heavy) choice of client library.

So the "RTDE" piece on the client side is just the *client library*, not
RTDE itself. The controller already has RTDE.

The current zero-dependency script is perfect for what you asked ("I just
want to try and enable force mode through the URSim").

Run it with plain `python3 force_mode_example.py`. No venv required for
this particular file.
"""

import socket
import time
import sys

HOST = "localhost"

# ==================== EASY CONFIGURATION ====================
# Edit these to experiment, exactly like the old forceMode() call.

TASK_FRAME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
SELECTION_VECTOR = [0, 0, 1, 0, 0, 0]     # Only Z compliant
WRENCH = [0.0, 0.0, 10.0, 0.0, 0.0, 0.0]  # 10 N push in Z
FORCE_TYPE = 2
LIMITS = [0.1, 0.1, 0.15, 0.17, 0.17, 0.17]

DURATION = 15.0  # How long to keep force mode active (seconds)
# ================================================================


def send_ur_script(host: str, script: str):
    """Send URScript to the Secondary Client (port 30002).
    This is the simplest way to command the robot without any libraries.
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(5.0)
            s.connect((host, 30002))
            s.sendall(script.encode("utf-8"))
            # The interface often replies with a short status. We can ignore it for this test.
            try:
                _ = s.recv(1024)
            except socket.timeout:
                pass
        return True
    except Exception as e:
        print(f"Failed to send script: {e}")
        return False


def main():
    print("=" * 72)
    print("URSim Force Mode test — ZERO external dependencies version")
    print("=" * 72)
    print("This script requires NOTHING except standard Python.")
    print("No ur_rtde, no cmake, no Boost, no pip packages.")
    print()

    # Build a tiny URScript that does exactly what the old rtde_c.forceMode() did
    # Note the exact parameter order: force_mode(task_frame, selection, wrench, type, limits)
    script = f"""
def force_mode_test():
  # Enable force mode
  force_mode(p[{TASK_FRAME[0]},{TASK_FRAME[1]},{TASK_FRAME[2]},{TASK_FRAME[3]},{TASK_FRAME[4]},{TASK_FRAME[5]}],
             [{SELECTION_VECTOR[0]},{SELECTION_VECTOR[1]},{SELECTION_VECTOR[2]},{SELECTION_VECTOR[3]},{SELECTION_VECTOR[4]},{SELECTION_VECTOR[5]}],
             [{WRENCH[0]},{WRENCH[1]},{WRENCH[2]},{WRENCH[3]},{WRENCH[4]},{WRENCH[5]}],
             {FORCE_TYPE},
             [{LIMITS[0]},{LIMITS[1]},{LIMITS[2]},{LIMITS[3]},{LIMITS[4]},{LIMITS[5]}])

  # Keep it active so you can observe in Polyscope
  sleep({DURATION})

  # Clean exit
  end_force_mode()
  stopl(1.0)
end
force_mode_test()
"""

    print("Parameters being used:")
    print(f"  Selection (compliant axes): {SELECTION_VECTOR}")
    print(f"  Wrench target: {WRENCH}")
    print(f"  Duration: {DURATION} seconds")
    print()
    print(">>> Sending force mode script to URSim (port 30002)...")

    if not send_ur_script(HOST, script):
        print("Could not connect. Is URSim running with port 30002 published?")
        print("Use: docker run ... -p 30002:30002 ... (or the full port range)")
        return 1

    print("Script sent successfully.")
    print()
    print(">>> FORCE MODE SHOULD NOW BE ACTIVE IN URSim <<<")
    print("Open (or refresh) your Polyscope UI:")
    print("  http://localhost:6080")
    print()
    print("Look for:")
    print("  - Force mode indicator / icon in the status area")
    print("  - The robot behaving compliantly on the selected axis (Z by default)")
    print()
    print(f"The mode will automatically turn off after ~{DURATION} seconds.")
    print("You can also stop it early from Polyscope if you want.")
    print()

    # Give the user time to switch windows and observe
    time.sleep(DURATION + 2)

    print("Done. If you didn't see force mode activate, make sure:")
    print("  - The container was started with the -p port forwards (see start_ursim.sh)")
    print("  - The robot is powered on and in REMOTE mode inside Polyscope")
    print("  - Try different SELECTION_VECTOR / WRENCH values and re-run this script")
    print()

    print("Once you're happy that force mode works in the simulator, you can move on to")
    print("the real external controllers (those still benefit from the full ur_rtde package).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
