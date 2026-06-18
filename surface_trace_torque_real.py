#!/usr/bin/env python3
"""
Launch surface_trace_torque_impedance.script on the real UR5e and stream FT telemetry.

Standalone script — no project imports required.

Robot connection (from force_mode_telemetry_real.py):
  HOST          = 192.168.1.2
  SCRIPT_PORT   = 30002   URScript / program upload
  REALTIME_PORT = 30003   integrated FT sensor telemetry
  RTDE_PORT     = 30004   RTDE (not used here; listed for reference)

Requirements:
  - Robot powered on, REMOTE mode, correct TCP/payload configured
  - Polyscope 5.22+ with direct_torque() support

Usage:
  python surface_trace_torque_real.py

Ctrl-C sends stopj() to the controller.
"""

import socket
import struct
import sys
import time
from pathlib import Path

# ====================== ROBOT CONNECTION (force_mode_telemetry_real.py) ======================
HOST = "192.168.1.2"
SCRIPT_PORT = 30002
REALTIME_PORT = 30003
RTDE_PORT = 30004
# ============================================================================================

SCRIPT_FILE = Path(__file__).resolve().parent / "surface_trace_torque_impedance.script"


def send_script(script: str, host: str = HOST, port: int = SCRIPT_PORT, timeout: float = 5.0) -> bool:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(timeout)
            s.connect((host, port))
            s.sendall(script.encode("utf-8"))
            try:
                s.recv(1024)
            except socket.timeout:
                pass
        return True
    except Exception as e:
        print(f"Script send failed on {host}:{port}: {e}")
        return False


def read_tcp_force(sock: socket.socket):
    """Read TCP wrench from real-time port 30003 (same logic as force_mode_telemetry_real.py)."""
    len_buf = b""
    while len(len_buf) < 4:
        chunk = sock.recv(4 - len(len_buf))
        if not chunk:
            return None
        len_buf += chunk
    msg_len = struct.unpack(">I", len_buf)[0]
    if msg_len < 4 or msg_len > 2000:
        try:
            sock.recv(1)
        except Exception:
            pass
        return None

    data = len_buf
    remaining = msg_len - 4
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            return None
        data += chunk
        remaining -= len(chunk)

    for off in (536, 540, 548, 532, 556, 564):
        if off + 48 <= len(data):
            try:
                f = struct.unpack(">6d", data[off : off + 48])
                if all(abs(x) < 300 for x in f):
                    return f
            except struct.error:
                continue

    i = 4
    while i + 48 <= len(data):
        try:
            f = struct.unpack(">6d", data[i : i + 48])
            if all(abs(x) < 300 for x in f) and max(abs(x) for x in f) > 0.001:
                return f
        except struct.error:
            pass
        i += 8
    return None


def main() -> int:
    print("=== Surface Trace (Torque Impedance) — Real UR5e ===")
    print(f"Host:          {HOST}")
    print(f"Script port:   {SCRIPT_PORT}")
    print(f"Realtime port: {REALTIME_PORT}")
    print(f"RTDE port:     {RTDE_PORT}")
    print()
    print("Robot must be in REMOTE mode with correct TCP/payload.")
    print("Press Ctrl-C to stop.\n")

    if not SCRIPT_FILE.is_file():
        print(f"ERROR: Script not found: {SCRIPT_FILE}")
        return 1

    urscript = SCRIPT_FILE.read_text(encoding="utf-8")
    print(f"Sending {SCRIPT_FILE.name} via port {SCRIPT_PORT} ...")
    if not send_script(urscript):
        print("Failed to send script. Check IP, network, and REMOTE mode.")
        return 1
    print("Script sent. Controller is executing the trace program.\n")

    time.sleep(0.8)

    print(f"Connecting to real-time interface on port {REALTIME_PORT} ...")
    rt_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    rt_sock.settimeout(3.0)
    try:
        rt_sock.connect((HOST, REALTIME_PORT))
    except Exception as e:
        print(f"Failed to connect to port {REALTIME_PORT}: {e}")
        send_script("stopj(2.0)\n", timeout=3.0)
        return 1

    print("Streaming TCP force [Fx Fy Fz Tx Ty Tz] (N, Nm). Ctrl-C to stop.\n")

    last_output = 0.0
    target_interval = 1.0 / 200.0

    try:
        while True:
            force = read_tcp_force(rt_sock)
            if force is None:
                continue
            now = time.monotonic()
            if (now - last_output) >= target_interval:
                print(
                    f"F=[{force[0]:7.2f} {force[1]:7.2f} {force[2]:7.2f} "
                    f"{force[3]:6.3f} {force[4]:6.3f} {force[5]:6.3f}]"
                )
                last_output = now
    except KeyboardInterrupt:
        print("\nStopping (user interrupt)...")
    finally:
        try:
            rt_sock.close()
        except Exception:
            pass
        if send_script("stopj(2.0)\n", timeout=3.0):
            print("stopj(2.0) sent.")
        else:
            print("Warning: could not send stopj. Use teach pendant if needed.")
        print("Done.")

    return 0


if __name__ == "__main__":
    sys.exit(main())