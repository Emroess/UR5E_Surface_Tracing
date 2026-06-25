#!/usr/bin/env python3
#
# UR5e zero additional torque commissioning test.
#
# SAFETY WARNINGS:
# - Run only in free space.
# - Confirm payload and TCP are correct before running.
# - Keep the E-stop accessible.
# - Zero torque here means zero additional commanded torque; UR internally
#   handles gravity compensation.
# - Do not run this in contact with a surface or tool fixture.

import argparse
import socket
import sys
import time


DEFAULT_ROBOT_IP = "192.168.1.2"
DEFAULT_PORT = 30002
DEFAULT_DURATION = 10.0
DEFAULT_SAMPLE_PERIOD = 0.5
SOCKET_TIMEOUT_S = 5.0
SOCKET_CLOSE_DELAY_S = 0.2


def build_urscript(duration_s, use_friction_scales=False):
    if use_friction_scales:
        direct_torque_call = (
            "direct_torque(zero_tau, "
            "viscous_scale=[0.9, 0.9, 0.8, 0.9, 0.9, 0.9], "
            "coulomb_scale=[0.8, 0.8, 0.7, 0.8, 0.8, 0.8])"
        )
    else:
        direct_torque_call = "direct_torque(zero_tau, friction_comp=True)"

    return f"""def ur5e_zero_torque_socket_program():
  def zero_torque_test(duration_s):
    textmsg("Starting zero additional torque test, duration_s=", duration_s)

    elapsed = 0.0
    zero_tau = [0, 0, 0, 0, 0, 0]

    while elapsed < duration_s:
      {direct_torque_call}
      elapsed = elapsed + get_steptime()
    end

    stopj(2.0)
    textmsg("Finished zero additional torque test")
  end

  zero_torque_test({duration_s:.6f})
end
"""


def build_smoke_test_urscript():
    return """def ur5e_socket_smoke_test():
  textmsg("UR5e socket smoke test START")
  sleep(0.25)
  textmsg("UR5e socket smoke test END")
end
"""


def build_ee_state_readout_urscript(duration_s, sample_period_s):
    return f"""def ur5e_ee_state_readout_socket_program():
  def ee_state_readout(duration_s, sample_period_s):
    textmsg("Starting EE state readout, duration_s=", duration_s)

    elapsed = 0.0
    next_sample = 0.0

    while elapsed < duration_s:
      if elapsed >= next_sample:
        tcp_pose = get_actual_tcp_pose()
        tcp_speed = get_actual_tcp_speed()
        tcp_wrench = get_tcp_force()

        textmsg("EE pose [x,y,z,rx,ry,rz]=", tcp_pose)
        textmsg("EE speed [vx,vy,vz,wx,wy,wz]=", tcp_speed)
        textmsg("EE wrench [fx,fy,fz,tx,ty,tz]=", tcp_wrench)

        next_sample = next_sample + sample_period_s
      end

      sync()
      elapsed = elapsed + get_steptime()
    end

    textmsg("Finished EE state readout")
  end

  ee_state_readout({duration_s:.6f}, {sample_period_s:.6f})
end
"""



def send_urscript(robot_ip, port, urscript):
    payload = urscript
    if not payload.endswith("\n"):
        payload += "\n"

    with socket.create_connection((robot_ip, port), timeout=SOCKET_TIMEOUT_S) as sock:
        sock.settimeout(SOCKET_TIMEOUT_S)
        sock.sendall(payload.encode("utf-8"))
        time.sleep(SOCKET_CLOSE_DELAY_S)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send a safe zero additional torque URScript test to a UR5e."
    )
    parser.add_argument("--robot-ip", default=DEFAULT_ROBOT_IP)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the generated URScript instead of sending it.",
    )
    parser.add_argument(
        "--smoke-test",
        action="store_true",
        help="Send only start/end textmsg calls to verify socket execution.",
    )
    parser.add_argument(
        "--ee-state-readout",
        action="store_true",
        help=(
            "Log measured TCP pose, TCP speed, and TCP wrench from the robot. "
            "No direct torque command is sent in this mode."
        ),
    )
    parser.add_argument(
        "--sample-period",
        type=float,
        default=DEFAULT_SAMPLE_PERIOD,
        help="Seconds between EE state log samples when using --ee-state-readout.",
    )
    parser.add_argument(
        "--use-friction-scales",
        action="store_true",
        help=(
            "Use viscous_scale/coulomb_scale keyword arguments instead of the "
            "documented PolyScope 5.25 direct_torque(torques, friction_comp=True) form."
        ),
    )
    return parser.parse_args()


def main():
    args = parse_args()

    if args.duration <= 0:
        print("ERROR: --duration must be greater than 0.", file=sys.stderr)
        return 2

    if args.sample_period <= 0:
        print("ERROR: --sample-period must be greater than 0.", file=sys.stderr)
        return 2

    if args.smoke_test and args.ee_state_readout:
        print("ERROR: choose only one mode: --smoke-test or --ee-state-readout.", file=sys.stderr)
        return 2

    if args.smoke_test:
        urscript = build_smoke_test_urscript()
    elif args.ee_state_readout:
        urscript = build_ee_state_readout_urscript(args.duration, args.sample_period)
    else:
        urscript = build_urscript(args.duration, args.use_friction_scales)

    print(f"Target robot: {args.robot_ip}:{args.port}")
    if args.smoke_test:
        print("Test mode: smoke-test text messages only")
    elif args.ee_state_readout:
        print("Test mode: EE state readout only")
        print(f"Readout duration: {args.duration:.3f} seconds")
        print(f"Sample period: {args.sample_period:.3f} seconds")
    else:
        print(f"Test duration: {args.duration:.3f} seconds")
        if args.use_friction_scales:
            print("Direct torque form: viscous_scale/coulomb_scale")
        else:
            print("Direct torque form: friction_comp=True")

    if args.dry_run:
        print()
        print(urscript, end="")
        return 0

    try:
        send_urscript(args.robot_ip, args.port, urscript)
    except socket.timeout:
        print("ERROR: Socket operation timed out.", file=sys.stderr)
        return 1
    except OSError as exc:
        print(f"ERROR: Could not send URScript: {exc}", file=sys.stderr)
        return 1

    print("URScript sent successfully.")
    if args.smoke_test:
        print("Check the PolyScope Log tab for the smoke-test START and END messages.")
    elif args.ee_state_readout:
        print("Check the PolyScope Log tab for EE pose, speed, and wrench samples.")
    else:
        print("Check the PolyScope Log tab for the zero-torque start and finish messages.")
        print("Watch the robot and confirm no jump, no drift, and no protective stop.")
    time.sleep(0.1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
