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
from pathlib import Path
import socket
import sys
import time


SCRIPT_VERSION = "V2"
DEFAULT_ROBOT_IP = "192.168.1.2"
DEFAULT_PORT = 30002
DEFAULT_DURATION = 10.0
DEFAULT_SAMPLE_PERIOD = 0.5
DEFAULT_FT_ZERO_SETTLE_S = 0.2
# Conservative default limits for the commissioning kernel. The zero-torque
# command should never reach these, but future nonzero controllers will reuse
# the same safety path before calling direct_torque().
DEFAULT_MAX_TORQUE_NM = 2.0
DEFAULT_MAX_TORQUE_RATE_NM_S = 10.0
DEFAULT_MAX_TCP_FORCE_N = 100.0
DEFAULT_MAX_TCP_TORQUE_NM = 10.0
DEFAULT_JOINT_DAMPING = [0.02, 0.02, 0.015, 0.005, 0.005, 0.003]
DEFAULT_JOINT_STIFFNESS = [0.2, 0.2, 0.15, 0.05, 0.05, 0.03]
DEFAULT_TELEMETRY_HOST = "auto"
DEFAULT_TELEMETRY_BIND_IP = "0.0.0.0"
DEFAULT_TELEMETRY_PORT = 50002
SOCKET_TIMEOUT_S = 5.0
SOCKET_CLOSE_DELAY_S = 0.2
TELEMETRY_ACCEPT_TIMEOUT_S = 10.0
TELEMETRY_IDLE_TIMEOUT_S = 2.0
TELEMETRY_LOG_PREFIX = "ur5e_telemetry"


def format_urscript_list(values):
    return "[" + ", ".join(f"{value:.6f}" for value in values) + "]"


def format_urscript_string(value):
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'


def guess_local_ip_for_robot(robot_ip):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect((robot_ip, DEFAULT_PORT))
        return sock.getsockname()[0]


def resolve_telemetry_host(telemetry_host, robot_ip):
    if telemetry_host != "auto":
        return telemetry_host

    try:
        return guess_local_ip_for_robot(robot_ip)
    except OSError as exc:
        raise RuntimeError(
            "Could not auto-detect the PC IP address reachable by the robot. "
            "Pass it explicitly with --telemetry-host."
        ) from exc


def build_urscript(
    duration_s,
    use_friction_scales=False,
    max_torque_nm=DEFAULT_MAX_TORQUE_NM,
    max_torque_rate_nm_s=DEFAULT_MAX_TORQUE_RATE_NM_S,
    max_tcp_force_n=DEFAULT_MAX_TCP_FORCE_N,
    max_tcp_torque_nm=DEFAULT_MAX_TCP_TORQUE_NM,
    sample_period_s=DEFAULT_SAMPLE_PERIOD,
    log_torque_loop_state=False,
    joint_damping_test=False,
    joint_impedance_test=False,
    joint_damping=DEFAULT_JOINT_DAMPING,
    joint_stiffness=DEFAULT_JOINT_STIFFNESS,
    telemetry_host="",
    telemetry_port=DEFAULT_TELEMETRY_PORT,
):
    if use_friction_scales:
        direct_torque_call = (
            "direct_torque(tau_safe, "
            "viscous_scale=[0.9, 0.9, 0.8, 0.9, 0.9, 0.9], "
            "coulomb_scale=[0.8, 0.8, 0.7, 0.8, 0.8, 0.8])"
        )
    else:
        direct_torque_call = "direct_torque(tau_safe, friction_comp=True)"

    log_state = "True" if log_torque_loop_state else "False"
    damping_test = "True" if joint_damping_test else "False"
    impedance_test = "True" if joint_impedance_test else "False"
    joint_damping_values = format_urscript_list(joint_damping)
    joint_stiffness_values = format_urscript_list(joint_stiffness)
    telemetry_host_literal = format_urscript_string(telemetry_host)

    return f"""def ur5e_zero_torque_socket_program():
  # Clamp one value symmetrically around zero. This is the final guard before
  # a torque value is allowed into the direct_torque() call.
  def clamp_value(value, limit):
    if value > limit:
      return limit
    elif value < -limit:
      return -limit
    else:
      return value
    end
  end

  # URScript list math is limited, so these helpers keep the safety logic clear.
  def abs_value(value):
    if value < 0:
      return -value
    else:
      return value
    end
  end

  # Limit commanded joint torque per joint. Future controllers should replace
  # tau_cmd, not bypass this clamp.
  def clamp_torque(tau_cmd, max_torque_nm):
    return [clamp_value(tau_cmd[0], max_torque_nm), clamp_value(tau_cmd[1], max_torque_nm), clamp_value(tau_cmd[2], max_torque_nm), clamp_value(tau_cmd[3], max_torque_nm), clamp_value(tau_cmd[4], max_torque_nm), clamp_value(tau_cmd[5], max_torque_nm)]
  end

  # Limit how quickly torque can change. This prevents a future controller from
  # stepping from a small command to a large command in one 2 ms cycle.
  def rate_limit_torque(tau_cmd, previous_tau, max_torque_rate_nm_s, dt):
    max_delta = max_torque_rate_nm_s * dt
    return [previous_tau[0] + clamp_value(tau_cmd[0] - previous_tau[0], max_delta), previous_tau[1] + clamp_value(tau_cmd[1] - previous_tau[1], max_delta), previous_tau[2] + clamp_value(tau_cmd[2] - previous_tau[2], max_delta), previous_tau[3] + clamp_value(tau_cmd[3] - previous_tau[3], max_delta), previous_tau[4] + clamp_value(tau_cmd[4] - previous_tau[4], max_delta), previous_tau[5] + clamp_value(tau_cmd[5] - previous_tau[5], max_delta)]
  end

  # Report whether each joint was changed by a limiter. A value of 1 means the
  # downstream command no longer matches the upstream command for that joint.
  def limit_changed_flag(before_value, after_value):
    if abs_value(before_value - after_value) > 0.000001:
      return 1
    else:
      return 0
    end
  end

  def limit_changed_flags(before_tau, after_tau):
    return [limit_changed_flag(before_tau[0], after_tau[0]), limit_changed_flag(before_tau[1], after_tau[1]), limit_changed_flag(before_tau[2], after_tau[2]), limit_changed_flag(before_tau[3], after_tau[3]), limit_changed_flag(before_tau[4], after_tau[4]), limit_changed_flag(before_tau[5], after_tau[5])]
  end

  # Positive joint power means the commanded torque is adding energy to motion;
  # negative joint power means it is removing energy.
  def joint_power(tau, joint_speeds):
    return [tau[0] * joint_speeds[0], tau[1] * joint_speeds[1], tau[2] * joint_speeds[2], tau[3] * joint_speeds[3], tau[4] * joint_speeds[4], tau[5] * joint_speeds[5]]
  end

  def sum_six(values):
    return values[0] + values[1] + values[2] + values[3] + values[4] + values[5]
  end

  # Component limits are intentionally simple for commissioning. They catch a
  # bad contact or bad sensor state before this script evolves into force work.
  def max_force_component(tcp_wrench):
    max_component = abs_value(tcp_wrench[0])
    if abs_value(tcp_wrench[1]) > max_component:
      max_component = abs_value(tcp_wrench[1])
    end
    if abs_value(tcp_wrench[2]) > max_component:
      max_component = abs_value(tcp_wrench[2])
    end
    return max_component
  end

  def max_torque_component(tcp_wrench):
    max_component = abs_value(tcp_wrench[3])
    if abs_value(tcp_wrench[4]) > max_component:
      max_component = abs_value(tcp_wrench[4])
    end
    if abs_value(tcp_wrench[5]) > max_component:
      max_component = abs_value(tcp_wrench[5])
    end
    return max_component
  end

  # Stream telemetry back to Python. The Python receiver writes these lines to
  # a timestamped text file instead of spamming the bash terminal.
  def send_telemetry(label, value):
    socket_send_string(label, "telemetry")
    socket_send_line(to_str(value), "telemetry")
  end

  def zero_torque_test(duration_s, max_torque_nm, max_torque_rate_nm_s, max_tcp_force_n, max_tcp_torque_nm, sample_period_s, log_torque_loop_state, joint_damping_test, joint_impedance_test, joint_damping, joint_stiffness):
    if joint_impedance_test:
      textmsg("Starting joint impedance direct torque test, duration_s=", duration_s)
    elif joint_damping_test:
      textmsg("Starting joint damping direct torque test, duration_s=", duration_s)
    else:
      textmsg("Starting zero additional torque test, duration_s=", duration_s)
    end

    elapsed = 0.0
    next_sample = 0.0
    stop_reason = "duration complete"
    previous_tau = [0, 0, 0, 0, 0, 0]
    q_hold = get_actual_joint_positions()
    telemetry_open = False

    if log_torque_loop_state:
      telemetry_open = socket_open({telemetry_host_literal}, {telemetry_port}, "telemetry")
      if telemetry_open:
        socket_send_line("Starting torque-loop telemetry", "telemetry")
        send_telemetry("q_hold=", q_hold)
      else:
        textmsg("Telemetry socket open failed; continuing without file telemetry")
      end
    end

    while elapsed < duration_s:
      dt = get_steptime()

      # Read the state that future impedance modes will need. The zero-torque
      # mode only uses the wrench for safety, but logging it proves the timing
      # and sensor path while direct_torque() is active.
      joint_positions = get_actual_joint_positions()
      joint_speeds = get_actual_joint_speeds()
      tcp_pose = get_actual_tcp_pose()
      tcp_speed = get_actual_tcp_speed()
      tcp_wrench = get_tcp_force()

      safety_ok = True
      if max_force_component(tcp_wrench) > max_tcp_force_n:
        stop_reason = "TCP force component limit"
        safety_ok = False
        elapsed = duration_s
      end

      if safety_ok and max_torque_component(tcp_wrench) > max_tcp_torque_nm:
        stop_reason = "TCP torque component limit"
        safety_ok = False
        elapsed = duration_s
      end

      if safety_ok:
        if joint_impedance_test:
          # First spring-damper impedance step: hold the starting joint pose
          # softly while damping velocity. This should feel like a weak virtual
          # spring around q_hold, not a stiff position servo.
          q_error = [q_hold[0] - joint_positions[0], q_hold[1] - joint_positions[1], q_hold[2] - joint_positions[2], q_hold[3] - joint_positions[3], q_hold[4] - joint_positions[4], q_hold[5] - joint_positions[5]]
          tau_spring = [joint_stiffness[0] * q_error[0], joint_stiffness[1] * q_error[1], joint_stiffness[2] * q_error[2], joint_stiffness[3] * q_error[3], joint_stiffness[4] * q_error[4], joint_stiffness[5] * q_error[5]]
          tau_damping = [-joint_damping[0] * joint_speeds[0], -joint_damping[1] * joint_speeds[1], -joint_damping[2] * joint_speeds[2], -joint_damping[3] * joint_speeds[3], -joint_damping[4] * joint_speeds[4], -joint_damping[5] * joint_speeds[5]]
          tau_cmd = [tau_spring[0] + tau_damping[0], tau_spring[1] + tau_damping[1], tau_spring[2] + tau_damping[2], tau_spring[3] + tau_damping[3], tau_spring[4] + tau_damping[4], tau_spring[5] + tau_damping[5]]
        elif joint_damping_test:
          # First nonzero controller step: oppose measured joint velocity.
          # This should remove motion energy instead of pulling toward a pose.
          q_error = [0, 0, 0, 0, 0, 0]
          tau_spring = [0, 0, 0, 0, 0, 0]
          tau_damping = [-joint_damping[0] * joint_speeds[0], -joint_damping[1] * joint_speeds[1], -joint_damping[2] * joint_speeds[2], -joint_damping[3] * joint_speeds[3], -joint_damping[4] * joint_speeds[4], -joint_damping[5] * joint_speeds[5]]
          tau_cmd = tau_damping
        else:
          # Zero additional torque remains the baseline commissioning command.
          # Later controllers should replace tau_cmd, not bypass the safety path.
          q_error = [0, 0, 0, 0, 0, 0]
          tau_spring = [0, 0, 0, 0, 0, 0]
          tau_damping = [0, 0, 0, 0, 0, 0]
          tau_cmd = [0, 0, 0, 0, 0, 0]
        end

        tau_clamped = clamp_torque(tau_cmd, max_torque_nm)
        torque_clamp_active = limit_changed_flags(tau_cmd, tau_clamped)
        tau_safe = rate_limit_torque(tau_clamped, previous_tau, max_torque_rate_nm_s, dt)
        torque_rate_active = limit_changed_flags(tau_clamped, tau_safe)
        tau_power = joint_power(tau_safe, joint_speeds)
        total_power = sum_six(tau_power)

        if log_torque_loop_state and telemetry_open and elapsed >= next_sample:
          send_telemetry("t=", elapsed)
          send_telemetry("q=", joint_positions)
          send_telemetry("qd=", joint_speeds)
          send_telemetry("q_error=", q_error)
          send_telemetry("EE pose [x,y,z,rx,ry,rz]=", tcp_pose)
          send_telemetry("EE speed [vx,vy,vz,wx,wy,wz]=", tcp_speed)
          send_telemetry("EE wrench [fx,fy,fz,tx,ty,tz]=", tcp_wrench)
          send_telemetry("tau_spring=", tau_spring)
          send_telemetry("tau_damping=", tau_damping)
          send_telemetry("tau_cmd=", tau_cmd)
          send_telemetry("tau_clamped=", tau_clamped)
          send_telemetry("tau_safe=", tau_safe)
          send_telemetry("torque_clamp_active=", torque_clamp_active)
          send_telemetry("torque_rate_active=", torque_rate_active)
          send_telemetry("joint_power=", tau_power)
          send_telemetry("total_power=", total_power)
          next_sample = next_sample + sample_period_s
        end

        {direct_torque_call}
        previous_tau = tau_safe
        elapsed = elapsed + dt
      end
    end

    stopj(2.0)
    if joint_impedance_test:
      textmsg("Finished joint impedance direct torque test, stop_reason=", stop_reason)
    elif joint_damping_test:
      textmsg("Finished joint damping direct torque test, stop_reason=", stop_reason)
    else:
      textmsg("Finished zero additional torque test, stop_reason=", stop_reason)
    end
    if telemetry_open:
      send_telemetry("stop_reason=", stop_reason)
      socket_close("telemetry")
    end
  end

  zero_torque_test({duration_s:.6f}, {max_torque_nm:.6f}, {max_torque_rate_nm_s:.6f}, {max_tcp_force_n:.6f}, {max_tcp_torque_nm:.6f}, {sample_period_s:.6f}, {log_state}, {damping_test}, {impedance_test}, {joint_damping_values}, {joint_stiffness_values})
end
"""


def build_smoke_test_urscript():
    return """def ur5e_socket_smoke_test():
  textmsg("UR5e socket smoke test START")
  sleep(0.25)
  textmsg("UR5e socket smoke test END")
end
"""


def build_ee_state_readout_urscript(duration_s, sample_period_s, telemetry_host, telemetry_port):
    telemetry_host_literal = format_urscript_string(telemetry_host)
    return f"""def ur5e_ee_state_readout_socket_program():
  def send_telemetry(label, value):
    socket_send_string(label, "telemetry")
    socket_send_line(to_str(value), "telemetry")
  end

  def ee_state_readout(duration_s, sample_period_s):
    textmsg("Starting EE state readout, duration_s=", duration_s)

    # Zero the built-in F/T sensor once before telemetry starts. The tool must
    # be in free space with no contact load, otherwise real contact force will
    # be hidden inside the bias.
    textmsg("Zeroing F/T sensor before EE state readout")
    zero_ftsensor()
    sleep({DEFAULT_FT_ZERO_SETTLE_S:.6f})

    telemetry_open = socket_open({telemetry_host_literal}, {telemetry_port}, "telemetry")
    if telemetry_open:
      socket_send_line("Starting EE state readout telemetry", "telemetry")
    else:
      textmsg("Telemetry socket open failed; continuing without file telemetry")
    end

    elapsed = 0.0
    next_sample = 0.0

    while elapsed < duration_s:
      if telemetry_open and elapsed >= next_sample:
        tcp_pose = get_actual_tcp_pose()
        tcp_speed = get_actual_tcp_speed()
        tcp_wrench = get_tcp_force()

        send_telemetry("t=", elapsed)
        send_telemetry("EE pose [x,y,z,rx,ry,rz]=", tcp_pose)
        send_telemetry("EE speed [vx,vy,vz,wx,wy,wz]=", tcp_speed)
        send_telemetry("EE wrench [fx,fy,fz,tx,ty,tz]=", tcp_wrench)

        next_sample = next_sample + sample_period_s
      end

      sync()
      elapsed = elapsed + get_steptime()
    end

    textmsg("Finished EE state readout")
    if telemetry_open:
      socket_send_line("Finished EE state readout telemetry", "telemetry")
      socket_close("telemetry")
    end
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


def make_telemetry_server(bind_ip, telemetry_port):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((bind_ip, telemetry_port))
    server.listen(1)
    server.settimeout(TELEMETRY_ACCEPT_TIMEOUT_S)
    return server


def make_telemetry_log_path():
    script_dir = Path(__file__).resolve().parent
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    return script_dir / f"{TELEMETRY_LOG_PREFIX}_{timestamp}.txt"


def create_telemetry_log(args, telemetry_host):
    log_path = make_telemetry_log_path()
    with log_path.open("w", encoding="utf-8") as log_file:
        log_file.write("# UR5e telemetry log\n")
        log_file.write(f"# script_version={SCRIPT_VERSION}\n")
        log_file.write(f"# created_local={time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        log_file.write(f"# command={' '.join(sys.argv)}\n")
        log_file.write(f"# robot_ip={args.robot_ip}\n")
        log_file.write(f"# robot_port={args.port}\n")
        log_file.write(f"# telemetry_host={telemetry_host}\n")
        log_file.write(f"# telemetry_port={args.telemetry_port}\n")
        log_file.write(f"# sample_period={args.sample_period}\n")
        if args.joint_impedance_test:
            log_file.write("# mode=joint_impedance_test\n")
            log_file.write(f"# joint_stiffness={args.joint_stiffness}\n")
            log_file.write(f"# joint_damping={args.joint_damping}\n")
        elif args.joint_damping_test:
            log_file.write("# mode=joint_damping_test\n")
            log_file.write(f"# joint_damping={args.joint_damping}\n")
        elif args.ee_state_readout:
            log_file.write("# mode=ee_state_readout\n")
        else:
            log_file.write("# mode=zero_torque\n")
        log_file.write(f"# max_torque={args.max_torque}\n")
        log_file.write(f"# max_torque_rate={args.max_torque_rate}\n")
        log_file.write("# data_format=label=value lines grouped by sample\n")
    return log_path


def receive_telemetry(server, log_path):
    try:
        conn, addr = server.accept()
    except socket.timeout:
        print("ERROR: Timed out waiting for robot telemetry connection.", file=sys.stderr)
        return 1

    print(f"Telemetry connected from {addr[0]}:{addr[1]}")
    conn.settimeout(TELEMETRY_IDLE_TIMEOUT_S)
    buffer = ""

    with conn, log_path.open("a", encoding="utf-8") as log_file:
        log_file.write(f"# telemetry_connected_from={addr[0]}:{addr[1]}\n")
        while True:
            try:
                chunk = conn.recv(4096)
            except socket.timeout:
                continue

            if not chunk:
                break

            buffer += chunk.decode("utf-8", errors="replace")
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                log_file.write(line.rstrip("\r") + "\n")
                log_file.flush()

    if buffer:
        with log_path.open("a", encoding="utf-8") as log_file:
            log_file.write(buffer.rstrip("\r") + "\n")

    print("Telemetry connection closed.")
    print(f"Telemetry log written to: {log_path}")
    return 0


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
        help=(
            "Seconds between log samples when using --ee-state-readout or "
            "--log-torque-loop-state."
        ),
    )
    parser.add_argument(
        "--telemetry-host",
        default=DEFAULT_TELEMETRY_HOST,
        help=(
            "PC IP address the robot should connect to for telemetry. "
            "Use 'auto' to infer it from the route to --robot-ip."
        ),
    )
    parser.add_argument(
        "--telemetry-bind-ip",
        default=DEFAULT_TELEMETRY_BIND_IP,
        help="Local interface for the Python telemetry server to listen on.",
    )
    parser.add_argument(
        "--telemetry-port",
        type=int,
        default=DEFAULT_TELEMETRY_PORT,
        help="TCP port for robot-to-PC telemetry streaming.",
    )
    parser.add_argument(
        "--max-torque",
        type=float,
        default=DEFAULT_MAX_TORQUE_NM,
        help="Maximum absolute commanded torque per joint in Nm.",
    )
    parser.add_argument(
        "--max-torque-rate",
        type=float,
        default=DEFAULT_MAX_TORQUE_RATE_NM_S,
        help="Maximum per-joint torque change rate in Nm/s.",
    )
    parser.add_argument(
        "--max-tcp-force",
        type=float,
        default=DEFAULT_MAX_TCP_FORCE_N,
        help="Maximum absolute TCP force component in N before stopping.",
    )
    parser.add_argument(
        "--max-tcp-torque",
        type=float,
        default=DEFAULT_MAX_TCP_TORQUE_NM,
        help="Maximum absolute TCP torque component in Nm before stopping.",
    )
    parser.add_argument(
        "--log-torque-loop-state",
        action="store_true",
        help=(
            "Log joint state, EE state, wrench, and tau_safe from inside the "
            "direct_torque loop."
        ),
    )
    parser.add_argument(
        "--joint-damping-test",
        action="store_true",
        help=(
            "Use the safe direct_torque loop to command tau = -Dq * qd. "
            "This is the first nonzero torque-control test."
        ),
    )
    parser.add_argument(
        "--joint-impedance-test",
        action="store_true",
        help=(
            "Use the safe direct_torque loop to hold the starting joint pose "
            "with tau = Kq * (q_hold - q) - Dq * qd."
        ),
    )
    parser.add_argument(
        "--joint-damping",
        type=float,
        nargs=6,
        metavar=("D0", "D1", "D2", "D3", "D4", "D5"),
        default=DEFAULT_JOINT_DAMPING,
        help=(
            "Six joint damping gains in Nms/rad for damping or impedance tests. "
            f"Default: {DEFAULT_JOINT_DAMPING}."
        ),
    )
    parser.add_argument(
        "--joint-stiffness",
        type=float,
        nargs=6,
        metavar=("K0", "K1", "K2", "K3", "K4", "K5"),
        default=DEFAULT_JOINT_STIFFNESS,
        help=(
            "Six joint stiffness gains in Nm/rad for --joint-impedance-test. "
            f"Default: {DEFAULT_JOINT_STIFFNESS}."
        ),
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

    if not 1 <= args.telemetry_port <= 65535:
        print("ERROR: --telemetry-port must be between 1 and 65535.", file=sys.stderr)
        return 2

    if args.max_torque <= 0:
        print("ERROR: --max-torque must be greater than 0.", file=sys.stderr)
        return 2

    if args.max_torque_rate <= 0:
        print("ERROR: --max-torque-rate must be greater than 0.", file=sys.stderr)
        return 2

    if args.max_tcp_force <= 0:
        print("ERROR: --max-tcp-force must be greater than 0.", file=sys.stderr)
        return 2

    if args.max_tcp_torque <= 0:
        print("ERROR: --max-tcp-torque must be greater than 0.", file=sys.stderr)
        return 2

    if any(gain < 0 for gain in args.joint_damping):
        print("ERROR: --joint-damping values must be zero or greater.", file=sys.stderr)
        return 2

    if any(gain < 0 for gain in args.joint_stiffness):
        print("ERROR: --joint-stiffness values must be zero or greater.", file=sys.stderr)
        return 2

    if args.smoke_test and args.ee_state_readout:
        print("ERROR: choose only one mode: --smoke-test or --ee-state-readout.", file=sys.stderr)
        return 2

    if args.joint_damping_test and args.joint_impedance_test:
        print(
            "ERROR: choose only one torque mode: --joint-damping-test or "
            "--joint-impedance-test.",
            file=sys.stderr,
        )
        return 2

    if (args.joint_damping_test or args.joint_impedance_test) and args.use_friction_scales:
        print(
            "ERROR: damping and impedance tests use the tested friction_comp=True baseline; "
            "do not combine it with --use-friction-scales.",
            file=sys.stderr,
        )
        return 2

    telemetry_enabled = args.ee_state_readout or args.log_torque_loop_state
    telemetry_host = ""
    telemetry_log_path = None
    if telemetry_enabled:
        try:
            telemetry_host = resolve_telemetry_host(args.telemetry_host, args.robot_ip)
        except RuntimeError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 2

    if args.smoke_test:
        urscript = build_smoke_test_urscript()
    elif args.ee_state_readout:
        urscript = build_ee_state_readout_urscript(
            args.duration,
            args.sample_period,
            telemetry_host,
            args.telemetry_port,
        )
    else:
        urscript = build_urscript(
            args.duration,
            use_friction_scales=args.use_friction_scales,
            max_torque_nm=args.max_torque,
            max_torque_rate_nm_s=args.max_torque_rate,
            max_tcp_force_n=args.max_tcp_force,
            max_tcp_torque_nm=args.max_tcp_torque,
            sample_period_s=args.sample_period,
            log_torque_loop_state=args.log_torque_loop_state,
            joint_damping_test=args.joint_damping_test,
            joint_impedance_test=args.joint_impedance_test,
            joint_damping=args.joint_damping,
            joint_stiffness=args.joint_stiffness,
            telemetry_host=telemetry_host,
            telemetry_port=args.telemetry_port,
        )

    print(f"Target robot: {args.robot_ip}:{args.port}")
    print(f"Script version: {SCRIPT_VERSION}")
    if args.smoke_test:
        print("Test mode: smoke-test text messages only")
    elif args.ee_state_readout:
        print("Test mode: EE state readout only")
        print(f"Readout duration: {args.duration:.3f} seconds")
        print(f"Sample period: {args.sample_period:.3f} seconds")
        print(f"Telemetry target: {telemetry_host}:{args.telemetry_port}")
    else:
        print(f"Test duration: {args.duration:.3f} seconds")
        if args.joint_impedance_test:
            print("Test mode: joint impedance direct-torque kernel")
            print(f"Joint stiffness gains: {args.joint_stiffness}")
            print(f"Joint damping gains: {args.joint_damping}")
        elif args.joint_damping_test:
            print("Test mode: joint damping direct-torque kernel")
            print(f"Joint damping gains: {args.joint_damping}")
        else:
            print("Test mode: safe zero-torque kernel")
        print(f"Max torque per joint: {args.max_torque:.3f} Nm")
        print(f"Max torque rate: {args.max_torque_rate:.3f} Nm/s")
        print(f"Max TCP force component: {args.max_tcp_force:.3f} N")
        print(f"Max TCP torque component: {args.max_tcp_torque:.3f} Nm")
        if args.log_torque_loop_state:
            print(f"Torque-loop state logging period: {args.sample_period:.3f} seconds")
            print(f"Telemetry target: {telemetry_host}:{args.telemetry_port}")
        if args.use_friction_scales:
            print("Direct torque form: viscous_scale/coulomb_scale")
        else:
            print("Direct torque form: friction_comp=True")

    if args.dry_run:
        print()
        print(urscript, end="")
        return 0

    try:
        if telemetry_enabled:
            telemetry_log_path = create_telemetry_log(args, telemetry_host)
            print(f"Telemetry log file: {telemetry_log_path}")
            with make_telemetry_server(args.telemetry_bind_ip, args.telemetry_port) as server:
                print(
                    "Telemetry server listening on "
                    f"{args.telemetry_bind_ip}:{args.telemetry_port}"
                )
                send_urscript(args.robot_ip, args.port, urscript)
                print("URScript sent successfully.")
                telemetry_status = receive_telemetry(server, telemetry_log_path)
                if telemetry_status != 0:
                    return telemetry_status
        else:
            send_urscript(args.robot_ip, args.port, urscript)
    except socket.timeout:
        print("ERROR: Socket operation timed out.", file=sys.stderr)
        return 1
    except OSError as exc:
        print(f"ERROR: Socket operation failed: {exc}", file=sys.stderr)
        return 1

    if not telemetry_enabled:
        print("URScript sent successfully.")
    if args.smoke_test:
        print("Check the PolyScope Log tab for the smoke-test START and END messages.")
    elif args.ee_state_readout:
        print(f"EE pose, speed, and wrench samples were written to {telemetry_log_path}.")
    else:
        if args.joint_impedance_test:
            print("Check the PolyScope Log tab for the joint-impedance start and finish messages.")
            print("Confirm q_error, tau_cmd, and tau_safe stay small and make sense.")
        elif args.joint_damping_test:
            print("Check the PolyScope Log tab for the joint-damping start and finish messages.")
            print("Confirm tau_safe is small and opposes measured qd.")
        else:
            print("Check the PolyScope Log tab for the zero-torque start and finish messages.")
        if args.log_torque_loop_state:
            print(
                "In-loop q, qd, EE state, wrench, torque terms, limiter flags, "
                f"and power samples were written to {telemetry_log_path}."
            )
        print("Watch the robot and confirm no jump, no drift, and no protective stop.")
    time.sleep(0.1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
