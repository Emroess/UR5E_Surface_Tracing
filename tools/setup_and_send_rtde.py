#!/usr/bin/env python3
"""Set payload/TCP, tare FT sensor, and send the commissioning script via ur_rtde."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import re
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCRIPT_PATH = ROOT / "ur5e_direct_torque_anisotropic_impedance_tests.script"
VALIDATOR_PATH = ROOT / "tools" / "validate_commissioning.py"

# Defaults copied from force_mode_telemetry_real.py in UR5E_Surface_Tracing.
DEFAULT_ROBOT_IP = "192.168.1.2"


def resolve_path(path_text: str) -> Path:
    path = Path(path_text).expanduser()
    if not path.is_absolute():
        path = (Path.cwd() / path).resolve()
    return path


def read_script(path: Path) -> str:
    if not path.exists():
        raise FileNotFoundError(f"script file not found: {path}")
    if not path.is_file():
        raise ValueError(f"script path is not a file: {path}")

    text = path.read_text(encoding="utf-8")
    if not text.endswith("\n"):
        text += "\n"
    return text


def find_assignment(script_text: str, name: str) -> str | None:
    match = re.search(rf"^\s*{re.escape(name)}\s*=\s*(.+?)\s*$", script_text, re.MULTILINE)
    if match:
        return match.group(1)
    return None


def parse_int_assignment(value: str | None) -> int | None:
    if value is None:
        return None
    try:
        return int(value.strip())
    except ValueError:
        return None


def run_static_validator() -> tuple[bool, str]:
    if not VALIDATOR_PATH.exists():
        return False, f"validator not found: {VALIDATOR_PATH}"

    result = subprocess.run(
        [sys.executable, str(VALIDATOR_PATH)],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
    )
    output = "\n".join(part for part in [result.stdout.strip(), result.stderr.strip()] if part)
    return result.returncode == 0, output


def require_not_false(result, action: str) -> None:
    if result is False:
        raise RuntimeError(f"{action} returned False")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Set payload, set TCP, zero/tare the UR5e FT sensor, and send a URScript file "
            "using the ur_rtde RTDEControlInterface helper."
        )
    )
    parser.add_argument(
        "script",
        nargs="?",
        default=str(DEFAULT_SCRIPT_PATH),
        help=f"URScript file to send. Default: {DEFAULT_SCRIPT_PATH}",
    )
    parser.add_argument(
        "--host",
        default=DEFAULT_ROBOT_IP,
        help=f"Robot IP address. Default: {DEFAULT_ROBOT_IP}",
    )
    parser.add_argument(
        "--payload-mass",
        type=float,
        required=True,
        help="Payload mass in kg. Required; do not guess.",
    )
    parser.add_argument(
        "--payload-cog",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        required=True,
        help="Payload center of gravity in meters, using the UR payload convention.",
    )
    parser.add_argument(
        "--tcp",
        type=float,
        nargs=6,
        metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
        required=True,
        help="TCP offset pose [x y z rx ry rz] in meters and radians.",
    )
    parser.add_argument(
        "--expect-test-id",
        type=int,
        choices=range(5),
        metavar="N",
        help="Abort unless the script contains TEST_ID = N. Valid values: 0, 1, 2, 3, 4.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the setup/send summary without connecting to the robot.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip the interactive confirmation prompt before touching the robot.",
    )
    parser.add_argument(
        "--skip-validation",
        action="store_true",
        help="Skip the static commissioning validator before a real send.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    script_path = resolve_path(args.script)
    try:
        script_text = read_script(script_path)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    payload = script_text.encode("utf-8")
    payload_size = len(payload)
    payload_sha256 = hashlib.sha256(payload).hexdigest()
    test_id = find_assignment(script_text, "TEST_ID")
    allow_test_4 = find_assignment(script_text, "ALLOW_TEST_4_BIASED_MOTION")
    parsed_test_id = parse_int_assignment(test_id)

    print("RTDE setup + script send summary")
    print(f"  File:  {script_path}")
    print(f"  Bytes: {payload_size}")
    print(f"  SHA-256: {payload_sha256}")
    print(f"  Host:  {args.host}")
    print(f"  Payload mass kg: {args.payload_mass}")
    print(f"  Payload COG m:   {args.payload_cog}")
    print(f"  TCP pose:        {args.tcp}")
    print(f"  TEST_ID: {test_id if test_id is not None else 'not found'}")
    print(
        "  ALLOW_TEST_4_BIASED_MOTION: "
        f"{allow_test_4 if allow_test_4 is not None else 'not found'}"
    )
    if args.expect_test_id is not None:
        print(f"  Expected TEST_ID: {args.expect_test_id}")
    print()

    if args.expect_test_id is not None and parsed_test_id != args.expect_test_id:
        print(
            "ERROR: expected TEST_ID "
            f"{args.expect_test_id}, but script contains {test_id if test_id is not None else 'not found'}.",
            file=sys.stderr,
        )
        return 1

    print("Planned robot operations:")
    print("  1. Connect using ur_rtde RTDEControlInterface.")
    print("  2. setPayload(payload_mass, payload_cog).")
    print("  3. setTcp(tcp).")
    print("  4. zeroFtSensor() to tare the built-in force/torque sensor.")
    print("  5. sendCustomScriptFile(script).")
    print()
    print("Before continuing, confirm on the robot:")
    print("  - PolyScope is in remote control mode.")
    print("  - The provided payload mass/COG and TCP are correct.")
    print("  - The tool is in the intended no-load tare condition for zeroFtSensor().")
    print("  - The robot is in free space for commissioning tests.")
    print("  - The teach pendant stop is immediately reachable.")
    print()

    if args.dry_run:
        print("Dry run only: no RTDE connection was opened and no script was sent.")
        return 0

    if not args.skip_validation:
        print("Running static commissioning validator before robot setup/send...")
        ok, validator_output = run_static_validator()
        if validator_output:
            print(validator_output)
        if not ok:
            print("Aborted: static validation failed.", file=sys.stderr)
            return 1
        print()
    else:
        print("WARNING: static validation was skipped by --skip-validation.")
        print()

    if not args.yes:
        answer = input("Type SEND to set payload/TCP, tare FT sensor, and send script: ").strip()
        if answer != "SEND":
            print("Aborted: confirmation did not match SEND.")
            return 1

    try:
        from rtde_control import RTDEControlInterface
    except ImportError as exc:
        print(
            "ERROR: Python package 'ur_rtde' is required for this helper "
            "(import name: rtde_control). Install it in the Python environment used to run this script.",
            file=sys.stderr,
        )
        print(f"Import error: {exc}", file=sys.stderr)
        return 1

    rtde_c = None
    try:
        print(f"Connecting to RTDE control at {args.host}...")
        rtde_c = RTDEControlInterface(args.host)

        print("Setting payload...")
        require_not_false(rtde_c.setPayload(args.payload_mass, list(args.payload_cog)), "setPayload")

        print("Setting TCP...")
        require_not_false(rtde_c.setTcp(list(args.tcp)), "setTcp")

        print("Taring force/torque sensor with zeroFtSensor()...")
        require_not_false(rtde_c.zeroFtSensor(), "zeroFtSensor")

        print("Sending URScript file with sendCustomScriptFile()...")
        require_not_false(rtde_c.sendCustomScriptFile(str(script_path)), "sendCustomScriptFile")

    except Exception as exc:
        print(f"ERROR: RTDE setup/send failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if rtde_c is not None:
            try:
                rtde_c.disconnect()
            except Exception:
                pass

    print("RTDE setup/send complete.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
