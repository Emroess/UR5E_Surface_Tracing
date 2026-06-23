#!/usr/bin/env python3
"""Send the commissioning URScript file to the UR PolyScope secondary client."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import socket
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCRIPT_PATH = ROOT / "ur5e_direct_torque_anisotropic_impedance_tests.script"
VALIDATOR_PATH = ROOT / "tools" / "validate_commissioning.py"

# Defaults copied from the real-robot telemetry script in UR5E_Surface_Tracing:
#   HOST = "192.168.1.2"
#   Standard UR ports: 30002 commands, 30003 real-time telemetry.
DEFAULT_ROBOT_IP = "192.168.1.2"
DEFAULT_COMMAND_PORT = 30002


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


def send_script(script_text: str, host: str, port: int, timeout: float) -> bytes:
    payload = script_text.encode("utf-8")
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall(payload)

        try:
            return sock.recv(4096)
        except socket.timeout:
            return b""


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


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def resolve_path(path_text: str) -> Path:
    path = Path(path_text).expanduser()
    if not path.is_absolute():
        path = (Path.cwd() / path).resolve()
    return path


def append_jsonl(path: Path, record: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        json.dump(record, handle, sort_keys=True)
        handle.write("\n")


def ensure_log_writable(path_text: str) -> bool:
    log_path = resolve_path(path_text)
    try:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("a", encoding="utf-8"):
            pass
    except OSError as exc:
        print(f"ERROR: cannot write send log {log_path}: {exc}", file=sys.stderr)
        return False

    return True


def make_log_record(
    *,
    args: argparse.Namespace,
    script_path: Path,
    payload_size: int,
    payload_sha256: str,
    test_id: str | None,
    allow_test_4: str | None,
    status: str,
    detail: str | None = None,
) -> dict:
    record = {
        "timestamp_utc": utc_now(),
        "status": status,
        "script_path": str(script_path),
        "script_bytes": payload_size,
        "script_sha256": payload_sha256,
        "host": args.host,
        "port": args.port,
        "test_id": test_id,
        "allow_test_4_biased_motion": allow_test_4,
        "expected_test_id": args.expect_test_id,
        "dry_run": args.dry_run,
        "skip_validation": args.skip_validation,
    }
    if detail:
        record["detail"] = detail
    return record


def maybe_log(args: argparse.Namespace, record: dict) -> bool:
    if not args.log_run:
        return True

    log_path = resolve_path(args.log_run)
    try:
        append_jsonl(log_path, record)
    except OSError as exc:
        print(f"ERROR: cannot write send log {log_path}: {exc}", file=sys.stderr)
        return False

    print(f"Logged send event to {log_path}")
    return True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Send a URScript .script file to the UR PolyScope secondary client. "
            "The robot must be powered on, reachable, and in remote control mode."
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
        "--port",
        type=int,
        default=DEFAULT_COMMAND_PORT,
        help=f"UR secondary-client command port. Default: {DEFAULT_COMMAND_PORT}",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Socket connect/read timeout in seconds. Default: 5.0",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the send summary without opening a robot socket.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip the interactive confirmation prompt before sending.",
    )
    parser.add_argument(
        "--skip-validation",
        action="store_true",
        help="Skip the static commissioning validator before a real send.",
    )
    parser.add_argument(
        "--expect-test-id",
        type=int,
        choices=range(5),
        metavar="N",
        help="Abort unless the script contains TEST_ID = N. Valid values: 0, 1, 2, 3, 4.",
    )
    parser.add_argument(
        "--log-run",
        metavar="PATH",
        help="Append a JSON Lines send record to PATH.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    script_path = Path(args.script).expanduser()
    if not script_path.is_absolute():
        script_path = (Path.cwd() / script_path).resolve()

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

    print("URScript send summary")
    print(f"  File:  {script_path}")
    print(f"  Bytes: {payload_size}")
    print(f"  SHA-256: {payload_sha256}")
    print(f"  Host:  {args.host}")
    print(f"  Port:  {args.port}")
    print(f"  TEST_ID: {test_id if test_id is not None else 'not found'}")
    print(
        "  ALLOW_TEST_4_BIASED_MOTION: "
        f"{allow_test_4 if allow_test_4 is not None else 'not found'}"
    )
    if args.expect_test_id is not None:
        print(f"  Expected TEST_ID: {args.expect_test_id}")
    print()

    if args.expect_test_id is not None and parsed_test_id != args.expect_test_id:
        detail = (
            "expected TEST_ID "
            f"{args.expect_test_id}, but script contains {test_id if test_id is not None else 'not found'}"
        )
        maybe_log(
            args,
            make_log_record(
                args=args,
                script_path=script_path,
                payload_size=payload_size,
                payload_sha256=payload_sha256,
                test_id=test_id,
                allow_test_4=allow_test_4,
                status="aborted_test_id_mismatch",
                detail=detail,
            ),
        )
        print(f"ERROR: {detail}.", file=sys.stderr)
        return 1

    print("Before sending, confirm on the robot:")
    print("  - PolyScope is in remote control mode.")
    print("  - Payload and TCP are correct.")
    print("  - The robot is in free space for commissioning tests.")
    print("  - The teach pendant stop is immediately reachable.")
    print()

    if args.dry_run:
        if not maybe_log(
            args,
            make_log_record(
                args=args,
                script_path=script_path,
                payload_size=payload_size,
                payload_sha256=payload_sha256,
                test_id=test_id,
                allow_test_4=allow_test_4,
                status="dry_run",
            ),
        ):
            return 1
        print("Dry run only: no socket was opened and no script was sent.")
        return 0

    if not args.skip_validation:
        print("Running static commissioning validator before send...")
        ok, validator_output = run_static_validator()
        if validator_output:
            print(validator_output)
        if not ok:
            maybe_log(
                args,
                make_log_record(
                    args=args,
                    script_path=script_path,
                    payload_size=payload_size,
                    payload_sha256=payload_sha256,
                    test_id=test_id,
                    allow_test_4=allow_test_4,
                    status="aborted_validation_failed",
                    detail=validator_output,
                ),
            )
            print("Aborted: static validation failed.", file=sys.stderr)
            return 1
        print()
    else:
        print("WARNING: static validation was skipped by --skip-validation.")
        print()

    if args.log_run and not ensure_log_writable(args.log_run):
        print("Aborted: requested send log is not writable.", file=sys.stderr)
        return 1

    if not args.yes:
        answer = input("Type SEND to transmit this script to the robot: ").strip()
        if answer != "SEND":
            maybe_log(
                args,
                make_log_record(
                    args=args,
                    script_path=script_path,
                    payload_size=payload_size,
                    payload_sha256=payload_sha256,
                    test_id=test_id,
                    allow_test_4=allow_test_4,
                    status="aborted_confirmation",
                ),
            )
            print("Aborted: confirmation did not match SEND.")
            return 1

    try:
        response = send_script(script_text, args.host, args.port, args.timeout)
    except OSError as exc:
        maybe_log(
            args,
            make_log_record(
                args=args,
                script_path=script_path,
                payload_size=payload_size,
                payload_sha256=payload_sha256,
                test_id=test_id,
                allow_test_4=allow_test_4,
                status="send_failed",
                detail=str(exc),
            ),
        )
        print(f"ERROR: failed to send script to {args.host}:{args.port}: {exc}", file=sys.stderr)
        return 1

    maybe_log(
        args,
        make_log_record(
            args=args,
            script_path=script_path,
            payload_size=payload_size,
            payload_sha256=payload_sha256,
            test_id=test_id,
            allow_test_4=allow_test_4,
            status="sent",
            detail=f"immediate_response_bytes={len(response)}",
        ),
    )

    print(f"Sent {payload_size} bytes to {args.host}:{args.port}.")
    if response:
        print("Immediate controller response:")
        print(response.decode("utf-8", errors="replace").rstrip())
    else:
        print("No immediate controller response received before timeout.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
