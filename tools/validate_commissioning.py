#!/usr/bin/env python3
"""Static checks for the UR5e torque commissioning artifacts."""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT / "ur5e_direct_torque_anisotropic_impedance_tests.script"
README_PATH = ROOT / "README.md"
PROTOCOL_PATH = ROOT / "COMMISSIONING_PROTOCOL.md"
PARAMETER_PATH = ROOT / "PARAMETER_REFERENCE.md"
SENDER_PATH = ROOT / "tools" / "send_urscript_to_polyscope.py"
RTDE_SENDER_PATH = ROOT / "tools" / "setup_and_send_rtde.py"


EXPECTED_STOP_REASONS = {
    "joint drift guard",
    "Cartesian X error guard",
    "Cartesian Y error guard",
    "Cartesian Z error guard",
    "time limit",
    "unknown",
}


SCRIPT_PARAMETER_SNIPPETS = {
    "shared viscous scale": "viscous_scale = [0.9, 0.9, 0.8, 0.9, 0.9, 0.9]",
    "shared coulomb scale": "coulomb_scale = [0.8, 0.8, 0.7, 0.8, 0.8, 0.8]",
    "test 2 joint stiffness": "Kq = [2.0, 2.0, 1.5, 0.15, 0.15, 0.08]",
    "cartesian stiffness x": "Kx = 100.0",
    "cartesian stiffness y": "Ky = 60.0",
    "cartesian stiffness z": "Kz = 20.0",
    "cartesian damping x": "Dx = 20.0",
    "cartesian damping y": "Dy = 15.0",
    "cartesian damping z": "Dz = 10.0",
    "cartesian nullspace damping": "Dq_null = [0.08, 0.08, 0.06, 0.010, 0.010, 0.006]",
    "cartesian abort limit": "x_err_abort = 0.060",
    "cartesian clip limit": "x_err_clip = 0.025",
    "test 3 no bias": "cartesian_anisotropic_core(5.0, 0.0, 0.0, 0.0)",
    "test 4 x bias": "cartesian_anisotropic_core(4.0, 0.003, 0.0, 0.0)",
}


REFERENCE_PARAMETER_SNIPPETS = {
    "shared viscous scale": "[0.9, 0.9, 0.8, 0.9, 0.9, 0.9]",
    "shared coulomb scale": "[0.8, 0.8, 0.7, 0.8, 0.8, 0.8]",
    "test 2 joint stiffness": "[2.0, 2.0, 1.5, 0.15, 0.15, 0.08]",
    "cartesian stiffness": "`100.0`, `60.0`, `20.0`",
    "cartesian damping": "`20.0`, `15.0`, `10.0`",
    "cartesian nullspace damping": "[0.08, 0.08, 0.06, 0.010, 0.010, 0.006]",
    "cartesian abort limit": "`0.060` on X, Y, and Z",
    "cartesian clip limit": "`0.025` on X, Y, and Z",
    "test 3 no bias": "[0.0, 0.0, 0.0]",
    "test 4 x bias": "[0.003, 0.0, 0.0]",
}


def read(path):
    if not path.exists():
        return None
    return path.read_text(encoding="utf-8")


def check(condition, message, errors):
    if not condition:
        errors.append(message)


def count_urscript_blocks(script):
    opener_re = re.compile(r"^\s*(def|while|if)\b")
    end_re = re.compile(r"^\s*end\s*$")

    openers = 0
    ends = 0
    for line in script.splitlines():
        if opener_re.match(line):
            openers += 1
        if end_re.match(line):
            ends += 1

    return openers, ends


def main():
    errors = []

    script = read(SCRIPT_PATH)
    readme = read(README_PATH)
    protocol = read(PROTOCOL_PATH)
    parameter_ref = read(PARAMETER_PATH)
    sender = read(SENDER_PATH)
    rtde_sender = read(RTDE_SENDER_PATH)

    check(script is not None, f"missing {SCRIPT_PATH.name}", errors)
    check(readme is not None, f"missing {README_PATH.name}", errors)
    check(protocol is not None, f"missing {PROTOCOL_PATH.name}", errors)
    check(parameter_ref is not None, f"missing {PARAMETER_PATH.name}", errors)
    check(sender is not None, f"missing {SENDER_PATH}", errors)
    check(rtde_sender is not None, f"missing {RTDE_SENDER_PATH}", errors)

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    check("TEST_ID = 0" in script, "script should default to TEST_ID = 0", errors)
    check(
        "ALLOW_TEST_4_BIASED_MOTION = False" in script,
        "script should default ALLOW_TEST_4_BIASED_MOTION to False",
        errors,
    )

    for test_id in range(5):
        if test_id == 0:
            marker = "if TEST_ID == 0:"
        else:
            marker = f"elif TEST_ID == {test_id}:"
        check(marker in script, f"dispatcher missing TEST_ID {test_id}", errors)

    test4_gate_re = re.compile(
        r"elif TEST_ID == 4:\s*"
        r"if ALLOW_TEST_4_BIASED_MOTION:\s*"
        r"test_04_cartesian_anisotropic_x_bias\(\)\s*"
        r"else:\s*"
        r"popup\(\"TEST 4 requires ALLOW_TEST_4_BIASED_MOTION = True\.",
        re.MULTILINE,
    )
    check(test4_gate_re.search(script) is not None, "TEST_ID 4 is not gated", errors)
    check(
        "cartesian_anisotropic_core(4.0, 0.003, 0.0, 0.0)" in script,
        "test 4 should use the documented 3 mm base-frame +X bias",
        errors,
    )

    openers, ends = count_urscript_blocks(script)
    check(openers == ends, f"URScript block count mismatch: openers={openers}, ends={ends}", errors)

    check(
        re.search(r'textmsg\("[^"\n]*",', script) is None,
        "textmsg calls should remain single-string style",
        errors,
    )

    stop_reasons = set(re.findall(r'textmsg\("Stop reason: ([^"]+)"\)', script))
    check(
        stop_reasons == EXPECTED_STOP_REASONS,
        f"unexpected stop reasons: {sorted(stop_reasons)}",
        errors,
    )

    for reason in EXPECTED_STOP_REASONS:
        check(reason in protocol, f"protocol missing stop reason: {reason}", errors)

    check(
        "COMMISSIONING_PROTOCOL.md" in readme,
        "README should link the commissioning protocol",
        errors,
    )
    check(
        "PARAMETER_REFERENCE.md" in readme,
        "README should link the parameter reference",
        errors,
    )
    check(
        "tools/send_urscript_to_polyscope.py" in readme,
        "README should document the PolyScope sender",
        errors,
    )
    check(
        "tools/setup_and_send_rtde.py" in readme,
        "README should document the RTDE setup/send helper",
        errors,
    )
    check(
        "Do not run these tests in contact with a surface" in protocol,
        "protocol should explicitly forbid contact during these tests",
        errors,
    )
    check(
        "| Script SHA-256 |" in protocol,
        "protocol run log should include script SHA-256",
        errors,
    )
    check(
        "| Sender JSONL log |" in protocol,
        "protocol run log should include sender JSONL log path",
        errors,
    )
    check(
        "`ALLOW_TEST_4_BIASED_MOTION = True`" in readme
        and "`ALLOW_TEST_4_BIASED_MOTION = True`" in protocol,
        "README and protocol should document the test 4 opt-in gate",
        errors,
    )

    for label, snippet in SCRIPT_PARAMETER_SNIPPETS.items():
        check(snippet in script, f"script missing parameter marker: {label}", errors)

    for label, snippet in REFERENCE_PARAMETER_SNIPPETS.items():
        check(snippet in parameter_ref, f"parameter reference missing marker: {label}", errors)

    check(
        "Do not tune these free-space values in contact with a surface" in parameter_ref,
        "parameter reference should keep tuning out of contact tests",
        errors,
    )

    check(
        'DEFAULT_ROBOT_IP = "192.168.1.2"' in sender,
        "sender should default to the documented robot IP",
        errors,
    )
    check(
        "DEFAULT_COMMAND_PORT = 30002" in sender,
        "sender should default to the UR secondary-client command port",
        errors,
    )
    check(
        "DEFAULT_SCRIPT_PATH = ROOT / \"ur5e_direct_torque_anisotropic_impedance_tests.script\"" in sender,
        "sender should default to the commissioning script",
        errors,
    )
    check(
        "--dry-run" in sender and "--yes" in sender and "--skip-validation" in sender,
        "sender should support dry-run, confirmation override, and validation override",
        errors,
    )
    check(
        "--expect-test-id" in sender and "parse_int_assignment" in sender,
        "sender should support intended TEST_ID checking",
        errors,
    )
    check(
        "--log-run" in sender and "append_jsonl" in sender and "make_log_record" in sender,
        "sender should support local JSON Lines send logging",
        errors,
    )
    check(
        "ensure_log_writable" in sender and "cannot write send log" in sender,
        "sender should handle unwritable send logs cleanly",
        errors,
    )
    check(
        "run_static_validator()" in sender,
        "sender should run static validation before real sends",
        errors,
    )
    check(
        "hashlib.sha256" in sender and "SHA-256:" in sender,
        "sender should print the script payload SHA-256",
        errors,
    )
    check(
        "--expect-test-id 0" in readme,
        "README sender examples should include expected TEST_ID checking",
        errors,
    )
    check(
        "--log-run logs\\send_log.jsonl" in readme,
        "README sender examples should include local send logging",
        errors,
    )
    check(
        "RTDEControlInterface" in rtde_sender,
        "RTDE helper should use ur_rtde RTDEControlInterface",
        errors,
    )
    check(
        "setPayload" in rtde_sender
        and "setTcp" in rtde_sender
        and "zeroFtSensor" in rtde_sender
        and "sendCustomScriptFile" in rtde_sender,
        "RTDE helper should set payload, set TCP, tare FT sensor, and send script file",
        errors,
    )
    check(
        "--payload-mass" in rtde_sender
        and "--payload-cog" in rtde_sender
        and "--tcp" in rtde_sender
        and "--dry-run" in rtde_sender
        and "--expect-test-id" in rtde_sender,
        "RTDE helper should require payload/TCP values and support dry-run/test-id guard",
        errors,
    )

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    print("PASS: commissioning artifacts are internally consistent")
    print(f"PASS: URScript block balance openers={openers} ends={ends}")
    print("PASS: test 4 biased motion remains explicitly gated")
    print("PASS: parameter reference includes key initial gains and guards")
    print("PASS: PolyScope sender defaults to 192.168.1.2:30002")
    print("PASS: PolyScope sender runs static validation before real sends")
    print("PASS: PolyScope sender prints script payload SHA-256")
    print("PASS: PolyScope sender supports intended TEST_ID checking")
    print("PASS: PolyScope sender supports local JSON Lines send logging")
    print("PASS: PolyScope sender handles unwritable send logs cleanly")
    print("PASS: RTDE helper sets payload/TCP, tares FT sensor, and sends script file")
    return 0


if __name__ == "__main__":
    sys.exit(main())
