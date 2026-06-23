# UR5e Direct Torque Anisotropic Impedance

This repository contains an early commissioning script for validating direct
joint-torque control on a Universal Robots UR5e running PolyScope 5.25.1. The
larger project direction is high-dexterity robotic surface tracing for composite
lamination, where a roller or lamination end-effector will eventually trace a
surface while controlling contact behavior, normal force, tangential motion, and
tool alignment.

The current scope is intentionally smaller: safely validate `direct_torque()`,
joint damping, joint-space impedance, and a first base-frame Cartesian
translational impedance controller.

## Platform Assumptions

- Robot: Universal Robots UR5e.
- Controller software: PolyScope 5.25.1.
- Torque interface: Direct Torque Control V2 through URScript.
- Execution direction: on-controller URScript loop for deterministic timing.
- Active installation payload and TCP must already be correct before running the
  script.

The commissioning script does not set payload, set TCP, tare the force/torque
sensor, use RTDE, use sockets, control orientation impedance, or perform
surface-frame control.

Use `COMMISSIONING_PROTOCOL.md` as the go/no-go procedure and run-log template
for first robot trials.

Use `PARAMETER_REFERENCE.md` as the compact record of initial gains, limits,
guards, durations, and test biases.

## Script Entry Point

The active test is selected by editing `TEST_ID` in
`ur5e_direct_torque_anisotropic_impedance_tests.script` and running one test at a
time:

| TEST_ID | Test | Purpose |
| --- | --- | --- |
| 0 | Zero extra torque | Enter `direct_torque()` with zero commanded extra torque. |
| 1 | Joint damping only | Apply low joint damping against measured joint velocity. |
| 2 | Joint-space impedance hold | Hold the starting joint configuration with low stiffness and damping. |
| 3 | Cartesian anisotropic hold | Hold the starting TCP translation in the robot base frame. |
| 4 | Cartesian anisotropic +X bias | Shift the base-frame X equilibrium by 3 mm after earlier tests pass. |

Test 4 is intentionally gated. To run it, set `TEST_ID = 4` and also set
`ALLOW_TEST_4_BIASED_MOTION = True` in the script. Leave that flag `False`
during tests 0 through 3.

## Common Safety Pattern

Each test follows the same commissioning pattern:

1. Capture the starting robot state as the reference.
2. Run a timed control loop using `get_steptime()`.
3. Read joint position and velocity, and for Cartesian tests also TCP pose and
   TCP speed.
4. Abort the loop if joint or Cartesian drift exceeds configured limits.
5. Clamp commanded torque magnitude.
6. Rate-limit torque changes before sending them to `direct_torque()`.
7. Ramp commanded torque back to zero.
8. Call `stopj(2.0)` to return to normal stopping behavior.

After shutdown, the script prints a stop reason such as time limit, joint drift
guard, or Cartesian axis error guard. Treat any guard-triggered stop as a failed
commissioning step until the cause is understood.

If a guard fires during a loop cycle, the script skips new torque-command
generation and proceeds to the zero-torque ramp.

This structure is meant for staged free-space validation before any surface
contact experiments.

## Controller Behavior

The joint damping test computes:

```text
tau = -Dq * qdot
```

The joint-space impedance test computes:

```text
tau = Kq * (q_ref - q) - Dq * qdot
```

The Cartesian impedance core captures the current TCP pose as the base-frame
equilibrium and computes only translational impedance:

```text
F = K * (x_ref - x) - D * xdot
tau = J^T * [Fx, Fy, Fz, 0, 0, 0] - Dq_null * qdot
```

The translational stiffness is anisotropic by design:

- X: 100 N/m
- Y: 60 N/m
- Z: 20 N/m

Orientation impedance is deliberately omitted from this first controller. Small
joint damping is added after the Jacobian transpose mapping to reduce uncontrolled
nullspace and wrist motion.

## Commissioning Sequence

Run tests in order, and only advance after the previous stage behaves safely:

1. Confirm payload, TCP, robot workspace, protective stops, and teach pendant
   access.
2. Run test 0 and verify `direct_torque()` entry is stable with zero extra
   torque.
3. Run test 1 and verify joint damping does not introduce unexpected motion.
4. Run test 2 and verify the soft joint hold behaves as expected.
5. Run test 3 and verify base-frame translational impedance near the starting
   TCP pose.
6. Run test 4 only after tests 0 through 3 pass, and only after setting
   `ALLOW_TEST_4_BIASED_MOTION = True`, because it intentionally shifts the
   equilibrium point by 3 mm in base-frame +X.

## Static Validation

Before loading an edited script onto the robot, run:

```powershell
& 'C:\Users\Lenovo ThinkPad\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' tools\validate_commissioning.py
```

The validator checks default-safe script settings, the test 4 opt-in gate,
stop-reason consistency between the script and protocol, simple URScript block
balance, and conservative `textmsg()` usage.

## Sending To PolyScope

The helper script `tools/send_urscript_to_polyscope.py` sends the commissioning
`.script` file to the UR secondary-client command port. Its defaults are:

- Robot IP: `192.168.1.2`
- Command port: `30002`

Preview the send without opening a robot socket:

```powershell
& 'C:\Users\Lenovo ThinkPad\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' tools\send_urscript_to_polyscope.py --dry-run --expect-test-id 0 --log-run logs\send_log.jsonl
```

Send to the robot after confirming PolyScope is in remote mode, payload/TCP are
correct, and the robot is in free space:

```powershell
& 'C:\Users\Lenovo ThinkPad\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' tools\send_urscript_to_polyscope.py --expect-test-id 0
```

For a real send, the helper runs the static validator before opening the robot
socket and prints a SHA-256 hash of the exact script payload. Record that hash in
the commissioning log. Use `--expect-test-id N` to make the sender abort if the
script is not set to the intended test. Use `--log-run logs\send_log.jsonl` to
append a local JSON Lines record of dry-runs, aborted sends, and successful sends.
For non-interactive use, add `--yes` to skip the confirmation prompt. Use
`--skip-validation` only when intentionally bypassing that pre-send check.

## RTDE Setup And Send

The helper script `tools/setup_and_send_rtde.py` uses the external `ur_rtde`
Python package to set payload, set TCP, tare the built-in force/torque sensor
with `zeroFtSensor()`, and send the `.script` file with
`sendCustomScriptFile()`.

Preview the operation without connecting to the robot:

```powershell
& 'C:\Users\Lenovo ThinkPad\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' tools\setup_and_send_rtde.py --dry-run --expect-test-id 0 --payload-mass <kg> --payload-cog <x> <y> <z> --tcp <x> <y> <z> <rx> <ry> <rz>
```

For a real run, remove `--dry-run` only after replacing every payload/TCP value
with measured values for the mounted tool:

```powershell
& 'C:\Users\Lenovo ThinkPad\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' tools\setup_and_send_rtde.py --expect-test-id 0 --payload-mass <kg> --payload-cog <x> <y> <z> --tcp <x> <y> <z> <rx> <ry> <rz>
```

This helper requires PolyScope remote mode and a Python environment with
`ur_rtde` installed. Do not use placeholder payload or TCP values on hardware.

## Path Toward Surface Tracing

The next control-development steps should preserve the staged validation style:

- Record structured observations from the commissioning protocol, then add
  on-controller or external logging for commanded torque, TCP error, and abort
  reason as the test campaign matures.
- Add a surface-frame representation once surface geometry and contact sensing
  are available.
- Add normal-force regulation and tangential impedance as separate validated
  layers.
- Add orientation/tool-alignment impedance only after translational behavior is
  understood.
- Keep free-space and contact commissioning tests separate.
