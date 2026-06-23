# Commissioning Protocol

This protocol is for free-space validation of the UR5e direct-torque
commissioning script before any surface contact or lamination-process testing.
It is not a substitute for the robot cell risk assessment, UR safety
configuration, or site operating procedure.

## Preconditions

Before running any test:

- Confirm the robot is a UR5e running PolyScope 5.25.1.
- Confirm the active installation payload is correct.
- Confirm the active TCP is correct for the mounted tool or test fixture.
- Confirm the robot is in free space, away from fixtures, surfaces, and people.
- Confirm protective stops, reduced mode, and speed/force limits are configured
  for commissioning.
- Confirm the operator can immediately stop the program from the teach pendant.
- Confirm `ALLOW_TEST_4_BIASED_MOTION = False` unless intentionally running
  test 4 after tests 0 through 3 have passed.

Do not run these tests in contact with a surface. Do not use them as a
lamination controller.

## Advancement Gates

Run the tests in order. A test passes only if it stops on `time limit` and the
observed motion remains acceptable to the operator.

| Test | Required script settings | Expected stop reason | Pass condition |
| --- | --- | --- | --- |
| 0 | `TEST_ID = 0` | `time limit` | Direct torque entry is stable with zero extra torque. |
| 1 | `TEST_ID = 1` | `time limit` | Joint damping does not create unexpected motion or vibration. |
| 2 | `TEST_ID = 2` | `time limit` | Soft joint hold behaves smoothly around the starting joint pose. |
| 3 | `TEST_ID = 3` | `time limit` | Cartesian translational hold is smooth near the starting TCP pose. |
| 4 | `TEST_ID = 4`, `ALLOW_TEST_4_BIASED_MOTION = True` | `time limit` | The TCP bias produces only the expected small base-frame +X response. |

Any other stop reason is a failed commissioning step:

- `joint drift guard`
- `Cartesian X error guard`
- `Cartesian Y error guard`
- `Cartesian Z error guard`
- `unknown`

After a failed step, do not continue to later tests until the cause is understood
and the failed test has been repeated successfully.

## Test-Specific Observations

For test 0, watch for unexpected drift after entering `direct_torque()` with zero
extra torque. This test has no stiffness, so the joint drift guard is deliberately
tight.

For test 1, watch for oscillation, audible roughness, or damping that appears to
push instead of resist motion.

For test 2, gently perturb only if the cell procedure allows it. The response
should feel like a low-stiffness joint-space hold, not a hard position lock.

For test 3, the robot should resist base-frame TCP translation anisotropically:
X stiffest, Y moderate, Z softest. Orientation is not controlled by this test.

For test 4, expect intentional motion because the equilibrium point shifts by
0.003 m in base-frame +X. This test is gated in the script because it is the
first intentional-motion Cartesian test.

## Run Log Template

Copy one row per run into a project log or lab notebook.

| Field | Entry |
| --- | --- |
| Date/time |  |
| Operator |  |
| Robot serial/cell |  |
| PolyScope version | 5.25.1 |
| Payload confirmed | yes/no |
| TCP confirmed | yes/no |
| Tool/end-effector mounted |  |
| TEST_ID |  |
| ALLOW_TEST_4_BIASED_MOTION | True/False |
| Script SHA-256 |  |
| Sender JSONL log |  |
| Stop reason printed |  |
| Pass/fail |  |
| Maximum observed motion |  |
| Vibration/noise observed |  |
| Protective stop or fault |  |
| Notes |  |

## Criteria Before Contact Work

Do not begin surface-contact or force-control experiments until:

- Tests 0 through 3 have passed in free space.
- Test 4 has passed if biased Cartesian motion will be used in later tests.
- Payload and TCP values have been reviewed against the real mounted tool.
- The team has a separate contact-test protocol with force limits, tool geometry,
  surface geometry, and abort criteria.
- The controller has a defined surface-frame or contact-frame representation.

The current script validates early direct-torque and Cartesian impedance
behavior only. Surface-frame impedance, normal-force regulation, tangential
motion control, and tool-alignment impedance are future layers.
