# GeoDude: UR5e External Impedance & Admittance Controllers (RTDE)

Project focused on developing **external** (Python-side) impedance and admittance controllers for the Universal Robots UR5e using the **RTDE** interface.

## Why External Controllers (instead of force_mode)?

- **URSim (Software-in-the-Loop)**: Excellent for core logic, math, gain tuning (free space), state machines, high-rate RTDE comms (pose/velocity/wrench read + servoj/speedj), timing validation.
- **Major limitation of URSim**: `force_mode()` (and Polyscope Force nodes) do **not** behave realistically. Contact physics are weak/absent and wrench feedback is unrealistic or absent. Confirmed via testing.
- **Recommendation** (from project discussion): Use **external Python RTDE loops** (admittance/impedance) for development. These work well even in URSim for everything except final contact-rich validation.
- Built-in `forceMode()` examples are kept for reference but **not recommended** for this project.

For full contact-rich validation (constant force while tracing a surface, etc.) you will eventually need:
- Real UR5e hardware (with proper risk assessment, safety configuration, and reduced speed/momentum), **or**
- A higher-fidelity simulator with proper contact physics, such as **NVIDIA Isaac Sim**.

URSim is excellent for developing the **logic, state machines, gains in free space, and RTDE communication**, but its force_mode and wrench feedback are not realistic enough to prove consistent contact force. Isaac Sim gives you proper rigid-body contact forces, friction, and material properties, so you can actually measure whether your controller is maintaining a target normal force while moving across a surface.

## Current Status (2026-06-14)

- URSim (UR5e) running in Docker on Linux host.
- VNC/Polyscope access: `localhost:6080` (browser) or `localhost:5900` (VNC client).
- User can iterate Python RTDE controller scripts **without physical robot access**.
- Force mode testing performed; external loops are the path forward.
- This repo provides clean, reusable implementations + examples for **external admittance** and **impedance** controllers.

## RTDE + URSim Docker Quick Setup Notes

**Important:** The command you are currently using:

```bash
docker run --rm -it universalrobots/ursim_e-series
```

does **not** publish any ports from the container to your Linux host. This means:

- You cannot access Polyscope at `localhost:6080` or `localhost:5900` from the host.
- Any RTDE or script clients running on the host cannot connect (port 30004 for RTDE, 30002 for simple script sending, etc.).

**Crucial clarification (answering "Do I need to install/compile RTDE on the client?")**:

**No.** RTDE is a protocol that is **already built into the robot controller** (real UR5e or URSim inside the Docker container). You do **not** install or compile anything on the "server" side.

What lives on your development machine (the client) is a **client library** that knows how to speak the RTDE protocol (or the simpler script interface) over TCP to the controller.

There are different ways to talk to a UR/URSim:

- **Port 30002 (Secondary client)**: Just send URScript text. Extremely simple. The current `force_mode_example.py` uses this and needs **zero** extra packages.
- **Port 30004 (RTDE)**: The proper bidirectional protocol for external closed-loop control (high-rate pose + wrench feedback + streaming commands like speedj/servoj/forceMode). This is what the project title refers to. You need a client library that implements the RTDE protocol.

Options for an RTDE client on your PC:
- The official pure-Python one from Universal Robots (recommended for avoiding compile pain): install with `pip install git+https://github.com/UniversalRobots/RTDE_Python_Client_Library.git`
- `ur_rtde` (the one that was hard to build): convenient high-level API but has C++ dependencies.
- Roll your own with sockets + struct (possible but a lot of work).

### Correct command (required for host ↔ URSim communication)

```bash
docker run --rm -it \
  -p 5900:5900 \
  -p 6080:6080 \
  -p 29999:29999 \
  -p 30001-30004:30001-30004 \
  universalrobots/ursim_e-series
```

Or use the helper script we added to this repo:

```bash
chmod +x start_ursim.sh
./start_ursim.sh
```

### What the ports give you
- `http://localhost:6080` → noVNC / Polyscope UI (easiest)
- VNC client on `localhost:5900`
- RTDE on `localhost` port 30004 (what all the Python scripts in this repo use)
- Dashboard on 29999 (useful for remote power-on, loading programs, etc.)

After the container starts:
1. Wait 30–90 seconds on first boot until the Polyscope interface is responsive in the browser.
2. In Polyscope: Power on the virtual robot.
3. Put it into **REMOTE** control mode (critical for external RTDE commands such as `forceMode()`).
4. Clear any protective stops.

Now you can run from another terminal on the host:

```bash
python force_mode_example.py
```

and watch the effect in the Polyscope VNC window at localhost:6080.

Other tips:
- The `--rm` flag means the container (and any unsaved programs) disappears when you exit. For development work, consider adding volume mounts so your Polyscope programs persist (see `start_ursim.sh` for commented examples).
- Alternative (advanced): `--network host` removes the need for most `-p` flags but has security and port conflict implications.
- The official image runs a UR5e model by default.

## Installation (on the host / dev machine that talks RTDE)

**Stupid simple (no compilation, no Boost, no cmake):**

The project now uses the official pure-Python RTDE client from Universal Robots.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

That's it. `requirements.txt` pulls the official client directly.

### Quick connectivity test (URSim must be running with ports forwarded via start_ursim.sh or equivalent)

```bash
python -c "
from utils.rtde_utils import RtdeInterface
iface = RtdeInterface('localhost')
if iface.connect():
    print('Connected!')
    print('Pose:', iface.get_state().tcp_pose)
    iface.disconnect()
"
```

## Contact-Rich Validation: Moving Beyond URSim

**Yes — Isaac Sim is the right tool for proving consistent force while tracing a surface.**

### Why URSim falls short for your goal
URSim (the official Docker image) has very weak contact physics and unrealistic wrench feedback. As you've experienced:
- Built-in `force_mode()` often reports near-zero or noisy forces even when active.
- The arm can drift or hit singularities unexpectedly because there's no real surface reaction.
- You cannot credibly "prove" that a constant normal force is being maintained while moving tangentially.

URSim is excellent for:
- Developing and debugging your external RTDE control loops (the admittance/impedance controllers).
- Testing timing, state machines, gain tuning in free space.
- Validating communication with the real robot's interfaces.

It is **not** suitable for contact-rich validation.

### Isaac Sim advantages for this use case
NVIDIA Isaac Sim uses PhysX with proper rigid-body dynamics, contact forces, friction models, and material properties. This means:

- You get **ground-truth simulated contact forces** (normal and tangential) directly from the physics engine.
- You can model a real surface (flat, curved, with realistic stiffness and friction).
- Your Python external controllers (`controllers/admittance.py` and `impedance.py`) become very powerful here:
  - Command a trajectory in the plane (X/Y motion across the surface).
  - Let the admittance controller maintain a target normal force by placing the virtual reference pose slightly "into" the surface.
  - Log the actual contact force reported by the simulator over time.
  - Quantify consistency (mean error, standard deviation, max deviation from target during the entire trace).

This gives you something you can actually show as evidence that "the controller applies consistent force while tracing."

### Recommended approach
1. Set up Isaac Sim (requires a decent GPU) with the UR5e asset (or URDF import) + a surface prim with a Physics Material.
2. Interface via ROS 2 (Isaac has excellent ROS 2 bridge) or the native Python API.
3. Adapt your existing `rtde_utils.py` + admittance controller:
   - Subscribe to joint states + simulated wrench / contact reports.
   - Publish joint velocities or Cartesian velocity commands (the controller already outputs velocity).
4. Implement a simple surface tracing task: constant or sinusoidal motion in X while the Z force is regulated.
5. Log and plot the normal force component. You should see it stay close to your target (e.g. 10 N) with low variance once contact is established, even as the arm moves.

The external controllers you already built are designed for exactly this kind of high-fidelity sim development. The built-in `force_mode()` path (the telemetry script) is more of a quick test for the UR interface itself.

### Limitations to be aware of
- Sim-to-real gap still exists (sensor noise, exact material properties, unmodeled compliance, timing differences).
- Isaac Sim contact forces are still simulated — the ultimate proof will be on real hardware with a calibrated force/torque sensor or instrumented surface.
- You will need to port the communication layer (no direct RTDE to Isaac; use ROS 2 topics or the Sim API).

If you want, we can:
- Clean up / extend the admittance controller with better surface contact logic and logging.
- Add a simple Isaac Sim example script (using the ROS 2 bridge) that reuses your existing code.
- Update the telemetry script or controllers to make the transition easier.

Just tell me the next concrete step you'd like help with. Isaac Sim + your external controllers is the correct path for what you're trying to prove.

## Validating Consistent Force on a Surface (Isaac Sim)

**Yes — Isaac Sim is a much better platform for proving that the robot can apply a consistent force while tracing a surface.**

### Why URSim is insufficient for this goal
URSim has very limited contact physics and unrealistic wrench feedback. As you've seen, `force_mode()` often produces near-zero or noisy force values even when commanded, and the arm can drift or hit singularities unexpectedly. It is excellent for developing the **logic, RTDE communication, timing, and free-space gains** of your external controllers, but it cannot credibly demonstrate "consistent normal force while moving across a surface."

### Why Isaac Sim works well
- NVIDIA Isaac Sim uses PhysX with proper rigid-body contact, friction, and material properties.
- You can get **ground-truth contact forces** from the physics engine (far more reliable than URSim's reported TCP force).
- Your existing **external admittance and impedance controllers** (`controllers/admittance.py`, `controllers/impedance.py`) are designed exactly for this kind of development:
  - Define a virtual surface.
  - Command tangential motion (e.g. a trajectory in X/Y).
  - Use the controller to maintain a target normal force (by placing the virtual reference pose slightly "into" the surface).
  - Log the actual simulated contact force over time and show that the normal component stays close to your target (e.g. 10 N ± small tolerance) while the tangential motion is executed.

This gives you a credible way to demonstrate and tune "constant force while tracing" in simulation before hardware.

### Practical path forward
1. Set up Isaac Sim with the UR5e (official asset or URDF import) + a surface with realistic friction.
2. Use ROS 2 (recommended) or the direct Python API to interface.
3. Adapt your `rtde_utils.py` + admittance/impedance controllers to publish velocity or pose commands and subscribe to simulated wrench / contact data.
4. Implement a surface-tracing task: constant or sinusoidal motion in the plane + force maintenance in the normal.
5. Log and plot the measured normal force vs. time / position. Compute statistics (mean, std, max deviation) to "prove" consistency.

### Limitations (still important)
- Sim-to-real gap remains (sensor noise, exact surface properties, timing, unmodeled compliance, etc.).
- Isaac Sim gives you much better evidence than URSim, but final validation should still be done on real hardware with a calibrated force sensor or instrumented surface.
- The built-in `force_mode()` behavior of the real UR controller will not be identical to what you simulate with an external loop.

If you'd like, we can start adapting one of the existing external controller examples or `force_mode_telemetry.py` style logic for Isaac Sim + ROS 2. Just say the word and we can keep it stupid simple.

Would you like me to outline the minimal changes needed to run your current admittance controller against a surface in Isaac Sim, or update any docs/scripts here first?

## Project Structure

```
.
├── README.md
├── requirements.txt
├── force_mode_example.py          # Simple zero-dependency script to enable forceMode in URSim (uses only stdlib socket)
├── controllers/
│   ├── __init__.py
│   ├── admittance.py              # External admittance controller
│   └── impedance.py               # External impedance controller
├── utils/
│   ├── __init__.py
│   ├── rtde_utils.py              # Connection wrapper, safety helpers, rate loop
│   └── filters.py                 # Wrench / signal filters
├── examples/
│   ├── basic_rtde_loop.py
│   ├── run_admittance_free_space.py
│   └── run_impedance_free_space.py
└── .gitignore
```

## Controller Concepts

### Admittance Control (external)
"Force in → motion out".

- Measures external wrench via RTDE.
- Computes a compliant velocity (or incremental pose) command.
- Sends via `speedL(...)` (Cartesian velocity) or integrates to a servo target.
- Typical use: human hand guiding, exploration, force-responsive behavior.
- Gains: primarily high **damping (D)**, optional **mass (M)**, low/zero **stiffness (K)** to a reference.
- When no force: stays put or slowly returns (if K > 0).

### Impedance Control (external)
"Position deviation → restoring force" (virtual spring + damper).

- Has a strong **desired pose** attractor.
- External wrench causes a (filtered) deviation from the desired pose.
- The loop continuously drives toward a "virtual" target that is offset by `F / K`.
- Sends pose updates via repeated `servoJ` / `speedL` or similar.
- When released: springs back toward the original desired pose.
- Good for "soft position control" that yields to contact.

Both are implemented as **discrete 6-DOF decoupled** (translational + rotational) mass-spring-damper models updated in the outer RTDE loop (typical 100-125 Hz).

**Important**: Rotational units are radians / Nm. Linear are meters / N. Tune separately.

## Recommended Development Flow (SIL on URSim)

1. **Free-space validation first** (no contact expected in sim):
   - Run basic RTDE loop, confirm rates, logging, no drift, correct connection.
   - Run admittance/impedance examples with **simulated wrench injection** (see examples).
   - Tune gains so behavior is stable and responsive in free space.
   - Verify timing (loop jitter), safety stops, reconnection logic.
   - State machine skeleton (IDLE → COMPLIANT → HOLD, etc.).

2. Observe in Polyscope while script runs (joint positions, TCP, etc.).

3. **Contact behavior**: Treat as logic-only in URSim. Any wrench you see will be unrealistic. Use this phase for:
   - State machine transitions on force thresholds.
   - Gain scheduling.
   - Trajectory blending + compliance.
   - Graceful stop / recovery.

4. **Hardware or high-fidelity sim** (Isaac Sim etc.):
   - Move to real UR5e (start at very low speed, use safety planes, reduced mode, have e-stop ready).
   - Or Isaac Sim with proper UR5e + contact materials + FT sensor plugin.
   - Re-tune gains (real robot dynamics + sensor noise differ).

## Example Usage

See scripts in `examples/`.

Quick start (admittance, free space with fake force for testing):

```bash
python examples/run_admittance_free_space.py
```

Inside the script you can toggle `USE_SIMULATED_WRENCH = True` to inject synthetic forces and observe the robot respond (directionally) even in URSim.

Same for impedance examples.

## Gain Tuning Advice (Free Space)

Start conservative:

- **Admittance**:
  - D (damping) high (e.g. 80–300 Ns/m for linear, 5–20 for angular).
  - M low (1–10 kg or less).
  - K very low or 0 (spring return to a home pose if desired).
  - Lowpass alpha on wrench: 0.05–0.25.

- **Impedance**:
  - K (stiffness) moderate to high depending on how "soft" you want (e.g. 200–2000 N/m linear).
  - D chosen for critical damping or slightly overdamped (D ≈ 2*sqrt(K*M)).
  - Small deadband on wrench to avoid drift from sensor noise/offset.

Always test one axis at a time. Monitor for oscillation.

Use the `utils` filters and the built-in logging in the example runners.

## Safety & Best Practices

- **Never** run at full speed first.
- Always have a way to kill the script (Ctrl-C handler that calls `speedStop()` / `servoStop()` / `stopScript()`).
- Check `getSafetyStatusBits()`, `getRobotStatus()` etc. in the receive interface.
- On real robot: configure safety in Polyscope (force/torque limits, planes, speed limits).
- For RTDE external control the robot must be in Remote mode.
- Log everything (pose, wrench, commands, timestamps) during experiments.
- On real hardware, start with a human nearby who can press the e-stop / pause.

## Transition Roadmap

- [x] URSim Docker + VNC + basic RTDE connectivity (you have this)
- [x] External admittance + impedance Python implementations (this repo)
- [ ] Free-space + state-machine testing in URSim
- [ ] Logging + visualization tools
- [ ] Real UR5e bring-up (low speed, monitored)
- [ ] Isaac Sim (or equivalent) contact validation
- [ ] Full task implementation (e.g. assembly, hand-guiding, force-controlled insertion...)

## References & Further Reading

- ur_rtde documentation & examples: https://sdurobotics.gitlab.io/ur_rtde/
- Universal Robots RTDE Guide (official)
- URScript manual (servoj, speedj, speedl parameters, t, lookahead_time, gain)
- Forum threads on streaming servoj at 125 Hz / 500 Hz update rates

## License / Notes

Internal project tooling. Adapt as needed. When moving to hardware, follow your organization's robot safety procedures.

---

**Next immediate actions suggested**:
- Run the examples against your URSim.
- Extend the state machine in `utils/rtde_utils.py`.
- Add data logging + simple plots (matplotlib) for gain tuning sessions.
- Once stable in free space, plan the hardware or Isaac Sim validation step.

Happy controlling!
