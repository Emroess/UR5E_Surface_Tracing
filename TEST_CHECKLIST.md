# UR5e Force Mode Test Checklist

Use this template to log each test run. One entry per run.  
Be disciplined: change **only one thing** between runs when possible.

---

## Test Run Log Template

**Run #:** ___  
**Date / Time:** ___  
**Goal of this run:** (e.g. "Phase 3 - simple contact + 3N hold, no trace")

### Script Settings Used
- `FREE_AIR_TEST`: `True` / `False`
- `FORCE_Z`: ___ N
- `DAMPING`: ___
- `GAIN_SCALING`: ___
- `TRACE_VEL`: ___ m/s   (`TRACE_ACC`: ___)
- `PAYLOAD_MASS`: ___ kg   (CoG: ___)
- Other notes: ___

### Observations
- **Reached START pose?**  [ ] Yes  [ ] No  [ ] Partial
- **0 mm APPROACH completed?**  [ ] Yes  [ ] No
- **force_mode enabled?** (look for "force_mode ENABLED" in robot log)  [ ] Yes  [ ] No
- **Force built after enable?**  
  - Target: ___ N  
  - Peak Fz seen: ___ N  
  - Steady Fz (after 3–5s): ___ N
- **Drift / oscillation in Z after force_mode?**  [ ] None  [ ] Mild  [ ] Severe
- **Did the arm start moving toward END pose?**  [ ] Yes (smooth)  [ ] Yes (jerky/drift)  [ ] No (stayed or drifted away)
- **Reached / got close to END pose?**  [ ] Yes  [ ] Close  [ ] No
- **Robot log messages seen** (from `textmsg`):
  - List key ones here (e.g. "force_mode ENABLED", "Starting TRACE...", "PROGRAM END")
- **Polyscope / physical notes**: (e.g. "pushed into surface hard", "vibration felt", "protective stop on Z", "looked stable")
- **Python console force output** (copy a few lines of `F=[...]` during key phases):
  ```
  (paste 3–5 lines here)
  ```

### Outcome & Next Step
- **This run was:**  [ ] Success (met goal)  [ ] Partial  [ ] Failure
- **What worked well:**
  ___
- **What went wrong / unexpected:**
  ___
- **Single change for next run:**
  ___
- **Next planned test:**
  ___

---

## Quick Phase Checklist (copy & mark as you go)

### Phase 1 – Motion Sequence Only (FREE_AIR_TEST = True, no contact)
- [ ] Robot reaches start pose cleanly
- [ ] 0 mm approach movel happens
- [ ] Arm moves toward end joints
- [ ] Program completes without stops
- [ ] No big unexpected motion after force_mode "enable" message

**Pass this phase before continuing.**

### Phase 2 – Force Mode in Free Space (FREE_AIR_TEST = True)
- [ ] After "force_mode ENABLED" message, arm stays reasonably still
- [ ] Printed forces stay near zero (some noise OK)
- [ ] No aggressive drive in Z

### Phase 3 – Simple Hold with Force (FREE_AIR_TEST = False, no trace motion)
Temporarily comment out or replace the final `movej(end)` with `sleep(8)` for this phase.

- [ ] Tool makes light contact after 0 mm approach
- [ ] After "force_mode ENABLED", Fz rises and stabilizes near target (within ~1 N)
- [ ] Arm does not oscillate badly or drift far in Z
- [ ] Robot log shows clear "force_mode ENABLED" and "Starting DWELL..."

**Target for this phase: stable ~3 N hold with minimal oscillation.**

### Phase 4 – Trace Under Force (Full program)
- [ ] Use low `TRACE_VEL` (0.02 or slower)
- [ ] After "Starting TRACE move to END pose", arm begins moving toward end joints
- [ ] Fz stays in a reasonable band around target during motion (note peaks and valleys)
- [ ] Arm makes visible progress toward end joints (even if it doesn't arrive perfectly)
- [ ] "TRACE movej completed" and "DWELL complete" messages appear
- [ ] No protective stops

### Phase 5 – Increase Force & Refine
- [ ] Gradually raise `FORCE_Z` only after Phase 4 is repeatable at lower value
- [ ] Re-teach END_JOINTS while force_mode + contact is active (highly recommended)
- [ ] Note how much Fz deviates during the full trace

---

## Tips for Good Data

- Always note the **exact robot log messages** (the `textmsg` lines) next to the Python force output.
- Copy 4–6 lines of the `F=[Fx Fy Fz ...]` output right after "force_mode ENABLED" and again during the trace.
- Take a quick photo or note the arm posture if it ends up in an unexpected place.
- If something surprising happens, **do not change multiple things** before the next run. Change one variable and re-test.

---

Copy the "Test Run Log Template" section for each actual test you perform.  
Keep this file in your project folder so you can refer back to what worked.

Good luck — small, documented steps will get you there faster than big jumps!