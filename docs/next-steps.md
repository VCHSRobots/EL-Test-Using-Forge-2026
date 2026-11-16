# Elevator stand: suggested next steps (after 16 Nov 2026)

Read `findings-2026-11-16.md` first; this list builds on it. Each step says what it settles. Every stand test is one Forge card, the person enables in Teleop, and the script ends **parked, motors off, waiting for the person's disable** (Forge's disable does not work while a script runs). Score from the robot's WPILOG, not the script's samples.

## 1. Find what changes the Talons' state (the load-sharing fault)

**1a. Repeat-start test, no code change (EL35 is enough).** One enable: `sololeft` x6 and `soloright` x6, alternating, each a fresh routine start, 30 in/s, 200 in/s^2, 6 V, 40 A. From the robot log, per routine: cruise volts, torque current, apparent back-EMF constant (volts minus IR over rotor speed), down-move current sign.
- If the constant jumps **between** starts and never **within** one, the trigger is the routine start (expected from run 2e332d79).
- If it also drifts within a routine, the trigger is something else (temperature, time).

**1b. Separate the trigger (one small build).** Add routines that repeat full travel **without** rewriting configs, only switching the control request, and others that rewrite configs but keep the same request (for example `fulltravel_noconfig`, `fulltravel_reconfig`). Run each x4 with one motor alone. Whichever shows the jumps is the trigger. If config writes are the trigger, stop rewriting configs at routine starts (write once while disabled; change Motion Magic values with the request's own fields where possible).

**1c. Log faster around starts.** The 512 flag (`StatorCurrLimit`) lit at logged 15-28 A with a 40 A limit. Raise the update rate of stator current, torque current, motor volts and rotor velocity on both Talons to 1 kHz for the test, or record a Phoenix signal log (hoot) with `SignalLogger`, to see whether real spikes exist or the flag fires early.

**1d. Hardware-side checks (a person, in Phoenix Tuner X, no code):**
- Self-test snapshot on both Talons (CAN 10, 11): any faults, and what it reports for the current-limit state.
- Re-flash 26.3.0.0 on both and repeat 1a. No change means it is not a corrupt flash.
- Check whether the Talons have **Phoenix Pro** licences. If they do, try **FOC** (`MotionMagicVoltage` with `EnableFOC`, or `MotionMagicTorqueCurrentFOC`), which handles the rotor angle differently; repeat 1a.
- Optional, to rule out the mechanism completely: take one pinion out of mesh and spin that motor free at fixed volts across several starts.

Decision rule: if the fault stays unexplained but stable, it is acceptable for accuracy (the goal was met with it present). It matters for headroom: on bad crossings about half the left motor's push is cancelled.

## 2. Make the fast settings the default (small PR)

Proposed defaults, from run f72f9052: **40 A, 6 V, 45 in/s, 300-400 in/s^2, jerk off**, kP 8, kD 0.10, kV 0.0643. Earlier advice was to wait for step 1 before defaulting to 400 in/s^2; a reasonable middle is **300 in/s^2 now** (0.33 s, 0.07 in) and 400 after step 1. Dal decides. After the PR: one card running `fulltravel` and `moves` on the new defaults with no tuning writes, to confirm the defaults alone reproduce the result, and a `thing_note` on the stand.

## 3. Margin on the 0.5 s goal: raise cruise speed

At 400 in/s^2 the carriage reaches 45 in/s, so cruise now limits the crossing. Sweep cruise 45 / 50 / 55 / 60 in/s (60 is the code's ceiling) at 400 in/s^2, 40 A, 6-8 V; score crossing time, worst error, overshoot, peak volts and current, and the 512 flag. Watch voltage: 60 in/s needs about 0.28 + 60 x 0.0643 = 4.1 V of feedforward before kP, so 6 V still has room.

## 4. Holding accuracy (the 0.02 in part of the goal)

Never scored properly from the robot log: the script saw too few samples during holds. Measure hold error and wobble from the WPILOG on the defaults from step 2, at the bottom, middle and top, for 2 s each.

## 5. The final use: joystick and the long dance

- Teleop: stick port 0 axis 1 sets the height across the measured range. Check it feels right at the fast defaults (Forge can drive a virtual pad on a card, or a person drives).
- Autonomous: run the dance for its full 180 s and then longer (`DanceSeconds` up to 1800). Watch motor temperatures, battery, the 512 flag and path error over time.

## 6. Housekeeping

- **Forge request:** write a brief in `EPICRobotzLab/EPIC-Forge` that `robot_disable` fails while a script runs (7 of 7) and works after; Dal agreed it should be written up.
- **bench-lamp-led:** its `NOTES.md` still calls itself `VCHSRobots/elevator`. Pending a Director re-adopting `VCHSRobots/bench-lamp-led` on the Things page so Forge can push the one-line fix.
- Optional: shorten auto-range's 0.85 s, ~16 A creep into the top stop if the stops should see less of it.
- The simulation (`SimElevatorIO`) has no backlash, no jerk and no Talon state effects; it predicted crossing times well (0.42 s vs 0.40 s measured at 200 in/s^2) but cannot reproduce the load-sharing fault.
