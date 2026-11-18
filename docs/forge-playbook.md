# Forge playbook for tuning a mechanism on a stand

> 18 Nov 2026: results and the final parameters are in `findings-2026-11-18.md`. The older `scripts end waiting for disable` rule below is replaced (section 3).

Skills and methods worked out while tuning this elevator stand (15-16 Nov 2026), written so a later Forge conversation, or a person, can reuse them on this stand or another mechanism. The results themselves are in `findings-2026-11-16.md`; the plan in `next-steps.md`.

## 1. Working with the director

- Dal lets Forge drive development: propose the next test without asking whether he is done with the robot, and do not post idle-program reminders.
- Optimise **speed and accuracy**, not sound; bad sounds are a debugging clue.
- Give observed and inferred separately, correct earlier mistakes openly (several were caught that way: kV, the 512 bit, "more drag"), and give numbers from the robot log.
- Answer the question asked first, then the detail. Explain the physics plainly when asked (e.g. "where do the 224 J come from").

## 2. The test loop

1. Change code on a branch; add a scenario test; `./gradlew build` here (and on the station via `git_clone_or_pull` + `gradle`); PR, merge, note on the thing.
2. One card per test: `propose_plan` (deploy, wait_connected, run_script) when a new build is needed, `run_script` when not.
3. The person enables in **Teleop** (Autonomous starts the dance). Auto-range runs by itself on the first Teleop enable after a deploy.
4. `plan_results`/`script_results`, then **wait for the robot WPILOG** on the run (`run_logs` a few minutes after the disable) and score from it.
5. Report, update the summary, `thing_note` for code or tuning changes.

## 3. Writing stand scripts (`tools/forge-scripts/isolation_test.py` is the template)

- First line of work: `wait_until(PositionInches is not None, 60)`. After a deploy the topic does not exist yet, and `limits` on a missing topic refuses the script.
- `limits` on `PositionInches` (-2.5 .. 10.3) and `ProfileErrorInches` (+/-0.5). A trip sets `/Forge/Abort`; the robot code outputs zero.
- `on_exit`: write `park` and the defaults. Defaults reach the Talons only when parked or disabled.
- Write tuning (`/Elevator/Tuning/...`) **before** `wait_enabled`, or while parked between sets. Gains in use show as `kPInUse`/`kDInUse`.
- Run routines by name: write `""`, sleep 0.3 s, write the name (`/Elevator/Control/Routine`). Wait on `/Elevator/Status/Parked` after `park`.
- `mark` each set, `record` every sample with `server_time`.
- **End (rule since 16 Nov, replaces the old one):** park, restore defaults in `on_exit`, log DONE and **finish**. Never wait for the person's disable at the end (`wait_disabled` as the last call is refused); the station's frame tells them to disable. On a station whose Driver Station runs as administrator Forge cannot disable at all, so the person does it.
- **Sweeps (18 Nov):** use `tools/forge-scripts/sweep_one_enable.py`: one enable, auto-range first, then per set: park, write the value, wait for the Talon's read-back (`/Elevator/Status/Talon...`, `ConfigOk`), `ResetPeaks`, run, read the robot-side peaks. Every row of a recorded CSV must have the same columns.
- Sweeps in one enable work because tuning is written while parked; one enable per set is not needed.

## 4. Reading the robot's log

- `run_logs` -> `log_signals` -> `log_read` with the signals you need; it writes a CSV in the working directory. One WPILOG covers the whole program lifetime since the deploy, so several runs share a file; cut by time (marks, `DS:enabled`, tuning values changing).
- `log_read` at 5000 rows over a long log gives ~20 ms rows; read a short span for fine timing. The logged signals are ~50-100 Hz anyway.
- Signals live under `NT:Robot/container/elevator/...` (Epilogue) and `NT:/Elevator/Status/...`; match by the end of the name.
- `tools/analysis/score_crossings.py run.csv` scores full-travel crossings: time to stay within 0.05 in, worst path error, overshoot, peak currents, grouped by (cruise, accel, jerk). `tools/analysis/wpilog.py` reads a raw .wpilog when needed.

## 5. Measurement recipes that worked

- **Characterization:** quasistatic ramps 0.25 V/s up and down, least squares `V = kS sign(v) + kG + kV v` (fit each direction too as a check); kA from 1 V steps with the others fixed. Check the fit by predicting the step speeds.
- **Path error:** use the Talon's own closed-loop error (one instant), never position minus reference from separate frames (~5 ms skew = 0.12 in at 25 in/s).
- **Motors opposed:** both torque currents above **1 A** with opposite signs (idle offsets are 0.2-0.6 A).
- **Chatter / shaking:** count reversals of the closed-loop output with swings above 1 V per gain set (`rev` in the earlier tools). kD 0.4 gave 56; good sets 0-1.
- **Roughness:** high-pass the velocity error (velocity minus planned, minus a 5-sample moving average) and take its standard deviation per move; compare the **same move at the same settings** across runs.
- **Is the error from kV or kA?** Regress error against planned velocity and against planned acceleration over all moving samples. Error tracking speed points to kV or drag; tracking acceleration points to kA.
- **Apparent back-EMF constant per motor:** at steady cruise, `(volts - I_torque x R) / rotor speed`; compare to the data sheet (Kraken X44 data sheet ~0.0148 V per rad/s of rotor speed). A wandering constant with steady speed means an electrical change, not mechanics.
- **Energy:** battery energy = sum of `batteryVolts x (leftSupplyAmps + rightSupplyAmps) x dt` (measured). Waste from uneven sharing = sum of `R (I_L^2 + I_R^2)` minus `R ((I_L + I_R)^2 / 2)` (needs an assumed winding resistance; give the range).
- **Encoder agreement:** derive each motor's speed from its own position (latency-compensated, same instant), not from the separately sampled velocity signals.

## 6. Physics of this stand worth remembering

- Two motors on one shaft: one closed loop plus a `Follower` (Opposed). Two loops fight through backlash.
- Different pinions (15T/16T) are handled by each Talon's `SensorToMechanismRatio`; with a follower both get the same volts.
- kG ~0.16 V and kS ~0.12 V: going up the feedforward starts at ~0.28 V, going down ~0.04 V; the carriage barely holds itself with no power.
- Crossing time on 8.94 in: at 200 in/s^2 acceleration limits it (0.40 s); at 400 in/s^2 the 45 in/s cruise limits it (0.29 s).
- Current limit too low (20 A) causes stumbles at peak acceleration; jerk limiting helps only by keeping current down.
- Fault bit 512 in the Talon fault field is `StatorCurrLimit`; log faults by name (`rightFaults`, `leftStickyFaults`, ...).

## 7. Station and Forge quirks

- `robot_disable` fails while a script runs; the person disables. Faults rely on `/Forge/Abort`.
- Cards expire after ten minutes; a message from anyone stops a running plan.
- The robot WPILOG reaches the run a few minutes after the disable; the station's own `nt.wpilog` and `samples.csv` come at once but at ~10 Hz and only on change.
- The simulation predicts crossing times well but has no backlash, jerk or Talon state; it cannot reproduce load-sharing faults.
- Station shell is `cmd`; see `sim-and-forge-lessons.md` for the simulator and 3D model.
