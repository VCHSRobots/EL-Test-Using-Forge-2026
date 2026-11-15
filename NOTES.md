# Notes

How this repository is organized and why. Forge reads this file before every change and keeps it current; so should anyone else who changes the code.

## What it is

A WPILib 2026 Java command-based robot program for a roboRIO, team 4415. It builds, simulates and deploys with the standard WPILib tools (VS Code, or `./gradlew` with the WPILib installation) and depends on nothing from Forge. Phoenix 6 (26.3.0) is in `vendordeps/` for the CTRE devices (Kraken X44 motors on TalonFX).

## The elevator stand

`VCHSRobots/EL-Test-Using-Forge-2026`: Started from the `VCHSRobots/elevator` template repository (now renamed `VCHSRobots/bench-lamp-led`) (without its bench-lamp code).

### The mechanism (facts from the director; keep these here)

- **One stage, 1:1**: the carriage moves exactly as far as the chain. Designed by Micah.
- **Travel: 9.375 in +/- 0.25 in, stop to stop** (measured 15 Nov 2026; an earlier "about 9.5 in" and the old "15 in" are superseded). Zero is the bottom stop.
- **Carriage: 3.959 lb by CAD, without the chains** (an earlier figure was 3.984 lb). Built of 1x2 in aluminium rectangular tube, 1/8 in wall.
- **Drive**: two Kraken X44 drive **one shared shaft**, each through the same gears: 16T pinion -> 24T idler -> 46T on the shaft, so **2.875 motor turns per shaft turn**. Two #25 chains (0.25 in pitch) on 16T sprockets on that shaft, kept in step by it: **4.0 in per shaft turn**, so **0.71875 motor turns per inch** (1.391 in per motor turn). The motors and gears are at the bottom; the side with them is the back.
- **Motors**: seen from the back, right motor CAN 10 on PDH channel 10, left motor CAN 11 on PDH channel 11, 40 A fuses. The right motor counter-clockwise raises the carriage, the left clockwise. PDH CAN 1, powered through a 120 A breaker; roboRIO 1 on PDH channel 22. CAN: roboRIO -> left -> right -> PDH (terminated).
- **Hard stops** at both ends, thick metal: they can take any force the motors can apply, but **repeated banging will wear the mechanism**, so the code commands heights 0.25 in inside them and only touches them while ranging, gently. **No limit switches.**
- **Goal** (15 Nov 2026): full travel in about 0.5 s, holding within about 0.02 in. Final use: drive it with a joystick, and an autonomous "dance" that runs for minutes.

### How the code runs it

- **Units**: both Talons have `SensorToMechanismRatio` = 0.71875, so the Talons, `ElevatorIO` and `Elevator` all work in **inches** (positions, in/s, soft limits, gains).
- **Leader/follower**: the right motor runs every control request; the left is a CTRE `Follower`, `MotorAlignmentValue.Opposed` (mounted mirrored; `Aligned` made them fight, run `03b00bef`). Height is the right (leader) encoder; the left is logged too, with `sideDiffInches`.
- **Moves: Motion Magic** (`MotionMagicVoltage`) on the leader at 1 kHz: trapezoid profile (cruise, acceleration, optional jerk), feedforward kS (velocity sign), kG (`Elevator_Static`), kV, kA, plus kP and kD. Every command asks for a height clamped to [0.25 in, travel - 0.25 in]; the Talons' soft limits sit 0.10 in inside each stop.
- **Live tuning** (`Tuning`, `/Elevator/Tuning/`): kS, kG, kV, kA, kP, kD, CruiseInPerSec, AccelInPerSec2, JerkInPerSec3, PeakVolts, StatorAmps, DanceSeconds, DynamicVolts. Each is clamped to a range (ceilings: 12 V, 40 A, 60 in/s, 600 in/s^2) and written into the Talons **only while disabled** (config writes block), so a change takes effect at the next enable. The values in use are logged (`kS` ... `statorAmps`).
- **Defaults** (`Elevator.DEFAULT_GAINS`, predicted, NOT measured): kS 0.10 V, kG 0.15 V, kV 0.069 V/(in/s), kA 0.0006 V/(in/s^2), kP 4 V/in, kD 0.05 V/(in/s), cruise 10 in/s, acceleration 60 in/s^2; caps 3 V and 20 A. kG and kV come from the Kraken X44 data and the carriage weight; kS from run EL21 (rough). With these a full move takes about 1.1 s in simulation (today's code: 4 s).
- **Auto-range** (`autoRange`): runs by itself at the first teleop enable after the program starts (and in autonomous, before the dance), again on stick button 1, or as the routine `range`. (1) Creep down at 0.6 V until a stall (speed < 0.3 in/s and >= 6 A for 0.2 s); that is 0 in (`bottomFoundInches` = where it was on the old zero). (2) Motion Magic up to 1.25 in below the nominal top. (3) Creep up at 0.5 V until a stall (>= 8 A); that height is the travel if it is within 0.75 in of 9.375 in, else it fails and keeps the nominal travel (`rangeStatus` FAILED, a DS warning, no automatic retry). The soft limit in the searched direction is off while it searches. Then soft limits go just inside both stops, the stick and the dance span the measured range, and it holds just below the top. Disabling interrupts it (it runs again at the next teleop enable). The encoder zero stays valid while the Talons stay powered, so **nothing re-zeroes while disabled any more**.
- **Teleop**: stick port 0, axis 1 (forward = up): released = bottom of the range (0.25 in), full forward = top of the range. Button 2 toggles the dance.
- **Autonomous**: the dance (`Dance.BEATS`: bounces, a staircase, a shimmy, half bounces; heights as fractions of the measured range; ~20 s a pass) looped for `DanceSeconds` (default 180 s), then to the bottom of the range. It never asks for a height outside the range, so it never touches a stop. Its speed is the Motion Magic tuning.
- **Routines for scripts**: write a name to `/Elevator/Control/Routine` while enabled: `range`, `moves` (1 -> 8 -> 4.5 -> 1 in, 1.5 s each), `quasistatic` (volts ramp 0.25 V/s up, then down, 1.25 in from the stops; gives kS, kG, kV), `dynamic` (a step of DynamicVolts up, then down; gives kA), `dance`. It runs when the name changes; write "" to run the same one again. `/Elevator/Status/Routine` is the running command, `RoutinesDone` counts finished ones, plus `PositionInches`, `TargetInches`, `TravelInches`.
- **Safety**: `/Forge/Abort` true = neutral output for as long as it is true (`aborted`). A lag fault latches (neutral until disabled) if the height is more than 1 in from the Motion Magic reference for 0.25 s (judged only while profiling). Nothing moves while disabled; on disable the target becomes the current height.
- **Telemetry**: as before, per motor at 100 Hz (position, velocity, volts, stator/supply/torque current, temperatures, duty, bus volts, closed-loop error/reference/output, faults), plus `mode`, `targetInches`, `profileInches`, `profileVelocityInPerSec`, `profileErrorInches`, `openLoopVolts`, `rangeStatus`, `travelInches`, `danceBeat`, `moveIndex`.
- **Simulation** (`SimElevatorIO`): plain Java, deterministic, 1 kHz. It copies the Talon (Motion Magic trapezoid, the same feedforward and PD, soft limits, peak volts, a current limit, brake in neutral) and models two Kraken X44 (WPILib `DCMotor`) on the shaft, with gravity on the 3.959 lb carriage, 0.28 kg of chain (estimate), rotor inertia 1.5e-5 kg m^2 per motor (ESTIMATE), Coulomb friction worth 0.12 V (from EL21) and inelastic hard stops at a settable true travel. Replace the estimates with the characterization's numbers.
- Removed in this version: the 20 ms setpoint ramp, the NetworkTables `TargetInches`/`SpeedInPerSec` control, zero-while-disabled, and the single-motor test pad (port 1).

## Layout

- `src/main/java/frc/robot/Main.java` starts the robot program, or the idle program when built with `-Pprogram=idle`.
- `Robot.java` runs the command scheduler and binds Epilogue logging. It holds nothing else.
- `RobotContainer.java` builds the subsystems from this unit's hardware map and binds the controls.
- `IdleRobot.java` is the idle program: it reports its identity and constructs no motor controller, so enabling it moves nothing. It is left on the robot between sessions.
- `forge/Identity.java` starts the WPILOG log (`DataLogManager`), reads the controller's serial, picks the unit, and publishes the identity (below).
- `hardware/` holds the hardware map: `Unit` is what differs between two units running the same code (CAN IDs, inversions, offsets), and `KnownUnits` lists every physical unit by its roboRIO's serial. An unknown serial gives an unknown unit and a warning on the Driver Station.
- `subsystems/` holds one class per subsystem, each annotated `@Logged` so Epilogue logs its fields. `Elevator` is the only subsystem. `ElevatorView` publishes what AdvantageScope shows under `/Forge/Elevator/`: `Mechanism` (a Mechanism2d), `Components` (the carriage as a robot-relative Pose3d array) and `RobotPose` (always the origin; put it on the Robot row and `Components` on a Component row under it). Display only.
- `assets/Robot_Forge/` is the stand's 3D model for AdvantageScope (`model.glb` the frame, `model_0.glb` the carriage, `config.json`); `assets/README.md` says how to make and show it. The carriage's place along the rails is still off: change `zeroedPosition` in `config.json` and `CARRIAGE_ORIGIN_AT_BOTTOM` in `ElevatorView` together.
- `docs/sim-and-forge-lessons.md`: lessons from the first simulation sessions (simulator joysticks, the station harness, the 3D model). Read it before simulating or changing the model.
- Repositories: all elevator-stand code lives here. `VCHSRobots/bench-lamp-led` (named `VCHSRobots/elevator` until 15 Nov 2026) is a different, electronics-only bench device (lamp and LED); the model and lessons above were moved here from it on 15 Nov 2026.
- `src/test/java/frc/robot/` holds the scenario tests (`ElevatorScenarioTest`: auto-range from a wrong zero, a rejected travel, ranging on the first teleop enable, goal and default tuning moves with `Metrics`, Abort, the lag fault, the dance, routines by name, characterization staying off the stops, nothing moves disabled; `SimTraceTest` writes `build/sim-trace.csv`): `Scenario` enables the simulated Driver Station, runs the scheduler in 20 ms steps of simulated time, and the test asserts on the subsystems. Nothing here touches a real robot.

## Build identity

The Gradle task `writeBuildConstants` writes `build/generated/sources/forge/java/frc/robot/BuildConstants.java` before every compile: repository, branch, commit, whether the tree was dirty, build time, builder (`FORGE_BUILDER`, else the user name) and program (`robot` or `idle`). At startup the program publishes them to NetworkTables (NT) under `/Forge/Build/` (`Repository`, `Branch`, `Commit`, `Dirty`, `BuiltAt`, `Builder`, `Program`), with `/Forge/Serial` and `/Forge/Unit`, and writes the same as one JSON entry, `/Forge/Identity`, into the WPILOG file. The Forge harness reads these to say which build ran in each run. Built outside a Git checkout, the Git fields are empty.

## Commands

- `./gradlew build` compiles and runs the scenario tests. `./gradlew test` runs only the tests.
- `./gradlew simulateJava` runs the program in simulation with the GUI; add `-Pheadless` for none.
- `./gradlew deploy` deploys to the roboRIO at 10.44.15.2 (or USB). Add `-Pprogram=idle` to build or deploy the idle program.
- After one online build, `--offline` works, as at competition.

## Conventions

- Each subsystem keeps its state in `@Logged` fields and offers commands as factory methods; constants a unit can differ in go in `Unit`, never in the subsystem.
- Every subsystem gets a simulation model in `simulationPeriodic` and at least one scenario test before it runs on hardware.
- Signals are logged through Epilogue and `DataLogManager`, so every run yields comparable WPILOG files.

## The WPILib year

This is the 2026 season: GradleRIO 2026.2.1 (`build.gradle`), `frcYear = '2026'` (`settings.gradle`, which also names the install folder `~/wpilib/2026` or `C:\Users\Public\wpilib\2026`), `projectYear` in `.wpilib/wpilib_preferences.json`, and the `frcYear` in both vendordeps. The roboRIO image, the Driver Station and WPILib must be the same season. To move to 2027:

1. Install that year's WPILib and flash the roboRIO. Set `WPILIB_HOME` for the harness, or change `default_wpilib_home` in `harness/src/forge_harness/config.py`.
2. Set the GradleRIO version, `frcYear` and `projectYear` to the new year. Read the version from the Gradle plugin portal (`edu.wpi.first.GradleRIO`).
3. Replace the vendordeps: `WPILibNewCommands.json` from the `allwpilib` tag of that release, and CTRE's `Phoenix6-frc<year>-latest.json` from `https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/`. Delete the old Phoenix file. GradleRIO refuses a vendordep whose year is wrong, which is how you find any you missed.
4. Run `./gradlew build`, then `--offline`. Fix what the new WPILib or Epilogue changed.

## Status and next steps (15 Nov 2026)

**Observed on the stand before this version** (EL18-EL22): leader/follower Opposed works; P-only moves at 2 in/s used ~0.41 V up, ~-0.16 V down, ~0.22 V holding; holding error 0.05 in mean, 0.12 in worst; the right encoder reads more than the left by ~0.06 in per inch of height (0.46 in at 7.5 in, back to ~0 at the bottom; cause unknown, both motors are on one shaft); the one auto-home (EL22) found the bottom 0.97 in below the disabled zero.

**Simulation only (this version, not yet on the stand)**: default tuning moves 0.25 -> 9.1 in in 1.06 s, peak 10 in/s; goal tuning (40 in/s, 250 in/s^2, 12 V, 40 A) settles within 0.02 in in 0.40 s, peak ~3.1 V and ~10 A a motor, no overshoot. The simulation has no chain slack or backlash, so the real stand will do worse.

**Next, on the stand** (each with the person's enable, scripts with limits):
1. Auto-range 3-5 times: bottom and top repeat within 0.02 in? Does the measured travel match 9.375 in (checks the 4.0 in per shaft turn)?
2. `quasistatic` and `dynamic`: fit kS, kG, kV, kA; put them in the defaults and in `SimElevatorIO`.
3. Sweep kP/kD on `moves`; then raise cruise/acceleration, then caps (3 -> 6 -> 12 V, 20 -> 40 A), only after clean steps.
4. Find out the left/right encoder difference (a tilt, a slipping set screw, a chain skipping?).
5. Run the dance for minutes; watch temperatures.
