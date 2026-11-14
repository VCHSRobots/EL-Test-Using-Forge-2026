# Notes

How this repository is organized and why. Forge reads this file before every change and keeps it current; so should anyone else who changes the code.

## What it is

A WPILib 2026 Java command-based robot program for a roboRIO, team 4415. It builds, simulates and deploys with the standard WPILib tools (VS Code, or `./gradlew` with the WPILib installation) and depends on nothing from Forge. Phoenix 6 (26.3.0) is in `vendordeps/` for the CTRE devices (Kraken X60 motors on TalonFX).

## The elevator stand

`VCHSRobots/EL-Test-Using-Forge-2026`: two Kraken X44 on a chain, one stage, 9.5 in of travel. Started from the `VCHSRobots/elevator` template repository (without its bench-lamp code).

- **Hardware map** (`hardware/KnownUnits`): roboRIO serial `031be686` (seen by the harness on 2026-11-11), right motor CAN 10, left motor CAN 11, PDH CAN 1 (the PDH is not used by code yet). An unknown serial builds no motors.
- **Directions** (stated by the team): the right motor spinning counter-clockwise raises the carriage; the left motor clockwise raises it. So the right is `CounterClockwise_Positive`, the left `Clockwise_Positive`, and positive motor rotation is UP for both (`TalonElevatorIO`).
- **Constants** (`Elevator`): 2.875 motor rotations per shaft turn, 4.0 in of chain per shaft turn (0.71875 motor rotations per inch), travel 9.5 in, zero = carriage on the bottom stop AT POWER-UP (no homing yet; always power up on the bottom stop).
- **Stick control (default command, `followStick`)**: the stick on port 0, axis 1 (forward = up) sets the height: released = 0 in (bottom), full forward = 8 in (`STICK_TOP_INCHES`), deadband 0.08, the setpoint ramped at 2 in/s (`STICK_SPEED_IN_PER_SEC`). No limit switches: while the robot is DISABLED the encoders are re-zeroed every loop (`ElevatorIO.zero`), so every enable starts at 0 and the carriage is taken to be on the bottom stop (the stand's hard stops are harmless). A missing pad reads as released. The single-motor test buttons moved to a second pad on port 1. Tested in simulation only (`StickHeightTest`).
- **Auto-home (`Elevator.autoHome`, stick button 1 on port 0)**: takes over, drives both motors DOWN open loop at 0.6 V (ramped in over 0.3 s; the left follows the right), with the reverse soft limit switched off (the zero it replaces may be wrong). The bottom hard stop is found by a stall: average stator current >= 6 A AND speed (from the position change, filtered) < 0.3 in/s for 0.2 s in a row, never judged in the first 0.5 s. Then neutral, the reverse soft limit back on, encoders zeroed (that position is 0 in), setpoint/target 0, and the command ends so `followStick` resumes. No stall within 8 s: no zero change, warning on the DS, `homeStatus` 3. Disabling cancels it (`homeStatus` 4). Logged: `homing`, `homeStatus` (0 idle, 1 running, 2 found, 3 timeout, 4 interrupted), `homeFoundInches` (where the bottom was on the old zero; also `/Elevator/Status/HomeFoundInches`), `homeStallSeconds`, `homeSpeedInPerSec`, `homeAmps`. The lag fault is not judged while homing. Thresholds are first guesses, untuned on the real stand. Tested in simulation only (`AutoHomeTest`).
- **Control over NetworkTables**: `/Elevator/Control/TargetInches` (absolute, clamped to 0.25..9.25 in) and `/Elevator/Control/SpeedInPerSec` (default 1.0, clamped to 0..2). Status in `/Elevator/Status/PositionInches` and `SetpointInches`. The default command ramps a setpoint toward the target at that speed and both motors run a position loop on it. Only while enabled; on disable the target snaps to the current position, so enabling never jumps.
- **Leader/follower** (default): the right motor runs the position loop and the left is a CTRE `Follower` of it (`MotorAlignmentValue.Opposed`: the motors are mounted mirrored and their inverts differ, so Aligned made them fight; see Status). Added because with two independent loops the motors fought each other (left ~8 A, right ~3 A mean, right torque current to -20 A) although each alone carried the load equally. `driveMode` 3.
- **Single-motor test**: pad 0 buttons choose which motors run the position loop: A = leader/follower (default), Y = both independent loops, B = right only, X = left only. The other motor gets neutral output (brake mode) but is still read and logged, and `driveMode` (0 both, 1 right, 2 left, 3 leader/follower) is in the log. It resets to leader/follower on every disable. Used to check whether the two sides carry the load unevenly.
- **Safety**: motor voltage capped at 3 V, stator and supply limits 20 A, Talon soft limits just inside the travel, and a latched fault (motors neutral until the next disable) if either motor lags the setpoint by more than 1 in for 0.25 s. kP = 6 V per motor rotation, no feedforward: first-test values, untuned on the real stand.
- **Telemetry**: every loop `TalonElevatorIO` refreshes, at 100 Hz, per motor: position, velocity, motor volts, stator/supply/torque current, device and processor temperature, duty cycle, supply (bus) voltage, closed-loop error/reference/output and the fault fields. `Elevator` copies them into `@Logged` fields (`rightSupplyAmps`, `leftTorqueAmps`, ...), with battery volts, brownout, CAN utilisation, loop time and the pad's axis 1, so Epilogue writes them to NT and the WPILOG. Fault fields are bit masks stored as doubles.
- **`ElevatorIO`**: `TalonElevatorIO` is the hardware, `SimElevatorIO` is plain Java physics used in simulation and tests (the CTRE simulator runs in wall-clock time and gave erratic results when the scenario steps faster than real time).

## Layout

- `src/main/java/frc/robot/Main.java` starts the robot program, or the idle program when built with `-Pprogram=idle`.
- `Robot.java` runs the command scheduler and binds Epilogue logging. It holds nothing else.
- `RobotContainer.java` builds the subsystems from this unit's hardware map and binds the controls.
- `IdleRobot.java` is the idle program: it reports its identity and constructs no motor controller, so enabling it moves nothing. It is left on the robot between sessions.
- `forge/Identity.java` starts the WPILOG log (`DataLogManager`), reads the controller's serial, picks the unit, and publishes the identity (below).
- `hardware/` holds the hardware map: `Unit` is what differs between two units running the same code (CAN IDs, inversions, offsets), and `KnownUnits` lists every physical unit by its roboRIO's serial. An unknown serial gives an unknown unit and a warning on the Driver Station.
- `subsystems/` holds one class per subsystem, each annotated `@Logged` so Epilogue logs its fields. `Elevator` is the only subsystem.
- `src/test/java/frc/robot/` (`ElevatorScenarioTest`: up 5 in at 1 in/s and back, clamping, nothing moves disabled) holds the scenario tests: `Scenario` enables the simulated Driver Station, runs the scheduler in 20 ms steps of simulated time, and the test asserts on the subsystems. Nothing here touches a real robot.

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

## Status and next steps (end of the first hardware session, 2026-11-11)

**Known (observed on the stand, build EL18 = `d70517b`):**
- Independent position loops on both motors fought each other (uneven current, right torque current to -20 A), although each motor alone lifts the carriage at about 15 A.
- Leader/follower with the left `Aligned` stalled: both motors at the 20 A limit, no motion, a squeal, the lag fault cut them (run `03b00bef`, build EL16 `d1891af`).
- Leader/follower with the left `Opposed` works: the carriage followed a 30 s jog (up 5 s, hold, down 5 s, hold, up 3 s, down 3 s) to about 5.1 in and back to about 0.2 in, lagging the setpoint by at most 0.35 in. No squeal or strain was reported. The motors shared the load: each averaged about half of one motor's lifting current, left and right within about 1 A, peak 8.2 A (run `f2745d3b`).
- Motion was **not smooth** (the director's observation). In the log the current swings 1 to 2 A (std) and velocity 0.2 to 0.6 rev/s while moving, on both motors together, so it comes from the shared command, not from the motors fighting. At rest the motors hold about +1.5 to +1.8 A. A second run was made for filming; it was not analysed.

**Not known / not done:**
- Why the motion is rough. Candidates (inference only): kP (6 V per rotation) too aggressive, no feedforward, the setpoint ramped in 20 ms steps, chain slack or backlash.
- The 15 in travel in the Thing description versus 9.5 in here: the code uses 9.5 in. Confirm the real travel and the soft limits.
- Homing: zero is wherever the carriage is at power-up (bottom stop). The PDH is not read by code yet.
- Several jogs were only to 5 in; nothing has been run near the top.

**Suggested next steps, in order:**
1. Re-read run logs of the filmed jog and compare it with `f2745d3b` to see if the roughness repeats.
2. Add gravity feedforward (kG, about the 1.5 to 2 A hold current) and kS, then lower kP and add a little kD; change one at a time and rerun the same jog.
3. Try Motion Magic (or a trapezoid profile) on the leader instead of the 20 ms ramp, with a gentle acceleration limit.
4. Check the chain tension and backlash by hand if the ripple stays.
5. Confirm the travel, set soft limits to match, add a homing routine, then raise the 3 V and 20 A caps in steps.
6. Leave the stand on the idle program when done.
