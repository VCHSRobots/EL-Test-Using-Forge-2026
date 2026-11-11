# Notes

How this repository is organized and why. Forge reads this file before every change and keeps it current; so should anyone else who changes the code.

## What it is

A WPILib 2026 Java command-based robot program for a roboRIO, team 4415. It builds, simulates and deploys with the standard WPILib tools (VS Code, or `./gradlew` with the WPILib installation) and depends on nothing from Forge. Phoenix 6 (26.3.0) is in `vendordeps/` for the CTRE devices (Kraken X60 motors on TalonFX).

## The elevator stand

`VCHSRobots/EL-Test-Using-Forge-2026`: two Kraken X44 on a chain, one stage, 9.5 in of travel. Started from the `VCHSRobots/elevator` template repository (without its bench-lamp code).

- **Hardware map** (`hardware/KnownUnits`): roboRIO serial `031be686` (seen by the harness on 2026-11-11), right motor CAN 10, left motor CAN 11, PDH CAN 1 (the PDH is not used by code yet). An unknown serial builds no motors.
- **Directions** (stated by the team): the right motor spinning counter-clockwise raises the carriage; the left motor clockwise raises it. So the right is `CounterClockwise_Positive`, the left `Clockwise_Positive`, and positive motor rotation is UP for both (`TalonElevatorIO`).
- **Constants** (`Elevator`): 2.875 motor rotations per shaft turn, 4.0 in of chain per shaft turn (0.71875 motor rotations per inch), travel 9.5 in, zero = carriage on the bottom stop AT POWER-UP (no homing yet; always power up on the bottom stop).
- **Control over NetworkTables**: `/Elevator/Control/TargetInches` (absolute, clamped to 0.25..9.25 in) and `/Elevator/Control/SpeedInPerSec` (default 1.0, clamped to 0..2). Status in `/Elevator/Status/PositionInches` and `SetpointInches`. The default command ramps a setpoint toward the target at that speed and both motors run a position loop on it. Only while enabled; on disable the target snaps to the current position, so enabling never jumps.
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
