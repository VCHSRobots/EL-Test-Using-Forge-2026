package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import java.util.function.Supplier;

/**
 * The two-motor elevator stand. Heights are inches above the bottom hard stop, found by
 * auto-range (bottom, then top) on the first enable after the program starts.
 *
 * <p>Moves are Motion Magic profiles run inside the right Talon at 1 kHz, with gravity (kG),
 * friction (kS), velocity (kV) and acceleration (kA) feedforward and a PD loop; the left Talon
 * follows it (the same volts). Gains, profile limits and output caps are live-tunable under /Elevator/Tuning/ (applied
 * while disabled, or while parked with the motors off). Commanded heights always stay {@link #STOP_MARGIN_INCHES} inside the measured
 * travel, and the Talons' soft limits sit just inside the stops.
 *
 * <p>Safety: /Forge/Abort true means zero output until it is cleared; a lag of more than
 * {@link #FAULT_ERROR_INCHES} behind the profile for {@link #FAULT_SECONDS} latches a fault (zero
 * output until the next disable). Nothing moves while disabled.
 */
@Logged
public class Elevator extends SubsystemBase {
  // --- The stand (director, 11 and 15 Nov 2026); the gearing is in hardware.Unit ---
  /** Measured stop to stop, +/- 0.25 in. Auto-range measures the real one. */
  public static final double NOMINAL_TRAVEL_INCHES = 9.375;
  /** Auto-range rejects a measured travel further than this from nominal. */
  public static final double TRAVEL_TOLERANCE_INCHES = 0.75;
  /** Commanded heights stay this far inside the stops. */
  public static final double STOP_MARGIN_INCHES = 0.25;
  /** The Talons' soft limits sit this far inside the stops. */
  public static final double SOFT_LIMIT_MARGIN_INCHES = 0.10;

  // --- Defaults: kS, kG, kV, kA measured on EL29 (run cf3877f1, see NOTES.md) ---
  // All four are for the PAIR of motors at the same volts, as they were measured and as the
  // follower runs them (the right Talon leads, the left copies its volts).
  // kP 8 V/in and kD 0.10 V/(in/s) on the pair: not tuned yet (the next step is a sweep).
  public static final ElevatorIO.Gains DEFAULT_GAINS =
      new ElevatorIO.Gains(0.12, 0.16, 0.0643, 0.0010, 4.0, 0.0, 90.0, 700.0, 0.0);
  public static final double DEFAULT_PEAK_VOLTS = 12.0;
  public static final double DEFAULT_STATOR_AMPS = 40.0;
  /** Hard ceilings the tuning values can never exceed. */
  public static final double MAX_PEAK_VOLTS = 12.0;
  public static final double MAX_STATOR_AMPS = 40.0;
  public static final double MAX_CRUISE_IN_PER_SEC = 90.0;
  public static final double MAX_ACCEL_IN_PER_SEC2 = 900.0;

  // --- Stick ---
  public static final int STICK_PORT = 0;
  public static final int STICK_AXIS = 1;
  public static final double STICK_DEADBAND = 0.08;

  // --- Safety ---
  public static final double FAULT_ERROR_INCHES = 1.0;
  public static final double FAULT_SECONDS = 0.25;

  // --- Auto-range (find both hard stops) ---
  /** Open-loop volts DOWN to find the bottom (EL22: 0.6 V found it in 0.8 s). */
  public static final double RANGE_DOWN_VOLTS = 0.6;
  /** Open-loop volts UP for the last creep into the top stop (about 3 in/s by EL21). */
  public static final double RANGE_UP_VOLTS = 0.5;
  public static final double RANGE_RAMP_SECONDS = 0.2;
  /** The fast part of the climb stops this far below the nominal top, then it creeps. */
  public static final double RANGE_CREEP_START_BELOW_TOP_INCHES = 1.25;
  public static final double RANGE_GRACE_SECONDS = 0.4;
  /** Stalled = speed below this (in/s) ... */
  public static final double RANGE_STALL_SPEED_IN_PER_SEC = 0.3;
  /** ... with average stator current at least this going down (gravity helps) ... */
  public static final double RANGE_DOWN_STALL_AMPS = 6.0;
  /** ... or this going up (holding the carriage alone takes ~4 A a motor) ... */
  public static final double RANGE_UP_STALL_AMPS = 8.0;
  /** ... for this long in a row. */
  public static final double RANGE_STALL_SECONDS = 0.2;
  public static final double RANGE_PHASE_TIMEOUT_SECONDS = 6.0;

  // --- Test routines ---
  /** The standard move sequence (inches) and how long each target is held. */
  public static final double[] MOVES_INCHES = {1.0, 8.0, 4.5, 1.0};
  public static final double MOVES_HOLD_SECONDS = 1.5;
  /**
   * The full-travel moves: bottom -> top -> bottom -> top -> bottom of the range (0.25 in inside
   * each stop, so about 8.9 in each way), each target held {@link #FULL_TRAVEL_HOLD_SECONDS}.
   */
  public static final int FULL_TRAVEL_TARGETS = 5;
  public static final double FULL_TRAVEL_HOLD_SECONDS = 1.5;
  /** Quasistatic ramp rate and ceiling. */
  public static final double QUASI_VOLTS_PER_SEC = 0.25;
  public static final double QUASI_MAX_VOLTS = 2.0;
  /** Open-loop ramp per start (one motor alone): faster than quasistatic, higher top. */
  public static final double RAMP_VOLTS_PER_SEC = 8.0;
  /** The ramps also end at this speed (in/s), before the carriage runs out of travel. */
  public static final double RAMP_STOP_IN_PER_SEC = 32.0;
  /** The ramps end this far from either stop (more than CHAR_GUARD_INCHES: they brake from a higher speed). */
  public static final double RAMP_GUARD_INCHES = 2.0;
  public static final double RAMP_MAX_VOLTS = 5.0;
  /** Open-loop phases end this far from either stop, then brake. */
  public static final double CHAR_GUARD_INCHES = 1.25;
  /** Park: the move to the bottom of the range gives up waiting to settle after this long. */
  public static final double PARK_SETTLE_SECONDS = 3.0;

  public enum RangeStatus { NEVER, RUNNING, DONE, FAILED, INTERRUPTED }

  private final ElevatorIO io;
  private final Tuning tuning = new Tuning("Elevator/Tuning");
  private final BooleanSubscriber abortSub;
  private final StringSubscriber routineSub;
  private final StringPublisher routinePub;
  private final IntegerPublisher routineCountPub;
  private final DoublePublisher positionPub;
  private final DoublePublisher targetPub;
  private final DoublePublisher travelPub;
  private final BooleanPublisher parkedPub;
  private final DoublePublisher profileErrorPub;
  private final DoublePublisher kPInUsePub;
  private final DoublePublisher kDInUsePub;
  private final ElevatorView view = new ElevatorView(NOMINAL_TRAVEL_INCHES);
  private final Map<String, Supplier<Command>> routines;
  private String lastRoutineRequest = "";
  private long routinesDone;
  private boolean tuningDirty = true;

  // --- Logged state ---
  String mode = "disabled";
  double targetInches;
  double positionInches;
  double rightInches;
  double leftInches;
  double sideDiffInches;
  /** Right minus left for the same instant (latency-compensated): backlash shows at reversals. */
  double alignedSideDiffInches;
  /** Which motor leads: RIGHT_LEADS normally; the others only in isolation routines. */
  String drive = "RIGHT_LEADS";
  /** Active and sticky faults by CTRE's names, comma-separated. */
  String rightFaults = "";
  String leftFaults = "";
  String rightStickyFaults = "";
  String leftStickyFaults = "";
  double velocityInPerSec;
  double profileInches;
  double profileVelocityInPerSec;
  double profileErrorInches;
  double openLoopVolts;
  boolean closedLoop;
  boolean fault;
  boolean aborted;
  boolean ranged;
  boolean parked;
  boolean zeroKnown;
  RangeStatus rangeStatus = RangeStatus.NEVER;
  double travelInches = NOMINAL_TRAVEL_INCHES;
  double bottomFoundInches;
  double topFoundInches;
  double rangeStallSeconds;
  double rangeAmps;
  double danceBeat = -1;
  double moveIndex = -1;
  double rightVolts;
  double leftVolts;
  double rightStatorAmps;
  double leftStatorAmps;
  double rightVelocityRps;
  double rightSupplyAmps;
  double rightTorqueAmps;
  double rightDeviceTempC;
  double rightProcessorTempC;
  double rightDuty;
  double rightSupplyVolts;
  double rightClosedLoopError;
  double rightClosedLoopReference;
  double rightClosedLoopOutput;
  double rightFaultField;
  double rightStickyFaultField;
  double leftVelocityRps;
  double leftSupplyAmps;
  double leftTorqueAmps;
  double leftDeviceTempC;
  double leftProcessorTempC;
  double leftDuty;
  double leftSupplyVolts;
  double leftFaultField;
  double leftStickyFaultField;
  // The leader's own loop breakdown (Phoenix sign: error = reference - position), volts and
  // inches, with each reading's age in ms when this loop read it.
  double loopP;
  double loopD;
  double loopFF;
  double loopOutput;
  double loopError;
  double loopReference;
  double loopReferenceSlope;
  double loopPosition;
  double loopMotorVolts;
  double loopFollowerVolts;
  double ageLoopPMs;
  double ageLoopFFMs;
  double ageLoopOutputMs;
  double ageLoopErrorMs;
  double ageLoopReferenceMs;
  double ageLoopReferenceSlopeMs;
  double ageLoopPositionMs;
  double ageLoopMotorVoltsMs;
  double ageLoopFollowerVoltsMs;
  double batteryVolts;
  double loopMs;
  double canUtilization;
  boolean browningOut;
  double joystickAxis;
  double kS, kG, kV, kA, kP, kD, cruiseInPerSec, accelInPerSec2, jerkInPerSec3, peakVolts, statorAmps;

  private double faultTimer;
  private double lastPeriodic = Timer.getFPGATimestamp();

  public Elevator(ElevatorIO io) {
    this.io = io;
    ElevatorIO.Gains g = DEFAULT_GAINS;
    tuning
        .add("kS", g.kS(), 0, 1)
        .add("kG", g.kG(), 0, 2)
        .add("kV", g.kV(), 0, 0.5)
        .add("kA", g.kA(), 0, 0.05)
        .add("kP", g.kP(), 0, 40)
        .add("kD", g.kD(), 0, 2)
        .add("CruiseInPerSec", g.cruiseInPerSec(), 0.5, MAX_CRUISE_IN_PER_SEC)
        .add("AccelInPerSec2", g.accelInPerSec2(), 1, MAX_ACCEL_IN_PER_SEC2)
        .add("JerkInPerSec3", g.jerkInPerSec3(), 0, 100000)
        .add("PeakVolts", DEFAULT_PEAK_VOLTS, 1, MAX_PEAK_VOLTS)
        .add("StatorAmps", DEFAULT_STATOR_AMPS, 5, MAX_STATOR_AMPS)
        .add("DanceSeconds", 180, 5, 1800)
        .add("DynamicVolts", 1.0, 0.3, 3.0);

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    abortSub = nt.getBooleanTopic("/Forge/Abort").subscribe(false);
    NetworkTable control = nt.getTable("Elevator/Control");
    routineSub = control.getStringTopic("Routine").subscribe("");
    control.getStringTopic("Routine").publish().set("");
    NetworkTable status = nt.getTable("Elevator/Status");
    routinePub = status.getStringTopic("Routine").publish();
    routineCountPub = status.getIntegerTopic("RoutinesDone").publish();
    positionPub = status.getDoubleTopic("PositionInches").publish();
    targetPub = status.getDoubleTopic("TargetInches").publish();
    travelPub = status.getDoubleTopic("TravelInches").publish();
    parkedPub = status.getBooleanTopic("Parked").publish();
    profileErrorPub = status.getDoubleTopic("ProfileErrorInches").publish();
    kPInUsePub = status.getDoubleTopic("kPInUse").publish();
    kDInUsePub = status.getDoubleTopic("kDInUse").publish();
    parkedPub.set(false);
    routinePub.set("");
    routines =
        Map.ofEntries(
            Map.entry("range", (Supplier<Command>) this::autoRange),
            Map.entry("moves", (Supplier<Command>) this::standardMoves),
            Map.entry("fulltravel", (Supplier<Command>) this::fullTravelMoves),
            Map.entry("soloright", (Supplier<Command>) () -> fullTravelWith(ElevatorIO.Drive.RIGHT_ALONE, "ElevatorSoloRight")),
            Map.entry("sololeft", (Supplier<Command>) () -> fullTravelWith(ElevatorIO.Drive.LEFT_ALONE, "ElevatorSoloLeft")),
            Map.entry("leftleads", (Supplier<Command>) () -> fullTravelWith(ElevatorIO.Drive.LEFT_LEADS, "ElevatorLeftLeads")),
            // Trigger tests (right or left alone): same requests, configs written or not.
            Map.entry("fulltravel_noconfig", (Supplier<Command>) () -> triggerTest(ElevatorIO.Drive.RIGHT_ALONE, false, "ElevatorNoConfigRight")),
            Map.entry("fulltravel_reconfig", (Supplier<Command>) () -> triggerTest(ElevatorIO.Drive.RIGHT_ALONE, true, "ElevatorReconfigRight")),
            Map.entry("fulltravel_noconfig_left", (Supplier<Command>) () -> triggerTest(ElevatorIO.Drive.LEFT_ALONE, false, "ElevatorNoConfigLeft")),
            Map.entry("fulltravel_reconfig_left", (Supplier<Command>) () -> triggerTest(ElevatorIO.Drive.LEFT_ALONE, true, "ElevatorReconfigLeft")),
            // Open-loop ramp per start (1c): one motor alone, VoltageOut ramp up then down, no Motion Magic in the ramps.
            Map.entry("rampsolo", (Supplier<Command>) () -> rampTest(ElevatorIO.Drive.RIGHT_ALONE, "ElevatorRampRight")),
            Map.entry("rampsolo_left", (Supplier<Command>) () -> rampTest(ElevatorIO.Drive.LEFT_ALONE, "ElevatorRampLeft")),
            Map.entry("quasistatic", (Supplier<Command>) this::quasistatic),
            Map.entry("dynamic", (Supplier<Command>) this::dynamic),
            Map.entry("dance", (Supplier<Command>) this::dance),
            Map.entry("park", (Supplier<Command>) this::park));
  }

  // ---------------------------------------------------------------- queries

  public double positionInches() { return positionInches; }
  public double targetInches() { return targetInches; }
  public double travelInches() { return travelInches; }
  public boolean isRanged() { return ranged; }
  public RangeStatus rangeStatus() { return rangeStatus; }
  public double bottomFoundInches() { return bottomFoundInches; }
  public boolean faulted() { return fault; }
  public boolean isParked() { return parked; }

  /** kP and kD in use (the last tuning written to the Talons). */
  public double kPInUse() { return kP; }

  public double kDInUse() { return kD; }
  public boolean aborted() { return aborted; }
  public String mode() { return mode; }
  public Tuning tuning() { return tuning; }

  /** Auto-range should run at the next teleop enable: never ranged, and never failed. */
  public boolean needsRange() {
    return !ranged && rangeStatus != RangeStatus.FAILED && rangeStatus != RangeStatus.RUNNING;
  }

  /** The lowest height a command may ask for. */
  public double lowInches() {
    return STOP_MARGIN_INCHES;
  }

  /** The highest height a command may ask for (measured travel minus the margin). */
  public double highInches() {
    return travelInches - STOP_MARGIN_INCHES;
  }

  public boolean atTarget() {
    return Math.abs(positionInches - targetInches) < 0.05 && Math.abs(velocityInPerSec) < 0.5;
  }

  // ---------------------------------------------------------------- outputs (all go through here)

  /** Profiled move to a height, clamped to the usable range. */
  private void moveTo(double inches) {
    targetInches = MathUtil.clamp(inches, lowInches(), highInches());
    closedLoop = true;
    openLoopVolts = 0;
    if (blocked()) {
      io.neutral();
    } else {
      io.moveTo(targetInches);
    }
  }

  /** Open-loop volts (up positive), for auto-range and characterization only. */
  private void volts(double v) {
    closedLoop = false;
    openLoopVolts = v;
    if (blocked()) {
      io.neutral();
    } else {
      io.setVolts(v);
    }
  }

  private void stop() {
    closedLoop = false;
    openLoopVolts = 0;
    io.neutral();
  }

  private boolean blocked() {
    return fault || aborted || !DriverStation.isEnabled();
  }

  // ---------------------------------------------------------------- driver commands

  /**
   * The stick on port 0, axis 1 (forward = up) sets the height across the whole usable range:
   * released = the bottom of the range, full forward = the top. Motion Magic sets the speed.
   */
  public Command followStick() {
    return run(() -> {
          mode = "stick";
          double a = stickValue();
          a = Math.abs(a) < STICK_DEADBAND ? 0.0 : a;
          double f = MathUtil.clamp(-a, 0.0, 1.0);
          moveTo(lowInches() + f * (highInches() - lowInches()));
        })
        .withName("ElevatorFollowStick");
  }

  /** Profiled move to a fixed height, held until interrupted (tests and scripts). */
  public Command goTo(double inches) {
    return run(() -> {
          mode = "goto";
          moveTo(inches);
        })
        .withName("ElevatorGoTo");
  }

  /** Holds the current target (or the current height, the first time). */
  public Command hold() {
    return startRun(() -> targetInches = positionInches, () -> {
          mode = "hold";
          moveTo(targetInches);
        })
        .withName("ElevatorHold");
  }

  /**
   * The dance: plays {@link Dance#BEATS} in a loop for DanceSeconds (Tuning), then goes to the
   * bottom of the range. Waits for a range first if there is none.
   */
  public Command dance() {
    Timer t = new Timer();
    Command play =
        new FunctionalCommand(
            t::restart,
            () -> {
              mode = "dance";
              int beat = Dance.beatAt(t.get());
              danceBeat = beat;
              double f = Dance.BEATS.get(beat).fraction();
              moveTo(lowInches() + f * (highInches() - lowInches()));
            },
            interrupted -> {
              danceBeat = -1;
              moveTo(lowInches());
            },
            () -> t.get() >= tuning.get("DanceSeconds"),
            this);
    return rangeIfNeeded().andThen(play).withName("ElevatorDance");
  }

  /**
   * The end of a test: a profiled move to the bottom of the range (0.25 in above the stop), then
   * the motors off (neutral, brake mode) and /Elevator/Status/Parked true, with a message on the
   * Driver Station that it is safe to disable. It stays parked, ignoring the stick, until another
   * routine or a stick button takes over or the robot is disabled. Scripts end with it, so the
   * end of every test is the same and nothing moves after it.
   */
  public Command park() {
    Command settle =
        run(() -> {
              mode = "park-move";
              moveTo(lowInches());
            })
            .until(this::atTarget)
            .withTimeout(PARK_SETTLE_SECONDS);
    Command rest =
        startRun(
                () -> {
                  io.setDrive(ElevatorIO.Drive.RIGHT_LEADS); // motors off now: back to normal
                  parked = true;
                  parkedPub.set(true);
                  DriverStation.reportWarning(
                      "Elevator test done: parked at the bottom, motors off. Safe to disable.", false);
                },
                () -> {
                  mode = "parked";
                  stop();
                })
            .finallyDo(() -> {
              parked = false;
              parkedPub.set(false);
            });
    return settle.andThen(rest).withName("ElevatorPark");
  }

  /** Runs auto-range first if the stops have not been found yet. */
  public Command rangeIfNeeded() {
    return Commands.either(Commands.none(), autoRange(), () -> ranged);
  }

  // ---------------------------------------------------------------- auto-range

  private enum RangePhase { DOWN, CLIMB, CREEP, DONE, FAILED }

  /**
   * Auto-range: finds the bottom stop (creep down at {@link #RANGE_DOWN_VOLTS} until the stall,
   * then zero there), climbs with a profile to {@link #RANGE_CREEP_START_BELOW_TOP_INCHES} below
   * the nominal top, creeps up at {@link #RANGE_UP_VOLTS} until the top stall, and takes that as
   * the travel if it is within {@link #TRAVEL_TOLERANCE_INCHES} of nominal. Then the soft limits
   * are set just inside both stops and the stick's full range becomes the measured one. The soft
   * limit in the searched direction is off while it searches. Disabling cancels it.
   */
  public Command autoRange() {
    var s = new Object() {
      RangePhase phase;
      double phaseTime;
      double speed;
    };
    return new FunctionalCommand(
            () -> {
              rangeStatus = RangeStatus.RUNNING;
              io.setDrive(ElevatorIO.Drive.RIGHT_LEADS);
              fault = false;
              faultTimer = 0;
              s.phase = RangePhase.DOWN;
              s.phaseTime = 0;
              s.speed = 0;
              rangeStallSeconds = 0;
              io.setSoftLimits(0, false, 0, false);
            },
            () -> {
              mode = "range-" + s.phase.name().toLowerCase();
              s.phaseTime += 0.02;
              s.speed += 0.5 * (velocityInPerSec - s.speed);
              rangeAmps = (Math.abs(rightStatorAmps) + Math.abs(leftStatorAmps)) / 2.0;
              double ramp = Math.min(1.0, s.phaseTime / RANGE_RAMP_SECONDS);
              switch (s.phase) {
                case DOWN -> {
                  volts(-RANGE_DOWN_VOLTS * ramp);
                  if (stalled(s.phaseTime, s.speed, RANGE_DOWN_STALL_AMPS)) {
                    bottomFoundInches = positionInches;
                    io.setPositionInches(0);
                    positionInches = rightInches = leftInches = 0;
                    zeroKnown = true;
                    s.phase = RangePhase.CLIMB;
                    s.phaseTime = 0;
                    rangeStallSeconds = 0;
                  }
                }
                case CLIMB -> {
                  double creepFrom = NOMINAL_TRAVEL_INCHES - RANGE_CREEP_START_BELOW_TOP_INCHES;
                  closedLoop = true;
                  targetInches = creepFrom;
                  openLoopVolts = 0;
                  if (blocked()) io.neutral(); else io.moveTo(creepFrom);
                  if (Math.abs(positionInches - creepFrom) < 0.15 && Math.abs(s.speed) < 1.0) {
                    s.phase = RangePhase.CREEP;
                    s.phaseTime = 0;
                  }
                }
                case CREEP -> {
                  volts(RANGE_UP_VOLTS * Math.max(ramp, 0.5));
                  if (stalled(s.phaseTime, s.speed, RANGE_UP_STALL_AMPS)) {
                    topFoundInches = positionInches;
                    if (Math.abs(topFoundInches - NOMINAL_TRAVEL_INCHES) <= TRAVEL_TOLERANCE_INCHES) {
                      travelInches = topFoundInches;
                      ranged = true;
                      s.phase = RangePhase.DONE;
                    } else {
                      s.phase = RangePhase.FAILED;
                    }
                  } else if (positionInches > NOMINAL_TRAVEL_INCHES + TRAVEL_TOLERANCE_INCHES) {
                    topFoundInches = positionInches;
                    s.phase = RangePhase.FAILED;
                  }
                }
                default -> {}
              }
              if (s.phaseTime > RANGE_PHASE_TIMEOUT_SECONDS) {
                s.phase = RangePhase.FAILED;
              }
            },
            interrupted -> {
              // Soft limits from what is known now (nominal travel if the top was not found).
              io.setSoftLimits(
                  SOFT_LIMIT_MARGIN_INCHES, true, travelInches - SOFT_LIMIT_MARGIN_INCHES, true);
              if (interrupted) {
                rangeStatus = RangeStatus.INTERRUPTED;
                stop();
              } else if (s.phase == RangePhase.DONE) {
                rangeStatus = RangeStatus.DONE;
                moveTo(highInches()); // hold just below the top; the next command takes it from here
              } else {
                rangeStatus = RangeStatus.FAILED;
                stop();
                DriverStation.reportWarning(
                    String.format(
                        "Elevator auto-range failed (phase %s, top %.2f in): travel stays %.3f in",
                        s.phase, topFoundInches, travelInches),
                    false);
              }
              travelPub.set(travelInches);
            },
            () -> s.phase == RangePhase.DONE || s.phase == RangePhase.FAILED,
            this)
        .withName("ElevatorAutoRange");
  }

  private boolean stalled(double phaseTime, double speed, double amps) {
    boolean now =
        phaseTime >= RANGE_GRACE_SECONDS
            && rangeAmps >= amps
            && Math.abs(speed) < RANGE_STALL_SPEED_IN_PER_SEC;
    rangeStallSeconds = now ? rangeStallSeconds + 0.02 : 0.0;
    return rangeStallSeconds >= RANGE_STALL_SECONDS;
  }

  // ---------------------------------------------------------------- test routines

  /** The standard moves: {@link #MOVES_INCHES}, each held {@link #MOVES_HOLD_SECONDS}. */
  public Command standardMoves() {
    Timer t = new Timer();
    return rangeIfNeeded()
        .andThen(
            new FunctionalCommand(
                t::restart,
                () -> {
                  mode = "moves";
                  int i = (int) Math.min(MOVES_INCHES.length - 1, Math.floor(t.get() / MOVES_HOLD_SECONDS));
                  moveIndex = i;
                  moveTo(MOVES_INCHES[i]);
                },
                interrupted -> moveIndex = -1,
                () -> t.get() >= MOVES_INCHES.length * MOVES_HOLD_SECONDS,
                this))
        .withName("ElevatorMoves");
  }

  /**
   * Full-travel moves, for timing a move across the whole range: the bottom of the range, then
   * the top, alternating, {@link #FULL_TRAVEL_TARGETS} targets each held {@link
   * #FULL_TRAVEL_HOLD_SECONDS}. Never closer to a stop than {@link #STOP_MARGIN_INCHES}.
   */
  public Command fullTravelMoves() {
    return rangeIfNeeded().andThen(fullTravelBody()).withName("ElevatorFullTravel");
  }

  /**
   * Isolation test: the full-travel moves with another {@link ElevatorIO.Drive} (one motor alone,
   * the other coasting; or the left leading). Ranges first if needed (always with the normal
   * drive), switches with the motors in neutral, and switches back to the right leading when it
   * ends by any ending (park and a disable do too).
   */
  public Command fullTravelWith(ElevatorIO.Drive d, String name) {
    return rangeIfNeeded()
        .andThen(runOnce(() -> { stop(); io.setDrive(d); }))
        .andThen(fullTravelBody())
        .finallyDo(() -> { stop(); io.setDrive(ElevatorIO.Drive.RIGHT_LEADS); })
        .withName(name);
  }

  /**
   * Trigger test for the start-to-start change in load (1b): the full-travel moves with one motor
   * alone, where the start either only switches the control requests ({@code writeConfigs} false)
   * or also rewrites every config ({@code true}, as soloright does). Ranges first with the normal
   * drive. The Talons keep whatever configs the last write left (normal gains, so the feed-forward
   * is not doubled; the position loop makes up the difference). Back to the right leading, requests
   * only, when it ends.
   */
  public Command triggerTest(ElevatorIO.Drive d, boolean writeConfigs, String name) {
    return rangeIfNeeded()
        .andThen(runOnce(() -> {
          stop();
          io.switchRequestOnly(d);
          if (writeConfigs) io.rewriteConfigs();
        }))
        .andThen(fullTravelBody())
        .finallyDo(() -> { stop(); io.switchRequestOnly(ElevatorIO.Drive.RIGHT_LEADS); })
        .withName(name);
  }

  /**
   * Open-loop ramp per start (1c): like the solo starts (ranges, switches drive with configs
   * written, goes to the bottom with Motion Magic), then one motor alone ramps VoltageOut up at
   * {@link #RAMP_VOLTS_PER_SEC} to {@link #RAMP_MAX_VOLTS} (or until {@link #CHAR_GUARD_INCHES}
   * below the top), brakes, then ramps down the same way. The log's volts and velocity give kS,
   * kV and the breakaway volts for that start. Back to the right leading when it ends.
   */
  public Command rampTest(ElevatorIO.Drive d, String name) {
    return rangeIfNeeded()
        .andThen(runOnce(() -> { stop(); io.setDrive(d); }))
        .andThen(goAndSettle(lowInches() + 0.25))
        .andThen(rampUntil(+RAMP_VOLTS_PER_SEC, RAMP_MAX_VOLTS, () -> positionInches >= highInches() - RAMP_GUARD_INCHES || velocityInPerSec >= RAMP_STOP_IN_PER_SEC))
        .andThen(brakeThenSettle())
        .andThen(rampUntil(-RAMP_VOLTS_PER_SEC, RAMP_MAX_VOLTS, () -> positionInches <= lowInches() + RAMP_GUARD_INCHES || velocityInPerSec <= -RAMP_STOP_IN_PER_SEC))
        .andThen(brakeThenSettle())
        .finallyDo(() -> { stop(); io.setDrive(ElevatorIO.Drive.RIGHT_LEADS); })
        .withName(name);
  }

  private Command fullTravelBody() {
    Timer t = new Timer();
    return new FunctionalCommand(
                t::restart,
                () -> {
                  mode = "fulltravel";
                  int i = (int) Math.min(FULL_TRAVEL_TARGETS - 1, Math.floor(t.get() / FULL_TRAVEL_HOLD_SECONDS));
                  moveIndex = i;
                  moveTo(i % 2 == 0 ? lowInches() : highInches());
                },
                interrupted -> moveIndex = -1,
                () -> t.get() >= FULL_TRAVEL_TARGETS * FULL_TRAVEL_HOLD_SECONDS,
                this);
  }

  /**
   * Quasistatic characterization: from near the bottom, volts ramp up at {@link
   * #QUASI_VOLTS_PER_SEC} until {@link #CHAR_GUARD_INCHES} below the top; brake and hold; then
   * from 0 V ramp down until the same distance above the bottom; brake and hold. The log's
   * volts and velocity give kS, kG and kV.
   */
  public Command quasistatic() {
    return rangeIfNeeded()
        .andThen(goAndSettle(lowInches() + 0.25))
        .andThen(rampUntil(+QUASI_VOLTS_PER_SEC, () -> positionInches >= highInches() - CHAR_GUARD_INCHES))
        .andThen(brakeThenSettle())
        .andThen(rampUntil(-QUASI_VOLTS_PER_SEC, () -> positionInches <= lowInches() + CHAR_GUARD_INCHES))
        .andThen(brakeThenSettle())
        .withName("ElevatorQuasistatic");
  }

  /** Dynamic characterization: a step of DynamicVolts up, then of DynamicVolts down; gives kA. */
  public Command dynamic() {
    return rangeIfNeeded()
        .andThen(goAndSettle(lowInches() + 0.25))
        .andThen(stepUntil(+1, () -> positionInches >= highInches() - CHAR_GUARD_INCHES))
        .andThen(brakeThenSettle())
        .andThen(stepUntil(-1, () -> positionInches <= lowInches() + CHAR_GUARD_INCHES))
        .andThen(brakeThenSettle())
        .withName("ElevatorDynamic");
  }

  private Command goAndSettle(double inches) {
    return run(() -> { mode = "settle"; moveTo(inches); }).withTimeout(1.5);
  }

  private Command brakeThenSettle() {
    return run(() -> { mode = "brake"; stop(); }).withTimeout(0.3)
        .andThen(startRun(() -> targetInches = positionInches, () -> moveTo(targetInches)).withTimeout(0.7));
  }

  private Command rampUntil(double voltsPerSec, java.util.function.BooleanSupplier done) {
    return rampUntil(voltsPerSec, QUASI_MAX_VOLTS, done);
  }

  private Command rampUntil(double voltsPerSec, double maxVolts, java.util.function.BooleanSupplier done) {
    Timer t = new Timer();
    return new FunctionalCommand(
        t::restart,
        () -> {
          mode = "quasistatic";
          volts(MathUtil.clamp(voltsPerSec * t.get(), -maxVolts, maxVolts));
        },
        interrupted -> stop(),
        () -> done.getAsBoolean() || Math.abs(voltsPerSec * t.get()) >= maxVolts,
        this);
  }

  private Command stepUntil(double sign, java.util.function.BooleanSupplier done) {
    return new FunctionalCommand(
        () -> {},
        () -> { mode = "dynamic"; volts(sign * tuning.get("DynamicVolts")); },
        interrupted -> stop(),
        done,
        this).withTimeout(3.0);
  }

  // ---------------------------------------------------------------- the loop

  private static double stickValue() {
    return DriverStation.isJoystickConnected(STICK_PORT)
            && DriverStation.getStickAxisCount(STICK_PORT) > STICK_AXIS
        ? DriverStation.getStickAxis(STICK_PORT, STICK_AXIS)
        : 0.0;
  }

  @Override
  public void periodic() {
    io.update();
    readSensors();

    boolean enabled = DriverStation.isEnabled();
    aborted = abortSub.get();
    if (tuning.refresh()) {
      tuningDirty = true;
    }
    if (!enabled) {
      mode = "disabled";
      // Never jump on the next enable; clear a fault; write new tuning (the writes block).
      targetInches = positionInches;
      fault = false;
      faultTimer = 0;
      stop();
      io.setDrive(ElevatorIO.Drive.RIGHT_LEADS); // no-op unless an isolation test was cut short
      if (tuningDirty) {
        applyTuning();
      }
    } else {
      // Lag fault: judged only against a running profile.
      double err = io.profileErrorInches();
      profileErrorInches = Double.isNaN(err) ? 0.0 : err;
      faultTimer =
          closedLoop && Math.abs(profileErrorInches) > FAULT_ERROR_INCHES ? faultTimer + 0.02 : 0.0;
      if (faultTimer >= FAULT_SECONDS && !fault) {
        fault = true;
        DriverStation.reportWarning("Elevator lag fault: output cut until disabled", false);
      }
      if (fault || aborted) {
        io.neutral();
      }
      // Parked = motors off, so new tuning can be written while enabled (a sweep changes kP/kD
      // between sets without a disable). Never while anything else runs.
      if (tuningDirty && parked) {
        applyTuning();
      }
      handleRoutineRequest();
    }
    positionPub.set(positionInches);
    targetPub.set(targetInches);
    travelPub.set(travelInches);
    profileErrorPub.set(profileErrorInches);
    kPInUsePub.set(kP);
    kDInUsePub.set(kD);
    view.update(positionInches);
    routinePub.set(getCurrentCommand() == null ? "" : getCurrentCommand().getName());
  }

  /** Runs a routine named in /Elevator/Control/Routine when the name changes (while enabled). */
  private void handleRoutineRequest() {
    String req = routineSub.get();
    if (req.equals(lastRoutineRequest)) {
      return;
    }
    lastRoutineRequest = req;
    Supplier<Command> make = routines.get(req);
    if (make != null) {
      Command c = make.get().finallyDo(() -> routineCountPub.set(++routinesDone));
      c.schedule();
    }
  }

  private void applyTuning() {
    kS = tuning.get("kS");
    kG = tuning.get("kG");
    kV = tuning.get("kV");
    kA = tuning.get("kA");
    kP = tuning.get("kP");
    kD = tuning.get("kD");
    cruiseInPerSec = tuning.get("CruiseInPerSec");
    accelInPerSec2 = tuning.get("AccelInPerSec2");
    jerkInPerSec3 = tuning.get("JerkInPerSec3");
    peakVolts = tuning.get("PeakVolts");
    statorAmps = tuning.get("StatorAmps");
    io.applyGains(new ElevatorIO.Gains(kS, kG, kV, kA, kP, kD, cruiseInPerSec, accelInPerSec2, jerkInPerSec3));
    io.applyCaps(peakVolts, statorAmps);
    tuningDirty = false;
  }

  private void readSensors() {
    rightInches = io.rightInches();
    leftInches = io.leftInches();
    positionInches = rightInches; // both encoders read true inches now; the right is the reference
    sideDiffInches = rightInches - leftInches;
    alignedSideDiffInches = io.alignedSideDiffInches();
    drive = io.drive().name();
    rightFaults = io.faultNames(true);
    leftFaults = io.faultNames(false);
    rightStickyFaults = io.stickyFaultNames(true);
    leftStickyFaults = io.stickyFaultNames(false);
    velocityInPerSec = io.velocityInPerSec();
    profileInches = io.profileInches();
    profileVelocityInPerSec = io.profileVelocityInPerSec();
    rightVolts = io.rightVolts();
    leftVolts = io.leftVolts();
    rightStatorAmps = io.rightStatorAmps();
    leftStatorAmps = io.leftStatorAmps();
    var r = io.rightExtras();
    var l = io.leftExtras();
    rightVelocityRps = r.velocityRps();
    rightSupplyAmps = r.supplyAmps();
    rightTorqueAmps = r.torqueAmps();
    rightDeviceTempC = r.deviceTempC();
    rightProcessorTempC = r.processorTempC();
    rightDuty = r.duty();
    rightSupplyVolts = r.supplyVolts();
    rightClosedLoopError = r.closedLoopError();
    rightClosedLoopReference = r.closedLoopReference();
    rightClosedLoopOutput = r.closedLoopOutput();
    rightFaultField = r.faultField();
    rightStickyFaultField = r.stickyFaultField();
    leftVelocityRps = l.velocityRps();
    leftSupplyAmps = l.supplyAmps();
    leftTorqueAmps = l.torqueAmps();
    leftDeviceTempC = l.deviceTempC();
    leftProcessorTempC = l.processorTempC();
    leftDuty = l.duty();
    leftSupplyVolts = l.supplyVolts();
    leftFaultField = l.faultField();
    leftStickyFaultField = l.stickyFaultField();
    var t = io.loopTerms();
    loopP = t.proportional();
    loopD = t.derivative();
    loopFF = t.feedForward();
    loopOutput = t.output();
    loopError = t.error();
    loopReference = t.reference();
    loopReferenceSlope = t.referenceSlope();
    loopPosition = t.position();
    loopMotorVolts = t.motorVolts();
    loopFollowerVolts = t.followerVolts();
    ageLoopPMs = t.ageProportionalMs();
    ageLoopFFMs = t.ageFeedForwardMs();
    ageLoopOutputMs = t.ageOutputMs();
    ageLoopErrorMs = t.ageErrorMs();
    ageLoopReferenceMs = t.ageReferenceMs();
    ageLoopReferenceSlopeMs = t.ageReferenceSlopeMs();
    ageLoopPositionMs = t.agePositionMs();
    ageLoopMotorVoltsMs = t.ageMotorVoltsMs();
    ageLoopFollowerVoltsMs = t.ageFollowerVoltsMs();
    batteryVolts = RobotController.getBatteryVoltage();
    browningOut = RobotController.isBrownedOut();
    canUtilization = RobotController.getCANStatus().percentBusUtilization;
    loopMs = (Timer.getFPGATimestamp() - lastPeriodic) * 1000.0;
    lastPeriodic = Timer.getFPGATimestamp();
    joystickAxis = stickValue();
  }
}
