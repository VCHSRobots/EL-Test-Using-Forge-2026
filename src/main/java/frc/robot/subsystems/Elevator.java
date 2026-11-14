package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The two-motor elevator stand. Position is in inches above the bottom stop (zero = carriage on
 * the bottom stop at power-up, so ALWAYS power up with the carriage resting on the bottom stop).
 *
 * <p>Each motor runs its own position loop on a setpoint that this class ramps at a limited
 * speed, so the carriage can never be asked to move faster than {@link #speedInPerSec}. Directions:
 * the right motor counter-clockwise raises the carriage, the left motor clockwise raises it, so
 * the right is built CounterClockwise_Positive and the left Clockwise_Positive: positive motor
 * rotation means "up" on both.
 *
 * <p>NetworkTables control (table /Elevator/Control): write TargetInches (absolute, clamped to the
 * travel) and optionally SpeedInPerSec (default 1, never above {@link #MAX_SPEED_IN_PER_SEC}).
 * Everything moves only while the robot is enabled; on disable the target and setpoint snap back
 * to where the carriage is, so an enable never jumps.
 */
@Logged
public class Elevator extends SubsystemBase {
  // --- Mechanical constants (from the stand's design) ---
  public static final double MOTOR_ROTATIONS_PER_SHAFT_TURN = 2.875; // 2.875:1 reduction
  public static final double INCHES_PER_SHAFT_TURN = 4.0; // chain travel per shaft turn
  public static final double MOTOR_ROTATIONS_PER_INCH =
      MOTOR_ROTATIONS_PER_SHAFT_TURN / INCHES_PER_SHAFT_TURN;
  public static final double TRAVEL_INCHES = 9.5;

  // --- Motion and safety limits (first-test values; raise only with care) ---
  public static final double DEFAULT_SPEED_IN_PER_SEC = 1.0;
  /** Stick control: full forward on the stick is this height above the bottom (director, 14 Nov). */
  public static final double STICK_TOP_INCHES = 8.0;
  /** The most the stick may move the carriage, in inches per second (the setpoint is ramped at it). */
  public static final double STICK_SPEED_IN_PER_SEC = 2.0;
  /** The stick axis (the first stick's Y; forward reads negative) and its port. */
  public static final int STICK_PORT = 0;
  public static final int STICK_AXIS = 1;
  public static final double STICK_DEADBAND = 0.08;
  public static final double MAX_SPEED_IN_PER_SEC = 2.0;
  /** Stay this far from the stops: the target is clamped to [MARGIN, TRAVEL - MARGIN]. */
  public static final double STOP_MARGIN_INCHES = 0.25;
  /** If a motor lags its setpoint by more than this for FAULT_SECONDS, the output is cut. */
  public static final double FAULT_ERROR_INCHES = 1.0;
  public static final double FAULT_SECONDS = 0.25;
  public static final double PEAK_VOLTS = 3.0;
  public static final double STATOR_LIMIT_AMPS = 20.0;
  public static final double SUPPLY_LIMIT_AMPS = 20.0;
  public static final double KP_VOLTS_PER_ROTATION = 6.0;

  // --- Auto-home (find the bottom hard stop by its stall) ---
  /** Open-loop volts pushing DOWN while homing (ramped in over HOME_RAMP_SECONDS). */
  public static final double HOME_VOLTS = 0.6;
  public static final double HOME_RAMP_SECONDS = 0.3;
  /** Ignore the stall test for this long after the start (spin-up current is not a stall). */
  public static final double HOME_GRACE_SECONDS = 0.5;
  /** Stalled = average stator current at least this ... */
  public static final double HOME_STALL_AMPS = 6.0;
  /** ... and speed below this (inches per second) ... */
  public static final double HOME_STALL_SPEED_IN_PER_SEC = 0.3;
  /** ... for this long in a row. */
  public static final double HOME_STALL_SECONDS = 0.2;
  /** Give up (no zero) after this long without a stall. */
  public static final double HOME_TIMEOUT_SECONDS = 8.0;

  /** Which motors run the position loop. The others are in neutral but still logged. */
  public enum Drive {
    BOTH,
    RIGHT_ONLY,
    LEFT_ONLY,
    /** Right motor runs the position loop; the left follows it (CTRE Follower, opposed). */
    LEADER_FOLLOWER
  }

  private final ElevatorIO io;
  private Drive drive = Drive.LEADER_FOLLOWER;

  private final DoubleSubscriber targetSub;
  private final DoubleSubscriber speedSub;
  private final DoublePublisher positionPub;
  private final DoublePublisher setpointPub;
  private final DoublePublisher homeOffsetPub;

  /** 0 = both motors, 1 = right only, 2 = left only (for the log). */
  double driveMode;
  double targetInches;
  double setpointInches;
  double speedInPerSec = DEFAULT_SPEED_IN_PER_SEC;
  double rightInches;
  double leftInches;
  double rightVolts;
  double leftVolts;
  double rightStatorAmps;
  double leftStatorAmps;
  boolean fault;
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
  double leftClosedLoopError;
  double leftClosedLoopReference;
  double leftClosedLoopOutput;
  double leftFaultField;
  double leftStickyFaultField;
  double batteryVolts;
  double loopMs;
  double canUtilization;
  boolean browningOut;
  double joystickAxis;
  /** True while the homing command runs. */
  boolean homing;
  /** 0 never run, 1 running, 2 found the bottom, 3 timed out (no zero set), 4 interrupted. */
  double homeStatus;
  /** Where the bottom was found, in inches on the OLD zero (negative: below it). Last homing. */
  double homeFoundInches;
  double homeStallSeconds;
  double homeSpeedInPerSec;
  double homeAmps;
  private double homeElapsed;
  private double homeLastInches;
  private boolean homeStalled;
  private double faultTimer;
  private double lastPeriodic = Timer.getFPGATimestamp();

  public Elevator(ElevatorIO io) {
    this.io = io;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Elevator/Control");
    targetSub = table.getDoubleTopic("TargetInches").subscribe(Double.NaN);
    speedSub = table.getDoubleTopic("SpeedInPerSec").subscribe(DEFAULT_SPEED_IN_PER_SEC);
    // Publish defaults so the topics exist and can be written from outside.
    table.getDoubleTopic("TargetInches").publish().set(0.0);
    table.getDoubleTopic("SpeedInPerSec").publish().set(DEFAULT_SPEED_IN_PER_SEC);
    NetworkTable status = NetworkTableInstance.getDefault().getTable("Elevator/Status");
    positionPub = status.getDoubleTopic("PositionInches").publish();
    setpointPub = status.getDoubleTopic("SetpointInches").publish();
    homeOffsetPub = status.getDoubleTopic("HomeFoundInches").publish();
  }

  /** Current carriage height in inches (average of the two motors). */
  public double positionInches() {
    return (rightInches + leftInches) / 2.0;
  }

  public double setpointInches() {
    return setpointInches;
  }

  public double targetInches() {
    return targetInches;
  }

  public boolean isHoming() {
    return homing;
  }

  /** 0 never run, 1 running, 2 found the bottom, 3 timed out, 4 interrupted. */
  public double homeStatus() {
    return homeStatus;
  }

  public double homeFoundInches() {
    return homeFoundInches;
  }

  public boolean faulted() {
    return fault;
  }

  /** Reached the target (within 0.1 in) and the setpoint has arrived. */
  public boolean atTarget() {
    return Math.abs(setpointInches - targetInches) < 1e-6
        && Math.abs(positionInches() - targetInches) < 0.1;
  }

  /** Choose which motors drive. Reset to LEADER_FOLLOWER on every disable. */
  public void setDrive(Drive d) {
    drive = d;
  }

  public Drive drive() {
    return drive;
  }

  /** Sets the target (clamped to the travel) from code. NetworkTables writes do the same. */
  public void setTarget(double inches) {
    targetInches = clampToTravel(inches);
  }

  private static double clampToTravel(double inches) {
    return MathUtil.clamp(inches, STOP_MARGIN_INCHES, TRAVEL_INCHES - STOP_MARGIN_INCHES);
  }

  /** The default command: follow the NetworkTables target at the NetworkTables speed. */
  public Command followNetworkTables() {
    return run(this::stepFromNetworkTables).withName("ElevatorFollowNT");
  }

  /**
   * Joystick jog: axis (-1..1, positive = up) times {@link #DEFAULT_SPEED_IN_PER_SEC} moves the
   * target; released holds position. The setpoint is still ramped at that speed limit.
   */
  public Command jog(DoubleSupplier axis) {
    return run(() -> {
          double a = MathUtil.applyDeadband(axis.getAsDouble(), 0.1);
          speedInPerSec = DEFAULT_SPEED_IN_PER_SEC;
          targetInches = clampToTravel(targetInches + a * speedInPerSec * 0.02);
          step();
        })
        .withName("ElevatorJog");
  }

  /**
   * Stick height control: the stick (axis 1 of port 0, forward = up) sets the wanted height, 0 in
   * when released and {@link #STICK_TOP_INCHES} at full forward. The carriage is assumed to be on
   * the bottom stop whenever the robot is disabled (see {@link #periodic}), so it can only go up
   * from there. The setpoint moves toward the wanted height at {@link #STICK_SPEED_IN_PER_SEC}.
   * A missing pad reads as released, so the carriage goes down.
   */
  public Command followStick() {
    return run(() -> {
          double a = stickValue();
          a = Math.abs(a) < STICK_DEADBAND ? 0.0 : a;
          targetInches = MathUtil.clamp(-a, 0.0, 1.0) * STICK_TOP_INCHES;
          speedInPerSec = STICK_SPEED_IN_PER_SEC;
          step();
        })
        .withName("ElevatorFollowStick");
  }

  /**
   * Auto-home: takes over the elevator, drives both motors DOWN at {@link #HOME_VOLTS} (low
   * speed, low current) until the carriage stalls on the bottom hard stop (average stator
   * current at least {@link #HOME_STALL_AMPS} while the speed is under {@link
   * #HOME_STALL_SPEED_IN_PER_SEC}, for {@link #HOME_STALL_SECONDS}, never judged in the first
   * {@link #HOME_GRACE_SECONDS}), then stops, makes that position 0 in, re-arms the soft limits
   * and ends so the stick takes over again. On a timeout it sets no zero. The reverse soft limit
   * is off while it runs, since the zero it replaces may be wrong. Disabling cancels it.
   */
  public Command autoHome() {
    return new FunctionalCommand(
            this::homeInit, this::homeStep, this::homeEnd, () -> homeStalled || homeElapsed >= HOME_TIMEOUT_SECONDS, this)
        .withName("ElevatorAutoHome");
  }

  private void homeInit() {
    homing = true;
    homeStalled = false;
    homeElapsed = 0.0;
    homeStallSeconds = 0.0;
    homeSpeedInPerSec = 0.0;
    homeLastInches = positionInches();
    homeStatus = 1;
    fault = false;
    faultTimer = 0;
    drive = Drive.LEADER_FOLLOWER;
    io.setReverseLimitEnabled(false);
  }

  private void homeStep() {
    homeElapsed += 0.02;
    double ramp = Math.min(1.0, homeElapsed / HOME_RAMP_SECONDS);
    io.homeDrive(HOME_VOLTS * ramp);

    // Speed from the position change, lightly filtered (the simulation has no velocity signal).
    double now = positionInches();
    double raw = (now - homeLastInches) / 0.02;
    homeLastInches = now;
    homeSpeedInPerSec += 0.25 * (raw - homeSpeedInPerSec);
    homeAmps = (Math.abs(rightStatorAmps) + Math.abs(leftStatorAmps)) / 2.0;

    boolean stalledNow =
        homeElapsed >= HOME_GRACE_SECONDS
            && homeAmps >= HOME_STALL_AMPS
            && Math.abs(homeSpeedInPerSec) < HOME_STALL_SPEED_IN_PER_SEC;
    homeStallSeconds = stalledNow ? homeStallSeconds + 0.02 : 0.0;
    if (homeStallSeconds >= HOME_STALL_SECONDS) {
      homeStalled = true;
    }
  }

  private void homeEnd(boolean interrupted) {
    io.neutral();
    io.setReverseLimitEnabled(true);
    homing = false;
    if (interrupted) {
      homeStatus = 4;
    } else if (homeStalled) {
      homeFoundInches = positionInches();
      io.zero();
      rightInches = 0.0;
      leftInches = 0.0;
      setpointInches = 0.0;
      targetInches = 0.0;
      homeStatus = 2;
    } else {
      homeStatus = 3;
      DriverStation.reportWarning("Elevator auto-home timed out without a stall: zero not changed", false);
    }
    homeOffsetPub.set(homeFoundInches);
  }

  private static double stickValue() {
    return DriverStation.isJoystickConnected(STICK_PORT)
            && DriverStation.getStickAxisCount(STICK_PORT) > STICK_AXIS
        ? DriverStation.getStickAxis(STICK_PORT, STICK_AXIS)
        : 0.0;
  }

  private void stepFromNetworkTables() {
    double nt = targetSub.get();
    if (!Double.isNaN(nt)) {
      targetInches = clampToTravel(nt);
    }
    double speed = speedSub.get();
    speedInPerSec =
        Double.isNaN(speed) ? DEFAULT_SPEED_IN_PER_SEC : MathUtil.clamp(speed, 0.0, MAX_SPEED_IN_PER_SEC);
    step();
  }

  private void step() {
    double step = speedInPerSec * 0.02;
    setpointInches += MathUtil.clamp(targetInches - setpointInches, -step, step);

    if (fault) {
      io.neutral();
      return;
    }
    if (drive == Drive.LEADER_FOLLOWER) {
      io.setLeaderFollower(setpointInches * MOTOR_ROTATIONS_PER_INCH);
      return;
    }
    io.setPosition(
        setpointInches * MOTOR_ROTATIONS_PER_INCH,
        drive != Drive.LEFT_ONLY,
        drive != Drive.RIGHT_ONLY);
  }

  @Override
  public void periodic() {
    io.update();
    rightInches = io.rightRotations() / MOTOR_ROTATIONS_PER_INCH;
    leftInches = io.leftRotations() / MOTOR_ROTATIONS_PER_INCH;
    rightVolts = io.rightVolts();
    leftVolts = io.leftVolts();
    rightStatorAmps = io.rightStatorAmps();
    leftStatorAmps = io.leftStatorAmps();

    var rightX = io.rightExtras();
    var leftX = io.leftExtras();
    rightVelocityRps = rightX.velocityRps();
    rightSupplyAmps = rightX.supplyAmps();
    rightTorqueAmps = rightX.torqueAmps();
    rightDeviceTempC = rightX.deviceTempC();
    rightProcessorTempC = rightX.processorTempC();
    rightDuty = rightX.duty();
    rightSupplyVolts = rightX.supplyVolts();
    rightClosedLoopError = rightX.closedLoopError();
    rightClosedLoopReference = rightX.closedLoopReference();
    rightClosedLoopOutput = rightX.closedLoopOutput();
    rightFaultField = rightX.faultField();
    rightStickyFaultField = rightX.stickyFaultField();
    leftVelocityRps = leftX.velocityRps();
    leftSupplyAmps = leftX.supplyAmps();
    leftTorqueAmps = leftX.torqueAmps();
    leftDeviceTempC = leftX.deviceTempC();
    leftProcessorTempC = leftX.processorTempC();
    leftDuty = leftX.duty();
    leftSupplyVolts = leftX.supplyVolts();
    leftClosedLoopError = leftX.closedLoopError();
    leftClosedLoopReference = leftX.closedLoopReference();
    leftClosedLoopOutput = leftX.closedLoopOutput();
    leftFaultField = leftX.faultField();
    leftStickyFaultField = leftX.stickyFaultField();
    batteryVolts = RobotController.getBatteryVoltage();
    browningOut = RobotController.isBrownedOut();
    canUtilization = RobotController.getCANStatus().percentBusUtilization;
    loopMs = (Timer.getFPGATimestamp() - lastPeriodic) * 1000.0;
    lastPeriodic = Timer.getFPGATimestamp();
    joystickAxis = DriverStation.isJoystickConnected(0) ? DriverStation.getStickAxis(0, 1) : 0.0;

    boolean enabled = DriverStation.isEnabled();
    if (!enabled) {
      // The carriage is taken to be on the bottom stop whenever disabled: zero the encoders here,
      // so every enable starts at 0 in. Hold the target and setpoint there (never jump on
      // enable); clear a fault.
      io.zero();
      rightInches = 0.0;
      leftInches = 0.0;
      setpointInches = 0.0;
      targetInches = setpointInches;
      fault = false;
      faultTimer = 0;
      drive = Drive.LEADER_FOLLOWER;
      io.neutral();
    } else {
      double worst =
          Math.max(Math.abs(rightInches - setpointInches), Math.abs(leftInches - setpointInches));
      faultTimer = !homing && worst > FAULT_ERROR_INCHES ? faultTimer + 0.02 : 0.0;
      if (faultTimer >= FAULT_SECONDS) {
        fault = true;
      }
    }
    driveMode = drive.ordinal();
    positionPub.set(positionInches());
    setpointPub.set(setpointInches);
  }
}
