package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
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

  private final ElevatorIO io;

  private final DoubleSubscriber targetSub;
  private final DoubleSubscriber speedSub;
  private final DoublePublisher positionPub;
  private final DoublePublisher setpointPub;

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
  private double faultTimer;

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

  public boolean faulted() {
    return fault;
  }

  /** Reached the target (within 0.1 in) and the setpoint has arrived. */
  public boolean atTarget() {
    return Math.abs(setpointInches - targetInches) < 1e-6
        && Math.abs(positionInches() - targetInches) < 0.1;
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

  private void stepFromNetworkTables() {
    double nt = targetSub.get();
    if (!Double.isNaN(nt)) {
      targetInches = clampToTravel(nt);
    }
    double speed = speedSub.get();
    speedInPerSec =
        Double.isNaN(speed) ? DEFAULT_SPEED_IN_PER_SEC : MathUtil.clamp(speed, 0.0, MAX_SPEED_IN_PER_SEC);

    double step = speedInPerSec * 0.02;
    setpointInches += MathUtil.clamp(targetInches - setpointInches, -step, step);

    if (fault) {
      io.neutral();
      return;
    }
    io.setPosition(setpointInches * MOTOR_ROTATIONS_PER_INCH);
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

    boolean enabled = DriverStation.isEnabled();
    if (!enabled) {
      // Never jump on enable: hold the target and setpoint where the carriage is; clear a fault.
      setpointInches = positionInches();
      targetInches = setpointInches;
      fault = false;
      faultTimer = 0;
      io.neutral();
    } else {
      double worst =
          Math.max(Math.abs(rightInches - setpointInches), Math.abs(leftInches - setpointInches));
      faultTimer = worst > FAULT_ERROR_INCHES ? faultTimer + 0.02 : 0.0;
      if (faultTimer >= FAULT_SECONDS) {
        fault = true;
      }
    }
    positionPub.set(positionInches());
    setpointPub.set(setpointInches);
  }
}
