package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.hardware.Unit;

/**
 * The two Kraken motors. The right motor counter-clockwise raises the carriage and the left motor
 * clockwise raises it, so the right is CounterClockwise_Positive and the left Clockwise_Positive:
 * positive rotation is UP on both. Each motor runs its own position loop on its own encoder.
 * Position is zeroed at construction: power up with the carriage on the bottom stop.
 */
public class TalonElevatorIO implements ElevatorIO {
  private final TalonFX right;
  private final TalonFX left;
  private final PositionVoltage rightRequest = new PositionVoltage(0);
  private final PositionVoltage leftRequest = new PositionVoltage(0);
  private final Follower followRequest;
  private final NeutralOut neutral = new NeutralOut();
  private double rightRot;
  private double leftRot;
  private double rightV;
  private double leftV;
  private double rightA;
  private double leftA;
  private static final double LOG_HZ = 100.0;
  private final StatusSignal<?>[] rightSignals;
  private final StatusSignal<?>[] leftSignals;
  private Extras rightX = Extras.ZERO;
  private Extras leftX = Extras.ZERO;

  public TalonElevatorIO(Unit unit) {
    right = new TalonFX(unit.rightMotorId());
    left = new TalonFX(unit.leftMotorId());
    followRequest = new Follower(right.getDeviceID(), MotorAlignmentValue.Opposed);
    configure(right, InvertedValue.CounterClockwise_Positive);
    configure(left, InvertedValue.Clockwise_Positive);
    right.setPosition(0);
    left.setPosition(0);
    rightSignals = signalsOf(right);
    leftSignals = signalsOf(left);
    BaseStatusSignal.setUpdateFrequencyForAll(LOG_HZ, rightSignals);
    BaseStatusSignal.setUpdateFrequencyForAll(LOG_HZ, leftSignals);
    right.optimizeBusUtilization();
    left.optimizeBusUtilization();
  }

  /** Every signal read each loop. Order matters: see {@link #extrasOf}. */
  private static StatusSignal<?>[] signalsOf(TalonFX m) {
    return new StatusSignal<?>[] {
      m.getPosition(), m.getMotorVoltage(), m.getStatorCurrent(), // 0-2
      m.getVelocity(), m.getSupplyCurrent(), m.getTorqueCurrent(), // 3-5
      m.getDeviceTemp(), m.getProcessorTemp(), m.getDutyCycle(), // 6-8
      m.getSupplyVoltage(), m.getClosedLoopError(), m.getClosedLoopReference(), // 9-11
      m.getClosedLoopOutput(), m.getFaultField(), m.getStickyFaultField() // 12-14
    };
  }

  private static Extras extrasOf(StatusSignal<?>[] s) {
    double[] v = new double[s.length];
    for (int i = 0; i < s.length; i++) {
      v[i] = s[i].getValueAsDouble();
    }
    return new Extras(v[3], v[4], v[5], v[6], v[7], v[8], v[9], v[10], v[11], v[12], v[13], v[14]);
  }

  private static void configure(TalonFX motor, InvertedValue direction) {
    var cfg = new TalonFXConfiguration();
    cfg.MotorOutput.Inverted = direction;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.Voltage.PeakForwardVoltage = Elevator.PEAK_VOLTS;
    cfg.Voltage.PeakReverseVoltage = -Elevator.PEAK_VOLTS;
    cfg.CurrentLimits.StatorCurrentLimit = Elevator.STATOR_LIMIT_AMPS;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = Elevator.SUPPLY_LIMIT_AMPS;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.Slot0.kP = Elevator.KP_VOLTS_PER_ROTATION;
    // Soft limits in motor rotations: a last line of defence inside the motor controller.
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        (Elevator.TRAVEL_INCHES - Elevator.STOP_MARGIN_INCHES / 2) * Elevator.MOTOR_ROTATIONS_PER_INCH;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        -Elevator.STOP_MARGIN_INCHES / 2 * Elevator.MOTOR_ROTATIONS_PER_INCH;
    motor.getConfigurator().apply(cfg);
  }

  @Override
  public void setPosition(double rotations) {
    right.setControl(rightRequest.withPosition(rotations));
    left.setControl(leftRequest.withPosition(rotations));
  }

  /** Left follows right. Opposed: the left motor is mounted mirrored (opposite invert), so it must turn opposite the right rotor. */
  @Override
  public void setLeaderFollower(double rotations) {
    right.setControl(rightRequest.withPosition(rotations));
    left.setControl(followRequest);
  }

  @Override
  public void setPosition(double rotations, boolean useRight, boolean useLeft) {
    if (useRight) {
      right.setControl(rightRequest.withPosition(rotations));
    } else {
      right.setControl(neutral);
    }
    if (useLeft) {
      left.setControl(leftRequest.withPosition(rotations));
    } else {
      left.setControl(neutral);
    }
  }

  @Override
  public void neutral() {
    right.setControl(neutral);
    left.setControl(neutral);
  }

  @Override
  public void update() {
    BaseStatusSignal.refreshAll(rightSignals);
    BaseStatusSignal.refreshAll(leftSignals);
    rightRot = rightSignals[0].getValueAsDouble();
    leftRot = leftSignals[0].getValueAsDouble();
    rightV = rightSignals[1].getValueAsDouble();
    leftV = leftSignals[1].getValueAsDouble();
    rightA = rightSignals[2].getValueAsDouble();
    leftA = leftSignals[2].getValueAsDouble();
    rightX = extrasOf(rightSignals);
    leftX = extrasOf(leftSignals);
  }

  @Override public Extras rightExtras() { return rightX; }
  @Override public Extras leftExtras() { return leftX; }

  @Override public double rightRotations() { return rightRot; }
  @Override public double leftRotations() { return leftRot; }
  @Override public double rightVolts() { return rightV; }
  @Override public double leftVolts() { return leftV; }
  @Override public double rightStatorAmps() { return rightA; }
  @Override public double leftStatorAmps() { return leftA; }
}
