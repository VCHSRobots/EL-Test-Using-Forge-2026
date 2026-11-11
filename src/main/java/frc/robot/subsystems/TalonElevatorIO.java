package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
  private final NeutralOut neutral = new NeutralOut();
  private double rightRot;
  private double leftRot;
  private double rightV;
  private double leftV;
  private double rightA;
  private double leftA;

  public TalonElevatorIO(Unit unit) {
    right = new TalonFX(unit.rightMotorId());
    left = new TalonFX(unit.leftMotorId());
    configure(right, InvertedValue.CounterClockwise_Positive);
    configure(left, InvertedValue.Clockwise_Positive);
    right.setPosition(0);
    left.setPosition(0);
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

  @Override
  public void neutral() {
    right.setControl(neutral);
    left.setControl(neutral);
  }

  @Override
  public void update() {
    rightRot = right.getPosition().getValueAsDouble();
    leftRot = left.getPosition().getValueAsDouble();
    rightV = right.getMotorVoltage().getValueAsDouble();
    leftV = left.getMotorVoltage().getValueAsDouble();
    rightA = right.getStatorCurrent().getValueAsDouble();
    leftA = left.getStatorCurrent().getValueAsDouble();
  }

  @Override public double rightRotations() { return rightRot; }
  @Override public double leftRotations() { return leftRot; }
  @Override public double rightVolts() { return rightV; }
  @Override public double leftVolts() { return leftV; }
  @Override public double rightStatorAmps() { return rightA; }
  @Override public double leftStatorAmps() { return leftA; }
}
