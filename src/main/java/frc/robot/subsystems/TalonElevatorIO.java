package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import frc.robot.hardware.Unit;

/**
 * The two Kraken X44 motors on one shaft. The right motor counter-clockwise raises the carriage
 * and the left motor clockwise raises it, so the right is CounterClockwise_Positive and the left
 * Clockwise_Positive.
 *
 * <p>ONE loop: the right Talon LEADS (Motion Magic with kS, kG, kV, kA, kP, kD, or open-loop
 * volts) and the left is a CTRE {@link Follower} of it, Opposed (mounted mirrored), so both motors
 * always get the same volts. That is how kS/kG/kV/kA were measured (EL29, both motors at the same
 * volts), so the gains are for the pair and the right's slot uses them unscaled. With each Talon
 * running its own copy of the move (EL28-EL30), the left's share depended on its own feedforward
 * being exact, and the two pulled against each other (EL30: opposed 38% of descents).
 *
 * <p>The sides are geared differently (right 15T pinion, left 16T, both to 46T on the shaft), so
 * each Talon has its OWN SensorToMechanismRatio and both encoders read true inches. With the same
 * volts the left, spinning 15/16 as fast, has less back-EMF and carries more current while moving
 * (about 0.1 A more per in/s); holding still they carry the same. The left has no soft limits of
 * its own, so it can never stop following on its own: the right's soft limits guard both.
 */
public class TalonElevatorIO implements ElevatorIO {
  private static final double CONFIG_TIMEOUT_S = 0.05;
  private static final double LOG_HZ = 100.0;

  private final TalonFX right;
  private final TalonFX left;
  private final MotionMagicVoltage move = new MotionMagicVoltage(0).withEnableFOC(false);
  private final VoltageOut voltsReq = new VoltageOut(0).withEnableFOC(false);
  private final Follower follow;
  private final NeutralOut neutral = new NeutralOut();
  private final StatusSignal<?>[] rightSignals;
  private final StatusSignal<?>[] leftSignals;
  private final StatusSignal<Double> refSlope;
  private final StatusSignal<ControlModeValue> controlMode;
  private Extras rightX = Extras.ZERO;
  private Extras leftX = Extras.ZERO;

  public TalonElevatorIO(Unit unit) {
    right = new TalonFX(unit.rightMotorId());
    left = new TalonFX(unit.leftMotorId());
    follow = new Follower(right.getDeviceID(), MotorAlignmentValue.Opposed);
    configure(right, InvertedValue.CounterClockwise_Positive, unit.rightRotationsPerInch(), true);
    configure(left, InvertedValue.Clockwise_Positive, unit.leftRotationsPerInch(), false);
    right.setPosition(0);
    left.setPosition(0);
    rightSignals = signalsOf(right);
    leftSignals = signalsOf(left);
    refSlope = right.getClosedLoopReferenceSlope();
    controlMode = right.getControlMode();
    BaseStatusSignal.setUpdateFrequencyForAll(LOG_HZ, rightSignals);
    BaseStatusSignal.setUpdateFrequencyForAll(LOG_HZ, leftSignals);
    BaseStatusSignal.setUpdateFrequencyForAll(LOG_HZ, refSlope, controlMode);
    right.optimizeBusUtilization();
    left.optimizeBusUtilization();
  }

  /** Every signal read each loop. Order matters: see {@link #extrasOf} and {@link #update}. */
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

  /** leader: the right Talon, with the gains and soft limits; the follower gets neither. */
  private static void configure(
      TalonFX motor, InvertedValue direction, double rotationsPerInch, boolean leader) {
    var cfg = new TalonFXConfiguration();
    cfg.MotorOutput.Inverted = direction;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Work in inches of carriage travel (each motor's own gearing).
    cfg.Feedback.SensorToMechanismRatio = rotationsPerInch;
    cfg.Voltage.PeakForwardVoltage = Elevator.DEFAULT_PEAK_VOLTS;
    cfg.Voltage.PeakReverseVoltage = -Elevator.DEFAULT_PEAK_VOLTS;
    cfg.CurrentLimits.StatorCurrentLimit = Elevator.DEFAULT_STATOR_AMPS;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = Elevator.DEFAULT_STATOR_AMPS;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    if (leader) {
      cfg.Slot0 = slot0(Elevator.DEFAULT_GAINS);
      cfg.MotionMagic = motionMagic(Elevator.DEFAULT_GAINS);
      // Until the travel is measured, soft limits assume zero is the bottom and nominal travel.
      cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
          Elevator.NOMINAL_TRAVEL_INCHES - Elevator.SOFT_LIMIT_MARGIN_INCHES;
      cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Elevator.SOFT_LIMIT_MARGIN_INCHES;
    }
    motor.getConfigurator().apply(cfg);
  }

  /** Slot 0 from the gains, for the pair of motors at the same volts (as measured). */
  private static Slot0Configs slot0(Gains g) {
    var s = new Slot0Configs();
    s.GravityType = GravityTypeValue.Elevator_Static;
    s.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    s.kS = g.kS();
    s.kG = g.kG();
    s.kV = g.kV();
    s.kA = g.kA();
    s.kP = g.kP();
    s.kD = g.kD();
    return s;
  }

  private static MotionMagicConfigs motionMagic(Gains g) {
    var m = new MotionMagicConfigs();
    m.MotionMagicCruiseVelocity = g.cruiseInPerSec();
    m.MotionMagicAcceleration = g.accelInPerSec2();
    m.MotionMagicJerk = g.jerkInPerSec3();
    return m;
  }

  @Override
  public void applyGains(Gains g) {
    // Only the leader runs the loop; the follower copies its volts.
    right.getConfigurator().apply(slot0(g), CONFIG_TIMEOUT_S);
    right.getConfigurator().apply(motionMagic(g), CONFIG_TIMEOUT_S);
  }

  @Override
  public void applyCaps(double peakVolts, double statorAmps) {
    var v = new VoltageConfigs();
    v.PeakForwardVoltage = peakVolts;
    v.PeakReverseVoltage = -peakVolts;
    var c = new CurrentLimitsConfigs();
    c.StatorCurrentLimit = statorAmps;
    c.StatorCurrentLimitEnable = true;
    c.SupplyCurrentLimit = statorAmps;
    c.SupplyCurrentLimitEnable = true;
    for (TalonFX m : new TalonFX[] {right, left}) {
      m.getConfigurator().apply(v, CONFIG_TIMEOUT_S);
      m.getConfigurator().apply(c, CONFIG_TIMEOUT_S);
    }
  }

  @Override
  public void setSoftLimits(double reverseInches, boolean reverseOn, double forwardInches, boolean forwardOn) {
    var s = new SoftwareLimitSwitchConfigs();
    s.ReverseSoftLimitThreshold = reverseInches;
    s.ReverseSoftLimitEnable = reverseOn;
    s.ForwardSoftLimitThreshold = forwardInches;
    s.ForwardSoftLimitEnable = forwardOn;
    // The leader's only: the follower must never stop following on its own.
    right.getConfigurator().apply(s, CONFIG_TIMEOUT_S);
  }

  @Override
  public void moveTo(double inches) {
    right.setControl(move.withPosition(inches));
    left.setControl(follow);
  }

  @Override
  public void setVolts(double volts) {
    right.setControl(voltsReq.withOutput(volts));
    left.setControl(follow);
  }

  @Override
  public void neutral() {
    right.setControl(neutral);
    left.setControl(neutral);
  }

  @Override
  public void setPositionInches(double inches) {
    right.setPosition(inches);
    left.setPosition(inches);
  }

  @Override
  public void update() {
    BaseStatusSignal.refreshAll(rightSignals);
    BaseStatusSignal.refreshAll(leftSignals);
    BaseStatusSignal.refreshAll(refSlope, controlMode);
    rightX = extrasOf(rightSignals);
    leftX = extrasOf(leftSignals);
  }

  private boolean profiling() {
    return controlMode.getValue() == ControlModeValue.MotionMagicVoltage;
  }

  @Override public Extras rightExtras() { return rightX; }
  @Override public Extras leftExtras() { return leftX; }
  @Override public double rightInches() { return rightSignals[0].getValueAsDouble(); }
  @Override public double leftInches() { return leftSignals[0].getValueAsDouble(); }
  @Override public double velocityInPerSec() { return rightSignals[3].getValueAsDouble(); }
  @Override public double profileInches() { return profiling() ? rightX.closedLoopReference() : Double.NaN; }
  // Phoenix 6 closed-loop error is reference - position; ours is position - reference.
  @Override public double profileErrorInches() { return profiling() ? -rightX.closedLoopError() : Double.NaN; }
  @Override public double profileVelocityInPerSec() { return profiling() ? refSlope.getValueAsDouble() : Double.NaN; }
  @Override public double rightVolts() { return rightSignals[1].getValueAsDouble(); }
  @Override public double leftVolts() { return leftSignals[1].getValueAsDouble(); }
  @Override public double rightStatorAmps() { return rightSignals[2].getValueAsDouble(); }
  @Override public double leftStatorAmps() { return leftSignals[2].getValueAsDouble(); }
}
