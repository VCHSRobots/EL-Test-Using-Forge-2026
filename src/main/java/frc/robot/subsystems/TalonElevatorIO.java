package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
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
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

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
 *
 * <p>For isolation tests only ({@link Drive}): the left can lead with the right following, or one
 * motor can drive alone with the other coasting. The leader always has the gains, Motion Magic
 * and the soft limits; the other motor never has soft limits. Each fault is also read by name.
 */
public class TalonElevatorIO implements ElevatorIO {
  private static final double CONFIG_TIMEOUT_S = 0.05;
  private static final double LOG_HZ = 100.0;
  // A Follower changes its output only when the leader's output signal (MotorVoltage for our
  // voltage requests, DutyCycle otherwise) arrives, so that signal's rate is the follower's
  // update rate. At 100 Hz the follower held each value for 10 ms while the leader's loop ran
  // at 1 kHz, and the two motors split the load (run bd0ca0b8). Send it at 1 kHz on both
  // Talons, since either may lead.
  private static final double FOLLOW_HZ = 1000.0;

  private final TalonFX right;
  private final TalonFX left;
  private final MotionMagicVoltage move = new MotionMagicVoltage(0).withEnableFOC(false);
  private final VoltageOut voltsReq = new VoltageOut(0).withEnableFOC(false);
  private final Follower followRight;
  private final Follower followLeft;
  private final CoastOut coast = new CoastOut();
  private final NeutralOut neutral = new NeutralOut();
  private final StatusSignal<?>[] rightSignals;
  private final StatusSignal<?>[] leftSignals;
  private final StatusSignal<Double> rightRefSlope;
  private final StatusSignal<Double> leftRefSlope;
  private final StatusSignal<ControlModeValue> rightControlMode;
  private final StatusSignal<ControlModeValue> leftControlMode;
  private final List<NamedFault> rightFaults;
  private final List<NamedFault> leftFaults;
  private Drive drive = Drive.RIGHT_LEADS;
  private Gains gains = Elevator.DEFAULT_GAINS;
  // The soft limits the leader gets (the last ones asked for).
  private double reverseLimit = Elevator.SOFT_LIMIT_MARGIN_INCHES;
  private double forwardLimit = Elevator.NOMINAL_TRAVEL_INCHES - Elevator.SOFT_LIMIT_MARGIN_INCHES;
  private boolean reverseLimitOn = true;
  private boolean forwardLimitOn = true;
  private Extras rightX = Extras.ZERO;
  private Extras leftX = Extras.ZERO;

  public TalonElevatorIO(Unit unit) {
    right = new TalonFX(unit.rightMotorId());
    left = new TalonFX(unit.leftMotorId());
    followRight = new Follower(right.getDeviceID(), MotorAlignmentValue.Opposed);
    followLeft = new Follower(left.getDeviceID(), MotorAlignmentValue.Opposed);
    configure(right, InvertedValue.CounterClockwise_Positive, unit.rightRotationsPerInch(), true);
    configure(left, InvertedValue.Clockwise_Positive, unit.leftRotationsPerInch(), false);
    right.setPosition(0);
    left.setPosition(0);
    rightSignals = signalsOf(right);
    leftSignals = signalsOf(left);
    rightRefSlope = right.getClosedLoopReferenceSlope();
    leftRefSlope = left.getClosedLoopReferenceSlope();
    rightControlMode = right.getControlMode();
    leftControlMode = left.getControlMode();
    rightFaults = faultsOf(right);
    leftFaults = faultsOf(left);
    BaseStatusSignal.setUpdateFrequencyForAll(LOG_HZ, rightSignals);
    BaseStatusSignal.setUpdateFrequencyForAll(LOG_HZ, leftSignals);
    BaseStatusSignal.setUpdateFrequencyForAll(
        LOG_HZ, rightRefSlope, leftRefSlope, rightControlMode, leftControlMode);
    BaseStatusSignal.setUpdateFrequencyForAll(
        FOLLOW_HZ, right.getMotorVoltage(), right.getDutyCycle(),
        left.getMotorVoltage(), left.getDutyCycle());
    for (List<NamedFault> list : List.of(rightFaults, leftFaults)) {
      for (NamedFault f : list) {
        BaseStatusSignal.setUpdateFrequencyForAll(LOG_HZ, f.now(), f.sticky());
      }
    }
    right.clearStickyFaults();
    left.clearStickyFaults();
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

  /** One fault by CTRE's name: now and sticky. */
  private record NamedFault(String name, StatusSignal<Boolean> now, StatusSignal<Boolean> sticky) {}

  private static NamedFault fault(
      TalonFX m, String name, Function<TalonFX, StatusSignal<Boolean>> now,
      Function<TalonFX, StatusSignal<Boolean>> sticky) {
    return new NamedFault(name, now.apply(m), sticky.apply(m));
  }

  /** Every fault a TalonFX reports (Phoenix 6 26.3), so a fault bit can be named in the log. */
  private static List<NamedFault> faultsOf(TalonFX m) {
    List<NamedFault> f = new ArrayList<>();
    f.add(fault(m, "Hardware", TalonFX::getFault_Hardware, TalonFX::getStickyFault_Hardware));
    f.add(fault(m, "ProcTemp", TalonFX::getFault_ProcTemp, TalonFX::getStickyFault_ProcTemp));
    f.add(fault(m, "DeviceTemp", TalonFX::getFault_DeviceTemp, TalonFX::getStickyFault_DeviceTemp));
    f.add(fault(m, "Undervoltage", TalonFX::getFault_Undervoltage, TalonFX::getStickyFault_Undervoltage));
    f.add(fault(m, "BootDuringEnable", TalonFX::getFault_BootDuringEnable, TalonFX::getStickyFault_BootDuringEnable));
    f.add(fault(m, "UnlicensedFeatureInUse", TalonFX::getFault_UnlicensedFeatureInUse, TalonFX::getStickyFault_UnlicensedFeatureInUse));
    f.add(fault(m, "BridgeBrownout", TalonFX::getFault_BridgeBrownout, TalonFX::getStickyFault_BridgeBrownout));
    f.add(fault(m, "RemoteSensorReset", TalonFX::getFault_RemoteSensorReset, TalonFX::getStickyFault_RemoteSensorReset));
    f.add(fault(m, "MissingDifferentialFX", TalonFX::getFault_MissingDifferentialFX, TalonFX::getStickyFault_MissingDifferentialFX));
    f.add(fault(m, "RemoteSensorPosOverflow", TalonFX::getFault_RemoteSensorPosOverflow, TalonFX::getStickyFault_RemoteSensorPosOverflow));
    f.add(fault(m, "OverSupplyV", TalonFX::getFault_OverSupplyV, TalonFX::getStickyFault_OverSupplyV));
    f.add(fault(m, "UnstableSupplyV", TalonFX::getFault_UnstableSupplyV, TalonFX::getStickyFault_UnstableSupplyV));
    f.add(fault(m, "ReverseHardLimit", TalonFX::getFault_ReverseHardLimit, TalonFX::getStickyFault_ReverseHardLimit));
    f.add(fault(m, "ForwardHardLimit", TalonFX::getFault_ForwardHardLimit, TalonFX::getStickyFault_ForwardHardLimit));
    f.add(fault(m, "ReverseSoftLimit", TalonFX::getFault_ReverseSoftLimit, TalonFX::getStickyFault_ReverseSoftLimit));
    f.add(fault(m, "ForwardSoftLimit", TalonFX::getFault_ForwardSoftLimit, TalonFX::getStickyFault_ForwardSoftLimit));
    f.add(fault(m, "MissingSoftLimitRemote", TalonFX::getFault_MissingSoftLimitRemote, TalonFX::getStickyFault_MissingSoftLimitRemote));
    f.add(fault(m, "MissingHardLimitRemote", TalonFX::getFault_MissingHardLimitRemote, TalonFX::getStickyFault_MissingHardLimitRemote));
    f.add(fault(m, "RemoteSensorDataInvalid", TalonFX::getFault_RemoteSensorDataInvalid, TalonFX::getStickyFault_RemoteSensorDataInvalid));
    f.add(fault(m, "FusedSensorOutOfSync", TalonFX::getFault_FusedSensorOutOfSync, TalonFX::getStickyFault_FusedSensorOutOfSync));
    f.add(fault(m, "StatorCurrLimit", TalonFX::getFault_StatorCurrLimit, TalonFX::getStickyFault_StatorCurrLimit));
    f.add(fault(m, "SupplyCurrLimit", TalonFX::getFault_SupplyCurrLimit, TalonFX::getStickyFault_SupplyCurrLimit));
    f.add(fault(m, "UsingFusedCANcoderWhileUnlicensed", TalonFX::getFault_UsingFusedCANcoderWhileUnlicensed, TalonFX::getStickyFault_UsingFusedCANcoderWhileUnlicensed));
    f.add(fault(m, "RotorFault1", TalonFX::getFault_RotorFault1, TalonFX::getStickyFault_RotorFault1));
    f.add(fault(m, "RotorFault2", TalonFX::getFault_RotorFault2, TalonFX::getStickyFault_RotorFault2));
    f.add(fault(m, "StaticBrakeDisabled", TalonFX::getFault_StaticBrakeDisabled, TalonFX::getStickyFault_StaticBrakeDisabled));
    return f;
  }

  private static String names(List<NamedFault> faults, boolean sticky) {
    StringBuilder b = new StringBuilder();
    for (NamedFault f : faults) {
      if (Boolean.TRUE.equals((sticky ? f.sticky() : f.now()).getValue())) {
        if (b.length() > 0) b.append(',');
        b.append(f.name());
      }
    }
    return b.toString();
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
    gains = g;
    configureLeader(leader());
  }

  private boolean alone() {
    return drive == Drive.RIGHT_ALONE || drive == Drive.LEFT_ALONE;
  }

  private TalonFX leader() {
    return drive == Drive.LEFT_LEADS || drive == Drive.LEFT_ALONE ? left : right;
  }

  private TalonFX other() {
    return leader() == right ? left : right;
  }

  /** What the motor that does not lead does: follow the leader, or coast (alone). */
  private ControlRequest otherRequest() {
    if (alone()) return coast;
    return leader() == right ? followRight : followLeft;
  }

  /** The leader's gains (feedforward doubled when it drives alone), Motion Magic and soft limits. */
  private void configureLeader(TalonFX m) {
    Gains g = gains;
    if (alone()) {
      double k = ALONE_FEEDFORWARD_SCALE;
      g = new Gains(g.kS() * k, g.kG() * k, g.kV(), g.kA() * k, g.kP(), g.kD(),
          g.cruiseInPerSec(), g.accelInPerSec2(), g.jerkInPerSec3());
    }
    m.getConfigurator().apply(slot0(g), CONFIG_TIMEOUT_S);
    m.getConfigurator().apply(motionMagic(g), CONFIG_TIMEOUT_S);
    m.getConfigurator().apply(softLimits(), CONFIG_TIMEOUT_S);
  }

  private SoftwareLimitSwitchConfigs softLimits() {
    var s = new SoftwareLimitSwitchConfigs();
    s.ReverseSoftLimitThreshold = reverseLimit;
    s.ReverseSoftLimitEnable = reverseLimitOn;
    s.ForwardSoftLimitThreshold = forwardLimit;
    s.ForwardSoftLimitEnable = forwardLimitOn;
    return s;
  }

  @Override
  public void setDrive(Drive d) {
    if (d == drive) return;
    right.setControl(neutral);
    left.setControl(neutral);
    drive = d;
    // The non-leader must never stop following because of a soft limit of its own.
    other().getConfigurator().apply(new SoftwareLimitSwitchConfigs(), CONFIG_TIMEOUT_S);
    configureLeader(leader());
    // So the sticky faults of each test belong to that test.
    right.clearStickyFaults();
    left.clearStickyFaults();
  }

  @Override
  public void switchRequestOnly(Drive d) {
    if (d == drive) return;
    right.setControl(neutral);
    left.setControl(neutral);
    drive = d; // requests only: no config writes, no sticky clear
  }

  @Override
  public void rewriteConfigs() {
    right.setControl(neutral);
    left.setControl(neutral);
    other().getConfigurator().apply(new SoftwareLimitSwitchConfigs(), CONFIG_TIMEOUT_S);
    configureLeader(leader());
    right.clearStickyFaults();
    left.clearStickyFaults();
  }

  @Override
  public Drive drive() {
    return drive;
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
    reverseLimit = reverseInches;
    reverseLimitOn = reverseOn;
    forwardLimit = forwardInches;
    forwardLimitOn = forwardOn;
    // The leader's only: the follower must never stop following on its own.
    leader().getConfigurator().apply(softLimits(), CONFIG_TIMEOUT_S);
  }

  @Override
  public void moveTo(double inches) {
    leader().setControl(move.withPosition(inches));
    other().setControl(otherRequest());
  }

  @Override
  public void setVolts(double volts) {
    leader().setControl(voltsReq.withOutput(volts));
    other().setControl(otherRequest());
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
    BaseStatusSignal.refreshAll(rightRefSlope, leftRefSlope, rightControlMode, leftControlMode);
    for (List<NamedFault> list : List.of(rightFaults, leftFaults)) {
      for (NamedFault f : list) {
        BaseStatusSignal.refreshAll(f.now(), f.sticky());
      }
    }
    rightX = extrasOf(rightSignals);
    leftX = extrasOf(leftSignals);
  }

  private boolean leftLeads() {
    return leader() == left;
  }

  private boolean profiling() {
    var mode = leftLeads() ? leftControlMode : rightControlMode;
    return mode.getValue() == ControlModeValue.MotionMagicVoltage;
  }

  private Extras leaderX() {
    return leftLeads() ? leftX : rightX;
  }

  @Override
  public double alignedSideDiffInches() {
    return BaseStatusSignal.getLatencyCompensatedValueAsDouble(rightSignals[0], rightSignals[3])
        - BaseStatusSignal.getLatencyCompensatedValueAsDouble(leftSignals[0], leftSignals[3]);
  }

  @Override
  public String faultNames(boolean rightMotor) {
    return names(rightMotor ? rightFaults : leftFaults, false);
  }

  @Override
  public String stickyFaultNames(boolean rightMotor) {
    return names(rightMotor ? rightFaults : leftFaults, true);
  }

  @Override public Extras rightExtras() { return rightX; }
  @Override public Extras leftExtras() { return leftX; }
  @Override public double rightInches() { return rightSignals[0].getValueAsDouble(); }
  @Override public double leftInches() { return leftSignals[0].getValueAsDouble(); }
  @Override public double velocityInPerSec() { return rightSignals[3].getValueAsDouble(); }
  @Override public double profileInches() { return profiling() ? leaderX().closedLoopReference() : Double.NaN; }
  // Phoenix 6 closed-loop error is reference - position; ours is position - reference (the leader's).
  @Override public double profileErrorInches() { return profiling() ? -leaderX().closedLoopError() : Double.NaN; }
  @Override public double profileVelocityInPerSec() {
    return profiling() ? (leftLeads() ? leftRefSlope : rightRefSlope).getValueAsDouble() : Double.NaN;
  }
  @Override public double rightVolts() { return rightSignals[1].getValueAsDouble(); }
  @Override public double leftVolts() { return leftSignals[1].getValueAsDouble(); }
  @Override public double rightStatorAmps() { return rightSignals[2].getValueAsDouble(); }
  @Override public double leftStatorAmps() { return leftSignals[2].getValueAsDouble(); }
}
