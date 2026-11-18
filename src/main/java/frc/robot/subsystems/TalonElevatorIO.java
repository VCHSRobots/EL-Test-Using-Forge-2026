package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
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
import edu.wpi.first.wpilibj.DriverStation;
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
  // Phoenix 6's own default for a config write; 50 ms timed out read-backs on the stand.
  private static final double CONFIG_TIMEOUT_S = 0.1;
  // A read-back is only a check: wait longer and try again before calling it failed.
  private static final double READBACK_TIMEOUT_S = 0.25;
  private static final int READBACK_TRIES = 3;
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
  private final DynamicMotionMagicVoltage moveDyn =
      new DynamicMotionMagicVoltage(0, 0, 0).withEnableFOC(false);
  private final MotionMagicVelocityVoltage moveVel =
      new MotionMagicVelocityVoltage(0).withSlot(1).withEnableFOC(false);
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
  // Each Talon's own breakdown of its closed-loop output: proportional, derivative, feedforward.
  private final StatusSignal<?>[] rightTerms;
  private final StatusSignal<?>[] leftTerms;
  // Every signal above, refreshed in one call so all readings belong to the same instant.
  private final BaseStatusSignal[] everySignal;
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
  private LoopTerms loop = LoopTerms.ZERO;
  // Every config write's answer since the last applyGains: "OK", or the failed writes by name.
  private final List<String> configFailures = new ArrayList<>();
  private int gainWrites;
  private Applied applied = new Applied(0, 0, 0, 0, 0, 0, false, "not read yet", 0);

  public TalonElevatorIO(Unit unit) {
    right = new TalonFX(unit.rightMotorId());
    left = new TalonFX(unit.leftMotorId());
    followRight = new Follower(right.getDeviceID(), MotorAlignmentValue.Opposed);
    followLeft = new Follower(left.getDeviceID(), MotorAlignmentValue.Opposed);
    check("right startup config",
        configure(right, InvertedValue.CounterClockwise_Positive, unit.rightRotationsPerInch(), true));
    check("left startup config",
        configure(left, InvertedValue.Clockwise_Positive, unit.leftRotationsPerInch(), false));
    readBack();
    right.setPosition(0);
    left.setPosition(0);
    rightSignals = signalsOf(right);
    leftSignals = signalsOf(left);
    rightRefSlope = right.getClosedLoopReferenceSlope();
    leftRefSlope = left.getClosedLoopReferenceSlope();
    rightControlMode = right.getControlMode();
    leftControlMode = left.getControlMode();
    rightTerms = termsOf(right);
    leftTerms = termsOf(left);
    rightFaults = faultsOf(right);
    leftFaults = faultsOf(left);
    BaseStatusSignal.setUpdateFrequencyForAll(LOG_HZ, rightSignals);
    BaseStatusSignal.setUpdateFrequencyForAll(LOG_HZ, leftSignals);
    BaseStatusSignal.setUpdateFrequencyForAll(
        LOG_HZ, rightRefSlope, leftRefSlope, rightControlMode, leftControlMode);
    BaseStatusSignal.setUpdateFrequencyForAll(LOG_HZ, rightTerms);
    BaseStatusSignal.setUpdateFrequencyForAll(LOG_HZ, leftTerms);
    BaseStatusSignal.setUpdateFrequencyForAll(
        FOLLOW_HZ, right.getMotorVoltage(), right.getDutyCycle(),
        left.getMotorVoltage(), left.getDutyCycle());
    for (List<NamedFault> list : List.of(rightFaults, leftFaults)) {
      for (NamedFault f : list) {
        BaseStatusSignal.setUpdateFrequencyForAll(LOG_HZ, f.now(), f.sticky());
      }
    }
    List<BaseStatusSignal> all = new ArrayList<>();
    all.addAll(List.of(rightSignals));
    all.addAll(List.of(leftSignals));
    all.addAll(List.of(rightTerms));
    all.addAll(List.of(leftTerms));
    all.addAll(List.of(rightRefSlope, leftRefSlope, rightControlMode, leftControlMode));
    for (List<NamedFault> list : List.of(rightFaults, leftFaults)) {
      for (NamedFault f : list) {
        all.add(f.now());
        all.add(f.sticky());
      }
    }
    everySignal = all.toArray(new BaseStatusSignal[0]);
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

  /** Proportional, derivative and feedforward parts of the closed-loop output, volts. */
  private static StatusSignal<?>[] termsOf(TalonFX m) {
    return new StatusSignal<?>[] {
      m.getClosedLoopProportionalOutput(), m.getClosedLoopDerivativeOutput(),
      m.getClosedLoopFeedForward()
    };
  }

  /** How old a reading is now: ms since the roboRIO received the frame that carried it. */
  private static double ageMs(BaseStatusSignal s, double nowSeconds) {
    return (nowSeconds - s.getAllTimestamps().getSystemTimestamp().getTime()) * 1000.0;
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
  private static StatusCode configure(
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
      cfg.Slot1 = slot1(Elevator.DEFAULT_GAINS);
      cfg.MotionMagic = motionMagic(Elevator.DEFAULT_GAINS);
      // Until the travel is measured, soft limits assume zero is the bottom and nominal travel.
      cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
          Elevator.NOMINAL_TRAVEL_INCHES - Elevator.SOFT_LIMIT_MARGIN_INCHES;
      cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Elevator.SOFT_LIMIT_MARGIN_INCHES;
    }
    return motor.getConfigurator().apply(cfg);
  }

  /**
   * Notes a config write's answer. A rejected write (an out-of-range value, a timeout) leaves the
   * Talon with its OLD settings, which nothing else would show, so it is reported and published.
   */
  private void check(String what, StatusCode code) {
    if (code.isOK()) return;
    configFailures.add(what + ": " + code.getName());
    DriverStation.reportWarning("Elevator config write refused: " + what + ": " + code.getName(), false);
  }

  /** Reads the leader's Motion Magic and slot 0 back from the Talon itself. Usually a few ms; blocks up to 1.5 s if the Talon does not answer. */
  private void readBack() {
    var mm = new MotionMagicConfigs();
    var s0 = new Slot0Configs();
    StatusCode mmCode = StatusCode.OK;
    StatusCode s0Code = StatusCode.OK;
    for (int i = 0; i < READBACK_TRIES; i++) {
      mmCode = leader().getConfigurator().refresh(mm, READBACK_TIMEOUT_S);
      if (mmCode.isOK()) break;
    }
    for (int i = 0; i < READBACK_TRIES; i++) {
      s0Code = leader().getConfigurator().refresh(s0, READBACK_TIMEOUT_S);
      if (s0Code.isOK()) break;
    }
    check("read back Motion Magic", mmCode);
    check("read back slot 0", s0Code);
    boolean ok = configFailures.isEmpty();
    applied = new Applied(mm.MotionMagicCruiseVelocity, mm.MotionMagicAcceleration, mm.MotionMagicJerk,
        s0.kA, s0.kP, s0.kD, ok, ok ? "OK" : String.join("; ", configFailures), gainWrites);
  }

  @Override
  public Applied applied() {
    return applied;
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

  /** Slot 1 (velocity mode): the same feedforward as slot 0, with a small P on speed error, no D. */
  private static Slot1Configs slot1(Gains g) {
    var s0 = slot0(g);
    var s = new Slot1Configs();
    s.GravityType = s0.GravityType;
    s.StaticFeedforwardSign = s0.StaticFeedforwardSign;
    s.kS = s0.kS;
    s.kG = s0.kG;
    s.kV = s0.kV;
    s.kA = s0.kA;
    s.kP = VELOCITY_KP;
    s.kD = 0;
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
    gainWrites++;
    configFailures.clear();
    configureLeader(leader());
    readBack();
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
    check("slot 0", m.getConfigurator().apply(slot0(g), CONFIG_TIMEOUT_S));
    check("slot 1", m.getConfigurator().apply(slot1(g), CONFIG_TIMEOUT_S));
    check("Motion Magic", m.getConfigurator().apply(motionMagic(g), CONFIG_TIMEOUT_S));
    check("soft limits", m.getConfigurator().apply(softLimits(), CONFIG_TIMEOUT_S));
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
    configFailures.clear();
    check("follower soft limits off",
        other().getConfigurator().apply(new SoftwareLimitSwitchConfigs(), CONFIG_TIMEOUT_S));
    configureLeader(leader());
    readBack(); // the new leader's settings
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
      String who = m == right ? "right " : "left ";
      check(who + "peak volts", m.getConfigurator().apply(v, CONFIG_TIMEOUT_S));
      check(who + "current limits", m.getConfigurator().apply(c, CONFIG_TIMEOUT_S));
    }
    // Caps are written right after the gains (Elevator.applyTuning): fold their answers in.
    boolean ok = configFailures.isEmpty();
    applied = new Applied(applied.cruiseInPerSec(), applied.accelInPerSec2(), applied.jerkInPerSec3(),
        applied.kA(), applied.kP(), applied.kD(), ok, ok ? "OK" : String.join("; ", configFailures), gainWrites);
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
  public void moveToDynamic(double inches, double cruiseInPerSec, double jerkInPerSec3) {
    leader().setControl(moveDyn.withPosition(inches).withVelocity(cruiseInPerSec)
        .withAcceleration(gains.accelInPerSec2()).withJerk(jerkInPerSec3));
    other().setControl(otherRequest());
  }

  @Override
  public void moveVelocity(double inPerSec) {
    leader().setControl(moveVel.withVelocity(inPerSec));
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
    BaseStatusSignal.refreshAll(everySignal); // one call: every reading from the same instant
    double now = Utils.getCurrentTimeSeconds();
    rightX = extrasOf(rightSignals);
    leftX = extrasOf(leftSignals);
    boolean l = leftLeads();
    StatusSignal<?>[] s = l ? leftSignals : rightSignals;
    StatusSignal<?>[] t = l ? leftTerms : rightTerms;
    StatusSignal<?> slope = l ? leftRefSlope : rightRefSlope;
    StatusSignal<?> followerVolts = (l ? rightSignals : leftSignals)[1];
    loop = new LoopTerms(
        t[0].getValueAsDouble(), t[1].getValueAsDouble(), t[2].getValueAsDouble(),
        s[12].getValueAsDouble(), s[10].getValueAsDouble(), s[11].getValueAsDouble(),
        slope.getValueAsDouble(), s[0].getValueAsDouble(), s[1].getValueAsDouble(),
        followerVolts.getValueAsDouble(),
        ageMs(t[0], now), ageMs(t[2], now), ageMs(s[12], now), ageMs(s[10], now),
        ageMs(s[11], now), ageMs(slope, now), ageMs(s[0], now), ageMs(s[1], now),
        ageMs(followerVolts, now));
  }

  private boolean leftLeads() {
    return leader() == left;
  }

  private boolean profiling() {
    var mode = leftLeads() ? leftControlMode : rightControlMode;
    return mode.getValue() == ControlModeValue.MotionMagicVoltage; // Dynamic Motion Magic reports as this too
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

  @Override public LoopTerms loopTerms() { return loop; }
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
