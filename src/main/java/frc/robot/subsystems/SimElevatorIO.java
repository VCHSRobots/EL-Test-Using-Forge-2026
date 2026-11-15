package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.hardware.KnownUnits;
import frc.robot.hardware.Unit;

/**
 * Plain-Java stand-in for the two Talons and the stand: a 1 kHz copy of what the Talon does
 * (Motion Magic trapezoid profile, kS/kG/kV/kA feedforward, kP/kD, soft limits, peak volts and a
 * current limit) driving a physics model of two Kraken X44 on one shaft, with gravity, Coulomb
 * friction and inelastic hard stops at the TRUE travel. Deterministic, so scenario tests can run
 * it faster than real time (the CTRE simulator cannot).
 *
 * <p>Each motor has its own gearing from the {@link Unit} (the stand: right 15T, left 16T pinion,
 * both to 46T), and each "Talon" runs its own loop with its own kV, like {@link TalonElevatorIO}.
 *
 * <p>The physical constants are the stand's (15 Nov 2026): carriage 3.959 lb by CAD, travel 9.375
 * in. The friction, chain mass and rotor inertia are estimates, to be replaced by measured values.
 */
public class SimElevatorIO implements ElevatorIO {
  private static final double DT = 0.02;
  private static final int SUBSTEPS = 20; // 1 kHz, like the Talon's loop
  private static final double G = 9.81;
  private static final double M_PER_IN = 0.0254;

  /** Carriage, 3.959 lb by CAD (without the chains). Gravity acts on this. */
  public static final double CARRIAGE_KG = 3.959 * 0.45359;
  /** The two #25 chain loops move with the carriage but balance under gravity (estimate). */
  public static final double CHAIN_KG = 0.28;
  /** One Kraken X44 rotor's inertia (ESTIMATE; the characterization's kA will tell). */
  public static final double ROTOR_KG_M2 = 1.5e-5;
  /** Coulomb friction, expressed as the volts per motor that overcome it (EL21 suggests ~0.13 V). */
  public static final double FRICTION_VOLTS = 0.12;

  private static final DCMotor MOTOR = DCMotor.getKrakenX44(1);
  private static final int RIGHT = 0;
  private static final int LEFT = 1;
  /** Motor radians per inch of carriage travel, right and left. */
  private final double[] radPerIn = new double[2];
  /** Newtons on the carriage per newton-metre of motor torque, right and left. */
  private final double[] nPerNm = new double[2];
  /** Each Talon's kV multiplier (the right's is 16/15 on the stand). */
  private final double[] kvScale = new double[2];
  private final double effectiveKg;
  private final double frictionN;

  private enum Mode { NEUTRAL, VOLTS, PROFILE }

  // --- the stand ---
  private double trueTravelInches = Elevator.NOMINAL_TRAVEL_INCHES;
  private double posIn; // above the real bottom stop
  private double velInPerSec;
  private double zeroOffsetIn; // physical height that reads as 0
  // --- the "Talon" ---
  private Mode mode = Mode.NEUTRAL;
  private double voltsCmd;
  private double goalIn;
  private TrapezoidProfile.State ref = new TrapezoidProfile.State();
  private double refAccel;
  private Gains gains = Elevator.DEFAULT_GAINS;
  private TrapezoidProfile profile = profileOf(gains);
  private double peakVolts = Elevator.DEFAULT_PEAK_VOLTS;
  private double statorAmps = Elevator.DEFAULT_STATOR_AMPS;
  private double revLimit = Elevator.SOFT_LIMIT_MARGIN_INCHES;
  private double fwdLimit = Elevator.NOMINAL_TRAVEL_INCHES - Elevator.SOFT_LIMIT_MARGIN_INCHES;
  private boolean revOn = true;
  private boolean fwdOn = true;
  // --- last outputs, right and left ---
  private final double[] appliedVolts = new double[2];
  private final double[] amps = new double[2];

  /** A simulated stand geared like the real one. */
  public SimElevatorIO() {
    this(KnownUnits.SIMULATION);
  }

  public SimElevatorIO(Unit unit) {
    radPerIn[RIGHT] = unit.rightRotationsPerInch() * 2 * Math.PI;
    radPerIn[LEFT] = unit.leftRotationsPerInch() * 2 * Math.PI;
    kvScale[RIGHT] = unit.rightKvScale();
    kvScale[LEFT] = 1.0;
    double inertia = 0;
    double friction = 0;
    for (int k = 0; k < 2; k++) {
      nPerNm[k] = radPerIn[k] / M_PER_IN;
      inertia += ROTOR_KG_M2 * nPerNm[k] * nPerNm[k];
      friction += MOTOR.KtNMPerAmp * (FRICTION_VOLTS / MOTOR.rOhms) * nPerNm[k];
    }
    effectiveKg = CARRIAGE_KG + CHAIN_KG + inertia;
    frictionN = friction;
  }

  private static TrapezoidProfile profileOf(Gains g) {
    return new TrapezoidProfile(new TrapezoidProfile.Constraints(g.cruiseInPerSec(), g.accelInPerSec2()));
  }

  /** Test hook: put the carriage physically at this height (inches above the real bottom stop). */
  public void placeCarriage(double inches) {
    posIn = inches;
    velInPerSec = 0;
  }

  /** Test hook: the true height above the real bottom stop, in inches. */
  public double physicalInches() {
    return posIn;
  }

  /** Test hook: the true stop-to-stop travel of this simulated stand. */
  public void setTrueTravel(double inches) {
    trueTravelInches = inches;
  }

  public double trueTravel() {
    return trueTravelInches;
  }

  /** Test hook: true speed, in/s. */
  public double physicalVelocity() {
    return velInPerSec;
  }

  private double readInches() {
    return posIn - zeroOffsetIn;
  }

  @Override
  public void applyGains(Gains g) {
    gains = g;
    profile = profileOf(g);
  }

  @Override
  public void applyCaps(double peak, double stator) {
    peakVolts = peak;
    statorAmps = stator;
  }

  @Override
  public void setSoftLimits(double reverseInches, boolean reverseOn, double forwardInches, boolean forwardOn) {
    revLimit = reverseInches;
    revOn = reverseOn;
    fwdLimit = forwardInches;
    fwdOn = forwardOn;
  }

  @Override
  public void moveTo(double inches) {
    if (mode != Mode.PROFILE) {
      // Like the Talon: the profile starts from where the carriage is and how fast it moves.
      ref = new TrapezoidProfile.State(readInches(), velInPerSec);
      refAccel = 0;
    }
    mode = Mode.PROFILE;
    goalIn = inches;
  }

  @Override
  public void setVolts(double volts) {
    mode = Mode.VOLTS;
    voltsCmd = volts;
  }

  @Override
  public void neutral() {
    mode = Mode.NEUTRAL;
  }

  @Override
  public void setPositionInches(double inches) {
    zeroOffsetIn = posIn - inches;
    if (mode == Mode.PROFILE) {
      ref = new TrapezoidProfile.State(inches, velInPerSec);
    }
  }

  @Override
  public void update() {
    double dt = DT / SUBSTEPS;
    for (int i = 0; i < SUBSTEPS; i++) {
      step(dt);
    }
  }

  private void step(double dt) {
    double base = 0; // volts both Talons share (everything but the kV term)
    if (mode == Mode.PROFILE) {
      // Both Talons start from the same state with the same limits, so one profile stands for both.
      double oldVel = ref.velocity;
      ref = profile.calculate(dt, ref, new TrapezoidProfile.State(goalIn, 0));
      refAccel = (ref.velocity - oldVel) / dt;
      Gains g = gains;
      base = g.kG()
          + g.kS() * Math.signum(ref.velocity)
          + g.kA() * refAccel
          + g.kP() * (ref.position - readInches())
          + g.kD() * (ref.velocity - velInPerSec);
    }
    double reading = readInches();
    double drive = 0; // newtons up, both motors
    for (int k = 0; k < 2; k++) {
      double v =
          switch (mode) {
            case VOLTS -> voltsCmd;
            case PROFILE -> base + gains.kV() * kvScale[k] * ref.velocity;
            default -> 0.0; // brake mode: the windings are shorted
          };
      // Soft limits: the Talon goes neutral (brake) in the blocked direction.
      if ((fwdOn && reading >= fwdLimit && v > 0) || (revOn && reading <= revLimit && v < 0)) {
        v = 0.0;
      }
      v = MathUtil.clamp(v, -peakVolts, peakVolts);
      // Motor current, limited like the stator limit (the Talon cuts the voltage to hold it).
      double omega = velInPerSec * radPerIn[k];
      double i = (v - omega / MOTOR.KvRadPerSecPerVolt) / MOTOR.rOhms;
      i = MathUtil.clamp(i, -statorAmps, statorAmps);
      appliedVolts[k] = i * MOTOR.rOhms + omega / MOTOR.KvRadPerSecPerVolt;
      amps[k] = i;
      drive += MOTOR.KtNMPerAmp * i * nPerNm[k];
    }

    double net = drive - CARRIAGE_KG * G;
    if (Math.abs(velInPerSec) < 1e-3) {
      // Static friction holds it unless the net force beats it.
      if (Math.abs(net) <= frictionN) {
        net = 0;
        velInPerSec = 0;
      } else {
        net -= Math.signum(net) * frictionN;
      }
    } else {
      net -= Math.signum(velInPerSec) * frictionN;
    }
    double accelInPerSec2 = net / effectiveKg / M_PER_IN;
    double newVel = velInPerSec + accelInPerSec2 * dt;
    // Kinetic friction never reverses the motion within a step.
    if (velInPerSec != 0 && Math.signum(newVel) != Math.signum(velInPerSec)
        && Math.abs(drive - CARRIAGE_KG * G) <= frictionN) {
      newVel = 0;
    }
    velInPerSec = newVel;
    posIn += velInPerSec * dt;
    // Hard stops: thick metal, inelastic.
    if (posIn <= 0 && velInPerSec <= 0) {
      posIn = 0;
      velInPerSec = 0;
    } else if (posIn >= trueTravelInches && velInPerSec >= 0) {
      posIn = trueTravelInches;
      velInPerSec = 0;
    }
  }

  @Override public double rightInches() { return readInches(); }
  @Override public double leftInches() { return readInches(); }
  @Override public double velocityInPerSec() { return velInPerSec; }
  @Override public double profileInches() { return mode == Mode.PROFILE ? ref.position : Double.NaN; }
  @Override public double profileVelocityInPerSec() { return mode == Mode.PROFILE ? ref.velocity : Double.NaN; }
  @Override public double rightVolts() { return appliedVolts[RIGHT]; }
  @Override public double leftVolts() { return appliedVolts[LEFT]; }
  @Override public double rightStatorAmps() { return amps[RIGHT]; }
  @Override public double leftStatorAmps() { return amps[LEFT]; }
}
