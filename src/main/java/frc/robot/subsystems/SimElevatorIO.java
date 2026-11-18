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
 * both to 46T). Like {@link TalonElevatorIO}, one loop (the right "Talon") computes the volts and
 * both motors get them (the left follows), each motor's current set by its own back-EMF.
 *
 * <p>The physical constants are the stand's (15 Nov 2026): carriage 3.959 lb by CAD, travel 9.375
 * in. Gravity, friction, back-EMF and moving mass are set from the stand's characterization
 * (EL29, run cf3877f1, both motors at the same open-loop volts): kS 0.122 V, kG 0.162 V, kV 0.0643
 * V/(in/s), kA 0.0010 V/(in/s^2). Measured against the Kraken X44 data sheet and the CAD weight,
 * the stand needs 14% more volts to hold against gravity, 9% fewer volts per in/s, and 1.6x the
 * estimated moving mass (gears, shaft and sprockets add inertia).
 */
public class SimElevatorIO implements ElevatorIO {
  private static final double DT = 0.02;
  private static final int SUBSTEPS = 20; // 1 kHz, like the Talon's loop
  private static final double G = 9.81;
  private static final double M_PER_IN = 0.0254;

  /** Carriage, 3.959 lb by CAD (without the chains). Gravity acts on this. */
  public static final double CARRIAGE_KG = 3.959 * 0.45359;
  /** Coulomb friction, as the volts on both motors that overcome it (kS measured on EL29). */
  public static final double FRICTION_VOLTS = 0.12;
  /** Volts on both motors that hold the carriage against gravity (kG measured on EL29). */
  public static final double GRAVITY_VOLTS = 0.162;
  /** Volts on both motors per in/s of carriage speed (kV measured on EL29, same volts on both). */
  public static final double MEASURED_KV = 0.0643;
  /** Volts on both motors per in/s^2: EL29 measured 0.0010 (rough, from 1 V steps), but the kA sweep and EL44 confirm (17 Nov) moved best at 0.0016, so the model takes the heavier value. */
  public static final double MEASURED_KA = 0.0016;

  private static final DCMotor MOTOR = DCMotor.getKrakenX44(1);
  private static final int RIGHT = 0;
  private static final int LEFT = 1;
  /** Motor radians per inch of carriage travel, right and left. */
  private final double[] radPerIn = new double[2];
  /** Newtons on the carriage per newton-metre of motor torque, right and left. */
  private final double[] nPerNm = new double[2];
  private final double effectiveKg;
  private final double frictionN;
  private final double gravityN;
  /** Back-EMF per rad/s, per motor: the data sheet's, scaled so the sim's kV is the measured one. */
  private final double emfVoltsPerRadPerSec;

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
  private Drive drive = Drive.RIGHT_LEADS;
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
    // Newtons on the carriage per volt on both motors at stall, and the data sheet's kV.
    double newtonsPerVolt = 0;
    double sheetKv = 0;
    for (int k = 0; k < 2; k++) {
      nPerNm[k] = radPerIn[k] / M_PER_IN;
      double n = MOTOR.KtNMPerAmp / MOTOR.rOhms * nPerNm[k];
      newtonsPerVolt += n;
      sheetKv += n * radPerIn[k] / MOTOR.KvRadPerSecPerVolt;
    }
    sheetKv /= newtonsPerVolt;
    emfVoltsPerRadPerSec = MEASURED_KV / sheetKv / MOTOR.KvRadPerSecPerVolt;
    frictionN = FRICTION_VOLTS * newtonsPerVolt;
    gravityN = GRAVITY_VOLTS * newtonsPerVolt;
    effectiveKg = MEASURED_KA * newtonsPerVolt / M_PER_IN;
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
    gainWrites++;
  }

  private int gainWrites;

  /** The simulation accepts every write. Its profile has NO jerk limit (a trapezoid), whatever jerk says. */
  @Override
  public Applied applied() {
    Gains g = gains;
    return new Applied(g.cruiseInPerSec(), g.accelInPerSec2(), g.jerkInPerSec3(), g.kA(), g.kP(), g.kD(),
        true, "OK", gainWrites);
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
  public void setDrive(Drive d) {
    if (d != drive) {
      mode = Mode.NEUTRAL; // like the Talon: both motors neutral while the roles change
      drive = d;
    }
  }

  @Override
  public Drive drive() {
    return drive;
  }

  private boolean alone() {
    return drive == Drive.RIGHT_ALONE || drive == Drive.LEFT_ALONE;
  }

  /** The motor that coasts (drives nothing) when one drives alone, else -1. */
  private int coasting() {
    return drive == Drive.RIGHT_ALONE ? LEFT : drive == Drive.LEFT_ALONE ? RIGHT : -1;
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
    double loopVolts = 0; // the leader's (right Talon's) Motion Magic output; the left follows it
    if (mode == Mode.PROFILE) {
      double oldVel = ref.velocity;
      ref = profile.calculate(dt, ref, new TrapezoidProfile.State(goalIn, 0));
      refAccel = (ref.velocity - oldVel) / dt;
      Gains g = gains;
      double ff = alone() ? ALONE_FEEDFORWARD_SCALE : 1.0; // one motor does the pair's work
      loopVolts = ff * g.kG()
          + ff * g.kS() * Math.signum(ref.velocity)
          + g.kV() * ref.velocity
          + ff * g.kA() * refAccel
          + g.kP() * (ref.position - readInches())
          + g.kD() * (ref.velocity - velInPerSec);
    }
    double reading = readInches();
    double drive = 0; // newtons up, both motors
    // The leader's volts: its soft limits (neutral, brake, in the blocked direction) and peak volts.
    double v =
        switch (mode) {
          case VOLTS -> voltsCmd;
          case PROFILE -> loopVolts;
          default -> 0.0; // brake mode: the windings are shorted
        };
    if ((fwdOn && reading >= fwdLimit && v > 0) || (revOn && reading <= revLimit && v < 0)) {
      v = 0.0;
    }
    v = MathUtil.clamp(v, -peakVolts, peakVolts);
    // Both motors get the leader's volts (the left follows); each draws its own current.
    for (int k = 0; k < 2; k++) {
      if (k == coasting()) {
        // Coasting: no current, the terminals just show the back-EMF.
        appliedVolts[k] = 0;
        amps[k] = 0;
        continue;
      }
      // Motor current, limited like the stator limit (the Talon cuts the voltage to hold it).
      double omega = velInPerSec * radPerIn[k];
      double i = (v - omega * emfVoltsPerRadPerSec) / MOTOR.rOhms;
      i = MathUtil.clamp(i, -statorAmps, statorAmps);
      appliedVolts[k] = i * MOTOR.rOhms + omega * emfVoltsPerRadPerSec;
      amps[k] = i;
      drive += MOTOR.KtNMPerAmp * i * nPerNm[k];
    }

    double net = drive - gravityN;
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
        && Math.abs(drive - gravityN) <= frictionN) {
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
