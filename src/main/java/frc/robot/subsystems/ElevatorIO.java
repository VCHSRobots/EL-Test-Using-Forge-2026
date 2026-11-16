package frc.robot.subsystems;

/**
 * What the Elevator needs from its two motors. Everything is in INCHES of carriage travel (up
 * positive) and volts; the implementations convert to motor rotations. {@link TalonElevatorIO} is
 * the real hardware (one loop in the leader, the other motor follows it); {@link SimElevatorIO} is
 * plain Java physics for simulation and tests.
 */
public interface ElevatorIO {
  /**
   * Which motor runs the loop. RIGHT_LEADS is normal (the left follows). The others are for
   * isolation tests only: LEFT_LEADS swaps the roles; RIGHT_ALONE / LEFT_ALONE drive with one
   * motor while the other coasts (no output at all), with the leader's kS, kG and kA doubled
   * because one motor does the pair's work.
   */
  enum Drive { RIGHT_LEADS, LEFT_LEADS, RIGHT_ALONE, LEFT_ALONE }

  /** Feedforward on a motor driving alone: kS, kG and kA times this (kV is per motor speed). */
  double ALONE_FEEDFORWARD_SCALE = 2.0;

  /**
   * Switches which motor leads. Puts both motors in neutral first and rewrites the leader's
   * configs (may block ~200 ms): call with the carriage at rest. No-op if already in that drive.
   */
  default void setDrive(Drive drive) {}

  /**
   * Trigger test: switches which motor leads (the requests each motor gets) with NO config
   * writes and no sticky-fault clear. The configs stay as the last setDrive/applyGains left them.
   */
  default void switchRequestOnly(Drive drive) {
    setDrive(drive);
  }

  /**
   * Trigger test: rewrites the leader's and the other motor's configs and clears sticky faults,
   * exactly as setDrive does, but keeps the current drive and its requests. Motors in neutral.
   */
  default void rewriteConfigs() {}

  default Drive drive() {
    return Drive.RIGHT_LEADS;
  }

  /**
   * Right minus left height, both read for the SAME instant (each latency-compensated to now);
   * near direction reversals this shows backlash between the two sides.
   */
  default double alignedSideDiffInches() {
    return rightInches() - leftInches();
  }

  /** The faults active now on one motor, by CTRE's names, comma-separated ("" for none). */
  default String faultNames(boolean rightMotor) {
    return "";
  }

  /** The sticky faults (set since the last clear) on one motor, by name. */
  default String stickyFaultNames(boolean rightMotor) {
    return "";
  }

  /**
   * Feedforward, feedback and profile settings, all in inch units: kS, kG volts; kV volts per
   * in/s; kA volts per in/s^2; kP volts per inch of error; kD volts per in/s of error; cruise in/s,
   * acceleration in/s^2, jerk in/s^3 (0 = no jerk limit).
   */
  record Gains(
      double kS,
      double kG,
      double kV,
      double kA,
      double kP,
      double kD,
      double cruiseInPerSec,
      double accelInPerSec2,
      double jerkInPerSec3) {}

  /** Writes gains and profile limits into the controllers. May block ~50 ms: call while disabled. */
  void applyGains(Gains gains);

  /** Writes the output caps (peak volts, stator and supply amps). May block: call while disabled. */
  void applyCaps(double peakVolts, double statorAmps);

  /** The controllers' own soft limits, in inches. May block ~50 ms per motor. */
  void setSoftLimits(double reverseInches, boolean reverseOn, double forwardInches, boolean forwardOn);

  /** Profiled move (Motion Magic) to this height; feedforward and PD run in the controller. */
  void moveTo(double inches);

  /** Open-loop volts on both motors (up positive). */
  void setVolts(double volts);

  /** Motors off (neutral output, brake mode). */
  void neutral();

  /** Tell the encoders the carriage is at this height now (used when a hard stop is found). */
  void setPositionInches(double inches);

  /** Call every loop: refresh readings (and step the physics, in simulation). */
  void update();

  double rightInches();

  double leftInches();

  /** Carriage speed from the right motor, in/s (up positive). */
  double velocityInPerSec();

  /** The profile's current position (Motion Magic reference), inches; NaN when not profiling. */
  double profileInches();

  /**
   * Position minus the profile's position, inches; NaN when not profiling. The default subtracts
   * the two readings; the Talon overrides it with its own closed-loop error, taken in one instant,
   * because its position and reference arrive in different CAN frames a few ms apart (at 25 in/s
   * that skew alone read as 0.2 in of error).
   */
  default double profileErrorInches() {
    double ref = profileInches();
    return Double.isNaN(ref) ? Double.NaN : rightInches() - ref;
  }

  /** The profile's current velocity, in/s; NaN when not profiling. */
  double profileVelocityInPerSec();

  double rightVolts();

  double leftVolts();

  double rightStatorAmps();

  double leftStatorAmps();

  /** Extra per-motor readings for logging only (all zero where an implementation has none). */
  record Extras(
      double velocityRps,
      double supplyAmps,
      double torqueAmps,
      double deviceTempC,
      double processorTempC,
      double duty,
      double supplyVolts,
      double closedLoopError,
      double closedLoopReference,
      double closedLoopOutput,
      double faultField,
      double stickyFaultField) {
    static final Extras ZERO = new Extras(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  /**
   * The leader's own breakdown of its closed-loop output, and how old each reading was when this
   * loop read it (ms since the roboRIO received its CAN frame). All read in one refresh, so a
   * term that disagrees with the others can be told from a term that is merely older. For
   * logging only (all zero where an implementation has none).
   */
  record LoopTerms(
      double proportional,
      double derivative,
      double feedForward,
      double output,
      double error,
      double reference,
      double referenceSlope,
      double position,
      double motorVolts,
      double followerVolts,
      double ageProportionalMs,
      double ageFeedForwardMs,
      double ageOutputMs,
      double ageErrorMs,
      double ageReferenceMs,
      double ageReferenceSlopeMs,
      double agePositionMs,
      double ageMotorVoltsMs,
      double ageFollowerVoltsMs) {
    static final LoopTerms ZERO =
        new LoopTerms(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  default LoopTerms loopTerms() {
    return LoopTerms.ZERO;
  }

  default Extras rightExtras() {
    return Extras.ZERO;
  }

  default Extras leftExtras() {
    return Extras.ZERO;
  }
}
