package frc.robot.subsystems;

/**
 * What the Elevator needs from its two motors. Everything is in INCHES of carriage travel (up
 * positive) and volts; the implementations convert to motor rotations. {@link TalonElevatorIO} is
 * the real hardware (each motor runs its own loop with its own gearing); {@link SimElevatorIO} is plain
 * Java physics for simulation and tests.
 */
public interface ElevatorIO {
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

  default Extras rightExtras() {
    return Extras.ZERO;
  }

  default Extras leftExtras() {
    return Extras.ZERO;
  }
}
