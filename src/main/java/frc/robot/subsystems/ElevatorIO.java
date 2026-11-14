package frc.robot.subsystems;

/**
 * What the Elevator needs from its two motors, in motor rotations where positive means UP for
 * both (the directions are handled inside the implementations). {@link TalonElevatorIO} is the
 * real hardware; {@link SimElevatorIO} is plain Java physics for simulation and tests.
 */
public interface ElevatorIO {
  /** Command both motors to this position (motor rotations, up positive) with a position loop. */
  void setPosition(double rotations);

  /**
   * Position loop on only the chosen motors; the other motor gets neutral output (it still turns
   * with the chain and is still read). Default for implementations with no single-motor mode: both.
   */
  default void setPosition(double rotations, boolean useRight, boolean useLeft) {
    setPosition(rotations);
  }

  /**
   * Right motor runs the position loop, left follows the right motor's output. Default for
   * implementations with no follower mode: both run their own loop.
   */
  default void setLeaderFollower(double rotations) {
    setPosition(rotations);
  }

  /** Take the carriage to be at the bottom now: both positions read 0. Default: nothing to do. */
  default void zero() {}

  /**
   * Homing: drive both motors DOWN open loop at this many volts (positive number), the left
   * following the right. Default: neutral (an implementation with no homing never moves).
   */
  default void homeDrive(double volts) {
    neutral();
  }

  /**
   * Allow the carriage to be driven below zero (the reverse soft limit off) while homing finds
   * the real bottom; true restores the limit. Default: nothing to do.
   */
  default void setReverseLimitEnabled(boolean enabled) {}

  /** Motors off (neutral output). */
  void neutral();

  /** Call every loop: refresh readings (and step the physics, in simulation). */
  void update();

  double rightRotations();

  double leftRotations();

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
