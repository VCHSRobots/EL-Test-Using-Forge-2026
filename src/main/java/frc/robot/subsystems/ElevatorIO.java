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
