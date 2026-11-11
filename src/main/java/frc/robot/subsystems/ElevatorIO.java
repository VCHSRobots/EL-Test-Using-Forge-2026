package frc.robot.subsystems;

/**
 * What the Elevator needs from its two motors, in motor rotations where positive means UP for
 * both (the directions are handled inside the implementations). {@link TalonElevatorIO} is the
 * real hardware; {@link SimElevatorIO} is plain Java physics for simulation and tests.
 */
public interface ElevatorIO {
  /** Command both motors to this position (motor rotations, up positive) with a position loop. */
  void setPosition(double rotations);

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
}
