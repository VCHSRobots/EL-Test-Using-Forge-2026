package frc.robot.hardware;

/**
 * One physical unit's hardware map: what differs between two units running the same code.
 *
 * @param rightMotorId CAN ID of the right motor (seen from the back); spinning it counter-clockwise raises the carriage
 * @param leftMotorId CAN ID of the left motor; spinning it clockwise raises the carriage
 */
public record Unit(String name, String serial, boolean known, int rightMotorId, int leftMotorId) {
  /** A unit with no hardware map: the serial is not in KnownUnits, so no motor is built for it. */
  public static Unit unknown(String serial) {
    return new Unit("unknown", serial, false, -1, -1);
  }
}
