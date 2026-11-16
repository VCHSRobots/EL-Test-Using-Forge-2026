package frc.robot.hardware;

/**
 * One physical unit's hardware map: what differs between two units running the same code.
 *
 * @param rightMotorId CAN ID of the right motor (seen from the back); spinning it counter-clockwise raises the carriage
 * @param leftMotorId CAN ID of the left motor; spinning it clockwise raises the carriage
 * @param rightPinionTeeth teeth on the right motor's pinion (it drives the 46T shaft gear through an idler)
 * @param leftPinionTeeth teeth on the left motor's pinion
 */
public record Unit(
    String name, String serial, boolean known, int rightMotorId, int leftMotorId,
    int rightPinionTeeth, int leftPinionTeeth) {
  /** Teeth on each shaft gear (part CUS-001-068; "068" is a part number, not a tooth count). */
  public static final int SHAFT_GEAR_TEETH = 46;
  /** #25 chain (0.25 in pitch) on a 16T sprocket: inches of travel per shaft turn. */
  public static final double INCHES_PER_SHAFT_TURN = 16 * 0.25;

  /** A unit with no hardware map: the serial is not in KnownUnits, so no motor is built for it. */
  public static Unit unknown(String serial) {
    return new Unit("unknown", serial, false, -1, -1, 16, 16);
  }

  /** Right motor rotations per inch of carriage travel (an idler does not change the ratio). */
  public double rightRotationsPerInch() {
    return (double) SHAFT_GEAR_TEETH / rightPinionTeeth / INCHES_PER_SHAFT_TURN;
  }

  /** Left motor rotations per inch of carriage travel. */
  public double leftRotationsPerInch() {
    return (double) SHAFT_GEAR_TEETH / leftPinionTeeth / INCHES_PER_SHAFT_TURN;
  }
}
