package frc.robot.hardware;

import java.util.List;

/** Every unit this code knows, one entry each, chosen by the controller's serial at startup. */
public final class KnownUnits {
  /** The simulated stand: geared like the real one, so simulation tests what the stand runs. */
  public static final Unit SIMULATION = new Unit("simulation", "simulation", true, 10, 11, 15, 16);

  /**
   * The elevator stand: roboRIO serial 031be686 (seen from the Driver Station harness), right motor
   * CAN 10 with a 15T pinion, left motor CAN 11 with a 16T pinion (both to 46T on the shaft; the
   * director, 15 Nov 2026; run EL27 measured the right turning 1.065 times the left, 16/15 = 1.067).
   */
  public static final Unit ELEVATOR_STAND = new Unit("elevator-stand", "031be686", true, 10, 11, 15, 16);

  public static final List<Unit> ALL = List.of(SIMULATION, ELEVATOR_STAND);

  private KnownUnits() {}

  /** The unit with this serial, or an unknown unit that names the serial. */
  public static Unit forSerial(String serial) {
    for (Unit unit : ALL) {
      if (unit.serial().equals(serial)) {
        return unit;
      }
    }
    return Unit.unknown(serial);
  }
}
