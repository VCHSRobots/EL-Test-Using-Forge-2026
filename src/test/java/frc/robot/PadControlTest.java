package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.hardware.KnownUnits;
import frc.robot.subsystems.Elevator;
import org.junit.jupiter.api.Test;

/** The Xbox pad on port 0 driving the simulated elevator (nothing here touches a real robot). */
class PadControlTest {
  private static void pad(double leftY, double rightTrigger, int buttonMask) {
    DriverStationSim.setJoystickAxisCount(0, 6);
    DriverStationSim.setJoystickButtonCount(0, 10);
    DriverStationSim.setJoystickAxis(0, XboxController.Axis.kLeftY.value, leftY);
    DriverStationSim.setJoystickAxis(0, XboxController.Axis.kRightTrigger.value, rightTrigger);
    DriverStationSim.setJoystickButtons(0, buttonMask);
    DriverStationSim.notifyNewData();
  }

  private static int button(XboxController.Button b) {
    return 1 << (b.value - 1);
  }

  /** Enables a fresh container, lets auto-range finish, and lets go of everything. */
  private static Elevator ranged(Scenario sc) {
    RobotContainer rc = new RobotContainer(KnownUnits.SIMULATION);
    pad(0, 0, 0);
    sc.run(0.2);
    sc.enable();
    sc.run(8.0);
    assertTrue(rc.elevator.isRanged());
    return rc.elevator;
  }

  private static void press(Scenario sc, XboxController.Button b, double settle) {
    pad(0, 0, button(b));
    sc.run(0.06);
    pad(0, 0, 0);
    sc.run(settle);
  }

  @Test
  void buttonsGoToTheMinimumTheMaximumAndTheThirds() {
    try (Scenario sc = Scenario.start()) {
      Elevator e = ranged(sc);
      double lo = e.lowInches();
      double span = e.highInches() - lo;
      press(sc, XboxController.Button.kY, 1.5);
      assertEquals(e.highInches(), e.positionInches(), 0.05);
      press(sc, XboxController.Button.kB, 1.5);
      assertEquals(lo + 0.33 * span, e.positionInches(), 0.05);
      press(sc, XboxController.Button.kX, 1.5);
      assertEquals(lo + 0.66 * span, e.positionInches(), 0.05);
      press(sc, XboxController.Button.kA, 1.5);
      assertEquals(lo, e.positionInches(), 0.05);
      assertFalse(e.faulted());
    }
  }

  @Test
  void releasedStickHoldsWhereItIs() {
    try (Scenario sc = Scenario.start()) {
      Elevator e = ranged(sc);
      press(sc, XboxController.Button.kB, 1.5);
      double h = e.positionInches();
      sc.run(2.0);
      assertEquals(h, e.positionInches(), 0.02);
    }
  }

  @Test
  void stickSetsSpeedAndSlowModeIsFractionOfIt() {
    try (Scenario sc = Scenario.start()) {
      Elevator e = ranged(sc);
      press(sc, XboxController.Button.kA, 1.5);
      double cruise = e.tuning().get("CruiseInPerSec");
      // Half stick forward (negative Y = up): about half the default speed.
      pad(-0.5, 0, 0);
      sc.run(0.12);
      double p0 = e.positionInches();
      sc.run(0.1);
      double fast = (e.positionInches() - p0) / 0.1;
      assertEquals(0.5 * cruise * 0.99, fast, 0.1 * cruise);
      // Release: it stops and holds.
      pad(0, 0, 0);
      sc.run(1.5);
      double h = e.positionInches();
      sc.run(1.0);
      assertEquals(h, e.positionInches(), 0.02);
      // Full stick with the trigger held: SLOW_FRACTION of the default.
      press(sc, XboxController.Button.kA, 1.5);
      pad(-1.0, 1.0, 0);
      sc.run(0.2);
      p0 = e.positionInches();
      sc.run(0.2);
      double slow = (e.positionInches() - p0) / 0.2;
      assertEquals(Elevator.SLOW_FRACTION * cruise, slow, 0.15 * cruise * Elevator.SLOW_FRACTION + 0.5);
      // Pushed on to the top, it stops at the measured top and never faults.
      pad(-1.0, 0, 0);
      sc.run(3.0);
      assertEquals(e.highInches(), e.positionInches(), 0.05);
      // Stick down drives back to the bottom.
      pad(1.0, 0, 0);
      sc.run(3.0);
      assertEquals(e.lowInches(), e.positionInches(), 0.05);
      assertFalse(e.faulted());
    }
  }

  @Test
  void stickTakesOverFromAPreset() {
    try (Scenario sc = Scenario.start()) {
      Elevator e = ranged(sc);
      press(sc, XboxController.Button.kY, 1.5);
      pad(1.0, 0, 0);
      sc.run(0.5);
      assertEquals("velocity", e.mode());
      assertTrue(e.positionInches() < e.highInches() - 1.0);
    }
  }
}
