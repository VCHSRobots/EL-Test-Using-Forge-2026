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
  void stickSetsSpeedWithACurveAndTheTriggerScalesIt() {
    try (Scenario sc = Scenario.start()) {
      Elevator e = ranged(sc);
      press(sc, XboxController.Button.kA, 1.5);
      // Half stick forward (negative Y = up): curve(0.4737 after the deadband) of 30 in/s.
      pad(-0.5, 0, 0);
      sc.run(0.3);
      assertEquals("velocity", e.mode());
      double p0 = e.positionInches();
      sc.run(0.1);
      double half = (e.positionInches() - p0) / 0.1;
      double x = (0.5 - Elevator.PAD_DEADBAND) / (1 - Elevator.PAD_DEADBAND);
      double expect = (0.3 * x + 0.7 * x * x * x) * Elevator.PAD_MAX_IN_PER_SEC;
      assertEquals(expect, half, 0.1 * expect + 0.3);
      // Release: it stops, then holds with no drift.
      pad(0, 0, 0);
      sc.run(1.0);
      double h = e.positionInches();
      sc.run(1.0);
      assertEquals(h, e.positionInches(), 0.02);
      // Full stick with the trigger fully pressed: SLOW_FRACTION of full speed.
      press(sc, XboxController.Button.kA, 1.5);
      pad(-1.0, 1.0, 0);
      sc.run(0.3);
      p0 = e.positionInches();
      sc.run(0.2);
      double slow = (e.positionInches() - p0) / 0.2;
      double want = Elevator.SLOW_FRACTION * Elevator.PAD_MAX_IN_PER_SEC;
      assertEquals(want, slow, 0.15 * want + 0.3);
      assertFalse(e.faulted());
    }
  }

  @Test
  void fullStickReachesFullSpeedAndGlidesToTheEnds() {
    try (Scenario sc = Scenario.start()) {
      Elevator e = ranged(sc);
      press(sc, XboxController.Button.kA, 1.5);
      pad(-1.0, 0, 0);
      double maxV = 0;
      double maxPos = 0;
      for (int i = 0; i < 100; i++) {
        sc.run(0.02);
        maxV = Math.max(maxV, e.velocityInPerSec());
        maxPos = Math.max(maxPos, e.positionInches());
      }
      assertEquals(Elevator.PAD_MAX_IN_PER_SEC, maxV, 0.12 * Elevator.PAD_MAX_IN_PER_SEC);
      // Held on the stick to the top: stops at the usable top, never past it, no fault.
      assertTrue(maxPos <= e.highInches() + 0.05, "overshot the top: " + maxPos);
      assertEquals(e.highInches(), e.positionInches(), 0.2);
      sc.run(1.0);
      assertEquals(e.highInches(), e.positionInches(), 0.2);
      // And the same to the bottom.
      pad(1.0, 0, 0);
      double minPos = 99;
      for (int i = 0; i < 100; i++) {
        sc.run(0.02);
        minPos = Math.min(minPos, e.positionInches());
      }
      assertTrue(minPos >= e.lowInches() - 0.05, "overshot the bottom: " + minPos);
      assertEquals(e.lowInches(), e.positionInches(), 0.2);
      assertFalse(e.faulted());
    }
  }

  @Test
  void speedChangesNeverExceedTheAccelerationLimit() {
    try (Scenario sc = Scenario.start()) {
      Elevator e = ranged(sc);
      press(sc, XboxController.Button.kA, 1.5);
      pad(-1.0, 0, 0);
      double prev = e.velocityInPerSec();
      double worst = 0;
      for (int i = 0; i < 40; i++) {
        sc.run(0.02);
        worst = Math.max(worst, Math.abs(e.velocityInPerSec() - prev) / 0.02);
        prev = e.velocityInPerSec();
      }
      pad(0, 0, 0);
      for (int i = 0; i < 50; i++) {
        sc.run(0.02);
        worst = Math.max(worst, Math.abs(e.velocityInPerSec() - prev) / 0.02);
        prev = e.velocityInPerSec();
      }
      // Step of the stick = a ramp at the learned 700 in/s^2 (a little slack for the 20 ms sampling).
      assertTrue(worst < 1.25 * e.tuning().get("AccelInPerSec2"), "peak accel " + worst);
    }
  }

  @Test
  void aSecondPresetPressInsideTheLockoutIsIgnored() {
    try (Scenario sc = Scenario.start()) {
      Elevator e = ranged(sc);
      press(sc, XboxController.Button.kY, 0.05);
      press(sc, XboxController.Button.kA, 0.05); // 0.11 s after Y: ignored
      sc.run(1.5);
      assertEquals(e.highInches(), e.positionInches(), 0.05);
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
