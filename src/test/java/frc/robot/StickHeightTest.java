package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SimElevatorIO;
import org.junit.jupiter.api.Test;

class StickHeightTest {
  private static void stick(double axis1) {
    DriverStationSim.setJoystickAxisCount(0, 4);
    DriverStationSim.setJoystickAxis(0, 1, axis1);
    DriverStationSim.notifyNewData();
  }

  @Test
  void stickSetsTheHeightFromZeroToEightInches() {
    try (Scenario scenario = Scenario.start()) {
      Elevator e = new Elevator(new SimElevatorIO());
      e.setDefaultCommand(e.followStick());
      stick(0.0);
      scenario.run(0.2);
      scenario.enable();
      stick(0.0);
      scenario.run(1.0);
      assertEquals(0.0, e.positionInches(), 0.1);
      stick(-1.0); // full forward
      scenario.run(7.0);
      assertEquals(8.0, e.positionInches(), 0.25);
      assertEquals(8.0, e.targetInches(), 1e-9);
      stick(-0.5);
      scenario.run(5.0);
      assertEquals(4.0, e.positionInches(), 0.25);
      stick(1.0); // pulled back: never below the bottom
      scenario.run(6.0);
      assertEquals(0.0, e.targetInches(), 1e-9);
      assertEquals(0.0, e.positionInches(), 0.25);
      assertFalse(e.faulted());
    }
  }

  @Test
  void theSetpointNeverMovesFasterThanTheStickRate() {
    try (Scenario scenario = Scenario.start()) {
      Elevator e = new Elevator(new SimElevatorIO());
      e.setDefaultCommand(e.followStick());
      scenario.run(0.2);
      scenario.enable();
      stick(-1.0);
      scenario.run(1.0);
      assertEquals(Elevator.STICK_SPEED_IN_PER_SEC * 1.0, e.setpointInches(), 0.15);
    }
  }
}
