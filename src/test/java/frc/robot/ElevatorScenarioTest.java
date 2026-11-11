package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.SimElevatorIO;
import frc.robot.subsystems.Elevator;
import org.junit.jupiter.api.Test;

class ElevatorScenarioTest {
  private static void setTarget(double inches) {
    NetworkTableInstance.getDefault().getEntry("/Elevator/Control/TargetInches").setDouble(inches);
  }

  private static Elevator build() {
    Elevator e = new Elevator(new SimElevatorIO());
    e.setDefaultCommand(e.followNetworkTables());
    return e;
  }

  @Test
  void goesUpFiveInchesAtOneInchPerSecondAndBackDown() {
    try (Scenario scenario = Scenario.start()) {
      Elevator e = build();
      scenario.run(0.2);
      scenario.enable();
      scenario.run(0.1);
      // First let it settle at the bottom margin, then ask for 5 in.
      scenario.run(1.0);
      double start = e.setpointInches();
      setTarget(5.0);
      scenario.run(2.0);
      // After 2 s the setpoint has moved 2 in (1 in/s), not jumped to 5.
      assertEquals(start + 2.0, e.setpointInches(), 0.05);
      assertEquals(e.setpointInches(), e.positionInches(), 0.5);
      scenario.run(4.0);
      assertEquals(5.0, e.positionInches(), 0.2);
      assertFalse(e.faulted());
      setTarget(0.0);
      scenario.run(6.0);
      assertEquals(Elevator.STOP_MARGIN_INCHES, e.positionInches(), 0.2);
      assertFalse(e.faulted());
    }
  }

  @Test
  void theTargetIsClampedToTheTravel() {
    try (Scenario scenario = Scenario.start()) {
      Elevator e = build();
      scenario.enable();
      setTarget(50.0);
      scenario.run(0.5);
      assertEquals(Elevator.TRAVEL_INCHES - Elevator.STOP_MARGIN_INCHES, e.targetInches(), 1e-9);
    }
  }

  @Test
  void nothingMovesWhileDisabled() {
    try (Scenario scenario = Scenario.start()) {
      Elevator e = build();
      setTarget(5.0);
      scenario.run(2.0);
      assertEquals(0.0, e.positionInches(), 1e-3);
      assertTrue(e.setpointInches() < 0.01);
    }
  }
}
