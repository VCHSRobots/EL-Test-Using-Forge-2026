package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SimElevatorIO;
import org.junit.jupiter.api.Test;

class AutoHomeTest {
  private static void stick(double axis1) {
    DriverStationSim.setJoystickAxisCount(0, 4);
    DriverStationSim.setJoystickAxis(0, 1, axis1);
    DriverStationSim.notifyNewData();
  }

  /** The carriage is really 3 in up while the robot believes it is at the bottom (a wrong zero). */
  @Test
  void homingFindsTheRealBottomFromAWrongZeroAndHandsBackToTheStick() {
    try (Scenario scenario = Scenario.start()) {
      SimElevatorIO io = new SimElevatorIO();
      Elevator e = new Elevator(io);
      e.setDefaultCommand(e.followStick());
      io.placeCarriage(3.0);
      stick(0.0);
      scenario.run(0.2); // disabled: the robot takes 3 in to be zero
      scenario.enable();
      scenario.run(0.2);
      double startedAt = io.physicalInches(); // about 3 in (the loop sags a little)
      assertEquals(3.0, startedAt, 0.6);

      Command home = e.autoHome();
      home.schedule();
      scenario.run(0.1);
      assertTrue(e.isHoming());
      scenario.run(10.0);

      assertFalse(home.isScheduled());
      assertEquals(2.0, e.homeStatus(), 0.0); // found the bottom
      assertEquals(0.0, io.physicalInches(), 0.1); // really on the stop
      assertEquals(0.0, e.positionInches(), 0.1); // and reading zero there
      assertEquals(-startedAt, e.homeFoundInches(), 0.3); // it was that far below the old zero
      assertFalse(e.faulted());

      // The stick works again from the new zero.
      stick(-0.5);
      scenario.run(5.0);
      assertEquals(4.0, e.positionInches(), 0.25);
      assertEquals(4.0, io.physicalInches(), 0.25);
    }
  }

  @Test
  void homingAtTheBottomAlreadyFindsZeroToo() {
    try (Scenario scenario = Scenario.start()) {
      SimElevatorIO io = new SimElevatorIO();
      Elevator e = new Elevator(io);
      e.setDefaultCommand(e.followStick());
      stick(0.0);
      scenario.run(0.2);
      scenario.enable();
      scenario.run(0.2);
      e.autoHome().schedule();
      scenario.run(3.0);
      assertEquals(2.0, e.homeStatus(), 0.0);
      assertEquals(0.0, e.homeFoundInches(), 0.1);
    }
  }

  @Test
  void disablingDuringHomingStopsItWithNoZeroChange() {
    try (Scenario scenario = Scenario.start()) {
      SimElevatorIO io = new SimElevatorIO();
      Elevator e = new Elevator(io);
      e.setDefaultCommand(e.followStick());
      io.placeCarriage(6.0);
      stick(0.0);
      scenario.run(0.2);
      scenario.enable();
      e.autoHome().schedule();
      scenario.run(0.3);
      scenario.disable();
      scenario.run(0.2);
      assertEquals(4.0, e.homeStatus(), 0.0);
      assertFalse(e.isHoming());
    }
  }
}
