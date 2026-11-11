package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SimElevatorIO;
import org.junit.jupiter.api.Test;

/** The single-motor mode asks only the chosen motor to drive, and resets to both on disable. */
class SingleMotorTest {
  /** Sim physics that also records which motors were last asked to drive. */
  static class Recording extends SimElevatorIO {
    boolean right;
    boolean left;

    boolean follower;

    @Override
    public void setLeaderFollower(double rotations) {
      follower = true;
      right = true;
      left = true;
      setPosition(rotations);
    }

    @Override
    public void setPosition(double rotations, boolean useRight, boolean useLeft) {
      right = useRight;
      left = useLeft;
      setPosition(rotations);
    }
  }

  @Test
  void onlyTheChosenMotorDrivesAndDisableResetsToBoth() {
    try (Scenario scenario = Scenario.start()) {
      Recording io = new Recording();
      Elevator e = new Elevator(io);
      e.setDefaultCommand(e.followNetworkTables());
      scenario.enable();
      scenario.run(0.2);
      assertTrue(io.follower && io.right && io.left);

      e.setDrive(Elevator.Drive.BOTH);
      io.follower = false;
      scenario.run(0.1);
      assertFalse(io.follower);
      assertTrue(io.right && io.left);

      e.setDrive(Elevator.Drive.RIGHT_ONLY);
      scenario.run(0.1);
      assertTrue(io.right);
      assertFalse(io.left);

      e.setDrive(Elevator.Drive.LEFT_ONLY);
      scenario.run(0.1);
      assertFalse(io.right);
      assertTrue(io.left);

      scenario.disable();
      scenario.run(0.1);
      assertEquals(Elevator.Drive.LEADER_FOLLOWER, e.drive());
    }
  }
}
