package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/** Starts the robot program, or the idle program when built with -Pprogram=idle. */
public final class Main {
  private Main() {}

  public static void main(String... args) {
    if ("idle".equals(BuildConstants.PROGRAM)) {
      RobotBase.startRobot(IdleRobot::new);
    } else {
      RobotBase.startRobot(Robot::new);
    }
  }
}
