package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.forge.Identity;

/**
 * The idle program: it reports its identity and does nothing in any mode. It constructs no
 * motor controller, so enabling it moves nothing. Forge leaves it on the robot between sessions.
 */
public class IdleRobot extends TimedRobot {
  public IdleRobot() {
    Identity.start();
  }

  // Empty on purpose: TimedRobot prints "Override me!" for each mode method left out.
  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationPeriodic() {}
}
