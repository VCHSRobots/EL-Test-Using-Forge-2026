package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.forge.Identity;

/** The robot program. Subsystems live in RobotContainer; this class only runs the scheduler. */
@Logged
public class Robot extends TimedRobot {
  final RobotContainer container;

  public Robot() {
    container = new RobotContainer(Identity.start());
    Epilogue.bind(this); // logs every @Logged field, through RobotContainer to each subsystem
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
