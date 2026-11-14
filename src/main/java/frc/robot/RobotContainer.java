package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.hardware.Unit;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SimElevatorIO;
import frc.robot.subsystems.TalonElevatorIO;

/** The subsystems of this unit, built from its hardware map, and the controls bound to them. */
@Logged
public class RobotContainer {
  final Unit unit;
  /** Null on a unit with no hardware map, so no motor is ever built for an unknown robot. */
  final Elevator elevator;

  public RobotContainer(Unit unit) {
    this.unit = unit;
    if (unit.known()) {
      elevator = new Elevator(RobotBase.isSimulation() ? new SimElevatorIO() : new TalonElevatorIO(unit));
      // Moves only while enabled. The stick on port 0, axis 1 sets the height: released = bottom,
      // full forward = 8 in (see Elevator.followStick). Zero is wherever the carriage is while disabled.
      elevator.setDefaultCommand(elevator.followStick());
      // Button 1 (the trigger / A) on the stick: auto-home. It takes over, finds the bottom stop by
      // its stall, zeroes there, and hands control back to the stick (see Elevator.autoHome).
      new edu.wpi.first.wpilibj2.command.button.JoystickButton(
              new edu.wpi.first.wpilibj.Joystick(Elevator.STICK_PORT), 1)
          .onTrue(elevator.autoHome());
      // Single-motor test on a second pad (port 1), so a stick on port 0 never triggers it:
      // A = leader/follower (default), Y = both independent, B = right only, X = left only.
      edu.wpi.first.wpilibj.XboxController testPad = new edu.wpi.first.wpilibj.XboxController(1);
      new edu.wpi.first.wpilibj2.command.button.JoystickButton(testPad, 1)
          .onTrue(edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> elevator.setDrive(Elevator.Drive.LEADER_FOLLOWER)));
      new edu.wpi.first.wpilibj2.command.button.JoystickButton(testPad, 4)
          .onTrue(edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> elevator.setDrive(Elevator.Drive.BOTH)));
      new edu.wpi.first.wpilibj2.command.button.JoystickButton(testPad, 2)
          .onTrue(edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> elevator.setDrive(Elevator.Drive.RIGHT_ONLY)));
      new edu.wpi.first.wpilibj2.command.button.JoystickButton(testPad, 3)
          .onTrue(edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> elevator.setDrive(Elevator.Drive.LEFT_ONLY)));
    } else {
      elevator = null;
      DriverStation.reportWarning("Unknown unit " + unit.serial() + ": elevator not built", false);
    }
  }
}
