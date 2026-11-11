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
      // Moves only while enabled. Left stick Y on pad 0 jogs it at 1 in/s (forward = up).
      // NetworkTables control (elevator.followNetworkTables()) is not the default for now.
      edu.wpi.first.wpilibj.XboxController pad = new edu.wpi.first.wpilibj.XboxController(0);
      // Single-motor test: A = both motors, B = right only, X = left only (back to both on disable).
      new edu.wpi.first.wpilibj2.command.button.JoystickButton(pad, 1)
          .onTrue(edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> elevator.setDrive(Elevator.Drive.BOTH)));
      new edu.wpi.first.wpilibj2.command.button.JoystickButton(pad, 2)
          .onTrue(edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> elevator.setDrive(Elevator.Drive.RIGHT_ONLY)));
      new edu.wpi.first.wpilibj2.command.button.JoystickButton(pad, 3)
          .onTrue(edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> elevator.setDrive(Elevator.Drive.LEFT_ONLY)));
      elevator.setDefaultCommand(elevator.jog(() -> -pad.getLeftY()));
    } else {
      elevator = null;
      DriverStation.reportWarning("Unknown unit " + unit.serial() + ": elevator not built", false);
    }
  }
}
