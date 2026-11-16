package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.hardware.Unit;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PowerLog;
import frc.robot.subsystems.SimElevatorIO;
import frc.robot.subsystems.TalonElevatorIO;

/** The subsystems of this unit, built from its hardware map, and the controls bound to them. */
@Logged
public class RobotContainer {
  final Unit unit;
  /** Null on a unit with no hardware map, so no motor is ever built for an unknown robot. */
  final Elevator elevator;
  /** The PDH's currents and voltage, logged every loop; null on an unknown unit. */
  final PowerLog power;

  public RobotContainer(Unit unit) {
    this.unit = unit;
    if (unit.known()) {
      power = new PowerLog();
      elevator = new Elevator(RobotBase.isSimulation() ? new SimElevatorIO(unit) : new TalonElevatorIO(unit));
      // Teleop: the stick on port 0, axis 1 sets the height over the measured range.
      elevator.setDefaultCommand(elevator.followStick());
      // The first teleop enable after the program starts finds both stops (auto-range).
      new Trigger(() -> DriverStation.isEnabled() && DriverStation.isTeleop() && elevator.needsRange())
          .onTrue(elevator.autoRange());
      Joystick stick = new Joystick(Elevator.STICK_PORT);
      // Button 1 (trigger / A): run auto-range again.
      new JoystickButton(stick, 1).onTrue(elevator.autoRange());
      // Button 2 (B): start the dance; press again to stop it and return to the stick.
      new JoystickButton(stick, 2).toggleOnTrue(elevator.dance());
    } else {
      elevator = null;
      power = null;
      DriverStation.reportWarning("Unknown unit " + unit.serial() + ": elevator not built", false);
    }
  }

  /** Autonomous: the dance (auto-range first if the stops are not known yet), then park. */
  public Command autonomousCommand() {
    return elevator == null ? Commands.none() : elevator.dance().andThen(elevator.park());
  }
}
