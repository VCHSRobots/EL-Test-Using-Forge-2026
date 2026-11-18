package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
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
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
      // Teleop, Xbox pad on port 0: the left stick sets the carriage's speed (forward = up),
      // full deflection = the default cruise speed; the right trigger held = slow mode.
      XboxController pad = new XboxController(Elevator.STICK_PORT);
      BooleanSupplier slow = () -> pad.getRightTriggerAxis() > Elevator.SLOW_TRIGGER_THRESHOLD;
      DoubleSupplier stickUp = () -> -pad.getLeftY();
      elevator.setDefaultCommand(elevator.driveVelocity(stickUp, slow));
      // The first teleop enable after the program starts finds both stops (auto-range).
      new Trigger(() -> DriverStation.isEnabled() && DriverStation.isTeleop() && elevator.needsRange())
          .onTrue(elevator.autoRange());
      // Presets, as fractions of the measured range; moving the stick takes over again.
      BooleanSupplier driverMoves = () -> Math.abs(pad.getLeftY()) > Elevator.PAD_DEADBAND;
      new JoystickButton(pad, XboxController.Button.kA.value)
          .onTrue(elevator.goToFraction(0.0, driverMoves));
      new JoystickButton(pad, XboxController.Button.kB.value)
          .onTrue(elevator.goToFraction(Elevator.PRESET_B, driverMoves));
      new JoystickButton(pad, XboxController.Button.kX.value)
          .onTrue(elevator.goToFraction(Elevator.PRESET_X, driverMoves));
      new JoystickButton(pad, XboxController.Button.kY.value)
          .onTrue(elevator.goToFraction(1.0, driverMoves));
      // Back: run auto-range again. Start: start the dance; press again to stop it.
      new JoystickButton(pad, XboxController.Button.kBack.value).onTrue(elevator.autoRange());
      new JoystickButton(pad, XboxController.Button.kStart.value).toggleOnTrue(elevator.dance());
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
