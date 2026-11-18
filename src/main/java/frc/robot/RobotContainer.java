package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
      DoubleSupplier slow = pad::getRightTriggerAxis;
      DoubleSupplier stickUp = () -> -pad.getLeftY();
      elevator.setDefaultCommand(elevator.driveVelocity(stickUp, slow));
      // The first teleop enable after the program starts finds both stops (auto-range).
      new Trigger(() -> DriverStation.isEnabled() && DriverStation.isTeleop() && elevator.needsRange())
          .onTrue(elevator.autoRange());
      // Presets, as fractions of the measured range; moving the stick takes over again.
      BooleanSupplier driverMoves = () -> Math.abs(pad.getLeftY()) > Elevator.PAD_DEADBAND;
      // A press within PRESET_LOCKOUT_SECONDS of the last accepted one is ignored, so quick
      // double presses do not reverse the profile.
      presetButton(pad, XboxController.Button.kA, 0.0, driverMoves);
      presetButton(pad, XboxController.Button.kB, Elevator.PRESET_B, driverMoves);
      presetButton(pad, XboxController.Button.kX, Elevator.PRESET_X, driverMoves);
      presetButton(pad, XboxController.Button.kY, 1.0, driverMoves);
      // Back: run auto-range again. Start: start the dance; press again to stop it.
      new JoystickButton(pad, XboxController.Button.kBack.value).onTrue(elevator.autoRange());
      new JoystickButton(pad, XboxController.Button.kStart.value).toggleOnTrue(elevator.dance());
    } else {
      elevator = null;
      power = null;
      DriverStation.reportWarning("Unknown unit " + unit.serial() + ": elevator not built", false);
    }
  }

  private double lastPreset = -1e9;

  private void presetButton(XboxController pad, XboxController.Button b, double fraction, BooleanSupplier override) {
    new JoystickButton(pad, b.value)
        .and(() -> Timer.getFPGATimestamp() - lastPreset >= Elevator.PRESET_LOCKOUT_SECONDS)
        .onTrue(
            Commands.runOnce(() -> lastPreset = Timer.getFPGATimestamp())
                .andThen(elevator.goToFraction(fraction, override, pad::getRightTriggerAxis)));
  }

  /** Autonomous: the dance (auto-range first if the stops are not known yet), then park. */
  public Command autonomousCommand() {
    return elevator == null ? Commands.none() : elevator.dance().andThen(elevator.park());
  }
}
