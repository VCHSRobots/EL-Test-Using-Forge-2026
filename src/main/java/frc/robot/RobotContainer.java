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
      // Moves only while enabled, only as NetworkTables /Elevator/Control/TargetInches says.
      elevator.setDefaultCommand(elevator.followNetworkTables());
    } else {
      elevator = null;
      DriverStation.reportWarning("Unknown unit " + unit.serial() + ": elevator not built", false);
    }
  }
}
