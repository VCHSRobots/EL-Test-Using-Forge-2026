package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * What AdvantageScope shows of the elevator, under {@code /Forge/Elevator/}: a Mechanism2d at
 * {@code Mechanism}, the carriage as a robot-relative Pose3d array at {@code Components} (one entry,
 * in the order of {@code assets/Robot_Forge/config.json}), and {@code RobotPose}, always the
 * origin. Put RobotPose on AdvantageScope's Robot row and Components on a Component row under it;
 * Components on the Robot row makes the whole frame rise with the carriage.
 *
 * <p>Display only: nothing here reads back into control.
 */
public class ElevatorView {
  public static final String TABLE = "/Forge/Elevator";
  private static final double METERS_PER_INCH = 0.0254;

  /**
   * Where the carriage model's origin sits, robot-relative, with the carriage at the bottom stop.
   * A stand-in for the 3D view only; the carriage's place along the rails is known to be off. It is
   * the negative of the component's {@code zeroedPosition} in config.json: change them together.
   */
  public static final Translation3d CARRIAGE_ORIGIN_AT_BOTTOM = new Translation3d(0.0, 0.0, 0.1);

  private final Mechanism2d mechanism;
  private final MechanismLigament2d carriage;
  private final SendableBuilderImpl mechanismBuilder = new SendableBuilderImpl();
  private final StructArrayPublisher<Pose3d> components;
  private final StructPublisher<Pose3d> robotPose;

  public ElevatorView(double travelInches) {
    double travelMeters = travelInches * METERS_PER_INCH;
    mechanism = new Mechanism2d(0.6, travelMeters + 0.4);
    mechanism
        .getRoot("base", 0.3, 0.1)
        .append(new MechanismLigament2d("travel", travelMeters, 90, 4, new Color8Bit(80, 80, 80)));
    carriage =
        mechanism
            .getRoot("carriage", 0.3, 0.1)
            .append(new MechanismLigament2d("height", 0.05, 90, 10, new Color8Bit(255, 160, 0)));
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    mechanismBuilder.setTable(nt.getTable(TABLE + "/Mechanism"));
    SendableRegistry.publish(mechanism, mechanismBuilder);
    mechanismBuilder.startListeners();
    components = nt.getStructArrayTopic(TABLE + "/Components", Pose3d.struct).publish();
    robotPose = nt.getStructTopic(TABLE + "/RobotPose", Pose3d.struct).publish();
  }

  /** Publish the carriage at this height above the bottom stop, in inches. */
  public void update(double heightInches) {
    double heightMeters = Math.max(heightInches, 0.0) * METERS_PER_INCH;
    carriage.setLength(heightMeters + 0.05);
    mechanismBuilder.update();
    robotPose.set(Pose3d.kZero);
    components.set(
        new Pose3d[] {
          new Pose3d(
              CARRIAGE_ORIGIN_AT_BOTTOM.plus(new Translation3d(0.0, 0.0, heightMeters)),
              Rotation3d.kZero)
        });
  }
}
