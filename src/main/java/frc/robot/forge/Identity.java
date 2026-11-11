package frc.robot.forge;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.BuildConstants;
import frc.robot.hardware.KnownUnits;
import frc.robot.hardware.Unit;

/**
 * What this program is and which unit it runs on, published at startup to NetworkTables under
 * /Forge/ and into the WPILOG file's metadata, so every run says what produced it. The Forge
 * harness reads these topics; nothing here depends on Forge.
 */
public final class Identity {
  public static final String TABLE = "Forge";

  private Identity() {}

  /** Starts logging, reads the controller's serial, publishes the identity, returns the unit. */
  public static Unit start() {
    DataLogManager.start(); // WPILOG under /home/lvuser/logs, or /U/logs with a USB stick
    DriverStation.startDataLog(DataLogManager.getLog());
    String serial = RobotBase.isReal() ? RobotController.getSerialNumber() : "simulation";
    Unit unit = KnownUnits.forSerial(serial);
    publish(NetworkTableInstance.getDefault(), serial, unit);
    record(DataLogManager.getLog(), serial, unit);
    if (!unit.known()) {
      DriverStation.reportWarning("Unknown controller serial " + serial + ": no hardware map", false);
    }
    return unit;
  }

  /** The identity topics: /Forge/Build/*, /Forge/Serial and /Forge/Unit. */
  public static void publish(NetworkTableInstance nt, String serial, Unit unit) {
    NetworkTable build = nt.getTable(TABLE).getSubTable("Build");
    build.getEntry("Repository").setString(BuildConstants.REPOSITORY);
    build.getEntry("Branch").setString(BuildConstants.BRANCH);
    build.getEntry("Commit").setString(BuildConstants.COMMIT);
    build.getEntry("Dirty").setBoolean(BuildConstants.DIRTY);
    build.getEntry("BuiltAt").setString(BuildConstants.BUILT_AT);
    build.getEntry("Builder").setString(BuildConstants.BUILDER);
    build.getEntry("Program").setString(BuildConstants.PROGRAM);
    nt.getTable(TABLE).getEntry("Serial").setString(serial);
    nt.getTable(TABLE).getEntry("Unit").setString(unit.name());
  }

  /** The same identity as one JSON entry in the log, with the JSON also as its metadata. */
  public static void record(DataLog log, String serial, Unit unit) {
    String json = json(serial, unit);
    new StringLogEntry(log, "/Forge/Identity", json).append(json);
  }

  public static String json(String serial, Unit unit) {
    return "{"
        + field("repository", BuildConstants.REPOSITORY) + ","
        + field("branch", BuildConstants.BRANCH) + ","
        + field("commit", BuildConstants.COMMIT) + ","
        + "\"dirty\":" + BuildConstants.DIRTY + ","
        + field("built_at", BuildConstants.BUILT_AT) + ","
        + field("builder", BuildConstants.BUILDER) + ","
        + field("program", BuildConstants.PROGRAM) + ","
        + field("serial", serial) + ","
        + field("unit", unit.name())
        + "}";
  }

  private static String field(String name, String value) {
    return "\"" + name + "\":\"" + value.replace("\\", "\\\\").replace("\"", "\\\"") + "\"";
  }
}
