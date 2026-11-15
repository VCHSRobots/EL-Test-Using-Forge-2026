package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Values a test script (or a person in a dashboard) can change over NetworkTables without a
 * redeploy, under /Elevator/Tuning/. Each has a default and a range it is clamped to. The Elevator
 * writes changed values into the motor controllers only while the robot is DISABLED (the writes
 * block), so a change takes effect at the next enable.
 */
public final class Tuning {
  private record Item(DoubleEntry entry, double def, double min, double max) {}

  private final Map<String, Item> items = new LinkedHashMap<>();
  private final Map<String, Double> values = new LinkedHashMap<>();
  private final NetworkTable table;

  public Tuning(String tableName) {
    table = NetworkTableInstance.getDefault().getTable(tableName);
  }

  /** Declares a value; it is published at its default so it can be seen and written. */
  public Tuning add(String name, double def, double min, double max) {
    DoubleEntry e = table.getDoubleTopic(name).getEntry(def);
    e.set(def);
    items.put(name, new Item(e, def, min, max));
    values.put(name, def);
    return this;
  }

  /** Reads every value; true if any changed since the last call. */
  public boolean refresh() {
    boolean changed = false;
    for (var kv : items.entrySet()) {
      Item it = kv.getValue();
      double raw = it.entry.get();
      double v = Double.isFinite(raw) ? MathUtil.clamp(raw, it.min, it.max) : it.def;
      Double old = values.put(kv.getKey(), v);
      if (old == null || old != v) {
        changed = true;
      }
    }
    return changed;
  }

  /** The value last read (clamped to its range). */
  public double get(String name) {
    return values.get(name);
  }

  /** Test hook: write a value as a script would. */
  public void set(String name, double value) {
    items.get(name).entry.set(value);
  }
}
