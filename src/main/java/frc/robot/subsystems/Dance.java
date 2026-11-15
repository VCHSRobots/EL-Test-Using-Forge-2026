package frc.robot.subsystems;

import java.util.List;

/**
 * The dance: a fixed list of beats, played in a loop for as long as the dance lasts. Each beat
 * names a height as a FRACTION of the usable range (0 = the lowest commanded height, just above
 * the bottom stop; 1 = the highest, just below the top stop) and how long the beat lasts (the
 * move and the hold together). Heights are fractions so the dance fits whatever travel auto-range
 * measured, and it never asks for a height outside the soft limits, so it never touches a stop.
 * How fast it moves between beats comes from the Motion Magic limits in /Elevator/Tuning/.
 */
public final class Dance {
  public record Beat(double fraction, double seconds) {}

  /** One pass of the dance, about 20 s long. Edit freely: fractions 0..1, seconds > 0. */
  public static final List<Beat> BEATS =
      List.of(
          // Big bounces, bottom to top.
          new Beat(1.00, 1.0), new Beat(0.00, 1.0), new Beat(1.00, 1.0), new Beat(0.00, 1.0),
          // A staircase up ...
          new Beat(0.25, 0.6), new Beat(0.50, 0.6), new Beat(0.75, 0.6), new Beat(1.00, 0.8),
          // ... and down.
          new Beat(0.75, 0.6), new Beat(0.50, 0.6), new Beat(0.25, 0.6), new Beat(0.00, 0.8),
          // A shimmy in the middle.
          new Beat(0.50, 0.8), new Beat(0.58, 0.3), new Beat(0.42, 0.3), new Beat(0.58, 0.3),
          new Beat(0.42, 0.3), new Beat(0.58, 0.3), new Beat(0.42, 0.3), new Beat(0.50, 0.6),
          // Half bounces from the top.
          new Beat(1.00, 0.8), new Beat(0.50, 0.6), new Beat(1.00, 0.6), new Beat(0.50, 0.6),
          new Beat(1.00, 0.6),
          // A slow-looking hold at the top, then home.
          new Beat(1.00, 1.0), new Beat(0.00, 1.5));

  /** Seconds in one pass of {@link #BEATS}. */
  public static double passSeconds() {
    return BEATS.stream().mapToDouble(Beat::seconds).sum();
  }

  /** The beat playing at this many seconds into the dance (it loops). */
  public static int beatAt(double seconds) {
    double t = seconds % passSeconds();
    for (int i = 0; i < BEATS.size(); i++) {
      t -= BEATS.get(i).seconds();
      if (t < 0) {
        return i;
      }
    }
    return BEATS.size() - 1;
  }

  private Dance() {}
}
