package frc.robot;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SimElevatorIO;

/** Settle time, overshoot, hold error and peaks of one profiled move on the simulated stand. */
record Metrics(double settleSeconds, double overshootInches, double holdErrorInches, double peakSpeed, double peakAmps) {
  /** Settled = within 0.02 in of the target from then on. */
  static Metrics move(Scenario sc, SimElevatorIO io, Elevator e, double target, double seconds) {
    double start = io.physicalInches();
    double dir = Math.signum(target - start);
    e.goTo(target).schedule();
    double settled = -1, over = 0, peakV = 0, peakA = 0;
    int n = (int) Math.round(seconds / Scenario.STEP_SECONDS);
    for (int i = 1; i <= n; i++) {
      sc.run(Scenario.STEP_SECONDS);
      double err = io.physicalInches() - target;
      over = Math.max(over, dir * err);
      peakV = Math.max(peakV, Math.abs(io.physicalVelocity()));
      peakA = Math.max(peakA, Math.abs(io.rightStatorAmps()));
      if (Math.abs(err) > 0.02) {
        settled = -1;
      } else if (settled < 0) {
        settled = i * Scenario.STEP_SECONDS;
      }
    }
    return new Metrics(settled < 0 ? 99 : settled, over, io.physicalInches() - target, peakV, peakA);
  }
}
