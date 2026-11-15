package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SimElevatorIO;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import org.junit.jupiter.api.Test;

/** Writes build/sim-trace.csv: auto-range, then the standard moves at default and at goal tuning. */
class SimTraceTest {
  @Test
  void writeTrace() throws IOException {
    try (Scenario sc = Scenario.start();
        PrintWriter out = new PrintWriter(new FileWriter("build/sim-trace.csv"))) {
      out.println("t,phase,true_in,read_in,target_in,profile_in,volts,amps,vel");
      SimElevatorIO io = new SimElevatorIO();
      io.setTrueTravel(9.30);
      io.placeCarriage(2.0);
      io.setPositionInches(0);
      Elevator e = new Elevator(io);
      e.setDefaultCommand(e.hold());
      double[] t = {0};
      sc.run(0.2);
      sc.enable();
      e.autoRange().schedule();
      t[0] = log(sc, out, io, e, t[0], 6.0, "range");
      t[0] = log(sc, out, io, e, t[0], 0.5, "range");
      Command m = e.standardMoves();
      m.schedule();
      t[0] = log(sc, out, io, e, t[0], 6.5, "moves-default");
      sc.disable();
      e.tuning().set("CruiseInPerSec", 40);
      e.tuning().set("AccelInPerSec2", 250);
      e.tuning().set("PeakVolts", 12);
      e.tuning().set("StatorAmps", 40);
      t[0] = log(sc, out, io, e, t[0], 0.5, "disabled");
      sc.enable();
      e.standardMoves().schedule();
      t[0] = log(sc, out, io, e, t[0], 6.5, "moves-goal");
    }
  }

  private static double log(Scenario sc, PrintWriter out, SimElevatorIO io, Elevator e, double t, double secs, String phase) {
    int n = (int) Math.round(secs / Scenario.STEP_SECONDS);
    for (int i = 0; i < n; i++) {
      sc.run(Scenario.STEP_SECONDS);
      t += Scenario.STEP_SECONDS;
      out.printf("%.2f,%s,%.4f,%.4f,%.4f,%.4f,%.3f,%.2f,%.3f%n", t, phase, io.physicalInches(),
          e.positionInches(), e.targetInches(), io.profileInches(), io.rightVolts(), io.rightStatorAmps(), io.physicalVelocity());
    }
    return t;
  }
}
