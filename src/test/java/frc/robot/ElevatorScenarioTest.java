package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.hardware.KnownUnits;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.RangeStatus;
import frc.robot.subsystems.ElevatorIO;
import frc.robot.subsystems.SimElevatorIO;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

/** Scenario tests of the elevator on the simulated stand (nothing here touches a real robot). */
class ElevatorScenarioTest {
  private static void stick(double axis1) {
    DriverStationSim.setJoystickAxisCount(0, 4);
    DriverStationSim.setJoystickAxis(0, 1, axis1);
    DriverStationSim.notifyNewData();
  }

  private static void abort(boolean on) {
    NetworkTableInstance.getDefault().getEntry("/Forge/Abort").setBoolean(on);
  }

  @AfterEach
  void clear() {
    abort(false);
    NetworkTableInstance.getDefault().getEntry("/Elevator/Control/Routine").setString("");
  }

  /** Builds an elevator on a simulated stand and runs auto-range once from a wrong zero. */
  private static Elevator ranged(Scenario sc, SimElevatorIO io, double trueTravel) {
    io.setTrueTravel(trueTravel);
    io.placeCarriage(3.0);
    io.setPositionInches(0); // like a Talon at power-up: the robot thinks this is zero
    Elevator e = new Elevator(io);
    e.setDefaultCommand(e.hold());
    sc.run(0.2);
    sc.enable();
    e.autoRange().schedule();
    sc.run(8.0);
    return e;
  }

  @Test
  void autoRangeFindsBothStopsFromAWrongZero() {
    try (Scenario sc = Scenario.start()) {
      SimElevatorIO io = new SimElevatorIO();
      Elevator e = ranged(sc, io, 9.20);
      assertEquals(RangeStatus.DONE, e.rangeStatus());
      assertTrue(e.isRanged());
      assertEquals(-3.0, e.bottomFoundInches(), 0.05);
      assertEquals(9.20, e.travelInches(), 0.03);
      // Reading and reality agree, and it holds just below the top, off the stop.
      assertEquals(io.physicalInches(), e.positionInches(), 1e-6);
      assertEquals(e.highInches(), io.physicalInches(), 0.05);
      assertFalse(e.faulted());
    }
  }

  @Test
  void autoRangeRejectsATravelFarFromNominal() {
    try (Scenario sc = Scenario.start()) {
      SimElevatorIO io = new SimElevatorIO();
      Elevator e = ranged(sc, io, 11.0);
      assertEquals(RangeStatus.FAILED, e.rangeStatus());
      assertFalse(e.isRanged());
      assertFalse(e.needsRange()); // no automatic retry after a failure
      assertEquals(Elevator.NOMINAL_TRAVEL_INCHES, e.travelInches(), 1e-9);
    }
  }

  @Test
  void firstTeleopEnableRangesByItselfThenTheStickCoversTheMeasuredRange() {
    try (Scenario sc = Scenario.start()) {
      RobotContainer rc = new RobotContainer(KnownUnits.SIMULATION);
      Elevator e = rc.elevator;
      stick(0.0);
      sc.run(0.2);
      sc.enable();
      sc.run(8.0);
      assertEquals(RangeStatus.DONE, e.rangeStatus());
      assertEquals(e.lowInches(), e.positionInches(), 0.03); // stick released = bottom of range
      stick(-1.0);
      sc.run(2.0);
      assertEquals(e.highInches(), e.positionInches(), 0.03);
      stick(-0.5);
      sc.run(2.0);
      assertEquals((e.lowInches() + e.highInches()) / 2, e.positionInches(), 0.03);
      // A second enable does not range again.
      sc.disable();
      sc.run(0.2);
      sc.enable();
      sc.run(0.5);
      assertFalse(e.mode().startsWith("range"));
      assertFalse(e.faulted());
    }
  }

  @Test
  void goalTuningMovesTheFullRangeInHalfASecond() {
    try (Scenario sc = Scenario.start()) {
      SimElevatorIO io = new SimElevatorIO();
      Elevator e = ranged(sc, io, Elevator.NOMINAL_TRAVEL_INCHES);
      sc.disable();
      e.tuning().set("CruiseInPerSec", 40);
      e.tuning().set("AccelInPerSec2", 250);
      e.tuning().set("PeakVolts", 12);
      e.tuning().set("StatorAmps", 40);
      sc.run(0.1); // applied while disabled
      sc.enable();
      Command down = e.goTo(e.lowInches());
      down.schedule();
      sc.run(2.0);
      Metrics m = Metrics.move(sc, io, e, e.highInches(), 2.0);
      System.out.println("goal tuning, full move: " + m);
      assertTrue(m.settleSeconds() < 0.6, m.toString());
      assertTrue(m.overshootInches() < 0.05, m.toString());
      assertTrue(Math.abs(m.holdErrorInches()) < 0.02, m.toString());
      assertFalse(e.faulted());
    }
  }

  @Test
  void defaultTuningIsGentleAndAccurate() {
    try (Scenario sc = Scenario.start()) {
      SimElevatorIO io = new SimElevatorIO();
      Elevator e = ranged(sc, io, Elevator.NOMINAL_TRAVEL_INCHES);
      e.goTo(e.lowInches()).schedule();
      sc.run(3.0);
      Metrics m = Metrics.move(sc, io, e, e.highInches(), 3.0);
      System.out.println("default tuning, full move: " + m);
      assertTrue(m.settleSeconds() < 1.4, m.toString());
      assertTrue(Math.abs(m.holdErrorInches()) < 0.02, m.toString());
      assertTrue(m.peakSpeed() < 10.5, m.toString());
    }
  }

  @Test
  void abortCutsTheOutputUntilCleared() {
    try (Scenario sc = Scenario.start()) {
      SimElevatorIO io = new SimElevatorIO();
      Elevator e = ranged(sc, io, Elevator.NOMINAL_TRAVEL_INCHES);
      e.goTo(5.0).schedule();
      sc.run(2.0);
      abort(true);
      sc.run(0.5);
      assertTrue(e.aborted());
      assertEquals(0.0, io.rightVolts(), 1e-9);
      abort(false);
      sc.run(2.0);
      assertEquals(5.0, e.positionInches(), 0.03);
    }
  }

  @Test
  void aProfileTheMotorsCannotFollowLatchesTheLagFault() {
    try (Scenario sc = Scenario.start()) {
      SimElevatorIO io = new SimElevatorIO();
      Elevator e = ranged(sc, io, Elevator.NOMINAL_TRAVEL_INCHES);
      sc.disable();
      e.tuning().set("PeakVolts", 1.0);
      e.tuning().set("CruiseInPerSec", 40);
      e.tuning().set("AccelInPerSec2", 400);
      sc.run(0.1);
      sc.enable();
      e.goTo(e.lowInches()).schedule();
      sc.run(3.0);
      assertTrue(e.faulted());
      assertEquals(0.0, io.rightVolts(), 1e-9);
    }
  }

  @Test
  void theDanceStaysOffTheStopsAndEndsAtTheBottom() {
    try (Scenario sc = Scenario.start()) {
      RobotContainer rc = new RobotContainer(KnownUnits.SIMULATION);
      Elevator e = rc.elevator;
      e.tuning().set("DanceSeconds", 45);
      sc.run(0.2);
      sc.enableAutonomous();
      rc.autonomousCommand().schedule();
      double lowest = 99, highest = -99;
      sc.run(8.0); // auto-range first
      assertEquals(RangeStatus.DONE, e.rangeStatus());
      for (int i = 0; i < 50 * 50 && !e.mode().startsWith("park"); i++) {
        sc.run(0.02);
        lowest = Math.min(lowest, e.positionInches());
        highest = Math.max(highest, e.positionInches());
      }
      assertTrue(lowest > 0.1, "lowest " + lowest);
      assertTrue(highest < e.travelInches() - 0.1, "highest " + highest);
      // Then it parks: at the bottom of the range, motors off.
      sc.run(3.0);
      assertTrue(e.isParked());
      assertTrue(e.positionInches() <= e.lowInches() + 0.05, "at " + e.positionInches());
      assertFalse(e.faulted());
    }
  }

  @Test
  void aScriptCanRunTheStandardMovesByName() {
    try (Scenario sc = Scenario.start()) {
      SimElevatorIO io = new SimElevatorIO();
      Elevator e = ranged(sc, io, Elevator.NOMINAL_TRAVEL_INCHES);
      NetworkTableInstance.getDefault().getEntry("/Elevator/Control/Routine").setString("moves");
      sc.run(0.5 + 1.5);
      assertEquals(8.0, e.targetInches(), 1e-9);
      sc.run(5.0);
      assertEquals(1.0, e.positionInches(), 0.03);
      assertEquals(
          1, NetworkTableInstance.getDefault().getEntry("/Elevator/Status/RoutinesDone").getInteger(0));
    }
  }

  @Test
  void quasistaticAndDynamicStayInsideTheStops() {
    try (Scenario sc = Scenario.start()) {
      SimElevatorIO io = new SimElevatorIO();
      Elevator e = ranged(sc, io, Elevator.NOMINAL_TRAVEL_INCHES);
      double lowest = 99, highest = -99;
      Command q = e.quasistatic().andThen(e.dynamic());
      q.schedule();
      for (int i = 0; i < 60 * 50 && q.isScheduled(); i++) {
        sc.run(0.02);
        lowest = Math.min(lowest, io.physicalInches());
        highest = Math.max(highest, io.physicalInches());
      }
      assertFalse(q.isScheduled());
      assertTrue(lowest > 0.05, "lowest " + lowest);
      assertTrue(highest < e.travelInches() - 0.05, "highest " + highest);
      assertFalse(e.faulted());
    }
  }

  @Test
  void theLeftFollowsTheRightAndTheyShareTheHold() {
    try (Scenario sc = Scenario.start()) {
      SimElevatorIO io = new SimElevatorIO(); // right 15T, left 16T, like the stand
      Elevator e = ranged(sc, io, Elevator.NOMINAL_TRAVEL_INCHES);
      sc.disable();
      e.tuning().set("CruiseInPerSec", 40);
      e.tuning().set("AccelInPerSec2", 250);
      e.tuning().set("PeakVolts", 12);
      e.tuning().set("StatorAmps", 40);
      sc.run(0.1);
      sc.enable();
      e.goTo(e.lowInches()).schedule();
      sc.run(2.0);
      // Holding still: the same volts and no back-EMF, so the same current in both.
      assertEquals(io.rightStatorAmps(), io.leftStatorAmps(), 0.05);
      for (boolean up : new boolean[] {true, false}) {
        e.goTo(up ? e.highInches() : e.lowInches()).schedule();
        double worstGap = 0, peak = 0, worstVoltGap = 0, worstPerSpeed = 0;
        for (int i = 0; i < 75; i++) {
          sc.run(0.02);
          double r = io.rightStatorAmps(), l = io.leftStatorAmps();
          worstGap = Math.max(worstGap, Math.abs(r - l));
          peak = Math.max(peak, Math.max(Math.abs(r), Math.abs(l)));
          worstVoltGap = Math.max(worstVoltGap, Math.abs(io.rightVolts() - io.leftVolts()));
          double speed = Math.abs(io.physicalVelocity());
          if (speed > 5) {
            worstPerSpeed = Math.max(worstPerSpeed, Math.abs(r - l) / speed);
          }
        }
        System.out.printf(
            "fast move %s: peak %.2f A, worst right-left gap %.3f A (%.3f A per in/s), volts gap %.4f V%n",
            up ? "up" : "down", peak, worstGap, worstPerSpeed, worstVoltGap);
        // One loop: the follower gets the leader's volts (below the current limit).
        assertEquals(0.0, worstVoltGap, 1e-9);
        // The only difference is back-EMF: the right spins 16/15 as fast. About 0.1 A per in/s.
        assertTrue(worstPerSpeed < 0.15, "gap per in/s " + worstPerSpeed);
        assertEquals(up ? e.highInches() : e.lowInches(), io.physicalInches(), 0.03);
        assertEquals(io.rightStatorAmps(), io.leftStatorAmps(), 0.05);
      }
    }
  }

  @Test
  void parkEndsATestAtTheBottomWithTheMotorsOff() {
    try (Scenario sc = Scenario.start()) {
      SimElevatorIO io = new SimElevatorIO();
      Elevator e = ranged(sc, io, Elevator.NOMINAL_TRAVEL_INCHES);
      var routine = NetworkTableInstance.getDefault().getEntry("/Elevator/Control/Routine");
      var parkedTopic = NetworkTableInstance.getDefault().getEntry("/Elevator/Status/Parked");
      routine.setString("park");
      sc.run(2.0);
      assertTrue(e.isParked());
      assertTrue(parkedTopic.getBoolean(false));
      assertEquals(0.0, io.rightVolts(), 1e-9);
      assertEquals(0.0, io.leftVolts(), 1e-9);
      assertTrue(io.physicalInches() <= e.lowInches() + 0.05, "at " + io.physicalInches());
      // Parked, the stick is ignored. With the motors off the carriage may settle the last
      // 0.25 in onto the bottom stop under brake, slowly: never a drop.
      stick(-1.0);
      double fastestDown = 0;
      for (int i = 0; i < 150; i++) {
        sc.run(0.02);
        fastestDown = Math.max(fastestDown, -io.physicalVelocity());
      }
      System.out.printf("parked: settled to %.3f in, fastest down %.3f in/s%n", io.physicalInches(), fastestDown);
      assertTrue(e.isParked());
      assertTrue(io.physicalInches() <= e.lowInches() + 0.05);
      assertTrue(fastestDown < 1.0, "fastest down " + fastestDown);
      stick(0.0);
      // Another routine takes over and clears Parked.
      routine.setString("moves");
      sc.run(1.0);
      assertFalse(e.isParked());
      assertFalse(parkedTopic.getBoolean(true));
      // Disabling also clears it.
      routine.setString("park");
      sc.run(7.0);
      assertTrue(e.isParked());
      sc.disable();
      sc.run(0.1);
      assertFalse(e.isParked());
    }
  }

  @Test
  void nothingMovesWhileDisabled() {
    try (Scenario sc = Scenario.start()) {
      SimElevatorIO io = new SimElevatorIO();
      io.placeCarriage(0.0);
      Elevator e = new Elevator(io);
      e.setDefaultCommand(e.followStick());
      stick(-1.0);
      sc.run(2.0);
      assertEquals(0.0, io.physicalInches(), 1e-9);
    }
  }

  @Test
  void newGainsApplyWhileParkedButNotWhileMoving() {
    try (Scenario sc = Scenario.start()) {
      SimElevatorIO io = new SimElevatorIO();
      Elevator e = ranged(sc, io, Elevator.NOMINAL_TRAVEL_INCHES);
      // Moving: a new kP waits.
      e.goTo(e.lowInches()).schedule();
      sc.run(0.1);
      e.tuning().set("kP", 20);
      sc.run(0.2);
      assertEquals(Elevator.DEFAULT_GAINS.kP(), e.kPInUse(), 1e-9);
      // Parked (motors off), still enabled: it is written.
      e.park().schedule();
      sc.run(3.0);
      assertTrue(e.isParked());
      assertEquals(20, e.kPInUse(), 1e-9);
      e.tuning().set("kD", 0.3);
      sc.run(0.1);
      assertEquals(0.3, e.kDInUse(), 1e-9);
      // And the next routine runs with them, still accurately.
      e.standardMoves().schedule();
      sc.run(6.5);
      assertEquals(1.0, e.positionInches(), 0.03);
      assertFalse(e.faulted());
    }
  }

  @Test
  void fullTravelRoutineCrossesTheRangeQuicklyAndStaysOffTheStops() {
    try (Scenario sc = Scenario.start()) {
      SimElevatorIO io = new SimElevatorIO();
      Elevator e = ranged(sc, io, Elevator.NOMINAL_TRAVEL_INCHES);
      e.tuning().set("PeakVolts", 6);
      e.tuning().set("CruiseInPerSec", 45);
      e.tuning().set("AccelInPerSec2", 200);
      e.park().schedule();
      sc.run(3.0);
      NetworkTableInstance.getDefault().getEntry("/Elevator/Control/Routine").setString("fulltravel");
      double lowest = 99, highest = -99, firstAtTop = -1, t = 0;
      for (int i = 0; i < 400; i++) {
        sc.run(0.02);
        t += 0.02;
        double x = io.physicalInches();
        // (parked, the carriage may have settled onto the bottom stop; judge the low end after
        // the first trip up)
        if (firstAtTop >= 0) lowest = Math.min(lowest, x);
        highest = Math.max(highest, x);
        if (firstAtTop < 0 && x >= e.highInches() - 0.05) firstAtTop = t;
      }
      System.out.printf("fulltravel: %.3f..%.3f in, first top at %.2f s%n", lowest, highest, firstAtTop);
      assertTrue(highest <= e.highInches() + 0.03, "highest " + highest);
      assertTrue(lowest >= e.lowInches() - 0.03, "lowest " + lowest);
      // 1.5 s at the bottom, then about 0.45 s across the range at 45 in/s, 200 in/s^2.
      assertTrue(firstAtTop > 1.5 && firstAtTop < 2.1, "first top at " + firstAtTop);
      assertEquals(e.lowInches(), e.positionInches(), 0.03);
      assertFalse(e.faulted());
    }
  }

  @Test
  void isolationRoutinesDriveWithOneMotorOrTheLeftLeadingThenGoBackToNormal() {
    try (Scenario sc = Scenario.start()) {
      SimElevatorIO io = new SimElevatorIO();
      Elevator e = ranged(sc, io, Elevator.NOMINAL_TRAVEL_INCHES);
      e.tuning().set("PeakVolts", 6);
      e.tuning().set("StatorAmps", 40);
      e.tuning().set("CruiseInPerSec", 30);
      e.tuning().set("AccelInPerSec2", 200);
      var routine = NetworkTableInstance.getDefault().getEntry("/Elevator/Control/Routine");
      String[] names = {"soloright", "sololeft", "leftleads", "fulltravel_noconfig", "fulltravel_reconfig", "fulltravel_noconfig_left", "fulltravel_reconfig_left"};
      ElevatorIO.Drive[] drives = {
        ElevatorIO.Drive.RIGHT_ALONE, ElevatorIO.Drive.LEFT_ALONE, ElevatorIO.Drive.LEFT_LEADS,
        ElevatorIO.Drive.RIGHT_ALONE, ElevatorIO.Drive.RIGHT_ALONE, ElevatorIO.Drive.LEFT_ALONE, ElevatorIO.Drive.LEFT_ALONE
      };
      for (int n = 0; n < names.length; n++) {
        routine.setString("park");
        sc.run(3.0);
        assertEquals(ElevatorIO.Drive.RIGHT_LEADS, io.drive());
        routine.setString(names[n]);
        double lowest = 99, highest = -99, worstErr = 0, idleAmps = 0, t = 0;
        boolean sawDrive = false;
        for (int i = 0; i < 400; i++) {
          sc.run(0.02);
          t += 0.02;
          double x = io.physicalInches();
          if (t > 1.5) lowest = Math.min(lowest, x);
          highest = Math.max(highest, x);
          if (io.drive() == drives[n]) {
            sawDrive = true;
            double err = io.profileErrorInches();
            if (!Double.isNaN(err)) worstErr = Math.max(worstErr, Math.abs(err));
            // Alone: the other motor coasts and carries nothing.
            if (drives[n] == ElevatorIO.Drive.RIGHT_ALONE) idleAmps = Math.max(idleAmps, Math.abs(io.leftStatorAmps()));
            if (drives[n] == ElevatorIO.Drive.LEFT_ALONE) idleAmps = Math.max(idleAmps, Math.abs(io.rightStatorAmps()));
          }
        }
        System.out.printf("%s: %.3f..%.3f in, worst path error %.3f in%n", names[n], lowest, highest, worstErr);
        assertTrue(sawDrive, names[n]);
        assertEquals(0.0, idleAmps, 1e-9);
        assertTrue(highest <= e.highInches() + 0.03, "highest " + highest);
        assertTrue(highest >= e.highInches() - 0.05, "highest " + highest);
        assertTrue(lowest >= e.lowInches() - 0.03, "lowest " + lowest);
        assertTrue(worstErr < 0.1, "path error " + worstErr);
        assertFalse(e.faulted());
        // Back to normal once it ends.
        assertEquals(ElevatorIO.Drive.RIGHT_LEADS, io.drive());
      }
    }
  }

  @Test
  void aDisableDuringAnIsolationRoutinePutsTheRightBackInTheLead() {
    try (Scenario sc = Scenario.start()) {
      SimElevatorIO io = new SimElevatorIO();
      Elevator e = ranged(sc, io, Elevator.NOMINAL_TRAVEL_INCHES);
      NetworkTableInstance.getDefault().getEntry("/Elevator/Control/Routine").setString("sololeft");
      sc.run(1.0);
      assertEquals(ElevatorIO.Drive.LEFT_ALONE, io.drive());
      sc.disable();
      sc.run(0.1);
      assertEquals(ElevatorIO.Drive.RIGHT_LEADS, io.drive());
      assertFalse(e.isParked());
    }
  }
}
