package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * A headless scripted scenario: the simulated Driver Station enables, commands run for a set
 * time in 20 ms steps on simulated time, and the test asserts on the subsystems' state. The
 * enable here is the simulator's; nothing touches a real robot.
 */
public final class Scenario implements AutoCloseable {
  public static final double STEP_SECONDS = 0.02;

  private Scenario() {}

  public static Scenario start() {
    if (!HAL.initialize(500, 0)) {
      throw new IllegalStateException("the simulation HAL did not start");
    }
    SimHooks.pauseTiming();
    DriverStationSim.resetData();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    return new Scenario();
  }

  public Scenario enable() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    return this;
  }

  public Scenario disable() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    return this;
  }

  /** Runs the scheduler (and every subsystem's periodic and simulation step) for this long. */
  public Scenario run(double seconds) {
    long steps = Math.round(seconds / STEP_SECONDS);
    for (long i = 0; i < steps; i++) {
      DriverStation.refreshData();
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(STEP_SECONDS);
    }
    return this;
  }

  @Override
  public void close() {
    disable();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    SimHooks.resumeTiming();
  }
}
