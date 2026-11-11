package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/**
 * Plain-Java stand-in for the motors: a position loop (same kP and voltage cap as the real
 * Talons) driving a physics model of two Kraken X44 on the chain. Deterministic, so scenario
 * tests can run it faster than real time (the CTRE simulator cannot).
 */
public class SimElevatorIO implements ElevatorIO {
  private static final double DT = 0.02;
  private static final double CARRIAGE_KG = 2.0; // guess
  private static final double DRUM_RADIUS_M = Elevator.INCHES_PER_SHAFT_TURN / (2 * Math.PI) * 0.0254;

  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getKrakenX44(2),
          Elevator.MOTOR_ROTATIONS_PER_SHAFT_TURN,
          CARRIAGE_KG,
          DRUM_RADIUS_M,
          0.0,
          Elevator.TRAVEL_INCHES * 0.0254,
          true,
          0.0);
  private double volts;
  private boolean driving;
  private double targetRotations;

  @Override
  public void setPosition(double rotations) {
    targetRotations = rotations;
    driving = true;
  }

  @Override
  public void neutral() {
    driving = false;
  }

  @Override
  public void update() {
    volts =
        driving
            ? MathUtil.clamp(
                Elevator.KP_VOLTS_PER_ROTATION * (targetRotations - rightRotations()),
                -Elevator.PEAK_VOLTS,
                Elevator.PEAK_VOLTS)
            : 0.0;
    sim.setInputVoltage(volts);
    sim.update(DT);
  }

  @Override
  public double rightRotations() {
    return sim.getPositionMeters() / 0.0254 * Elevator.MOTOR_ROTATIONS_PER_INCH;
  }

  @Override public double leftRotations() { return rightRotations(); }
  @Override public double rightVolts() { return volts; }
  @Override public double leftVolts() { return volts; }
  @Override public double rightStatorAmps() { return sim.getCurrentDrawAmps() / 2; }
  @Override public double leftStatorAmps() { return sim.getCurrentDrawAmps() / 2; }
}
