package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Reads the REV PDH every loop and logs it, as an independent check of what the Talons report
 * about their own supply current, and as the true battery current (roboRIO and the rest included).
 * The main values and their peaks since the last enable also go to /Power/Status/ in NetworkTables,
 * so a test script records them even when the robot's own log does not reach Forge.
 */
@Logged
public class PowerLog extends SubsystemBase {
  public static final int PDH_CAN_ID = 1;
  public static final int RIGHT_MOTOR_CHANNEL = 10;
  public static final int LEFT_MOTOR_CHANNEL = 11;
  public static final int RIO_CHANNEL = 22;

  private final PowerDistribution pdh = new PowerDistribution(PDH_CAN_ID, ModuleType.kRev);

  double pdhVolts;
  double pdhTotalAmps;
  double pdhRightMotorAmps;
  double pdhLeftMotorAmps;
  double pdhMotorsAmps;
  double pdhRioAmps;
  double pdhTotalWatts;
  double pdhTempC;
  double peakPdhTotalAmps;
  double peakPdhMotorsAmps;
  double minPdhVolts;

  private boolean wasEnabled;
  // A script changes /Elevator/Control/ResetPeaks to restart the peaks without a new enable.
  private final DoubleSubscriber resetPeaksSub =
      NetworkTableInstance.getDefault().getDoubleTopic(Elevator.RESET_PEAKS_TOPIC).subscribe(0);
  private double lastResetPeaks;
  private final DoublePublisher voltsPub;
  private final DoublePublisher totalAmpsPub;
  private final DoublePublisher rightMotorAmpsPub;
  private final DoublePublisher leftMotorAmpsPub;
  private final DoublePublisher motorsAmpsPub;
  private final DoublePublisher peakTotalAmpsPub;
  private final DoublePublisher peakMotorsAmpsPub;
  private final DoublePublisher minVoltsPub;

  public PowerLog() {
    NetworkTable status = NetworkTableInstance.getDefault().getTable("Power/Status");
    voltsPub = status.getDoubleTopic("PdhVolts").publish();
    totalAmpsPub = status.getDoubleTopic("PdhTotalAmps").publish();
    rightMotorAmpsPub = status.getDoubleTopic("PdhRightMotorAmps").publish();
    leftMotorAmpsPub = status.getDoubleTopic("PdhLeftMotorAmps").publish();
    motorsAmpsPub = status.getDoubleTopic("PdhMotorsAmps").publish();
    peakTotalAmpsPub = status.getDoubleTopic("PeakPdhTotalAmps").publish();
    peakMotorsAmpsPub = status.getDoubleTopic("PeakPdhMotorsAmps").publish();
    minVoltsPub = status.getDoubleTopic("MinPdhVolts").publish();
  }

  @Override
  public void periodic() {
    double[] amps = pdh.getAllCurrents(); // one read for every channel
    pdhVolts = pdh.getVoltage();
    pdhTotalAmps = pdh.getTotalCurrent();
    pdhRightMotorAmps = amps[RIGHT_MOTOR_CHANNEL];
    pdhLeftMotorAmps = amps[LEFT_MOTOR_CHANNEL];
    pdhMotorsAmps = pdhRightMotorAmps + pdhLeftMotorAmps;
    pdhRioAmps = amps[RIO_CHANNEL];
    pdhTotalWatts = pdhVolts * pdhTotalAmps;
    pdhTempC = pdh.getTemperature();

    // Peaks since the last enable (reset at each enable, or when ResetPeaks changes).
    boolean enabled = DriverStation.isEnabled();
    double reset = resetPeaksSub.get();
    boolean resetAsked = reset != lastResetPeaks;
    lastResetPeaks = reset;
    if ((enabled && !wasEnabled) || resetAsked) {
      peakPdhTotalAmps = 0;
      peakPdhMotorsAmps = 0;
      minPdhVolts = 0;
    }
    wasEnabled = enabled;
    if (enabled) {
      peakPdhTotalAmps = Math.max(peakPdhTotalAmps, pdhTotalAmps);
      peakPdhMotorsAmps = Math.max(peakPdhMotorsAmps, pdhMotorsAmps);
      if (pdhVolts > 1.0) {
        minPdhVolts = minPdhVolts == 0 ? pdhVolts : Math.min(minPdhVolts, pdhVolts);
      }
    }
    voltsPub.set(pdhVolts);
    totalAmpsPub.set(pdhTotalAmps);
    rightMotorAmpsPub.set(pdhRightMotorAmps);
    leftMotorAmpsPub.set(pdhLeftMotorAmps);
    motorsAmpsPub.set(pdhMotorsAmps);
    peakTotalAmpsPub.set(peakPdhTotalAmps);
    peakMotorsAmpsPub.set(peakPdhMotorsAmps);
    minVoltsPub.set(minPdhVolts);
  }
}
