package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Reads the REV PDH every loop and logs it, as an independent check of what the Talons report
 * about their own supply current, and as the true battery current (roboRIO and the rest included).
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
  }
}
