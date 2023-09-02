package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.time.Clock;

public class Gyron extends SubsystemBase {
  private ShuffleboardTab tab = Shuffleboard.getTab("Gyro");
  private int start_x;
  private int start_y;
  private int start_z;
  private NetworkTableEntry yaw = tab.add("yaw", 0).getEntry();
  private NetworkTableEntry pitch = tab.add("pitch", 0).getEntry();
  private NetworkTableEntry roll = tab.add("roll", 0).getEntry();
  private NetworkTableEntry time = tab.add("time", 0).getEntry();
  private WPI_Pigeon2 pigeon = new WPI_Pigeon2(Constants.GYRO_ID);
  Clock clock = Clock.systemDefaultZone();

  public Gyron() {
    time.setString("Hello");
  }

  public Gyron(int start_x, int start_y, int start_z) {}

  @Override
  public void periodic() {
    time.setDouble(clock.millis());
    yaw.setDouble(getYaw());
    roll.setDouble(getRoll());
    pitch.setDouble(getPitch());
  }

  @Override
  public void simulationPeriodic() {
    // unused
  }

  public double getYaw() {
    return positiveDegrees(pigeon.getYaw());
  }

  public double getPitch() {
    return positiveDegrees(pigeon.getPitch());
  }

  public double getRoll() {
    return positiveDegrees(pigeon.getRoll());
  }

  private static double positiveDegrees(double angle) {
    angle = angle % 360;
    if (angle < 0) {
      angle += 360;
    }
    return Math.abs(angle);
  }

  /*
   * Update the internal value of yaw to be within the range of [0, 360)
   */
  public void normalizeYaw() {
    pigeon.setYaw(positiveDegrees(pigeon.getYaw()));
  }

  public void setYaw(double inout) {
    pigeon.setYaw(inout, 0);
  }
}
