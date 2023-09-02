package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.Position;

public class HTCVive extends SubsystemBase {
  private NetworkTableInstance inst;
  private NetworkTable table;

  private NetworkTableEntry center_x_entry;
  private NetworkTableEntry center_y_entry;
  private NetworkTableEntry center_z_entry;
  private Position startPosition;

  Double center_x;
  Double center_y;
  Double center_z;

  // DoubleSupplier center_x_supplier = () -> center_x;
  // DoubleSupplier center_y_supplier = () -> center_y;
  // DoubleSupplier center_z_supplier = () -> center_z;

  public HTCVive() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("Vive Position");
    center_x_entry = table.getEntry("x");
    center_y_entry = table.getEntry("y");
    center_z_entry = table.getEntry("z");
    try {
      center_x = center_x_entry.getDouble(-1);
      center_y = center_y_entry.getDouble(-1);
      center_z = center_z_entry.getDouble(-1);
      startPosition =
          new Position(
              (int) Math.round(center_x_entry.getDouble(-1)),
              (int) Math.round(center_y_entry.getDouble(-1)));
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    try {
      center_x = center_x_entry.getDouble(-1);
      center_y = center_y_entry.getDouble(-1);
      center_z = center_z_entry.getDouble(-1);
      SmartDashboard.putNumber("Vive Mapped Yaw", getYaw());
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void simulationPeriodic() {
    // unused
  }

  public int getCenterX() {
    return (int) Math.round(center_x);
  }

  public int getCenterY() {
    return (int) Math.round(center_y);
  }

  public int getCenterZ() {
    return (int) Math.round(center_z);
  }

  public Position getPos() {
    return new Position((int) Math.round(center_x), (int) Math.round(center_y));
  }

  public Position getStartPos() {
    return startPosition;
  }

  public double getYaw() {
    return (((-1) * table.getEntry("yaw").getDouble(0)) + 360) % 360;
  }

  public double getRawYaw() {
    return table.getEntry("yaw").getDouble(0);
  }
}
