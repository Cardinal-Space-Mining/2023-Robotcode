package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.LidarObject;
import java.util.ArrayList;

public class Lidar extends SubsystemBase {
  // private ShuffleboardTab lidarTab = Shuffleboard.getTab("Lidar");
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable lidar = inst.getTable("Lidar");
  private NetworkTableEntry distsEntry;
  private NetworkTableEntry anglesEntry;
  private NetworkTableEntry widthsEntry;
  private double[] dists;
  private double[] angles;
  private double[] widths;
  // private Supplier<double[]> distSup = () -> dists;
  // private Supplier<double[]> angleSup = () -> angles;
  // private Supplier<double[]> widthSup = () -> widths;

  private ArrayList<LidarObject> objects = new ArrayList<LidarObject>();

  public Lidar() {
    // lidarTab.addDoubleArray("Objects' Distance", distSup);
    // lidarTab.addDoubleArray("Objects' Angle", angleSup);
    // lidarTab.addDoubleArray("Objects' Width", widthSup);
  }

  @Override
  public void periodic() {
    distsEntry = lidar.getEntry("distance");
    anglesEntry = lidar.getEntry("angle");
    widthsEntry = lidar.getEntry("width");
    dists = distsEntry.getDoubleArray(new double[0]);
    angles = anglesEntry.getDoubleArray(new double[0]);
    widths = widthsEntry.getDoubleArray(new double[0]);
    if (dists.length != 0) {
      addObjects();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }

  public double[] getObjectDistanceArray() {
    return dists;
  }

  public double[] getObjectAngleArray() {
    return angles;
  }

  public double[] getObjectWidthArray() {
    return widths;
  }

  private void addObjects() {
    for (int i = 0; i < dists.length; i++) {
      objects.add(new LidarObject((int) dists[i], (int) angles[i], (int) widths[i]));
    }
  }

  public ArrayList<LidarObject> getObjects() {
    return objects;
  }
}
