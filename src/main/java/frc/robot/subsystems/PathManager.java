package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.*;
import java.util.ArrayList;

public class PathManager extends SubsystemBase {

  private NetworkTableInstance inst;
  private NetworkTable table;
  private NetworkTableEntry coord;
  private NetworkTableEntry t_state;
  private NetworkTableEntry at_pos;
  private NetworkTableEntry at_dest;
  private NetworkTableEntry final_path_x;
  private NetworkTableEntry final_path_y;
  // private Double[] target;
  // private Boolean atDestination;
  // private Boolean atPosition;
  // private Number tState;

  public PathManager() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("Path Plan");
    coord = table.getEntry("coord");
    t_state = table.getEntry("t_state");
    at_dest = table.getEntry("at_dest");
    at_pos = table.getEntry("at_pos");
    final_path_x = table.getEntry("final_path_x");
    final_path_y = table.getEntry("final_path_y");

    t_state.setNumber(0);
    at_dest.setBoolean(false);
    at_pos.setBoolean(false);
    coord.setDoubleArray(new double[] {-1d, -1d});
    final_path_x.setDoubleArray(new double[] {0});
    final_path_y.setDoubleArray(new double[] {0});
  }

  @Override
  public void periodic() {
    //Do we really need to keep a periodic method that mimics the networktables
    //when we can just call what we need from the table?

  }

  @Override
  public void simulationPeriodic() {}

  public Boolean getAtPosition() {
    return at_pos.getBoolean(false);
  }

  public void setAtPosition(Boolean atpos) {
    at_pos.setBoolean(atpos);
  }

  public Boolean getAtDestination() {
    return at_dest.getBoolean(false);
  }

  public void setAtDestination(Boolean atDest) {
    at_dest.setBoolean(atDest);
  }

  public int getState() {
    return (int) t_state.getNumber(0);
  }

  public void setState(int state) {
    t_state.setNumber(state);
  }

  public Position getTargetPosition() {
    Double[] d = coord.getDoubleArray(new Double[] {-1d, -1d});
    return new Position(d[0].intValue(), d[1].intValue());
  }

  /**
   * Only use on initialization to get good behavior on starting up autonomous routine
   *
   * @param p
   */
  public void setTargetPosition(Position p) {
    coord.setDoubleArray(new double[] {p.getX(), p.getY()});
  }

  public void setFinalPath(double[] x, double[] y) {
    final_path_y.setDoubleArray(y);
    final_path_x.setDoubleArray(x);
  }

  public ArrayList<Position> getFinalPath() {
    double[] xs = final_path_x.getDoubleArray(new double[] {0});
    double[] ys = final_path_y.getDoubleArray(new double[] {0});
    ArrayList<Position> path = new ArrayList<>();
    for (int i = 0; i < xs.length; i++) {
      path.add(new Position((int) xs[i], (int) ys[i]));
    }
    return path;
  }
}
