package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.helpers.Path;
import frc.robot.helpers.Position;
import frc.robot.subsystems.*;
import java.util.ArrayList;

public class TeleopWaypoint extends CommandBase {

  private final Tracks m_tracks;
  private final HTCVive m_vive;
  private final PathManager m_manager;
  // private final XboxController controller;

  private boolean isFinished;

  private Position curpos;
  private Position prevpos;

  private Path path;

  private XboxController controller;

  public TeleopWaypoint(
      Tracks tracks, HTCVive vive, PathManager manager, XboxController controller) {

    isFinished = false;
    m_tracks = tracks;
    m_vive = vive;
    m_manager = manager;

    m_tracks.resetEncoderTargets();
    m_tracks.resetQueues();
    m_tracks.updateHeading(0);

    this.controller = new XboxController(0);
    path = new Path();
  }

  @Override
  public void initialize() {

    System.out.println("Initialize");

    m_tracks.resetEncoderTargets();
    m_tracks.resetQueues();
    m_tracks.updateHeading(0);

    isFinished = false;

    curpos = prevpos = m_vive.getPos();
  }

  @Override
  public void execute() {
    curpos = m_vive.getPos();

    if (controller.getRawButtonPressed(8) || curpos.getX() >= Constants.END_X) {
      isFinished = true;
      m_tracks.enableDisabledControl();
      return;
    }
    // Track Mapping update
    double left = controller.getLeftY();
    double right = controller.getRightY();
    this.m_tracks.moveVelocityControl(left * 2000, right * 2000);

    if (((int) curpos.getX() != (int) prevpos.getX())
        || ((int) curpos.getY() != (int) prevpos.getY())) {
      path.add(curpos);
      prevpos = curpos;
    }

    SmartDashboard.putBoolean("in teleway", !isFinished);
    SmartDashboard.putNumber("left joy", left);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tracks.resetEncoderTargets();
    m_tracks.resetQueues();
    m_tracks.updateHeading(0);
    //Update shuffleboard with new path
    ArrayList<Position> compressed = new ArrayList<>();
    Position[] goodPath = getVertices(path).toArray(new Position[compressed.size()]);
    double[] x_path = new double[goodPath.length];
    double[] y_path = new double[goodPath.length];

    for (int i = 0; i < goodPath.length; i++) {
      x_path[i] = goodPath[i].getX();
      y_path[i] = goodPath[i].getY();
    }

    m_manager.setFinalPath(x_path, y_path);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }

  public static ArrayList<Position> getVertices(Path path) {
    ArrayList<Position> vertices = new ArrayList<>();

    boolean changingY = false;
    boolean prevChangingY = false;
    boolean changingX = false;
    boolean prevChangingX = false;
    if (path.size() < 2) return new ArrayList<Position>();
    Position curPoint = path.remove();
    Position nextPoint = path.remove();

    while (!path.isEmpty()) {

      if (curPoint.getX() != nextPoint.getX()) {
        changingX = true;
      } else {
        changingX = false;
      }
      if (curPoint.getY() != nextPoint.getY()) {
        changingY = true;
      } else {
        changingY = false;
      }
      if (prevChangingX != changingX || prevChangingY != changingY) {
        vertices.add(curPoint);
      }
      curPoint = nextPoint;
      nextPoint = path.remove();
      prevChangingX = changingX;
      prevChangingY = changingY;
    }

    vertices.add(curPoint);

    return vertices;
  }
}
