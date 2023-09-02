package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helpers.Position;
import frc.robot.subsystems.*;

public class DriveForward extends CommandBase {

  private final HTCVive m_vive;
  private final Tracks m_tracks;
  private final PathManager m_manager;
  private final Gyron m_gyro;
  private final Position target;
  private Position current;
  private final int boundRadius = 1;

  private boolean rightTarget;
  private boolean upperTarget;

  boolean xIn;
  boolean yIn;
  double velocity;

  private double start_heading;
  private double left_vel;
  private double right_vel;

  public DriveForward(
      Tracks tracks, HTCVive vive, PathManager manager, Gyron gyro, Position target) {
    this.m_tracks = tracks;
    this.m_vive = vive;
    this.target = target;
    this.m_manager = manager;
    m_gyro = gyro;
    xIn = false;
    yIn = false;
    velocity = 0;
    addRequirements(m_tracks, m_vive, manager);
  }

  @Override
  public void initialize() {
    current = m_vive.getPos();
    rightTarget = current.getX() <= target.getX();
    upperTarget = current.getY() <= target.getY();
    SmartDashboard.putString("Target Position", "Going to " + target.getX() + ", " + target.getY());
    left_vel = 1000;
    right_vel = 1000;
    // assume for now we start pointing in the right direction
    start_heading = m_gyro.getYaw();
  }

  @Override
  public void execute() {
    current = m_vive.getPos();
    /**
     * Quadrant Checking Once the current x and y values are past their respective goal based on
     * their initial orientation, set boolean saying each are in bounds If Y or X are out of bounds,
     * then it is set to false
     */
    yIn = (upperTarget ? target.getY() <= current.getY() : target.getY() >= current.getY());
    xIn = (rightTarget ? target.getX() <= current.getX() : target.getX() >= current.getX());
    /**
     * Barrier Checking Once the current x and y pair is within a 1 inch radius of the current
     * target, then we are also in bounds If one of the previous bound checks have been met, we want
     * to consider than in our second bounds check
     */
    xIn =
        xIn
            || (current.getX() >= target.getX() - boundRadius
                && current.getX() <= target.getX() + boundRadius);
    yIn =
        yIn
            || (current.getY() >= target.getY() - boundRadius
                && current.getY() <= target.getY() + boundRadius);

    SmartDashboard.putBoolean("Y in", yIn);
    SmartDashboard.putBoolean("X in", xIn);

    double heading_diff = start_heading - m_gyro.getYaw();
    SmartDashboard.putNumber("heading diff", heading_diff);
    if (Math.abs(heading_diff) >= 5) {
      if (heading_diff < 0) {
        left_vel = -1000;
        right_vel = 1000;
      } else {
        left_vel = 1000;
        right_vel = -1000;
      }
    } else {
      if (Math.abs(left_vel) == 1000
          && Math.abs(right_vel) == 1000
          && Math.signum(left_vel) != Math.signum(right_vel)) {
        left_vel = 1000;
        right_vel = 1000;
      }
      left_vel = left_vel * (1 - Math.sin(Math.toRadians((-1 * heading_diff) / 4)));
      right_vel = right_vel * (1 - Math.sin(Math.toRadians(heading_diff / 4)));
    }
    m_tracks.moveVelocityControl(left_vel, right_vel);
  }

  @Override
  public void end(boolean interrupted) {
    m_manager.setAtPosition(true);
    m_tracks.enableDisabledControl();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xIn && yIn;
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
}
