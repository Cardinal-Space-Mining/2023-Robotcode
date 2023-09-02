package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Tracks;

public class DriveEncoderDistance extends CommandBase {

  private boolean isFinished;
  private Tracks m_tracks;
  private int distance;

  public DriveEncoderDistance(Tracks tracks, int distance) {
    isFinished = false;
    m_tracks = tracks;
    this.distance = distance;
    addRequirements(m_tracks);
  }

  @Override
  public void initialize() {
    isFinished = false;

    double leftTarget = m_tracks.getLeftEncoder();
    double rightTarget = m_tracks.getRightEncoder();

    leftTarget = leftTarget + (distance * Constants.DISTANCE_TO_ENCODER);
    rightTarget = rightTarget - (distance * Constants.DISTANCE_TO_ENCODER);

    m_tracks.moveMotionMagic(leftTarget, rightTarget);
  }

  @Override
  public void execute() {
    isFinished = m_tracks.atGoalPosition();
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
