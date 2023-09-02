package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tracks;

public class DriveBackwardEncoderTicks extends CommandBase {

  private final Tracks m_tracks;
  private boolean isFinished;
  private double startRight;
  // private double startLeft;
  private int encoderTarget;

  public DriveBackwardEncoderTicks(Tracks tracks, int target) {
    this.m_tracks = tracks;
    isFinished = false;
    this.encoderTarget = target;
    addRequirements(m_tracks);
  }

  @Override
  public void initialize() {
    isFinished = false;
    startRight = m_tracks.getRightEncoder();
    // startLeft = m_tracks.getLeftEncoder();
  }

  @Override
  public void execute() {
    m_tracks.moveVelocityControl(1000, 1000);
    if (Math.abs(m_tracks.getRightEncoder() - startRight) > encoderTarget) {
      m_tracks.enableDisabledControl();
      isFinished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
}
