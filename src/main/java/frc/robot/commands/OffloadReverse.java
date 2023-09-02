package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Tracks;

/**
 * Offload reverse is the command that will move backwards for about 3 seconds, then stop once there
 * is a current spiker over the average current draw
 */
public class OffloadReverse extends CommandBase {

  public final double RIGHT_IDLE_CURRENT_DRAW = 0.5;
  public final double LEFT_IDLE_CURRENT_DRAW = 0.5;
  public final double BACKUP_TIMEOUT = 20_000;
  public final double CURRENT_SPIKE = 1.5;
  public final double ITERATIONS = 50;

  private final Tracks m_tracks;
  private double startTime;
  private boolean isFinished;
  private double leftCurrentSum;
  private double rightCurrentSum;
  private double totalIterations;

  /** Reverse but detects a current spike that is CURRENT_SPIKE times the average */
  public OffloadReverse(Tracks tracks) {
    isFinished = false;
    this.m_tracks = tracks;
    addRequirements(tracks);
    leftCurrentSum = 0.0;
    rightCurrentSum = 0.0;
    totalIterations = 0.0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initialize");
    isFinished = false;
    leftCurrentSum = 0.0;
    rightCurrentSum = 0.0;
    totalIterations = 0.0;
    startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    if (System.currentTimeMillis() - BACKUP_TIMEOUT > startTime) {
      m_tracks.movePercentageControl(0, 0);
      m_tracks.enableDisabledControl();
      isFinished = true;
    }

    m_tracks.moveVelocityControl(Constants.TRACK_VELOCITY, Constants.TRACK_VELOCITY);
    ;

    // Helpful debugging info:
    // SmartDashboard.putNumber("iterations", totalIterations);
    // SmartDashboard.putNumber("OFLCurr", m_tracks.leftCurrentDraw());
    // SmartDashboard.putNumber("OFRCurr", m_tracks.rightCurrentDraw());
    SmartDashboard.putNumber("Left Current Average", (leftCurrentSum / totalIterations));
    SmartDashboard.putNumber("Right Curernt Average", (rightCurrentSum / totalIterations));

    // Do not test for a current spike if the iterations are too small
    if (this.totalIterations < ITERATIONS) {
      leftCurrentSum += m_tracks.leftCurrentDraw();
      rightCurrentSum += m_tracks.rightCurrentDraw();
      totalIterations++;

      // Otherwise check for current spike
    } else if (m_tracks.leftCurrentDraw() > LEFT_IDLE_CURRENT_DRAW
        && m_tracks.rightCurrentDraw() > RIGHT_IDLE_CURRENT_DRAW) {

      leftCurrentSum += m_tracks.leftCurrentDraw();
      rightCurrentSum += m_tracks.rightCurrentDraw();
      totalIterations++;

      double leftAverage = leftCurrentSum / totalIterations;
      double rightAverage = rightCurrentSum / totalIterations;

      // If larger than a given current spike, stop the motors and finish the command
      if (m_tracks.rightCurrentDraw() > (CURRENT_SPIKE * rightAverage)
          || m_tracks.leftCurrentDraw() > (CURRENT_SPIKE * leftAverage)) {
        m_tracks.movePercentageControl(0, 0);
        m_tracks.enableDisabledControl();
        isFinished = true;
      }
    } else {
      rightCurrentSum = 0;
      leftCurrentSum = 0;
      totalIterations = 0;
      isFinished = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tracks.enableDisabledControl();
    SmartDashboard.putNumber("Left Current Average", 0);
    SmartDashboard.putNumber("Right Curernt Average", 0);
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
}
