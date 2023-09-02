package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Collection;
import frc.robot.subsystems.Tracks;

public class Trench extends CommandBase {

  private final Collection m_trencher;
  private final Tracks m_tracks;

  private boolean isFinished;
  private long jamStartTime;
  private boolean checkingForJam;
  private long trenchStartTime;
  private long trenchTimeout;

  private double targetDistance;
  private double currentDistance;
  private double startDistance;

  private long recoveryStartTime;
  private boolean attemptingRecovery;

  public Trench(
      Collection trencher, Tracks tracks, int targetDistance) { //targetDistance is Encoder ticks
    this.m_trencher = trencher;
    this.m_tracks = tracks;
    isFinished = false;
    jamStartTime = -1;
    checkingForJam = false;

    trenchStartTime = -1;
    trenchTimeout = Constants.TRENCHING_TIME;

    recoveryStartTime = -1;
    attemptingRecovery = false;

    this.targetDistance = targetDistance;
    startDistance = 0;
    currentDistance = 0;

    addRequirements(this.m_trencher);
    addRequirements(this.m_tracks);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initialize");
    isFinished = false;
    checkingForJam = false;
    attemptingRecovery = false;
    jamStartTime = -1;
    recoveryStartTime = -1;
    trenchStartTime = System.currentTimeMillis();
    m_trencher.setTrencherJammed(false);

    startDistance = m_tracks.getRightEncoder();
  }

  @Override
  public void execute() {

    //-----DISTANCE STOP-----
    currentDistance = m_tracks.getRightEncoder();
    if (Math.abs(currentDistance - startDistance) > targetDistance) {
      isFinished = true;
      m_tracks.enableDisabledControl();
      return;
    }

    //-----TIME STOP-----
    if (System.currentTimeMillis() - trenchStartTime > trenchTimeout) {
      isFinished = true;
      m_tracks.enableDisabledControl();
      return;
    }

    //-----JAM DETECTION-----

    // If trencher is not moving, begin JAM CHECK
    if (m_trencher.getTrencherVelocity() < Constants.TRENCH_JAM_VEL_LIMIT) {
      // If we're not checking for a jam, begin checking for a jam
      if (!checkingForJam && !attemptingRecovery) {
        jamStartTime = System.currentTimeMillis();
        checkingForJam = true;
      }
      // else if trencher is moving, stop checking for a jam
    } else {
      checkingForJam = false;
    }

    // if we're monitering a jam and enough time has elapsed, JAM DETECTED
    if (checkingForJam
        && (System.currentTimeMillis() - jamStartTime) > Constants.TRENCH_JAM_DURATION) {
      // WE'RE JAMMED
      m_trencher.setTrencherJammed(true);
      checkingForJam = false;

      // Begin recovery
      attemptingRecovery = true;
      recoveryStartTime = System.currentTimeMillis();

      // stop trencher & tracks
      m_trencher.enableTrencherDisabledControl();
      m_tracks.enableDisabledControl();
    }

    //-----JAM RECOVERY------

    //Time-based recovery routine
    if (attemptingRecovery) {

      // For first X seconds of recovery, raise sector
      if (System.currentTimeMillis() - recoveryStartTime < Constants.RECOVERY_RAISE_TIME) {
        m_trencher.moveSectorVelocityControl(Constants.SECTOR_CALC_RECOVER_MINING_VELOCITY);
      }

      // For second X seconds of recovery, run trencher backwards
      else if (System.currentTimeMillis() - recoveryStartTime < Constants.RECOVERY_TEST_TIME) {
        // m_trencher.moveTrencherVelocity(-Constants.TRENCH_MINING_VELOCITY);
        m_trencher.enableSectorDisabledControl();
        m_trencher.moveTrencherPercentageControl(-0.50);
      }

      // For third X seconds of recovery, run trecher forwards
      else if (System.currentTimeMillis() - recoveryStartTime
          < Constants.RECOVERY_TEST_TIME + 1000) {
        // m_trencher.moveTrencherVelocity(Constants.TRENCH_MINING_VELOCITY);
        m_trencher.enableSectorDisabledControl();
        m_trencher.moveTrencherPercentageControl(0.50);
      }

      // Run trencher backwards
      else if (System.currentTimeMillis() - recoveryStartTime
          < Constants.RECOVERY_TEST_TIME + 2 * 1000) {
        // m_trencher.moveTrencherVelocity(-Constants.TRENCH_MINING_VELOCITY);
        m_trencher.enableSectorDisabledControl();
        m_trencher.moveTrencherPercentageControl(-0.50);
      }

      // Run trecher forwards
      else if (System.currentTimeMillis() - recoveryStartTime
          < Constants.RECOVERY_TEST_TIME + 3 * 1000) {
        // m_trencher.moveTrencherVelocity(Constants.TRENCH_MINING_VELOCITY);
        m_trencher.enableSectorDisabledControl();
        m_trencher.moveTrencherPercentageControl(0.50);
      }

      // Reset jam detection, start trencher
      else {
        attemptingRecovery = false;
        m_trencher.setTrencherJammed(false);
        m_trencher.moveTrencherVelocity(Constants.TRENCH_MINING_VELOCITY);
        m_tracks.enableDisabledControl();
      }
    }

    //------NOMINAL MINING VELOCITY COMMANDS-----

    // Pause track motion if trencher jamming or jammed
    if ((m_trencher.getTrencherAvgCurrent() > Constants.TRENCH_SAFE_CURRENT_THRESH)
        || checkingForJam) {
      m_tracks.enableDisabledControl();
    }
    // else continue motion if no jam or jamming is detected
    else if (!m_trencher.getTrencherJammed()) {
      // m_tracks.moveVelocityControl(-Constants.TRENCHING_SPEED, Constants.TRENCHING_SPEED);
      m_tracks.movePercentageControl(0.1, 0.1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tracks.enableDisabledControl();
    isFinished = true;
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
