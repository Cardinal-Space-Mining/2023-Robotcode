package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Collection;

public class SectorPlunge extends CommandBase {

  private final Collection m_trencher;
  private boolean isFinished;
  private long jamStartTime;
  private boolean checkingForJam;
  private long plungeStartTime;
  private long plungeTimeout;

  private long gravelStartTime;
  private long gravelTimeout;

  private double targetAngle;
  private double targetLocation;

  private long recoveryStartTime;
  private boolean attemptingRecovery;

  public SectorPlunge(Collection trencher, int targetAngle) {
    this.m_trencher = trencher;
    isFinished = false;
    jamStartTime = -1;
    checkingForJam = false;

    plungeStartTime = -1;
    plungeTimeout = Constants.PLUNGE_TIME;
    gravelStartTime = -1;
    gravelTimeout = Constants.GRAVEL_TIME;

    recoveryStartTime = -1;
    attemptingRecovery = false;

    this.targetAngle = targetAngle;
    addRequirements(this.m_trencher);
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
    plungeStartTime = System.currentTimeMillis();
    m_trencher.setTrencherJammed(false);

    targetLocation = m_trencher.getEncoderTartgetFromAngle(targetAngle);
    m_trencher.updateSectorTarget(targetLocation);

    //start trencher
    m_trencher.moveTrencherVelocity(Constants.TRENCH_MINING_VELOCITY);
  }

  @Override
  public void execute() {
    //-----TIME STOP-----
    if (System.currentTimeMillis() - plungeStartTime > plungeTimeout) {
      isFinished = true;
      m_trencher.enableSectorDisabledControl();
      return;
    }

    //-----POSITIONAL STOPS-----
    if (m_trencher.getDownLimit()) {
      m_trencher.enableSectorDisabledControl();
      isFinished = true;
      return;
    } else if (m_trencher.sectorAtTarget(10000)) {
      m_trencher.enableSectorDisabledControl();
      isFinished = true;
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

      // stop trencher & sector
      m_trencher.enableTrencherDisabledControl();
      m_trencher.enableSectorDisabledControl();
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
          < Constants.RECOVERY_TEST_TIME + 2 * 1000) {
        // m_trencher.moveTrencherVelocity(Constants.TRENCH_MINING_VELOCITY);
        m_trencher.enableSectorDisabledControl();
        m_trencher.moveTrencherPercentageControl(0.50);
      }

      // Run trencher backwards
      else if (System.currentTimeMillis() - recoveryStartTime
          < Constants.RECOVERY_TEST_TIME + 4 * 1000) {
        // m_trencher.moveTrencherVelocity(-Constants.TRENCH_MINING_VELOCITY);
        m_trencher.enableSectorDisabledControl();
        m_trencher.moveTrencherPercentageControl(-0.50);
      }

      // Run trecher forwards
      else if (System.currentTimeMillis() - recoveryStartTime
          < Constants.RECOVERY_TEST_TIME + 6 * 1000) {
        // m_trencher.moveTrencherVelocity(Constants.TRENCH_MINING_VELOCITY);
        m_trencher.enableSectorDisabledControl();
        m_trencher.moveTrencherPercentageControl(0.50);
      }

      // Reset jam detection, start trencher
      else {
        attemptingRecovery = false;
        m_trencher.setTrencherJammed(false);
        m_trencher.moveTrencherVelocity(Constants.TRENCH_MINING_VELOCITY);
        m_trencher.enableSectorDisabledControl();
      }
    }

    //------NOMINAL MINING VELOCITY COMMANDS-----

    // Pause plunge motion if trencher jamming or jammed
    if ((m_trencher.getTrencherAvgCurrent() > Constants.TRENCH_SAFE_CURRENT_THRESH)
        || checkingForJam) {
      // m_trencher.moveSectorVelocityControl(0);
      m_trencher.enableSectorDisabledControl();
    }
    // else continue plunge if no jam or jamming is detected
    else if (!m_trencher.getTrencherJammed()) {
      m_trencher.moveSectorVelocityControl(-Constants.SECTOR_CALC_MINING_VELOCITY);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_trencher.enableSectorDisabledControl();
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
