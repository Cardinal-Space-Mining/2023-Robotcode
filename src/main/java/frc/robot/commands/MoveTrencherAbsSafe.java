package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collection;

/**
 * @author Jeff Camp
 * @author Gavin Fisher
 */
public class MoveTrencherAbsSafe extends CommandBase {

  private final Collection m_trencher;
  private boolean isFinished;
  private double targetAngle;
  private double velocity;
  private double targetLocation;

  public MoveTrencherAbsSafe(Collection subsystem, int targetAngle, double velocity) {

    isFinished = false;
    m_trencher = subsystem;
    addRequirements(m_trencher);
    this.velocity = velocity;
    this.targetAngle = targetAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initialize");
    isFinished = false;

    targetLocation = m_trencher.getEncoderTartgetFromAngle(targetAngle);

    m_trencher.updateSectorTarget(targetLocation);
    // m_trencher.moveSectorVelocityControl(this.velocity);
    m_trencher.setSectorTalonVelocity(velocity);
  }

  @Override
  public void execute() {
    // if the trencher done mining, this command is done.
    if (m_trencher.sectorAtTarget(20000)) {
      m_trencher.setSectorTalonVelocity(0); //m_trencher.getSectorEncoder() >= targetLocation) {
      isFinished = true;
    }

    // if current is over the threshold, stop the arm velocity
    // else if (m_trencher.getSectorCurrent() > Constants.SECTOR_SAFE_CURRENT_THRESH) {
    //     m_trencher.moveSectorVelocityControl(0);
    // }
    // // if trencher current is safe, keep pushing arm into material.
    // else if (m_trencher.getSectorCurrent() <= Constants.SECTOR_SAFE_CURRENT_THRESH) /* Constants.TRENCHER_SAFE_THRESHOLD_ */ {
    //     m_trencher.moveSectorVelocityControl(velocity);
    // }
    // m_trencher.setTalonVelocity(velocity);
    SmartDashboard.putNumber("Sector Velocity", velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
