package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collection;

/**
 * @author Jeff Camp
 * @author Gavin Fisher
 */
public class MoveSectorAbsSafe extends CommandBase {

  private final Collection m_trencher;
  private boolean isFinished;
  private double targetAngle;
  private double velocity;
  private double targetLocation;

  public MoveSectorAbsSafe(Collection subsystem, int targetAngle, double velocity) {

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

    // targetLocation = m_trencher.getEncoderTartgetFromAngle(targetAngle);
    // m_trencher.updateSectorTarget(targetLocation);
  }

  @Override
  public void execute() {
    //VERY Dumb feedback loop here, so make sure sectorAtTarget band is good and big
    // if the trencher done mining, this command is done.
    // double currentLocation = m_trencher.getSectorEncoder();
    double currentAngle = m_trencher.getSectorAngle();

    if (m_trencher.sectorAtTargetAngle(
        this.targetAngle, 10)) { //(m_trencher.sectorAtTarget(20000))
      m_trencher.enableSectorDisabledControl();
      isFinished = true;
      // Else if not at target, keep moving towards it
    } else if (currentAngle < targetAngle) {
      m_trencher.moveSectorVelocityControl(-velocity);
      // Else if passed target, move back towards it
    } else if (currentAngle > targetAngle) {
      m_trencher.moveSectorVelocityControl(velocity);
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
