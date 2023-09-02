package frc.robot.commands;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Collection;

/**
 * Moves the trencher arm back to the home position. The trencher must be in the home position
 * before it can be moved.
 */
public class HomeTrencher extends CommandBase {

  private final Collection m_trencher;
  private boolean isFinished;

  public HomeTrencher(Collection subsystem) {

    isFinished = false;
    m_trencher = subsystem;
    addRequirements(m_trencher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initialize");
    isFinished = false;
  }

  @Override
  public void execute() {

    //Move Trencher to Zero Position
    if (m_trencher.getUpLimit()) {
      isFinished = true;
    } else {
      // m_trencher.setSectorTalonVelocity(Constants.SECTOR_CALC_RAPID_VELOCITY);
      m_trencher.moveSectorVelocityControl(Constants.SECTOR_CALC_RAPID_VELOCITY);
    }
  }

  // Called once the command ends or is interrupted.
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
