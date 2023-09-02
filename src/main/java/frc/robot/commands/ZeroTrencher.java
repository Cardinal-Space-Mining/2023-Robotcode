package frc.robot.commands;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Collection;

/** Measures whether or not the trencher has been zeroed. */
public class ZeroTrencher extends CommandBase {

  private final Collection m_trencher;
  private boolean isFinished;

  private boolean zeroTrencher;

  public ZeroTrencher(Collection subsystem) {

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

    //First we need to ensure the sector has been zeroed.
    //If it has not been zeroed, Zero It
    zeroTrencher = !m_trencher.hasZeroed();
    if (zeroTrencher) {
      //If Trencher has not found zero, move up sector at 6% power until it does.
      m_trencher.moveSectorPercentageControl(Constants.SECTOR_ZEROING_PERCENTAGE);
    } else {

      isFinished = true;
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
