package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Collection;

public class StartTrencher extends CommandBase {

  private final Collection m_trencher;
  private boolean isFinished;

  public StartTrencher(Collection subsystem) {
    isFinished = false;
    m_trencher = subsystem;
    addRequirements(m_trencher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_trencher.moveTrencherCurrent(20);
    m_trencher.moveTrencherVelocity(
        Constants.TRENCH_MINING_VELOCITY); // TODO - return to this before LTC
    // m_trencher.moveTrencherVelocity(5000); // Slower for testing (noise)
    isFinished = true;
  }

  @Override
  public void execute() {}

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
