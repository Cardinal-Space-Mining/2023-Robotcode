package frc.robot.commands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;

public class StartHopper extends CommandBase {
  private Hopper hopper;
  private boolean isFinished;

  public StartHopper(Hopper hopper) {
    this.hopper = hopper;
    isFinished = false;
    addRequirements(this.hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initialize");
    // hopper.moveHopperVelocity(500);
    hopper.moveHopperPercentage(-0.1);
  }

  @Override
  public void execute() {
    isFinished = true;
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
