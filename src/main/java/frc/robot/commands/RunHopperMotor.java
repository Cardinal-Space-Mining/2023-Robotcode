package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;

public class RunHopperMotor extends CommandBase {
  private Hopper hopper;
  private boolean isFinished;
  private long startTime;

  public RunHopperMotor(Hopper hopper) {
    this.hopper = hopper;
    isFinished = true;
    addRequirements(this.hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initialize");
    isFinished = false;
    startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {

    boolean shouldStop = (System.currentTimeMillis() - startTime) > Constants.OFFLOAD_HOPPER_TIME;
    SmartDashboard.putBoolean("HopperMotor Shouldstop", shouldStop);
    SmartDashboard.putNumber("TimeElapsed", System.currentTimeMillis() - startTime);
    if (!shouldStop) {
      // hopper.moveHopperPercentage(-0.15);
      hopper.moveHopperVelocity(Constants.HOPPER_BELT_VELOCITY);
    } else {
      hopper.disableHopperMotor();
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
