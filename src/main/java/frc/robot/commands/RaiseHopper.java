package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.HopperStatus;

public class RaiseHopper extends CommandBase {
  private final Hopper hopper;
  private boolean isFinished;
  private long startTime;

  public RaiseHopper(Hopper hopper) {
    this.hopper = hopper;
    isFinished = false;
    startTime = -1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    if (hopper.isRaised()) {
      isFinished = true;
      return;
    }
    boolean shouldStop = (System.currentTimeMillis() - startTime) > Constants.HOPPER_TIMOUT_MS;
    if (!shouldStop) {
      hopper.raiseActuator();
    } else {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      hopper.setHopperStatus(HopperStatus.UNKNOWN);
    } else {
      hopper.setHopperStatus(HopperStatus.RAISED);
    }
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
