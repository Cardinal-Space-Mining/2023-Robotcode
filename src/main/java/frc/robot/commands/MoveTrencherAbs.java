package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collection;

public class MoveTrencherAbs extends CommandBase {

  private final Collection m_trencher;
  private boolean isFinished;
  private double targetAngle;
  private int velocity;

  public MoveTrencherAbs(Collection subsystem, int angleDepth, int velocity) {

    isFinished = false;
    m_trencher = subsystem;
    addRequirements(m_trencher);
    this.velocity = velocity;
    targetAngle = angleDepth;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initialize");
    isFinished = false;

    double targetLocation = m_trencher.getEncoderTartgetFromAngle(targetAngle);

    m_trencher.moveSectorMotionMagic(targetLocation, this.velocity);
  }

  @Override
  public void execute() {

    // SmartDashboard.putBoolean("Sector At Target", m_trencher.sectorAtTarget());
    if (m_trencher.sectorAtTarget()) {
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
