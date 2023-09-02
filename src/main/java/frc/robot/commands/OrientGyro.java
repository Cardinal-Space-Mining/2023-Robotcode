package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class OrientGyro extends CommandBase {

  private final HTCVive m_vive;
  private final Gyron m_gyro;
  private final Tracks m_tracks;

  private boolean isFinished;

  private double current;
  private boolean increasing;
  private double velocity;

  NetworkTableEntry isOriented;

  public OrientGyro(HTCVive vive, Gyron gyro, Tracks track) {
    m_gyro = gyro;
    m_vive = vive;
    m_tracks = track;
    isFinished = false;
    isOriented = NetworkTableInstance.getDefault().getTable("Path Plan").getEntry("gyro_oriented");
    addRequirements(m_vive, m_gyro, m_tracks);
  }

  @Override
  public void initialize() {
    isFinished = false;
    current = m_vive.getRawYaw();
    increasing = current < 180;
  }

  @Override
  public void execute() {
    current = m_vive.getRawYaw();
    increasing = current < 180;

    if (current > (350) || current < (10)) {
      velocity = 400;
    } else if (current > (335) || current < (25)) {
      velocity = 700;
    } else {
      velocity = Constants.TURN_VELOCITY;
    }
    m_tracks.setTurnVelocities(velocity, velocity, increasing);

    boolean lowerBound = current >= 359.5;
    boolean upperBound = current <= 0.5;

    if ((!increasing && lowerBound) || (increasing && upperBound)) {
      System.out.print("Stopping");
      m_tracks.setTurnVelocities(0, 0, increasing);
      isOriented.setBoolean(true);
      isFinished = true;
    } else {
      isOriented.setBoolean(false);
    }
    m_gyro.setYaw(0);
    // SmartDashboard.putNumber("Degree Turning To:", 0);
    // SmartDashboard.putNumber("current vive yaw", current);
    // SmartDashboard.putBoolean("Increasing Yaw", increasing);
    // SmartDashboard.putBoolean("Lower Bound", lowerBound);
    // SmartDashboard.putBoolean("Upper Bound", upperBound);
    // SmartDashboard.putBoolean("Turn isFinished", isFinished);

  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

  @Override
  public void end(boolean interrputed) {}
}
