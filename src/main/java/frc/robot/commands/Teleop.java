package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Hopper.HopperStatus;

public class Teleop extends CommandBase {

  private final Tracks m_tracks;
  private final Hopper m_hopper;
  private final Collection m_trencher;

  private final XboxController controller;
  private final Joystick joystick;

  private double trencherSpeed;

  private boolean isFinished;

  public Teleop(
      Tracks tracks,
      Hopper hopper,
      Collection trencher,
      XboxController controller,
      Joystick joystick) {

    isFinished = false;
    m_tracks = tracks;
    m_hopper = hopper;
    m_trencher = trencher;

    addRequirements(m_tracks);
    addRequirements(m_hopper);
    addRequirements(m_trencher);

    m_tracks.resetEncoderTargets();
    m_tracks.resetQueues();
    m_tracks.updateHeading(0);

    m_trencher.resetQueues();
    m_trencher.resetEncoderTargets();

    this.controller = controller;
    this.joystick = joystick;
    trencherSpeed = 0;
  }

  @Override
  public void initialize() {

    System.out.println("Initialize");

    m_tracks.resetEncoderTargets();
    m_tracks.resetQueues();
    m_tracks.updateHeading(0);

    m_trencher.resetQueues();
    m_trencher.resetEncoderTargets();

    isFinished = false;
  }

  @Override
  public void execute() {

    // Track Mapping update
    double left = controller.getRightY();
    double right = controller.getLeftY();

    // Collection Mapping update
    double sector = joystick.getY();

    double linear = joystick.getThrottleChannel();
    boolean linearUp = joystick.getPOV(0) == 0 ? true : false;
    boolean linearDown = joystick.getPOV(0) == 180 ? true : false;
    // boolean linearEnable = joystick.getRawButton(7);

    boolean trencherFwd = joystick.getTrigger();
    boolean trencherBkw = joystick.getRawButton(3);
    boolean trencherSpeedUp = joystick.getRawButtonPressed(5);
    boolean trencherSpeedDown = joystick.getRawButtonPressed(3);

    // Hopper Mapping update
    boolean hopperFwd = joystick.getRawButton(6);
    boolean hopperBkw = joystick.getRawButton(4);

    // boolean bucketFwd = joystick.getRawButton(2);
    // boolean bucketBkw = joystick.getRawButton(11);

    // Move the Tracks
    // ------------------------------------------------------
    this.m_tracks.moveVelocityControl(-left * 2000, -right * 2000);

    // Move the Hopper
    // ------------------------------------------------------
    double HopperVelocity = 0;
    if (hopperFwd) {
      HopperVelocity = -700;
    }
    if (hopperBkw) {
      HopperVelocity = 700;
    }

    this.m_hopper.moveHopperVelocity(HopperVelocity);

    // Move the Sector
    // ------------------------------------------------------
    // double sectorPercentage = sector;

    this.m_trencher.moveSectorVelocityControl(2000 * sector);

    // Move the Trencher
    // ------------------------------------------------------
    if (trencherSpeedUp && trencherSpeed < 1) {
      trencherSpeed += .05;
    }
    if (trencherSpeedDown && trencherSpeed > -1) {
      trencherSpeed -= .05;
    }

    double trencherVelocity = 0;
    if (trencherFwd && !trencherBkw) {
      // trencherCurrent = -30 * trencherSpeed;
      trencherVelocity = Constants.TRENCH_MINING_VELOCITY;
    }
    if (trencherFwd && trencherBkw) {
      // trencherCurrent = 30 * trencherSpeed;
      trencherVelocity = -Constants.TRENCH_MINING_VELOCITY;
    }

    this.m_trencher.moveTrencherVelocity(trencherVelocity);

    // Move Linear Actuator.
    // ------------------------------------------------------
    double linearPercentage = 0;

    if (linearUp) {
      linearPercentage = Math.min(linear, 1);
    }
    if (linearDown) {
      linearPercentage = -1 * Math.min(linear, 1);
    }

    this.m_hopper.moveActuatorPercentageControl(linearPercentage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tracks.resetEncoderTargets();
    m_tracks.resetQueues();
    m_tracks.updateHeading(0);
    m_trencher.resetQueues();
    m_trencher.resetEncoderTargets();
    m_hopper.setHopperStatus(HopperStatus.UNKNOWN);
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
