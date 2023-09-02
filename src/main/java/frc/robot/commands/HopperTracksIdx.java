package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class HopperTracksIdx extends CommandBase {

  private final Hopper hopper;
  private final Collection collection;
  private final Tracks tracks;
  private boolean isFinished;
  private long startTime;
  private double currentEncoder;
  private double startEncoder;
  private double previousTrenchEncoder;
  private double newTrenchEncoder;
  private boolean runningHopper;
  private int state;

  private double currentRightTrackPos;

  public HopperTracksIdx(Hopper hopper, Collection collection, Tracks tracks) {
    this.hopper = hopper;
    this.collection = collection;
    this.tracks = tracks;
    isFinished = false;
    startTime = -1;
    currentEncoder = 0;
    startEncoder = 0;
    currentRightTrackPos = -1;
    runningHopper = false;
    addRequirements(this.hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initialize");
    isFinished = false;
    runningHopper = false;
    currentRightTrackPos = tracks.getRightEncoder();

    startEncoder = hopper.getHopperMotorPosition();
    currentEncoder = 0;
  }

  @Override
  public void execute() {
    // If the hopper has moved far enough that it would be dumping material, finish
    currentEncoder = hopper.getHopperMotorPosition();
    if (Math.abs(currentEncoder - startEncoder) > Constants.HOPPER_TRENCHING_BELT_DIST) {
      isFinished = true;
      hopper.disableHopperMotor();
      return;
    }

    // If tracks have moved far enough to warrant indexing the hopper and not already
    // running the hopper, set runningHopper to true
    if (Math.abs(tracks.getRightEncoder() - currentRightTrackPos) > 15000 && !runningHopper) {
      runningHopper = true;
      startTime = System.currentTimeMillis();
    }

    if (runningHopper) {
      //Run forwards for first 1.5 secs
      if (System.currentTimeMillis() - startTime < 1250) {
        hopper.moveHopperPercentage(-0.15);

        //Run backwards for last 0.5 sec
      } else if (System.currentTimeMillis() - startTime < 1500) {
        hopper.moveHopperPercentage(0.15);
      } else {
        hopper.disableHopperMotor();
        runningHopper = false;
        currentRightTrackPos = tracks.getRightEncoder();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
