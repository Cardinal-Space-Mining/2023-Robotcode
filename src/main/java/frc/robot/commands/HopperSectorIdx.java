package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class HopperSectorIdx extends CommandBase {

  private final Hopper hopper;
  private final Collection collection;
  private boolean isFinished;
  private long startTime;
  private double currentEncoder;
  private double previousTrenchEncoder;
  private double newTrenchEncoder;
  private boolean runningHopper;
  private int state;

  private double currentSectorPos;

  public HopperSectorIdx(Hopper hopper, Collection collection) {
    this.hopper = hopper;
    this.collection = collection;
    isFinished = false;
    startTime = -1;
    currentEncoder = 0;
    currentSectorPos = -1;
    runningHopper = false;
    addRequirements(this.hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initialize");
    isFinished = false;
    runningHopper = false;
    currentSectorPos = collection.getSectorEncoder();

    // currentEncoder = hopper.getHopperMotorPosition();
    // previousTrenchEncoder = 0;
    // newTrenchEncoder = 0;
    // state = 0;
  }

  @Override
  public void execute() {
    if (Math.abs(collection.getSectorEncoder() - currentSectorPos) > 10000 && !runningHopper) {
      runningHopper = true;
      startTime = System.currentTimeMillis();
    }

    if (runningHopper) {
      //Run forwards for first 2 secs
      if (System.currentTimeMillis() - startTime < 1250) {
        hopper.moveHopperPercentage(-0.15);
        ;

        //Run backwards for last 1 sec
      } else if (System.currentTimeMillis() - startTime < 1500) {
        hopper.moveHopperPercentage(0.15);
        ;
      } else {
        hopper.disableHopperMotor();
        runningHopper = false;
        currentSectorPos = collection.getSectorEncoder();
      }
    }

    SmartDashboard.putBoolean("Idx Hopper", runningHopper);

    // if(runningHopper){
    //     hopper.moveHopperPercentage(-0.15);
    //     if(Math.abs(hopper.getHopperMotorPosition() - currentEncoder) > 10000){
    //         hopper.disableHopperMotor();
    //         runningHopper = false;
    //         startTime = System.currentTimeMillis();
    //         currentEncoder = hopper.getHopperMotorPosition();
    //     } else {
    //         hopper.moveHopperPercentage(0.1);
    //         startTime = System.currentTimeMillis();
    //         runningHopper = false;
    //     }
    // }

    // //Hopper only moves after sector moves every few encoder ticks
    // newTrenchEncoder += previousTrenchEncoder;
    // newTrenchEncoder += collection.getSectorEncoder();
    // //state 0 still, 1 forward, -1 backwards
    // if (state == 0) {
    //     hopper.moveHopperPercentage(0);
    //     //No clue what the encoder tick limit should be 500 is arbitrary it may be low or really high
    //     if (newTrenchEncoder - previousTrenchEncoder > 500) {
    //         previousTrenchEncoder += collection.getSectorEncoder();
    //         state = 1;
    //         startTime = System.currentTimeMillis();
    //     }

    // }
    // else if (state == 1) {
    //     hopper.moveHopperPercentage(0.1);
    //     if (System.currentTimeMillis() - startTime > 666) {
    //         state = 2;
    //         startTime = System.currentTimeMillis();
    //     }
    // }
    // else if (state == 2) {
    //     hopper.moveHopperPercentage(-0.07);
    //     if (System.currentTimeMillis() - startTime > 333) {
    //         state = 0;
    //     }

    // }

    //moves hopper forward(up ramp) for 2/3 second and backwards for 1/3 second
    //THIS WORKS AS A BACKUP
    // if (System.currentTimeMillis() % 1000 < 333)
    // {
    //     hopper.moveHopperPercentage(0.1);
    // }
    // else
    // {
    //     hopper.moveHopperPercentage(-0.1);
    // }

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
