package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helpers.Position;
import frc.robot.subsystems.Collection;
import frc.robot.subsystems.Gyron;
import frc.robot.subsystems.HTCVive;
import frc.robot.subsystems.Tracks;

/*
    Turn to an absolute degree
*/
public class TurnMiningDirection extends CommandBase {

  private final double bound_angle = 0.5;
  /** The base heading the bot is facing Should be initialized to a normal yaw value */
  double base;

  private Position miningStartPos;

  /** The target heading which the robot will turn to */
  double target;

  /** The distance in degrees between the base and target angles */
  double distance;

  /** The range of error in degrees around the target which the robot is allowed to turn to */
  double eRange;
  /**
   * Boolean determining which direction the bot will turn increasing direction is in a direction
   * where the angle is increasing This usuallly counter clockwise
   */
  boolean increasing;

  /**
   * This command is finished when the robot is turned to a heading withing the error range of the
   * target heading
   */
  boolean isFinished;

  private Tracks tracks;
  private Gyron gyro;
  private HTCVive vive;
  private Collection trencher;

  private int sortienum;

  //error range

  public TurnMiningDirection(
      Tracks tracks, Collection trencher, HTCVive vive, Gyron gyro, int sortienum) {
    this.tracks = tracks;
    this.gyro = gyro;
    this.vive = vive;
    this.trencher = trencher;
    this.sortienum = sortienum;
    isFinished = false;
    addRequirements(this.tracks, this.gyro, this.vive, this.trencher);
  }

  /*
   * Initialize Procedure:
   * Normalize the value of Yaw in the gyroscope, set it to base
   * Normalize the target angle
   * Calculate the shortest distance between those two degrees
   */
  @Override
  public void initialize() {

    // gyron.normalizeYaw();

    miningStartPos = vive.getPos();

    //determine mining direction

    if (sortienum == 0) {
      target = 180.0;
    }
    //turning commands if toward sieve side of arena
    else if (miningStartPos.getY() < -15) {
      if (sortienum == 1) {
        target = 135.0;
      } else {
        target = 100.0;
      }
    } else if (miningStartPos.getY() > 15) {
      if (sortienum == 1) {
        target = 225.0;
      } else {
        target = 260.0;
      }
    }
    //turning commands if toward starting zone size of arena
    else { //miningStartPosition.getY() is betweeen -25 and 25
      if (sortienum == 1) {
        target = 135.0;
      } else {
        target = 225.0;
      }
    }

    /**
     * after distance is calculated, we need to un-normalize the target goal. This is so we can turn
     * the shortest distance without worrying about bounds checking when crossing 360 degrees Our
     * target is a just base position plus the distance to get there.
     *
     * <p>Because the distance is positive/negative based on the shortest direction we can add it to
     * the base without checking a direction
     */
    // target = base + distance;

    //When the turning distance is 180 degrees, it will turn in a decreasing direction
    // increasing = distance > 0;
    isFinished = false;
  }

  @Override
  public void execute() {

    // double yaw = gyron.getYaw();

    double yaw = Position.positiveDegrees(gyro.getYaw());
    distance = getTurnDirection(target, yaw);
    boolean turn_dir = Math.signum(distance) >= 0;

    SmartDashboard.putNumber("mining dir", target);

    boolean turned = Math.abs(distance) >= bound_angle ? false : true;

    if (turned) {
      isFinished = true;
      tracks.setTurnVelocities(0, 0, turn_dir);
      return;
    } else {
      int turn_vel = 900;

      if (Math.abs(distance) <= 20) turn_vel = 700;
      if (Math.abs(distance) <= 10) turn_vel = 650;
      if (Math.abs(distance) <= 5) turn_vel = 400;
      tracks.setTurnVelocities(turn_vel, turn_vel, turn_dir);
    }

    /** SmartDashboard updating */
    SmartDashboard.putNumber("Degree Turning To:", target);
    SmartDashboard.putBoolean("turned", turned);
    SmartDashboard.putNumber("heading diff", distance);
    // SmartDashboard.putBoolean("Lower Bound", lowerBound);
    // SmartDashboard.putBoolean("Upper Bound", upperBound);
    SmartDashboard.putBoolean("Turn isFinished", isFinished);
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

  @Override
  public void end(boolean interrupted) {
    tracks.enableDisabledControl();
  }

  /** Returns a positive, normalized angle */
  private static double positiveDegrees(double angle) {
    angle = angle % 360;
    if (angle < 0) {
      angle += 360;
    }
    return Math.abs(angle);
  }

  /**
   * based on: https://math.stackexchange.com/questions/110080/shortest-way-to-achieve-target-angle
   *
   * @param target The normalized target angle
   * @param base The normalized base angle
   * @return An angle that is not normalized representing the shortest distance from base to target
   *     A positive return represents an increasing motion A negative return represents a decreasing
   *     motion
   */
  private static double getTurnDirection(double target, double base) {
    Double a = target - base;
    Double b = target - base + 360;
    Double y = target - base - 360;

    Double[] obj = new Double[] {a, b, y};
    double[] abs = new double[] {Math.abs(a), Math.abs(b), Math.abs(y)};

    int index = 0;
    //find the index of the smallest of the three absolute values
    //Loop only runs twice, runtime is fine
    for (int i = 1; i < 3; i++) {
      if (abs[i] < abs[index]) index = i;
    }

    return obj[index];
  }
}
