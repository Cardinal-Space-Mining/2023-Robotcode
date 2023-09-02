package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.helpers.Position;
import frc.robot.subsystems.*;
import java.util.ArrayList;

public class TraversePath extends CommandBase {
  /** Final subsytem requirements */
  private final HTCVive m_vive;

  private final Tracks m_tracks;
  private final PathManager m_manager;
  private final Gyron m_gyro;

  private static final int ITERATIONS = 100;
  private static final int SPIKED_ITERATIONS = 20;
  private static final double CURRENT_SPIKE = 55;

  /**
   * Position units to determine the radius around a point which is considered in bounds Useful to
   * catch robot in case driving error, such as slipping, so the robot is in bounds when it is
   * "Close enough"
   */
  private final int boundRadius = 2;

  /** Internal int to keep track of the state of destination movement. */
  private PathDirection dir;

  private RobotOrientation orientation;
  private PathState path_state;

  //Target position to be updated every time we reach it
  private Position target;
  private Position newTarget;

  //Current position to be updated on initialization and execute loop
  private Position current;

  //2D Distance from current position to target position
  private double tDistance;

  /** Doubles determine the left and right talon velocities; */
  private double left_vel;

  private double right_vel;

  //Our target heading for any given target position.
  private double tHeading;

  private double heading_diff;
  private boolean turn_dir;

  //States if we have already turned to a heading based on
  private boolean turned;

  //Our current heading
  private double cHeading;

  private ArrayList<Position> path;
  private int index;

  private boolean isFinished;

  private boolean prev_overshoot_forward;
  private boolean prev_overshoot_backward;
  private boolean overshoot_forward;
  private boolean overshoot_backward;

  private int iterations;
  private int currentSpikedIterations;
  private double leftCurrentSum;
  private double rightCurrentSum;
  NetworkTableEntry currentSpikePos;
  NetworkTableEntry currentSpike;

  /**
   * SmartDashboard.putNumber("lvel", left_vel); SmartDashboard.putNumber("rvel", right_vel);
   * SmartDashboard.putBoolean("turned?", turned); SmartDashboard.putBoolean("forward_overshoot",
   * overshoot_forward); SmartDashboard.putBoolean("backward_overshott?", overshoot_backward);
   * SmartDashboard.putNumber("Turning To", tHeading); SmartDashboard.putNumber("Target Distance",
   * tDistance); SmartDashboard.putNumber("heading diff", heading_diff); SmartDashboard.putString(
   * "Target Position", "Going to " + target.getX() + ", " + target.getY());
   */

  // /**
  //  * state the robot will traverse in FORWARD - from starting zone to mining zone BACKWARD - from
  //  * mining zone to starting zone
  //  */

  public enum PathDirection {
    FORWARD,
    BACKWARD,
  }

  /** traverse with hopper or trencher forward */
  public enum RobotOrientation {
    HOPPER_FORWARD,
    TRENCHER_FORWARD
  }

  /**
   * DYNAMIC - if robot hasn't made it to the mining zone once yet STATIC - has already found a path
   * and is traversing the same path POSITION - traverse to a position
   */
  public enum PathState {
    DYNAMIC,
    STATIC,
    POSITION
  }

  public TraversePath(
      HTCVive vive,
      Tracks tracks,
      Gyron gyro,
      PathManager manager,
      PathDirection direction,
      RobotOrientation orientation,
      PathState path_state,
      Position position) {
    m_tracks = tracks;
    m_vive = vive;
    m_gyro = gyro;
    m_manager = manager;
    this.dir = direction;
    this.path_state = path_state;
    this.orientation = orientation;
    this.path = new ArrayList<>();
    this.newTarget = position;
    this.iterations = 0;
    this.currentSpikedIterations = 0;
    this.leftCurrentSum = 0.0;
    this.rightCurrentSum = 0.0;
    index = 0;
    isFinished = false;
    prev_overshoot_backward = false;
    prev_overshoot_forward = false;
    this.currentSpikePos =
        NetworkTableInstance.getDefault().getTable("Path Plan").getEntry("obs_pos");
    this.currentSpike =
        NetworkTableInstance.getDefault().getTable("Path Plan").getEntry("obs_curr_spike");
    addRequirements(m_tracks, m_vive, m_gyro, m_manager);
  }

  @Override
  public void initialize() {
    addRequirements(m_tracks, m_vive, m_gyro, m_manager);

    Position cpos = m_vive.getPos();
    m_manager.setAtDestination(false);
    m_manager.setAtPosition(false);

    // if(orientation == RobotOrientation.TRENCHER_FORWARD)
    //     left_vel = right_vel = -1000;
    // else
    //     left_vel = right_vel = 1000;

    turned = false;
    isFinished = false;

    if (path_state == PathState.DYNAMIC) {
      m_manager.setState(1);
    } else {
      m_manager.setState(0);
    }

    //Set our current heading to point backwards if we are navigating to the starting zone
    // if (orientation == RobotOrientation.TRENCHER_FORWARD) m_gyro.setYaw(Position.positiveDegrees(m_gyro.getYaw() - 180));

    //Set target position on initialization so we get good behavior
    //when getting target position at beginning of commmand
    m_manager.setTargetPosition(cpos);
    target = cpos;

    if (path_state == PathState.STATIC) path = m_manager.getFinalPath();

    index = 1;

    if (dir == PathDirection.BACKWARD) index = path.size() - 2;
  }

  /**
   * This execute loop is an interesting merge of DriveForward and TurnDegrees Most of the turning
   * logic is merged with the velocity logic TurnDegrees init is used to ensure we are turning a
   * shortest distance
   *
   * <p>We first check if our current target position is not the same as our manager's target.
   * Manager's target will update once we reach our target position When we reach a position, we can
   * assume some lag to get a new Position when we set manager's atPosition to be true This check
   * will only update our new position and re-initialize driving and turning These two values will
   * only ever be different on when we are at a target position
   *
   * <p>Then, the velocity of both tracks are then set based on if the robot is at its destination
   * or if its current heading is off target
   */
  @Override
  public void execute() {

    current = m_vive.getPos();
    cHeading = Position.positiveDegrees(m_gyro.getYaw());

    // Check for an obstacle by seeing if the current spikes above a threshold
    // boolean currentIsSpiking =
    //     checkCurrentSpike() && (leftCurrentIsSpiking() || rightCurrentIsSpiking()) && !(overshoot_forward || overshoot_backward);

    boolean currentIsSpiking =
        (leftCurrentIsSpiking() || rightCurrentIsSpiking())
            && !(overshoot_forward || overshoot_backward);
    // if (currentIsSpiking) {
    //   currentSpikedIterations++;
    // } else {
    //   this.currentSpikedIterations = 0;
    // }

    // if (this.currentSpikedIterations > SPIKED_ITERATIONS) {
    if (currentIsSpiking) {
      // update the display to true in there is a prolnged spike
      Constants.OBSTACLE_DETECTED_BY_CURRENT.setBoolean(true);
      currentSpike.setBoolean(true);

      double x = current.getX() + Constants.OBJ_DIST * Math.cos(cHeading * Math.PI / 180);
      double y = current.getY() + Constants.OBJ_DIST * Math.sin(cHeading * Math.PI / 180);
      currentSpikePos.setDoubleArray(new double[] {x, y});
      // this.currentSpikedIterations = 0;
    } else {
      currentSpike.setBoolean(false);
    }
    if (orientation == RobotOrientation.TRENCHER_FORWARD)
      cHeading = Position.positiveDegrees(cHeading - 180);
    // cHeading = m_vive.getYaw();

    if ((path_state == PathState.STATIC) && ((index >= path.size()) || (index < 0))) {
      isFinished = true;
      return;
    }
    if ((path_state == PathState.DYNAMIC) && m_manager.getAtDestination()) {
      isFinished = true;
      return;
    } else {
      if (path_state == PathState.STATIC) newTarget = path.get(index);
      else if (path_state == PathState.DYNAMIC) {
        newTarget = m_manager.getTargetPosition();
      }

      if (newTarget.getX() != target.getX() || newTarget.getY() != target.getY()) {
        m_manager.setAtPosition(false);
        target = newTarget;
        // left_vel = 1000;
        // right_vel = 1000;

        turned = false;
      }

      overshoot_forward =
          (dir == PathDirection.FORWARD && path_state != PathState.POSITION)
              && ((!prev_overshoot_forward && (target.getX() < (current.getX() - boundRadius)))
                  || (prev_overshoot_forward
                      && (target.getX()
                          < (current.getX()
                              + 1)))); // && (Math.abs(target.getX() - current.getX()) > Math.abs(target.getY() - current.getY()));
      overshoot_backward =
          (dir == PathDirection.BACKWARD && path_state != PathState.POSITION)
              && ((!prev_overshoot_backward && (target.getX() > (current.getX() + boundRadius)))
                  || (prev_overshoot_backward
                      && (target.getX()
                          > (current.getX()
                              - 1)))); // && (Math.abs(target.getX() - current.getX()) > Math.abs(target.getY() - current.getY()));

      // boolean overshoot_forward = false;
      // boolean overshoot_backward = false;

      tHeading = Position.positiveDegrees(m_vive.getPos().cartesianToPolarDegrees(target));

      if (overshoot_forward) {
        tHeading = Position.positiveDegrees(tHeading - 180);
      }
      if (overshoot_backward) {
        tHeading = Position.positiveDegrees(tHeading + 180);
      }

      heading_diff = getTurnDirection(tHeading, cHeading);
      turn_dir = Math.signum(heading_diff) >= 0;

      tDistance = current.cartesianToPolarDistance(target);

      double bound_angle = 15 / Math.log10(tDistance + 3);

      turned = Math.abs(heading_diff) >= bound_angle ? false : true;

      /**
       * Velocity Logic Allows for auto course correcting the velocity of the robot Correct to the
       * correct calculated heading, DOES NOT recalculate heading based on current position
       */
      if (((path_state == PathState.STATIC || path_state == PathState.POSITION)
              && (tDistance <= boundRadius))
          || ((path_state == PathState.DYNAMIC) && (m_manager.getAtPosition()))) {
        left_vel = right_vel = 0.0;
        m_manager.setAtPosition(true);
        m_tracks.moveVelocityControl(left_vel, right_vel);
        if (dir == PathDirection.FORWARD) index++;
        else index--;

        if (path_state == PathState.POSITION) {
          isFinished = true;
          return;
        }
      } else if (!turned) {
        // left_vel = right_vel = 1000;
        int turn_vel = 900;

        if (Math.abs(heading_diff) <= 20) turn_vel = 700;
        if (Math.abs(heading_diff) <= 10) turn_vel = 650;
        if (Math.abs(heading_diff) <= 5) turn_vel = 400;

        m_tracks.setTurnVelocities(turn_vel, turn_vel, turn_dir);
      } else {

        int left_vel_temp = 1000;
        int right_vel_temp = 1000;

        if (tDistance < 15) {
          left_vel_temp = 850;
          right_vel_temp = 850;
        }
        if (tDistance < 5) {
          left_vel_temp = 725;
          right_vel_temp = 725;
        }

        if ((orientation == RobotOrientation.TRENCHER_FORWARD
                && (!overshoot_backward && !overshoot_forward))
            || ((overshoot_forward || overshoot_backward)
                && orientation == RobotOrientation.HOPPER_FORWARD)) {
          left_vel_temp = -1 * left_vel_temp;
          right_vel_temp = -1 * right_vel_temp;
        }

        left_vel = left_vel_temp * (1 - (Math.sin(Math.toRadians((-1 * heading_diff / 1)))));
        right_vel = right_vel_temp * (1 - (Math.sin(Math.toRadians(heading_diff / 1))));

        if ((orientation == RobotOrientation.TRENCHER_FORWARD
                && (!overshoot_backward && !overshoot_forward))
            || ((overshoot_forward || overshoot_backward)
                && orientation == RobotOrientation.HOPPER_FORWARD)) {
          left_vel = left_vel_temp * (1 - (Math.sin(Math.toRadians((heading_diff / 1)))));
          right_vel = right_vel_temp * (1 - (Math.sin(Math.toRadians(-1 * heading_diff / 1))));
        }
        m_tracks.moveVelocityControl(left_vel, right_vel);

        // Basically only calculate the average if the iterations are less than the Constant ITERATIONS
        // if (!checkCurrentSpike()) {
        //   iterations++;
        //   leftCurrentSum += m_tracks.leftCurrentDraw();
        //   rightCurrentSum += m_tracks.rightCurrentDraw();
        // }

        prev_overshoot_forward = overshoot_forward;
        prev_overshoot_backward = overshoot_backward;
      }

      // SmartDashboard.putNumber("lvel", left_vel);
      // SmartDashboard.putNumber("rvel", right_vel);
      // SmartDashboard.putBoolean("turned?", turned);
      // SmartDashboard.putBoolean("forward_overshoot", overshoot_forward);
      // SmartDashboard.putBoolean("backward_overshott?", overshoot_backward);
      // SmartDashboard.putNumber("Turning To", tHeading);
      // SmartDashboard.putNumber("Target Distance", tDistance);
      // SmartDashboard.putNumber("heading diff", heading_diff);
      // SmartDashboard.putString(
      //     "Target Position", "Going to " + target.getX() + ", " + target.getY());

      Constants.lvel.setDouble(left_vel);
      Constants.rvel.setDouble(right_vel);
      Constants.network_turned.setBoolean(turned);
      Constants.f_overshoot.setBoolean(overshoot_forward);
      Constants.b_overshoot.setBoolean(overshoot_backward);
      Constants.turning_to.setDouble(tHeading);
      Constants.target_distance.setDouble(tDistance);
      Constants.network_heading_diff.setDouble(heading_diff);
      Constants.netowrk_target_pos.setString("Going to " + target.getX() + ", " + target.getY());
    }
  }

  /**
   * based on: https://math.stackexchange.com/questions/110080/shortest-way-to-achieve-target-angle
   *
   * <p>Gets the shortest distance from the current heading to a target heading
   *
   * @param target The normalized target angle
   * @param base The normalized current angle
   * @return An angle that is not normalized representing the shortest distance from base to target
   *     A positive return represents an increasing motion A negative return represents a decreasing
   *     motion
   */
  private static double getTurnDirection(double target, double base) {
    double a = target - base;
    double b = target - base + 360;
    double y = target - base - 360;

    double[] obj = new double[] {a, b, y};
    double[] abs = new double[] {Math.abs(a), Math.abs(b), Math.abs(y)};

    int index = 0;
    //find the index of the smallest of the three absolute values
    //Loop only runs twice, runtime is fine
    for (int i = 1; i < 3; i++) {
      if (abs[i] < abs[index]) index = i;
    }

    return obj[index];
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

  @Override
  public void end(boolean interrupted) {
    // if (orientation == RobotOrientation.TRENCHER_FORWARD) m_gyro.setYaw(Position.positiveDegrees(m_gyro.getYaw() + 180));
    m_tracks.enableDisabledControl();
    m_manager.setState(0);
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }

  private boolean checkCurrentSpike() {
    return this.iterations > ITERATIONS;
  }

  private boolean leftCurrentIsSpiking() {
    // return this.m_tracks.leftCurrentDraw()
    //     > (this.leftCurrentSum / this.iterations) * CURRENT_SPIKE;

    return this.m_tracks.leftCurrentDraw() > CURRENT_SPIKE;
  }

  private boolean rightCurrentIsSpiking() {
    // return this.m_tracks.rightCurrentDraw()
    //     > (this.rightCurrentSum / this.iterations) * CURRENT_SPIKE;

    return this.m_tracks.rightCurrentDraw() > CURRENT_SPIKE;
  }
}
