//https://patorjk.com/software/taag/#p=display&f=ANSI%20Regular&t=Type%20Something%20
//Use that link for generating ASCII text for headers

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {

  //-------------------------------------------------------------------
  //-------------------------------------------------------------------
  //-------------------------------------------------------------------
  //-------------------- CONFIGURATION CONSTANTS ----------------------
  //-------------------------------------------------------------------
  //-------------------------------------------------------------------
  //-------------------------------------------------------------------

  public static final int DRIVER_XBOX_CONTROLLER_PORT = 0;
  public static final String ROBOT_NAME = "ATLAS";

  public static final int END_X = 200;

  public static final int ARENA_WIDTH = 45; // 98; // inches rounded down
  public static final int ARENA_HEIGHT = 45; // 174; // inches rounded u p

  public static final int DEFAULT_OBJECT_HEIGHT = 3; //40;//26;
  public static final int DEFAULT_OBJECT_WIDTH = 3; //40;//26;

  public static final int LIDAR_DEGREES = 10;

  //These specify the offset of the lidar from the center of the robot.
  public static final int LIDAR_X_SHIFT = 21;
  public static final int LIDAR_Y_SHIFT = 6;

  //-------------------------------------------------------------------
  //-------------------------------------------------------------------
  //-------------------------------------------------------------------
  //-------------------- TRAVERSAL CONSTANTS --------------------------
  //-------------------------------------------------------------------
  //-------------------------------------------------------------------
  //-------------------------------------------------------------------

  public static final int STARTING_X = 65;
  public static final int STARTING_Y = -8;

  public static final int X_TOLERANCE = 4;
  public static final int Y_TOLERANCE = 4;

  public static final double ERROR_PERCENTAGE = 0.09;

  public static final int OBJ_DIST = 19;

  /**
   * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2 or 3. Only the
   * first two (0,1) are visible in web-based configuration.
   */
  public static final int kSlotIdx = 0;

  /** Talon FX supports multiple (cascaded) PID loops. For now we just want the primary one. */
  public static final int kPIDLoopIdx = 0;

  /**
   * set to zero to skip waiting for confirmation, set to nonzero to wait and report to DS if action
   * fails.
   */
  public static final int kTimeoutMs = 30;

  //-------------------------------------------------------------------
  //-------------------------------------------------------------------
  //-------------------------------------------------------------------
  //-------------------- COLLECTION CONSTANTS --------------------------
  //-------------------------------------------------------------------
  //-------------------------------------------------------------------
  //-------------------------------------------------------------------

  // TRENCHER
  public static final int TRENCH_MOTOR_ID = 3;
  /**
   * Gains used in Motion Magic, to be adjusted accordingly Gains(kp, ki, kd, kf, izone, peak
   * output);
   */
  public static final Gains TrencherGains = new Gains(0.0, 0, 0, 0.06, 0, 1.0);

  public static final double TRENCH_CURR_THRESH = 30;
  public static final int TRENCH_MINING_VELOCITY = 17500; //tick per 0.1s (or ms?)
  public static final int TRENCH_RECOVER_VELOCITY = 17500 / 2;

  public static final int TRENCH_JAM_VEL_LIMIT = 4000;
  public static final int TRENCH_JAM_DURATION = 500; //ms units

  // SECTOR
  public static final int SECTOR_MOTOR_ID = 4;
  /**
   * Gains used in Motion Magic, to be adjusted accordingly Gains(kp, ki, kd, kf, izone, peak
   * output);
   */
  public static final Gains SectorGains = new Gains(0, 0, 0, 0.2, 0, 1.0);

  public static final int TRENCH_UP_LIMIT = 0;
  public static final int TRENCH_DOWN_LIMIT = 1;
  public static final int SECTOR_ENCODER_FLOOR = -67000;
  // Tolerance is in encoder ticks
  public static final int SECTOR_ENCODER_TOLERANCE = 50;

  public static final double SECTOR_ZEROING_PERCENTAGE = 0.3;
  public static final double SECTOR_HOMING_PERCENTAGE = 0.3;

  public static final int SECTOR_ENCODER_DEG = 4856; //1168;
  public static final int SECTOR_HORIZONTAL_OFFSET = 0;
  public static final int SECTOR_PLUNGE_DEPTH = 155; //205;

  //Current thresholds for plunging
  public static final double TRENCH_SAFE_CURRENT_THRESH = 30;
  public static final double TRENCH_UNSAFE_CURRENT_THRESH = 80;

  // These numbers represent Velocity in degrees per second
  public static final double SECTOR_RAPID_VELOCITY = 2;
  public static final double SECTOR_MINING_VELOCITY = 0.4;
  public static final double SECTOR_LIGHT_MINING_VELOCITY = 0.7;
  public static final double SECTOR_RECOVER_MINING_VELOCITY = 1;

  // Converts the Velocities from degrees per second to encoder ticks per 100ms as needed by motion magic
  public static final int SECTOR_CALC_RAPID_VELOCITY =
      (int) ((SECTOR_RAPID_VELOCITY * SECTOR_ENCODER_DEG) * 0.1);
  public static final int SECTOR_CALC_MINING_VELOCITY =
      (int) ((SECTOR_MINING_VELOCITY * SECTOR_ENCODER_DEG) * 0.1);
  public static final int SECTOR_CALC_LIGHT_MINING_VELOCITY =
      (int) ((SECTOR_LIGHT_MINING_VELOCITY * SECTOR_ENCODER_DEG) * 0.1);
  public static final int SECTOR_CALC_RECOVER_MINING_VELOCITY =
      (int) ((SECTOR_RECOVER_MINING_VELOCITY * SECTOR_ENCODER_DEG) * 0.1);

  public static final int TRENCHING_TIME = 45000;
  public static final int PLUNGE_TIME = 90000;
  public static final int GRAVEL_TIME = 20000;

  public static final int RECOVERY_RAISE_TIME = 1000;
  public static final int RECOVERY_TEST_TIME = 2500;

  //-------------------------------------------------------------------
  //-------------------------------------------------------------------
  //-------------------------------------------------------------------
  //----------------------- TRACK CONSTANTS ---------------------------
  //-------------------------------------------------------------------
  //-------------------------------------------------------------------
  //-------------------------------------------------------------------
  public static final int GYRO_ID = 5;

  public static final int LEFT_DRIVE_MOTOR_ID = 1;
  public static final int RIGHT_DRIVE_MOTOR_ID = 2;
  // Tolerance is in encoder ticks
  public static final int TRACK_ENCODER_TOLERANCE = 50;

  //These values are for HARDFLOOR
  public static final int DEGREES_TO_ENCODER = 1700;
  // Distance is in inches.
  public static final int DISTANCE_TO_ENCODER = 8700;

  public static final int TRENCHING_SPEED = 250;

  public static final int TRACK_RECOVERY_TIME = 750;

  public static final int TRACK_SAFE_CURRENT_THRESH = 15;

  public static final int MINING_FORWARD_SCOOT = 220000;
  public static final int MINING_TRENCH_LENGTH = 200000;
  public static final int MINING_RETURN_DIST = MINING_FORWARD_SCOOT - MINING_TRENCH_LENGTH;
  //These values are for SAND
  //public static final int DEGREES_TO_ENCODER = 1760;
  // Distance is in inches.
  //public static final int DISTANCE_TO_ENCODER = 8400;

  /**
   * Gains used in Motion Magic, to be adjusted accordingly Gains(kp, ki, kd, kf, izone, peak
   * output);
   */
  public static final Gains kGains = new Gains(0.0, 0, 0.0, 0.2, 0, 1.0);

  //-------------------------------------------------------------------
  //-------------------------------------------------------------------
  //-------------------------------------------------------------------
  //----------------------- HOPPER CONSTANTS --------------------------
  //-------------------------------------------------------------------
  //-------------------------------------------------------------------
  //-------------------------------------------------------------------
  public static final int HOPPER_MOTOR_ID = 0;
  public static final Gains hopperGains = new Gains(0.0, 0.0, 0.0, 0.2, 0, 1.0);

  public static final int LINEAR_ACTUATOR_MOTOR_CONTROLLER_ID = 6;
  public static final int LINEAR_ACTUATOR_MOTOR_ENCODER_CHANNEL = 2;
  // Velocities in Encoder Ticks per 100ms
  public static final int HOPPER_BELT_VELOCITY = 750;
  public static final int HOPPER_BELT_VELOCITY_MINING = 500;
  public static final int OFFLOAD_HOPPER_TIME = 26_000;

  public static final int HOPPER_TRENCHING_BELT_DIST = 300000; // encoder ticks

  public static final double YAW_TOLERANCE = 0.15;
  public static final double TRACK_VELOCITY = 1000;
  public static final double SLOW_TRACK_VELOCITY = 300;
  public static final double TURN_VELOCITY = 1000;
  public static final double TRACK_ACCELERATION = 100;

  public static final boolean LEFT_INVERTED = true;
  public static final boolean RIGHT_INVERTED = false;

  public static final double HOPPER_TIMOUT_MS =
      8_500; // Time it takes to raise/lower the hopper in miliseconds

  /** TRAVERSE PATH NETWORK TABLES */
  public static ShuffleboardTab traverse_tab = Shuffleboard.getTab("Traversal");

  public static NetworkTableEntry lvel = traverse_tab.add("lvel", 0).getEntry();
  public static NetworkTableEntry rvel = traverse_tab.add("rvel", 0).getEntry();
  public static NetworkTableEntry network_turned = traverse_tab.add("turned?", false).getEntry();
  public static NetworkTableEntry f_overshoot =
      traverse_tab.add("forward overshoot", false).getEntry();
  public static NetworkTableEntry b_overshoot =
      traverse_tab.add("backward overshoot", false).getEntry();
  public static NetworkTableEntry turning_to = traverse_tab.add("Turning To", 0).getEntry();
  public static NetworkTableEntry target_distance =
      traverse_tab.add("Target Distance", 0).getEntry();
  public static NetworkTableEntry network_heading_diff =
      traverse_tab.add("Heading Diff", 0).getEntry();
  public static NetworkTableEntry netowrk_target_pos =
      traverse_tab.add("Target Position", "temp").getEntry();
  public static NetworkTableEntry OBSTACLE_DETECTED_BY_CURRENT =
      traverse_tab.add("Obstacle detected by current", false).getEntry();
}
