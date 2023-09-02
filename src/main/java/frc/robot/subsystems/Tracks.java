package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.helpers.EncoderQueue;
import frc.robot.helpers.Path;
import frc.robot.helpers.Position;

public class Tracks extends SubsystemBase {

  private final WPI_TalonFX left = new WPI_TalonFX(Constants.LEFT_DRIVE_MOTOR_ID);
  private final WPI_TalonFX right = new WPI_TalonFX(Constants.RIGHT_DRIVE_MOTOR_ID);
  private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  //Shuffleboard for setting PID
  private ShuffleboardTab tab = Shuffleboard.getTab("PID");
  private ShuffleboardTab trackTab = Shuffleboard.getTab("Track");
  private ShuffleboardTab miningTab = Shuffleboard.getTab("Mining");

  private double leftPercentage = 0;
  private double rightPercentage = 0;
  private double leftCurrent = 0;
  private double rightCurrent = 0;
  private double leftEncoder = 0;
  private double rightEncoder = 0;
  private double leftSpeed = 0;
  private double rightSpeed = 0;
  private double accelerometerX = 0;
  private double accelerometerY = 0;
  private double accelerometerZ = 0;

  private double heading = 0;
  private Position trackPosition = new Position(0, 0);
  private Position trackGoal = new Position(0, 0);
  private Path goalPositions;

  private double leftTarget = 0;
  private double rightTarget = 0;

  private EncoderQueue leftQueue;
  private EncoderQueue rightQueue;

  //Add all the sensor values to the shuffleboard track tab
  private NetworkTableEntry network_leftEncoder =
      trackTab.add("Left Encoder Value", leftEncoder).getEntry();
  private NetworkTableEntry network_rightEncoder =
      trackTab.add("Right Encoder Value", rightEncoder).getEntry();
  private NetworkTableEntry network_heading = trackTab.add("Heading Tab", heading).getEntry();
  private NetworkTableEntry network_leftTarget = trackTab.add("Left Target", leftTarget).getEntry();
  private NetworkTableEntry network_rightTarget =
      trackTab.add("Right Target", rightTarget).getEntry();
  private NetworkTableEntry network_goalPositions =
      trackTab.add("Position Queue Size", 0).getEntry();
  private NetworkTableEntry network_leftQueue = trackTab.add("Left Queue Size", 0).getEntry();
  private NetworkTableEntry network_rightQueue = trackTab.add("Right Queue Size", 0).getEntry();

  //Add specific values to Mining Tab
  private NetworkTableEntry mining_left = miningTab.add("Left Encoder", leftEncoder).getEntry();
  private NetworkTableEntry mining_right = miningTab.add("Right Encoder", rightEncoder).getEntry();
  private NetworkTableEntry mining_left_cur = miningTab.add("Left Current", leftCurrent).getEntry();
  private NetworkTableEntry mining_right_cur =
      miningTab.add("Right Current", rightCurrent).getEntry();

  //Add all the pid values to the shuffleboard pid tab.
  private NetworkTableEntry TracksPIDP = tab.add("PID P", Constants.kGains.kP).getEntry();
  private NetworkTableEntry TracksPIDI = tab.add("PID I", Constants.kGains.kI).getEntry();
  private NetworkTableEntry TracksPIDD = tab.add("PID D", Constants.kGains.kD).getEntry();

  public Tracks() {
    goalPositions = new Path();
    leftQueue = new EncoderQueue();
    rightQueue = new EncoderQueue();
    talonInit(left);
    talonInit(right);
    left.setInverted(Constants.LEFT_INVERTED);
    right.setInverted(Constants.RIGHT_INVERTED);
  }

  @Override
  public void periodic() {
    super.periodic();
    // This method will be called once per scheduler run
    this.leftEncoder = left.getSensorCollection().getIntegratedSensorPosition();
    this.rightEncoder = right.getSensorCollection().getIntegratedSensorPosition();
    this.leftSpeed = left.getSensorCollection().getIntegratedSensorVelocity();
    this.rightSpeed = right.getSensorCollection().getIntegratedSensorVelocity();

    this.accelerometerX = accelerometer.getX();
    this.accelerometerY = accelerometer.getY();
    this.accelerometerZ = accelerometer.getZ();

    network_leftEncoder.setDouble(leftEncoder);
    network_rightEncoder.setDouble(rightEncoder);
    network_heading.setDouble(heading);
    network_leftTarget.setDouble(leftTarget);
    network_rightTarget.setDouble(rightTarget);
    network_goalPositions.setDouble(goalPositions.size());
    network_leftQueue.setDouble(leftQueue.size());
    network_rightQueue.setDouble(rightQueue.size());

    mining_left.setDouble(leftEncoder);
    mining_right.setDouble(rightEncoder);
    mining_left_cur.setDouble(left.getStatorCurrent());
    mining_right_cur.setDouble(right.getStatorCurrent());

    double kP = TracksPIDP.getDouble(Constants.kGains.kP);
    double kI = TracksPIDI.getDouble(Constants.kGains.kI);
    double kD = TracksPIDD.getDouble(Constants.kGains.kD);

    left.config_kP(Constants.kSlotIdx, kP, Constants.kTimeoutMs);
    left.config_kI(Constants.kSlotIdx, kI, Constants.kTimeoutMs);
    left.config_kD(Constants.kSlotIdx, kD, Constants.kTimeoutMs);

    right.config_kP(Constants.kSlotIdx, kP, Constants.kTimeoutMs);
    right.config_kI(Constants.kSlotIdx, kI, Constants.kTimeoutMs);
    right.config_kD(Constants.kSlotIdx, kD, Constants.kTimeoutMs);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation

  }

  public void setDriveStates(TrapezoidProfile.State lState, TrapezoidProfile.State rState) {
    moveVelocityControl(lState.velocity, rState.velocity);
  }

  public void setTurnVelocities(double lVel, double rVel, boolean clockwise) {
    if (clockwise) {
      moveVelocityControl(lVel, -rVel);
    } else {
      moveVelocityControl(-lVel, rVel);
    }
  }

  public void setInvertedControl(Boolean left, Boolean right) {
    this.left.setInverted(left);
    this.right.setInverted(right);
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public void resetEncoderTargets() {
    leftTarget = leftEncoder;
    rightTarget = rightEncoder;
  }

  public void resetQueues() {
    goalPositions.clear();
    leftQueue.clear();
    rightQueue.clear();
  }

  public void moveMotionMagic() {
    left.set(ControlMode.MotionMagic, leftTarget);
    right.set(ControlMode.MotionMagic, rightTarget);
  }

  public void moveMotionMagic(double leftTarget, double rightTarget) {
    this.leftTarget = leftTarget;
    this.rightTarget = rightTarget;
    left.set(ControlMode.MotionMagic, leftTarget);
    right.set(ControlMode.MotionMagic, rightTarget);
  }

  public void movePercentageControl() {
    left.set(ControlMode.PercentOutput, leftPercentage);
    right.set(ControlMode.PercentOutput, rightPercentage);
  }

  public void movePercentageControl(double leftPercentage, double rightPercentage) {
    this.leftPercentage = leftPercentage;
    this.rightPercentage = rightPercentage;
    left.set(ControlMode.PercentOutput, this.leftPercentage);
    right.set(ControlMode.PercentOutput, this.rightPercentage);
  }

  public void moveCurrentControl() {
    left.set(ControlMode.Current, leftCurrent);
    right.set(ControlMode.Current, rightCurrent);
  }

  public void moveCurrentControl(double leftCurrent, double rightCurrent) {
    this.leftCurrent = leftCurrent;
    this.rightCurrent = rightCurrent;
    left.set(ControlMode.Current, leftCurrent);
    right.set(ControlMode.Current, rightCurrent);
  }

  public void moveVelocityControl(double leftVel, double rightVel) {
    left.set(ControlMode.Velocity, leftVel);
    right.set(ControlMode.Velocity, rightVel);
  }

  public void enableDisabledControl() {
    left.set(ControlMode.Disabled, 0);
    right.set(ControlMode.Disabled, 0);
  }

  public void updateTrackCurrent(double leftTrack, double rightTrack) {
    leftCurrent = leftTrack;
    rightCurrent = rightTrack;
  }

  public void updateTrackTargets(double leftTrack, double rightTrack) {
    leftTarget = leftTrack;
    rightTarget = rightTrack;
  }

  public void addTrackTarget(double leftTrack, double rightTrack) {
    leftTarget += leftTrack;
    rightTarget += rightTrack;
  }

  public void addPathSetpoint(Position p) {
    goalPositions.add(p);
  }

  public Position getNextPositionSetpoint() {
    trackGoal = goalPositions.poll();
    return trackGoal;
  }

  public void updateCurrentPosition(Position p) {
    trackPosition = p;
  }

  public void updateHeading(double theta) {
    heading = theta;
  }

  public double getLeftEncoder() {
    return leftEncoder;
  }

  public double getRightEncoder() {
    return rightEncoder;
  }

  // Returns true if the motor current drawn is over the given current
  public double leftCurrentDraw() {
    return left.getStatorCurrent();
  }

  // Returns true if the motor current drawn is over the given current
  public double rightCurrentDraw() {
    return right.getStatorCurrent();
  }

  //This just calculates the relative encoder movement. Absolute is done at time of popping from queue.
  public void addEncoderToQueue(Position p) {

    //Add the rotation first
    double theta = trackPosition.angle2DDeg(p);

    //Find the shortest rotation
    double oppTheta = 360 - theta;

    if (Math.abs(theta) > Math.abs(oppTheta)) {
      double diffTheta = heading - oppTheta;

      if (diffTheta != 0) {
        rightQueue.add(-1 * diffTheta * Constants.DEGREES_TO_ENCODER);
        leftQueue.add(-1 * diffTheta * Constants.DEGREES_TO_ENCODER);
        heading = heading + diffTheta;
      }
    } else {

      double diffTheta = theta - heading;

      if (diffTheta != 0) {
        rightQueue.add(-1 * diffTheta * Constants.DEGREES_TO_ENCODER);
        leftQueue.add(-1 * diffTheta * Constants.DEGREES_TO_ENCODER);
        heading = heading + diffTheta;
      }
    }

    //Add the distance second

    double distance = trackPosition.euclideanDistance2D(p);

    // SmartDashboard.putNumber("Distance", distance);

    rightQueue.add(-1 * distance * Constants.DISTANCE_TO_ENCODER);
    leftQueue.add(distance * Constants.DISTANCE_TO_ENCODER);
  }

  public boolean updateEncoderSetpoints() {

    if (leftQueue.isEmpty() || rightQueue.isEmpty()) {
      return false;
    }

    Double leftTemp = leftQueue.poll();
    Double rightTemp = rightQueue.poll();

    if (leftTemp == null || rightTemp == null) {
      leftTarget = leftEncoder;
      rightTarget = rightEncoder;
      return false;
    }

    leftTarget = leftEncoder + leftTemp;
    rightTarget = rightEncoder + rightTemp;

    return true;
  }

  public boolean moveWithPosition(Position absolutePosition) {

    addEncoderToQueue(absolutePosition);

    if (updateEncoderSetpoints()) {
      moveMotionMagic();
      return true;
    }
    //If updateEncoderSetpoints returns false, the queue is in an unknown state and we want to clear it.
    rightQueue.clear();
    leftQueue.clear();
    return false;
  }

  public boolean atGoalPosition() {

    double diffLeft = leftTarget - leftEncoder;
    double diffRight = rightTarget - rightEncoder;

    double absDiffLeft = Math.abs(diffLeft);
    double absDiffRight = Math.abs(diffRight);

    if (absDiffRight <= Constants.TRACK_ENCODER_TOLERANCE
        && absDiffLeft <= Constants.TRACK_ENCODER_TOLERANCE) {
      return true;
    } else {
      return false;
    }
  }

  public boolean atGoalPosition(double tolerance) {

    double diffLeft = leftTarget - leftEncoder;
    double diffRight = rightTarget - rightEncoder;

    double absDiffLeft = Math.abs(diffLeft);
    double absDiffRight = Math.abs(diffRight);

    if (absDiffRight <= tolerance && absDiffLeft <= tolerance) {
      return true;
    } else {
      return false;
    }
  }

  public void setTrackPosToGoal() {
    trackPosition = trackGoal;
  }

  public boolean isPathEmpty() {
    return goalPositions.isEmpty();
  }

  public boolean encoderQueuesEmpty() {
    return leftQueue.isEmpty() || rightQueue.isEmpty();
  }

  public void setTalonVelocity(double velocity) {
    left.configMotionCruiseVelocity(velocity, Constants.kTimeoutMs);
    // left.configMotionAcceleration(acceleration, Constants.kTimeoutMs);
    right.configMotionCruiseVelocity(velocity, Constants.kTimeoutMs);
    // right.configMotionAcceleration(acceleration, Constants.kTimeoutMs);
  }

  private void talonInit(WPI_TalonFX _talon) {
    _talon.configFactoryDefault();

    /* Configure Sensor Source for Pirmary PID */
    _talon.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    /* set deadband to super small 0.001 (0.1 %).
    The default deadband is 0.04 (4 %) */
    _talon.configNeutralDeadband(0.001, Constants.kTimeoutMs);

    /**
     * Configure Talon FX Output and Sesnor direction accordingly Invert Motor to have green LEDs
     * when driving Talon Forward / Requesting Postiive Output Phase sensor to have positive
     * increment when driving Talon Forward (Green LED)
     */
    _talon.setSensorPhase(false);
    _talon.setInverted(false);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    _talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    _talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

    /* Set the peak and nominal outputs */
    _talon.configNominalOutputForward(0, Constants.kTimeoutMs);
    _talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
    _talon.configPeakOutputForward(1, Constants.kTimeoutMs);
    _talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
    _talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    _talon.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
    _talon.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
    _talon.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    _talon.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    _talon.configMotionCruiseVelocity(Constants.TRACK_VELOCITY, Constants.kTimeoutMs);
    _talon.configMotionAcceleration(Constants.TRACK_ACCELERATION, Constants.kTimeoutMs);

    /* Zero the sensor once on robot boot up */
    _talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  }

  // @Override
  // protected void useState(State state) {

  //     moveVelocityControl(-state.velocity, state.velocity);
  // }
}
