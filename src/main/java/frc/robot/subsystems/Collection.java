package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.helpers.EncoderQueue;
import frc.robot.helpers.Position;
import java.util.ArrayList;

public class Collection extends SubsystemBase {

  //Trencher and Sector Motor Controller Setup
  private final WPI_TalonFX trench = new WPI_TalonFX(Constants.TRENCH_MOTOR_ID);
  private final WPI_TalonFX sector = new WPI_TalonFX(Constants.SECTOR_MOTOR_ID);

  //Sector Arc Limit Switches
  private final DigitalInput upLimit = new DigitalInput(Constants.TRENCH_UP_LIMIT);
  private final DigitalInput downLimit = new DigitalInput(Constants.TRENCH_DOWN_LIMIT);

  private ShuffleboardTab PID_tab = Shuffleboard.getTab("PID");
  private ShuffleboardTab miningTab = Shuffleboard.getTab("Mining");

  private double sectorEncoder = 0;
  private double sectorAngle = 0;
  public double sectorTarget = 0;
  private double sectorCurrent = 0;

  private double sectorRelative = 0;

  private double trencherEncoder = 0;
  private double trencherSpeed = 0;
  private double trencherCurrent = 0;
  private double trencherVoltage = 0;

  private double busVoltage = 0;

  // private double curAngle = 0;
  private EncoderQueue sectorQueue;

  private boolean hasZeroed = false;
  private boolean trencherIsJammed = false;

  private ArrayList<Double> trencherCurrentList = new ArrayList<Double>();
  private int movingAvgRange = 10;
  private double trench_avg_current = 0.0;

  private Position miningStartPosition;
  private int sortieCount = 0;

  //Mining Tab Network Tables
  private NetworkTableEntry network_sectorEncoder =
      miningTab.add("Sector Encoder", sectorEncoder).getEntry();
  private NetworkTableEntry network_sectorCurrent =
      miningTab.add("Sector Current", sectorCurrent).getEntry();
  private NetworkTableEntry network_trencherSpeed =
      miningTab.add("Trencher Speed", trencherSpeed).getEntry();
  private NetworkTableEntry network_trencherCurrent =
      miningTab.add("Trencher Current", trencherCurrent).getEntry();
  private NetworkTableEntry network_trencherVoltage =
      miningTab.add("Trencher Voltage", trencherVoltage).getEntry();
  private NetworkTableEntry network_upLimit =
      miningTab.add("Trencher Up", upLimit.get()).getEntry();
  private NetworkTableEntry network_downLimit =
      miningTab.add("Trencher Down", !downLimit.get()).getEntry();
  private NetworkTableEntry network_busVoltage =
      miningTab.add("Bus Voltage", busVoltage).getEntry();
  private NetworkTableEntry network_isJammed =
      miningTab.add("Trancher Jammed", trencherIsJammed).getEntry();

  //PID Tab Network Tables
  private NetworkTableEntry TrencherPIDP =
      PID_tab.add("Trencher P", Constants.TrencherGains.kP).getEntry();
  private NetworkTableEntry TrencherPIDI =
      PID_tab.add("Trencher I", Constants.TrencherGains.kI).getEntry();
  private NetworkTableEntry TrencherPIDD =
      PID_tab.add("Trencher D", Constants.TrencherGains.kD).getEntry();
  private NetworkTableEntry SectorPIDP =
      PID_tab.add("Sector P", Constants.SectorGains.kP).getEntry();
  private NetworkTableEntry SectorPIDI =
      PID_tab.add("Sector I", Constants.SectorGains.kI).getEntry();
  private NetworkTableEntry SectorPIDD =
      PID_tab.add("Sector D", Constants.SectorGains.kD).getEntry();

  /** Create new collection subsystem */
  public Collection() {
    sectorQueue = new EncoderQueue();

    talonInit(trench, false);
    talonInit(sector, true);
  }

  @Override
  public void periodic() {

    this.sectorEncoder = sector.getSensorCollection().getIntegratedSensorPosition();
    this.sectorAngle =
        (-1 * ((double) (sectorEncoder - sectorRelative)) / Constants.SECTOR_ENCODER_DEG);
    this.sectorCurrent = sector.getStatorCurrent();
    this.trencherEncoder = trench.getSensorCollection().getIntegratedSensorPosition();
    this.trencherSpeed = trench.getSensorCollection().getIntegratedSensorVelocity();
    this.trencherCurrent = trench.getStatorCurrent();
    this.trencherVoltage = trench.getMotorOutputVoltage();
    this.busVoltage = trench.getBusVoltage();

    network_sectorEncoder.setDouble(sectorEncoder);
    network_sectorCurrent.setDouble(sectorCurrent);
    network_trencherSpeed.setDouble(trencherSpeed);
    network_trencherCurrent.setDouble(trencherCurrent);
    network_trencherVoltage.setDouble(trencherVoltage);
    network_upLimit.setBoolean(upLimit.get());
    network_downLimit.setBoolean(!downLimit.get());
    network_busVoltage.setDouble(busVoltage);
    network_isJammed.setBoolean(getTrencherJammed());

    double tP = TrencherPIDP.getDouble(Constants.TrencherGains.kP);
    double tI = TrencherPIDI.getDouble(Constants.TrencherGains.kI);
    double tD = TrencherPIDD.getDouble(Constants.TrencherGains.kD);

    double sP = SectorPIDP.getDouble(Constants.SectorGains.kP);
    double sI = SectorPIDI.getDouble(Constants.SectorGains.kI);
    double sD = SectorPIDD.getDouble(Constants.SectorGains.kD);

    trench.config_kP(Constants.kSlotIdx, tP, Constants.kTimeoutMs);
    trench.config_kI(Constants.kSlotIdx, tI, Constants.kTimeoutMs);
    trench.config_kD(Constants.kSlotIdx, tD, Constants.kTimeoutMs);

    sector.config_kP(Constants.kSlotIdx, sP, Constants.kTimeoutMs);
    sector.config_kI(Constants.kSlotIdx, sI, Constants.kTimeoutMs);
    sector.config_kD(Constants.kSlotIdx, sD, Constants.kTimeoutMs);

    //DISABLE SECTOR IF UPPER OR LOWER LIMIT SWITCHES ARE PRESSED
    //if(upLimit.get()||sectorEncoder-sectorRelative <= Constants.SECTOR_ENCODER_FLOOR){

    SmartDashboard.putNumber("sector percentage", sector.getMotorOutputPercent());
    if (upLimit.get() && sector.getMotorOutputPercent() > 0
        || !downLimit.get() && sector.getMotorOutputPercent() < 0) {
      enableSectorDisabledControl();
    }

    //SECTOR ZEROING
    if (upLimit.get()) {
      sectorRelative = sectorEncoder;
      hasZeroed = true;
    }

    //CURRENT SIGNAL SMOOTHING
    //Trencher current signal smoothing stuff
    double new_current = getTrencherCurrent();
    trencherCurrentList.add(new_current);
    trench_avg_current = getMovingAverage(trencherCurrentList, movingAvgRange);
  }

  @Override
  public void simulationPeriodic() {}

  public void resetEncoderTargets() {
    sectorTarget = sectorEncoder;
  }

  public void resetQueues() {
    sectorQueue.clear();
  }

  public double getMiningDirection(Position startPos) {
    miningStartPosition = startPos;

    double turingDir = 0.0;

    if (sortieCount == 0) {
      turingDir = 180.0;
    }
    //turning commands if toward sieve side of arena
    else if (miningStartPosition.getY() < 0) {
      if (sortieCount == 1) {
        turingDir = 135.0;
      } else {
        turingDir = 90.0;
      }
    }
    //turning commands if toward starting zone size of arena
    else { //miningStartPosition.getY() is <= 0
      if (sortieCount == 1) {
        turingDir = 225.0;
      } else {
        turingDir = 270.0;
      }
    }

    sortieCount++;
    return turingDir;
  }

  /**
   * Translates input angle (degrees) to encoder tick count, relative to 0
   *
   * @param angle
   * @return
   */
  public double getEncoderTartgetFromAngle(double angle) {
    // return sectorRelative - ((angle + Constants.SECTOR_HORIZONTAL_OFFSET) * Constants.SECTOR_ENCODER_DEG);
    return sectorRelative - (angle * Constants.SECTOR_ENCODER_DEG);
  }

  public void moveSectorMotionMagic(int velocity) {
    boolean noLimitIssues = true;
    if (upLimit.get()) {
      //If the up limit switch is set, and we are trying to go further disable the motor.
      if (sectorTarget > sectorEncoder) {
        sector.set(ControlMode.Disabled, 0);
        noLimitIssues = false;
      }
    }

    //if(sectorEncoder-sectorRelative <= Constants.SECTOR_ENCODER_FLOOR){
    if (!downLimit.get()) {
      if (sectorTarget < sectorEncoder) {
        sector.set(ControlMode.Disabled, 0);
        noLimitIssues = false;
      }
    }

    if (noLimitIssues) {
      sector.configMotionCruiseVelocity(velocity);
      sector.set(ControlMode.MotionMagic, sectorTarget);
    }
  }

  public void moveSectorMotionMagic(double sectorTarget, int velocity) {
    this.sectorTarget = sectorTarget;
    moveSectorMotionMagic(velocity);
  }

  public void moveSectorPercentageControl(double percentage) {

    boolean noLimitIssues = true;

    if (upLimit.get()) {
      //If the up limit switch is set, and we are trying to go further disable the motor.
      if (percentage > 0) {
        sector.set(ControlMode.Disabled, 0);
        noLimitIssues = false;
      }
    }

    if (!downLimit.get()) {
      //If the down limit switch is set, and we are trying to go further disable the motor.
      if (percentage < 0) {
        sector.set(ControlMode.Disabled, 0);
        noLimitIssues = false;
      }
    }

    // If no limit switch issues have been detected, run the motor
    if (noLimitIssues) {
      sector.set(ControlMode.PercentOutput, percentage);
    }
  }

  public void moveSectorCurrentControl(double current) {
    sector.set(ControlMode.Current, current);

    boolean noLimitIssues = true;

    if (upLimit.get()) {
      //If the up limit switch is set, and we are trying to go further disable the motor.
      if (current > 0) {
        sector.set(ControlMode.Disabled, 0);
        noLimitIssues = false;
      }
    }

    //if(sectorEncoder-sectorRelative <= Constants.SECTOR_ENCODER_FLOOR){
    if (!downLimit.get()) {
      if (current < 0) {
        sector.set(ControlMode.Disabled, 0);
        noLimitIssues = false;
      }
    }

    if (noLimitIssues) {
      sector.set(ControlMode.Current, current);
    }
  }

  public void moveSectorVelocityControl(double velocity) {

    boolean noLimitIssues = true;

    // DISABLE LIMIT ISSUE CHECKS - TO BE HANDLED BY COMMANDS

    if (upLimit.get()) {
      //If the up limit switch is set, and we are trying to go further disable the motor.
      if (velocity > 0) {
        sector.set(ControlMode.Disabled, 0);
        noLimitIssues = false;
      }
    }

    if (!downLimit.get()) {
      //If the dowm limit switch is set, and we are trying to go further disable the motor.
      if (velocity < 0) {
        sector.set(ControlMode.Disabled, 0);
        noLimitIssues = false;
      }
    }

    //if there are no limit issues, and we are not yet at target, move
    if (noLimitIssues) {
      sector.set(ControlMode.Velocity, velocity);
    }
  }

  public void setSectorTalonVelocity(double velocity) {
    sector.set(ControlMode.Velocity, velocity);
  }

  public boolean getUpLimit() {
    return upLimit.get();
  }

  public boolean getDownLimit() {
    return !downLimit.get();
  }

  public void enableSectorDisabledControl() {
    sector.set(ControlMode.Disabled, 0);
  }

  public void moveTrencherVelocity(double velocity) {
    trench.set(ControlMode.Velocity, velocity);
  }

  public void moveTrencherCurrent(double current) {
    trench.set(ControlMode.Current, current);
  }

  public void moveTrencherPercentageControl(double percentage) {
    trench.set(ControlMode.PercentOutput, percentage);
  }

  public void enableTrencherDisabledControl() {
    trench.set(ControlMode.Disabled, 0);
  }

  public void updateSectorTarget(double newTarget) {
    this.sectorTarget = newTarget;
  }

  public void addSectorTarget(double targetToAdd) {
    sectorTarget += targetToAdd;
  }

  public double getSectorEncoder() {
    return sectorEncoder;
  }

  public double getSectorAngle() {
    return sectorAngle;
  }

  /**
   * Return true if current sector position is within default band of tolerance
   *
   * @return
   */
  public boolean sectorAtTarget() {
    double diffSector = sectorTarget - sectorEncoder;
    double absDiffSector = Math.abs(diffSector);

    return absDiffSector <= Constants.SECTOR_ENCODER_TOLERANCE;
  }

  /**
   * Return true of current sector position is within user-set band of tolerance
   *
   * @param tolerance user-set band of tolerance (+/- tolerance)
   * @return
   */
  public boolean sectorAtTarget(double tolerance) {
    double diffSector = sectorTarget - sectorEncoder;
    double absDiffSector = Math.abs(diffSector);

    return absDiffSector <= tolerance;
  }

  public boolean sectorAtTargetAngle(double targetAngle, double tolerance) {
    double diffSector = targetAngle - sectorAngle;
    double absDiffSector = Math.abs(diffSector);

    return absDiffSector <= tolerance;
  }

  /**
   * Returns true if the sector encoder count has been zeroed
   *
   * @return
   */
  public boolean hasZeroed() {
    return hasZeroed;
  }

  public boolean getTrencherJammed() {
    return trencherIsJammed;
  }

  public void setTrencherJammed(boolean status) {
    trencherIsJammed = status;
  }

  public double getTrencherEncoder() {
    return trencherEncoder;
  }

  public double getTrencherVelocity() {
    return trencherSpeed;
  }

  /**
   * No clue what this bs is about. Sachin Patel might know.
   *
   * @return
   */
  public boolean isEncoderQueueEmpty() {
    return sectorQueue.isEmpty();
  }

  /**
   * Gets current for trencher, updated periodically
   *
   * @return
   */
  public double getTrencherCurrent() {
    return trench.getStatorCurrent();
  }

  /**
   * Gets current for sector, updated periodically
   *
   * @return
   */
  public double getSectorCurrent() {
    return sector.getStatorCurrent();
  }

  public double getTrencherAvgCurrent() {
    return trench_avg_current;
  }

  public static double getMovingAverage(ArrayList<Double> motorDataList, int movingAvgRange) {
    //assuming that motorDataList has movingAvgRange(amount of data points we want to observe)
    //takes average of current points
    double currentAverage = 0;
    if (motorDataList.size() > movingAvgRange) {
      motorDataList.remove(0); //remove old data point
    }
    for (int i = 0; i < motorDataList.size(); i++) {
      currentAverage += motorDataList.get(i);
    }
    currentAverage /= motorDataList.size();
    return currentAverage;
  }

  /**
   * Some sort of witchcraft that only Erik Sandberg knows how to decipher. Or documentation.
   *
   * @param _talon Motor being set up
   * @param sector True if initializing the sector, false otherwise
   */
  private void talonInit(WPI_TalonFX _talon, boolean sector) {
    _talon.configFactoryDefault();

    /* Configure Sensor Source for Pirmary PID */
    _talon.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    /* set deadband to super small 0.001 (0.1 %).
    The default deadband is 0.04 (4 %) */
    _talon.configNeutralDeadband(0.001, Constants.kTimeoutMs);

    /**
     * Configure Talon FX Output and Sensor direction accordingly Invert Motor to have green LEDs
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

    /**
     * Configure the current limits that will be used Stator Current is the current that passes
     * through the motor stators. Use stator current limits to limit rotor acceleration/heat
     * production Supply Current is the current that passes into the controller from the supply Use
     * supply current limits to prevent breakers from tripping
     *
     * <p>Once limiting is active, current limiting will deactivate if motor controller can apply
     * the requested motor output and still measure current-draw under the Continuous Current Limit.
     *
     * <p>enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s)
     */
    if (sector) {
      _talon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 40, 0.1));
    } else {
      _talon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 100, 110, 0.1));
    }
    //_talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,      10,                15,                0.5));

    /* Set Motion Magic gains in slot0 - see documentation */
    _talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    if (sector) {
      _talon.config_kF(Constants.kSlotIdx, Constants.SectorGains.kF, Constants.kTimeoutMs);
      _talon.config_kP(Constants.kSlotIdx, Constants.SectorGains.kP, Constants.kTimeoutMs);
      _talon.config_kI(Constants.kSlotIdx, Constants.SectorGains.kI, Constants.kTimeoutMs);
      _talon.config_kD(Constants.kSlotIdx, Constants.SectorGains.kD, Constants.kTimeoutMs);

    } else {
      _talon.config_kF(Constants.kSlotIdx, Constants.TrencherGains.kF, Constants.kTimeoutMs);
      _talon.config_kP(Constants.kSlotIdx, Constants.TrencherGains.kP, Constants.kTimeoutMs);
      _talon.config_kI(Constants.kSlotIdx, Constants.TrencherGains.kI, Constants.kTimeoutMs);
      _talon.config_kD(Constants.kSlotIdx, Constants.TrencherGains.kD, Constants.kTimeoutMs);
    }

    /* Set acceleration and vcruise velocity - see documentation */
    _talon.configMotionCruiseVelocity(10000, Constants.kTimeoutMs);
    _talon.configMotionAcceleration(6000, Constants.kTimeoutMs);

    /* Zero the sensor once on robot boot up */
    _talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  }
}
