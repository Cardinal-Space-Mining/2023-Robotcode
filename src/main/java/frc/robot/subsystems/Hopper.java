package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {

  // TODO The hopper this year will be the only subsystem. The bucket ladder is no
  // more.
  // We need to add the linear actuator to this class. The linear actuator will
  // only be used during the offload command.
  // The motor will be used for the mining command and the offload command.

  // the following copied from 2021-2022's Bucketladder subsystem
  private final WPI_TalonSRX actuator =
      new WPI_TalonSRX(Constants.LINEAR_ACTUATOR_MOTOR_CONTROLLER_ID);
  private final WPI_TalonFX hopperMotor = new WPI_TalonFX(Constants.HOPPER_MOTOR_ID);
  // private final DigitalInput actuatorEncoder = new
  // DigitalInput(Constants.LINEAR_ACTUATOR_MOTOR_ENCODER_CHANNEL);

  // Shuffleboard for setting PID
  private ShuffleboardTab hopperTab = Shuffleboard.getTab("Hopper");

  // private double unloadPercentage = 0;
  private double unloadCurrent = 0;
  private double unloadEncoder = 0;
  private HopperStatus hopperStatus = HopperStatus.UNKNOWN;

  //Mining Shuffleboard Tab
  private ShuffleboardTab miningTab = Shuffleboard.getTab("Mining");

  NetworkTableEntry hopMiningEncoder = miningTab.add("Hopper Encoder", unloadEncoder).getEntry();
  NetworkTableEntry hopMiningCurrent = miningTab.add("Hopper Current", 0).getEntry();

  NetworkTableEntry hopEncoder = hopperTab.add("Hopper Encoder", 0).getEntry();
  NetworkTableEntry hopCurrent = hopperTab.add("Hopper Current", 0).getEntry();
  NetworkTableEntry revLimit = hopperTab.add("Rev Limit Closed", false).getEntry();
  NetworkTableEntry actuatorCurrent = hopperTab.add("Actuator Current", 0).getEntry();
  NetworkTableEntry actuatorGet = hopperTab.add("Actuator Get", 0).getEntry();
  NetworkTableEntry isAlive = hopperTab.add("Is Alive", false).getEntry();
  NetworkTableEntry supplyCurrent = hopperTab.add("Supply Current", 0).getEntry();
  NetworkTableEntry outVoltage = hopperTab.add("Output Voltage", 0).getEntry();
  NetworkTableEntry outPercent = hopperTab.add("Output Percent", 0).getEntry();

  public Hopper() {
    talonInit(hopperMotor);
    talonSRXInit(actuator);
    hopperTab.add("Unload Ladder Encoder Value", unloadEncoder);
    hopperMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.unloadEncoder = hopperMotor.getSensorCollection().getIntegratedSensorPosition();
    this.unloadCurrent = hopperMotor.getStatorCurrent();

    hopEncoder.setDouble(unloadEncoder);
    hopCurrent.setDouble(unloadCurrent);
    revLimit.setBoolean(actuator.isRevLimitSwitchClosed() == 1);
    actuatorCurrent.setDouble(actuator.getStatorCurrent());
    actuatorGet.setDouble(actuator.get());
    isAlive.setBoolean(actuator.isAlive());
    supplyCurrent.setDouble(actuator.getSupplyCurrent());
    outVoltage.setDouble(actuator.getMotorOutputVoltage());
    outPercent.setDouble(actuator.getMotorOutputPercent());

    hopMiningEncoder.setDouble(unloadEncoder);
    hopMiningCurrent.setDouble(unloadCurrent);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
  }

  public double getHopperMotorPosition() {
    return unloadEncoder;
  }

  public void moveActuatorPercentageControl(double percentage) {
    actuator.set(TalonSRXControlMode.PercentOutput, percentage);
  }

  public void raiseActuator() {
    actuator.set(TalonSRXControlMode.PercentOutput, 1);
  }

  public void lowerActuator() {
    actuator.set(TalonSRXControlMode.PercentOutput, -1);
  }

  public boolean actuatorIsAtTop() {
    return actuator.isRevLimitSwitchClosed() == 1;
  }

  public void moveHopperPercentage(double percentage) {
    hopperMotor.set(TalonFXControlMode.PercentOutput, -1 * percentage);
  }

  public boolean actuatorIsAtBottom() {
    return actuator.isFwdLimitSwitchClosed() == 1;
  }

  public void moveHopperVelocity(double encoderTicks) {
    hopperMotor.set(ControlMode.Velocity, encoderTicks);
    // hopperMotor.set(TalonFXControlMode.Velocity, encoderTicks);
  }

  public void disableHopperMotor() {
    hopperMotor.set(ControlMode.Disabled, 0);
  }

  private void talonInit(WPI_TalonFX _talon) {
    _talon.configFactoryDefault();

    /* Configure Sensor Source for Pirmary PID */
    _talon.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    /*
     * set deadband to super small 0.001 (0.1 %).
     * The default deadband is 0.04 (4 %)
     */
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
    _talon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 100, 110, 0.1));
    // _talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10,
    // 15, 0.5));

    /* Set the peak and nominal outputs */
    _talon.configNominalOutputForward(0, Constants.kTimeoutMs);
    _talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
    _talon.configPeakOutputForward(1, Constants.kTimeoutMs);
    _talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
    _talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    _talon.config_kF(Constants.kSlotIdx, Constants.hopperGains.kF, Constants.kTimeoutMs);
    _talon.config_kP(Constants.kSlotIdx, Constants.hopperGains.kP, Constants.kTimeoutMs);
    _talon.config_kI(Constants.kSlotIdx, Constants.hopperGains.kI, Constants.kTimeoutMs);
    _talon.config_kD(Constants.kSlotIdx, Constants.hopperGains.kD, Constants.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    _talon.configMotionCruiseVelocity(10000, Constants.kTimeoutMs);
    _talon.configMotionAcceleration(6000, Constants.kTimeoutMs);

    /* Zero the sensor once on robot boot up */
    _talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  }

  /**
   * Copied from last 2021-2022's BucketLadder subsystem
   *
   * @param _talonSRX
   */
  private void talonSRXInit(WPI_TalonSRX _talonSRX) {
    _talonSRX.configFactoryDefault();

    /*
     * set deadband to super small 0.001 (0.1 %).
     * The default deadband is 0.04 (4 %)
     */
    _talonSRX.configNeutralDeadband(0.001, Constants.kTimeoutMs);

    /**
     * Configure Talon FX Output and Sesnor direction accordingly Invert Motor to have green LEDs
     * when driving Talon Forward / Requesting Postiive Output Phase sensor to have positive
     * increment when driving Talon Forward (Green LED)
     */
    _talonSRX.setSensorPhase(false);
    _talonSRX.setInverted(false);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    _talonSRX.setStatusFramePeriod(
        StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    _talonSRX.setStatusFramePeriod(
        StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

    /* Set the peak and nominal outputs */
    _talonSRX.configNominalOutputForward(0, Constants.kTimeoutMs);
    _talonSRX.configNominalOutputReverse(0, Constants.kTimeoutMs);
    _talonSRX.configPeakOutputForward(1, Constants.kTimeoutMs);
    _talonSRX.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    _talonSRX.configMotionCruiseVelocity(10000, Constants.kTimeoutMs);
    _talonSRX.configMotionAcceleration(6000, Constants.kTimeoutMs);

    /* Zero the sensor once on robot boot up */
    _talonSRX.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  }

  /** Copied from 2021-2022's Bucketladder subsystem */
  public void enableActuatorDisabledControl() {
    actuator.set(ControlMode.Disabled, 0);
  }

  public enum HopperStatus {
    RAISED,
    LOWERED,
    UNKNOWN,
  }

  public void setHopperStatus(HopperStatus hopperStatus) {
    this.hopperStatus = hopperStatus;
  }

  public boolean isLowered() {
    return this.hopperStatus == HopperStatus.LOWERED;
  }

  public boolean isRaised() {
    return this.hopperStatus == HopperStatus.RAISED;
  }
}
