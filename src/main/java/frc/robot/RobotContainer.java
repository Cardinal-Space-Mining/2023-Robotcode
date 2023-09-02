package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.commands.Sortie.AutonomyState;
import frc.robot.commands.TraversePath.PathDirection;
import frc.robot.commands.TraversePath.PathState;
import frc.robot.commands.TraversePath.RobotOrientation;
import frc.robot.helpers.Position;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

  // The robot's subsystems
  private final Hopper m_hopper = new Hopper();
  private final Collection m_trencher = new Collection();
  public final Tracks m_tracks = new Tracks();
  public final HTCVive m_vive = new HTCVive();
  public final Lidar m_lidar = new Lidar();
  public final Gyron m_gyro = new Gyron();
  public final PathManager m_manager = new PathManager();

  // Joysticks
  private final XboxController xboxController1 = new XboxController(0);
  private final Joystick logitec3d = new Joystick(1);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Specify the Teleop Drive Command
  private final Teleop teleopCommand =
      new Teleop(m_tracks, m_hopper, m_trencher, xboxController1, logitec3d);

  private ShuffleboardTab command_tab = Shuffleboard.getTab("Commands");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private RobotContainer() {

    // Smartdashboard Subsystems
    command_tab.add(
        "TeleOp", new Teleop(m_tracks, m_hopper, m_trencher, xboxController1, logitec3d));
    command_tab.add("Orient Gyro", new OrientGyro(m_vive, m_gyro, m_tracks));

    command_tab.add("Tele Way", new TeleopWaypoint(m_tracks, m_vive, m_manager, xboxController1));
    command_tab.add(
        "Traverse Path",
        new TraversePath(
            m_vive,
            m_tracks,
            m_gyro,
            m_manager,
            PathDirection.FORWARD,
            RobotOrientation.TRENCHER_FORWARD,
            PathState.STATIC,
            null));

    command_tab.add(
        "Traverse Path Back",
        new TraversePath(
            m_vive,
            m_tracks,
            m_gyro,
            m_manager,
            PathDirection.BACKWARD,
            RobotOrientation.TRENCHER_FORWARD,
            PathState.STATIC,
            null));

    command_tab.add(
        "Traverse Path DYN",
        new TraversePath(
            m_vive,
            m_tracks,
            m_gyro,
            m_manager,
            PathDirection.FORWARD,
            RobotOrientation.HOPPER_FORWARD,
            PathState.DYNAMIC,
            null));

    command_tab.add(
        "Move to position",
        new TraversePath(
            m_vive,
            m_tracks,
            m_gyro,
            m_manager,
            PathDirection.FORWARD,
            RobotOrientation.TRENCHER_FORWARD,
            PathState.POSITION,
            new Position(200, 0)));

    // Testing to just go to 0,0
    Constants.traverse_tab.add(
        new TraversePath(
            m_vive,
            m_tracks,
            m_gyro,
            new PathManager(),
            PathDirection.FORWARD,
            RobotOrientation.HOPPER_FORWARD,
            PathState.POSITION,
            new Position(0, 0)));
    // SmartDashboard.putData(
    //     "One Sortie",
    //     new Sortie(m_gyro, m_hopper, m_vive, m_lidar, m_tracks, m_trencher, m_manager, 0));
    // SmartDashboard.putData("Turn degrees", new TurnDegrees(135.0, m_tracks, m_gyro));
    // SmartDashboard.putData("raise hop", new RaiseHopper(m_hopper));
    // SmartDashboard.putData("low hop", new LowerHopper(m_hopper));
    // SmartDashboard.putData("offload pos", new OffloadPosition(m_tracks, m_vive, m_gyro, m_manager));
    // SmartDashboard.putData("offload rev", new OffloadBackup(m_tracks, m_hopper, m_trencher));
    // SmartDashboard.putData(
    // "full offload", new Offload(m_tracks, m_hopper, m_vive, m_gyro, m_manager, m_trencher));

    command_tab.add(
        "One Sortie",
        new Sortie(
            m_gyro,
            m_hopper,
            m_vive,
            m_lidar,
            m_tracks,
            m_trencher,
            m_manager,
            0,
            AutonomyState.PathPlan));

    command_tab.add("Turn degrees", new TurnDegrees(135.0, m_tracks, m_gyro));
    command_tab.add("raise hop", new RaiseHopper(m_hopper));
    command_tab.add("low hop", new LowerHopper(m_hopper));
    command_tab.add("Offload No vive", new OffloadBackup(m_tracks, m_hopper, m_trencher));
    command_tab.add(
        "offload", new Offload(m_tracks, m_hopper, m_vive, m_gyro, m_manager, m_trencher));
    command_tab.add(
        "offload & position before and after",
        new OffloadAndStartStopPosition(m_tracks, m_hopper, m_trencher, m_vive, m_gyro));

    command_tab.add(
        "Plunge, Trench", new PlungeTrench(m_trencher, m_tracks, m_hopper, m_vive, m_gyro, 0));
    command_tab.add(
        "Single Plunge", new SinglePlunge(m_trencher, m_tracks, m_hopper, m_vive, m_gyro, 0));
    command_tab.add("PlungeTrench NoVive", new PlungeTrench_NoVive(m_trencher, m_tracks, m_hopper));
    command_tab.add("SinglePlunge NoVive", new SinglePlunge_NoVive(m_trencher, m_tracks, m_hopper));

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands

    configureButtonBindings();
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Create some buttons

  }

  public XboxController getXboxController1() {
    return xboxController1;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(int sortie_num) {
    // The selected command will be run in autonomous
    return new Sortie(
        m_gyro,
        m_hopper,
        m_vive,
        m_lidar,
        m_tracks,
        m_trencher,
        m_manager,
        sortie_num,
        AutonomyState.PathPlan);

    // return new Sortie(
    //     m_gyro,
    //     m_hopper,
    //     m_vive,
    //     m_lidar,
    //     m_tracks,
    //     m_trencher,
    //     m_manager,
    //     sortie_num,
    //     AutonomyState.Waypoint);

    // return new TeleopWaypoint(m_tracks, m_vive, m_manager, xboxController1);
  }

  public Command getTeleopCommand() {
    return teleopCommand;
  }
}
