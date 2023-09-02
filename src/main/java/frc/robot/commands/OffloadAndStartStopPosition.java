package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.TraversePath.PathDirection;
import frc.robot.commands.TraversePath.PathState;
import frc.robot.commands.TraversePath.RobotOrientation;
import frc.robot.helpers.Position;
import frc.robot.subsystems.Collection;
import frc.robot.subsystems.Gyron;
import frc.robot.subsystems.HTCVive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.PathManager;
import frc.robot.subsystems.Tracks;
import java.util.ArrayList;

public class OffloadAndStartStopPosition extends SequentialCommandGroup {
  private Tracks m_tracks;
  private Hopper m_hopper;
  private Collection m_collection;
  private HTCVive m_vive;
  private Gyron m_gyron;
  private ArrayList<Command> commandList;
  private Command[] commandArray;

  public OffloadAndStartStopPosition(
      Tracks tracks, Hopper hopper, Collection collection, HTCVive vive, Gyron gyron) {
    this.m_tracks = tracks;
    this.m_hopper = hopper;
    this.m_collection = collection;
    this.m_vive = vive;
    this.m_gyron = gyron;
    this.commandList = new ArrayList<>();
    commandList.add(
        new TraversePath(
            m_vive,
            m_tracks,
            m_gyron,
            new PathManager(),
            PathDirection.FORWARD,
            RobotOrientation.TRENCHER_FORWARD,
            PathState.POSITION,
            new Position(Constants.STARTING_X, Constants.STARTING_Y)));

    commandList.add(new TurnDegrees(135.0, m_tracks, m_gyron));
    commandList.add(new OffloadBackup(m_tracks, m_hopper, m_collection));
    commandList.add(
        new TraversePath(
            m_vive,
            m_tracks,
            m_gyron,
            new PathManager(),
            PathDirection.FORWARD,
            RobotOrientation.TRENCHER_FORWARD,
            PathState.POSITION,
            new Position(Constants.STARTING_X, Constants.STARTING_Y)));

    commandArray = commandList.toArray(new Command[commandList.size()]);
    addCommands(commandArray);
  }

  @Override
  public boolean isFinished() {
    for (Command c : this.commandArray) {
      if (!c.isFinished()) {
        return false;
      }
    }

    return true;
  }
}
