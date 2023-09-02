package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Collection;
import frc.robot.subsystems.Gyron;
import frc.robot.subsystems.HTCVive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.PathManager;
import frc.robot.subsystems.Tracks;
import java.util.ArrayList;

/** Assumes that the robot is already in the correct position */
public class Offload extends SequentialCommandGroup {

  private Tracks m_tracks;
  private Hopper m_hopper;
  private HTCVive m_vive;
  private Gyron m_gyron;
  private PathManager m_manager;
  private Collection m_collection;

  private ArrayList<Command> commands = new ArrayList<>();
  private Command[] command_arr;

  public Offload(
      Tracks tracks,
      Hopper hopper,
      HTCVive vive,
      Gyron gyron,
      PathManager manager,
      Collection collection) {
    this.m_hopper = hopper;
    this.m_tracks = tracks;
    this.m_vive = vive;
    this.m_gyron = gyron;
    this.m_manager = manager;
    this.m_collection = collection;

    // addRequirements(m_tracks, m_hopper, m_gyron, m_vive, m_collection);

    commands.add(new TurnDegrees(135.0, m_tracks, m_gyron));
    commands.add(new OffloadBackup(m_tracks, m_hopper, m_collection));
    commands.add(new LowerHopper(m_hopper));

    command_arr = commands.toArray(new Command[commands.size()]);
    addCommands(command_arr);
  }

  @Override
  public boolean isFinished() {
    for (Command c : command_arr) {
      if (!c.isFinished()) {
        return false;
      }
    }

    return true;
  }
}
