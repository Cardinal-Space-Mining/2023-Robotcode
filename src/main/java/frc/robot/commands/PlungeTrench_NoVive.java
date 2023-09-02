package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Collection;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Tracks;
import java.util.ArrayList;

public class PlungeTrench_NoVive extends SequentialCommandGroup {

  private final Collection m_trencher;
  private final Tracks m_tracks;
  private final Hopper m_hopper;
  private ArrayList<Command> commands = new ArrayList<>();
  private Command[] command_arr;

  public PlungeTrench_NoVive(Collection trench, Tracks tracks, Hopper hopper) {
    m_trencher = trench;
    m_hopper = hopper;
    m_tracks = tracks;

    commands.add(new DriveForwardEncoderTicks(m_tracks, Constants.MINING_FORWARD_SCOOT));
    commands.add(new LowerHopper(m_hopper));
    commands.add(new HomeTrencher(m_trencher));
    commands.add(new MoveSectorAbsSafe(m_trencher, 150, Constants.SECTOR_CALC_RAPID_VELOCITY));
    // commands.add(new StartTrencher(m_trencher));
    commands.add(
        new ParallelRaceGroup(
            new HopperSectorIdx(m_hopper, m_trencher),
            new SectorPlunge(m_trencher, Constants.SECTOR_PLUNGE_DEPTH)));

    commands.add(new StopHopper(m_hopper));
    commands.add(new StartTrencher(m_trencher));
    commands.add(
        new ParallelRaceGroup(
            new HopperTracksIdx(m_hopper, m_trencher, m_tracks),
            new Trench(m_trencher, m_tracks, Constants.MINING_TRENCH_LENGTH)));
    commands.add(new StopHopper(m_hopper));
    commands.add(new MoveSectorAbsSafe(m_trencher, 90, Constants.SECTOR_CALC_RAPID_VELOCITY));
    commands.add(new StopTrencher(m_trencher));
    commands.add(new DriveBackwardEncoderTicks(m_tracks, -Constants.MINING_RETURN_DIST));
    commands.add(new HomeTrencher(m_trencher));

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
