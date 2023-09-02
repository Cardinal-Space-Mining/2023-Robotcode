package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Collection;
import frc.robot.subsystems.Gyron;
import frc.robot.subsystems.HTCVive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Tracks;
import java.util.ArrayList;

public class SinglePlunge extends SequentialCommandGroup {

  private final Collection m_trencher;
  private final Tracks m_tracks;
  private final Hopper m_hopper;
  private final HTCVive m_vive;
  private final Gyron m_gyron;
  private ArrayList<Command> commands = new ArrayList<>();
  private Command[] command_arr;

  // private Position miningStartPos;
  // private double miningDirection;

  public SinglePlunge(
      Collection trench, Tracks tracks, Hopper hopper, HTCVive vive, Gyron gyron, int sortienum) {
    m_trencher = trench;
    m_hopper = hopper;
    m_tracks = tracks;
    m_gyron = gyron;
    m_vive = vive;
    // addRequirements(m_trencher, m_tracks, m_hopper, m_gyron, m_vive);

    commands.add(new TurnMiningDirection(m_tracks, m_trencher, m_vive, m_gyron, sortienum));
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
    commands.add(new MoveSectorAbsSafe(m_trencher, 90, Constants.SECTOR_CALC_RAPID_VELOCITY));
    commands.add(new StopTrencher(m_trencher));
    commands.add(new DriveBackwardEncoderTicks(m_tracks, Constants.MINING_FORWARD_SCOOT));
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
