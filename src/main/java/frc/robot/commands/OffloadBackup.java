package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Collection;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Tracks;
import java.util.ArrayList;

/**
 * Offload backup does esentially all of the offload command with exception of positioning the
 * robot. It will run the following commands:
 * <li>Raise Hopper
 * <li>Offload Reverse
 * <li>RunHopperMotor
 * <li>Move forward a litle
 * <li>Lower Hopper
 */
public class OffloadBackup extends SequentialCommandGroup {
  // First forward is the amount the motor moves after it stops going backwards
  public static final int OFFLOAD_FIRST_FORWARD_ENCODER_TICKS = 31_500;

  // Second forward is the amount the motor moves after it is done offloading
  // before it lowers the hopper
  public static final int OFFLOAD_SECOND_FORWARD_ENCODER_TICKS = 200_000;
  private Tracks m_tracks;
  private Hopper m_hopper;
  private Collection m_collection;
  private ArrayList<Command> commands = new ArrayList<>();
  private Command[] command_arr;

  public OffloadBackup(Tracks tracks, Hopper hopper, Collection collection) {
    this.m_tracks = tracks;
    this.m_hopper = hopper;
    this.m_collection = collection;

    // addRequirements(m_tracks, m_hopper);

    commands.add(new LowerHopper(hopper));
    commands.add(new HomeTrencher(m_collection));
    commands.add(new MoveSectorAbsSafe(m_collection, 30, Constants.SECTOR_CALC_RAPID_VELOCITY));
    commands.add(new RaiseHopper(m_hopper));
    commands.add(new OffloadReverse(m_tracks));
    commands.add(new DriveForwardEncoderTicks(m_tracks, OFFLOAD_FIRST_FORWARD_ENCODER_TICKS));
    commands.add(new RunHopperMotor(m_hopper));
    commands.add(
        new ParallelCommandGroup(
            new DriveForwardEncoderTicks(m_tracks, OFFLOAD_FIRST_FORWARD_ENCODER_TICKS),
            new LowerHopper(m_hopper)));
    commands.add(new HomeTrencher(m_collection));

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
