package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.TraversePath.PathDirection;
import frc.robot.commands.TraversePath.PathState;
import frc.robot.commands.TraversePath.RobotOrientation;
import frc.robot.helpers.Position;
import frc.robot.subsystems.*;
import java.util.ArrayList;

public class SortieWay extends SequentialCommandGroup {
  Gyron gyro;
  Hopper hopper;
  HTCVive vive;
  Lidar lidar;
  Tracks tracks;
  Collection trencher;
  int sortie_num;
  Command traversal_Command;
  ArrayList<Command> commands = new ArrayList<>();
  Command[] command_arr;
  AutonomyState auto_state;
  XboxController controller;

  public enum AutonomyState {
    Waypoint,
    PathPlan
  }

  public SortieWay(
      Gyron gyro,
      Hopper hopper,
      HTCVive vive,
      Lidar lidar,
      Tracks tracks,
      Collection trencher,
      PathManager manager,
      int sortie_num,
      AutonomyState auto_state,
      XboxController controller) {

    this.auto_state = auto_state;
    this.controller = controller;

    //Traverse through static path if we are on a soritie greater than 0
    if (sortie_num > 0) {
      commands.add(
          new TraversePath(
              vive,
              tracks,
              gyro,
              manager,
              PathDirection.FORWARD,
              RobotOrientation.TRENCHER_FORWARD,
              PathState.POSITION,
              startingPosition()));

      commands.add(new TeleopWaypoint(tracks, vive, manager, controller));
    }
    //Otherwise we have to find a path through the obstacle zone.
    else {
      commands.add(new HomeTrencher(trencher));
      commands.add(
          new ParallelCommandGroup(new OrientGyro(vive, gyro, tracks), new LowerHopper(hopper)));

      commands.add(
          new TraversePath(
              vive,
              tracks,
              gyro,
              manager,
              PathDirection.FORWARD,
              RobotOrientation.HOPPER_FORWARD,
              PathState.POSITION,
              startingPosition()));

      if (auto_state == AutonomyState.PathPlan) {
        commands.add(
            new TraversePath(
                vive,
                tracks,
                gyro,
                manager,
                PathDirection.FORWARD,
                RobotOrientation.HOPPER_FORWARD,
                PathState.DYNAMIC,
                null));
      } else {
        commands.add(new TeleopWaypoint(tracks, vive, manager, controller));
      }
    }

    // commands.add(new PlungeTrench(trencher, tracks, hopper, vive, gyro));

    if (sortie_num > 0) {
      commands.add(new PlungeTrench(trencher, tracks, hopper, vive, gyro, sortie_num));
    } else {
      commands.add(new SinglePlunge(trencher, tracks, hopper, vive, gyro, sortie_num));
    }

    // commands.add(new DriveForwardEncoderTicks(tracks, 100000));

    commands.add(
        new TraversePath(
            vive,
            tracks,
            gyro,
            manager,
            PathDirection.BACKWARD,
            RobotOrientation.HOPPER_FORWARD,
            PathState.STATIC,
            null));
    commands.add(
        new TraversePath(
            vive,
            tracks,
            gyro,
            manager,
            PathDirection.BACKWARD,
            RobotOrientation.HOPPER_FORWARD,
            PathState.POSITION,
            startingPosition()));

    commands.add(new Offload(tracks, hopper, vive, gyro, manager, trencher));

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

  private Position startingPosition() {
    return new Position(Constants.STARTING_X, Constants.STARTING_Y);
  }
}
