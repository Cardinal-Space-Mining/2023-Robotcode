package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Sortie.AutonomyState;
import frc.robot.subsystems.*;

public class SortieManager extends CommandBase {
  Gyron gyro;
  Hopper hopper;
  HTCVive vive;
  Lidar lidar;
  Tracks tracks;
  Collection collection;
  Timer time;
  PathManager manager;
  Sortie curr_sortie;
  Sortie next_sortie;
  //Initialize to null on first run
  private int sortie_num;
  XboxController controller;

  /**
   * @param gyro Gyro SUbssytem
   * @param hopper Hopper Subsystem
   * @param vive HTCVive Subsystem
   * @param lidar Lidar Subsystem
   * @param tracks Tracks Subsystem
   * @param trencher Trencher Subsystem
   * @param manager Path Manager Subsystem
   * @param sortie_num The current sortie number
   * @param curr_sortie The currently running sortie. User should always initialize this to null.
   */
  public SortieManager(
      Gyron gyro,
      Hopper hopper,
      HTCVive vive,
      Lidar lidar,
      Tracks tracks,
      Collection collection,
      PathManager manager,
      int sortie_num,
      Sortie curr_sortie,
      XboxController controller) {
    this.gyro = gyro;
    this.hopper = hopper;
    this.vive = vive;
    this.lidar = lidar;
    this.tracks = tracks;
    this.collection = collection;
    this.manager = manager;
    this.sortie_num = sortie_num;
    this.curr_sortie = curr_sortie;
    this.controller = controller;
    next_sortie =
        new Sortie(
            gyro,
            hopper,
            vive,
            lidar,
            tracks,
            collection,
            manager,
            sortie_num,
            AutonomyState.PathPlan);
  }

  @Override
  public void execute() {

    //Only schedule the next sortie when the current running sortie is either non-existent or not scheduled
    if (curr_sortie == null || !curr_sortie.isScheduled()) {
      next_sortie.schedule();
      sortie_num++;
    }
  }

  //Only need to run execute once
  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    //TODO add timer to determine when to stop scheduling next sortie
    new SortieManager(
        gyro,
        hopper,
        vive,
        lidar,
        tracks,
        collection,
        manager,
        sortie_num,
        next_sortie,
        controller);
  }
}
