package frc.robot.commands;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Collection;
import java.util.ArrayList;

/**
 * Moves the trencher arm back to the home position. The trencher must be in the home position
 * before it can be moved.
 */
public class MoveSectorBottomLimit extends CommandBase {

  private final Collection m_trencher;
  private boolean isFinished;

  ArrayList<Double> trencherCurrentList; //Added for current smoothing
  int movingAvgRange;
  // private boolean limitReached; //Added to monitor trencher stopping

  public MoveSectorBottomLimit(Collection subsystem) {
    isFinished = false;
    m_trencher = subsystem;
    addRequirements(m_trencher);
    // limitReached = false;

    trencherCurrentList = new ArrayList<Double>();
    //larger movingAverageRange will react to motor current spike periods less
    //while lower movingAverageRange will react more often
    //must be whole number likely between 10-5ish probably
    movingAvgRange = 8;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initialize");
    isFinished = false;
    m_trencher.setTrencherJammed(false);
  }

  @Override
  public void execute() {
    //Trencher current signal smoothing stuff
    double new_current = m_trencher.getTrencherCurrent();
    trencherCurrentList.add(new_current);
    double avg_current = getMovingAverage(trencherCurrentList, movingAvgRange);

    SmartDashboard.putNumber("Trench Curr Avg", avg_current);

    //Move Trencher to full depth
    if (m_trencher.getDownLimit()) {
      m_trencher.enableSectorDisabledControl();
      isFinished = true;
    }
    // stop plunge if trencher jamming or jammed
    else if (avg_current > Constants.TRENCH_SAFE_CURRENT_THRESH || m_trencher.getTrencherJammed()) {
      // m_trencher.setSectorTalonVelocity(0);
      m_trencher.moveSectorVelocityControl(0);
    }
    // plunge if no jam or jamming is detected
    else if (!m_trencher.getTrencherJammed()) {
      // m_trencher.setSectorTalonVelocity(-Constants.SECTOR_CALC_LIGHT_MINING_VELOCITY);
      m_trencher.moveSectorVelocityControl(-Constants.SECTOR_CALC_MINING_VELOCITY);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }

  // Defined here for now, but may be nice to move into Trencher
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
}
