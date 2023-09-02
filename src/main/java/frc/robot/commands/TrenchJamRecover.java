package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Collection;

public class TrenchJamRecover extends SequentialCommandGroup {

  private final Collection m_trencher;

  public TrenchJamRecover(Collection trench) {
    m_trencher = trench;
    addRequirements(m_trencher);

    addCommands(

        //stop trencher
        new StopTrencher(m_trencher),
        //raise trencher 5 deg or so

        //current sector encoder position = m_trencher.getSectorEncoder

        new MoveTrencherAbs(m_trencher, 0, Constants.SECTOR_CALC_LIGHT_MINING_VELOCITY),
        //attempt to run trencher forwards and/or backwards
        //be doing jam checks

        //BELOW LIES OLD STUFF
        //prep for mining
        new HomeTrencher(m_trencher),

        //plunge
        new StartTrencher(m_trencher),
        //fast plunge rate at shallow depth
        new MoveTrencherAbs(m_trencher, 20, Constants.SECTOR_CALC_LIGHT_MINING_VELOCITY),
        //slow plunge rate at deeper depth
        new MoveTrencherAbs(m_trencher, 50, Constants.SECTOR_CALC_MINING_VELOCITY),
        //stop and raise trencher

        new MoveTrencherAbs(m_trencher, 0, Constants.SECTOR_CALC_RAPID_VELOCITY));
  }
}
