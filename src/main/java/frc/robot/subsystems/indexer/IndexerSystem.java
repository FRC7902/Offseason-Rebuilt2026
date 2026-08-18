package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.belt.IndexerBeltSubsystem;
import frc.robot.subsystems.indexer.feeder.FeederSubsystem;
import frc.robot.subsystems.indexer.roller.RollerFloorSubsystem;

public class IndexerSystem {

  private final IndexerBeltSubsystem m_indexerBelt;
  private final FeederSubsystem m_feeder;
  private final RollerFloorSubsystem m_rollerFloor;

  public IndexerSystem(
      IndexerBeltSubsystem indexerBelt, FeederSubsystem feeder, RollerFloorSubsystem rollerFloor) {
    m_indexerBelt = indexerBelt;
    m_feeder = feeder;
    m_rollerFloor = rollerFloor;
  }

  // Run roller floor, indexer belt, and feeder simultaneously indefinitely
  public Command feedFuel() {
    throw new UnsupportedOperationException("Not yet implemented.");
  }

  // Indefinitely run roller floor, but run indexer with low duty cycle (don't run the feeder)
  public Command storeFuel() {
    throw new UnsupportedOperationException("Not yet implemented.");
  }
}
