package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.belt.IndexerBeltConstants;
import frc.robot.subsystems.indexer.belt.IndexerBeltSubsystem;
import frc.robot.subsystems.indexer.feeder.FeederConstants;
import frc.robot.subsystems.indexer.feeder.FeederSubsystem;
import frc.robot.subsystems.indexer.roller.RollerFloorConstants;
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

  /**
   * Creates a command that runs the roller floor, indexer belt, and feeder simultaneously to move
   * fuel through the indexer system.
   *
   * @return command that runs indefinitely until interrupted
   */
  public Command feedFuel() {
    return Commands.parallel(
        m_rollerFloor.setVelocity(RollerFloorConstants.ROLLER_SPEED),
        m_indexerBelt.setVelocity(IndexerBeltConstants.INDEXER_SPEED),
        m_feeder.setVelocity(FeederConstants.FEEDER_SPEED));
  }

  /**
   * Creates a command that runs the roller floor indefinitely while maintaining a lower duty cycle
   * for the indexer belt and leaving the feeder off to store fuel without feeding it through the
   * system.
   *
   * @return command that runs indefinitely until interrupted
   */
  public Command storeFuel() {
    return Commands.parallel(
        m_rollerFloor.setVelocity(RollerFloorConstants.FLOOR_STORE_SPEED),
        m_indexerBelt.setVelocity(IndexerBeltConstants.BELT_STORE_SPEED),
        m_feeder.stop());
  }
}
