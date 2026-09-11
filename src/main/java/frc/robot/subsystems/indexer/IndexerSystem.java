package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.belt.IndexerBeltConstants;
import frc.robot.subsystems.indexer.belt.IndexerBeltSubsystem;
import frc.robot.subsystems.indexer.feeder.FeederSubsystem;
import frc.robot.subsystems.indexer.roller_floor.RollerFloorConstants;
import frc.robot.subsystems.indexer.roller_floor.RollerFloorSubsystem;
import frc.robot.subsystems.indexer.vertical_roller.VerticalRollerConstants;
import frc.robot.subsystems.indexer.vertical_roller.VerticalRollerSubsystem;

public class IndexerSystem extends SubsystemBase {

  private final IndexerBeltSubsystem m_indexerBelt;
  private final FeederSubsystem m_feeder;
  private final RollerFloorSubsystem m_rollerFloor;
  private final VerticalRollerSubsystem m_verticalRoller;

  public IndexerSystem(
      IndexerBeltSubsystem indexerBelt,
      FeederSubsystem feeder,
      RollerFloorSubsystem rollerFloor,
      VerticalRollerSubsystem verticalRoller) {
    m_indexerBelt = indexerBelt;
    m_feeder = feeder;
    m_rollerFloor = rollerFloor;
    m_verticalRoller = verticalRoller;
  }

  /**
   * Creates a command that runs the roller floor, indexer belt, and feeder simultaneously to move
   * fuel through the indexer system.
   *
   * @return command that runs indefinitely until interrupted
   */
  public Command feedFuel() {
    return Commands.parallel(
        m_rollerFloor.setDutyCycle(0.6),
        m_indexerBelt.setDutyCycle(1),
        m_feeder.setDutyCycle(1),
        m_verticalRoller.setDutyCycle(1));
  }

  public Command reverseRollerFloor() {
    return m_rollerFloor.setDutyCycle(-0.6);
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
        m_rollerFloor.setVelocity(RollerFloorConstants.STORING_SPEED),
        m_indexerBelt.setVelocity(IndexerBeltConstants.STORING_SPEED),
        m_verticalRoller.setVelocity(VerticalRollerConstants.STORING_SPEED),
        m_feeder.stop());
  }

  /**
   * Creates a command that stops all motors in the indexer system.
   *
   * @return command that runs indefinitely until interrupted
   */
  public Command stop() {
    return Commands.parallel(
        m_rollerFloor.stop(), m_indexerBelt.stop(), m_feeder.stop(), m_verticalRoller.stop());
  }
}
