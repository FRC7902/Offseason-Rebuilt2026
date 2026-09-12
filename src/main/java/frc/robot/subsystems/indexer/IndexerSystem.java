package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.belt.IndexerBeltConstants;
import frc.robot.subsystems.indexer.belt.IndexerBeltSubsystem;
import frc.robot.subsystems.indexer.feeder.FeederConstants;
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
  private static IndexerSystem m_instance;

  private IndexerSystem() {
    m_indexerBelt = IndexerBeltSubsystem.getInstance();
    m_feeder = FeederSubsystem.getInstance();
    m_rollerFloor = RollerFloorSubsystem.getInstance();
    m_verticalRoller = VerticalRollerSubsystem.getInstance();
  }

  public static IndexerSystem getInstance() {
    if (m_instance == null) {
      m_instance = new IndexerSystem();
    }
    return m_instance;
  }

  /**
   * Creates a command that runs the roller floor, indexer belt, and feeder simultaneously to move
   * fuel through the indexer system.
   *
   * @return command that runs indefinitely until interrupted
   */
  public Command feedFuel() {
    return Commands.parallel(
        m_rollerFloor.setVelocity(RollerFloorConstants.FEEDING_SPEED),
        m_indexerBelt.setDutyCycle(IndexerBeltConstants.FEEDING_DUTY_CYCLE),
        m_feeder.setDutyCycle(FeederConstants.FEEDING_DUTY_CYCLE),
        m_verticalRoller.setDutyCycle(VerticalRollerConstants.FEEDING_DUTY_CYCLE));
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
        m_indexerBelt.setDutyCycle(IndexerBeltConstants.STORING_DUTY_CYCLE),
        m_verticalRoller.setDutyCycle(VerticalRollerConstants.STORING_DUTY_CYCLE),
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
