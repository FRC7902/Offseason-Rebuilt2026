package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

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

  public boolean isStuck() {
    return m_rollerFloor.getVelocity().lt(RollerFloorConstants.FEEDING_SPEED.times(0.75));
  }

  public boolean isReversing() {
    return m_rollerFloor.getVelocity().lt(RollerFloorConstants.IS_REVERSING_SPEED_MIN);
  }

  public Command reverseIndexer() {
    return Commands.parallel(m_rollerFloor.setDutyCycle(-1), m_indexerBelt.setDutyCycle(-1));
  }

  /**
   * Creates a command that runs the roller floor, indexer belt, and feeder simultaneously to move
   * fuel through the indexer system.
   *
   * @return command that runs indefinitely until interrupted
   */
  public Command feedFuel() {
    return Commands.parallel(
        Commands.sequence(
                Commands.parallel(
                        m_indexerBelt.setDutyCycle(IndexerBeltConstants.FEEDING_DUTY_CYCLE),
                        m_rollerFloor.setVelocity(RollerFloorConstants.FEEDING_SPEED))
                    .withTimeout(1),
                Commands.waitUntil(() -> isStuck()),
                Commands.parallel(reverseIndexer()).withTimeout(0.2),
                Commands.waitUntil(() -> isReversing()).withTimeout(1))
            .repeatedly(),
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
            m_rollerFloor.setDutyCycle(RollerFloorConstants.STORING_DUTY_CYCLE),
            m_indexerBelt.setDutyCycle(IndexerBeltConstants.STORING_DUTY_CYCLE),
            m_verticalRoller.setDutyCycle(VerticalRollerConstants.STORING_DUTY_CYCLE),
            m_feeder.stop())
        .withDeadline(
            Commands.waitSeconds(0.5).until(() -> m_rollerFloor.getVelocity().lt(RPM.of(50))))
        .andThen(stop());
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
