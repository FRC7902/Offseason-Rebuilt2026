package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.LinearVelocity;
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
    return m_rollerFloor.getVelocity().lt(RollerFloorConstants.FEEDING_SPEED.times(0.15));
  }

  public boolean isReversing() {
    return m_rollerFloor.getVelocity().lt(RollerFloorConstants.IS_REVERSING_SPEED_MIN);
  }

  public Command reverseRollerFloor() {
    return m_rollerFloor.setDutyCycle(-1);
  }

  public Command indexerbeltSync(LinearVelocity flywheelSpeed)
  {
    return (m_feeder.setVelocity(RPM.of(flywheelSpeed.in(InchesPerSecond)/FeederConstants.CIRCUMFERENCE.in(Inches)/60)));
  }

  public Command feederSync(LinearVelocity flywheelSpeed)
  {
    return (m_verticalRoller.setVelocity(RPM.of(flywheelSpeed.in(InchesPerSecond)/VerticalRollerConstants.CIRCUMFERENCE.in(Inches)/60)));
  }

  public Command verticalrollerSync(LinearVelocity flywheelSpeed)
  {
    return (m_verticalRoller.setVelocity(RPM.of(flywheelSpeed.in(InchesPerSecond)/VerticalRollerConstants.CIRCUMFERENCE.in(Inches)/60)));
  }

  public Command rollerfloorSync(LinearVelocity flywheelSpeed)
  {
    return (m_rollerFloor.setVelocity(RPM.of(flywheelSpeed.in(InchesPerSecond)/RollerFloorConstants.CIRCUMFERENCE.in(Inches)/60)));
  }

  /**
   * Creates a command that runs the roller floor, indexer belt, and feeder simultaneously to move
   * fuel through the indexer system.
   *
   * @return command that runs indefinitely until interrupted
   */
  public Command feedFuel(LinearVelocity flywheelSpeed) {
    return Commands.sequence(
        Commands.parallel(
                indexerbeltSync(flywheelSpeed),
                feederSync(flywheelSpeed),
                verticalrollerSync(flywheelSpeed),
                rollerfloorSync(flywheelSpeed))
            .withDeadline(Commands.waitSeconds(0.25))
            .andThen(
                Commands.sequence(
                        m_rollerFloor
                            .setVelocity(RollerFloorConstants.FEEDING_SPEED)
                            .withTimeout(1),
                        Commands.waitUntil(() -> isStuck()),
                        Commands.parallel(reverseRollerFloor()).withTimeout(0.1),
                        Commands.waitUntil(() -> isReversing()).withTimeout(1))
                    .repeatedly()));
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
            Commands.waitSeconds(0.5)
                .andThen(Commands.waitUntil(() -> m_rollerFloor.getVelocity().lt(RPM.of(50)))))
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
