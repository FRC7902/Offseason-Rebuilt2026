package frc.robot;

import choreo.auto.AutoFactory;
import frc.robot.subsystems.indexer.IndexerSystem;
import frc.robot.subsystems.intake.IntakeSystem;
import frc.robot.subsystems.shooter.ShooterSystem;

public class Choreo {
  private final AutoFactory m_autoFactory;

  private final IndexerSystem m_indexerSystem;
  private final IntakeSystem m_intakeSystem;
  private final ShooterSystem m_shooterSystem;

  public Choreo(RobotContainer robotContainer) {
    m_autoFactory = robotContainer.m_autoFactory;

    m_indexerSystem = robotContainer.m_indexerSystem;
    m_intakeSystem = robotContainer.m_intakeSystem;
    m_shooterSystem = robotContainer.m_shooterSystem;
  }
}
