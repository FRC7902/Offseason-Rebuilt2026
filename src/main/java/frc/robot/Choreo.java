package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.IndexerSystem;
import frc.robot.subsystems.intake.IntakeSystem;
import frc.robot.subsystems.shooter.ShooterSystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;

public class Choreo {
  private final AutoFactory m_autoFactory;

  private final IndexerSystem m_indexerSystem;
  private final IntakeSystem m_intakeSystem;
  private final ShooterSystem m_shooterSystem;
  private final SwerveDriveSubsystem m_swerveDriveSubsystem;

  public Choreo(RobotContainer robotContainer) {
    m_autoFactory = robotContainer.m_autoFactory;

    m_indexerSystem = robotContainer.m_indexerSystem;
    m_intakeSystem = robotContainer.m_intakeSystem;
    m_shooterSystem = robotContainer.m_shooterSystem;
    m_swerveDriveSubsystem = robotContainer.m_swerveDriveSubsystem;
  }

  public Command testPath() {
    return Commands.sequence(
        m_autoFactory.resetOdometry("NewPath"),
        m_autoFactory.trajectoryCmd("NewPath"),
        m_swerveDriveSubsystem.stop());
  }
}
