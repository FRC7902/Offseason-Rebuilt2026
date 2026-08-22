// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.indexer.IndexerSystem;
import frc.robot.subsystems.indexer.belt.IndexerBeltSubsystem;
import frc.robot.subsystems.indexer.feeder.FeederSubsystem;
import frc.robot.subsystems.indexer.roller.RollerFloorSubsystem;
import frc.robot.subsystems.intake.IntakeSystem;
import frc.robot.subsystems.intake.linear.LinearIntakeSubsystem;
import frc.robot.subsystems.intake.roller.IntakeRollerSubsystem;
import frc.robot.subsystems.shooter.ShooterSystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import yams.mechanisms.swerve.utility.SwerveInputStream;

public class RobotContainer {

  private final CommandPS5Controller m_driverController =
      new CommandPS5Controller(Constants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);

  private final IndexerBeltSubsystem m_indexerBeltSubsystem;
  private final FeederSubsystem m_feederSubsystem;
  private final RollerFloorSubsystem m_rollerFloorSubsystem;

  private final LinearIntakeSubsystem m_linearIntakeSubsystem;
  private final IntakeRollerSubsystem m_intakeRollerSubsystem;

  private final FlywheelSubsystem m_flywheelSubsystem;
  private final HoodSubsystem m_hoodSubsystem;
  private final TurretSubsystem m_turretSubsystem;

  private final IndexerSystem m_indexerSystem;
  private final IntakeSystem m_intakeSystem;
  private final ShooterSystem m_shooterSystem;

  private final SwerveDriveSubsystem m_swerveDriveSubsystem;
  private final SwerveInputStream driveAngularVelocity;

  private final StructArrayPublisher<Pose3d> posesPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("/3D/ComponentPoses", Pose3d.struct)
          .publish();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    // Start data logging
    DataLogManager.start();
    // Include DriverStation data in the log
    DriverStation.startDataLog(DataLogManager.getLog());

    m_indexerBeltSubsystem = new IndexerBeltSubsystem();
    m_feederSubsystem = new FeederSubsystem();
    m_rollerFloorSubsystem = new RollerFloorSubsystem();

    m_linearIntakeSubsystem = new LinearIntakeSubsystem();
    m_intakeRollerSubsystem = new IntakeRollerSubsystem();

    m_flywheelSubsystem = new FlywheelSubsystem();
    m_hoodSubsystem = new HoodSubsystem();
    m_turretSubsystem = new TurretSubsystem();

    m_swerveDriveSubsystem = new SwerveDriveSubsystem();
    driveAngularVelocity =
        m_swerveDriveSubsystem
            .getAngularVelocityStream(
                m_driverController::getLeftY,
                m_driverController::getLeftX,
                () -> m_driverController.getRawAxis(2))
            .withAllianceRelativeControl();

    m_indexerSystem =
        new IndexerSystem(m_indexerBeltSubsystem, m_feederSubsystem, m_rollerFloorSubsystem);
    m_intakeSystem = new IntakeSystem(m_linearIntakeSubsystem, m_intakeRollerSubsystem);
    m_shooterSystem = new ShooterSystem(m_flywheelSubsystem, m_hoodSubsystem, m_turretSubsystem);

    // NamedCommands.registerCommand("extendAndIntake", m_intakeSystem.extendAndIntake());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {

    m_swerveDriveSubsystem.setDefaultCommand(m_swerveDriveSubsystem.drive(driveAngularVelocity));

    /*
     * TODO: Bind driver controller L2
     * - When held, extend intake and run intake rollers
     * - When held, but not shooting (operator's R2 not held), run indexer to store
     * fuel
     * - When released, retract intake and stop intake rollers
     */

    /*
     * TODO: Bind operator controller R2
     * - When held run shooter, and run indexer to feed balls into shooter when
     * shooter is ready. Stop shooting when released
     * - When held and shooter is ready, shuffle the hopper using the intake. Stop
     * shuffling when released
     */
    m_operatorController
        .rightTrigger()
        .whileTrue(Commands.parallel(m_shooterSystem.aimAndShoot(), m_indexerSystem.feedFuel()))
        .whileTrue(
            Commands.sequence(
                Commands.waitUntil(m_shooterSystem::isShooterReady),
                m_intakeSystem.shuffleHopper()))
        .onFalse(
            Commands.sequence(
                m_indexerSystem.stop(),
                m_shooterSystem.stop(),
                m_intakeSystem.retractToMidpointThenStopIntake()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void publishComponentPoses() {
    Pose3d turretPose = m_turretSubsystem.getPose3d();
    Pose3d hoodPose = m_hoodSubsystem.getPose3d(turretPose);
    Pose3d linearIntakePose = m_linearIntakeSubsystem.getPose3d();

    posesPublisher.set(new Pose3d[] {linearIntakePose, turretPose, hoodPose, new Pose3d()});
  }
}
